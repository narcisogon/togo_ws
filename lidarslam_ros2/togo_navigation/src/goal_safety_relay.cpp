#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Existing goal bridge also gates Nav2's final smoothed commands on backend
// freshness. It does not classify terrain, inflate costs, or relocate goals.
class GoalSafetyRelay : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using Steady = std::chrono::steady_clock;
  GoalSafetyRelay()
  : Node("goal_safety_relay"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose_requested");
    const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
    const auto action_name = declare_parameter<std::string>("action_name", "navigate_to_pose");
    allow_unknown_ = declare_parameter<bool>("allow_unknown_goals", true);
    ready_timeout_ = declare_parameter<double>("ready_timeout_sec", 1.0);
    command_timeout_ = declare_parameter<double>("command_timeout_sec", 0.5);
    goal_wait_timeout_ = declare_parameter<double>("goal_wait_timeout_sec", 5.0);
    if (!std::isfinite(ready_timeout_) || ready_timeout_ <= 0.0 ||
      !std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
      !std::isfinite(goal_wait_timeout_) || goal_wait_timeout_ <= 0.0)
    {throw std::invalid_argument("relay watchdog limits must be positive and finite");}
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {map_ = std::move(msg);});
    ready_sub_ = create_subscription<std_msgs::msg::Bool>("/hazard/ready", rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        ready_ = msg->data; last_ready_ = Steady::now();
        if (!ready_) {publishStop();}
      });
    status_sub_ = create_subscription<std_msgs::msg::String>(
      "/hazard/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {backend_status_ = msg->data;});
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(goal_topic, 5,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {receiveGoal(*msg);});
    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/platform_velocity_controller/cmd_vel", rclcpp::QoS(1));
    command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/nav2/cmd_vel_smoothed", rclcpp::QoS(1),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        last_command_ = Steady::now();
        const auto & t = msg->twist;
        if (healthy() && std::isfinite(t.linear.x) && std::isfinite(t.linear.y) &&
          std::isfinite(t.linear.z) && std::isfinite(t.angular.x) &&
          std::isfinite(t.angular.y) && std::isfinite(t.angular.z))
        {command_pub_->publish(*msg);} else {publishStop();}
      });
    watchdog_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      if (!healthy() || std::chrono::duration<double>(Steady::now() - last_command_).count() >
        command_timeout_) {publishStop();}
      tryPendingGoal();
    });
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name);
  }

private:
  bool healthy() const
  {
    return ready_ && map_ &&
           std::chrono::duration<double>(Steady::now() - last_ready_).count() <= ready_timeout_;
  }
  void publishStop()
  {
    if (!command_pub_) {return;}
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now(); msg.header.frame_id = "base_link";
    command_pub_->publish(msg);
  }
  void receiveGoal(geometry_msgs::msg::PoseStamped goal)
  {
    if (pending_goal_) {RCLCPP_INFO(get_logger(), "Replacing the pending waypoint with the latest request");}
    pending_goal_ = std::move(goal);
    pending_since_ = Steady::now();
    pending_reason_.clear();
    tryPendingGoal();
  }
  std::string waitingReason() const
  {
    if (!map_) {return "waiting_for_map";}
    if (last_ready_ == Steady::time_point{}) {return "waiting_for_backend_heartbeat";}
    if (std::chrono::duration<double>(Steady::now() - last_ready_).count() > ready_timeout_) {
      return "backend_heartbeat_timeout";
    }
    if (!ready_) {return backend_status_ == "ready" ? "backend_not_ready" : backend_status_;}
    if (!action_client_->action_server_is_ready()) {return "waiting_for_navigate_to_pose";}
    return {};
  }
  void tryPendingGoal()
  {
    if (!pending_goal_) {return;}
    const auto reason = waitingReason();
    if (std::chrono::duration<double>(Steady::now() - pending_since_).count() >= goal_wait_timeout_) {
      RCLCPP_WARN(get_logger(), "Goal expired after %.1fs waiting: %s",
        goal_wait_timeout_, reason.empty() ? "readiness deadline elapsed" : reason.c_str());
      pending_goal_.reset();
      return;
    }
    if (!reason.empty()) {
      if (pending_reason_ != reason) {
        RCLCPP_INFO(get_logger(), "Waypoint pending (up to %.1fs): %s",
          goal_wait_timeout_, reason.c_str());
        pending_reason_ = reason;
      }
      return;
    }
    auto goal = std::move(*pending_goal_);
    pending_goal_.reset();
    // Validate against the current map only AFTER readiness recovers.
    try {
      if (goal.header.frame_id != map_->header.frame_id) {
        goal = tf_buffer_.transform(goal, map_->header.frame_id, tf2::durationFromSec(0.0));
      }
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "Goal rejected: %s", e.what()); return;
    }
    const auto & map = *map_;
    const double x = goal.pose.position.x, y = goal.pose.position.y;
    if (!std::isfinite(x) || !std::isfinite(y) || map.info.resolution <= 0.0 ||
      map.data.size() != static_cast<std::size_t>(map.info.width) * map.info.height)
    {RCLCPP_WARN(get_logger(), "Goal rejected: invalid goal/map"); return;}
    if (std::abs(map.info.origin.orientation.w - 1.0) > 1e-6) {
      RCLCPP_WARN(get_logger(), "Goal rejected: unsupported rotated map origin"); return;
    }
    const double gx = std::floor((x - map.info.origin.position.x) / map.info.resolution);
    const double gy = std::floor((y - map.info.origin.position.y) / map.info.resolution);
    if (gx < 0 || gy < 0 || gx >= map.info.width || gy >= map.info.height) {
      RCLCPP_WARN(get_logger(), "Goal rejected: outside backend map bounds"); return;
    }
    const auto cost = map.data[static_cast<std::size_t>(gy) * map.info.width +
      static_cast<std::size_t>(gx)];
    if (cost == 100 || (cost < 0 && !allow_unknown_)) {
      RCLCPP_WARN(get_logger(), "Goal rejected at requested position: cell cost=%d", cost); return;
    }
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "Goal rejected: navigate_to_pose is not active"); return;
    }
    NavigateToPose::Goal request; request.pose = goal;
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [this](const auto & handle) {
      if (!handle) {RCLCPP_WARN(get_logger(), "Nav2 rejected the requested goal");}
    };
    action_client_->async_send_goal(request, options);
    RCLCPP_INFO(get_logger(), "Forwarded waypoint to Nav2");
  }

  bool allow_unknown_ {true}, ready_ {false};
  double ready_timeout_ {1.0}, command_timeout_ {0.5};
  double goal_wait_timeout_ {5.0};
  Steady::time_point last_ready_ {}, last_command_ {};
  Steady::time_point pending_since_ {};
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;
  std::string backend_status_ {"backend_not_ready"}, pending_reason_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSafetyRelay>());
  rclcpp::shutdown();
  return 0;
}
