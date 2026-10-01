#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Sits between RViz's goal tool and Nav2's NavigateToPose action. RViz
// publishes to /goal_pose_requested (not /goal_pose, so bt_navigator's own
// built-in topic->action bridge never sees it directly). This node checks
// the requested goal against the live global costmap: if the goal cell is
// hazardous, it relocates to the nearest non-hazardous cell within
// search_radius_m before calling the action; if no safe cell exists nearby,
// it rejects the goal outright rather than guessing further. Either way,
// nothing downstream ever receives a goal sitting on a known hazard.
class GoalSafetyRelay : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  GoalSafetyRelay()
  : Node("goal_safety_relay")
  {
    lethal_cost_threshold_ = declare_parameter<int>("lethal_cost_threshold", 70);
    // A relocated goal previously landed on the *nearest* safe cell, which
    // by definition sits right at the boundary between safe and hazardous
    // -- almost no margin. The very next replan (~1Hz) can then find that
    // same cell has tipped over into "occupied" from a tiny recomputation
    // difference (pose jitter, a patch reprojecting slightly differently),
    // and the robot gets stuck unable to plan from an occupied start. This
    // margin requires relocation targets (and the original goal, if
    // unmodified) to sit meaningfully below the lethal threshold, not just
    // barely under it.
    safety_margin_cost_ = declare_parameter<int>("safety_margin_cost", 20);
    search_radius_m_ = declare_parameter<double>("search_radius_m", 3.0);
    costmap_topic_ = declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose_requested");
    action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");

    rclcpp::QoS costmap_qos(1);
    costmap_qos.reliable();
    costmap_qos.transient_local();
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, costmap_qos,
      std::bind(&GoalSafetyRelay::receiveCostmap, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(5).reliable(),
      std::bind(&GoalSafetyRelay::receiveGoal, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    RCLCPP_INFO(
      get_logger(),
      "goal_safety_relay: %s -> %s, lethal_cost_threshold=%d, safety_margin_cost=%d, "
      "search_radius=%.1fm",
      goal_topic_.c_str(), action_name_.c_str(), lethal_cost_threshold_, safety_margin_cost_,
      search_radius_m_);
  }

private:
  void receiveCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(costmap_mtx_);
    latest_costmap_ = msg;
  }

  void receiveGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
    {
      std::lock_guard<std::mutex> lock(costmap_mtx_);
      costmap = latest_costmap_;
    }

    geometry_msgs::msg::PoseStamped goal = *msg;

    if (!costmap) {
      RCLCPP_WARN(
        get_logger(),
        "No costmap received yet -- sending goal (%.2f, %.2f) unchecked",
        goal.pose.position.x, goal.pose.position.y);
      sendGoal(goal);
      return;
    }

    if (costmap->header.frame_id != goal.header.frame_id) {
      RCLCPP_WARN(
        get_logger(),
        "Goal frame '%s' != costmap frame '%s' -- sending goal unchecked "
        "(no TF lookup implemented for cross-frame goals)",
        goal.header.frame_id.c_str(), costmap->header.frame_id.c_str());
      sendGoal(goal);
      return;
    }

    const auto relocated = relocateIfNeeded(*costmap, goal.pose.position.x, goal.pose.position.y);
    if (!relocated.has_value()) {
      RCLCPP_ERROR(
        get_logger(),
        "Goal (%.2f, %.2f) is hazardous and no safe cell found within %.1fm -- rejecting goal",
        goal.pose.position.x, goal.pose.position.y, search_radius_m_);
      return;
    }

    if (relocated->first != goal.pose.position.x || relocated->second != goal.pose.position.y) {
      const double moved = std::hypot(
        relocated->first - goal.pose.position.x, relocated->second - goal.pose.position.y);
      RCLCPP_WARN(
        get_logger(),
        "Goal (%.2f, %.2f) was hazardous -- relocated %.2fm to nearest safe cell (%.2f, %.2f)",
        goal.pose.position.x, goal.pose.position.y, moved, relocated->first, relocated->second);
      goal.pose.position.x = relocated->first;
      goal.pose.position.y = relocated->second;
    }

    sendGoal(goal);
  }

  // Returns the (x, y) to actually navigate to, or nullopt if the goal is
  // hazardous and no safe cell exists within search_radius_m_.
  std::optional<std::pair<double, double>> relocateIfNeeded(
    const nav_msgs::msg::OccupancyGrid & costmap, double x, double y) const
  {
    if (!cellIsHazardous(costmap, x, y)) {
      return std::make_pair(x, y);
    }

    const double resolution = costmap.info.resolution;
    const int search_cells = static_cast<int>(std::ceil(search_radius_m_ / resolution));
    const int cx = static_cast<int>(std::floor((x - costmap.info.origin.position.x) / resolution));
    const int cy = static_cast<int>(std::floor((y - costmap.info.origin.position.y) / resolution));

    double best_dist_sq = std::numeric_limits<double>::max();
    std::optional<std::pair<double, double>> best;
    for (int dy = -search_cells; dy <= search_cells; ++dy) {
      for (int dx = -search_cells; dx <= search_cells; ++dx) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(costmap.info.width) ||
          ny >= static_cast<int>(costmap.info.height))
        {
          continue;
        }
        const int8_t value =
          costmap.data[static_cast<size_t>(ny) * costmap.info.width + static_cast<size_t>(nx)];
        if (value >= 0 && value >= lethal_cost_threshold_ - safety_margin_cost_) {
          continue;  // still hazardous, or too close to the threshold to have real margin
        }
        const double wx = costmap.info.origin.position.x + (nx + 0.5) * resolution;
        const double wy = costmap.info.origin.position.y + (ny + 0.5) * resolution;
        const double dist_sq = (wx - x) * (wx - x) + (wy - y) * (wy - y);
        if (dist_sq > search_radius_m_ * search_radius_m_) {
          continue;
        }
        if (dist_sq < best_dist_sq) {
          best_dist_sq = dist_sq;
          best = std::make_pair(wx, wy);
        }
      }
    }
    return best;
  }

  bool cellIsHazardous(const nav_msgs::msg::OccupancyGrid & costmap, double x, double y) const
  {
    const double resolution = costmap.info.resolution;
    const int cx = static_cast<int>(std::floor((x - costmap.info.origin.position.x) / resolution));
    const int cy = static_cast<int>(std::floor((y - costmap.info.origin.position.y) / resolution));
    if (cx < 0 || cy < 0 || cx >= static_cast<int>(costmap.info.width) ||
      cy >= static_cast<int>(costmap.info.height))
    {
      return false;  // outside known costmap extent -- can't judge, don't block
    }
    const int8_t value =
      costmap.data[static_cast<size_t>(cy) * costmap.info.width + static_cast<size_t>(cx)];
    return value >= 0 && value >= lethal_cost_threshold_ - safety_margin_cost_;
  }

  void sendGoal(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available");
      return;
    }
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = pose;
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    action_client_->async_send_goal(goal_msg, options);
  }

  int lethal_cost_threshold_ {};
  int safety_margin_cost_ {};
  double search_radius_m_ {};
  std::string costmap_topic_;
  std::string goal_topic_;
  std::string action_name_;

  std::mutex costmap_mtx_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSafetyRelay>());
  rclcpp::shutdown();
  return 0;
}
