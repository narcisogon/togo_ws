// MIT License
//
// Short-horizon IMU bridge for high-rate local pose estimates.

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
Eigen::Quaterniond msgToQuat(const geometry_msgs::msg::Quaternion & q_msg)
{
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  if (!std::isfinite(q.norm()) || q.norm() < 1.0e-6) {
    return Eigen::Quaterniond::Identity();
  }
  return q.normalized();
}

geometry_msgs::msg::Quaternion quatToMsg(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion msg;
  const Eigen::Quaterniond normalized = q.normalized();
  msg.x = normalized.x();
  msg.y = normalized.y();
  msg.z = normalized.z();
  msg.w = normalized.w();
  return msg;
}

Eigen::Quaterniond integrateGyroBody(
  const Eigen::Quaterniond & orientation,
  const Eigen::Vector3d & angular_velocity,
  const double dt)
{
  const double angle = angular_velocity.norm() * dt;
  if (!std::isfinite(angle) || angle < 1.0e-9) {
    return orientation;
  }
  const Eigen::Vector3d axis = angular_velocity.normalized();
  return (orientation * Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis))).normalized();
}
}  // namespace

class ImuPredictionNode : public rclcpp::Node
{
public:
  explicit ImuPredictionNode(const rclcpp::NodeOptions & options)
  : Node("imu_prediction_node", options)
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/rko_lio/odometry");
    imu_topic_ = declare_parameter<std::string>(
      "imu_topic", "/a300_0000/sensors/seyond_robin_w/imu");
    predicted_odom_topic_ = declare_parameter<std::string>(
      "predicted_odom_topic", "/rko_lio/odometry_imu_predict");
    predicted_path_topic_ = declare_parameter<std::string>(
      "predicted_path_topic", "/rko_lio/path_imu_predict");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_link_imu_predict");
    max_prediction_horizon_sec_ = declare_parameter<double>("max_prediction_horizon_sec", 0.12);
    max_imu_dt_sec_ = declare_parameter<double>("max_imu_dt_sec", 0.05);
    max_publish_rate_hz_ = declare_parameter<double>("max_publish_rate_hz", 80.0);
    max_path_poses_ = declare_parameter<int>("max_path_poses", 12000);
    path_min_distance_m_ = declare_parameter<double>("path_min_distance_m", 0.03);
    prefer_odom_twist_ = declare_parameter<bool>("prefer_odom_twist", false);
    twist_in_child_frame_ = declare_parameter<bool>("twist_in_child_frame", true);
    use_acceleration_ = declare_parameter<bool>("use_acceleration", false);
    gravity_mps2_ = declare_parameter<double>("gravity_mps2", 1.625);
    acceleration_blend_ = declare_parameter<double>("acceleration_blend", 0.15);
    velocity_smoothing_alpha_ = declare_parameter<double>("velocity_smoothing_alpha", 0.9);
    stationary_velocity_threshold_ = declare_parameter<double>("stationary_velocity_threshold", 0.02);
    gyro_deadband_rad_s_ = declare_parameter<double>("gyro_deadband_rad_s", 0.002);
    max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 2.0);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 3.0);
    max_position_extrapolation_m_ = declare_parameter<double>("max_position_extrapolation_m", 0.08);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(predicted_odom_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(predicted_path_topic_, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&ImuPredictionNode::receiveOdometry, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuPredictionNode::receiveImu, this, std::placeholders::_1));

    path_msg_.header.frame_id = fixed_frame_;

    RCLCPP_INFO(
      get_logger(),
      "IMU bridge: odom=%s imu=%s predicted_odom=%s predicted_path=%s horizon=%.2fs",
      odom_topic_.c_str(), imu_topic_.c_str(), predicted_odom_topic_.c_str(),
      predicted_path_topic_.c_str(), max_prediction_horizon_sec_);
  }

private:
  void receiveOdometry(const nav_msgs::msg::Odometry & msg)
  {
    const Eigen::Vector3d position(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z);
    if (!position.allFinite()) {
      return;
    }

    const rclcpp::Time stamp(msg.header.stamp);
    const Eigen::Quaterniond orientation = msgToQuat(msg.pose.pose.orientation);
    const Eigen::Vector3d twist_linear(
      msg.twist.twist.linear.x,
      msg.twist.twist.linear.y,
      msg.twist.twist.linear.z);
    Eigen::Vector3d measured_velocity_world = Eigen::Vector3d::Zero();
    const Eigen::Vector3d odom_twist_world =
      twist_in_child_frame_ ? orientation * twist_linear : twist_linear;

    if (prefer_odom_twist_ && odom_twist_world.allFinite() &&
        odom_twist_world.norm() > stationary_velocity_threshold_) {
      measured_velocity_world = odom_twist_world;
    } else if (initialized_) {
      const double odom_dt = (stamp - anchor_stamp_).seconds();
      if (odom_dt > 1.0e-3 && odom_dt < 2.0) {
        measured_velocity_world = (position - anchor_position_) / odom_dt;
      }
    }

    if (!measured_velocity_world.allFinite() ||
        measured_velocity_world.norm() < stationary_velocity_threshold_) {
      measured_velocity_world.setZero();
    }
    if (measured_velocity_world.norm() > max_linear_velocity_) {
      measured_velocity_world = measured_velocity_world.normalized() * max_linear_velocity_;
    }

    if (initialized_) {
      const double alpha = std::clamp(velocity_smoothing_alpha_, 0.0, 1.0);
      anchor_velocity_world_ =
        alpha * measured_velocity_world + (1.0 - alpha) * anchor_velocity_world_;
      if (anchor_velocity_world_.norm() < stationary_velocity_threshold_) {
        anchor_velocity_world_.setZero();
      }
    } else {
      anchor_velocity_world_ = measured_velocity_world;
    }

    anchor_position_ = position;
    anchor_orientation_ = orientation;
    prediction_orientation_ = orientation;
    anchor_stamp_ = stamp;
    last_imu_stamp_ = stamp;
    initialized_ = true;
  }

  void receiveImu(const sensor_msgs::msg::Imu & msg)
  {
    if (!initialized_) {
      return;
    }

    const rclcpp::Time stamp(msg.header.stamp);
    if (stamp <= last_imu_stamp_) {
      return;
    }

    const double age_since_odom = (stamp - anchor_stamp_).seconds();
    if (age_since_odom > max_prediction_horizon_sec_) {
      publishPrediction(stamp, false);
      return;
    }

    const double raw_dt = (stamp - last_imu_stamp_).seconds();
    const double dt = std::clamp(raw_dt, 0.0, max_imu_dt_sec_);
    if (dt <= 0.0) {
      return;
    }

    const Eigen::Vector3d angular_velocity(
      msg.angular_velocity.x,
      msg.angular_velocity.y,
      msg.angular_velocity.z);
    if (angular_velocity.allFinite() && angular_velocity.norm() >= gyro_deadband_rad_s_) {
      Eigen::Vector3d bounded_angular_velocity = angular_velocity;
      if (bounded_angular_velocity.norm() > max_angular_velocity_) {
        bounded_angular_velocity = bounded_angular_velocity.normalized() * max_angular_velocity_;
      }
      prediction_orientation_ = integrateGyroBody(prediction_orientation_, bounded_angular_velocity, dt);
    }

    if (use_acceleration_) {
      Eigen::Vector3d acceleration_body(
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z);
      if (acceleration_body.allFinite()) {
        Eigen::Vector3d acceleration_world = prediction_orientation_ * acceleration_body;
        acceleration_world.z() += gravity_mps2_;
        anchor_velocity_world_ += acceleration_world * dt * acceleration_blend_;
        if (anchor_velocity_world_.norm() > max_linear_velocity_) {
          anchor_velocity_world_ = anchor_velocity_world_.normalized() * max_linear_velocity_;
        }
      }
    }

    last_imu_stamp_ = stamp;

    publishPrediction(stamp, false);
  }

  void publishPrediction(const rclcpp::Time & stamp, const bool force)
  {
    if (!force && max_publish_rate_hz_ > 0.0 && last_publish_stamp_.nanoseconds() != 0) {
      const double publish_dt = (stamp - last_publish_stamp_).seconds();
      if (publish_dt < 1.0 / max_publish_rate_hz_) {
        return;
      }
    }
    last_publish_stamp_ = stamp;

    const double age = std::clamp((stamp - anchor_stamp_).seconds(), 0.0, max_prediction_horizon_sec_);
    Eigen::Vector3d offset = anchor_velocity_world_ * age;
    if (offset.norm() > max_position_extrapolation_m_) {
      offset = offset.normalized() * max_position_extrapolation_m_;
    }
    const Eigen::Vector3d predicted_position = anchor_position_ + offset;
    const Eigen::Vector3d predicted_velocity_body =
      prediction_orientation_.inverse() * anchor_velocity_world_;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = fixed_frame_;
    odom_msg.child_frame_id = child_frame_;
    odom_msg.pose.pose.position.x = predicted_position.x();
    odom_msg.pose.pose.position.y = predicted_position.y();
    odom_msg.pose.pose.position.z = predicted_position.z();
    odom_msg.pose.pose.orientation = quatToMsg(prediction_orientation_);
    odom_msg.twist.twist.linear.x = predicted_velocity_body.x();
    odom_msg.twist.twist.linear.y = predicted_velocity_body.y();
    odom_msg.twist.twist.linear.z = predicted_velocity_body.z();
    odom_pub_->publish(odom_msg);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = odom_msg.header;
    pose_msg.pose = odom_msg.pose.pose;
    const bool append_path =
      !last_path_xy_.has_value() ||
      std::hypot(
        predicted_position.x() - last_path_xy_->x(),
        predicted_position.y() - last_path_xy_->y()) >= path_min_distance_m_;
    if (append_path) {
      path_poses_.push_back(pose_msg);
      last_path_xy_ = Eigen::Vector2d(predicted_position.x(), predicted_position.y());
    }
    while (max_path_poses_ > 0 && static_cast<int>(path_poses_.size()) > max_path_poses_) {
      path_poses_.pop_front();
    }

    path_msg_.header.stamp = stamp;
    path_msg_.poses.assign(path_poses_.begin(), path_poses_.end());
    path_pub_->publish(path_msg_);
  }

  std::string odom_topic_;
  std::string imu_topic_;
  std::string predicted_odom_topic_;
  std::string predicted_path_topic_;
  std::string fixed_frame_;
  std::string child_frame_;
  double max_prediction_horizon_sec_ {0.12};
  double max_imu_dt_sec_ {0.05};
  double max_publish_rate_hz_ {60.0};
  int max_path_poses_ {12000};
  double path_min_distance_m_ {0.03};
  bool prefer_odom_twist_ {false};
  bool twist_in_child_frame_ {true};
  bool use_acceleration_ {false};
  double gravity_mps2_ {1.625};
  double acceleration_blend_ {0.15};
  double velocity_smoothing_alpha_ {0.9};
  double stationary_velocity_threshold_ {0.02};
  double gyro_deadband_rad_s_ {0.002};
  double max_linear_velocity_ {2.0};
  double max_angular_velocity_ {3.0};
  double max_position_extrapolation_m_ {0.08};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  bool initialized_ {false};
  rclcpp::Time anchor_stamp_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_stamp_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_stamp_ {0, 0, RCL_ROS_TIME};
  Eigen::Vector3d anchor_position_ {Eigen::Vector3d::Zero()};
  Eigen::Quaterniond anchor_orientation_ {Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond prediction_orientation_ {Eigen::Quaterniond::Identity()};
  Eigen::Vector3d anchor_velocity_world_ {Eigen::Vector3d::Zero()};
  std::optional<Eigen::Vector2d> last_path_xy_;
  std::deque<geometry_msgs::msg::PoseStamped> path_poses_;
  nav_msgs::msg::Path path_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPredictionNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
