#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdometryToTf : public rclcpp::Node
{
public:
  OdometryToTf()
  : Node("odometry_to_tf")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/dlio/odometry");
    parent_frame_override_ = declare_parameter<std::string>("parent_frame", "");
    child_frame_override_ = declare_parameter<std::string>("child_frame", "");

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OdometryToTf::receiveOdometry, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Broadcasting TF from odometry topic %s",
      odom_topic_.c_str());
  }

private:
  void receiveOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const std::string parent_frame =
      parent_frame_override_.empty() ? msg->header.frame_id : parent_frame_override_;
    const std::string child_frame =
      child_frame_override_.empty() ? msg->child_frame_id : child_frame_override_;

    if (parent_frame.empty() || child_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Cannot broadcast odometry TF with empty parent or child frame");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg->header;
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    transform.transform.rotation = msg->pose.pose.orientation;
    broadcaster_->sendTransform(transform);
  }

  std::string odom_topic_;
  std::string parent_frame_override_;
  std::string child_frame_override_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryToTf>());
  rclcpp::shutdown();
  return 0;
}
