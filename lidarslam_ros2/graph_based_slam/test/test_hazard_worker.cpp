#include "hazard/hazard_worker.hpp"
#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <thread>

using namespace graphslam::hazard;
using namespace std::chrono_literals;

namespace
{
sensor_msgs::msg::PointCloud2::SharedPtr cloud(rclcpp::Time stamp, double slope, double offset)
{
  auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();
  result->header.stamp = stamp; result->header.frame_id = "odom";
  sensor_msgs::PointCloud2Modifier modifier(*result);
  modifier.setPointCloud2FieldsByString(1, "xyz"); modifier.resize(1600);
  sensor_msgs::PointCloud2Iterator<float> x(*result, "x"), y(*result, "y"), z(*result, "z");
  for (int iy = -20; iy < 20; ++iy) {
    for (int ix = -20; ix < 20; ++ix, ++x, ++y, ++z) {
      *x = (ix + 0.5) * 0.05 + offset; *y = (iy + 0.5) * 0.05;
      *z = -0.3 + slope * (ix + 0.5) * 0.05;
    }
  }
  return result;
}

int costAt(const nav_msgs::msg::OccupancyGrid & map, double x, double y)
{
  const int ix = std::floor((x - map.info.origin.position.x) / map.info.resolution);
  const int iy = std::floor((y - map.info.origin.position.y) / map.info.resolution);
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(map.info.width) ||
    iy >= static_cast<int>(map.info.height)) {return -1;}
  return map.data[static_cast<std::size_t>(iy) * map.info.width + ix];
}
}

TEST(HazardWorker, FreshTerrainSurvivesGraphCorrectionAndRemovedTerrainDisappears)
{
  // This test must never join the rover's domain.
  ASSERT_EQ(std::string(std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : ""), "199");
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("hazard_worker_regression",
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("hazard/enabled", true), rclcpp::Parameter("hazard/batch_clouds", 1),
      rclcpp::Parameter("hazard/max_input_age_sec", 5.0),
      rclcpp::Parameter("hazard/footprint", std::vector<double>(8, 0.0))}));
  nav_msgs::msg::OccupancyGrid map;
  std::size_t messages = 0;
  auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>("hazard/raw",
    rclcpp::QoS(1).reliable().transient_local(),
    [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {map = *m; ++messages;});
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
  const auto wait = [&](const auto & condition) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(); if (condition()) {return true;}
      std::this_thread::sleep_for(5ms);
    }
    return false;
  };
  {
    Worker worker(*node, "map", "odom");
    const auto feed = [&](double slope, double offset) {
      const auto stamp = node->now();
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp; odom.header.frame_id = "odom"; odom.child_frame_id = "base_link";
      odom.pose.pose.orientation.w = 1;
      worker.receiveOdometry(odom);
      auto points = cloud(stamp, slope, offset);
      worker.receiveCloud(points);
      return points;
    };
    auto original = feed(std::tan(M_PI / 6), 0);
    Source source; source.id = 0; source.stamp = rclcpp::Time(original->header.stamp).nanoseconds();
    source.cloud = original;
    worker.updateGraph({source}, Eigen::Isometry3d::Identity());
    ASSERT_TRUE(wait([&]() {return costAt(map, 0.025, 0.025) == 100;}));
    auto before_partial = messages;
    std::this_thread::sleep_for(5ms); feed(0, 0);
    ASSERT_TRUE(wait([&]() {return messages > before_partial;}));
    EXPECT_EQ(costAt(map, 0.025, 0.025), 100);
    Eigen::Isometry3d temporary = Eigen::Isometry3d::Identity();
    temporary.translation().x() = 2; source.pose = temporary;
    worker.updateGraph({source}, temporary);
    ASSERT_TRUE(wait([&]() {return costAt(map, 2.025, 0.025) == 100;}));
    EXPECT_EQ(costAt(map, 0.025, 0.025), -1);
    source.pose = Eigen::Isometry3d::Identity();
    worker.updateGraph({source}, Eigen::Isometry3d::Identity());
    ASSERT_TRUE(wait([&]() {return costAt(map, 0.025, 0.025) == 100;}));
    for (int i = 0; i < 8; ++i) {
      const auto before = messages;
      std::this_thread::sleep_for(5ms); feed(0, 0);
      ASSERT_TRUE(wait([&]() {return messages > before;}));
    }
    ASSERT_TRUE(wait([&]() {return messages > 1 && costAt(map, 0.025, 0.025) == 0;}));
    std::this_thread::sleep_for(5ms); feed(0, 4);
    ASSERT_TRUE(wait([&]() {return costAt(map, 4.025, 0.025) == 0;}));
    EXPECT_EQ(costAt(map, 0.025, 0.025), 0);  // Unseen earlier terrain is retained.
    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
    correction.translation().x() = 2;
    source.pose = correction;
    worker.updateGraph({source}, correction);
    ASSERT_TRUE(wait([&]() {return costAt(map, 6.025, 0.025) == 0;}));
    EXPECT_EQ(costAt(map, 2.025, 0.025), 0);  // Latest flat observation beats old steep source.
    EXPECT_EQ(costAt(map, 0.025, 0.025), -1);  // No old-position hazard ghosts.
    EXPECT_EQ(std::count(map.data.begin(), map.data.end(), 100), 0);
    const auto before = messages;
    worker.updateGraph({}, correction);
    ASSERT_TRUE(wait([&]() {return messages > before && costAt(map, 2.025, 0.025) == -1;}));
    EXPECT_EQ(costAt(map, 6.025, 0.025), 0);
  }
  executor.remove_node(node); sub.reset(); node.reset(); rclcpp::shutdown();
}
