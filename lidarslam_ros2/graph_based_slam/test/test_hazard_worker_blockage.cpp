#include "hazard/hazard_worker.hpp"
#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <thread>

using namespace graphslam::hazard;
using namespace std::chrono_literals;

namespace
{
sensor_msgs::msg::PointCloud2::SharedPtr pointCloud(rclcpp::Time stamp,
  const std::vector<Eigen::Vector3d> & points)
{
  auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();
  result->header.stamp = stamp; result->header.frame_id = "odom";
  sensor_msgs::PointCloud2Modifier modifier(*result);
  modifier.setPointCloud2FieldsByString(1, "xyz"); modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(*result, "x"), y(*result, "y"), z(*result, "z");
  for (const auto & p : points) {*x = p.x(); *y = p.y(); *z = p.z(); ++x; ++y; ++z;}
  return result;
}

int costAt(const nav_msgs::msg::OccupancyGrid & map, const Eigen::Vector3d & point)
{
  if (map.info.resolution <= 0) {return -1;}
  const int x = std::floor((point.x() - map.info.origin.position.x) / map.info.resolution);
  const int y = std::floor((point.y() - map.info.origin.position.y) / map.info.resolution);
  if (x < 0 || y < 0 || x >= static_cast<int>(map.info.width) ||
    y >= static_cast<int>(map.info.height)) {return -1;}
  return map.data[static_cast<std::size_t>(y) * map.info.width + x];
}

std::optional<double> probabilityAt(const sensor_msgs::msg::PointCloud2 & cloud,
  const Eigen::Vector3d & point)
{
  if (cloud.data.empty()) {return std::nullopt;}
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"),
    probability(cloud, "intensity");
  for (; x != x.end(); ++x, ++y, ++z, ++probability) {
    if (std::abs(*x - point.x()) < 0.01 && std::abs(*y - point.y()) < 0.01 &&
      std::abs(*z - point.z()) < 0.01) {return *probability;}
  }
  return std::nullopt;
}

Eigen::Vector3d voxelCenter(const Eigen::Vector3d & point)
{
  return {(std::floor(point.x() / 0.05) + 0.5) * 0.05,
    (std::floor(point.y() / 0.05) + 0.5) * 0.05,
    (std::floor(point.z() / 0.10) + 0.5) * 0.10};
}
}  // namespace

TEST(HazardWorkerBlockage, GraphReprojectionPreservesEvidenceAndRemovedSourceDropsBlockage)
{
  ASSERT_EQ(std::string(std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : ""), "199");
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("hazard_worker_blockage_regression",
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("hazard/enabled", true), rclcpp::Parameter("hazard/batch_clouds", 1),
      rclcpp::Parameter("hazard/max_input_age_sec", 30.0),
      rclcpp::Parameter("hazard/footprint", std::vector<double>(8, 0.0))}));
  nav_msgs::msg::OccupancyGrid map;
  sensor_msgs::msg::PointCloud2 obstacles;
  std::size_t map_messages = 0, obstacle_messages = 0;
  auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>("hazard/raw",
    rclcpp::QoS(1).reliable().transient_local(),
    [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {map = *message; ++map_messages;});
  auto obstacle_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("hazard/obstacles",
    rclcpp::QoS(1).reliable().transient_local(),
    [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
      obstacles = *message; ++obstacle_messages;
    });
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
    ASSERT_TRUE(wait([&]() {
      return map_sub->get_publisher_count() > 0 && obstacle_sub->get_publisher_count() > 0;
    }));
    const auto feed = [&](const std::vector<Eigen::Vector3d> & points) {
        const auto stamp = node->now();
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp; odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link"; odom.pose.pose.orientation.w = 1;
        worker.receiveOdometry(odom);
        auto scan = pointCloud(stamp, points); worker.receiveCloud(scan); return scan;
      };
    std::vector<Eigen::Vector3d> ground;
    for (int y = -20; y <= 20; ++y) {
      for (int x = -20; x <= 60; ++x) {ground.emplace_back((x + .5) * .05, (y + .5) * .05, -.3);}
    }
    auto baseline = feed(ground);
    Source source; source.id = 0;
    source.stamp = rclcpp::Time(baseline->header.stamp).nanoseconds(); source.cloud = baseline;
    worker.updateGraph({source}, Eigen::Isometry3d::Identity());
    const Eigen::Vector3d target(1.025, .025, .35);
    ASSERT_TRUE(wait([&]() {return costAt(map, target) == 0;}));
    std::this_thread::sleep_for(5ms);
    auto target_cloud = feed({{1.021, .022, .346}, target, {1.029, .028, .354}});
    ASSERT_TRUE(wait([&]() {
      return costAt(map, target) == 100 && probabilityAt(obstacles, target).has_value();
    }));
    const auto initial_probability = probabilityAt(obstacles, target);
    ASSERT_TRUE(initial_probability.has_value()); EXPECT_NEAR(*initial_probability, 85.0, 1e-4);

    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
    correction.linear() = Eigen::AngleAxisd(M_PI / 18.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
    correction.translation().x() = 2.0; source.pose = correction;
    const auto corrected_center = voxelCenter(correction * target);
    const auto before_correction = map_messages;
    worker.updateGraph({source}, correction);
    ASSERT_TRUE(wait([&]() {
      return map_messages > before_correction && costAt(map, corrected_center) == 100 &&
             probabilityAt(obstacles, corrected_center).has_value();
    }));
    EXPECT_NE(costAt(map, target), 100);
    EXPECT_FALSE(probabilityAt(obstacles, target).has_value());
    EXPECT_NEAR(*probabilityAt(obstacles, corrected_center), *initial_probability, 1e-4);

    // Duplicate scans and unchanged graphs are ignored. A cost-only rebuild
    // reuses the latest cloud while preserving its original evidence count.
    const auto before_replay = obstacle_messages;
    worker.receiveCloud(target_cloud);  // Duplicate acquisition timestamp.
    worker.updateGraph({source}, correction);
    ASSERT_TRUE(node->set_parameters_atomically({
      rclcpp::Parameter("hazard/lethal_inflation_m", 0.11)}).successful);
    ASSERT_TRUE(wait([&]() {return obstacle_messages > before_replay;}));
    ASSERT_TRUE(probabilityAt(obstacles, corrected_center).has_value());
    EXPECT_NEAR(*probabilityAt(obstacles, corrected_center), *initial_probability, 1e-4);

    // Rebuild at the original pose to exercise historical replay a second time.
    source.pose = Eigen::Isometry3d::Identity();
    const auto before_return = obstacle_messages;
    worker.updateGraph({source}, Eigen::Isometry3d::Identity());
    ASSERT_TRUE(wait([&]() {
      return obstacle_messages > before_return && costAt(map, target) == 100 &&
             probabilityAt(obstacles, target).has_value();
    }));
    EXPECT_NE(costAt(map, corrected_center), 100);
    EXPECT_NEAR(*probabilityAt(obstacles, target), *initial_probability, 1e-4);

    const auto before_removal = obstacle_messages;
    const auto maps_before_removal = map_messages;
    worker.updateGraph({}, Eigen::Isometry3d::Identity());
    ASSERT_TRUE(wait([&]() {
      return obstacle_messages > before_removal && map_messages > maps_before_removal &&
             obstacles.width * obstacles.height == 0 && costAt(map, target) == -1;
    }));
    EXPECT_EQ(costAt(map, target), -1);
    EXPECT_FALSE(probabilityAt(obstacles, target).has_value());
  }
  executor.remove_node(node); map_sub.reset(); obstacle_sub.reset(); node.reset(); rclcpp::shutdown();
}
