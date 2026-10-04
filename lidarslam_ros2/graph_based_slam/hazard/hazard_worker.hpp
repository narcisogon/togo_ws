#ifndef TOGO_GRAPH_HAZARD_WORKER_HPP_
#define TOGO_GRAPH_HAZARD_WORKER_HPP_

#include "hazard_mapping.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace graphslam::hazard
{
struct Source
{
  int id {0};
  int64_t stamp {0};
  Eigen::Isometry3d pose {Eigen::Isometry3d::Identity()};
  Eigen::Vector3d up_in_body {Eigen::Vector3d::UnitZ()};
  std::string pcd_path;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
};

// Owns one background thread and publishers ON the existing backend node.
// Queues the newest complete graph snapshot and a bounded batch of fresh scans.
class Worker
{
public:
  Worker(rclcpp::Node & node, std::string map_frame, std::string odom_frame);
  ~Worker();
  void stop();
  bool enabled() const {return enabled_;}
  void receiveOdometry(const nav_msgs::msg::Odometry & odom);
  void receiveCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  std::optional<Eigen::Isometry3d> acquisitionPose(int64_t stamp);
  void updateGraph(std::vector<Source> sources, const Eigen::Isometry3d & correction);

private:
  struct OdomSample {int64_t stamp; Eigen::Isometry3d pose;};
  struct QueuedCloud
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    std::chrono::steady_clock::time_point arrived;
  };
  struct AnchoredCell
  {
    int anchor {-1};  // graph submap ID; -1 is odom before the first submap
    int64_t stamp {0};
    Eigen::Vector3d point;
    Eigen::Vector3d up;
    Eigen::Vector3d candidate_point;
    TerrainState state;
  };
  struct AnchoredObstacle
  {
    int anchor {-1};
    Eigen::Vector3d point;
    ObstacleState state;
  };
  struct Snapshot
  {
    Config config;
    double max_age {1.0};
    double update_period {0.2};
    int batch_clouds {3};
    std::uint64_t config_revision {0};
    std::uint64_t graph_revision {0};
    std::uint64_t geometry_revision {0};
    std::uint64_t cloud_revision {0};
    std::shared_ptr<const std::vector<Source>> sources;
    Eigen::Isometry3d correction {Eigen::Isometry3d::Identity()};
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    std::deque<QueuedCloud> clouds;
    std::uint64_t dropped_clouds {0};
    std::deque<OdomSample> odometry;
    std::string body_frame;
  };
  void run();
  bool process(const Snapshot & snapshot);
  void mergeObservation(const Contribution & cells, int64_t stamp,
    int anchor, const Eigen::Isometry3d & anchor_pose, bool fresh = true);
  std::optional<Eigen::Isometry3d> poseAt(const Snapshot & snapshot, int64_t stamp) const;
  bool rayOrigin(const Snapshot & snapshot, int64_t stamp,
    const Eigen::Isometry3d & pose, Eigen::Vector3d & origin) const;
  // Caller holds mutex_; readiness describes the last published map.
  void publishHealthLocked();
  nav_msgs::msg::OccupancyGrid message(const Grid & grid, bool raw) const;
  rcl_interfaces::msg::SetParametersResult parameters(const std::vector<rclcpp::Parameter> & values);

  rclcpp::Node & node_;
  std::string map_frame_, odom_frame_;
  std::string lidar_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  bool enabled_ {false};
  std::atomic<bool> stopping_ {false};
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable wake_;
  Snapshot pending_;
  std::vector<double> footprint_;
  IncrementalGrid terrain_;  // worker-thread only
  std::unordered_map<Key, AnchoredCell, KeyHash> observations_;
  std::unordered_map<VoxelKey, AnchoredObstacle, VoxelHash> obstacles_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> deferred_rays_;  // worker-thread only
  std::uint64_t dropped_rays_ {0};
  bool terrain_initialized_ {false};
  std::uint64_t terrain_config_ {0}, terrain_geometry_ {0};
  Config last_config_;
  double terrain_resolution_ {0.05};
  std::uint64_t completed_config_ {0}, completed_graph_ {0}, completed_cloud_ {0};
  std::uint64_t completed_geometry_ {0};
  bool have_map_ {false};
  bool job_failed_ {false};
  std::string last_health_reason_;
  int64_t last_source_stamp_ {0};
  nav_msgs::msg::OccupancyGrid last_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_, raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr slope_pub_, cloud_pub_, obstacle_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}  // namespace graphslam::hazard
#endif
