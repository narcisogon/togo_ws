/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

/*
File summary:
Declares DLIO's optional local map accumulator. It combines odom-frame keyframe
clouds for visualization and exposes a service that saves the accumulated local
frontend map as PCD.

Important boundary:
This is not the loop-corrected graph-SLAM map. It remains in DLIO's odom frame
and does not apply the backend map -> odom correction.
*/

#include "dlio/dlio.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "direct_lidar_inertial_odometry/srv/save_pcd.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

/*
Summary:
Accumulates and publishes DLIO keyframes as a simple odom-frame point-cloud map.
*/
class dlio::MapNode: public rclcpp::Node {
public:

  /*
  Summary:
  Loads map settings and creates keyframe, map, and save-service ROS interfaces.
  */
  MapNode();

  /*
  Summary:
  Releases the local map node and its accumulated cloud.
  */
  ~MapNode();

  /*
  Summary:
  Provides the same post-construction startup hook used by the DLIO executables.
  */
  void start();
private:

  /*
  Summary:
  Reads the odom frame and sparse-map voxel resolution.
  */
  void getParams();

  /*
  Summary:
  Downsamples one keyframe, appends it to the local map, and republishes the map.
  */
  void callbackKeyframe(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& keyframe);

  /*
  Summary:
  Downsamples a copy of the accumulated local map and writes dlio_map.pcd.
  */
  void savePCD(std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePCD::Request> req,
               std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePCD::Response> res);


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub;
  rclcpp::CallbackGroup::SharedPtr keyframe_cb_group, save_pcd_cb_group;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;

  rclcpp::Service<direct_lidar_inertial_odometry::srv::SavePCD>::SharedPtr save_pcd_srv;

  pcl::PointCloud<PointType>::Ptr dlio_map;
  pcl::VoxelGrid<PointType> voxelgrid;

  std::string odom_frame;

  double leaf_size_;

};
