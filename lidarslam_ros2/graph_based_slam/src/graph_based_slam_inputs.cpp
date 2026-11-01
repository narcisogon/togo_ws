// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
File summary:
Implements direct frontend ingestion and optional GNSS/IMU measurements for the
graph backend. It validates and buffers aiding data, maintains map-to-odom, and
turns synchronized odometry plus point clouds into new graph submaps.
*/

#include "graph_based_slam/graph_based_slam_component.h"
#include "hazard/hazard_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <numeric>
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "graph_based_slam/adjacent_edge_auto_scale.hpp"
#include "graph_based_slam/bev_mutual_visibility.hpp"
#include "graph_based_slam/dynamic_object_filter.hpp"
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace graphslam
{
/*
Summary:
Validates a GNSS fix, resolves its timestamp and covariance-derived weights,
initializes the ENU origin when needed, and buffers the resulting constraint.
*/
void GraphBasedSlamComponent::receiveNavSatFix(const sensor_msgs::msg::NavSatFix & msg)
{
  if (msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    return;  // No valid fix
  }
  if (!isUsableGnssFix(msg)) {
    return;
  }

  std::lock_guard<std::mutex> lock(gnss_mtx_);

  if (!gnss_origin_set_) {
    tryInitializeGnssOrigin(msg.latitude, msg.longitude, msg.altitude);
    if (!gnss_origin_set_) {
      return;
    }
  }

  Eigen::Vector3d enu = geodeticToEnu(msg.latitude, msg.longitude, msg.altitude);
  detail::GnssWeightingConfig weighting_config;
  weighting_config.base_info_weight = gnss_info_weight_;
  weighting_config.vertical_weight_scale = 0.1;
  weighting_config.use_covariance_weighting = gnss_use_covariance_weighting_;
  weighting_config.covariance_min_variance_m2 = gnss_covariance_min_variance_m2_;
  weighting_config.covariance_max_variance_m2 = gnss_covariance_max_variance_m2_;
  weighting_config.rtk_fix_max_horizontal_stddev_m = gnss_rtk_fix_max_horizontal_stddev_m_;
  weighting_config.rtk_fix_weight_scale = gnss_rtk_fix_weight_scale_;
  weighting_config.non_rtk_weight_scale = gnss_non_rtk_weight_scale_;
  const detail::GnssConstraintWeights gnss_weights =
    detail::computeGnssConstraintWeights(msg, weighting_config);
  const double receive_time_sec = get_clock()->now().seconds();
  const double header_time_sec = rclcpp::Time(msg.header.stamp).seconds();
  const detail::GnssTimestampResolution stamp_resolution =
    detail::resolveGnssMeasurementStamp(
    header_time_sec, receive_time_sec, gnss_header_stamp_max_skew_sec_);
  GnssEnu g;
  g.stamp = stamp_resolution.stamp_sec;
  g.x = enu.x();
  g.y = enu.y();
  g.z = enu.z();
  g.info_x = gnss_weights.info_x;
  g.info_y = gnss_weights.info_y;
  g.info_z = gnss_weights.info_z;
  g.covariance_valid = gnss_weights.covariance_valid;
  g.rtk_like = gnss_weights.rtk_like;
  g.horizontal_stddev_m = gnss_weights.horizontal_stddev_m;
  gnss_buffer_.push_back(g);

  if (debug_flag_ && stamp_resolution.used_fallback) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "GNSS header stamp %.3f s differs from receive time %.3f s by more than "
      "%.3f s; using receive time",
      header_time_sec, receive_time_sec, gnss_header_stamp_max_skew_sec_);
  }

  if (debug_flag_ && gnss_weights.covariance_valid) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "GNSS covariance weighting: horizontal_stddev=%.3f m, class=%s, info=(%.3f, %.3f, %.3f)",
      gnss_weights.horizontal_stddev_m,
      gnss_weights.rtk_like ? "rtk_like" : "non_rtk",
      gnss_weights.info_x, gnss_weights.info_y, gnss_weights.info_z);
  }

  // Limit buffer size
  if (gnss_buffer_.size() > 100000) {
    gnss_buffer_.erase(gnss_buffer_.begin(), gnss_buffer_.begin() + 25000);
  }
}

/*
Summary:
Returns whether a NavSatFix has a valid status and finite geodetic coordinates.
*/
bool GraphBasedSlamComponent::isUsableGnssFix(const sensor_msgs::msg::NavSatFix & msg) const
{
  if (!std::isfinite(msg.latitude) || !std::isfinite(msg.longitude) ||
    !std::isfinite(msg.altitude))
  {
    return false;
  }
  if (msg.latitude < -90.0 || msg.latitude > 90.0) {
    return false;
  }
  if (msg.longitude < -180.0 || msg.longitude > 180.0) {
    return false;
  }
  if (std::abs(msg.latitude) < 1e-6 && std::abs(msg.longitude) < 1e-6) {
    return false;
  }
  return true;
}

/*
Summary:
Collects mutually consistent fixes and establishes the local ENU origin once
the configured sample count is reached.
*/
void GraphBasedSlamComponent::tryInitializeGnssOrigin(double lat, double lon, double alt)
{
  GnssOriginSample sample {lat, lon, alt};

  if (!gnss_origin_candidates_.empty()) {
    double mean_lat = 0.0;
    double mean_lon = 0.0;
    double mean_alt = 0.0;
    for (const auto & candidate : gnss_origin_candidates_) {
      mean_lat += candidate.lat;
      mean_lon += candidate.lon;
      mean_alt += candidate.alt;
    }
    mean_lat /= gnss_origin_candidates_.size();
    mean_lon /= gnss_origin_candidates_.size();
    mean_alt /= gnss_origin_candidates_.size();

    const double jump_m = approximateGeodeticDistanceMeters(mean_lat, mean_lon, lat, lon);
    if (jump_m > gnss_origin_consistency_threshold_m_) {
      RCLCPP_WARN(
        get_logger(),
        "Resetting GNSS origin initialization after %.1f m jump in candidate fixes",
        jump_m);
      gnss_origin_candidates_.clear();
    }
  }

  gnss_origin_candidates_.push_back(sample);

  if (static_cast<int>(gnss_origin_candidates_.size()) < gnss_origin_min_samples_) {
    return;
  }

  double mean_lat = 0.0;
  double mean_lon = 0.0;
  double mean_alt = 0.0;
  for (const auto & candidate : gnss_origin_candidates_) {
    mean_lat += candidate.lat;
    mean_lon += candidate.lon;
    mean_alt += candidate.alt;
  }
  mean_lat /= gnss_origin_candidates_.size();
  mean_lon /= gnss_origin_candidates_.size();
  mean_alt /= gnss_origin_candidates_.size();

  double max_deviation_m = 0.0;
  for (const auto & candidate : gnss_origin_candidates_) {
    const double deviation_m = approximateGeodeticDistanceMeters(
      mean_lat, mean_lon, candidate.lat, candidate.lon);
    if (deviation_m > max_deviation_m) {
      max_deviation_m = deviation_m;
    }
  }

  if (max_deviation_m > gnss_origin_consistency_threshold_m_) {
    const GnssOriginSample latest = gnss_origin_candidates_.back();
    gnss_origin_candidates_.clear();
    gnss_origin_candidates_.push_back(latest);
    RCLCPP_WARN(
      get_logger(),
      "GNSS origin candidates were inconsistent (max deviation %.1f m), restarting accumulation",
      max_deviation_m);
    return;
  }

  gnss_origin_lat_ = mean_lat;
  gnss_origin_lon_ = mean_lon;
  gnss_origin_alt_ = mean_alt;
  gnss_origin_set_ = true;
  gnss_origin_candidates_.clear();
  RCLCPP_INFO(
    get_logger(),
    "GNSS origin set from %d consistent fixes: lat=%.8f, lon=%.8f, alt=%.2f",
    gnss_origin_min_samples_, gnss_origin_lat_, gnss_origin_lon_, gnss_origin_alt_);
}

/*
Summary:
Approximates horizontal separation between two latitude/longitude positions.
*/
double GraphBasedSlamComponent::approximateGeodeticDistanceMeters(
  double lat0, double lon0, double lat1, double lon1) const
{
  constexpr double kEarthRadiusM = 6378137.0;
  auto toRad = [](double deg) {return deg * M_PI / 180.0;};

  const double lat0_rad = toRad(lat0);
  const double lat1_rad = toRad(lat1);
  const double dlat = lat1_rad - lat0_rad;
  const double dlon = toRad(lon1 - lon0);
  const double x = dlon * std::cos((lat0_rad + lat1_rad) * 0.5);
  const double y = dlat;
  return std::sqrt(x * x + y * y) * kEarthRadiusM;
}

/*
Summary:
Converts a geodetic position into local east-north-up coordinates at the active
GNSS origin.
*/
Eigen::Vector3d GraphBasedSlamComponent::geodeticToEnu(
  double lat, double lon, double alt) const
{
  // WGS84 parameters
  constexpr double a = 6378137.0;              // semi-major axis [m]
  constexpr double f = 1.0 / 298.257223563;    // flattening
  constexpr double e2 = 2 * f - f * f;         // eccentricity squared

  auto toRad = [](double deg) {return deg * M_PI / 180.0;};

  double lat0 = toRad(gnss_origin_lat_);
  double lon0 = toRad(gnss_origin_lon_);
  double lat1 = toRad(lat);
  double lon1 = toRad(lon);

  double dlat = lat1 - lat0;
  double dlon = lon1 - lon0;
  double dalt = alt - gnss_origin_alt_;

  double sin_lat0 = std::sin(lat0);
  double N = a / std::sqrt(1.0 - e2 * sin_lat0 * sin_lat0);
  double M = a * (1.0 - e2) / std::pow(1.0 - e2 * sin_lat0 * sin_lat0, 1.5);

  // ENU: East = dlon * N * cos(lat), North = dlat * M, Up = dalt
  double east = dlon * N * std::cos(lat0);
  double north = dlat * M;
  double up = dalt;

  return Eigen::Vector3d(east, north, up);
}

/*
Summary:
Stores a timestamped IMU sample and bounds the preintegration buffer size.
*/
void GraphBasedSlamComponent::receiveImu(const sensor_msgs::msg::Imu & msg)
{
  std::lock_guard<std::mutex> lock(imu_mtx_);
  StampedImu imu;
  imu.stamp = rclcpp::Time(msg.header.stamp).seconds();
  imu.gx = msg.angular_velocity.x;
  imu.gy = msg.angular_velocity.y;
  imu.gz = msg.angular_velocity.z;
  imu.ax = msg.linear_acceleration.x;
  imu.ay = msg.linear_acceleration.y;
  imu.az = msg.linear_acceleration.z;
  imu.qx = msg.orientation.x;
  imu.qy = msg.orientation.y;
  imu.qz = msg.orientation.z;
  imu.qw = msg.orientation.w;
  imu_buffer_.push_back(imu);
  if (imu_buffer_.size() > kMaxImuBufferSize) {
    imu_buffer_.erase(imu_buffer_.begin(), imu_buffer_.begin() + kMaxImuBufferSize / 4);
  }
}

/*
Summary:
Integrates buffered angular velocity between two timestamps into a relative
rotation suitable for an adjacent graph constraint.
*/
Eigen::Quaterniond GraphBasedSlamComponent::integrateImuRotation(double t0, double t1) const
{
  // Integrate gyroscope measurements between t0 and t1
  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();

  // Find first IMU sample >= t0
  auto it = std::lower_bound(
    imu_buffer_.begin(), imu_buffer_.end(), t0,
    [](const StampedImu & imu, double t) {return imu.stamp < t;});

  if (it == imu_buffer_.end()) {
    return delta_q;  // no data
  }

  double prev_t = t0;
  for (; it != imu_buffer_.end() && it->stamp <= t1; ++it) {
    double dt = it->stamp - prev_t;
    if (dt <= 0.0 || dt > 0.5) {
      prev_t = it->stamp;
      continue;
    }
    // Small angle quaternion integration
    Eigen::Vector3d omega(it->gx, it->gy, it->gz);
    double angle = omega.norm() * dt;
    if (angle > 1e-10) {
      Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, omega.normalized()));
      delta_q = delta_q * dq;
      delta_q.normalize();
    }
    prev_t = it->stamp;
  }

  return delta_q;
}

/*
Summary:
Recomputes the map-to-odom correction from a frontend odometry pose and its
optimized graph pose.
*/
void GraphBasedSlamComponent::updateMapToOdomCorrection(
  const geometry_msgs::msg::Pose & odom_pose,
  const Eigen::Isometry3d & optimized_map_pose)
{
  Eigen::Affine3d odom_affine;
  tf2::fromMsg(odom_pose, odom_affine);
  const Eigen::Isometry3d odom_pose_iso(odom_affine.matrix());
  const Eigen::Isometry3d correction = optimized_map_pose * odom_pose_iso.inverse();

  std::lock_guard<std::mutex> lock(map_to_odom_mtx_);
  map_to_odom_ = correction;
}

/*
Summary:
Publishes the latest map-to-odom correction with monotonic timestamp handling.
*/
void GraphBasedSlamComponent::publishMapToOdomTf(const rclcpp::Time & stamp)
{
  if (!publish_map_to_odom_tf_) {return;}

  // Odometry callbacks use the sensor stamp while backend/heartbeat paths use
  // this->now(). During rosbag replay the sensor can legitimately lead /clock,
  // so those paths may race and otherwise publish map->odom out of timestamp
  // order. Serialize the complete send and drop regressions before tf2 sees
  // them as TF_OLD_DATA.
  std::lock_guard<std::mutex> publish_lock(map_to_odom_tf_publish_mtx_);
  const rclcpp::Time tf_stamp = stamp +
    rclcpp::Duration::from_seconds(std::max(0.0, map_to_odom_tf_future_offset_sec_));
  if (last_map_to_odom_tf_stamp_ns_ > 0 &&
    tf_stamp.nanoseconds() <= last_map_to_odom_tf_stamp_ns_)
  {
    return;
  }

  Eigen::Isometry3d correction;
  {
    std::lock_guard<std::mutex> lock(map_to_odom_mtx_);
    correction = map_to_odom_;
  }

  geometry_msgs::msg::TransformStamped tf_msg = tf2::eigenToTransform(correction);
  tf_msg.header.stamp = tf_stamp;
  tf_msg.header.frame_id = global_frame_id_;
  tf_msg.child_frame_id = odom_frame_id_;
  broadcaster_.sendTransform(tf_msg);
  last_map_to_odom_tf_stamp_ns_ = tf_stamp.nanoseconds();
}

/*
Summary:
Stores the newest frontend cloud and attempts submap creation with the latest
odometry sample.
*/
void GraphBasedSlamComponent::receiveCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (debug_flag_ && !latest_cloud_) {
    RCLCPP_INFO(get_logger(), "First cloud received, %zu bytes", msg->data.size());
  }
  latest_cloud_ = msg;
  latest_cloud_stamp_ = rclcpp::Time(msg->header.stamp);
  if (hazard_mapping_) {hazard_mapping_->receiveCloud(msg);}
  // When cloud arrives, try to create submap with latest odom
  tryCreateSubmap();
}

/*
Summary:
Stores the newest frontend odometry and attempts submap creation with the latest
point cloud.
*/
void GraphBasedSlamComponent::receiveOdometry(const nav_msgs::msg::Odometry & msg)
{
  // Buffer latest odom
  Eigen::Vector3d pos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
  if (!std::isfinite(pos.x()) || !std::isfinite(pos.y()) || !std::isfinite(pos.z())) {
    return;
  }
  if (debug_flag_ && !latest_odom_valid_) {
    RCLCPP_INFO(get_logger(), "First odom received: (%.2f, %.2f, %.2f)", pos.x(), pos.y(), pos.z());
  }
  latest_odom_ = msg;
  latest_odom_valid_ = true;
  if (hazard_mapping_) {hazard_mapping_->receiveOdometry(msg);}
  publishMapToOdomTf(rclcpp::Time(msg.header.stamp));
  // A cloud may precede the bracketing odometry sample on DDS.
  if (hazard_mapping_ && hazard_mapping_->enabled()) {tryCreateSubmap();}
}

/*
Summary:
Buffers the VIO's pose (the rig's IMU in the VIO's own frame), brought to base_link, for the adjacent VIO edges.
*/
void GraphBasedSlamComponent::receiveVio(const nav_msgs::msg::Odometry & msg)
{
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {return;}
  Eigen::Isometry3d t_vio_rig = Eigen::Isometry3d::Identity();
  t_vio_rig.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  t_vio_rig.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  VioPose sample;
  sample.stamp = rclcpp::Time(msg.header.stamp).seconds();
  sample.pose = t_vio_rig * vio_T_base_rig_.inverse();    // base_link in the VIO's frame
  std::lock_guard<std::mutex> lock(vio_mtx_);
  if (!vio_buffer_.empty()) {
    const VioPose & last = vio_buffer_.back();
    const bool jump = (t_vio_rig.translation() - (last.pose * vio_T_base_rig_).translation()).norm() > vio_edge_frame_jump_m_;
    if (sample.stamp < last.stamp || jump) {
      vio_buffer_.clear();                 // a restarted VIO: a new frame, nothing may span the restart
      ++vio_frame_resets_;
    } else if (sample.stamp == last.stamp) {
      return;
    }
  }
  vio_buffer_.push_back(sample);
  if (vio_buffer_.size() > 400000) {
    vio_buffer_.erase(vio_buffer_.begin(), vio_buffer_.begin() + static_cast<std::ptrdiff_t>(vio_buffer_.size() / 2));
  }
}

/*
Summary:
The VIO's base_link pose at a stamp, interpolated between the two buffered samples around it; false when the
buffer does not bracket the stamp within vio_edge_max_stamp_gap_s.
*/
bool GraphBasedSlamComponent::vioBasePoseAt(double stamp, Eigen::Isometry3d & out) const
{
  std::lock_guard<std::mutex> lock(vio_mtx_);
  if (vio_buffer_.size() < 2) {return false;}
  auto after = std::lower_bound(
    vio_buffer_.begin(), vio_buffer_.end(), stamp,
    [](const VioPose & s, double t) {return s.stamp < t;});
  if (after == vio_buffer_.begin() || after == vio_buffer_.end()) {return false;}
  const VioPose & b = *after;
  const VioPose & a = *(after - 1);
  if (b.stamp - a.stamp > vio_edge_max_stamp_gap_s_) {return false;}
  const double w = (b.stamp - a.stamp) > 1e-9 ? (stamp - a.stamp) / (b.stamp - a.stamp) : 1.0;
  const Eigen::Quaterniond qa(a.pose.linear());
  const Eigen::Quaterniond qb(b.pose.linear());
  out = Eigen::Isometry3d::Identity();
  out.linear() = qa.slerp(w, qb).normalized().toRotationMatrix();
  out.translation() = a.pose.translation() + w * (b.pose.translation() - a.pose.translation());
  return true;
}

/*
Summary:
Creates a new odom-frame submap after sufficient travel, associates optional
GNSS/IMU data, caches its cloud, and notifies graph and mapping consumers.
*/
void GraphBasedSlamComponent::tryCreateSubmap()
{
  if (!latest_odom_valid_ || !latest_cloud_) {return;}

  auto acquisition_odom = latest_odom_;
  if (hazard_mapping_ && hazard_mapping_->enabled()) {
    const auto pose = hazard_mapping_->acquisitionPose(latest_cloud_stamp_.nanoseconds());
    if (!pose) {return;}
    acquisition_odom.pose.pose = tf2::toMsg(*pose);
    acquisition_odom.header.stamp = latest_cloud_->header.stamp;
  }

  Eigen::Vector3d pos(
    acquisition_odom.pose.pose.position.x,
    acquisition_odom.pose.pose.position.y,
    acquisition_odom.pose.pose.position.z);

  // Check distance threshold
  if (last_submap_position_valid_) {
    double dist = (pos - last_submap_position_).norm();
    if (dist < submap_distance_threshold_) {return;}
    if (dist > 100.0) {return;}
    accumulated_distance_ += dist;
  }
  last_submap_position_ = pos;
  last_submap_position_valid_ = true;

  // The pose is frontend odom at submap time. Pose graph optimization later
  // turns the optimized latest submap pose into a map->odom correction.
  lidarslam_msgs::msg::SubMap submap;
  std::vector<uint16_t> submap_gray;      // texture/enabled: gray + 1 per point of this keyframe
  submap.header.stamp = acquisition_odom.header.stamp;
  submap.header.frame_id = global_frame_id_;
  submap.distance = accumulated_distance_;
  submap.pose = acquisition_odom.pose.pose;
  if (odom_input_cloud_in_odom_frame_) {
    static bool warned_odom_cloud_conversion = false;
    if (debug_flag_ && !warned_odom_cloud_conversion) {
      RCLCPP_INFO(
        get_logger(),
        "Odom input cloud is already in odom frame; converting each cloud back to %s before submap storage",
        latest_odom_.child_frame_id.c_str());
      warned_odom_cloud_conversion = true;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr odom_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*latest_cloud_, *odom_cloud);

    Eigen::Affine3d odom_affine;
    tf2::fromMsg(acquisition_odom.pose.pose, odom_affine);
    pcl::transformPointCloud(
      *odom_cloud,
      *local_cloud,
      odom_affine.inverse().matrix().cast<float>());

    pcl::toROSMsg(*local_cloud, submap.cloud);
    submap.cloud.header = latest_cloud_->header;
    if (texture_enabled_) {
      submap_gray = colorizeSubmap(*local_cloud, latest_cloud_->header.stamp, latest_odom_.child_frame_id);
    }
  } else {
    submap.cloud = *latest_cloud_;
    if (texture_enabled_) {
      pcl::PointCloud<pcl::PointXYZI> local_cloud;
      pcl::fromROSMsg(*latest_cloud_, local_cloud);
      submap_gray = colorizeSubmap(local_cloud, latest_cloud_->header.stamp, latest_odom_.child_frame_id);
    }
  }
  submap.cloud.header.frame_id = latest_odom_.child_frame_id;

  // Reserve the index under a short lock. tryCreateSubmap() is the sole
  // appender to map_array_msg_ in odom-input mode (this callback runs
  // serially on the executor thread), so the index read here is still
  // valid when the second lock below re-acquires mtx_ to push the submap.
  int idx;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    idx = static_cast<int>(map_array_msg_.submaps.size());
  }

  // PCD file I/O happens outside mtx_ so it never blocks the map_array
  // subscription callback or the backend worker's snapshot reads.
  if (use_pcd_cache_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(submap.cloud, *cloud);
    saveSubmapToPCD(idx, cloud);
  }

  int n;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    map_array_msg_.header.stamp = latest_odom_.header.stamp;
    map_array_msg_.header.frame_id = global_frame_id_;
    map_array_msg_.submaps.push_back(submap);
    if (texture_enabled_) {submap_gray_.push_back(std::move(submap_gray));}
    n = static_cast<int>(map_array_msg_.submaps.size());

    if (use_pcd_cache_) {
      // Clear cloud data from memory (keep pose and metadata); already
      // saved to disk above.
      map_array_msg_.submaps.back().cloud = sensor_msgs::msg::PointCloud2();
    }

    initial_map_array_received_ = true;
    is_map_array_updated_ = true;
  }
  search_worker_cv_.notify_one();

  {
    lidarslam_msgs::msg::NewSubmap ns;
    ns.header.stamp = submap.header.stamp;
    ns.header.frame_id = global_frame_id_;
    ns.submap_index = static_cast<uint32_t>(idx);
    ns.pose = submap.pose;
    ns.cloud = submap.cloud;
    submap_created_pub_->publish(ns);
  }

  if (n % 50 == 0) {
    RCLCPP_INFO(get_logger(), "Odom input: %d submaps, distance: %.1fm", n, accumulated_distance_);
  }
}

/*
Summary:
Keeps the last texture/image_buffer_s seconds of left-camera images (mono8 or the first channel of a
multi-channel image) for colouring keyframes.
*/
void GraphBasedSlamComponent::receiveImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg->width == 0 || msg->height == 0 || msg->data.empty()) {return;}
  TextureImage image;
  image.stamp = rclcpp::Time(msg->header.stamp).seconds();
  image.width = msg->width;
  image.height = msg->height;
  image.step = msg->step;
  if (msg->encoding == "mono8" || msg->encoding == "8UC1") {
    image.data = msg->data;
  } else {
    // multi-channel 8-bit: keep the first channel (the images are gray anyway)
    const uint32_t channels = msg->width > 0 ? msg->step / msg->width : 1;
    if (channels == 0 || msg->step * msg->height > msg->data.size()) {return;}
    image.step = msg->width;
    image.data.resize(static_cast<std::size_t>(msg->width) * msg->height);
    for (uint32_t v = 0; v < msg->height; ++v) {
      const uint8_t * src = msg->data.data() + static_cast<std::size_t>(v) * msg->step;
      uint8_t * dst = image.data.data() + static_cast<std::size_t>(v) * msg->width;
      for (uint32_t u = 0; u < msg->width; ++u) {dst[u] = src[u * channels];}
    }
  }
  std::lock_guard<std::mutex> lock(texture_mtx_);
  texture_images_.push_back(std::move(image));
  const double oldest_kept = texture_images_.back().stamp - texture_image_buffer_s_;
  while (!texture_images_.empty() && texture_images_.front().stamp < oldest_kept) {
    texture_images_.pop_front();
  }
}

void GraphBasedSlamComponent::receiveCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (msg->k.size() < 9 || msg->k[0] <= 0.0 || msg->k[4] <= 0.0) {return;}
  std::lock_guard<std::mutex> lock(texture_mtx_);
  texture_fx_ = msg->k[0];
  texture_fy_ = msg->k[4];
  texture_cx_ = msg->k[2];
  texture_cy_ = msg->k[5];
  texture_image_width_ = msg->width;
  texture_image_height_ = msg->height;
  texture_intrinsics_valid_ = true;
}

/*
Summary:
Looks the camera's optical frame up in TF (once; the rig is fixed) and keeps base_link -> camera.
*/
bool GraphBasedSlamComponent::textureExtrinsics(const std::string & base_frame)
{
  if (texture_extrinsics_valid_) {return true;}
  try {
    const auto tf = tfbuffer_.lookupTransform(texture_camera_frame_, base_frame, tf2::TimePointZero);
    texture_T_cam_base_ = tf2::transformToEigen(tf.transform);
    texture_extrinsics_valid_ = true;
    const Eigen::Vector3d p = texture_T_cam_base_.inverse().translation();
    RCLCPP_INFO(
      get_logger(), "Textured map: camera %s at (%.3f, %.3f, %.3f) in %s",
      texture_camera_frame_.c_str(), p.x(), p.y(), p.z(), base_frame.c_str());
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000, "Textured map: no TF %s -> %s yet (%s)",
      base_frame.c_str(), texture_camera_frame_.c_str(), e.what());
    return false;
  }
}

/*
Summary:
Gray value per point of a keyframe cloud (base_link axes at the keyframe's stamp) from the image nearest that
stamp: projected through the camera, with a coarse depth buffer so a point behind a nearer one in the same
image cell is not painted with the nearer surface. 0 = not seen (outside the image, occluded, no image).
*/
std::vector<uint16_t> GraphBasedSlamComponent::colorizeSubmap(
  const pcl::PointCloud<pcl::PointXYZI> & local_cloud, const rclcpp::Time & stamp,
  const std::string & base_frame)
{
  std::vector<uint16_t> gray(local_cloud.size(), 0);
  if (!textureExtrinsics(base_frame)) {return gray;}
  TextureImage image;
  double fx, fy, cx, cy;
  {
    std::lock_guard<std::mutex> lock(texture_mtx_);
    if (!texture_intrinsics_valid_ || texture_images_.empty()) {++texture_submaps_without_image_; return gray;}
    const double t = stamp.seconds();
    const TextureImage * best = nullptr;
    for (const auto & im : texture_images_) {
      if (best == nullptr || std::fabs(im.stamp - t) < std::fabs(best->stamp - t)) {best = &im;}
    }
    if (best == nullptr || std::fabs(best->stamp - t) > texture_max_image_age_s_) {
      ++texture_submaps_without_image_;
      return gray;
    }
    image = *best;
    fx = texture_fx_; fy = texture_fy_; cx = texture_cx_; cy = texture_cy_;
  }
  const int cell = std::max(1, texture_depth_cell_px_);
  const int cells_x = (static_cast<int>(image.width) + cell - 1) / cell;
  const int cells_y = (static_cast<int>(image.height) + cell - 1) / cell;
  std::vector<float> nearest(static_cast<std::size_t>(cells_x) * cells_y, std::numeric_limits<float>::infinity());
  std::vector<float> depth(local_cloud.size(), -1.0F);
  std::vector<int> px(local_cloud.size(), -1);
  std::vector<int> py(local_cloud.size(), -1);
  const Eigen::Matrix3d r = texture_T_cam_base_.linear();
  const Eigen::Vector3d tr = texture_T_cam_base_.translation();
  for (std::size_t k = 0; k < local_cloud.size(); ++k) {
    const auto & p = local_cloud.points[k];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    const Eigen::Vector3d c = r * Eigen::Vector3d(p.x, p.y, p.z) + tr;
    if (c.z() < 0.05) {continue;}
    const int u = static_cast<int>(std::floor(fx * c.x() / c.z() + cx));
    const int v = static_cast<int>(std::floor(fy * c.y() / c.z() + cy));
    if (u < 0 || v < 0 || u >= static_cast<int>(image.width) || v >= static_cast<int>(image.height)) {continue;}
    px[k] = u;
    py[k] = v;
    depth[k] = static_cast<float>(c.z());
    float & near = nearest[static_cast<std::size_t>(v / cell) * cells_x + u / cell];
    near = std::min(near, depth[k]);
  }
  std::size_t coloured = 0;
  for (std::size_t k = 0; k < local_cloud.size(); ++k) {
    if (px[k] < 0) {continue;}
    const float near = nearest[static_cast<std::size_t>(py[k] / cell) * cells_x + px[k] / cell];
    if (depth[k] > near + static_cast<float>(texture_depth_tolerance_m_)) {continue;}
    const uint8_t g = image.data[static_cast<std::size_t>(py[k]) * image.step + px[k]];
    gray[k] = static_cast<uint16_t>(g) + 1;
    ++coloured;
  }
  ++texture_submaps_coloured_;
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 30000, "Textured map: keyframe %zu of %zu points coloured (%zu keyframes done, %zu without an image)",
    coloured, local_cloud.size(), texture_submaps_coloured_, texture_submaps_without_image_);
  return gray;
}

}  // namespace graphslam
