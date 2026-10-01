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
Connects graph construction, GTSAM optimization, corrected submap assembly,
map and trajectory publication, DEM snapshot scheduling, and persistent map
saving. This is the main boundary between pose-graph results and downstream
map, terrain, hazard-source, and navigation-facing products.

Threading:
Pose adjustment is serialized with corrected-map publication so the persistent
iSAM2 session and cached assembled outputs are never updated concurrently.
*/

#include "graph_based_slam/graph_based_slam_component.h"

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
#include "graph_based_slam/pose_graph_optimizer.hpp"
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace graphslam
{
namespace
{
/*
Summary:
Converts timed voxel accumulators into deterministic centroid records for the
timed corrected-map output.
*/
std::vector<TimedMapPoint> timedVoxelCentroids(
  const std::unordered_map<
    TimedVoxelKey, TimedVoxelAccumulator, TimedVoxelKeyHash> & voxels)
{
  std::vector<TimedMapPoint> voxelized;
  voxelized.reserve(voxels.size());
  for (const auto & item : voxels) {
    voxelized.push_back(item.second.centroid());
  }
  return voxelized;
}

/*
Summary:
Copies one scalar value into a serialized PointCloud2 record at a byte offset.
*/
void writePointCloud2Field(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::string & name,
  uint32_t offset,
  uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  msg.fields.push_back(field);
}

/*
Summary:
Serializes corrected map points with intensity, timestamp, and source-submap
metadata into a ROS PointCloud2 message.
*/
sensor_msgs::msg::PointCloud2 makeTimedPointCloud2(
  const std::vector<TimedMapPoint> & points,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.is_dense = true;
  // Pad each record to 32 bytes so the FLOAT64 time field remains naturally
  // aligned for every point, including consumers that use typed iterators.
  msg.point_step = 32;
  msg.row_step = msg.point_step * msg.width;
  writePointCloud2Field(msg, "x", 0, sensor_msgs::msg::PointField::FLOAT32);
  writePointCloud2Field(msg, "y", 4, sensor_msgs::msg::PointField::FLOAT32);
  writePointCloud2Field(msg, "z", 8, sensor_msgs::msg::PointField::FLOAT32);
  writePointCloud2Field(msg, "intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
  writePointCloud2Field(msg, "time", 16, sensor_msgs::msg::PointField::FLOAT64);
  writePointCloud2Field(msg, "submap_index", 24, sensor_msgs::msg::PointField::UINT32);
  msg.data.resize(static_cast<std::size_t>(msg.row_step));

  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto & point = points[i];
    uint8_t * dst = msg.data.data() + i * msg.point_step;
    std::memcpy(dst + 0, &point.x, sizeof(float));
    std::memcpy(dst + 4, &point.y, sizeof(float));
    std::memcpy(dst + 8, &point.z, sizeof(float));
    std::memcpy(dst + 12, &point.intensity, sizeof(float));
    std::memcpy(dst + 16, &point.time, sizeof(double));
    std::memcpy(dst + 24, &point.submap_index, sizeof(uint32_t));
  }
  return msg;
}
}  // namespace
namespace
{
struct LoopCandidate
{
  enum class Source
  {
    DISTANCE,
    SCAN_CONTEXT,
    BEV_DESCRIPTOR,
    SOLID_DESCRIPTOR,
    TRIANGLE_DESCRIPTOR
  };

  int index {-1};
  double selection_metric {std::numeric_limits<double>::max()};
  Source source {Source::DISTANCE};
  double yaw_rad {0.0};
  // Recovered SE(3) from the descriptor that proposed this candidate
  // (currently only triangle). Identity unless populated. Used as the NDT
  // initial guess instead of the pose-derived guess when source matches.
  Eigen::Matrix4f relative_transform {Eigen::Matrix4f::Identity()};
  bool has_relative_transform {false};
  // Triangle RANSAC inlier evidence (source == TRIANGLE_DESCRIPTOR only).
  // -1 / -1.0 = not populated. This is the correctness signal used to gate
  // the relaxed-fitness acceptance path further down in the registration
  // loop -- GICP/NDT fitness measures overlap, not correctness, so a
  // thin-overlap reverse-direction revisit can have strong inlier evidence
  // and poor fitness at the same time.
  int triangle_inliers {-1};
  float triangle_inlier_ratio {-1.0f};
};

struct LoopCandidateResult
{
  bool valid {false};
  int index {-1};
  double selection_metric {std::numeric_limits<double>::max()};
  double fitness_score {std::numeric_limits<double>::max()};
  double travel_distance {0.0};
  double euclidean_distance {0.0};
  double translation_delta_m {0.0};
  double rotation_delta_deg {0.0};
  LoopCandidate::Source source {LoopCandidate::Source::DISTANCE};
  bool used_3d_bbs {false};
  double three_d_bbs_score_percentage {0.0};
  double three_d_bbs_elapsed_msec {0.0};
  Eigen::Matrix4f final_transformation {Eigen::Matrix4f::Identity()};
};

/*
Summary:
Returns the stable diagnostic label for the source of an accepted loop edge.
*/
const char * candidate_source_name(LoopCandidate::Source source)
{
  switch (source) {
    case LoopCandidate::Source::SCAN_CONTEXT:
      return "scan_context";
    case LoopCandidate::Source::BEV_DESCRIPTOR:
      return "bev_descriptor";
    case LoopCandidate::Source::SOLID_DESCRIPTOR:
      return "solid_descriptor";
    case LoopCandidate::Source::TRIANGLE_DESCRIPTOR:
      return "triangle_descriptor";
    case LoopCandidate::Source::DISTANCE:
    default:
      return "distance";
  }
}

/*
Summary:
Returns steady-clock milliseconds elapsed since the supplied start time.
*/
double elapsedMillis(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

/*
Summary:
Returns a JSON boolean literal without allocating a temporary string.
*/
const char * jsonBool(bool value)
{
  return value ? "true" : "false";
}

// Used by doPoseAdjustment()'s per-submap assembly cache to decide whether a
// submap's optimized pose moved enough since the cached contribution was
// built to warrant recomputing it (PCD reload + transform + serialize).
// Re-running a batch optimizer from scratch can produce tiny floating-point
// differences even for vertices whose local subgraph didn't change (solver
// ordering effects), so this is an epsilon comparison, not exact equality.
/*
Summary:
Tests whether two optimized submap poses are close enough to reuse assembled
map products.
*/
bool posesNearlyEqual(
  const Eigen::Isometry3d & a, const Eigen::Isometry3d & b,
  double translation_eps_m, double rotation_eps_rad)
{
  if ((a.translation() - b.translation()).norm() > translation_eps_m) {
    return false;
  }
  const Eigen::AngleAxisd angle_axis(a.rotation().transpose() * b.rotation());
  return std::abs(angle_axis.angle()) <= rotation_eps_rad;
}

// Robust ground height: the z of the lowest decile of points. Used to
// pre-shift a loop candidate's NDT/GICP initial guess in z before
// registration -- odometry drift is typically worst in z (least observable
// axis for ground vehicles), so the pose-composed initial guess can be off
// by the full accumulated drift with no correction. Registration's
// correspondence search only reaches so far; a bad z guess alone can be
// enough to keep the correct alignment out of reach. nth_element on a copy
// keeps this O(n) without disturbing the caller's point order.
/*
Summary:
Estimates a robust low-percentile ground height for vertical loop-edge handling.
*/
float robustGroundZ(const pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  if (cloud.empty()) {return 0.0f;}
  std::vector<float> zs;
  zs.reserve(cloud.points.size());
  for (const auto & p : cloud.points) {
    zs.push_back(p.z);
  }
  const std::size_t k = std::min(zs.size() - 1, zs.size() / 10);
  std::nth_element(zs.begin(), zs.begin() + k, zs.end());
  return zs[k];
}
}  // namespace

/*
Summary:
Builds the complete current pose graph, runs the selected GTSAM backend, and
reconstructs corrected outputs from the optimized submap poses.

Inputs:
- map_array_msg: Snapshot of current submap poses, timestamps, and clouds.
- loop_edges: Accepted loop constraints to include in the graph.
- do_save_map: Requests persistent pose-graph and point-cloud map output.

Important behavior:
The function serializes optimizer and corrected-map state, reuses unchanged
submap contributions when safe, updates map-to-odom, schedules DEM snapshots,
and publishes timing diagnostics. The optimizer changes corrected geometry but
does not apply downstream hazard or Nav2 policy.
*/
void GraphBasedSlamComponent::doPoseAdjustment(
  lidarslam_msgs::msg::MapArray map_array_msg,
  const LoopEdges & loop_edges,
  bool do_save_map)
{
  std::lock_guard<std::mutex> publish_lock(modified_map_publish_mtx_);

  // A new loop constraint changes the corrected geometry globally, and an
  // explicit map save is expected to capture the newest state. Do not let the
  // normal periodic DEM rate limit suppress either event.
  const bool force_dem_update =
    do_save_map || loop_edges.size() != cached_map_loop_edge_count_;

  const auto adjustment_start = std::chrono::steady_clock::now();
  auto stage_start = adjustment_start;

  optimization::PoseGraphProblem pose_graph;
  pose_graph.fixed_pose_index = 0;
  pose_graph.max_iterations = 10;

  const int submaps_size = static_cast<int>(map_array_msg.submaps.size());
  pose_graph.initial_poses.reserve(static_cast<std::size_t>(submaps_size));
  std::vector<Eigen::Isometry3d> raw_poses;
  raw_poses.reserve(static_cast<std::size_t>(submaps_size));
  std::vector<std::size_t> adjacent_constraint_indices;
  for (int i = 0; i < submaps_size; i++) {
    Eigen::Affine3d affine;
    Eigen::fromMsg(map_array_msg.submaps[i].pose, affine);
    Eigen::Isometry3d pose(affine.matrix());

    // Warm-start the vertex from this submap's last optimized pose (cached
    // by the incremental assembly loop below) when available, instead of
    // always restarting from raw odometry. This does NOT change
    // relative_pose below -- that stays derived from raw-odometry `pose`,
    // as required for a correct adjacent-edge measurement. It only changes
    // where the solver starts, which is what lets the assembly cache's
    // epsilon-based pose-reuse check actually hit for submaps a loop edge
    // hasn't newly touched (see the comment on submap_assembly_cache_ in
    // the header for why re-solving from a fixed raw-odometry seed every
    // call defeated that cache once the graph had a loop edge in it).
    Eigen::Isometry3d initial_estimate = pose;
    const auto warm_start_it = submap_assembly_cache_.find(i);
    if (warm_start_it != submap_assembly_cache_.end()) {
      initial_estimate = warm_start_it->second.pose;
    }

    raw_poses.push_back(pose);
    pose_graph.initial_poses.push_back(initial_estimate);

    if (i > 0) {
      const int start_idx = std::max(0, i - num_adjacent_pose_cnstraints_);
      for (int pre_idx = start_idx; pre_idx < i; pre_idx++) {
        const Eigen::Isometry3d & pre_pose = raw_poses[static_cast<std::size_t>(pre_idx)];
        Eigen::Isometry3d relative_pose = pre_pose.inverse() * pose;

        const int separation = i - pre_idx;
        const double sep_d = static_cast<double>(separation);
        optimization::BetweenConstraint constraint;
        constraint.from = pre_idx;
        constraint.to = i;
        constraint.measurement = relative_pose;
        constraint.kind = optimization::ConstraintKind::ADJACENT;
        if (adjacent_edge_info_auto_scale_split_trans_rot_) {
          // Block-diag with independent translation / rotation weights, each
          // attenuated by edge separation just like the unified scalar path.
          const double w_trans = adjacent_edge_info_weight_trans_ / sep_d;
          const double w_rot = adjacent_edge_info_weight_rot_ / sep_d;
          constraint.translation_information.diagonal().setConstant(w_trans);
          constraint.rotation_information.diagonal().setConstant(w_rot);
        } else {
          const double edge_weight = adjacent_edge_info_weight_ / sep_d;
          constraint.translation_information = Eigen::Matrix3d::Identity() * edge_weight;
          constraint.rotation_information = Eigen::Matrix3d::Identity() * edge_weight;
        }
        // Neither branch above distinguishes z from xy within translation --
        // scale just the z diagonal entry (index 2) relative to whatever
        // weight was just set. 1.0 default = no change either branch.
        constraint.translation_information(2, 2) *= adjacent_edge_info_weight_z_scale_;
        adjacent_constraint_indices.push_back(pose_graph.between_constraints.size());
        pose_graph.between_constraints.push_back(std::move(constraint));
      }
    }
  }
  /* IMU rotation constraint edges */
  if (use_imu_preintegration_ && submaps_size > 1) {
    std::lock_guard<std::mutex> imu_lock(imu_mtx_);
    int imu_edges_added = 0;
    for (int i = 1; i < submaps_size; i++) {
      double t0 = rclcpp::Time(map_array_msg.submaps[i - 1].header.stamp).seconds();
      double t1 = rclcpp::Time(map_array_msg.submaps[i].header.stamp).seconds();
      if (t1 <= t0 || t1 - t0 > 30.0) {continue;}

      Eigen::Quaterniond imu_delta_q = integrateImuRotation(t0, t1);
      if (imu_delta_q.isApprox(Eigen::Quaterniond::Identity(), 1e-8)) {continue;}

      // Build relative pose measurement: translation from odometry, rotation from IMU
      Eigen::Affine3d affine_prev, affine_curr;
      Eigen::fromMsg(map_array_msg.submaps[i - 1].pose, affine_prev);
      Eigen::fromMsg(map_array_msg.submaps[i].pose, affine_curr);
      Eigen::Isometry3d odom_prev(affine_prev.matrix());
      Eigen::Isometry3d odom_curr(affine_curr.matrix());
      Eigen::Isometry3d odom_relative = odom_prev.inverse() * odom_curr;

      // Replace rotation with IMU-integrated rotation
      Eigen::Isometry3d imu_relative = Eigen::Isometry3d::Identity();
      imu_relative.linear() = imu_delta_q.toRotationMatrix();
      imu_relative.translation() = odom_relative.translation();

      optimization::BetweenConstraint constraint;
      constraint.from = i - 1;
      constraint.to = i;
      constraint.measurement = imu_relative;
      constraint.kind = optimization::ConstraintKind::IMU_ROTATION;
      constraint.rotation_information(0, 0) = imu_rotation_info_roll_pitch_;
      constraint.rotation_information(1, 1) = imu_rotation_info_roll_pitch_;
      constraint.rotation_information(2, 2) = imu_rotation_info_yaw_;
      // Translation stays zero: accelerometer double integration is not used.
      pose_graph.between_constraints.push_back(std::move(constraint));
      imu_edges_added++;
    }
    if (debug_flag_) {
      RCLCPP_INFO(get_logger(), "Added %d IMU rotation constraint edges", imu_edges_added);
    }
  }

  /* loop edge */
  const auto loop_kernel_type =
    graphslam::robust::parseLoopEdgeKernelType(loop_edge_robust_kernel_type_);
  for (const auto & loop_edge : loop_edges) {
    optimization::BetweenConstraint constraint;
    constraint.from = loop_edge.pair_id.first;
    constraint.to = loop_edge.pair_id.second;
    constraint.measurement = loop_edge.relative_pose;
    constraint.kind = optimization::ConstraintKind::LOOP;
    const double fitness = std::max(loop_edge.fitness_score, 1e-3);
    const double loop_weight = loop_edge_info_weight_ / fitness;
    constraint.translation_information = Eigen::Matrix3d::Identity() * loop_weight;
    constraint.rotation_information = Eigen::Matrix3d::Identity() * loop_weight;
    constraint.robust = true;
    constraint.robust_kernel = loop_kernel_type;
    constraint.robust_delta = loop_edge_robust_kernel_delta_;
    pose_graph.between_constraints.push_back(std::move(constraint));
  }

  /* GNSS position constraints */
  if (use_gnss_ && gnss_origin_set_) {
    std::lock_guard<std::mutex> gnss_lock(gnss_mtx_);
    int gnss_edges_added = 0;
    int gnss_rtk_like_edges_added = 0;
    int gnss_edges_rejected_by_residual = 0;

    for (int i = 0; i < submaps_size; i++) {
      double submap_time = rclcpp::Time(map_array_msg.submaps[i].header.stamp).seconds();
      // Find nearest GNSS measurement
      double best_dt = std::numeric_limits<double>::max();
      GnssEnu best_gnss;
      bool found = false;
      for (const auto & g : gnss_buffer_) {
        double dt = std::abs(g.stamp - submap_time);
        if (dt < best_dt) {
          best_dt = dt;
          best_gnss = g;
          found = true;
        }
      }
      if (!found || best_dt > 1.0) {continue;}  // Skip if no GNSS within 1 second

      const Eigen::Vector3d submap_position =
        pose_graph.initial_poses[static_cast<std::size_t>(i)].translation();
      const double horizontal_residual = std::hypot(
        best_gnss.x - submap_position.x(),
        best_gnss.y - submap_position.y());
      if (gnss_max_horizontal_residual_m_ > 0.0 &&
        horizontal_residual > gnss_max_horizontal_residual_m_)
      {
        ++gnss_edges_rejected_by_residual;
        continue;
      }

      optimization::PositionConstraint constraint;
      constraint.pose_index = i;
      constraint.position = Eigen::Vector3d(best_gnss.x, best_gnss.y, best_gnss.z);
      constraint.information.diagonal() =
        Eigen::Vector3d(best_gnss.info_x, best_gnss.info_y, best_gnss.info_z);
      pose_graph.position_constraints.push_back(std::move(constraint));
      if (best_gnss.rtk_like) {
        gnss_rtk_like_edges_added++;
      }
      gnss_edges_added++;
    }
    if (debug_flag_) {
      RCLCPP_INFO(
        get_logger(),
        "Added %d GNSS position constraint edges (%d RTK-like by covariance); "
        "rejected %d above %.1f m horizontal residual",
        gnss_edges_added, gnss_rtk_like_edges_added, gnss_edges_rejected_by_residual,
        gnss_max_horizontal_residual_m_);
    }
  }

  // Graph serialization remains tied to an explicit map save, and now lands
  // alongside the other map artifacts instead of in the process cwd.
  if (do_save_map) {
    std::error_code create_error;
    std::filesystem::create_directories(map_save_dir_, create_error);
    if (create_error) {
      RCLCPP_WARN(
        get_logger(), "Could not create map_save_dir '%s': %s",
        map_save_dir_.c_str(), create_error.message().c_str());
    }
    pose_graph.save_path = map_save_dir_ + "/pose_graph.g2o";
  }

  const auto backend = optimization::parsePoseGraphBackend(optimizer_backend_);
  if (!pose_graph_optimizer_ || pose_graph_optimizer_->backend() != backend) {
    pose_graph_optimizer_ = std::make_unique<optimization::PoseGraphOptimizer>(backend);
  }
  const auto optimization_result = pose_graph_optimizer_->optimize(pose_graph);
  if (!optimization_result.success) {
    RCLCPP_ERROR(
      get_logger(), "Pose graph optimization with %s failed: %s",
      optimization::poseGraphBackendName(backend),
      optimization_result.error_message.c_str());
    return;
  }
  if (optimization_result.graph_save_requested && !optimization_result.graph_saved) {
    RCLCPP_WARN(
      get_logger(), "Pose graph solved but graph serialization failed: %s",
      optimization_result.graph_save_error.c_str());
  }
  const auto & optimized_poses = optimization_result.poses;
  const double graph_build_ms = optimization_result.graph_build_ms;
  const double optimize_ms = optimization_result.optimize_ms;
  const double graph_save_ms = optimization_result.graph_save_ms;

  stage_start = std::chrono::steady_clock::now();
  if (adjacent_edge_info_auto_scale_ && !adjacent_constraint_indices.empty()) {
    graphslam::detail::AutoScaleConfig cfg;
    cfg.ema_alpha = adjacent_edge_info_auto_scale_ema_alpha_;
    cfg.min_scale = adjacent_edge_info_auto_scale_min_;
    cfg.max_scale = adjacent_edge_info_auto_scale_max_;

    if (adjacent_edge_info_auto_scale_split_trans_rot_) {
      // Level 2: split the post-opt residuals into translation / rotation
      // blocks and rescale w_trans and w_rot independently. For diagonal
      // block-diag Information matrices, trans_chi2 = w_trans *
      // ||e.head<3>()||^2 and rot_chi2 = w_rot * ||e.tail<3>()||^2.
      std::vector<double> trans_chi2_values;
      std::vector<double> rot_chi2_values;
      trans_chi2_values.reserve(adjacent_constraint_indices.size());
      rot_chi2_values.reserve(adjacent_constraint_indices.size());
      for (const std::size_t constraint_index : adjacent_constraint_indices) {
        const auto & residual = optimization_result.constraint_residuals[constraint_index];
        const auto & constraint = pose_graph.between_constraints[constraint_index];
        // For the block-diag construction above, the diagonals encode the
        // per-block scale of I_3 already attenuated by separation, so
        // multiplying ||delta||^2 by the leading diagonal of each block
        // reproduces the standard chi^2 contribution of that block.
        const double w_t = constraint.translation_information(0, 0);
        const double w_r = constraint.rotation_information(0, 0);
        const double trans = w_t * residual.translation.squaredNorm();
        const double rot = w_r * residual.rotation.squaredNorm();
        if (std::isfinite(trans)) {
          trans_chi2_values.push_back(trans);
        }
        if (std::isfinite(rot)) {
          rot_chi2_values.push_back(rot);
        }
      }
      const double median_chi2_trans = graphslam::detail::medianChi2(trans_chi2_values);
      const double median_chi2_rot = graphslam::detail::medianChi2(rot_chi2_values);

      cfg.target_nis = adjacent_edge_info_auto_scale_target_nis_trans_;
      const double prev_w_trans = adjacent_edge_info_weight_trans_;
      adjacent_edge_info_weight_trans_ =
        graphslam::detail::nextScale(prev_w_trans, median_chi2_trans, cfg);

      cfg.target_nis = adjacent_edge_info_auto_scale_target_nis_rot_;
      const double prev_w_rot = adjacent_edge_info_weight_rot_;
      adjacent_edge_info_weight_rot_ =
        graphslam::detail::nextScale(prev_w_rot, median_chi2_rot, cfg);

      RCLCPP_INFO(
        get_logger(),
        "[auto_scale_split] trans median_chi2=%.3f target=%.3f w_trans=%.3f -> %.3f | "
        "rot median_chi2=%.3f target=%.3f w_rot=%.3f -> %.3f (n=%zu)",
        median_chi2_trans, adjacent_edge_info_auto_scale_target_nis_trans_,
        prev_w_trans, adjacent_edge_info_weight_trans_,
        median_chi2_rot, adjacent_edge_info_auto_scale_target_nis_rot_,
        prev_w_rot, adjacent_edge_info_weight_rot_,
        trans_chi2_values.size());
    } else {
      std::vector<double> chi2_values;
      chi2_values.reserve(adjacent_constraint_indices.size());
      for (const std::size_t constraint_index : adjacent_constraint_indices) {
        const double v = optimization_result.constraint_residuals[constraint_index].chi2;
        if (std::isfinite(v)) {
          chi2_values.push_back(v);
        }
      }
      const double median_chi2 = graphslam::detail::medianChi2(chi2_values);

      cfg.target_nis = adjacent_edge_info_auto_scale_target_nis_;
      const double prev_weight = adjacent_edge_info_weight_;
      adjacent_edge_info_weight_ =
        graphslam::detail::nextScale(prev_weight, median_chi2, cfg);

      RCLCPP_INFO(
        get_logger(),
        "[auto_scale] median_chi2=%.3f (n=%zu) target=%.3f weight=%.3f -> %.3f",
        median_chi2, chi2_values.size(), cfg.target_nis, prev_weight,
        adjacent_edge_info_weight_);
    }
  }
  const double auto_scale_ms = elapsedMillis(stage_start);

  /* modified_map publish */
  stage_start = std::chrono::steady_clock::now();
  std::cout << "modified_map publish" << std::endl;
  const double timed_leaf_size =
    modified_map_timed_leaf_size_ >= 0.0 ?
    modified_map_timed_leaf_size_ : modified_map_leaf_size_;
  bool append_only = assembled_products_valid_ &&
    assembled_submap_count_ >= 0 &&
    assembled_submap_count_ <= submaps_size &&
    assembled_submap_poses_.size() == static_cast<std::size_t>(assembled_submap_count_) &&
    !graph_dirty_.load() &&
    loop_edges.size() == cached_map_loop_edge_count_ &&
    (!publish_modified_map_timed_ ||
    std::abs(assembled_timed_voxel_leaf_size_ - timed_leaf_size) < 1.0e-9) &&
    !(do_save_map && use_dynamic_object_filter_);
  if (append_only) {
    for (int i = 0; i < assembled_submap_count_; ++i) {
      if (!posesNearlyEqual(
          assembled_submap_poses_[static_cast<std::size_t>(i)],
          optimized_poses[static_cast<std::size_t>(i)],
          submap_assembly_reuse_translation_eps_m_,
          submap_assembly_reuse_rotation_eps_deg_ * M_PI / 180.0))
      {
        append_only = false;
        break;
      }
    }
  }

  const int assembly_start_submap = append_only ? assembled_submap_count_ : 0;
  if (!append_only) {
    assembled_map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    assembled_timed_map_points_.clear();
    assembled_timed_voxels_.clear();
    assembled_timed_input_point_count_ = 0;
    assembled_timed_voxel_leaf_size_ = timed_leaf_size;
    assembled_map_array_msg_ = lidarslam_msgs::msg::MapArray();
    assembled_path_msg_ = nav_msgs::msg::Path();
    assembled_submap_poses_.clear();
    assembled_submap_count_ = 0;
  }
  assembled_map_array_msg_.header = map_array_msg.header;
  assembled_map_array_msg_.cloud_coordinate = assembled_map_array_msg_.LOCAL;
  assembled_path_msg_.header.frame_id = global_frame_id_;

  auto & modified_map_array_msg = assembled_map_array_msg_;
  auto & path = assembled_path_msg_;
  auto map_ptr = assembled_map_cloud_;
  auto & timed_map_points = assembled_timed_map_points_;
  double timed_voxel_update_ms = 0.0;
  std::vector<TimedSubmapCloud> dynamic_filter_submaps;
  if (do_save_map && use_dynamic_object_filter_) {
    dynamic_filter_submaps.reserve(submaps_size);
  }
  for (int i = assembly_start_submap; i < submaps_size; i++) {
    const Eigen::Isometry3d & se3 = optimized_poses[static_cast<std::size_t>(i)];
    geometry_msgs::msg::Pose pose = tf2::toMsg(se3);

    /* map: reuse the cached transformed cloud + serialized message for this
       submap if its optimized pose hasn't moved since the cache was built;
       otherwise reload from PCD, retransform, and refresh the cache entry. */
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr;
    sensor_msgs::msg::PointCloud2::SharedPtr local_cloud_msg_ptr(
      new sensor_msgs::msg::PointCloud2);

    const auto cache_it = submap_assembly_cache_.find(i);
    const bool cache_hit = cache_it != submap_assembly_cache_.end() &&
      posesNearlyEqual(
        cache_it->second.pose, se3,
        submap_assembly_reuse_translation_eps_m_,
        submap_assembly_reuse_rotation_eps_deg_ * M_PI / 180.0);

    if (cache_hit) {
      transformed_cloud_ptr = cache_it->second.transformed_cloud;
      *local_cloud_msg_ptr = cache_it->second.local_cloud_msg;
    } else {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr;
      if (use_pcd_cache_) {
        cloud_ptr = loadSubmapFromPCD(i);
      } else {
        cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(map_array_msg.submaps[i].cloud, *cloud_ptr);
      }
      transformed_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, se3.matrix().cast<float>());
      pcl::toROSMsg(*cloud_ptr, *local_cloud_msg_ptr);
      local_cloud_msg_ptr->header = map_array_msg.submaps[i].cloud.header;

      SubmapAssemblyCacheEntry entry;
      entry.pose = se3;
      entry.transformed_cloud = transformed_cloud_ptr;
      entry.local_cloud_msg = *local_cloud_msg_ptr;
      submap_assembly_cache_[i] = std::move(entry);
    }

    *map_ptr += *transformed_cloud_ptr;
    if (publish_modified_map_timed_) {
      const auto timed_update_start = std::chrono::steady_clock::now();
      const double submap_time =
        rclcpp::Time(map_array_msg.submaps[i].header.stamp).seconds();
      const uint32_t submap_index = static_cast<uint32_t>(std::max(0, i));
      if (timed_leaf_size > 0.0) {
        for (const auto & point : transformed_cloud_ptr->points) {
          TimedVoxelKey key;
          key.ix = static_cast<int>(std::floor(point.x / timed_leaf_size));
          key.iy = static_cast<int>(std::floor(point.y / timed_leaf_size));
          key.iz = static_cast<int>(std::floor(point.z / timed_leaf_size));
          assembled_timed_voxels_[key].add(point, submap_time, submap_index);
        }
      } else {
        timed_map_points.reserve(timed_map_points.size() + transformed_cloud_ptr->size());
        for (const auto & point : transformed_cloud_ptr->points) {
          TimedMapPoint timed_point;
          timed_point.x = point.x;
          timed_point.y = point.y;
          timed_point.z = point.z;
          timed_point.intensity = point.intensity;
          timed_point.time = submap_time;
          timed_point.submap_index = submap_index;
          timed_map_points.push_back(timed_point);
        }
      }
      assembled_timed_input_point_count_ += transformed_cloud_ptr->size();
      timed_voxel_update_ms += elapsedMillis(timed_update_start);
    }
    if (do_save_map && use_dynamic_object_filter_) {
      dynamic_filter_submaps.push_back(
        TimedSubmapCloud{
          i,
          Eigen::Vector3d(se3.translation().x(), se3.translation().y(), se3.translation().z()),
          transformed_cloud_ptr});
    }

    /* submap */
    lidarslam_msgs::msg::SubMap submap;
    submap.header = map_array_msg.submaps[i].header;
    submap.pose = pose;
    // Keep MapArray's declared LOCAL coordinate contract.  The previous
    // implementation stored the already map-transformed cloud here while
    // leaving cloud_coordinate at its default LOCAL value.  Late-joining
    // per-submap consumers then had no safe way to reconstruct local hazard
    // patches.  /modified_map remains the assembled global cloud above.
    submap.cloud = *local_cloud_msg_ptr;
    submap.cloud.header = map_array_msg.submaps[i].cloud.header;
    modified_map_array_msg.submaps.push_back(submap);

    /* path */
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = submap.header;
    pose_stamped.pose = submap.pose;
    path.poses.push_back(pose_stamped);
    assembled_submap_poses_.push_back(se3);
  }
  assembled_submap_count_ = submaps_size;
  assembled_products_valid_ = true;
  const double map_assembly_ms = elapsedMillis(stage_start);
  stage_start = std::chrono::steady_clock::now();

  double map_to_odom_tf_ms = 0.0;
  double map_array_publish_ms = 0.0;
  double path_publish_ms = 0.0;
  double dem_schedule_ms = 0.0;
  double regular_voxelize_ms = 0.0;
  double regular_serialize_ms = 0.0;
  double regular_publish_call_ms = 0.0;
  double regular_cache_update_ms = 0.0;
  double timed_voxelize_ms = 0.0;
  double timed_serialize_ms = 0.0;
  double timed_publish_call_ms = 0.0;
  double timed_cache_update_ms = 0.0;
  const bool map_array_published =
    !append_only || do_save_map || publish_modified_map_array_on_append_;

  auto publish_stage_start = std::chrono::steady_clock::now();
  if (use_odom_input_ && publish_map_to_odom_tf_ && submaps_size > 0) {
    updateMapToOdomCorrection(
      map_array_msg.submaps[submaps_size - 1].pose,
      optimized_poses[static_cast<std::size_t>(submaps_size - 1)]);
    publishMapToOdomTf(this->now());
  }
  map_to_odom_tf_ms = elapsedMillis(publish_stage_start);

  if (map_array_published) {
    publish_stage_start = std::chrono::steady_clock::now();
    modified_map_array_pub_->publish(modified_map_array_msg);
    map_array_publish_ms = elapsedMillis(publish_stage_start);
  }

  publish_stage_start = std::chrono::steady_clock::now();
  modified_path_pub_->publish(path);
  path_publish_ms = elapsedMillis(publish_stage_start);

  const auto publish_stamp = this->now();
  // DEM consumes the full corrected source cloud before the optional
  // visualization voxel filter. This keeps its measurement-count threshold
  // meaningful and makes DEM recording independent of /modified_map.
  publish_stage_start = std::chrono::steady_clock::now();
  scheduleDemJob(map_ptr, publish_stamp, force_dem_update);
  dem_schedule_ms = elapsedMillis(publish_stage_start);

  if (publish_modified_map_ && modified_map_pub_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_to_publish = map_ptr;
    publish_stage_start = std::chrono::steady_clock::now();
    if (modified_map_leaf_size_ > 0.0 && !map_ptr->empty()) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_map(
        new pcl::PointCloud<pcl::PointXYZI>);
      pcl::VoxelGrid<pcl::PointXYZI> publish_voxelgrid;
      publish_voxelgrid.setInputCloud(map_ptr);
      const auto leaf_size = static_cast<float>(modified_map_leaf_size_);
      publish_voxelgrid.setLeafSize(leaf_size, leaf_size, leaf_size);
      publish_voxelgrid.filter(*downsampled_map);
      map_to_publish = downsampled_map;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Voxelized /modified_map points %zu -> %zu leaf=%.3fm",
        map_ptr->size(), map_to_publish->size(), modified_map_leaf_size_);
    }
    regular_voxelize_ms = elapsedMillis(publish_stage_start);

    publish_stage_start = std::chrono::steady_clock::now();
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_to_publish, map_msg);
    map_msg.header.frame_id = global_frame_id_;
    map_msg.header.stamp = publish_stamp;
    regular_serialize_ms = elapsedMillis(publish_stage_start);

    publish_stage_start = std::chrono::steady_clock::now();
    modified_map_pub_->publish(map_msg);
    regular_publish_call_ms = elapsedMillis(publish_stage_start);

    // Cache the assembled message so publishMapAndPose() can cheaply
    // re-stamp and republish it when the graph is unchanged.
    publish_stage_start = std::chrono::steady_clock::now();
    cached_modified_map_msg_ = std::move(map_msg);
    regular_cache_update_ms = elapsedMillis(publish_stage_start);
  }
  if (publish_modified_map_timed_ && modified_map_timed_pub_) {
    publish_stage_start = std::chrono::steady_clock::now();
    const auto timed_points_to_publish =
      timed_leaf_size > 0.0 ?
      timedVoxelCentroids(assembled_timed_voxels_) : timed_map_points;
    timed_voxelize_ms = elapsedMillis(publish_stage_start);

    std_msgs::msg::Header timed_header;
    timed_header.frame_id = global_frame_id_;
    timed_header.stamp = publish_stamp;
    publish_stage_start = std::chrono::steady_clock::now();
    auto timed_msg = makeTimedPointCloud2(timed_points_to_publish, timed_header);
    timed_serialize_ms = elapsedMillis(publish_stage_start);

    publish_stage_start = std::chrono::steady_clock::now();
    modified_map_timed_pub_->publish(timed_msg);
    timed_publish_call_ms = elapsedMillis(publish_stage_start);

    publish_stage_start = std::chrono::steady_clock::now();
    cached_modified_map_timed_msg_ = timed_msg;
    timed_cache_update_ms = elapsedMillis(publish_stage_start);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Published /modified_map_timed points %zu -> %zu leaf=%.3fm",
      assembled_timed_input_point_count_, timed_points_to_publish.size(), timed_leaf_size);
  }
  if (publish_modified_map_ || publish_modified_map_timed_ || publish_map_to_odom_tf_) {
    last_periodic_output_publish_time_ = std::chrono::steady_clock::now();
  }
  const double publish_ms = elapsedMillis(stage_start);

  stage_start = std::chrono::steady_clock::now();
  if (do_save_map) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_to_save = map_ptr;
    if (use_dynamic_object_filter_) {
      DynamicObjectFilterConfig filter_config;
      filter_config.voxel_size = dynamic_object_filter_voxel_size_;
      filter_config.min_observations = dynamic_object_filter_min_observations_;
      filter_config.temporal_window = dynamic_object_filter_temporal_window_;
      filter_config.max_range_from_sensor_m = dynamic_object_filter_max_range_from_sensor_m_;
      const auto filter_result =
        buildDynamicObjectFilteredMap(dynamic_filter_submaps, filter_config);
      if (!filter_result.cloud->empty()) {
        map_to_save = filter_result.cloud;
      }
      RCLCPP_INFO(
        get_logger(),
        "Dynamic object filter: input_points %zu, kept %zu/%zu candidate voxels, "
        "removed %zu, always_keep %zu, output_points %zu",
        filter_result.stats.input_points,
        filter_result.stats.kept_candidate_voxels,
        filter_result.stats.candidate_voxels,
        filter_result.stats.removed_candidate_voxels,
        filter_result.stats.always_keep_voxels,
        filter_result.stats.output_points);
    }
    saveGridDividedMap(map_to_save);
  }
  const double grid_map_save_ms = elapsedMillis(stage_start);

  // Record the fingerprint this rebuild was assembled from and clear the
  // dirty flag -- still under publish_lock (held since function entry), so
  // this is serialized against publishMapAndPose()'s read of the same
  // fields. Cleared here regardless of which thread/reason triggered this
  // call (loop edge, periodic publish, or explicit save): whoever just did
  // a full rebuild made the cache fresh, full stop. This means a loop edge
  // that lands (upsertLoopEdge, under mtx_, not this lock) in the narrow
  // window between this rebuild's snapshot and this line -- and that
  // *replaces* an existing edge rather than appending one -- can have its
  // dirty signal clobbered here; that's an accepted, self-healing edge case
  // (any subsequent change re-sets the flag) rather than something worth a
  // more complex exchange-based handshake, since loop edges are rare and
  // the alternative (only clearing dirty in publishMapAndPose) would force
  // a redundant full rebuild after every single accepted loop edge.
  cached_map_valid_ = true;
  cached_map_submap_count_ = submaps_size;
  cached_map_loop_edge_count_ = loop_edges.size();
  graph_dirty_ = false;

  {
    std::ostringstream diag;
    diag << std::fixed << std::setprecision(3)
         << "{\"event\":\"pose_adjustment_timing\""
         << ",\"num_submaps\":" << submaps_size
         << ",\"num_loop_edges\":" << loop_edges.size()
         << ",\"optimizer_backend\":\"" << optimization::poseGraphBackendName(backend) << "\""
         << ",\"optimizer_iterations\":" << optimization_result.iterations
         << ",\"optimizer_state_rebuilt\":" <<
      (optimization_result.state_rebuilt ? "true" : "false")
         << ",\"optimizer_rebuild_reason\":\"" <<
      optimization_result.state_rebuild_reason << "\""
         << ",\"optimizer_variables_added\":" << optimization_result.variables_added
         << ",\"optimizer_factors_added\":" << optimization_result.factors_added
         << ",\"optimizer_total_variables\":" << optimization_result.total_variables
         << ",\"optimizer_total_factors\":" << optimization_result.total_factors
         << ",\"do_save_map\":" << (do_save_map ? "true" : "false")
         << ",\"assembly_mode\":\"" << (append_only ? "append" : "full") << "\""
         << ",\"submaps_appended\":" << (submaps_size - assembly_start_submap)
         << ",\"graph_build_ms\":" << graph_build_ms
         << ",\"optimize_ms\":" << optimize_ms
         << ",\"graph_save_ms\":" << graph_save_ms
         << ",\"auto_scale_ms\":" << auto_scale_ms
         << ",\"map_assembly_ms\":" << map_assembly_ms
         << ",\"map_to_odom_tf_ms\":" << map_to_odom_tf_ms
         << ",\"map_array_published\":" << (map_array_published ? "true" : "false")
         << ",\"map_array_publish_ms\":" << map_array_publish_ms
         << ",\"path_publish_ms\":" << path_publish_ms
         << ",\"dem_schedule_ms\":" << dem_schedule_ms
         << ",\"regular_voxelize_ms\":" << regular_voxelize_ms
         << ",\"regular_serialize_ms\":" << regular_serialize_ms
         << ",\"regular_publish_call_ms\":" << regular_publish_call_ms
         << ",\"regular_cache_update_ms\":" << regular_cache_update_ms
         << ",\"timed_voxel_update_ms\":" << timed_voxel_update_ms
         << ",\"timed_voxelize_ms\":" << timed_voxelize_ms
         << ",\"timed_serialize_ms\":" << timed_serialize_ms
         << ",\"timed_publish_call_ms\":" << timed_publish_call_ms
         << ",\"timed_cache_update_ms\":" << timed_cache_update_ms
         << ",\"publish_ms\":" << publish_ms
         << ",\"grid_map_save_ms\":" << grid_map_save_ms
         << ",\"total_ms\":" << elapsedMillis(adjustment_start)
         << "}";
    publishBackendTimingDiagnostic(diag.str());
  }
}

}  // namespace graphslam
