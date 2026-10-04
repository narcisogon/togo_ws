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
Implements loop-candidate generation, descriptor database maintenance,
registration verification, geometric guardrails, and accepted loop-edge
insertion for every enabled place-recognition source.
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
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace graphslam
{
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
Returns the stable diagnostic label for a loop-candidate source.
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
Tests whether two SE(3) poses differ by less than translation and rotation
tolerances used for cache and scheduling decisions.
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
Estimates a robust low-percentile ground height from a submap point cloud.
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
Snapshots the graph, advances descriptor databases, and schedules one or more
query submaps according to deterministic or latest-only search mode.
*/
void GraphBasedSlamComponent::searchLoop()
{
  lidarslam_msgs::msg::MapArray map_array_msg;
  LoopEdges loop_edges;
  if (!snapshotGraphState(map_array_msg, loop_edges, true)) {return;}
  if (map_array_msg.submaps.size() < 2) {return;}
  if (map_array_msg.cloud_coordinate != map_array_msg.LOCAL) {
    RCLCPP_WARN(get_logger(), "cloud_coordinate should be local, but it's not local.");
  }
  int num_submaps = map_array_msg.submaps.size();

  if (debug_flag_) {
    RCLCPP_INFO(get_logger(), "searching Loop, num_submaps:%d", num_submaps);
  }

  const auto search_loop_timing_start = std::chrono::steady_clock::now();
  double scan_context_db_ms = 0.0;
  double bev_descriptor_db_ms = 0.0;
  double solid_descriptor_db_ms = 0.0;
  double triangle_descriptor_db_ms = 0.0;
  double loop_query_dispatch_ms = 0.0;
  int queries_processed = 0;

  const auto build_filtered_local_submap =
    [this, &map_array_msg](int ref_idx) -> pcl::PointCloud<pcl::PointXYZI>::Ptr {
      pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
      Eigen::Affine3d reference_affine;
      tf2::fromMsg(map_array_msg.submaps[ref_idx].pose, reference_affine);
      // Symmetric window, matching the registration target-side aggregation
      // below (see the loop candidate registration loop). This used to be
      // trailing-only (k = 0..search_submap_num_-1, ref_idx - k), which is
      // fine for a freshly-added submap (no future submaps exist yet to
      // include either way) but silently starves descriptor building for
      // OLDER submap indices reprocessed later (e.g. multiple submaps
      // arriving in one searchLoop() batch) of geometry that's already in
      // map_array_msg on the other side of ref_idx -- exactly the "source
      // window extends away from the overlap region on a reverse pass"
      // asymmetry. When the newer side genuinely doesn't exist yet, this is
      // a no-op (src_idx >= num_submaps is skipped), so it only ever adds
      // coverage, never removes it.
      const int num_submaps_snapshot = static_cast<int>(map_array_msg.submaps.size());
      for (int offset = -search_submap_num_; offset <= search_submap_num_; ++offset) {
        const int src_idx = ref_idx + offset;
        if (src_idx < 0 || src_idx >= num_submaps_snapshot) {
          continue;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        if (use_pcd_cache_) {
          cloud = loadSubmapFromPCDCached(src_idx);
        } else {
          cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
          pcl::fromROSMsg(map_array_msg.submaps[src_idx].cloud, *cloud);
        }
        if (!cloud || cloud->empty()) {
          continue;
        }
        Eigen::Affine3d src_affine;
        tf2::fromMsg(map_array_msg.submaps[src_idx].pose, src_affine);
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(
          new pcl::PointCloud<pcl::PointXYZI>);
        const Eigen::Matrix4f local_transform =
          (reference_affine.inverse() * src_affine).matrix().cast<float>();
        pcl::transformPointCloud(*cloud, *transformed_cloud, local_transform);
        *aggregated_cloud += *transformed_cloud;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
      if (aggregated_cloud->empty()) {
        return filtered_cloud;
      }
      voxelgrid_.setInputCloud(aggregated_cloud);
      voxelgrid_.filter(*filtered_cloud);
      return filtered_cloud;
    };

  // Keep Scan Context database aligned 1:1 with submap indices.
  if (use_scan_context_ && scan_context_db_.nextSubmapIndex() < num_submaps) {
    const auto stage_start = std::chrono::steady_clock::now();
    for (int idx = scan_context_db_.nextSubmapIndex(); idx < num_submaps; ++idx) {
      const auto filtered_aggregated_cloud = build_filtered_local_submap(idx);
      if (filtered_aggregated_cloud->empty()) {
        scan_context_db_.add(
          idx, ScanContext::Descriptor::Zero(
            ScanContext::NUM_RINGS,
            ScanContext::NUM_SECTORS));
        continue;
      }
      scan_context_db_.add(idx, ScanContext::computeDescriptor(filtered_aggregated_cloud));
    }
    scan_context_db_ms = elapsedMillis(stage_start);
  }

  if (use_bev_descriptor_ && bev_descriptor_db_.nextSubmapIndex() < num_submaps) {
    const auto stage_start = std::chrono::steady_clock::now();
    for (int idx = bev_descriptor_db_.nextSubmapIndex(); idx < num_submaps; ++idx) {
      const auto filtered_aggregated_cloud = build_filtered_local_submap(idx);
      bev_descriptor_db_.add(
        idx,
        SubmapBEVDescriptor::computeDescriptor(
          filtered_aggregated_cloud,
          bev_descriptor_grid_size_m_,
          bev_descriptor_grid_cells_));
    }
    bev_descriptor_db_ms = elapsedMillis(stage_start);
  }
  if (use_solid_descriptor_ && solid_descriptor_db_.nextSubmapIndex() < num_submaps) {
    const auto stage_start = std::chrono::steady_clock::now();
    for (int idx = solid_descriptor_db_.nextSubmapIndex(); idx < num_submaps; ++idx) {
      const auto filtered_aggregated_cloud = build_filtered_local_submap(idx);
      solid_descriptor_db_.add(
        idx,
        SolidDescriptor::computeDescriptor(filtered_aggregated_cloud));
    }
    solid_descriptor_db_ms = elapsedMillis(stage_start);
  }
  if (use_triangle_descriptor_ && triangle_descriptor_next_submap_idx_ < num_submaps) {
    const auto stage_start = std::chrono::steady_clock::now();
    graphslam::triangle::KeypointExtractionConfig kp_cfg;
    if (triangle_descriptor_keypoint_mode_ == "edge_3d") {
      kp_cfg.mode = graphslam::triangle::KeypointMode::EDGE_3D;
    } else if (triangle_descriptor_keypoint_mode_ == "surface_saliency") {
      kp_cfg.mode = graphslam::triangle::KeypointMode::SURFACE_SALIENCY;
    } else {
      kp_cfg.mode = graphslam::triangle::KeypointMode::BEV_MAX_HEIGHT;
    }
    kp_cfg.grid_size_m = triangle_descriptor_grid_size_m_;
    kp_cfg.grid_cells = triangle_descriptor_grid_cells_;
    kp_cfg.min_salience_m = static_cast<float>(triangle_descriptor_min_salience_m_);
    kp_cfg.max_keypoints = triangle_descriptor_max_keypoints_;
    kp_cfg.edge_voxel_size_m = static_cast<float>(triangle_descriptor_edge_voxel_size_m_);
    kp_cfg.edge_neighbor_radius_m =
      static_cast<float>(triangle_descriptor_edge_neighbor_radius_m_);
    kp_cfg.edge_min_neighbors = triangle_descriptor_edge_min_neighbors_;
    kp_cfg.edge_min_edgeness = static_cast<float>(triangle_descriptor_edge_min_edgeness_);
    kp_cfg.edge_nms_radius_m = static_cast<float>(triangle_descriptor_edge_nms_radius_m_);
    kp_cfg.surface_plane_fit_percentile = triangle_descriptor_surface_plane_fit_percentile_;
    kp_cfg.surface_curvature_radius_cells =
      triangle_descriptor_surface_curvature_radius_cells_;
    kp_cfg.surface_min_saliency_percentile =
      triangle_descriptor_surface_min_saliency_percentile_;
    graphslam::triangle::TriangleBuildConfig build_cfg;
    build_cfg.min_edge_m = static_cast<float>(triangle_descriptor_min_edge_m_);
    build_cfg.max_edge_m = static_cast<float>(triangle_descriptor_max_edge_m_);
    build_cfg.max_triangles = triangle_descriptor_max_triangles_;
    graphslam::triangle::HashConfig hash_cfg;
    hash_cfg.edge_bin_m = static_cast<float>(triangle_descriptor_edge_bin_m_);
    hash_cfg.quad_feature_bin_m =
      static_cast<float>(triangle_descriptor_quad_feature_bin_m_);
    for (int idx = triangle_descriptor_next_submap_idx_; idx < num_submaps; ++idx) {
      const auto filtered_aggregated_cloud = build_filtered_local_submap(idx);
      std::vector<graphslam::triangle::Keypoint> kps;
      std::vector<graphslam::triangle::TriangleDescriptor> tris;
      if (filtered_aggregated_cloud && !filtered_aggregated_cloud->empty()) {
        kps = graphslam::triangle::extractKeypoints(*filtered_aggregated_cloud, kp_cfg);
        tris = graphslam::triangle::buildTriangles(kps, build_cfg);
      }
      TrianglePerSubmap entry;
      entry.keypoints = kps;
      entry.triangles = tris;
      triangle_descriptor_per_submap_.push_back(entry);
      triangle_descriptor_db_.addSubmap(idx, kps, tris, hash_cfg);
    }
    triangle_descriptor_next_submap_idx_ = num_submaps;
    triangle_descriptor_db_ms = elapsedMillis(stage_start);
  }

  {
    const auto stage_start = std::chrono::steady_clock::now();
    if (deterministic_loop_scheduling_) {
      // Catch up over every submap not yet used as a loop-search query so the
      // query set is a deterministic function of the map, independent of how the
      // wall-clock timer batched submap arrivals (v0.4 D1 reproducibility fix).
      int query_start = last_searched_submap_idx_ + 1;
      if (query_start < 1) {query_start = 1;}
      int query_end = num_submaps;
      if (deterministic_loop_scheduling_max_queries_per_tick_ > 0) {
        // Bound this call's catch-up work; the remainder of the backlog (if
        // any) is picked up on subsequent ticks. Every submap still
        // eventually gets queried exactly once -- only the worst-case
        // latency of a single searchLoop() call is bounded here, not
        // coverage.
        query_end = std::min(
          num_submaps,
          query_start + deterministic_loop_scheduling_max_queries_per_tick_);
      }
      for (int q = query_start; q < query_end; ++q) {
        searchLoopForLatest(map_array_msg, loop_edges, num_submaps, q);
        queries_processed++;
      }
      last_searched_submap_idx_ = query_end - 1;
    } else {
      // Default (historical) behaviour: query only the single latest submap.
      searchLoopForLatest(map_array_msg, loop_edges, num_submaps, num_submaps - 1);
      queries_processed = 1;
    }
    loop_query_dispatch_ms = elapsedMillis(stage_start);
  }

  {
    std::ostringstream diag;
    diag << std::fixed << std::setprecision(3)
         << "{\"event\":\"search_loop_timing\""
         << ",\"num_submaps\":" << num_submaps
         << ",\"queries_processed\":" << queries_processed
         << ",\"scan_context_db_ms\":" << scan_context_db_ms
         << ",\"bev_descriptor_db_ms\":" << bev_descriptor_db_ms
         << ",\"solid_descriptor_db_ms\":" << solid_descriptor_db_ms
         << ",\"triangle_descriptor_db_ms\":" << triangle_descriptor_db_ms
         << ",\"loop_query_dispatch_ms\":" << loop_query_dispatch_ms
         << ",\"total_ms\":" << elapsedMillis(search_loop_timing_start)
         << "}";
    publishBackendTimingDiagnostic(diag.str());
  }
}

/*
Summary:
Generates candidates for one query submap, verifies them through registration
and source-specific gates, then inserts the best valid loop constraints.
*/
void GraphBasedSlamComponent::searchLoopForLatest(
  const lidarslam_msgs::msg::MapArray & map_array_msg,
  LoopEdges & loop_edges,
  int num_submaps,
  int latest_idx)
{
  const auto loop_search_start = std::chrono::steady_clock::now();
  double scan_context_query_ms = -1.0;
  double best_scan_context_distance = std::numeric_limits<double>::max();
  int best_scan_context_index = -1;

  const auto & latest_submap = map_array_msg.submaps[latest_idx];
  Eigen::Affine3d latest_affine;
  tf2::fromMsg(latest_submap.pose, latest_affine);

  // Aggregate latest N submaps as source (improves matching quality)
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_latest_submap_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_latest_submap_cloud_sc_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_local_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_local_bbs_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  // Symmetric window (was trailing-only: k = 0..search_submap_num_-1,
  // latest_idx - k). In deterministic_loop_scheduling_ mode, latest_idx here
  // can be an older submap index with genuinely newer submaps already
  // present in this map_array_msg snapshot -- a trailing-only source window
  // extends away from the true overlap region on a reverse-direction pass,
  // while the registration target-side aggregation (search_submap_num_
  // offset loop later in this function) already uses a symmetric window.
  // Same no-op-when-absent property as the descriptor-DB-build fix above:
  // when the newer side doesn't exist yet (the common non-deterministic
  // case, latest_idx == num_submaps-1), this only adds coverage, never
  // removes it.
  for (int offset = -search_submap_num_; offset <= search_submap_num_; ++offset) {
    const int src_idx = latest_idx + offset;
    if (src_idx < 0 || src_idx >= num_submaps) {
      continue;
    }
    const int k = std::abs(offset);
    const auto & src_submap = map_array_msg.submaps[src_idx];
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud;
    if (use_pcd_cache_) {
      src_cloud = loadSubmapFromPCDCached(src_idx);
    } else {
      src_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(src_submap.cloud, *src_cloud);
    }
    if (src_cloud->empty()) {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_src(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Affine3d src_affine;
    tf2::fromMsg(src_submap.pose, src_affine);
    pcl::transformPointCloud(*src_cloud, *transformed_src, src_affine.matrix().cast<float>());
    *transformed_latest_submap_cloud_ptr += *transformed_src;
    if (k < three_d_bbs_source_submap_num_) {
      *transformed_latest_submap_cloud_sc_ptr += *transformed_src;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_src_local(
      new pcl::PointCloud<pcl::PointXYZI>);
    const Eigen::Matrix4f latest_frame_transform =
      (latest_affine.inverse() * src_affine).matrix().cast<float>();
    pcl::transformPointCloud(*src_cloud, *transformed_src_local, latest_frame_transform);
    *latest_submap_cloud_local_ptr += *transformed_src_local;
    if (k < three_d_bbs_source_submap_num_) {
      *latest_submap_cloud_local_bbs_ptr += *transformed_src_local;
    }
  }
  if (
    transformed_latest_submap_cloud_ptr->empty() ||
    transformed_latest_submap_cloud_sc_ptr->empty() ||
    latest_submap_cloud_local_ptr->empty() ||
    latest_submap_cloud_local_bbs_ptr->empty())
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source(new pcl::PointCloud<pcl::PointXYZI>);
  voxelgrid_.setInputCloud(transformed_latest_submap_cloud_ptr);
  voxelgrid_.filter(*filtered_source);
  if (filtered_source->empty()) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_sc(new pcl::PointCloud<pcl::PointXYZI>);
  voxelgrid_.setInputCloud(transformed_latest_submap_cloud_sc_ptr);
  voxelgrid_.filter(*filtered_source_sc);
  if (filtered_source_sc->empty()) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_local(new pcl::PointCloud<pcl::PointXYZI>);
  voxelgrid_.setInputCloud(latest_submap_cloud_local_ptr);
  voxelgrid_.filter(*filtered_source_local);
  if (filtered_source_local->empty()) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_local_bbs(
    new pcl::PointCloud<pcl::PointXYZI>);
  voxelgrid_.setInputCloud(latest_submap_cloud_local_bbs_ptr);
  voxelgrid_.filter(*filtered_source_local_bbs);
  if (filtered_source_local_bbs->empty()) {
    return;
  }
  // NOTE: no upfront registration_->setInputSource() here -- the per-candidate
  // loop below always calls it before align() (source cloud depends on
  // candidate.source: filtered_source vs filtered_source_sc), so an earlier
  // call here would just be immediately-discarded NDT preprocessing work.

  const double latest_moving_distance = latest_submap.distance;
  const Eigen::Vector3d latest_submap_pos{
    latest_submap.pose.position.x,
    latest_submap.pose.position.y,
    latest_submap.pose.position.z};

  std::vector<LoopCandidate> candidates;
  auto add_candidate =
    [&candidates](
    int index,
    double selection_metric,
    LoopCandidate::Source source,
    double yaw_rad = 0.0,
    const Eigen::Matrix4f * relative_transform = nullptr,
    int triangle_inliers = -1,
    float triangle_inlier_ratio = -1.0f)
    {
      if (index < 0) {
        return;
      }
      for (auto & candidate : candidates) {
        if (candidate.index != index || candidate.source != source) {
          continue;
        }
        candidate.selection_metric = std::min(candidate.selection_metric, selection_metric);
        candidate.yaw_rad = yaw_rad;
        if (relative_transform != nullptr) {
          candidate.relative_transform = *relative_transform;
          candidate.has_relative_transform = true;
        }
        candidate.triangle_inliers = triangle_inliers;
        candidate.triangle_inlier_ratio = triangle_inlier_ratio;
        return;
      }

      LoopCandidate candidate;
      candidate.index = index;
      candidate.selection_metric = selection_metric;
      candidate.source = source;
      candidate.yaw_rad = yaw_rad;
      if (relative_transform != nullptr) {
        candidate.relative_transform = *relative_transform;
        candidate.has_relative_transform = true;
      }
      candidate.triangle_inliers = triangle_inliers;
      candidate.triangle_inlier_ratio = triangle_inlier_ratio;
      candidates.push_back(candidate);
    };

  struct DescriptorRerankHint
  {
    double score = std::numeric_limits<double>::max();
    double yaw_rad = 0.0;
  };

  std::vector<std::pair<double, int>> distance_candidates;
  distance_candidates.reserve(num_submaps);
  for (int i = 0; i < latest_idx; i++) {
    const auto & submap = map_array_msg.submaps[i];
    const Eigen::Vector3d submap_pos{
      submap.pose.position.x,
      submap.pose.position.y,
      submap.pose.position.z};
    const double dist = (latest_submap_pos - submap_pos).norm();
    if (latest_moving_distance - submap.distance <= distance_loop_closure_) {
      continue;
    }
    if (dist >= range_of_searching_loop_closure_) {
      continue;
    }
    distance_candidates.emplace_back(dist, i);
  }
  std::sort(distance_candidates.begin(), distance_candidates.end());

  if (use_scan_context_ && scan_context_db_.size() > ScanContext::EXCLUDE_RECENT) {
    const auto scan_context_query_start = std::chrono::steady_clock::now();
    const auto sc_matches = scan_context_db_.queryTopMatchesWithYaw(
      scan_context_db_.descriptors[latest_idx],
      max_loop_candidate_count_,
      ScanContext::NUM_CANDIDATES,
      ScanContext::EXCLUDE_RECENT,
      scan_context_threshold_);
    scan_context_query_ms = elapsedMillis(scan_context_query_start);

    if (!sc_matches.empty()) {
      best_scan_context_distance = sc_matches.front().distance;
      best_scan_context_index = sc_matches.front().submap_id;
    }

    if (!sc_matches.empty()) {
      bool added_scan_context_candidate = false;
      for (const auto & sc_match : sc_matches) {
        const int sc_idx = sc_match.submap_id;
        const double sc_dist = sc_match.distance;
        if (sc_idx < 0 || sc_idx >= latest_idx) {
          continue;
        }
        const double sc_travel_distance =
          latest_moving_distance - map_array_msg.submaps[sc_idx].distance;
        if (sc_travel_distance <= distance_loop_closure_) {
          if (debug_flag_) {
            RCLCPP_INFO(
              get_logger(),
              "Skip ScanContext candidate %d because travel distance %.3f m is below %.3f m",
              sc_idx,
              sc_travel_distance,
              distance_loop_closure_);
          }
          continue;
        }
        double sc_yaw_rad =
          -static_cast<double>(sc_match.yaw_shift) * 2.0 * M_PI / ScanContext::NUM_SECTORS;
        while (sc_yaw_rad > M_PI) {
          sc_yaw_rad -= 2.0 * M_PI;
        }
        while (sc_yaw_rad < -M_PI) {
          sc_yaw_rad += 2.0 * M_PI;
        }
        add_candidate(sc_idx, sc_dist, LoopCandidate::Source::SCAN_CONTEXT, sc_yaw_rad);
        {
          std::ostringstream diag;
          diag << std::fixed << std::setprecision(6)
               << "{\"event\":\"scan_context_candidate\""
               << ",\"latest_idx\":" << latest_idx
               << ",\"candidate_idx\":" << sc_idx
               << ",\"sc_dist\":" << sc_dist
               << ",\"sc_threshold\":" << scan_context_threshold_
               << ",\"sc_yaw_deg\":" << sc_yaw_rad * 180.0 / M_PI
               << ",\"travel_distance_m\":" << sc_travel_distance
               << ",\"scan_context_query_ms\":" << scan_context_query_ms
               << "}";
          publishLoopDiagnostic(diag.str());
        }
        std::cout << "ScanContext loop candidate: id=" << sc_idx
                  << " sc_dist=" << sc_dist
                  << " yaw_deg=" << sc_yaw_rad * 180.0 / M_PI << std::endl;
        added_scan_context_candidate = true;
        break;
      }
      if (!added_scan_context_candidate && debug_flag_) {
        std::cout << "ScanContext matches exist but none satisfied travel-distance gating"
                  << std::endl;
      }
      if (!added_scan_context_candidate) {
        std::ostringstream diag;
        diag << std::fixed << std::setprecision(6)
             << "{\"event\":\"scan_context_no_usable_candidate\""
             << ",\"latest_idx\":" << latest_idx
             << ",\"best_candidate_idx\":" << best_scan_context_index
             << ",\"best_sc_dist\":" << best_scan_context_distance
             << ",\"sc_threshold\":" << scan_context_threshold_
             << ",\"reason\":\"travel_distance_or_index_gate\""
             << ",\"scan_context_query_ms\":" << scan_context_query_ms
             << "}";
        publishLoopDiagnostic(diag.str());
      }
    } else if (debug_flag_) {
      auto [sc_idx, sc_dist] = scan_context_db_.query(
        scan_context_db_.descriptors[latest_idx],
        ScanContext::NUM_CANDIDATES,
        ScanContext::EXCLUDE_RECENT,
        std::numeric_limits<double>::max());
      static_cast<void>(sc_idx);
      std::cout << "ScanContext no match: best_sc_dist=" << sc_dist
                << " threshold=" << scan_context_threshold_ << std::endl;
      std::ostringstream diag;
      diag << std::fixed << std::setprecision(6)
           << "{\"event\":\"scan_context_no_match\""
           << ",\"latest_idx\":" << latest_idx
           << ",\"best_candidate_idx\":" << sc_idx
           << ",\"best_sc_dist\":" << sc_dist
           << ",\"sc_threshold\":" << scan_context_threshold_
           << ",\"scan_context_query_ms\":" << scan_context_query_ms
           << "}";
      publishLoopDiagnostic(diag.str());
    } else {
      auto [sc_idx, sc_dist] = scan_context_db_.query(
        scan_context_db_.descriptors[latest_idx],
        ScanContext::NUM_CANDIDATES,
        ScanContext::EXCLUDE_RECENT,
        std::numeric_limits<double>::max());
      std::ostringstream diag;
      diag << std::fixed << std::setprecision(6)
           << "{\"event\":\"scan_context_no_match\""
           << ",\"latest_idx\":" << latest_idx
           << ",\"best_candidate_idx\":" << sc_idx
           << ",\"best_sc_dist\":" << sc_dist
           << ",\"sc_threshold\":" << scan_context_threshold_
           << ",\"scan_context_query_ms\":" << scan_context_query_ms
           << "}";
      publishLoopDiagnostic(diag.str());
    }
  }

  if (use_bev_descriptor_ &&
    bev_descriptor_db_.size() > SubmapBEVDescriptor::DEFAULT_EXCLUDE_RECENT)
  {
    std::unordered_map<int, DescriptorRerankHint> bev_rerank_hints;
    const int bev_rerank_candidates = std::min(
      std::max(max_loop_candidate_count_ * 4, max_loop_candidate_count_),
      static_cast<int>(distance_candidates.size()));
    bool added_bev_candidate = false;
    double best_bev_dist = std::numeric_limits<double>::max();
    int best_bev_idx = -1;
    for (int i = 0; i < bev_rerank_candidates; ++i) {
      const int bev_idx = distance_candidates[i].second;
      if (bev_idx < 0 || bev_idx >= latest_idx || bev_idx >= bev_descriptor_db_.size()) {
        continue;
      }
      const auto & bev_submap = map_array_msg.submaps[bev_idx];
      const Eigen::Vector3d bev_submap_pos(
        bev_submap.pose.position.x,
        bev_submap.pose.position.y,
        bev_submap.pose.position.z);
      const double bev_euclidean_distance = (latest_submap_pos - bev_submap_pos).norm();
      if (
        bev_descriptor_max_euclidean_distance_m_ > 0.0 &&
        bev_euclidean_distance > bev_descriptor_max_euclidean_distance_m_)
      {
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip BEV candidate %d because euclidean distance %.3f m exceeds %.3f m",
            bev_idx,
            bev_euclidean_distance,
            bev_descriptor_max_euclidean_distance_m_);
        }
        continue;
      }

      SubmapBEVDescriptor::Match bev_match;
      if (bev_use_mutual_visibility_) {
        graphslam::bev::MutualVisibilityConfig mv_cfg;
        mv_cfg.min_overlap_ratio = bev_mutual_visibility_min_overlap_ratio_;
        mv_cfg.occupancy_eps =
          static_cast<float>(bev_mutual_visibility_occupancy_eps_);
        const auto fov = graphslam::bev::mutualVisibilityWithYawSearch(
          bev_descriptor_db_.descriptors[latest_idx],
          bev_descriptor_db_.descriptors[bev_idx],
          bev_idx,
          bev_descriptor_yaw_bins_,
          mv_cfg);
        bev_match.submap_id = fov.submap_id;
        bev_match.distance = fov.valid ? fov.distance : 1.0;
        bev_match.yaw_bin = fov.yaw_bin;
        bev_match.yaw_rad = fov.yaw_rad;
      } else {
        bev_match = SubmapBEVDescriptor::distanceWithAlignment(
          bev_descriptor_db_.descriptors[latest_idx],
          bev_descriptor_db_.descriptors[bev_idx],
          bev_idx,
          bev_descriptor_yaw_bins_);
      }
      if (bev_match.distance < best_bev_dist) {
        best_bev_dist = bev_match.distance;
        best_bev_idx = bev_idx;
      }
      if (bev_match.distance >= bev_descriptor_threshold_) {
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip BEV candidate %d because descriptor distance %.3f exceeds %.3f",
            bev_idx,
            bev_match.distance,
            bev_descriptor_threshold_);
        }
        continue;
      }

      double bev_yaw_rad = bev_match.yaw_rad;
      while (bev_yaw_rad > M_PI) {
        bev_yaw_rad -= 2.0 * M_PI;
      }
      while (bev_yaw_rad < -M_PI) {
        bev_yaw_rad += 2.0 * M_PI;
      }
      double bev_sequence_metric = bev_match.distance;
      if (bev_descriptor_sequence_window_ > 0) {
        double bev_sequence_distance_sum = bev_match.distance;
        int bev_sequence_count = 1;
        for (int offset = 1; offset <= bev_descriptor_sequence_window_; ++offset) {
          const int query_idx = latest_idx - offset;
          const int candidate_sequence_idx = bev_idx - offset;
          if (
            query_idx < 0 || candidate_sequence_idx < 0 ||
            query_idx >= bev_descriptor_db_.size() ||
            candidate_sequence_idx >= bev_descriptor_db_.size())
          {
            break;
          }
          const auto rotated_candidate_descriptor = SubmapBEVDescriptor::rotateDescriptor(
            bev_descriptor_db_.descriptors[candidate_sequence_idx],
            bev_yaw_rad);
          double sequence_distance;
          if (bev_use_mutual_visibility_) {
            graphslam::bev::MutualVisibilityConfig mv_cfg;
            mv_cfg.min_overlap_ratio = bev_mutual_visibility_min_overlap_ratio_;
            mv_cfg.occupancy_eps =
              static_cast<float>(bev_mutual_visibility_occupancy_eps_);
            const auto fov = graphslam::bev::mutualVisibilityDistance(
              bev_descriptor_db_.descriptors[query_idx],
              rotated_candidate_descriptor,
              mv_cfg);
            sequence_distance = fov.valid ? fov.distance : 1.0;
          } else {
            sequence_distance = SubmapBEVDescriptor::descriptorDistance(
              bev_descriptor_db_.descriptors[query_idx],
              rotated_candidate_descriptor);
          }
          bev_sequence_distance_sum += sequence_distance;
          ++bev_sequence_count;
        }
        bev_sequence_metric = bev_sequence_distance_sum / static_cast<double>(bev_sequence_count);
      }
      if (bev_sequence_metric >= bev_descriptor_sequence_threshold_) {
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip BEV candidate %d because sequence metric %.3f exceeds %.3f",
            bev_idx,
            bev_sequence_metric,
            bev_descriptor_sequence_threshold_);
        }
        continue;
      }
      double bev_pose_consistency_metric = -1.0;
      if (
        bev_descriptor_pose_consistency_threshold_m_ > 0.0 &&
        bev_descriptor_sequence_window_ > 0)
      {
        Eigen::Affine3d bev_candidate_affine;
        tf2::fromMsg(map_array_msg.submaps[bev_idx].pose, bev_candidate_affine);
        const Eigen::AngleAxisd yaw_correction(bev_yaw_rad, Eigen::Vector3d::UnitZ());
        double bev_pose_consistency_sum = 0.0;
        int bev_pose_consistency_count = 0;
        for (int offset = 1; offset <= bev_descriptor_sequence_window_; ++offset) {
          const int query_idx = latest_idx - offset;
          const int candidate_sequence_idx = bev_idx - offset;
          if (query_idx < 0 || candidate_sequence_idx < 0) {
            break;
          }

          Eigen::Affine3d query_prev_affine;
          Eigen::Affine3d candidate_prev_affine;
          tf2::fromMsg(map_array_msg.submaps[query_idx].pose, query_prev_affine);
          tf2::fromMsg(map_array_msg.submaps[candidate_sequence_idx].pose, candidate_prev_affine);

          const Eigen::Vector3d query_delta =
            (latest_affine.inverse() * query_prev_affine).translation();
          const Eigen::Vector3d candidate_delta =
            yaw_correction * (bev_candidate_affine.inverse() * candidate_prev_affine).translation();
          bev_pose_consistency_sum +=
            (query_delta.head<2>() - candidate_delta.head<2>()).norm();
          ++bev_pose_consistency_count;
        }
        if (bev_pose_consistency_count > 0) {
          bev_pose_consistency_metric =
            bev_pose_consistency_sum / static_cast<double>(bev_pose_consistency_count);
          if (bev_pose_consistency_metric >= bev_descriptor_pose_consistency_threshold_m_) {
            if (debug_flag_) {
              RCLCPP_INFO(
                get_logger(),
                "Skip BEV candidate %d because pose consistency %.3f m exceeds %.3f m",
                bev_idx,
                bev_pose_consistency_metric,
                bev_descriptor_pose_consistency_threshold_m_);
            }
            continue;
          }
        }
      }
      auto & bev_hint = bev_rerank_hints[bev_idx];
      if (bev_sequence_metric < bev_hint.score) {
        bev_hint.score = bev_sequence_metric;
        bev_hint.yaw_rad = bev_yaw_rad;
      }
      std::cout << "BEV rerank hint: id=" << bev_idx
                << " bev_dist=" << bev_match.distance
                << " seq_dist=" << bev_sequence_metric
                << " pose_seq_m=" << bev_pose_consistency_metric
                << " yaw_deg=" << bev_yaw_rad * 180.0 / M_PI << std::endl;
      added_bev_candidate = true;
    }
    if (!added_bev_candidate && debug_flag_) {
      std::cout << "BEV rerank no candidate: best_idx=" << best_bev_idx
                << " best_bev_dist=" << best_bev_dist
                << " threshold=" << bev_descriptor_threshold_ << std::endl;
    }

    auto bev_adjusted_distance =
      [this, &bev_rerank_hints](const std::pair<double, int> & candidate) {
        const auto bev_hint = bev_rerank_hints.find(candidate.second);
        if (bev_hint == bev_rerank_hints.end()) {
          return candidate.first;
        }
        return candidate.first +
               bev_descriptor_rerank_weight_m_ *
               (bev_hint->second.score - bev_descriptor_threshold_);
      };

    std::stable_sort(
      distance_candidates.begin(),
      distance_candidates.end(),
      [&bev_adjusted_distance](const auto & lhs, const auto & rhs) {
        const double lhs_adjusted = bev_adjusted_distance(lhs);
        const double rhs_adjusted = bev_adjusted_distance(rhs);
        if (lhs_adjusted != rhs_adjusted) {
          return lhs_adjusted < rhs_adjusted;
        }
        return lhs.first < rhs.first;
      });

    if (use_distance_loop_candidates_) {
      const int num_distance_candidates =
        std::min(max_loop_candidate_count_, static_cast<int>(distance_candidates.size()));
      for (int i = 0; i < num_distance_candidates; ++i) {
        const int candidate_idx = distance_candidates[i].second;
        const auto bev_hint = bev_rerank_hints.find(candidate_idx);
        const double adjusted_distance = bev_adjusted_distance(distance_candidates[i]);
        if (bev_hint != bev_rerank_hints.end()) {
          add_candidate(
            candidate_idx,
            adjusted_distance,
            LoopCandidate::Source::DISTANCE,
            bev_hint->second.yaw_rad);
          std::cout << "Distance candidate reranked by BEV: id=" << candidate_idx
                    << " dist_m=" << distance_candidates[i].first
                    << " bev_score=" << bev_hint->second.score
                    << " adjusted_dist_m=" << adjusted_distance
                    << " yaw_deg=" << bev_hint->second.yaw_rad * 180.0 / M_PI << std::endl;
        } else {
          add_candidate(
            candidate_idx,
            adjusted_distance,
            LoopCandidate::Source::DISTANCE);
        }
      }
    }
  } else if (use_distance_loop_candidates_) {
    const int num_distance_candidates =
      std::min(max_loop_candidate_count_, static_cast<int>(distance_candidates.size()));
    for (int i = 0; i < num_distance_candidates; i++) {
      add_candidate(
        distance_candidates[i].second,
        distance_candidates[i].first,
        LoopCandidate::Source::DISTANCE);
    }
  }
  if (
    use_solid_descriptor_ &&
    solid_descriptor_db_.size() > SolidDescriptor::DEFAULT_EXCLUDE_RECENT)
  {
    const int solid_rerank_candidates = std::min(
      std::max(max_loop_candidate_count_ * 4, max_loop_candidate_count_),
      static_cast<int>(distance_candidates.size()));
    bool added_solid_candidate = false;
    double best_solid_similarity = -1.0;
    int best_solid_idx = -1;
    for (int i = 0; i < solid_rerank_candidates; ++i) {
      const int solid_idx = distance_candidates[i].second;
      if (solid_idx < 0 || solid_idx >= latest_idx || solid_idx >= solid_descriptor_db_.size()) {
        continue;
      }
      const auto & solid_submap = map_array_msg.submaps[solid_idx];
      const Eigen::Vector3d solid_submap_pos(
        solid_submap.pose.position.x,
        solid_submap.pose.position.y,
        solid_submap.pose.position.z);
      const double solid_euclidean_distance = (latest_submap_pos - solid_submap_pos).norm();
      if (
        solid_descriptor_max_euclidean_distance_m_ > 0.0 &&
        solid_euclidean_distance > solid_descriptor_max_euclidean_distance_m_)
      {
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip SOLiD candidate %d because euclidean distance %.3f m exceeds %.3f m",
            solid_idx,
            solid_euclidean_distance,
            solid_descriptor_max_euclidean_distance_m_);
        }
        continue;
      }

      const double solid_similarity = SolidDescriptor::loopSimilarity(
        solid_descriptor_db_.descriptors[latest_idx],
        solid_descriptor_db_.descriptors[solid_idx]);
      if (solid_similarity > best_solid_similarity) {
        best_solid_similarity = solid_similarity;
        best_solid_idx = solid_idx;
      }
      if (solid_similarity < solid_descriptor_min_similarity_) {
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip SOLiD candidate %d because similarity %.3f is below %.3f",
            solid_idx,
            solid_similarity,
            solid_descriptor_min_similarity_);
        }
        continue;
      }

      double solid_yaw_rad = SolidDescriptor::poseYawRad(
        solid_descriptor_db_.descriptors[latest_idx],
        solid_descriptor_db_.descriptors[solid_idx]);
      while (solid_yaw_rad > M_PI) {
        solid_yaw_rad -= 2.0 * M_PI;
      }
      while (solid_yaw_rad < -M_PI) {
        solid_yaw_rad += 2.0 * M_PI;
      }

      double solid_sequence_similarity = solid_similarity;
      if (solid_descriptor_sequence_window_ > 0) {
        double solid_sequence_similarity_sum = solid_similarity;
        int solid_sequence_count = 1;
        for (int offset = 1; offset <= solid_descriptor_sequence_window_; ++offset) {
          const int query_idx = latest_idx - offset;
          const int candidate_sequence_idx = solid_idx - offset;
          if (
            query_idx < 0 || candidate_sequence_idx < 0 ||
            query_idx >= solid_descriptor_db_.size() ||
            candidate_sequence_idx >= solid_descriptor_db_.size())
          {
            break;
          }
          solid_sequence_similarity_sum += SolidDescriptor::loopSimilarity(
            solid_descriptor_db_.descriptors[query_idx],
            solid_descriptor_db_.descriptors[candidate_sequence_idx]);
          ++solid_sequence_count;
        }
        solid_sequence_similarity =
          solid_sequence_similarity_sum / static_cast<double>(solid_sequence_count);
      }
      if (solid_sequence_similarity < solid_descriptor_sequence_min_similarity_) {
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip SOLiD candidate %d because sequence similarity %.3f is below %.3f",
            solid_idx,
            solid_sequence_similarity,
            solid_descriptor_sequence_min_similarity_);
        }
        continue;
      }

      double solid_pose_consistency_metric = -1.0;
      if (
        solid_descriptor_pose_consistency_threshold_m_ > 0.0 &&
        solid_descriptor_sequence_window_ > 0)
      {
        Eigen::Affine3d solid_candidate_affine;
        tf2::fromMsg(map_array_msg.submaps[solid_idx].pose, solid_candidate_affine);
        const Eigen::AngleAxisd yaw_correction(solid_yaw_rad, Eigen::Vector3d::UnitZ());
        double solid_pose_consistency_sum = 0.0;
        int solid_pose_consistency_count = 0;
        for (int offset = 1; offset <= solid_descriptor_sequence_window_; ++offset) {
          const int query_idx = latest_idx - offset;
          const int candidate_sequence_idx = solid_idx - offset;
          if (query_idx < 0 || candidate_sequence_idx < 0) {
            break;
          }

          Eigen::Affine3d query_prev_affine;
          Eigen::Affine3d candidate_prev_affine;
          tf2::fromMsg(map_array_msg.submaps[query_idx].pose, query_prev_affine);
          tf2::fromMsg(map_array_msg.submaps[candidate_sequence_idx].pose, candidate_prev_affine);

          const Eigen::Vector3d query_delta =
            (latest_affine.inverse() * query_prev_affine).translation();
          const Eigen::Vector3d candidate_delta =
            yaw_correction *
            (solid_candidate_affine.inverse() * candidate_prev_affine).translation();
          solid_pose_consistency_sum +=
            (query_delta.head<2>() - candidate_delta.head<2>()).norm();
          ++solid_pose_consistency_count;
        }
        if (solid_pose_consistency_count > 0) {
          solid_pose_consistency_metric =
            solid_pose_consistency_sum / static_cast<double>(solid_pose_consistency_count);
          if (
            solid_pose_consistency_metric >=
            solid_descriptor_pose_consistency_threshold_m_)
          {
            if (debug_flag_) {
              RCLCPP_INFO(
                get_logger(),
                "Skip SOLiD candidate %d because pose consistency %.3f m exceeds %.3f m",
                solid_idx,
                solid_pose_consistency_metric,
                solid_descriptor_pose_consistency_threshold_m_);
            }
            continue;
          }
        }
      }

      add_candidate(
        solid_idx,
        1.0 - solid_sequence_similarity,
        LoopCandidate::Source::SOLID_DESCRIPTOR,
        solid_yaw_rad);
      std::cout << "SOLiD rerank candidate: id=" << solid_idx
                << " solid_sim=" << solid_similarity
                << " seq_sim=" << solid_sequence_similarity
                << " pose_seq_m=" << solid_pose_consistency_metric
                << " yaw_deg=" << solid_yaw_rad * 180.0 / M_PI << std::endl;
      added_solid_candidate = true;
    }
    if (!added_solid_candidate && debug_flag_) {
      std::cout << "SOLiD rerank no candidate: best_idx=" << best_solid_idx
                << " best_similarity=" << best_solid_similarity
                << " threshold=" << solid_descriptor_min_similarity_ << std::endl;
    }
  }

  if (
    use_triangle_descriptor_ &&
    static_cast<int>(triangle_descriptor_per_submap_.size()) > latest_idx &&
    triangle_descriptor_db_.submapCount() >
    static_cast<std::size_t>(triangle_descriptor_exclude_recent_))
  {
    const auto & query_kps = triangle_descriptor_per_submap_[latest_idx].keypoints;
    const auto & query_tris = triangle_descriptor_per_submap_[latest_idx].triangles;
    if (!query_tris.empty()) {
      graphslam::triangle::HashConfig hash_cfg;
      hash_cfg.edge_bin_m = static_cast<float>(triangle_descriptor_edge_bin_m_);
      hash_cfg.quad_feature_bin_m =
        static_cast<float>(triangle_descriptor_quad_feature_bin_m_);
      graphslam::triangle::VoteConfig vote_cfg;
      vote_cfg.exclude_submap_id = -1;
      graphslam::triangle::VerificationConfig verify_cfg;
      verify_cfg.inlier_translation_m =
        static_cast<float>(triangle_descriptor_inlier_translation_m_);
      verify_cfg.inlier_rotation_deg =
        static_cast<float>(triangle_descriptor_inlier_rotation_deg_);
      verify_cfg.min_inliers = triangle_descriptor_min_inliers_;
      verify_cfg.min_inlier_ratio =
        static_cast<float>(triangle_descriptor_min_inlier_ratio_);
      verify_cfg.max_pairs = triangle_descriptor_max_pairs_;
      verify_cfg.min_4th_point_agreements =
        triangle_descriptor_min_4th_point_agreements_;
      verify_cfg.fourth_point_max_distance_m =
        static_cast<float>(triangle_descriptor_fourth_point_max_distance_m_);
      verify_cfg.refine_se3_with_all_inliers =
        triangle_descriptor_refine_se3_with_all_inliers_;

      // Mask out the latest_idx and any recent submaps so we don't loop on
      // ourselves. We do this by running the vote step first and dropping any
      // candidate whose submap_id is too close to latest_idx.
      const auto votes = graphslam::triangle::accumulateVotes(
        triangle_descriptor_db_, query_kps, query_tris, hash_cfg, vote_cfg);

      // Top-K verify: a permissive keypoint stage (e.g. surface_saliency)
      // generates more hash collisions, so trusting the single top-voted
      // submap lets one stale/aliased submap eat every vote and hand RANSAC
      // a wrong SE(3) that NDT cannot refine. Instead, verify each of the
      // top-K vote-getters in its own single-submap scoped_db and pick the
      // winner by inlier_ratio (ties broken by inlier count). K=1
      // reproduces the legacy top-1-by-votes behaviour exactly.
      struct TriangleAttempt
      {
        int submap_id {-1};
        int votes {0};
        bool verified {false};
        graphslam::triangle::LoopCandidate cand;
      };
      std::vector<TriangleAttempt> attempts;
      const int top_k = std::max(1, triangle_descriptor_verify_top_k_);
      if (!triangle_descriptor_skip_ransac_) {
        vote_cfg.exclude_submap_id = -1;
        for (const auto & v : votes) {
          if (static_cast<int>(attempts.size()) >= top_k) {break;}
          if (v.submap_id < 0) {continue;}
          if (latest_idx - v.submap_id < triangle_descriptor_exclude_recent_) {continue;}
          TriangleAttempt att;
          att.submap_id = v.submap_id;
          att.votes = v.votes;
          if (v.votes >= triangle_descriptor_min_votes_) {
            const auto db_kps_idx = static_cast<std::size_t>(v.submap_id);
            if (db_kps_idx < triangle_descriptor_per_submap_.size()) {
              graphslam::triangle::TriangleDatabase scoped_db;
              scoped_db.addSubmap(
                v.submap_id,
                triangle_descriptor_per_submap_[db_kps_idx].keypoints,
                triangle_descriptor_per_submap_[db_kps_idx].triangles,
                hash_cfg);
              att.cand = graphslam::triangle::findLoopCandidate(
                scoped_db, query_kps, query_tris, hash_cfg, vote_cfg, verify_cfg);
              att.verified = true;
            }
          }
          attempts.push_back(att);
        }
      }

      const TriangleAttempt * best = nullptr;
      for (const auto & att : attempts) {
        if (!att.verified || !att.cand.accepted) {continue;}
        if (
          !best ||
          att.cand.inlier_ratio > best->cand.inlier_ratio ||
          (att.cand.inlier_ratio == best->cand.inlier_ratio &&
          att.cand.inliers > best->cand.inliers))
        {
          best = &att;
        }
      }

      if (best) {
        const int chosen_submap_id = best->submap_id;
        const int chosen_votes = best->votes;
        const auto & cand = best->cand;
        const double travel_distance =
          latest_moving_distance - map_array_msg.submaps[chosen_submap_id].distance;
        bool bev_cross_verify_ok = true;
        double bev_cross_verify_distance = std::numeric_limits<double>::infinity();
        if (
          triangle_verify_with_bev_ &&
          use_bev_descriptor_ &&
          chosen_submap_id < bev_descriptor_db_.size() &&
          !bev_descriptor_db_.descriptors.empty())
        {
          graphslam::bev::MutualVisibilityConfig mv_cfg;
          mv_cfg.min_overlap_ratio = bev_mutual_visibility_min_overlap_ratio_;
          mv_cfg.occupancy_eps =
            static_cast<float>(bev_mutual_visibility_occupancy_eps_);
          const auto fov = graphslam::bev::mutualVisibilityWithYawSearch(
            bev_descriptor_db_.descriptors[latest_idx],
            bev_descriptor_db_.descriptors[chosen_submap_id],
            chosen_submap_id,
            bev_descriptor_yaw_bins_,
            mv_cfg);
          bev_cross_verify_distance = fov.valid ?
            fov.distance : std::numeric_limits<double>::infinity();
          bev_cross_verify_ok =
            fov.valid && fov.distance <= triangle_verify_bev_max_distance_;
        }
        if (travel_distance > distance_loop_closure_ && bev_cross_verify_ok) {
          const Eigen::Matrix3f R = cand.transform.block<3, 3>(0, 0);
          const Eigen::Vector3f euler = R.eulerAngles(2, 1, 0);
          double tri_yaw_rad = static_cast<double>(euler[0]);
          while (tri_yaw_rad > M_PI) {tri_yaw_rad -= 2.0 * M_PI;}
          while (tri_yaw_rad < -M_PI) {tri_yaw_rad += 2.0 * M_PI;}
          const double tri_metric =
            1.0 / (1.0 + static_cast<double>(cand.inliers));
          add_candidate(
            chosen_submap_id,
            tri_metric,
            LoopCandidate::Source::TRIANGLE_DESCRIPTOR,
            tri_yaw_rad,
            &cand.transform,
            cand.inliers,
            cand.inlier_ratio);
          std::cout << "Triangle loop candidate: id=" << chosen_submap_id
                    << " votes=" << chosen_votes
                    << " inliers=" << cand.inliers
                    << " eval_n=" << cand.eval_n
                    << " inlier_ratio="
                    << std::fixed << std::setprecision(3) << cand.inlier_ratio
                    << std::defaultfloat
                    << " yaw_deg=" << tri_yaw_rad * 180.0 / M_PI;
          if (triangle_verify_with_bev_) {
            std::cout << " bev_xv_dist=" << bev_cross_verify_distance;
          }
          std::cout << std::endl;
        } else if (!bev_cross_verify_ok && debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip Triangle candidate %d: BEV cross-verify distance %.3f > %.3f",
            chosen_submap_id, bev_cross_verify_distance,
            triangle_verify_bev_max_distance_);
        } else if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "Skip Triangle candidate %d (travel %.3f m <= %.3f m)",
            chosen_submap_id, travel_distance, distance_loop_closure_);
        }
      } else if (debug_flag_) {
        // Best is null: either no attempt was verified+accepted, or nothing
        // reached verification at all. Report the first verified-but-rejected
        // attempt if there is one (surfaces the ratio gate: an inlier count
        // that beats the absolute min can still fail when min_inlier_ratio
        // is set and eval_n is high), else fall back to the top raw vote.
        const TriangleAttempt * rejected = nullptr;
        for (const auto & att : attempts) {
          if (att.verified) {rejected = &att; break;}
        }
        if (rejected) {
          RCLCPP_INFO(
            get_logger(),
            "Triangle votes for %d (%d votes) rejected: inliers %d/%d "
            "(ratio %.3f) below min_inliers=%d min_inlier_ratio=%.3f",
            rejected->submap_id, rejected->votes, rejected->cand.inliers,
            rejected->cand.eval_n, rejected->cand.inlier_ratio,
            triangle_descriptor_min_inliers_, triangle_descriptor_min_inlier_ratio_);
        } else if (!votes.empty()) {
          RCLCPP_INFO(
            get_logger(),
            "Triangle top vote %d only %d votes (need %d) or excluded",
            votes.front().submap_id, votes.front().votes,
            triangle_descriptor_min_votes_);
        }
      }
    }
  }
  if (candidates.empty()) {
    std::ostringstream diag;
    diag << std::fixed << std::setprecision(6)
         << "{\"event\":\"loop_no_candidates\""
         << ",\"latest_idx\":" << latest_idx
         << ",\"scan_context_query_ms\":" << scan_context_query_ms
         << ",\"loop_search_ms\":" << elapsedMillis(loop_search_start)
         << "}";
    publishLoopDiagnostic(diag.str());
    return;
  }

  LoopCandidateResult best_candidate;
  LoopCandidateResult best_scan_context_candidate;
  LoopCandidateResult best_attempt;
  bool attempted_registration = false;

  for (const auto & candidate : candidates) {
    if (candidate.index < 0 || candidate.index >= latest_idx) {
      continue;
    }

    const auto & candidate_submap = map_array_msg.submaps[candidate.index];
    Eigen::Affine3d candidate_affine;
    tf2::fromMsg(candidate_submap.pose, candidate_affine);
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_clouds_bbs_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
    for (int offset = -search_submap_num_; offset <= search_submap_num_; ++offset) {
      const int near_idx = candidate.index + offset;
      if (near_idx < 0 || near_idx >= num_submaps) {
        continue;
      }
      const auto & near_submap = map_array_msg.submaps[near_idx];
      pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr;
      if (use_pcd_cache_) {
        submap_cloud_ptr = loadSubmapFromPCDCached(near_idx);
      } else {
        submap_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(near_submap.cloud, *submap_cloud_ptr);
      }
      if (submap_cloud_ptr->empty()) {
        continue;
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
      Eigen::Affine3d affine;
      tf2::fromMsg(near_submap.pose, affine);
      pcl::transformPointCloud(
        *submap_cloud_ptr, *transformed_submap_cloud_ptr,
        affine.matrix().cast<float>());
      *submap_clouds_ptr += *transformed_submap_cloud_ptr;
      if (std::abs(offset) <= three_d_bbs_target_submap_radius_) {
        *submap_clouds_bbs_ptr += *transformed_submap_cloud_ptr;
      }
    }
    if (submap_clouds_ptr->empty() || submap_clouds_bbs_ptr->empty()) {
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxelgrid_.setInputCloud(submap_clouds_ptr);
    voxelgrid_.filter(*filtered_clouds_ptr);
    if (filtered_clouds_ptr->empty()) {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_clouds_sc_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
    voxelgrid_.setInputCloud(submap_clouds_bbs_ptr);
    voxelgrid_.filter(*filtered_clouds_sc_ptr);
    if (filtered_clouds_sc_ptr->empty()) {
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    bool used_3d_bbs = false;
    double three_d_bbs_score_percentage = 0.0;
    double three_d_bbs_elapsed_msec = 0.0;
    Eigen::Matrix4f initial_guess =
      (candidate_affine.matrix() * latest_affine.inverse().matrix()).cast<float>();
    if (
      candidate.source == LoopCandidate::Source::TRIANGLE_DESCRIPTOR &&
      candidate.has_relative_transform)
    {
      // Triangle proposes a SE(3) that maps latest-submap-local points to
      // chosen-submap-local points. NDT works in world frame, so chain in
      // the two submap poses to recover the source -> target world guess.
      initial_guess =
        (candidate_affine.matrix() *
        candidate.relative_transform.cast<double>() *
        latest_affine.inverse().matrix()).cast<float>();
    } else if (std::abs(candidate.yaw_rad) > 1e-6) {
      Eigen::Affine3d yaw_correction = Eigen::Affine3d::Identity();
      yaw_correction.rotate(Eigen::AngleAxisd(candidate.yaw_rad, Eigen::Vector3d::UnitZ()));
      initial_guess =
        (candidate_affine.matrix() * yaw_correction.matrix() * latest_affine.inverse().matrix()).
        cast<float>();
    }
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr registration_source;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr registration_target;
    if (candidate.source == LoopCandidate::Source::SCAN_CONTEXT) {
      registration_source = filtered_source_sc;
      registration_target = filtered_clouds_sc_ptr;
    } else {
      registration_source = filtered_source;
      registration_target = filtered_clouds_ptr;
    }
    registration_->setInputSource(registration_source);
    registration_->setInputTarget(registration_target);
    if (candidate.source == LoopCandidate::Source::SCAN_CONTEXT && use_3d_bbs_for_scan_context_) {
      pcl::VoxelGrid<pcl::PointXYZI> three_d_bbs_voxelgrid;
      three_d_bbs_voxelgrid.setLeafSize(
        three_d_bbs_voxel_leaf_size_,
        three_d_bbs_voxel_leaf_size_,
        three_d_bbs_voxel_leaf_size_);
      pcl::PointCloud<pcl::PointXYZI>::Ptr three_d_bbs_source(
        new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr three_d_bbs_target(
        new pcl::PointCloud<pcl::PointXYZI>);
      three_d_bbs_voxelgrid.setInputCloud(filtered_source_local_bbs);
      three_d_bbs_voxelgrid.filter(*three_d_bbs_source);
      three_d_bbs_voxelgrid.setInputCloud(submap_clouds_bbs_ptr);
      three_d_bbs_voxelgrid.filter(*three_d_bbs_target);

      ThreeDBBSLoopVerifierConfig bbs_config;
      bbs_config.min_level_res = three_d_bbs_min_level_res_;
      bbs_config.max_level = three_d_bbs_max_level_;
      bbs_config.score_threshold_percentage = three_d_bbs_score_threshold_percentage_;
      bbs_config.timeout_msec = three_d_bbs_timeout_msec_;
      bbs_config.num_threads = three_d_bbs_num_threads_;
      bbs_config.translation_search_margin_m = three_d_bbs_translation_search_margin_m_;
      bbs_config.roll_pitch_search_deg = three_d_bbs_roll_pitch_search_deg_;
      bbs_config.yaw_search_deg = three_d_bbs_yaw_search_deg_;
      const auto bbs_result = three_d_bbs_loop_verifier_.localize(
        three_d_bbs_source,
        three_d_bbs_target,
        Eigen::Isometry3d(latest_affine.matrix()),
        Eigen::Isometry3d(candidate_affine.matrix()),
        bbs_config);
      if (bbs_result.available) {
        three_d_bbs_score_percentage = bbs_result.score_percentage;
        three_d_bbs_elapsed_msec = bbs_result.elapsed_msec;
        used_3d_bbs = bbs_result.localized;
        if (bbs_result.localized) {
          initial_guess = bbs_result.correction_guess;
        }
        if (debug_flag_) {
          RCLCPP_INFO(
            get_logger(),
            "3D-BBS %s for loop candidate %d -> %d "
            "(score=%.3f elapsed=%.2f ms timed_out=%s "
            "src=%zu tar=%zu)",
            bbs_result.localized ? "localized" : "missed",
            candidate.index,
            latest_idx,
            bbs_result.score_percentage,
            bbs_result.elapsed_msec,
            bbs_result.timed_out ? "true" : "false",
            three_d_bbs_source->size(),
            three_d_bbs_target->size());
        }
      }
    }
    if (loop_z_preshift_enabled_ && !used_3d_bbs &&
      candidate.source != LoopCandidate::Source::DISTANCE)
    {
      // Odometry drift is typically worst in z (the least-observable axis
      // for a ground vehicle), so the pose-composed initial_guess above can
      // carry the full accumulated z error into registration with no
      // correction -- and NDT/GICP's correspondence search only reaches so
      // far, so a bad z guess alone can keep the correct alignment out of
      // reach even when xy/yaw are close. Pre-shift z using robust ground
      // height (10th-percentile z) of source-in-target-frame vs target,
      // clamped so a bad estimate can't inject a wild jump.
      pcl::PointCloud<pcl::PointXYZI> source_in_target_frame;
      pcl::transformPointCloud(*registration_source, source_in_target_frame, initial_guess);
      const float z_shift = robustGroundZ(*registration_target) -
        robustGroundZ(source_in_target_frame);
      const float clamp = static_cast<float>(loop_z_preshift_max_m_);
      initial_guess(2, 3) += std::max(-clamp, std::min(clamp, z_shift));
    }
    const auto registration_start = std::chrono::steady_clock::now();
    if (candidate.source != LoopCandidate::Source::DISTANCE || used_3d_bbs) {
      registration_->align(*output_cloud_ptr, initial_guess);
    } else {
      registration_->align(*output_cloud_ptr);
    }
    const double registration_ms = elapsedMillis(registration_start);
    attempted_registration = true;
    if (!registration_->hasConverged()) {
      std::ostringstream diag;
      diag << std::fixed << std::setprecision(6)
           << "{\"event\":\"loop_candidate_result\""
           << ",\"latest_idx\":" << latest_idx
           << ",\"candidate_idx\":" << candidate.index
           << ",\"source\":\"" << candidate_source_name(candidate.source) << "\""
           << ",\"selection_metric\":" << candidate.selection_metric
           << ",\"registration_converged\":false"
           << ",\"accepted\":false"
           << ",\"reject_reason\":\"registration_not_converged\""
           << ",\"registration_ms\":" << registration_ms
           << ",\"scan_context_query_ms\":" << scan_context_query_ms
           << ",\"loop_search_ms\":" << elapsedMillis(loop_search_start)
           << "}";
      publishLoopDiagnostic(diag.str());
      if (debug_flag_) {
        RCLCPP_INFO(
          get_logger(),
          "Rejected loop candidate %d -> %d because registration did not converge",
          candidate.index,
          latest_idx);
      }
      continue;
    }

    const double fitness_score = registration_->getFitnessScore();
    const Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
    const Eigen::Vector3f translation = final_transformation.block<3, 1>(0, 3);
    const double translation_delta_m = translation.cast<double>().norm();
    const Eigen::Matrix3f rotation = final_transformation.block<3, 3>(0, 0);
    const double trace = static_cast<double>(rotation.trace());
    const double cos_theta = std::max(-1.0, std::min(1.0, 0.5 * (trace - 1.0)));
    const double rotation_delta_deg = std::acos(cos_theta) * 180.0 / M_PI;

    LoopCandidateResult candidate_result;
    candidate_result.index = candidate.index;
    candidate_result.selection_metric = candidate.selection_metric;
    candidate_result.fitness_score = fitness_score;
    candidate_result.travel_distance = latest_moving_distance - candidate_submap.distance;
    const Eigen::Vector3d candidate_submap_pos(
      candidate_submap.pose.position.x,
      candidate_submap.pose.position.y,
      candidate_submap.pose.position.z);
    candidate_result.euclidean_distance = (latest_submap_pos - candidate_submap_pos).norm();
    candidate_result.translation_delta_m = translation_delta_m;
    candidate_result.rotation_delta_deg = rotation_delta_deg;
    candidate_result.source = candidate.source;
    candidate_result.used_3d_bbs = used_3d_bbs;
    candidate_result.three_d_bbs_score_percentage = three_d_bbs_score_percentage;
    candidate_result.three_d_bbs_elapsed_msec = three_d_bbs_elapsed_msec;
    candidate_result.final_transformation = final_transformation;

    if (best_attempt.index < 0 || fitness_score < best_attempt.fitness_score) {
      best_attempt = candidate_result;
    }

    // Triangle inliers are the correctness signal; GICP/NDT fitness is an
    // overlap signal. A candidate only earns the relaxed triangle fitness
    // ceiling when its RANSAC inlier evidence clears the (independent,
    // typically stricter) guardrail below -- weak triangles still face the
    // generic threshold_loop_closure_score_. Fitness keeps flowing into
    // loop_edge_info_weight_ unchanged either way (see doPoseAdjustment),
    // so a thin-overlap accepted loop still ends up a weaker edge.
    const bool triangle_strong_evidence =
      candidate.source == LoopCandidate::Source::TRIANGLE_DESCRIPTOR &&
      triangle_loop_closure_score_threshold_ > 0.0 &&
      (triangle_relaxed_fitness_min_inliers_ < 0 ||
      candidate.triangle_inliers >= triangle_relaxed_fitness_min_inliers_) &&
      (triangle_relaxed_fitness_min_inlier_ratio_ < 0.0 ||
      candidate.triangle_inlier_ratio >= triangle_relaxed_fitness_min_inlier_ratio_);
    const double loop_score_threshold =
      (candidate.source == LoopCandidate::Source::SCAN_CONTEXT &&
      scan_context_loop_closure_score_threshold_ > 0.0) ?
      scan_context_loop_closure_score_threshold_ :
      triangle_strong_evidence ? triangle_loop_closure_score_threshold_ :
      threshold_loop_closure_score_;

    if (fitness_score >= loop_score_threshold) {
      std::ostringstream diag;
      diag << std::fixed << std::setprecision(6)
           << "{\"event\":\"loop_candidate_result\""
           << ",\"latest_idx\":" << latest_idx
           << ",\"candidate_idx\":" << candidate.index
           << ",\"source\":\"" << candidate_source_name(candidate.source) << "\""
           << ",\"selection_metric\":" << candidate.selection_metric
           << ",\"travel_distance_m\":" << candidate_result.travel_distance
           << ",\"euclidean_distance_m\":" << candidate_result.euclidean_distance
           << ",\"registration_converged\":true"
           << ",\"fitness\":" << fitness_score
           << ",\"fitness_threshold\":" << loop_score_threshold
           << ",\"translation_delta_m\":" << translation_delta_m
           << ",\"rotation_delta_deg\":" << rotation_delta_deg
           << ",\"used_3d_bbs\":" << jsonBool(used_3d_bbs)
           << ",\"three_d_bbs_score\":" << three_d_bbs_score_percentage
           << ",\"three_d_bbs_ms\":" << three_d_bbs_elapsed_msec
           << ",\"accepted\":false"
           << ",\"reject_reason\":\"fitness_threshold\""
           << ",\"registration_ms\":" << registration_ms
           << ",\"scan_context_query_ms\":" << scan_context_query_ms
           << ",\"loop_search_ms\":" << elapsedMillis(loop_search_start)
           << "}";
      publishLoopDiagnostic(diag.str());
      if (debug_flag_) {
        RCLCPP_INFO(
          get_logger(),
          "Rejected loop candidate %d -> %d because fitness %.6f exceeds threshold %.6f",
          candidate.index,
          latest_idx,
          fitness_score,
          loop_score_threshold);
      }
      continue;
    }
    // Descriptor-sourced candidates (TRIANGLE / SCAN_CONTEXT / BEV / SOLID)
    // already passed a place-recognition gate, so they can accept a larger
    // NDT correction when the operator opts in. DISTANCE candidates (close
    // in stored pose) keep the strict generic cap.
    const bool is_descriptor_source =
      candidate.source != LoopCandidate::Source::DISTANCE;
    const double effective_translation_cap =
      (is_descriptor_source && loop_max_translation_delta_descriptor_ > 0.0) ?
      loop_max_translation_delta_descriptor_ : loop_max_translation_delta_;
    const double effective_rotation_cap_deg =
      (is_descriptor_source && loop_max_rotation_delta_deg_descriptor_ > 0.0) ?
      loop_max_rotation_delta_deg_descriptor_ : loop_max_rotation_delta_deg_;

    auto publish_candidate_diagnostic =
      [&](bool accepted, const char * reject_reason)
      {
        std::ostringstream diag;
        diag << std::fixed << std::setprecision(6)
             << "{\"event\":\"loop_candidate_result\""
             << ",\"latest_idx\":" << latest_idx
             << ",\"candidate_idx\":" << candidate.index
             << ",\"source\":\"" << candidate_source_name(candidate.source) << "\""
             << ",\"selection_metric\":" << candidate.selection_metric
             << ",\"travel_distance_m\":" << candidate_result.travel_distance
             << ",\"euclidean_distance_m\":" << candidate_result.euclidean_distance
             << ",\"registration_converged\":true"
             << ",\"fitness\":" << fitness_score
             << ",\"fitness_threshold\":" << loop_score_threshold
             << ",\"translation_delta_m\":" << translation_delta_m
             << ",\"translation_cap_m\":" << effective_translation_cap
             << ",\"rotation_delta_deg\":" << rotation_delta_deg
             << ",\"rotation_cap_deg\":" << effective_rotation_cap_deg
             << ",\"used_3d_bbs\":" << jsonBool(used_3d_bbs)
             << ",\"three_d_bbs_score\":" << three_d_bbs_score_percentage
             << ",\"three_d_bbs_ms\":" << three_d_bbs_elapsed_msec
             << ",\"accepted\":" << jsonBool(accepted)
             << ",\"reject_reason\":\"" << reject_reason << "\""
             << ",\"registration_ms\":" << registration_ms
             << ",\"scan_context_query_ms\":" << scan_context_query_ms
             << ",\"loop_search_ms\":" << elapsedMillis(loop_search_start)
             << "}";
        publishLoopDiagnostic(diag.str());
      };

    if (translation_delta_m > effective_translation_cap) {
      publish_candidate_diagnostic(false, "translation_cap");
      if (debug_flag_) {
        RCLCPP_INFO(
          get_logger(),
          "Rejected loop candidate %d -> %d because translation correction %.3f m exceeds %.3f m",
          candidate.index,
          latest_idx,
          translation_delta_m,
          effective_translation_cap);
      }
      continue;
    }
    if (rotation_delta_deg > effective_rotation_cap_deg) {
      publish_candidate_diagnostic(false, "rotation_cap");
      if (debug_flag_) {
        RCLCPP_INFO(
          get_logger(),
          "Rejected loop candidate %d -> %d because rotation correction %.3f deg exceeds %.3f deg",
          candidate.index,
          latest_idx,
          rotation_delta_deg,
          effective_rotation_cap_deg);
      }
      continue;
    }

    candidate_result.valid = true;
    publish_candidate_diagnostic(true, "");
    if (!best_candidate.valid || fitness_score < best_candidate.fitness_score) {
      best_candidate = candidate_result;
    }
    if (
      candidate.source == LoopCandidate::Source::SCAN_CONTEXT &&
      (!best_scan_context_candidate.valid ||
      fitness_score < best_scan_context_candidate.fitness_score))
    {
      best_scan_context_candidate = candidate_result;
    }
  }

  if (prefer_scan_context_candidates_ && best_scan_context_candidate.valid) {
    if (
      !best_candidate.valid ||
      best_candidate.index != best_scan_context_candidate.index ||
      best_candidate.source != LoopCandidate::Source::SCAN_CONTEXT)
    {
      std::cout << "Preferring valid ScanContext candidate id:" <<
        best_scan_context_candidate.index << " over best candidate id:" <<
        (best_candidate.valid ? std::to_string(best_candidate.index) : std::string("none"))
                << std::endl;
    }
    best_candidate = best_scan_context_candidate;
  }

  if (!best_candidate.valid) {
    if (best_attempt.index >= 0) {
      std::ostringstream diag;
      diag << std::fixed << std::setprecision(6)
           << "{\"event\":\"loop_no_valid_candidate\""
           << ",\"latest_idx\":" << latest_idx
           << ",\"best_attempt_idx\":" << best_attempt.index
           << ",\"best_attempt_source\":\"" << candidate_source_name(best_attempt.source) << "\""
           << ",\"best_attempt_fitness\":" << best_attempt.fitness_score
           << ",\"best_attempt_translation_delta_m\":" << best_attempt.translation_delta_m
           << ",\"best_attempt_rotation_delta_deg\":" << best_attempt.rotation_delta_deg
           << ",\"scan_context_query_ms\":" << scan_context_query_ms
           << ",\"loop_search_ms\":" << elapsedMillis(loop_search_start)
           << "}";
      publishLoopDiagnostic(diag.str());
      std::cout << "best_loop_candidate id:" << best_attempt.index
                << " source:" << candidate_source_name(best_attempt.source)
                << " latest_id:" << latest_idx
                << " travel_distance:" << best_attempt.travel_distance
                << " euclidean_distance:" << best_attempt.euclidean_distance
                << " fitness:" << best_attempt.fitness_score
                << " correction_translation:" << best_attempt.translation_delta_m
                << " correction_rotation_deg:" << best_attempt.rotation_delta_deg
                << " used_3d_bbs:" << best_attempt.used_3d_bbs
                << " 3d_bbs_score:" << best_attempt.three_d_bbs_score_percentage
                << std::endl;
    } else if (attempted_registration && debug_flag_) {
      std::ostringstream diag;
      diag << std::fixed << std::setprecision(6)
           << "{\"event\":\"loop_no_valid_candidate\""
           << ",\"latest_idx\":" << latest_idx
           << ",\"best_attempt_idx\":-1"
           << ",\"scan_context_query_ms\":" << scan_context_query_ms
           << ",\"loop_search_ms\":" << elapsedMillis(loop_search_start)
           << "}";
      publishLoopDiagnostic(diag.str());
      RCLCPP_INFO(
        get_logger(), "No converged loop candidate remained for latest submap %d",
        latest_idx);
    }
    return;
  }

  Eigen::Affine3d init_affine;
  tf2::fromMsg(latest_submap.pose, init_affine);
  Eigen::Affine3d submap_affine;
  tf2::fromMsg(map_array_msg.submaps[best_candidate.index].pose, submap_affine);

  LoopEdge loop_edge;
  loop_edge.pair_id = std::pair<int, int>(best_candidate.index, latest_idx);
  Eigen::Isometry3d from = Eigen::Isometry3d(submap_affine.matrix());
  Eigen::Isometry3d to = Eigen::Isometry3d(
    best_candidate.final_transformation.cast<double>() * init_affine.matrix());

  loop_edge.relative_pose = Eigen::Isometry3d(from.inverse() * to);
  loop_edge.fitness_score = best_candidate.fitness_score;
  const bool graph_changed = upsertLoopEdge(loop_edge);
  {
    std::ostringstream diag;
    diag << std::fixed << std::setprecision(6)
         << "{\"event\":\"loop_edge_result\""
         << ",\"latest_idx\":" << latest_idx
         << ",\"candidate_idx\":" << best_candidate.index
         << ",\"source\":\"" << candidate_source_name(best_candidate.source) << "\""
         << ",\"fitness\":" << best_candidate.fitness_score
         << ",\"translation_delta_m\":" << best_candidate.translation_delta_m
         << ",\"rotation_delta_deg\":" << best_candidate.rotation_delta_deg
         << ",\"graph_changed\":" << jsonBool(graph_changed)
         << ",\"used_3d_bbs\":" << jsonBool(best_candidate.used_3d_bbs)
         << ",\"three_d_bbs_score\":" << best_candidate.three_d_bbs_score_percentage
         << ",\"three_d_bbs_ms\":" << best_candidate.three_d_bbs_elapsed_msec
         << ",\"scan_context_query_ms\":" << scan_context_query_ms
         << ",\"loop_search_ms\":" << elapsedMillis(loop_search_start)
         << "}";
    publishLoopDiagnostic(diag.str());
  }

  std::cout << "---" << std::endl;
  std::cout << "PoseAdjustment distance:" << best_candidate.travel_distance
            << ", score:" << best_candidate.fitness_score << std::endl;
  std::cout << "id_loop_point 1:" << best_candidate.index
            << " id_loop_point 2:" << latest_idx << std::endl;
  std::cout << "loop_candidate_source:" << candidate_source_name(best_candidate.source) <<
    std::endl;
  if (best_candidate.used_3d_bbs) {
    std::cout << "3d_bbs_score_percentage:" << best_candidate.three_d_bbs_score_percentage
              << " elapsed_msec:" << best_candidate.three_d_bbs_elapsed_msec << std::endl;
  }
  std::cout << "correction translation[m]:" << best_candidate.translation_delta_m
            << " rotation[deg]:" << best_candidate.rotation_delta_deg << std::endl;
  std::cout << "final transformation:" << std::endl;
  std::cout << best_candidate.final_transformation << std::endl;
  if (!graph_changed) {
    std::cout << "loop edge skipped as redundant or lower quality" << std::endl;
    return;
  }
  snapshotLoopEdges(loop_edges);
  // Optimization runs on the search worker's own cadence, immediately after
  // this searchLoop() pass returns -- see searchWorkerLoop().
  optimize_requested_ = true;
}  // NOLINT(readability/fn_size)

}  // namespace graphslam
