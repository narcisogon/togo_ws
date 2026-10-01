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
Constructs and configures the graph-SLAM ROS 2 component. This file owns
parameter loading, registration selection, ROS interface initialization, worker
startup, and orderly shutdown; runtime work is split across the other
graph_based_slam source files by responsibility.
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
#include "graph_based_slam/loop_edge_robustifier.hpp"
#include "graph_based_slam/pose_graph_optimizer.hpp"
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace graphslam
{
/*
Summary:
Loads backend parameters, creates the selected registration and optimizer
implementations, initializes ROS interfaces, and starts background workers.
*/
GraphBasedSlamComponent::GraphBasedSlamComponent(const rclcpp::NodeOptions & options)
: Node("graph_based_slam", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(this)
{
  RCLCPP_INFO(get_logger(), "initialization start");
  std::string registration_method;
  double voxel_leaf_size;
  double ndt_resolution;
  int ndt_num_threads;

  declare_parameter("registration_method", "NDT");
  get_parameter("registration_method", registration_method);
  declare_parameter("voxel_leaf_size", 0.2);
  get_parameter("voxel_leaf_size", voxel_leaf_size);
  declare_parameter("ndt_resolution", 5.0);
  get_parameter("ndt_resolution", ndt_resolution);
  declare_parameter("ndt_num_threads", 0);
  get_parameter("ndt_num_threads", ndt_num_threads);
  declare_parameter("global_frame_id", std::string("map"));
  get_parameter("global_frame_id", global_frame_id_);
  declare_parameter("loop_detection_period", 1000);
  get_parameter("loop_detection_period", loop_detection_period_);
  declare_parameter("deterministic_loop_scheduling", false);
  get_parameter("deterministic_loop_scheduling", deterministic_loop_scheduling_);
  declare_parameter("deterministic_loop_scheduling_max_queries_per_tick", 0);
  get_parameter(
    "deterministic_loop_scheduling_max_queries_per_tick",
    deterministic_loop_scheduling_max_queries_per_tick_);
  declare_parameter("threshold_loop_closure_score", 1.0);
  get_parameter("threshold_loop_closure_score", threshold_loop_closure_score_);
  declare_parameter("scan_context_loop_closure_score_threshold", -1.0);
  get_parameter(
    "scan_context_loop_closure_score_threshold",
    scan_context_loop_closure_score_threshold_);
  declare_parameter("triangle_loop_closure_score_threshold", -1.0);
  get_parameter(
    "triangle_loop_closure_score_threshold",
    triangle_loop_closure_score_threshold_);
  declare_parameter("triangle_relaxed_fitness_min_inliers", -1);
  get_parameter(
    "triangle_relaxed_fitness_min_inliers",
    triangle_relaxed_fitness_min_inliers_);
  declare_parameter("triangle_relaxed_fitness_min_inlier_ratio", -1.0);
  get_parameter(
    "triangle_relaxed_fitness_min_inlier_ratio",
    triangle_relaxed_fitness_min_inlier_ratio_);
  declare_parameter("distance_loop_closure", 20.0);
  get_parameter("distance_loop_closure", distance_loop_closure_);
  declare_parameter("range_of_searching_loop_closure", 20.0);
  get_parameter("range_of_searching_loop_closure", range_of_searching_loop_closure_);
  declare_parameter("search_submap_num", 3);
  get_parameter("search_submap_num", search_submap_num_);
  declare_parameter("max_loop_candidate_count", 3);
  get_parameter("max_loop_candidate_count", max_loop_candidate_count_);
  declare_parameter("loop_edge_dedup_index_window", 8);
  get_parameter("loop_edge_dedup_index_window", loop_edge_dedup_index_window_);
  declare_parameter("loop_max_translation_delta", 15.0);
  get_parameter("loop_max_translation_delta", loop_max_translation_delta_);
  declare_parameter("loop_max_rotation_delta_deg", 45.0);
  get_parameter("loop_max_rotation_delta_deg", loop_max_rotation_delta_deg_);
  declare_parameter("loop_max_translation_delta_descriptor", -1.0);
  get_parameter("loop_max_translation_delta_descriptor", loop_max_translation_delta_descriptor_);
  declare_parameter("loop_max_rotation_delta_deg_descriptor", -1.0);
  get_parameter("loop_max_rotation_delta_deg_descriptor", loop_max_rotation_delta_deg_descriptor_);
  declare_parameter("loop_z_preshift_enabled", false);
  get_parameter("loop_z_preshift_enabled", loop_z_preshift_enabled_);
  declare_parameter("loop_z_preshift_max_m", 5.0);
  get_parameter("loop_z_preshift_max_m", loop_z_preshift_max_m_);
  declare_parameter("num_adjacent_pose_cnstraints", 5);
  get_parameter("num_adjacent_pose_cnstraints", num_adjacent_pose_cnstraints_);
  declare_parameter("optimizer_backend", "gtsam_isam2");
  get_parameter("optimizer_backend", optimizer_backend_);
  declare_parameter("use_save_map_in_loop", true);
  get_parameter("use_save_map_in_loop", use_save_map_in_loop_);
  declare_parameter("debug_flag", false);
  get_parameter("debug_flag", debug_flag_);
  declare_parameter("use_distance_loop_candidates", true);
  get_parameter("use_distance_loop_candidates", use_distance_loop_candidates_);
  declare_parameter("adjacent_edge_info_weight", 1000.0);
  get_parameter("adjacent_edge_info_weight", adjacent_edge_info_weight_);
  declare_parameter("adjacent_edge_info_auto_scale", false);
  get_parameter("adjacent_edge_info_auto_scale", adjacent_edge_info_auto_scale_);
  declare_parameter("adjacent_edge_info_auto_scale_target_nis", 6.0);
  get_parameter(
    "adjacent_edge_info_auto_scale_target_nis",
    adjacent_edge_info_auto_scale_target_nis_);
  declare_parameter("adjacent_edge_info_auto_scale_ema_alpha", 0.3);
  get_parameter(
    "adjacent_edge_info_auto_scale_ema_alpha",
    adjacent_edge_info_auto_scale_ema_alpha_);
  declare_parameter("adjacent_edge_info_auto_scale_min", 1.0);
  get_parameter("adjacent_edge_info_auto_scale_min", adjacent_edge_info_auto_scale_min_);
  declare_parameter("adjacent_edge_info_auto_scale_max", 1.0e6);
  get_parameter("adjacent_edge_info_auto_scale_max", adjacent_edge_info_auto_scale_max_);
  declare_parameter("adjacent_edge_info_auto_scale_split_trans_rot", false);
  get_parameter(
    "adjacent_edge_info_auto_scale_split_trans_rot",
    adjacent_edge_info_auto_scale_split_trans_rot_);
  declare_parameter("adjacent_edge_info_weight_trans", -1.0);
  get_parameter("adjacent_edge_info_weight_trans", adjacent_edge_info_weight_trans_);
  declare_parameter("adjacent_edge_info_weight_rot", -1.0);
  get_parameter("adjacent_edge_info_weight_rot", adjacent_edge_info_weight_rot_);
  declare_parameter("adjacent_edge_info_auto_scale_target_nis_trans", 3.0);
  get_parameter(
    "adjacent_edge_info_auto_scale_target_nis_trans",
    adjacent_edge_info_auto_scale_target_nis_trans_);
  declare_parameter("adjacent_edge_info_auto_scale_target_nis_rot", 3.0);
  get_parameter(
    "adjacent_edge_info_auto_scale_target_nis_rot",
    adjacent_edge_info_auto_scale_target_nis_rot_);
  declare_parameter("adjacent_edge_info_weight_z_scale", 1.0);
  get_parameter("adjacent_edge_info_weight_z_scale", adjacent_edge_info_weight_z_scale_);
  // Trans/rot weights default to the unified weight when negative (i.e., user
  // has not provided an explicit per-block override). This keeps the split
  // mode safe to enable on existing YAMLs that only set
  // adjacent_edge_info_weight.
  if (adjacent_edge_info_weight_trans_ <= 0.0) {
    adjacent_edge_info_weight_trans_ = adjacent_edge_info_weight_;
  }
  if (adjacent_edge_info_weight_rot_ <= 0.0) {
    adjacent_edge_info_weight_rot_ = adjacent_edge_info_weight_;
  }
  declare_parameter("loop_edge_info_weight", 100.0);
  get_parameter("loop_edge_info_weight", loop_edge_info_weight_);
  declare_parameter("loop_edge_robust_kernel_delta", 1.0);
  get_parameter("loop_edge_robust_kernel_delta", loop_edge_robust_kernel_delta_);
  declare_parameter("loop_edge_robust_kernel_type", std::string("huber"));
  get_parameter("loop_edge_robust_kernel_type", loop_edge_robust_kernel_type_);
  declare_parameter("use_scan_context", false);
  get_parameter("use_scan_context", use_scan_context_);
  declare_parameter("use_bev_descriptor", false);
  get_parameter("use_bev_descriptor", use_bev_descriptor_);
  declare_parameter("use_solid_descriptor", false);
  get_parameter("use_solid_descriptor", use_solid_descriptor_);
  declare_parameter("use_triangle_descriptor", false);
  get_parameter("use_triangle_descriptor", use_triangle_descriptor_);
  declare_parameter("triangle_descriptor_grid_size_m", 60.0);
  get_parameter("triangle_descriptor_grid_size_m", triangle_descriptor_grid_size_m_);
  declare_parameter("triangle_descriptor_grid_cells", 100);
  get_parameter("triangle_descriptor_grid_cells", triangle_descriptor_grid_cells_);
  declare_parameter("triangle_descriptor_max_keypoints", 40);
  get_parameter("triangle_descriptor_max_keypoints", triangle_descriptor_max_keypoints_);
  declare_parameter("triangle_descriptor_min_salience_m", 0.8);
  get_parameter("triangle_descriptor_min_salience_m", triangle_descriptor_min_salience_m_);
  declare_parameter("triangle_descriptor_min_edge_m", 2.0);
  get_parameter("triangle_descriptor_min_edge_m", triangle_descriptor_min_edge_m_);
  declare_parameter("triangle_descriptor_max_edge_m", 50.0);
  get_parameter("triangle_descriptor_max_edge_m", triangle_descriptor_max_edge_m_);
  declare_parameter("triangle_descriptor_max_triangles", 3000);
  get_parameter("triangle_descriptor_max_triangles", triangle_descriptor_max_triangles_);
  declare_parameter("triangle_descriptor_edge_bin_m", 0.5);
  get_parameter("triangle_descriptor_edge_bin_m", triangle_descriptor_edge_bin_m_);
  declare_parameter("triangle_descriptor_quad_feature_bin_m", 0.0);
  get_parameter(
    "triangle_descriptor_quad_feature_bin_m",
    triangle_descriptor_quad_feature_bin_m_);
  declare_parameter<std::string>(
    "triangle_descriptor_keypoint_mode", triangle_descriptor_keypoint_mode_);
  get_parameter("triangle_descriptor_keypoint_mode", triangle_descriptor_keypoint_mode_);
  declare_parameter("triangle_descriptor_edge_voxel_size_m", 0.4);
  get_parameter(
    "triangle_descriptor_edge_voxel_size_m", triangle_descriptor_edge_voxel_size_m_);
  declare_parameter("triangle_descriptor_edge_neighbor_radius_m", 1.0);
  get_parameter(
    "triangle_descriptor_edge_neighbor_radius_m",
    triangle_descriptor_edge_neighbor_radius_m_);
  declare_parameter("triangle_descriptor_edge_min_neighbors", 6);
  get_parameter(
    "triangle_descriptor_edge_min_neighbors", triangle_descriptor_edge_min_neighbors_);
  declare_parameter("triangle_descriptor_edge_min_edgeness", 0.5);
  get_parameter(
    "triangle_descriptor_edge_min_edgeness", triangle_descriptor_edge_min_edgeness_);
  declare_parameter("triangle_descriptor_edge_nms_radius_m", 2.0);
  get_parameter(
    "triangle_descriptor_edge_nms_radius_m", triangle_descriptor_edge_nms_radius_m_);
  declare_parameter("triangle_descriptor_surface_plane_fit_percentile", 0.3);
  get_parameter(
    "triangle_descriptor_surface_plane_fit_percentile",
    triangle_descriptor_surface_plane_fit_percentile_);
  declare_parameter("triangle_descriptor_surface_curvature_radius_cells", 1);
  get_parameter(
    "triangle_descriptor_surface_curvature_radius_cells",
    triangle_descriptor_surface_curvature_radius_cells_);
  declare_parameter("triangle_descriptor_surface_min_saliency_percentile", 0.0);
  get_parameter(
    "triangle_descriptor_surface_min_saliency_percentile",
    triangle_descriptor_surface_min_saliency_percentile_);
  declare_parameter("triangle_descriptor_min_votes", 6);
  get_parameter("triangle_descriptor_min_votes", triangle_descriptor_min_votes_);
  declare_parameter("triangle_descriptor_min_inliers", 4);
  get_parameter("triangle_descriptor_min_inliers", triangle_descriptor_min_inliers_);
  declare_parameter("triangle_descriptor_verify_top_k", 1);
  get_parameter(
    "triangle_descriptor_verify_top_k", triangle_descriptor_verify_top_k_);
  declare_parameter("triangle_descriptor_min_inlier_ratio", 0.0);
  get_parameter(
    "triangle_descriptor_min_inlier_ratio",
    triangle_descriptor_min_inlier_ratio_);
  declare_parameter("triangle_descriptor_max_pairs", 64);
  get_parameter("triangle_descriptor_max_pairs", triangle_descriptor_max_pairs_);
  declare_parameter("triangle_descriptor_min_4th_point_agreements", 0);
  get_parameter(
    "triangle_descriptor_min_4th_point_agreements",
    triangle_descriptor_min_4th_point_agreements_);
  declare_parameter("triangle_descriptor_fourth_point_max_distance_m", 2.0);
  get_parameter(
    "triangle_descriptor_fourth_point_max_distance_m",
    triangle_descriptor_fourth_point_max_distance_m_);
  declare_parameter("triangle_descriptor_refine_se3_with_all_inliers", false);
  get_parameter(
    "triangle_descriptor_refine_se3_with_all_inliers",
    triangle_descriptor_refine_se3_with_all_inliers_);
  declare_parameter("triangle_descriptor_skip_ransac", false);
  get_parameter(
    "triangle_descriptor_skip_ransac",
    triangle_descriptor_skip_ransac_);
  declare_parameter("triangle_descriptor_inlier_translation_m", 2.0);
  get_parameter(
    "triangle_descriptor_inlier_translation_m",
    triangle_descriptor_inlier_translation_m_);
  declare_parameter("triangle_descriptor_inlier_rotation_deg", 5.0);
  get_parameter(
    "triangle_descriptor_inlier_rotation_deg",
    triangle_descriptor_inlier_rotation_deg_);
  declare_parameter("triangle_descriptor_exclude_recent", 4);
  get_parameter("triangle_descriptor_exclude_recent", triangle_descriptor_exclude_recent_);
  declare_parameter("triangle_verify_with_bev", false);
  get_parameter("triangle_verify_with_bev", triangle_verify_with_bev_);
  declare_parameter("triangle_verify_bev_max_distance", 0.30);
  get_parameter("triangle_verify_bev_max_distance", triangle_verify_bev_max_distance_);
  declare_parameter("use_pcd_cache", false);
  get_parameter("use_pcd_cache", use_pcd_cache_);
  declare_parameter("pcd_cache_dir", std::string("/tmp/graph_slam_pcd_cache"));
  get_parameter("pcd_cache_dir", pcd_cache_root_dir_);
  if (use_pcd_cache_) {
    if (!initializePcdCacheSession()) {
      RCLCPP_ERROR(
        get_logger(),
        "Could not initialize PCD cache beneath '%s'; falling back to in-memory submaps",
        pcd_cache_root_dir_.c_str());
      use_pcd_cache_ = false;
    }
  }
  declare_parameter("scan_context_threshold", 0.3);
  get_parameter("scan_context_threshold", scan_context_threshold_);
  declare_parameter("bev_descriptor_threshold", 0.20);
  get_parameter("bev_descriptor_threshold", bev_descriptor_threshold_);
  declare_parameter("bev_descriptor_grid_size_m", 80.0);
  get_parameter("bev_descriptor_grid_size_m", bev_descriptor_grid_size_m_);
  declare_parameter("bev_descriptor_grid_cells", 40);
  get_parameter("bev_descriptor_grid_cells", bev_descriptor_grid_cells_);
  declare_parameter("bev_descriptor_yaw_bins", 24);
  get_parameter("bev_descriptor_yaw_bins", bev_descriptor_yaw_bins_);
  declare_parameter("bev_descriptor_sequence_window", 0);
  get_parameter("bev_descriptor_sequence_window", bev_descriptor_sequence_window_);
  declare_parameter("bev_descriptor_sequence_threshold", -1.0);
  get_parameter("bev_descriptor_sequence_threshold", bev_descriptor_sequence_threshold_);
  declare_parameter("bev_descriptor_pose_consistency_threshold_m", -1.0);
  get_parameter(
    "bev_descriptor_pose_consistency_threshold_m",
    bev_descriptor_pose_consistency_threshold_m_);
  declare_parameter("bev_descriptor_max_euclidean_distance_m", -1.0);
  get_parameter(
    "bev_descriptor_max_euclidean_distance_m",
    bev_descriptor_max_euclidean_distance_m_);
  declare_parameter("bev_descriptor_rerank_weight_m", 100.0);
  get_parameter("bev_descriptor_rerank_weight_m", bev_descriptor_rerank_weight_m_);
  declare_parameter("bev_use_mutual_visibility", false);
  get_parameter("bev_use_mutual_visibility", bev_use_mutual_visibility_);
  declare_parameter("bev_mutual_visibility_min_overlap_ratio", 0.05);
  get_parameter(
    "bev_mutual_visibility_min_overlap_ratio",
    bev_mutual_visibility_min_overlap_ratio_);
  declare_parameter("bev_mutual_visibility_occupancy_eps", 0.5);
  get_parameter(
    "bev_mutual_visibility_occupancy_eps",
    bev_mutual_visibility_occupancy_eps_);
  declare_parameter("solid_descriptor_min_similarity", 0.70);
  get_parameter("solid_descriptor_min_similarity", solid_descriptor_min_similarity_);
  declare_parameter("solid_descriptor_sequence_window", 0);
  get_parameter("solid_descriptor_sequence_window", solid_descriptor_sequence_window_);
  declare_parameter("solid_descriptor_sequence_min_similarity", -1.0);
  get_parameter(
    "solid_descriptor_sequence_min_similarity",
    solid_descriptor_sequence_min_similarity_);
  declare_parameter("solid_descriptor_pose_consistency_threshold_m", -1.0);
  get_parameter(
    "solid_descriptor_pose_consistency_threshold_m",
    solid_descriptor_pose_consistency_threshold_m_);
  declare_parameter("solid_descriptor_max_euclidean_distance_m", -1.0);
  get_parameter(
    "solid_descriptor_max_euclidean_distance_m",
    solid_descriptor_max_euclidean_distance_m_);
  declare_parameter("prefer_scan_context_candidates", false);
  get_parameter("prefer_scan_context_candidates", prefer_scan_context_candidates_);
  declare_parameter("use_3d_bbs_for_scan_context", false);
  get_parameter("use_3d_bbs_for_scan_context", use_3d_bbs_for_scan_context_);
  declare_parameter("three_d_bbs_min_level_res", 1.0);
  get_parameter("three_d_bbs_min_level_res", three_d_bbs_min_level_res_);
  declare_parameter("three_d_bbs_max_level", 3);
  get_parameter("three_d_bbs_max_level", three_d_bbs_max_level_);
  declare_parameter("three_d_bbs_score_threshold_percentage", 0.25);
  get_parameter(
    "three_d_bbs_score_threshold_percentage",
    three_d_bbs_score_threshold_percentage_);
  declare_parameter("three_d_bbs_timeout_msec", 50);
  get_parameter("three_d_bbs_timeout_msec", three_d_bbs_timeout_msec_);
  declare_parameter("three_d_bbs_num_threads", 0);
  get_parameter("three_d_bbs_num_threads", three_d_bbs_num_threads_);
  declare_parameter("three_d_bbs_voxel_leaf_size", 1.0);
  get_parameter("three_d_bbs_voxel_leaf_size", three_d_bbs_voxel_leaf_size_);
  declare_parameter("three_d_bbs_source_submap_num", 2);
  get_parameter("three_d_bbs_source_submap_num", three_d_bbs_source_submap_num_);
  declare_parameter("three_d_bbs_target_submap_radius", 1);
  get_parameter("three_d_bbs_target_submap_radius", three_d_bbs_target_submap_radius_);
  declare_parameter("three_d_bbs_translation_search_margin_m", 15.0);
  get_parameter(
    "three_d_bbs_translation_search_margin_m",
    three_d_bbs_translation_search_margin_m_);
  declare_parameter("three_d_bbs_roll_pitch_search_deg", 10.0);
  get_parameter(
    "three_d_bbs_roll_pitch_search_deg",
    three_d_bbs_roll_pitch_search_deg_);
  declare_parameter("three_d_bbs_yaw_search_deg", 180.0);
  get_parameter("three_d_bbs_yaw_search_deg", three_d_bbs_yaw_search_deg_);
  declare_parameter("use_dynamic_object_filter", false);
  get_parameter("use_dynamic_object_filter", use_dynamic_object_filter_);
  declare_parameter("dynamic_object_filter_voxel_size", 0.3);
  get_parameter("dynamic_object_filter_voxel_size", dynamic_object_filter_voxel_size_);
  declare_parameter("dynamic_object_filter_min_observations", 2);
  get_parameter(
    "dynamic_object_filter_min_observations",
    dynamic_object_filter_min_observations_);
  declare_parameter("dynamic_object_filter_temporal_window", 5);
  get_parameter(
    "dynamic_object_filter_temporal_window",
    dynamic_object_filter_temporal_window_);
  declare_parameter("dynamic_object_filter_max_range_from_sensor_m", 30.0);
  get_parameter(
    "dynamic_object_filter_max_range_from_sensor_m",
    dynamic_object_filter_max_range_from_sensor_m_);
  declare_parameter("map_save_dir", std::string("."));
  get_parameter("map_save_dir", map_save_dir_);
  declare_parameter("map_grid_size_x", 20.0);
  get_parameter("map_grid_size_x", map_grid_size_x_);
  declare_parameter("map_grid_size_y", 20.0);
  get_parameter("map_grid_size_y", map_grid_size_y_);
  declare_parameter("map_leaf_size", 0.2);
  get_parameter("map_leaf_size", map_leaf_size_);
  declare_parameter("dem/enabled", false);
  get_parameter("dem/enabled", dem_enabled_);
  declare_parameter("dem/update_period_sec", 10.0);
  get_parameter("dem/update_period_sec", dem_update_period_sec_);
  declare_parameter("dem/storage_resolution_m", 0.10);
  get_parameter("dem/storage_resolution_m", dem_storage_resolution_m_);
  declare_parameter("dem/preview_resolution_m", 0.50);
  get_parameter("dem/preview_resolution_m", dem_preview_resolution_m_);
  declare_parameter("dem/min_points_per_cell", 3);
  get_parameter("dem/min_points_per_cell", dem_min_points_per_cell_);
  declare_parameter("dem/terrain_percentile", 0.25);
  get_parameter("dem/terrain_percentile", dem_terrain_percentile_);
  declare_parameter("dem/interpolation/max_component_cells", 4);
  get_parameter(
    "dem/interpolation/max_component_cells", dem_interpolation_max_component_cells_);
  declare_parameter("dem/interpolation/search_radius_cells", 2);
  get_parameter(
    "dem/interpolation/search_radius_cells", dem_interpolation_search_radius_cells_);
  declare_parameter("dem/interpolation/min_valid_neighbors", 6);
  get_parameter(
    "dem/interpolation/min_valid_neighbors", dem_interpolation_min_valid_neighbors_);
  declare_parameter("dem/interpolation/max_neighbor_height_range_m", 0.10);
  get_parameter(
    "dem/interpolation/max_neighbor_height_range_m",
    dem_interpolation_max_neighbor_height_range_m_);
  declare_parameter("dem/interpolation/max_plane_residual_m", 0.03);
  get_parameter(
    "dem/interpolation/max_plane_residual_m", dem_interpolation_max_plane_residual_m_);
  declare_parameter("dem/publish_preview", true);
  get_parameter("dem/publish_preview", dem_publish_preview_);
  declare_parameter("dem/preview_max_cells", 250000);
  get_parameter("dem/preview_max_cells", dem_preview_max_cells_);
  declare_parameter("dem/max_raster_cells", 4000000);
  get_parameter("dem/max_raster_cells", dem_max_raster_cells_);
  declare_parameter("dem/output_dir", std::string(""));
  get_parameter("dem/output_dir", dem_output_dir_);
  if (dem_output_dir_.empty()) {
    dem_output_dir_ = map_save_dir_ + "/lunar_dem";
  }
  dem_update_period_sec_ = std::max(0.1, dem_update_period_sec_);
  dem_storage_resolution_m_ = std::max(0.01, dem_storage_resolution_m_);
  dem_preview_resolution_m_ = std::max(dem_storage_resolution_m_, dem_preview_resolution_m_);
  dem_min_points_per_cell_ = std::max(1, dem_min_points_per_cell_);
  dem_preview_max_cells_ = std::max(1, dem_preview_max_cells_);
  dem_max_raster_cells_ = std::max(1000, dem_max_raster_cells_);
  declare_parameter("use_gnss", false);
  get_parameter("use_gnss", use_gnss_);
  declare_parameter("gnss_topic", std::string("/gnss/fix"));
  get_parameter("gnss_topic", gnss_topic_);
  declare_parameter("gnss_info_weight", 1.0);
  get_parameter("gnss_info_weight", gnss_info_weight_);
  declare_parameter("gnss_use_covariance_weighting", true);
  get_parameter("gnss_use_covariance_weighting", gnss_use_covariance_weighting_);
  declare_parameter("gnss_covariance_min_variance_m2", 0.01);
  get_parameter("gnss_covariance_min_variance_m2", gnss_covariance_min_variance_m2_);
  declare_parameter("gnss_covariance_max_variance_m2", 25.0);
  get_parameter("gnss_covariance_max_variance_m2", gnss_covariance_max_variance_m2_);
  declare_parameter("gnss_rtk_fix_max_horizontal_stddev_m", 0.3);
  get_parameter(
    "gnss_rtk_fix_max_horizontal_stddev_m",
    gnss_rtk_fix_max_horizontal_stddev_m_);
  declare_parameter("gnss_rtk_fix_weight_scale", 3.0);
  get_parameter("gnss_rtk_fix_weight_scale", gnss_rtk_fix_weight_scale_);
  declare_parameter("gnss_non_rtk_weight_scale", 1.0);
  get_parameter("gnss_non_rtk_weight_scale", gnss_non_rtk_weight_scale_);
  declare_parameter("gnss_header_stamp_max_skew_sec", 30.0);
  get_parameter("gnss_header_stamp_max_skew_sec", gnss_header_stamp_max_skew_sec_);
  declare_parameter("gnss_origin_min_samples", 3);
  get_parameter("gnss_origin_min_samples", gnss_origin_min_samples_);
  declare_parameter("gnss_origin_consistency_threshold_m", 20.0);
  get_parameter(
    "gnss_origin_consistency_threshold_m",
    gnss_origin_consistency_threshold_m_);
  declare_parameter("gnss_max_horizontal_residual_m", 10.0);
  get_parameter("gnss_max_horizontal_residual_m", gnss_max_horizontal_residual_m_);
  declare_parameter("use_imu_preintegration", false);
  get_parameter("use_imu_preintegration", use_imu_preintegration_);
  declare_parameter("imu_rotation_info_roll_pitch", 100.0);
  get_parameter("imu_rotation_info_roll_pitch", imu_rotation_info_roll_pitch_);
  declare_parameter("imu_rotation_info_yaw", 10.0);
  get_parameter("imu_rotation_info_yaw", imu_rotation_info_yaw_);

  if (gnss_origin_min_samples_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_origin_min_samples must be >= 1, clamping %d to 1",
      gnss_origin_min_samples_);
    gnss_origin_min_samples_ = 1;
  }
  if (gnss_origin_consistency_threshold_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_origin_consistency_threshold_m must be positive, resetting %.3f to 20.0",
      gnss_origin_consistency_threshold_m_);
    gnss_origin_consistency_threshold_m_ = 20.0;
  }
  if (gnss_covariance_min_variance_m2_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_covariance_min_variance_m2 must be positive, resetting %.6f to 0.01",
      gnss_covariance_min_variance_m2_);
    gnss_covariance_min_variance_m2_ = 0.01;
  }
  if (gnss_covariance_max_variance_m2_ < gnss_covariance_min_variance_m2_) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_covariance_max_variance_m2 must be >= min variance, resetting %.6f to %.6f",
      gnss_covariance_max_variance_m2_, gnss_covariance_min_variance_m2_);
    gnss_covariance_max_variance_m2_ = gnss_covariance_min_variance_m2_;
  }
  if (gnss_rtk_fix_max_horizontal_stddev_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_rtk_fix_max_horizontal_stddev_m must be positive, resetting %.3f to 0.3",
      gnss_rtk_fix_max_horizontal_stddev_m_);
    gnss_rtk_fix_max_horizontal_stddev_m_ = 0.3;
  }
  if (gnss_rtk_fix_weight_scale_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_rtk_fix_weight_scale must be positive, resetting %.3f to 3.0",
      gnss_rtk_fix_weight_scale_);
    gnss_rtk_fix_weight_scale_ = 3.0;
  }
  if (gnss_non_rtk_weight_scale_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_non_rtk_weight_scale must be positive, resetting %.3f to 1.0",
      gnss_non_rtk_weight_scale_);
    gnss_non_rtk_weight_scale_ = 1.0;
  }
  if (gnss_header_stamp_max_skew_sec_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "gnss_header_stamp_max_skew_sec must be positive, resetting %.3f to 30.0",
      gnss_header_stamp_max_skew_sec_);
    gnss_header_stamp_max_skew_sec_ = 30.0;
  }
  if (search_submap_num_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "search_submap_num must be >= 1, clamping %d to 1",
      search_submap_num_);
    search_submap_num_ = 1;
  }
  if (max_loop_candidate_count_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "max_loop_candidate_count must be >= 1, clamping %d to 1",
      max_loop_candidate_count_);
    max_loop_candidate_count_ = 1;
  }
  if (loop_edge_dedup_index_window_ < 0) {
    RCLCPP_WARN(
      get_logger(),
      "loop_edge_dedup_index_window must be >= 0, clamping %d to 0",
      loop_edge_dedup_index_window_);
    loop_edge_dedup_index_window_ = 0;
  }
  if (loop_max_translation_delta_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "loop_max_translation_delta must be positive, resetting %.3f to 15.0",
      loop_max_translation_delta_);
    loop_max_translation_delta_ = 15.0;
  }
  if (loop_max_rotation_delta_deg_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "loop_max_rotation_delta_deg must be positive, resetting %.3f to 45.0",
      loop_max_rotation_delta_deg_);
    loop_max_rotation_delta_deg_ = 45.0;
  }
  // Descriptor overrides: -1.0 = disabled (fall back to generic cap).
  // Any other non-positive value is treated as invalid and clamped to -1.0
  // so that operators see clear feedback instead of silently disabling the
  // generic cap for descriptor sources.
  if (loop_max_translation_delta_descriptor_ <= 0.0 &&
    loop_max_translation_delta_descriptor_ != -1.0)
  {
    RCLCPP_WARN(
      get_logger(),
      "loop_max_translation_delta_descriptor must be > 0 (override) or -1 "
      "(disabled); resetting %.3f to -1",
      loop_max_translation_delta_descriptor_);
    loop_max_translation_delta_descriptor_ = -1.0;
  }
  if (loop_max_rotation_delta_deg_descriptor_ <= 0.0 &&
    loop_max_rotation_delta_deg_descriptor_ != -1.0)
  {
    RCLCPP_WARN(
      get_logger(),
      "loop_max_rotation_delta_deg_descriptor must be > 0 (override) or -1 "
      "(disabled); resetting %.3f to -1",
      loop_max_rotation_delta_deg_descriptor_);
    loop_max_rotation_delta_deg_descriptor_ = -1.0;
  }
  if (loop_z_preshift_max_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "loop_z_preshift_max_m must be > 0, resetting %.3f to 5.0",
      loop_z_preshift_max_m_);
    loop_z_preshift_max_m_ = 5.0;
  }
  if (num_adjacent_pose_cnstraints_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "num_adjacent_pose_cnstraints must be >= 1, clamping %d to 1",
      num_adjacent_pose_cnstraints_);
    num_adjacent_pose_cnstraints_ = 1;
  }
  if (!optimization::isPoseGraphBackendName(optimizer_backend_)) {
    RCLCPP_WARN(
      get_logger(),
      "Unknown optimizer_backend '%s'; falling back to gtsam_isam2",
      optimizer_backend_.c_str());
    optimizer_backend_ = "gtsam_isam2";
  } else {
    optimizer_backend_ = optimization::poseGraphBackendName(
      optimization::parsePoseGraphBackend(optimizer_backend_));
  }
  if (adjacent_edge_info_weight_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "adjacent_edge_info_weight must be positive, resetting %.3f to 1000.0",
      adjacent_edge_info_weight_);
    adjacent_edge_info_weight_ = 1000.0;
  }
  if (adjacent_edge_info_weight_trans_ <= 0.0) {
    adjacent_edge_info_weight_trans_ = adjacent_edge_info_weight_;
  }
  if (adjacent_edge_info_weight_rot_ <= 0.0) {
    adjacent_edge_info_weight_rot_ = adjacent_edge_info_weight_;
  }
  if (adjacent_edge_info_auto_scale_target_nis_trans_ <= 0.0) {
    adjacent_edge_info_auto_scale_target_nis_trans_ = 3.0;
  }
  if (adjacent_edge_info_weight_z_scale_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "adjacent_edge_info_weight_z_scale must be positive, resetting %.3f to 1.0",
      adjacent_edge_info_weight_z_scale_);
    adjacent_edge_info_weight_z_scale_ = 1.0;
  }
  if (adjacent_edge_info_auto_scale_target_nis_rot_ <= 0.0) {
    adjacent_edge_info_auto_scale_target_nis_rot_ = 3.0;
  }
  if (loop_edge_info_weight_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "loop_edge_info_weight must be positive, resetting %.3f to 100.0",
      loop_edge_info_weight_);
    loop_edge_info_weight_ = 100.0;
  }
  if (loop_edge_robust_kernel_delta_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "loop_edge_robust_kernel_delta must be positive, resetting %.3f to 1.0",
      loop_edge_robust_kernel_delta_);
    loop_edge_robust_kernel_delta_ = 1.0;
  }
  if (bev_descriptor_threshold_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_threshold must be positive, resetting %.3f to 0.20",
      bev_descriptor_threshold_);
    bev_descriptor_threshold_ = 0.20;
  }
  if (bev_descriptor_grid_size_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_grid_size_m must be positive, resetting %.3f to 80.0",
      bev_descriptor_grid_size_m_);
    bev_descriptor_grid_size_m_ = 80.0;
  }
  if (bev_descriptor_grid_cells_ < 8) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_grid_cells must be >= 8, clamping %d to 8",
      bev_descriptor_grid_cells_);
    bev_descriptor_grid_cells_ = 8;
  }
  if (bev_descriptor_yaw_bins_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_yaw_bins must be >= 1, clamping %d to 1",
      bev_descriptor_yaw_bins_);
    bev_descriptor_yaw_bins_ = 1;
  }
  if (bev_descriptor_sequence_window_ < 0) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_sequence_window must be >= 0, clamping %d to 0",
      bev_descriptor_sequence_window_);
    bev_descriptor_sequence_window_ = 0;
  }
  if (bev_descriptor_sequence_threshold_ <= 0.0) {
    bev_descriptor_sequence_threshold_ = bev_descriptor_threshold_;
  }
  if (bev_descriptor_rerank_weight_m_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_rerank_weight_m must be >= 0.0, clamping %.3f to 0.0",
      bev_descriptor_rerank_weight_m_);
    bev_descriptor_rerank_weight_m_ = 0.0;
  }
  if (bev_descriptor_pose_consistency_threshold_m_ == 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "bev_descriptor_pose_consistency_threshold_m must be negative (disabled) or positive, "
      "resetting 0.0 to disabled");
    bev_descriptor_pose_consistency_threshold_m_ = -1.0;
  }
  if (triangle_descriptor_keypoint_mode_ != "bev_max_height" &&
    triangle_descriptor_keypoint_mode_ != "edge_3d" &&
    triangle_descriptor_keypoint_mode_ != "surface_saliency")
  {
    RCLCPP_WARN(
      get_logger(),
      "triangle_descriptor_keypoint_mode must be 'bev_max_height', 'edge_3d', or "
      "'surface_saliency', got '%s'; falling back to 'bev_max_height'",
      triangle_descriptor_keypoint_mode_.c_str());
    triangle_descriptor_keypoint_mode_ = "bev_max_height";
  }
  if (triangle_descriptor_edge_voxel_size_m_ < 0.0) {
    triangle_descriptor_edge_voxel_size_m_ = 0.0;
  }
  if (triangle_descriptor_edge_neighbor_radius_m_ <= 0.05) {
    triangle_descriptor_edge_neighbor_radius_m_ = 0.05;
  }
  if (triangle_descriptor_edge_min_neighbors_ < 4) {
    triangle_descriptor_edge_min_neighbors_ = 4;
  }
  if (triangle_descriptor_edge_min_edgeness_ < 0.0) {
    triangle_descriptor_edge_min_edgeness_ = 0.0;
  } else if (triangle_descriptor_edge_min_edgeness_ > 1.0) {
    triangle_descriptor_edge_min_edgeness_ = 1.0;
  }
  if (triangle_descriptor_edge_nms_radius_m_ < 0.0) {
    triangle_descriptor_edge_nms_radius_m_ = 0.0;
  }
  if (
    triangle_descriptor_surface_plane_fit_percentile_ < 0.05 ||
    triangle_descriptor_surface_plane_fit_percentile_ > 1.0)
  {
    RCLCPP_WARN(
      get_logger(),
      "triangle_descriptor_surface_plane_fit_percentile must be in [0.05, 1.0]; "
      "clamping %.3f",
      triangle_descriptor_surface_plane_fit_percentile_);
    triangle_descriptor_surface_plane_fit_percentile_ = std::max(
      0.05, std::min(1.0, triangle_descriptor_surface_plane_fit_percentile_));
  }
  if (triangle_descriptor_surface_curvature_radius_cells_ < 1) {
    triangle_descriptor_surface_curvature_radius_cells_ = 1;
  }
  if (
    triangle_descriptor_surface_min_saliency_percentile_ < 0.0 ||
    triangle_descriptor_surface_min_saliency_percentile_ > 1.0)
  {
    RCLCPP_WARN(
      get_logger(),
      "triangle_descriptor_surface_min_saliency_percentile must be in [0, 1]; "
      "clamping %.3f",
      triangle_descriptor_surface_min_saliency_percentile_);
    triangle_descriptor_surface_min_saliency_percentile_ = std::max(
      0.0, std::min(1.0, triangle_descriptor_surface_min_saliency_percentile_));
  }
  if (triangle_descriptor_grid_size_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "triangle_descriptor_grid_size_m must be positive, resetting %.3f to 60.0",
      triangle_descriptor_grid_size_m_);
    triangle_descriptor_grid_size_m_ = 60.0;
  }
  if (triangle_descriptor_grid_cells_ < 8) {
    RCLCPP_WARN(
      get_logger(),
      "triangle_descriptor_grid_cells must be >= 8, clamping %d to 8",
      triangle_descriptor_grid_cells_);
    triangle_descriptor_grid_cells_ = 8;
  }
  if (triangle_descriptor_max_keypoints_ < 4) {
    RCLCPP_WARN(
      get_logger(),
      "triangle_descriptor_max_keypoints must be >= 4, clamping %d to 4",
      triangle_descriptor_max_keypoints_);
    triangle_descriptor_max_keypoints_ = 4;
  }
  if (triangle_descriptor_min_edge_m_ <= 0.0) {
    triangle_descriptor_min_edge_m_ = 2.0;
  }
  if (triangle_descriptor_max_edge_m_ <= triangle_descriptor_min_edge_m_) {
    RCLCPP_WARN(
      get_logger(),
      "triangle_descriptor_max_edge_m (%.3f) must exceed min_edge_m (%.3f); using min*5",
      triangle_descriptor_max_edge_m_, triangle_descriptor_min_edge_m_);
    triangle_descriptor_max_edge_m_ = triangle_descriptor_min_edge_m_ * 5.0;
  }
  if (triangle_descriptor_max_triangles_ < 100) {
    triangle_descriptor_max_triangles_ = 100;
  }
  if (triangle_descriptor_edge_bin_m_ <= 0.0) {
    triangle_descriptor_edge_bin_m_ = 1.0;
  }
  if (triangle_descriptor_quad_feature_bin_m_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "triangle_descriptor_quad_feature_bin_m must be >= 0 (0 = disabled); "
      "resetting %.3f to 0",
      triangle_descriptor_quad_feature_bin_m_);
    triangle_descriptor_quad_feature_bin_m_ = 0.0;
  }
  if (triangle_descriptor_min_votes_ < 1) {
    triangle_descriptor_min_votes_ = 1;
  }
  if (triangle_descriptor_min_inliers_ < 1) {
    triangle_descriptor_min_inliers_ = 1;
  }
  if (triangle_descriptor_verify_top_k_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "triangle_descriptor_verify_top_k must be >= 1, clamping %d to 1",
      triangle_descriptor_verify_top_k_);
    triangle_descriptor_verify_top_k_ = 1;
  }
  if (triangle_descriptor_min_inlier_ratio_ < 0.0) {
    triangle_descriptor_min_inlier_ratio_ = 0.0;
  } else if (triangle_descriptor_min_inlier_ratio_ > 1.0) {
    RCLCPP_WARN(
      get_logger(),
      "triangle_descriptor_min_inlier_ratio must be in [0, 1]; clamping %.3f to 1.0",
      triangle_descriptor_min_inlier_ratio_);
    triangle_descriptor_min_inlier_ratio_ = 1.0;
  }
  if (triangle_descriptor_max_pairs_ < 3) {
    RCLCPP_WARN(
      get_logger(),
      "triangle_descriptor_max_pairs must be >= 3; clamping %d to 3",
      triangle_descriptor_max_pairs_);
    triangle_descriptor_max_pairs_ = 3;
  }
  if (triangle_descriptor_min_4th_point_agreements_ < 0) {
    triangle_descriptor_min_4th_point_agreements_ = 0;
  }
  if (triangle_descriptor_fourth_point_max_distance_m_ <= 0.0) {
    triangle_descriptor_fourth_point_max_distance_m_ = 2.0;
  }
  if (triangle_descriptor_inlier_translation_m_ <= 0.0) {
    triangle_descriptor_inlier_translation_m_ = 2.0;
  }
  if (triangle_descriptor_inlier_rotation_deg_ <= 0.0) {
    triangle_descriptor_inlier_rotation_deg_ = 5.0;
  }
  if (triangle_descriptor_exclude_recent_ < 0) {
    triangle_descriptor_exclude_recent_ = 0;
  }
  if (triangle_verify_bev_max_distance_ <= 0.0) {
    triangle_verify_bev_max_distance_ = 0.30;
  }
  if (
    solid_descriptor_min_similarity_ <= -1.0 ||
    solid_descriptor_min_similarity_ > 1.0)
  {
    RCLCPP_WARN(
      get_logger(),
      "solid_descriptor_min_similarity must be in (-1, 1], resetting %.3f to 0.70",
      solid_descriptor_min_similarity_);
    solid_descriptor_min_similarity_ = 0.70;
  }
  if (solid_descriptor_sequence_window_ < 0) {
    RCLCPP_WARN(
      get_logger(),
      "solid_descriptor_sequence_window must be >= 0, clamping %d to 0",
      solid_descriptor_sequence_window_);
    solid_descriptor_sequence_window_ = 0;
  }
  if (
    solid_descriptor_sequence_min_similarity_ <= -1.0 ||
    solid_descriptor_sequence_min_similarity_ > 1.0)
  {
    solid_descriptor_sequence_min_similarity_ = solid_descriptor_min_similarity_;
  }
  if (solid_descriptor_pose_consistency_threshold_m_ == 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "solid_descriptor_pose_consistency_threshold_m must be negative (disabled) or positive, "
      "resetting 0.0 to disabled");
    solid_descriptor_pose_consistency_threshold_m_ = -1.0;
  }
  if (three_d_bbs_min_level_res_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_min_level_res must be positive, resetting %.3f to 1.0",
      three_d_bbs_min_level_res_);
    three_d_bbs_min_level_res_ = 1.0;
  }
  if (three_d_bbs_max_level_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_max_level must be >= 1, clamping %d to 1",
      three_d_bbs_max_level_);
    three_d_bbs_max_level_ = 1;
  }
  if (three_d_bbs_score_threshold_percentage_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_score_threshold_percentage must be positive, resetting %.3f to 0.25",
      three_d_bbs_score_threshold_percentage_);
    three_d_bbs_score_threshold_percentage_ = 0.25;
  }
  if (three_d_bbs_voxel_leaf_size_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_voxel_leaf_size must be positive, resetting %.3f to 1.0",
      three_d_bbs_voxel_leaf_size_);
    three_d_bbs_voxel_leaf_size_ = 1.0;
  }
  if (three_d_bbs_source_submap_num_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_source_submap_num must be >= 1, clamping %d to 1",
      three_d_bbs_source_submap_num_);
    three_d_bbs_source_submap_num_ = 1;
  }
  if (three_d_bbs_target_submap_radius_ < 0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_target_submap_radius must be >= 0, clamping %d to 0",
      three_d_bbs_target_submap_radius_);
    three_d_bbs_target_submap_radius_ = 0;
  }
  if (three_d_bbs_translation_search_margin_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_translation_search_margin_m must be positive, resetting %.3f to 15.0",
      three_d_bbs_translation_search_margin_m_);
    three_d_bbs_translation_search_margin_m_ = 15.0;
  }
  if (three_d_bbs_roll_pitch_search_deg_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_roll_pitch_search_deg must be positive, resetting %.3f to 10.0",
      three_d_bbs_roll_pitch_search_deg_);
    three_d_bbs_roll_pitch_search_deg_ = 10.0;
  }
  if (three_d_bbs_yaw_search_deg_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "three_d_bbs_yaw_search_deg must be positive, resetting %.3f to 180.0",
      three_d_bbs_yaw_search_deg_);
    three_d_bbs_yaw_search_deg_ = 180.0;
  }
  if (dynamic_object_filter_voxel_size_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "dynamic_object_filter_voxel_size must be positive, resetting %.3f to 0.3",
      dynamic_object_filter_voxel_size_);
    dynamic_object_filter_voxel_size_ = 0.3;
  }
  if (dynamic_object_filter_min_observations_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "dynamic_object_filter_min_observations must be >= 1, clamping %d to 1",
      dynamic_object_filter_min_observations_);
    dynamic_object_filter_min_observations_ = 1;
  }
  if (dynamic_object_filter_temporal_window_ < 0) {
    RCLCPP_WARN(
      get_logger(),
      "dynamic_object_filter_temporal_window must be >= 0, clamping %d to 0",
      dynamic_object_filter_temporal_window_);
    dynamic_object_filter_temporal_window_ = 0;
  }
  if (dynamic_object_filter_max_range_from_sensor_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "dynamic_object_filter_max_range_from_sensor_m must be positive, resetting %.3f to 30.0",
      dynamic_object_filter_max_range_from_sensor_m_);
    dynamic_object_filter_max_range_from_sensor_m_ = 30.0;
  }
  std::cout << "registration_method:" << registration_method << std::endl;
  std::cout << "voxel_leaf_size[m]:" << voxel_leaf_size << std::endl;
  std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
  std::cout << "ndt_num_threads:" << ndt_num_threads << std::endl;
  std::cout << "loop_detection_period[Hz]:" << loop_detection_period_ << std::endl;
  std::cout << "threshold_loop_closure_score:" << threshold_loop_closure_score_ << std::endl;
  std::cout << "scan_context_loop_closure_score_threshold:" <<
    scan_context_loop_closure_score_threshold_ << std::endl;
  std::cout << "triangle_loop_closure_score_threshold:" <<
    triangle_loop_closure_score_threshold_ << std::endl;
  std::cout << "triangle_relaxed_fitness_min_inliers:" <<
    triangle_relaxed_fitness_min_inliers_ << std::endl;
  std::cout << "triangle_relaxed_fitness_min_inlier_ratio:" <<
    triangle_relaxed_fitness_min_inlier_ratio_ << std::endl;
  std::cout << "distance_loop_closure[m]:" << distance_loop_closure_ << std::endl;
  std::cout << "use_distance_loop_candidates:" << std::boolalpha <<
    use_distance_loop_candidates_ << std::endl;
  std::cout << "range_of_searching_loop_closure[m]:" << range_of_searching_loop_closure_ <<
    std::endl;
  std::cout << "search_submap_num:" << search_submap_num_ << std::endl;
  std::cout << "max_loop_candidate_count:" << max_loop_candidate_count_ << std::endl;
  std::cout << "loop_edge_dedup_index_window:" << loop_edge_dedup_index_window_ << std::endl;
  std::cout << "loop_max_translation_delta[m]:" << loop_max_translation_delta_ << std::endl;
  std::cout << "loop_max_rotation_delta[deg]:" << loop_max_rotation_delta_deg_ << std::endl;
  std::cout << "loop_max_translation_delta_descriptor[m]:" <<
    loop_max_translation_delta_descriptor_ << std::endl;
  std::cout << "loop_max_rotation_delta_deg_descriptor[deg]:" <<
    loop_max_rotation_delta_deg_descriptor_ << std::endl;
  std::cout << "loop_z_preshift_enabled:" << std::boolalpha <<
    loop_z_preshift_enabled_ << std::endl;
  std::cout << "loop_z_preshift_max_m:" << loop_z_preshift_max_m_ << std::endl;
  std::cout << "num_adjacent_pose_cnstraints:" << num_adjacent_pose_cnstraints_ << std::endl;
  std::cout << "optimizer_backend:" << optimizer_backend_ << std::endl;
  std::cout << "adjacent_edge_info_weight:" << adjacent_edge_info_weight_ << std::endl;
  std::cout << "adjacent_edge_info_weight_z_scale:" <<
    adjacent_edge_info_weight_z_scale_ << std::endl;
  std::cout << "adjacent_edge_info_auto_scale:" << std::boolalpha
            << adjacent_edge_info_auto_scale_ << std::endl;
  if (adjacent_edge_info_auto_scale_) {
    std::cout << "adjacent_edge_info_auto_scale_split_trans_rot:" << std::boolalpha
              << adjacent_edge_info_auto_scale_split_trans_rot_ << std::endl;
    if (adjacent_edge_info_auto_scale_split_trans_rot_) {
      std::cout << "adjacent_edge_info_weight_trans:"
                << adjacent_edge_info_weight_trans_ << std::endl;
      std::cout << "adjacent_edge_info_weight_rot:"
                << adjacent_edge_info_weight_rot_ << std::endl;
      std::cout << "adjacent_edge_info_auto_scale_target_nis_trans:"
                << adjacent_edge_info_auto_scale_target_nis_trans_ << std::endl;
      std::cout << "adjacent_edge_info_auto_scale_target_nis_rot:"
                << adjacent_edge_info_auto_scale_target_nis_rot_ << std::endl;
    } else {
      std::cout << "adjacent_edge_info_auto_scale_target_nis:"
                << adjacent_edge_info_auto_scale_target_nis_ << std::endl;
    }
  }
  std::cout << "loop_edge_info_weight:" << loop_edge_info_weight_ << std::endl;
  std::cout << "loop_edge_robust_kernel_delta:" << loop_edge_robust_kernel_delta_ << std::endl;
  std::cout << "loop_edge_robust_kernel_type:"
            << graphslam::robust::loopEdgeKernelTypeName(
    graphslam::robust::parseLoopEdgeKernelType(loop_edge_robust_kernel_type_))
            << std::endl;
  std::cout << "use_save_map_in_loop:" << std::boolalpha << use_save_map_in_loop_ << std::endl;
  std::cout << "debug_flag:" << std::boolalpha << debug_flag_ << std::endl;
  std::cout << "use_scan_context:" << std::boolalpha << use_scan_context_ << std::endl;
  if (use_scan_context_) {
    std::cout << "scan_context_threshold:" << scan_context_threshold_ << std::endl;
    std::cout << "prefer_scan_context_candidates:" << std::boolalpha <<
      prefer_scan_context_candidates_ << std::endl;
    std::cout << "use_3d_bbs_for_scan_context:" << std::boolalpha <<
      use_3d_bbs_for_scan_context_ << std::endl;
    if (use_3d_bbs_for_scan_context_) {
      std::cout << "three_d_bbs_min_level_res:" << three_d_bbs_min_level_res_ << std::endl;
      std::cout << "three_d_bbs_max_level:" << three_d_bbs_max_level_ << std::endl;
      std::cout << "three_d_bbs_score_threshold_percentage:" <<
        three_d_bbs_score_threshold_percentage_ << std::endl;
      std::cout << "three_d_bbs_timeout_msec:" << three_d_bbs_timeout_msec_ << std::endl;
      std::cout << "three_d_bbs_num_threads:" << three_d_bbs_num_threads_ << std::endl;
      std::cout << "three_d_bbs_voxel_leaf_size:" << three_d_bbs_voxel_leaf_size_ << std::endl;
      std::cout << "three_d_bbs_source_submap_num:" << three_d_bbs_source_submap_num_ <<
        std::endl;
      std::cout << "three_d_bbs_target_submap_radius:" << three_d_bbs_target_submap_radius_ <<
        std::endl;
      std::cout << "three_d_bbs_translation_search_margin_m:" <<
        three_d_bbs_translation_search_margin_m_ << std::endl;
      std::cout << "three_d_bbs_roll_pitch_search_deg:" <<
        three_d_bbs_roll_pitch_search_deg_ << std::endl;
      std::cout << "three_d_bbs_yaw_search_deg:" << three_d_bbs_yaw_search_deg_ << std::endl;
    }
  }
  std::cout << "use_bev_descriptor:" << std::boolalpha << use_bev_descriptor_ << std::endl;
  if (use_bev_descriptor_) {
    std::cout << "bev_descriptor_threshold:" << bev_descriptor_threshold_ << std::endl;
    std::cout << "bev_descriptor_grid_size_m:" << bev_descriptor_grid_size_m_ << std::endl;
    std::cout << "bev_descriptor_grid_cells:" << bev_descriptor_grid_cells_ << std::endl;
    std::cout << "bev_descriptor_yaw_bins:" << bev_descriptor_yaw_bins_ << std::endl;
    std::cout << "bev_descriptor_sequence_window:" << bev_descriptor_sequence_window_ <<
      std::endl;
    std::cout << "bev_descriptor_sequence_threshold:" << bev_descriptor_sequence_threshold_ <<
      std::endl;
    std::cout << "bev_descriptor_pose_consistency_threshold_m:" <<
      bev_descriptor_pose_consistency_threshold_m_ << std::endl;
    std::cout << "bev_descriptor_max_euclidean_distance_m:" <<
      bev_descriptor_max_euclidean_distance_m_ << std::endl;
    std::cout << "bev_descriptor_rerank_weight_m:" << bev_descriptor_rerank_weight_m_ <<
      std::endl;
    std::cout << "bev_use_mutual_visibility:" << std::boolalpha <<
      bev_use_mutual_visibility_ << std::endl;
    if (bev_use_mutual_visibility_) {
      std::cout << "bev_mutual_visibility_min_overlap_ratio:" <<
        bev_mutual_visibility_min_overlap_ratio_ << std::endl;
      std::cout << "bev_mutual_visibility_occupancy_eps:" <<
        bev_mutual_visibility_occupancy_eps_ << std::endl;
    }
  }
  std::cout << "use_solid_descriptor:" << std::boolalpha << use_solid_descriptor_ << std::endl;
  if (use_solid_descriptor_) {
    std::cout << "solid_descriptor_min_similarity:" << solid_descriptor_min_similarity_ <<
      std::endl;
    std::cout << "solid_descriptor_sequence_window:" << solid_descriptor_sequence_window_ <<
      std::endl;
    std::cout << "solid_descriptor_sequence_min_similarity:" <<
      solid_descriptor_sequence_min_similarity_ << std::endl;
    std::cout << "solid_descriptor_pose_consistency_threshold_m:" <<
      solid_descriptor_pose_consistency_threshold_m_ << std::endl;
    std::cout << "solid_descriptor_max_euclidean_distance_m:" <<
      solid_descriptor_max_euclidean_distance_m_ << std::endl;
  }
  std::cout << "use_triangle_descriptor:" << std::boolalpha <<
    use_triangle_descriptor_ << std::endl;
  if (use_triangle_descriptor_) {
    std::cout << "triangle_descriptor_keypoint_mode:" <<
      triangle_descriptor_keypoint_mode_ << std::endl;
    std::cout << "triangle_descriptor_grid_size_m:" <<
      triangle_descriptor_grid_size_m_ << std::endl;
    std::cout << "triangle_descriptor_grid_cells:" <<
      triangle_descriptor_grid_cells_ << std::endl;
    std::cout << "triangle_descriptor_max_keypoints:" <<
      triangle_descriptor_max_keypoints_ << std::endl;
    std::cout << "triangle_descriptor_min_salience_m:" <<
      triangle_descriptor_min_salience_m_ << std::endl;
    std::cout << "triangle_descriptor_edge_voxel_size_m:" <<
      triangle_descriptor_edge_voxel_size_m_ << std::endl;
    std::cout << "triangle_descriptor_edge_neighbor_radius_m:" <<
      triangle_descriptor_edge_neighbor_radius_m_ << std::endl;
    std::cout << "triangle_descriptor_edge_min_neighbors:" <<
      triangle_descriptor_edge_min_neighbors_ << std::endl;
    std::cout << "triangle_descriptor_edge_min_edgeness:" <<
      triangle_descriptor_edge_min_edgeness_ << std::endl;
    std::cout << "triangle_descriptor_edge_nms_radius_m:" <<
      triangle_descriptor_edge_nms_radius_m_ << std::endl;
    std::cout << "triangle_descriptor_surface_plane_fit_percentile:" <<
      triangle_descriptor_surface_plane_fit_percentile_ << std::endl;
    std::cout << "triangle_descriptor_surface_curvature_radius_cells:" <<
      triangle_descriptor_surface_curvature_radius_cells_ << std::endl;
    std::cout << "triangle_descriptor_surface_min_saliency_percentile:" <<
      triangle_descriptor_surface_min_saliency_percentile_ << std::endl;
    std::cout << "triangle_descriptor_min_edge_m:" <<
      triangle_descriptor_min_edge_m_ << std::endl;
    std::cout << "triangle_descriptor_max_edge_m:" <<
      triangle_descriptor_max_edge_m_ << std::endl;
    std::cout << "triangle_descriptor_max_triangles:" <<
      triangle_descriptor_max_triangles_ << std::endl;
    std::cout << "triangle_descriptor_edge_bin_m:" <<
      triangle_descriptor_edge_bin_m_ << std::endl;
    std::cout << "triangle_descriptor_quad_feature_bin_m:" <<
      triangle_descriptor_quad_feature_bin_m_ << std::endl;
    std::cout << "triangle_descriptor_min_votes:" <<
      triangle_descriptor_min_votes_ << std::endl;
    std::cout << "triangle_descriptor_min_inliers:" <<
      triangle_descriptor_min_inliers_ << std::endl;
    std::cout << "triangle_descriptor_verify_top_k:" <<
      triangle_descriptor_verify_top_k_ << std::endl;
    std::cout << "triangle_descriptor_min_inlier_ratio:" <<
      triangle_descriptor_min_inlier_ratio_ << std::endl;
    std::cout << "triangle_descriptor_max_pairs:" <<
      triangle_descriptor_max_pairs_ << std::endl;
    std::cout << "triangle_descriptor_min_4th_point_agreements:" <<
      triangle_descriptor_min_4th_point_agreements_ << std::endl;
    std::cout << "triangle_descriptor_fourth_point_max_distance_m:" <<
      triangle_descriptor_fourth_point_max_distance_m_ << std::endl;
    std::cout << "triangle_descriptor_refine_se3_with_all_inliers:" <<
      std::boolalpha << triangle_descriptor_refine_se3_with_all_inliers_ << std::endl;
    std::cout << "triangle_descriptor_skip_ransac:" <<
      std::boolalpha << triangle_descriptor_skip_ransac_ << std::endl;
    std::cout << "triangle_descriptor_inlier_translation_m:" <<
      triangle_descriptor_inlier_translation_m_ << std::endl;
    std::cout << "triangle_descriptor_inlier_rotation_deg:" <<
      triangle_descriptor_inlier_rotation_deg_ << std::endl;
    std::cout << "triangle_descriptor_exclude_recent:" <<
      triangle_descriptor_exclude_recent_ << std::endl;
  }
  std::cout << "use_dynamic_object_filter:" << std::boolalpha << use_dynamic_object_filter_ <<
    std::endl;
  if (use_dynamic_object_filter_) {
    std::cout << "dynamic_object_filter_voxel_size:" << dynamic_object_filter_voxel_size_ <<
      std::endl;
    std::cout << "dynamic_object_filter_min_observations:" <<
      dynamic_object_filter_min_observations_ << std::endl;
    std::cout << "dynamic_object_filter_temporal_window:" <<
      dynamic_object_filter_temporal_window_ << std::endl;
    std::cout << "dynamic_object_filter_max_range_from_sensor_m:" <<
      dynamic_object_filter_max_range_from_sensor_m_ << std::endl;
  }
  declare_parameter("use_odom_input", false);
  get_parameter("use_odom_input", use_odom_input_);
  declare_parameter("submap_distance_threshold", 1.5);
  get_parameter("submap_distance_threshold", submap_distance_threshold_);
  declare_parameter("publish_map_to_odom_tf", false);
  get_parameter("publish_map_to_odom_tf", publish_map_to_odom_tf_);
  declare_parameter("map_to_odom_tf_future_offset_sec", 0.0);
  get_parameter("map_to_odom_tf_future_offset_sec", map_to_odom_tf_future_offset_sec_);
  declare_parameter("modified_map_publish_period_sec", 0.0);
  get_parameter("modified_map_publish_period_sec", modified_map_publish_period_sec_);
  declare_parameter("publish_modified_map", true);
  get_parameter("publish_modified_map", publish_modified_map_);
  declare_parameter("publish_modified_map_array_on_append", true);
  get_parameter(
    "publish_modified_map_array_on_append",
    publish_modified_map_array_on_append_);
  declare_parameter("modified_map_leaf_size", 0.0);
  get_parameter("modified_map_leaf_size", modified_map_leaf_size_);
  declare_parameter("publish_modified_map_timed", true);
  get_parameter("publish_modified_map_timed", publish_modified_map_timed_);
  declare_parameter("modified_map_timed_leaf_size", -1.0);
  get_parameter("modified_map_timed_leaf_size", modified_map_timed_leaf_size_);
  declare_parameter("submap_assembly_reuse_translation_eps_m", 1.0e-3);
  get_parameter(
    "submap_assembly_reuse_translation_eps_m",
    submap_assembly_reuse_translation_eps_m_);
  declare_parameter("submap_assembly_reuse_rotation_eps_deg", 0.05);
  get_parameter(
    "submap_assembly_reuse_rotation_eps_deg",
    submap_assembly_reuse_rotation_eps_deg_);
  declare_parameter("odom_input_cloud_in_odom_frame", false);
  get_parameter("odom_input_cloud_in_odom_frame", odom_input_cloud_in_odom_frame_);
  declare_parameter("odom_frame_id", std::string("odom"));
  get_parameter("odom_frame_id", odom_frame_id_);
  std::cout << "use_odom_input:" << std::boolalpha << use_odom_input_ << std::endl;
  if (use_odom_input_) {
    std::cout << "submap_distance_threshold[m]:" << submap_distance_threshold_ << std::endl;
    std::cout << "publish_map_to_odom_tf:" << std::boolalpha << publish_map_to_odom_tf_ <<
      std::endl;
    std::cout << "map_to_odom_tf_future_offset_sec:" <<
      map_to_odom_tf_future_offset_sec_ << std::endl;
    std::cout << "modified_map_publish_period_sec:" <<
      modified_map_publish_period_sec_ << std::endl;
    std::cout << "publish_modified_map:" << std::boolalpha <<
      publish_modified_map_ << std::endl;
    std::cout << "publish_modified_map_array_on_append:" << std::boolalpha <<
      publish_modified_map_array_on_append_ << std::endl;
    std::cout << "modified_map_leaf_size:" << modified_map_leaf_size_ << std::endl;
    std::cout << "publish_modified_map_timed:" << std::boolalpha <<
      publish_modified_map_timed_ << std::endl;
    std::cout << "modified_map_timed_leaf_size:" << modified_map_timed_leaf_size_ << std::endl;
    std::cout << "odom_input_cloud_in_odom_frame:" << std::boolalpha <<
      odom_input_cloud_in_odom_frame_ << std::endl;
    std::cout << "global_frame_id:" << global_frame_id_ << std::endl;
    std::cout << "odom_frame_id:" << odom_frame_id_ << std::endl;
  }
  std::cout << "use_imu_preintegration:" << std::boolalpha << use_imu_preintegration_ << std::endl;
  if (use_imu_preintegration_) {
    std::cout << "imu_rotation_info_roll_pitch:" << imu_rotation_info_roll_pitch_ << std::endl;
    std::cout << "imu_rotation_info_yaw:" << imu_rotation_info_yaw_ << std::endl;
  }
  if (use_gnss_) {
    std::cout << "gnss_topic:" << gnss_topic_ << std::endl;
    std::cout << "gnss_info_weight:" << gnss_info_weight_ << std::endl;
    std::cout << "gnss_use_covariance_weighting:" << std::boolalpha <<
      gnss_use_covariance_weighting_ << std::endl;
    std::cout << "gnss_covariance_min_variance_m2:" << gnss_covariance_min_variance_m2_ <<
      std::endl;
    std::cout << "gnss_covariance_max_variance_m2:" << gnss_covariance_max_variance_m2_ <<
      std::endl;
    std::cout << "gnss_rtk_fix_max_horizontal_stddev_m:" <<
      gnss_rtk_fix_max_horizontal_stddev_m_ << std::endl;
    std::cout << "gnss_max_horizontal_residual_m:" <<
      gnss_max_horizontal_residual_m_ << std::endl;
    std::cout << "gnss_rtk_fix_weight_scale:" << gnss_rtk_fix_weight_scale_ << std::endl;
    std::cout << "gnss_non_rtk_weight_scale:" << gnss_non_rtk_weight_scale_ << std::endl;
    std::cout << "gnss_origin_min_samples:" << gnss_origin_min_samples_ << std::endl;
    std::cout << "gnss_origin_consistency_threshold_m:"
              << gnss_origin_consistency_threshold_m_ << std::endl;
  }
  std::cout << "------------------" << std::endl;

  voxelgrid_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

  if (registration_method == "NDT") {
    boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>
    ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setMaximumIterations(100);
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(0.01);
    // ndt->setTransformationEpsilon(1e-6);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}
    registration_ = ndt;
  } else if (registration_method == "GICP") {
    boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>
    gicp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setMaxCorrespondenceDistance(30);
    gicp->setMaximumIterations(100);
    // gicp->setCorrespondenceRandomness(20);
    gicp->setTransformationEpsilon(1e-8);
    gicp->setEuclideanFitnessEpsilon(1e-6);
    gicp->setRANSACIterations(0);
    registration_ = gicp;
  } else {
    RCLCPP_ERROR(get_logger(), "invalid registration_method");
    exit(1);
  }

  bev_descriptor_db_.configure(
    bev_descriptor_grid_size_m_,
    bev_descriptor_grid_cells_,
    bev_descriptor_yaw_bins_);

  initializePubSub();

  map_save_srv_ = create_service<std_srvs::srv::Empty>(
    "map_save",
    std::bind(
      &GraphBasedSlamComponent::handleMapSaveRequest,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3));

  search_worker_ = std::thread(&GraphBasedSlamComponent::searchWorkerLoop, this);
  publish_worker_ = std::thread(&GraphBasedSlamComponent::publishWorkerLoop, this);
  if (dem_enabled_) {
    dem_worker_ = std::thread(&GraphBasedSlamComponent::demWorkerLoop, this);
    RCLCPP_INFO(
      get_logger(),
      "Lunar DEM recorder enabled: storage=%.3fm preview=%.3fm period=%.1fs output=%s",
      dem_storage_resolution_m_, dem_preview_resolution_m_, dem_update_period_sec_,
      dem_output_dir_.c_str());
  }
}  // NOLINT(readability/fn_size)

/*
Summary:
Requests shutdown, joins every backend worker, and releases the PCD cache lock.
*/
GraphBasedSlamComponent::~GraphBasedSlamComponent()
{
  requestShutdown();
  if (search_worker_.joinable()) {
    search_worker_.join();
  }
  if (publish_worker_.joinable()) {
    publish_worker_.join();
  }
  if (dem_worker_.joinable()) {
    dem_worker_.join();
  }
  cleanupPcdCacheSession();
}

/*
Summary:
Atomically marks the component as stopping and wakes all sleeping workers.
*/
void GraphBasedSlamComponent::requestShutdown()
{
  if (shutting_down_.exchange(true)) {
    return;
  }
  search_worker_cv_.notify_all();
  publish_worker_cv_.notify_all();
  dem_worker_cv_.notify_all();
}

/*
Summary:
Creates graph inputs, corrected-map outputs, diagnostics, and map-save service
interfaces, including direct odometry/cloud subscriptions when enabled.
*/
void GraphBasedSlamComponent::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");

  auto map_array_callback =
    [this](const typename lidarslam_msgs::msg::MapArray::SharedPtr msg_ptr) -> void
    {
      std::lock_guard<std::mutex> lock(mtx_);
      map_array_msg_ = *msg_ptr;
      // Save new submaps to PCD and clear cloud from memory
      if (use_pcd_cache_) {
        for (int i = 0; i < static_cast<int>(map_array_msg_.submaps.size()); i++) {
          auto & sub = map_array_msg_.submaps[i];
          if (sub.cloud.data.size() > 0) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(sub.cloud, *cloud);
            if (cloud->size() > 0) {
              saveSubmapToPCD(i, cloud);
              sub.cloud = sensor_msgs::msg::PointCloud2();  // Free memory
            }
          }
        }
      }
      // Publish NewSubmap for every submap appended since the previous
      // map_array update, using msg_ptr's cloud (not yet cleared above).
      const int total_submaps = static_cast<int>(map_array_msg_.submaps.size());
      for (int i = previous_submaps_num_; i < total_submaps; ++i) {
        lidarslam_msgs::msg::NewSubmap ns;
        ns.header.stamp = msg_ptr->submaps[i].header.stamp;
        ns.header.frame_id = global_frame_id_;
        ns.submap_index = static_cast<uint32_t>(i);
        ns.pose = msg_ptr->submaps[i].pose;
        ns.cloud = msg_ptr->submaps[i].cloud;
        submap_created_pub_->publish(ns);
      }
      previous_submaps_num_ = total_submaps;
      initial_map_array_received_ = true;
      is_map_array_updated_ = true;
    };

  map_array_sub_ =
    create_subscription<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), map_array_callback);

  if (use_odom_input_) {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom_input", 10,
      std::bind(&GraphBasedSlamComponent::receiveOdometry, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_input", rclcpp::SensorDataQoS(),
      std::bind(&GraphBasedSlamComponent::receiveCloud, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Direct odom+cloud input mode enabled");
  }

  // Loop search cadence and periodic modified-map publishing run on their
  // own dedicated worker threads (searchWorkerLoop() / publishWorkerLoop(),
  // started at the end of the constructor), not on executor timers.

  if (publish_modified_map_) {
    modified_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "modified_map",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  }

  if (publish_modified_map_timed_) {
    modified_map_timed_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "modified_map_timed",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  }

  loop_diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
    "loop_diagnostics",
    rclcpp::QoS(50));

  backend_timing_diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
    "backend_timing_diagnostics",
    rclcpp::QoS(50));

  dem_preview_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "lunar_dem/preview",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  dem_status_pub_ = create_publisher<std_msgs::msg::String>(
    "lunar_dem/status",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  modified_map_array_pub_ = create_publisher<lidarslam_msgs::msg::MapArray>(
    "modified_map_array",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  modified_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "modified_path",
    rclcpp::QoS(10));

  submap_created_pub_ = create_publisher<lidarslam_msgs::msg::NewSubmap>(
    "submap_created", rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

  if (modified_map_publish_period_sec_ > 0.0) {
    RCLCPP_INFO(
      get_logger(),
      "Periodic modified map publishing enabled at %.2f sec",
      modified_map_publish_period_sec_);
  }

  if (use_imu_preintegration_) {
    auto imu_callback =
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) -> void
      {
        receiveImu(*msg);
      };
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(), imu_callback);
    RCLCPP_INFO(get_logger(), "IMU preintegration enabled, subscribed to /imu");
  }

  if (use_gnss_) {
    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gnss_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {receiveNavSatFix(*msg);});
    RCLCPP_INFO(
      get_logger(),
      "GNSS constraints enabled, subscribed to %s",
      gnss_topic_.c_str());
  }

  RCLCPP_INFO(get_logger(), "initialization end");
}

}  // namespace graphslam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::GraphBasedSlamComponent)
