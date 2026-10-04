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
Declares the graph-SLAM ROS 2 component and the state shared by ingestion,
loop-search, optimization, corrected-map publication, PCD caching, GNSS/IMU
aiding, and DEM workers. Method implementations are divided by responsibility
across the graph_based_slam source files.
*/

#ifndef GRAPH_BASED_SLAM__GRAPH_BASED_SLAM_COMPONENT_H_
#define GRAPH_BASED_SLAM__GRAPH_BASED_SLAM_COMPONENT_H_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GS_GBS_EXPORT __attribute__ ((dllexport))
    #define GS_GBS_IMPORT __attribute__ ((dllimport))
  #else
    #define GS_GBS_EXPORT __declspec(dllexport)
    #define GS_GBS_IMPORT __declspec(dllimport)
  #endif
  #ifdef GS_GBS_BUILDING_DLL
    #define GS_GBS_PUBLIC GS_GBS_EXPORT
  #else
    #define GS_GBS_PUBLIC GS_GBS_IMPORT
  #endif
  #define GS_GBS_PUBLIC_TYPE GS_GBS_PUBLIC
  #define GS_GBS_LOCAL
#else
  #define GS_GBS_EXPORT __attribute__ ((visibility("default")))
  #define GS_GBS_IMPORT
  #if __GNUC__ >= 4
    #define GS_GBS_PUBLIC __attribute__ ((visibility("default")))
    #define GS_GBS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GS_GBS_PUBLIC
    #define GS_GBS_LOCAL
  #endif
  #define GS_GBS_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <pcl/point_types.h>  // NOLINT(build/include_order)
#include <pcl/io/pcd_io.h>  // NOLINT(build/include_order)
#include <pcl/registration/gicp.h>  // NOLINT(build/include_order)
#include <pcl/registration/ndt.h>  // NOLINT(build/include_order)
#include <pcl_conversions/pcl_conversions.h>  // NOLINT(build/include_order)
#include <pclomp/gicp_omp.h>  // NOLINT(build/include_order)
#include <pclomp/ndt_omp.h>  // NOLINT(build/include_order)
#include <pclomp/voxel_grid_covariance_omp.h>  // NOLINT(build/include_order)
#include <tf2_ros/buffer.h>  // NOLINT(build/include_order)
#include <tf2_ros/transform_broadcaster.h>  // NOLINT(build/include_order)
#include <tf2_ros/transform_listener.h>  // NOLINT(build/include_order)

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lidarslam_msgs/msg/map_array.hpp>
#include <lidarslam_msgs/msg/new_submap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pclomp/gicp_omp_impl.hpp>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "graph_based_slam/gnss_weighting.hpp"
#include "graph_based_slam/scan_context.hpp"
#include "graph_based_slam/solid_descriptor.hpp"
#include "graph_based_slam/submap_bev_descriptor.hpp"
#include "graph_based_slam/three_d_bbs_loop_verifier.hpp"
#include "graph_based_slam/triangle_descriptor_database.hpp"

namespace graphslam
{
namespace hazard {class Worker;}
  namespace optimization
  {
    class PoseGraphOptimizer;
  }

  /*
  Summary:
  Stores one corrected map point with source time and submap provenance.
  */
  struct TimedMapPoint  // NOLINT(runtime/indentation_namespace)
  {
    float x {0.0F};
    float y {0.0F};
    float z {0.0F};
    float intensity {0.0F};
    // FLOAT32 loses submap-level resolution near current epoch timestamps.
    double time {0.0};
    uint32_t submap_index {0U};
  };

  struct TimedVoxelKey  // NOLINT(runtime/indentation_namespace)
  {
    int ix {0};
    int iy {0};
    int iz {0};

    /*
    Summary:
    Compares integer voxel coordinates for unordered-map lookup.
    */
    bool operator==(const TimedVoxelKey & other) const
    {
      return ix == other.ix && iy == other.iy && iz == other.iz;
    }
  };

  struct TimedVoxelKeyHash  // NOLINT(runtime/indentation_namespace)
  {
    /*
    Summary:
    Hashes a three-dimensional integer voxel coordinate.
    */
    std::size_t operator()(const TimedVoxelKey & key) const
    {
      std::size_t hash = static_cast<std::size_t>(key.ix) * 73856093U;
      hash ^= static_cast<std::size_t>(key.iy) * 19349663U;
      hash ^= static_cast<std::size_t>(key.iz) * 83492791U;
      return hash;
    }
  };

  struct TimedVoxelAccumulator  // NOLINT(runtime/indentation_namespace)
  {
    double x {0.0};
    double y {0.0};
    double z {0.0};
    double intensity {0.0};
    double newest_time {-std::numeric_limits<double>::infinity()};
    uint32_t newest_submap_index {0U};
    int count {0};

    /*
    Summary:
    Accumulates a point while retaining the newest timestamp and submap index.
    */
    void add(const pcl::PointXYZI & point, double time, uint32_t submap_index)
    {
      x += point.x;
      y += point.y;
      z += point.z;
      intensity += point.intensity;
      if (time >= newest_time) {
        newest_time = time;
        newest_submap_index = submap_index;
      }
      ++count;
    }

    /*
    Summary:
    Produces the averaged timed point represented by this voxel accumulator.
    */
    TimedMapPoint centroid() const
    {
      TimedMapPoint point;
      const double inverse_count = count > 0 ? 1.0 / static_cast<double>(count) : 0.0;
      point.x = static_cast<float>(x * inverse_count);
      point.y = static_cast<float>(y * inverse_count);
      point.z = static_cast<float>(z * inverse_count);
      point.intensity = static_cast<float>(intensity * inverse_count);
      point.time = newest_time;
      point.submap_index = newest_submap_index;
      return point;
    }
  };

  /*
  Summary:
  Owns the complete graph-SLAM backend, its ROS interfaces, and background workers.
  */
  class GraphBasedSlamComponent: public rclcpp::Node  // NOLINT(runtime/indentation_namespace)
  {
public:
    GS_GBS_PUBLIC
    /*
    Summary:
    Loads configuration, initializes backend components, and starts workers.
    */
    explicit GraphBasedSlamComponent(const rclcpp::NodeOptions & options);
    GS_GBS_PUBLIC
    /*
    Summary:
    Stops and joins workers and releases the active PCD cache session.
    */
    ~GraphBasedSlamComponent() override;
    GS_GBS_PUBLIC
    /*
    Summary:
    Signals every backend worker to stop and wakes blocked condition variables.
    */
    void requestShutdown();

private:
    std::unique_ptr<hazard::Worker> hazard_mapping_;
    std::mutex mtx_;

    rclcpp::Clock clock_;
    tf2_ros::Buffer tfbuffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;

    boost::shared_ptr < pcl::Registration < pcl::PointXYZI, pcl::PointXYZI >> registration_;
    pcl::VoxelGrid < pcl::PointXYZI > voxelgrid_;

    lidarslam_msgs::msg::MapArray map_array_msg_;
    rclcpp::Subscription < lidarslam_msgs::msg::MapArray > ::SharedPtr map_array_sub_;
    rclcpp::Publisher < lidarslam_msgs::msg::MapArray > ::SharedPtr modified_map_array_pub_;
    rclcpp::Publisher < nav_msgs::msg::Path > ::SharedPtr modified_path_pub_;
    rclcpp::Publisher < sensor_msgs::msg::PointCloud2 > ::SharedPtr modified_map_pub_;
    rclcpp::Publisher < sensor_msgs::msg::PointCloud2 > ::SharedPtr modified_map_timed_pub_;
    rclcpp::Publisher < std_msgs::msg::String > ::SharedPtr loop_diagnostics_pub_;
    // Dedicated topic (separate from loop_diagnostics_pub_) for per-stage
    // timing breakdowns of searchLoop() / doPoseAdjustment() / the backend
    // worker iteration -- used to diagnose why /modified_map sometimes lags
    // (e.g. under fast motion, when descriptor DB rebuilding or grid map
    // save work grows past the worker's iteration budget).
    rclcpp::Publisher < std_msgs::msg::String > ::SharedPtr backend_timing_diagnostics_pub_;
    rclcpp::Publisher < sensor_msgs::msg::PointCloud2 > ::SharedPtr dem_preview_pub_;
    rclcpp::Publisher < std_msgs::msg::String > ::SharedPtr dem_status_pub_;
    rclcpp::Publisher < lidarslam_msgs::msg::NewSubmap > ::SharedPtr submap_created_pub_;
    rclcpp::Service < std_srvs::srv::Empty > ::SharedPtr map_save_srv_;

    // Two independent backend worker threads so loop search (which can take
    // over a second while catching up on many new submaps) can never block
    // periodic /modified_map publication, and neither ever runs on the ROS
    // executor thread (which must stay free to service ingestion callbacks
    // at frontend rate).
    //   search_worker_: searchLoop() + the loop-edge-triggered
    //     doPoseAdjustment() (see searchWorkerLoop()).
    //   publish_worker_: periodic doPoseAdjustment()-driven publish and
    //     /map_save handling, on its own cadence (see publishWorkerLoop()).
    // doPoseAdjustment() itself may run concurrently from either thread --
    // it already serializes internally via modified_map_publish_mtx_, and
    // only touches state read from a fresh snapshotGraphState() copy, so the
    // two threads never need to coordinate directly.
    std::thread search_worker_;
    std::thread publish_worker_;
    std::condition_variable search_worker_cv_;
    std::mutex search_worker_cv_mtx_;
    std::condition_variable publish_worker_cv_;
    std::mutex publish_worker_cv_mtx_;
    std::atomic<bool> shutting_down_ {false};
    std::atomic<bool> optimize_requested_ {false};
    std::atomic<bool> save_map_requested_ {false};
    /*
    Summary:
    Periodically searches for new loop constraints and requests optimization.
    */
    void searchWorkerLoop();

    /*
    Summary:
    Publishes corrected products and services queued map-save operations.
    */
    void publishWorkerLoop();

    struct LoopEdge
    {
      std::pair < int, int > pair_id;
      Eigen::Isometry3d relative_pose;
      double fitness_score {0.0};
    };
    using LoopEdges = std::vector < LoopEdge >;
    using MapSaveRequestHeader = std::shared_ptr < rmw_request_id_t >;
    using MapSaveRequest = std::shared_ptr < std_srvs::srv::Empty::Request >;
    using MapSaveResponse = std::shared_ptr < std_srvs::srv::Empty::Response >;

    /*
    Summary:
    Creates all subscriptions, publishers, diagnostics, and map-save services.
    */
    void initializePubSub();

    /*
    Summary:
    Queues a non-blocking map-save request for the publication worker.
    */
    void handleMapSaveRequest(
      const MapSaveRequestHeader request_header,
      const MapSaveRequest request,
      const MapSaveResponse response);
    /*
    Summary:
    Advances descriptor databases and schedules loop searches for pending submaps.
    */
    void searchLoop();
    // Per-query loop search body, factored out of searchLoop() so the scheduler
    // can run it for one (default) or many (deterministic mode) query submaps.
    /*
    Summary:
    Generates and verifies loop candidates for one selected query submap.
    */
    void searchLoopForLatest(
      const lidarslam_msgs::msg::MapArray & map_array_msg,
      LoopEdges & loop_edges,
      int num_submaps,
      int latest_idx);
    /*
    Summary:
    Copies map and loop-edge state for lock-free worker processing.
    */
    bool snapshotGraphState(
      lidarslam_msgs::msg::MapArray & map_array_msg,
      LoopEdges & loop_edges,
      bool consume_map_update);
    /*
    Summary:
    Copies the currently accepted loop constraints.
    */
    void snapshotLoopEdges(LoopEdges & loop_edges);

    /*
    Summary:
    Inserts or improves one loop constraint and marks graph products dirty.
    */
    bool upsertLoopEdge(const LoopEdge & loop_edge);

    /*
    Summary:
    Solves the pose graph and rebuilds or reuses corrected map products.
    */
    void doPoseAdjustment(
      lidarslam_msgs::msg::MapArray map_array_msg,
      const LoopEdges & loop_edges,
      bool do_save_map);
    /*
    Summary:
    Publishes cached corrected outputs or rebuilds them from current graph state.
    */
    void publishMapAndPose();

    /*
    Summary:
    Updates the correction relating optimized map poses to frontend odometry.
    */
    void updateMapToOdomCorrection(
      const geometry_msgs::msg::Pose & odom_pose,
      const Eigen::Isometry3d & optimized_map_pose);
    /*
    Summary:
    Broadcasts the current map-to-odom correction at a safe timestamp.
    */
    void publishMapToOdomTf(const rclcpp::Time & stamp);

    /*
    Summary:
    Publishes one structured loop-search diagnostic payload.
    */
    void publishLoopDiagnostic(const std::string & payload);

    /*
    Summary:
    Publishes one structured backend timing diagnostic payload.
    */
    void publishBackendTimingDiagnostic(const std::string & payload);

    // Matching-ready lunar DEM recorder. Graph SLAM only hands an immutable
    // corrected-map snapshot to this worker; rasterization, disk I/O and the
    // optional preview publication never execute in a backend critical path.
    struct DemJob
    {
      pcl::PointCloud < pcl::PointXYZI > ::ConstPtr cloud;
      uint64_t revision {0};
      rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
    };
    bool dem_enabled_ {false};
    double dem_update_period_sec_ {10.0};
    double dem_storage_resolution_m_ {0.10};
    double dem_preview_resolution_m_ {0.50};
    int dem_min_points_per_cell_ {3};
    double dem_terrain_percentile_ {0.25};
    int dem_interpolation_max_component_cells_ {4};
    int dem_interpolation_search_radius_cells_ {2};
    int dem_interpolation_min_valid_neighbors_ {6};
    double dem_interpolation_max_neighbor_height_range_m_ {0.10};
    double dem_interpolation_max_plane_residual_m_ {0.03};
    bool dem_publish_preview_ {true};
    int dem_preview_max_cells_ {250000};
    int dem_max_raster_cells_ {4000000};
    std::string dem_output_dir_;
    std::thread dem_worker_;
    std::condition_variable dem_worker_cv_;
    std::mutex dem_worker_mtx_;
    std::unique_ptr < DemJob > pending_dem_job_;
    std::atomic<uint64_t> dem_revision_counter_ {0};
    std::atomic<uint64_t> dem_last_completed_revision_ {0};
    std::chrono::steady_clock::time_point dem_last_scheduled_time_ {};
    /*
    Summary:
    Queues an immutable corrected-map snapshot for asynchronous DEM generation.
    */
    void scheduleDemJob(
      const pcl::PointCloud < pcl::PointXYZI > ::ConstPtr & cloud,
      const rclcpp::Time & stamp,
      bool force = false);
    /*
    Summary:
    Consumes queued DEM jobs until component shutdown.
    */
    void demWorkerLoop();

    /*
    Summary:
    Rasterizes, stores, and previews one corrected-map DEM snapshot.
    */
    void processDemJob(const DemJob & job);

    /*
    Summary:
    Publishes DEM worker state and detail for monitoring.
    */
    void publishDemStatus(const std::string & state, const std::string & detail);

    // loop search parameter
    int loop_detection_period_;
    double threshold_loop_closure_score_;
    double scan_context_loop_closure_score_threshold_ {-1.0};
    // Relaxed fitness ceiling for TRIANGLE_DESCRIPTOR candidates with strong
    // RANSAC inlier evidence, mirroring scan_context_loop_closure_score_threshold_.
    // -1.0 = disabled (fall back to threshold_loop_closure_score_).
    // Triangle inliers are the correctness signal; GICP/NDT fitness is an
    // overlap signal. A thin-overlap reverse-direction revisit can have
    // excellent inliers and poor fitness purely from one-sided sensing --
    // gating solely on fitness discards otherwise-correct closures. Fitness
    // still scales loop_edge_info_weight_ downstream (loop_edge_info_weight_
    // / fitness in doPoseAdjustment), so a thin-overlap loop still becomes a
    // WEAKER edge, never a rejected one.
    double triangle_loop_closure_score_threshold_ {-1.0};
    // Guardrail: the relaxed threshold above only applies when a candidate's
    // inlier evidence clears these (independent of, and typically stricter
    // than, the base triangle_descriptor_min_inliers_ / min_inlier_ratio_
    // RANSAC acceptance gate). -1 / -1.0 = no extra bar beyond the base gate.
    // Strong triangles buy fitness leniency; weak ones still get the generic
    // threshold_loop_closure_score_.
    int triangle_relaxed_fitness_min_inliers_ {-1};
    double triangle_relaxed_fitness_min_inlier_ratio_ {-1.0};
    double distance_loop_closure_;
    double range_of_searching_loop_closure_;
    int search_submap_num_;
    int max_loop_candidate_count_ {3};
    int loop_edge_dedup_index_window_ {8};
    double loop_max_translation_delta_ {15.0};
    double loop_max_rotation_delta_deg_ {45.0};
    // Per-source overrides for descriptor-based candidates (TRIANGLE,
    // SCAN_CONTEXT, BEV, SOLID). When positive, replace the generic caps
    // above for those sources only — DISTANCE keeps the strict default.
    // -1.0 = disabled / fall back to the generic cap.
    double loop_max_translation_delta_descriptor_ {-1.0};
    double loop_max_rotation_delta_deg_descriptor_ {-1.0};
    // Pre-shift the NDT/GICP initial guess in z using robust ground height
    // before registration (non-DISTANCE candidates only, skipped when 3D-BBS
    // already localized). Off by default so existing tuned baselines
    // (NTU VIRAL/MID-360/indoor) stay bit-for-bit unchanged; opt in for
    // scenes with significant odometry z drift.
    bool loop_z_preshift_enabled_ {false};
    double loop_z_preshift_max_m_ {5.0};
    // Deterministic loop scheduling (opt-in, v0.4 D1). When false (default),
    // searchLoop queries only the single latest submap per timer tick — the
    // historical wall-clock-driven behaviour, whose (query, db) pair set depends
    // on timer batching rather than the map. When true, searchLoop catches up
    // over every submap index not yet used as a query, so the set of loop-search
    // queries is a pure function of the map regardless of tick timing.
    bool deterministic_loop_scheduling_ {false};
    int last_searched_submap_idx_ {-1};
    // Bounds how many un-queried submaps a single searchLoop() call will
    // catch up on in deterministic mode. 0 (default) = unbounded, preserving
    // prior behaviour exactly -- with it unset, a burst of fast motion can
    // make one searchLoop() call take tens of seconds (all of it still
    // eventually gets queried, just possibly in one very long pass). Set > 0
    // to spread a large backlog across multiple ticks instead; determinism
    // (every submap is queried exactly once, eventually) is preserved either
    // way, only the latency of any single searchLoop() call changes.
    int deterministic_loop_scheduling_max_queries_per_tick_ {0};

    // pose graph optimization parameter
    std::string optimizer_backend_ {"gtsam_isam2"};
    std::unique_ptr < optimization::PoseGraphOptimizer > pose_graph_optimizer_;
    int num_adjacent_pose_cnstraints_;
    bool use_save_map_in_loop_ {true};
    double adjacent_edge_info_weight_ {1000.0};
    double loop_edge_info_weight_ {100.0};
    double loop_edge_robust_kernel_delta_ {1.0};
    std::string loop_edge_robust_kernel_type_ {"huber"};

    // Auto-scaling for adjacent_edge_info_weight (Level 1: NIS median tracking).
    // When enabled, the post-optimisation chi-squared of adjacent edges is
    // monitored and adjacent_edge_info_weight_ is mixed toward
    // current * target_nis / median_chi2 via EMA, clamped to [min, max].
    bool adjacent_edge_info_auto_scale_ {false};
    double adjacent_edge_info_auto_scale_target_nis_ {6.0};
    double adjacent_edge_info_auto_scale_ema_alpha_ {0.3};
    double adjacent_edge_info_auto_scale_min_ {1.0};
    double adjacent_edge_info_auto_scale_max_ {1.0e6};
    // Level 2: split the adjacent edge Information matrix into translation /
    // rotation blocks (block-diag with weights w_trans, w_rot on I_3 each) so
    // the auto-scaler can balance translation residuals and rotation residuals
    // independently. When split mode is off the legacy single-scalar shape is
    // used. Targets default to 3.0 (3 DoF per block, vs 6 for the unified
    // mode); the EMA / min / max defaults are shared with Level 1.
    bool adjacent_edge_info_auto_scale_split_trans_rot_ {false};
    double adjacent_edge_info_weight_trans_ {-1.0};
    double adjacent_edge_info_weight_rot_ {-1.0};
    double adjacent_edge_info_auto_scale_target_nis_trans_ {3.0};
    double adjacent_edge_info_auto_scale_target_nis_rot_ {3.0};
    // Neither the unified scalar path nor the split trans/rot path above
    // distinguish z from xy within the translation block -- x/y/z all get
    // the same weight, even though z is typically the least-observable axis
    // for a ground vehicle and odometry drift concentrates there. This
    // multiplies the (2,2) (z) diagonal entry by this factor relative to
    // whatever w_trans/edge_weight would otherwise apply. 1.0 = no change
    // (default, preserves existing tuned baselines); < 1.0 trusts adjacent
    // z odometry less, letting loop-closure z corrections propagate through
    // the chain instead of being fought by an overconfident z prior.
    double adjacent_edge_info_weight_z_scale_ {1.0};

    bool initial_map_array_received_ {false};
    bool is_map_array_updated_ {false};
    int previous_submaps_num_ {0};

    LoopEdges loop_edges_;

    bool debug_flag_ {false};

    // Scan Context loop detection
    bool use_distance_loop_candidates_ {true};
    bool use_scan_context_ {false};
    double scan_context_threshold_ {0.3};
    bool prefer_scan_context_candidates_ {false};
    ScanContext::Database scan_context_db_;
    bool use_bev_descriptor_ {false};
    double bev_descriptor_threshold_ {0.20};
    double bev_descriptor_grid_size_m_ {80.0};
    int bev_descriptor_grid_cells_ {40};
    int bev_descriptor_yaw_bins_ {24};
    int bev_descriptor_sequence_window_ {0};
    double bev_descriptor_sequence_threshold_ {-1.0};
    double bev_descriptor_pose_consistency_threshold_m_ {-1.0};
    double bev_descriptor_max_euclidean_distance_m_ {-1.0};
    double bev_descriptor_rerank_weight_m_ {100.0};
    // FOV-aware (mutual-visibility) distance for the BEV descriptor. Default
    // off so the cosine-distance baseline stays unchanged on 360° LiDAR.
    bool bev_use_mutual_visibility_ {false};
    double bev_mutual_visibility_min_overlap_ratio_ {0.05};
    double bev_mutual_visibility_occupancy_eps_ {0.5};
    SubmapBEVDescriptor::Database bev_descriptor_db_;
    // Triangle (STD/BTC-style) descriptor place-recognition path. Built on the
    // BSD-2 primitives in graph_based_slam/triangle_descriptor*. Default off
    // so the existing default workflow stays unchanged.
    bool use_triangle_descriptor_ {false};
    // Tuned 2026-05-18 on NTU VIRAL tnp_01 ablation v4. The earlier loose
    // defaults (60 cells, 0.3 m salience, 80 keypoints, 1.0 m edge bin)
    // produced false-positive vote buckets where one stale submap collected
    // every triangle match; this triggered randomly-rotating SE(3) outputs
    // that NDT could not refine. The tighter values below caused triangle
    // to vote across distinct submap ids (32 / 40 / 17 / 9) and produced
    // the first triangle-sourced accepted loop closure (32 <-> 95, 0.49 m
    // / 1.06 deg correction).
    double triangle_descriptor_grid_size_m_ {60.0};
    int triangle_descriptor_grid_cells_ {100};
    int triangle_descriptor_max_keypoints_ {40};
    double triangle_descriptor_min_salience_m_ {0.8};
    double triangle_descriptor_min_edge_m_ {2.0};
    double triangle_descriptor_max_edge_m_ {50.0};
    int triangle_descriptor_max_triangles_ {3000};
    double triangle_descriptor_edge_bin_m_ {0.5};
    // Quad-hash 4th-point feature bin (m). 0 = disabled (legacy 3-edge hash).
    // When > 0, the bucket key also includes the quantized distance from the
    // triangle centroid to the nearest non-vertex keypoint, which makes the
    // hash 4-dim and rejects wrong-but-agreeing triangle pairs in repeated
    // geometry (corridor / parking-row / parallel column rows).
    double triangle_descriptor_quad_feature_bin_m_ {0.0};
    // Keypoint extractor mode. "bev_max_height" is the original outdoor-only
    // extractor; "edge_3d" enables PCA-edgeness keypoints that survive in
    // narrow-FOV / indoor scenes (MID-360, Newer College math_hard) where
    // BEV max-height keypoint repeatability collapses; "surface_saliency"
    // detrends a robust ground-plane fit out of the 2.5D grid and picks
    // curvature extrema (crater rims, curbs, boulders) so open/sloped
    // terrain (parking lots, hills) that starves both other modes still
    // yields ~max_keypoints via percentile top-N rather than a hard gate.
    std::string triangle_descriptor_keypoint_mode_ {"bev_max_height"};
    double triangle_descriptor_edge_voxel_size_m_ {0.4};
    double triangle_descriptor_edge_neighbor_radius_m_ {1.0};
    int triangle_descriptor_edge_min_neighbors_ {6};
    double triangle_descriptor_edge_min_edgeness_ {0.5};
    double triangle_descriptor_edge_nms_radius_m_ {2.0};
    // ----- SURFACE_SALIENCY params (see KeypointExtractionConfig for the
    // per-field rationale; these just plumb the same values from ROS params).
    double triangle_descriptor_surface_plane_fit_percentile_ {0.3};
    int triangle_descriptor_surface_curvature_radius_cells_ {1};
    double triangle_descriptor_surface_min_saliency_percentile_ {0.0};
    // 5-inlier floor would have killed the only accepted loop in v4 (id=32
    // emitted with 4 inliers), so settle on 4 as the compromise between
    // recall and noise. Votes can stay loose because the tighter keypoint
    // / hash params suppress most false buckets on their own.
    int triangle_descriptor_min_votes_ {6};
    int triangle_descriptor_min_inliers_ {4};
    // Number of top vote-getting submaps to verify (each scoped to its own
    // single-submap TriangleDatabase) before picking a winner by
    // inlier_ratio (ties broken by inlier count) instead of trusting
    // top-1-by-votes. A permissive keypoint stage (e.g. surface_saliency)
    // generates more hash collisions, so the single top-voted submap is not
    // reliably the true match; verifying several candidates and letting
    // RANSAC inlier quality pick the winner is the aliasing fix. K=1
    // reproduces the legacy top-1-by-votes behaviour bit-for-bit, which is
    // why it's the default (preserves the NTU VIRAL / Newer College tuning).
    int triangle_descriptor_verify_top_k_ {1};
    // Companion to min_inliers expressed as inliers / eval_n. Zero disables.
    // Lets the operator combine a low absolute count with a meaningful
    // relative-density floor (e.g. 4 inliers / max_pairs 64 = 6% vs the
    // same 4 inliers / max_pairs 20 = 20%).
    double triangle_descriptor_min_inlier_ratio_ {0.0};
    // Cap on triangle pairs evaluated inside the RANSAC consensus check.
    // Lower numbers make min_inlier_ratio more informative; default 64 keeps
    // the previous behaviour.
    int triangle_descriptor_max_pairs_ {64};
    // 4-point consensus: after the 3-point RANSAC picks a winning SE(3),
    // optionally project every query keypoint by that transform and require
    // this many to fall within `fourth_point_max_distance_m` of some
    // database keypoint in the chosen submap. Three points uniquely
    // determine SE(3), so even a strong 3-point consensus can be fooled by
    // repeated structure; the 4-point gate adds an independent constraint.
    // Default 0 disables the gate.
    int triangle_descriptor_min_4th_point_agreements_ {0};
    double triangle_descriptor_fourth_point_max_distance_m_ {2.0};
    // After the 3-point RANSAC picks the winning SE(3), re-estimate it by
    // pooling the 3 * N_inliers point correspondences and running a single
    // N-point Umeyama least-squares. Reduces translation noise by √N versus
    // keeping the single 3-point hypothesis.
    bool triangle_descriptor_refine_se3_with_all_inliers_ {false};
    // Diagnostic-only: when true, run accumulateVotes (and submap_id selection)
    // but skip the RANSAC findLoopCandidate inner loop. Used to isolate
    // "executor scheduling cost of enabling triangle pipeline" from
    // "RANSAC compute cost" when investigating APE drift on tuned configs.
    // Default false (production).
    bool triangle_descriptor_skip_ransac_ {false};
    double triangle_descriptor_inlier_translation_m_ {2.0};
    double triangle_descriptor_inlier_rotation_deg_ {5.0};
    int triangle_descriptor_exclude_recent_ {4};
    // Cross-verification: when both use_triangle_descriptor and
    // use_bev_descriptor are true, gate the triangle candidate by also
    // requiring the BEV mutual-visibility distance to clear an upper bound.
    // Helps filter false positives caused by repeated geometry (corridors,
    // facades) at the cost of triangle-only recall.
    bool triangle_verify_with_bev_ {false};
    double triangle_verify_bev_max_distance_ {0.30};
    graphslam::triangle::TriangleDatabase triangle_descriptor_db_;
    struct TrianglePerSubmap
    {
      std::vector < graphslam::triangle::Keypoint > keypoints;
      std::vector < graphslam::triangle::TriangleDescriptor > triangles;
    };
    std::vector < TrianglePerSubmap > triangle_descriptor_per_submap_;
    int triangle_descriptor_next_submap_idx_ {0};
    bool use_solid_descriptor_ {false};
    double solid_descriptor_min_similarity_ {0.70};
    int solid_descriptor_sequence_window_ {0};
    double solid_descriptor_sequence_min_similarity_ {-1.0};
    double solid_descriptor_pose_consistency_threshold_m_ {-1.0};
    double solid_descriptor_max_euclidean_distance_m_ {-1.0};
    SolidDescriptor::Database solid_descriptor_db_;
    bool use_3d_bbs_for_scan_context_ {false};
    double three_d_bbs_min_level_res_ {1.0};
    int three_d_bbs_max_level_ {3};
    double three_d_bbs_score_threshold_percentage_ {0.25};
    int three_d_bbs_timeout_msec_ {50};
    int three_d_bbs_num_threads_ {0};
    double three_d_bbs_voxel_leaf_size_ {1.0};
    int three_d_bbs_source_submap_num_ {2};
    int three_d_bbs_target_submap_radius_ {1};
    double three_d_bbs_translation_search_margin_m_ {15.0};
    double three_d_bbs_roll_pitch_search_deg_ {10.0};
    double three_d_bbs_yaw_search_deg_ {180.0};
    ThreeDBBSLoopVerifier three_d_bbs_loop_verifier_;

    bool use_dynamic_object_filter_ {false};
    double dynamic_object_filter_voxel_size_ {0.3};
    int dynamic_object_filter_min_observations_ {2};
    int dynamic_object_filter_temporal_window_ {5};
    double dynamic_object_filter_max_range_from_sensor_m_ {30.0};

    // PCD disk cache for memory-efficient submap storage. The configured
    // root contains one locked run_* directory per live graph process.
    std::string pcd_cache_root_dir_;
    std::string pcd_cache_dir_;
    int pcd_cache_lock_fd_ {-1};
    bool use_pcd_cache_ {false};
    /*
    Summary:
    Creates and exclusively locks a run-scoped submap PCD cache directory.
    */
    bool initializePcdCacheSession();

    /*
    Summary:
    Releases the cache lock and removes the completed run directory.
    */
    void cleanupPcdCacheSession();

    /*
    Summary:
    Persists one submap cloud in the active run-scoped PCD cache.
    */
    void saveSubmapToPCD(
      int idx,
      const pcl::PointCloud < pcl::PointXYZI > ::Ptr & cloud);
    /*
    Summary:
    Loads one submap cloud from the active PCD cache.
    */
    pcl::PointCloud < pcl::PointXYZI > ::Ptr loadSubmapFromPCD(int idx);

    // Search-worker-thread-only cache over loadSubmapFromPCD(), used solely
    // by searchLoop()/searchLoopForLatest(). Those two functions only ever
    // run on search_worker_, so this needs no locking -- but for that same
    // reason it must NEVER be touched from doPoseAdjustment() (which can run
    // on either search_worker_ or publish_worker_). Adjacent loop-search
    // queries and per-candidate target windows overlap heavily (symmetric
    // +/-search_submap_num_ windows sliding by ~1 index at a time), so this
    // turns most repeat loadSubmapFromPCD calls into a map lookup instead of
    // a disk read + deserialize. PCD files are written once and never
    // change, so cached entries never go stale. FIFO eviction (not true
    // LRU) is enough here since access is roughly sequential.
    /*
    Summary:
    Loads a submap through the search worker's bounded in-memory cloud cache.
    */
    pcl::PointCloud < pcl::PointXYZI > ::Ptr loadSubmapFromPCDCached(int idx);
    std::unordered_map < int, pcl::PointCloud < pcl::PointXYZI > ::Ptr > submap_cloud_cache_;
    std::deque < int > submap_cloud_cache_order_;
    static constexpr size_t kSubmapCloudCacheMaxEntries = 256;

    // Autoware-compatible grid-divided PCD map output
    std::string map_save_dir_ {"."};
    double map_grid_size_x_ {20.0};
    double map_grid_size_y_ {20.0};
    double map_leaf_size_ {0.2};
    /*
    Summary:
    Writes an Autoware-compatible grid-divided PCD map and metadata.
    */
    void saveGridDividedMap(
      const pcl::PointCloud < pcl::PointXYZI > ::Ptr & map);

    // Direct odometry + cloud input mode (for LIO frontends)
    bool use_odom_input_ {false};
    double submap_distance_threshold_ {1.5};
    bool publish_map_to_odom_tf_ {false};
    double map_to_odom_tf_future_offset_sec_ {0.0};
    std::mutex map_to_odom_tf_publish_mtx_;
    int64_t last_map_to_odom_tf_stamp_ns_ {0};
    double modified_map_publish_period_sec_ {0.0};
    bool publish_modified_map_ {true};
    bool publish_modified_map_array_on_append_ {true};
    double modified_map_leaf_size_ {0.0};
    bool publish_modified_map_timed_ {false};
    double modified_map_timed_leaf_size_ {-1.0};
    bool odom_input_cloud_in_odom_frame_ {false};
    std::string global_frame_id_ {"map"};
    std::string odom_frame_id_ {"odom"};
    Eigen::Isometry3d map_to_odom_ {Eigen::Isometry3d::Identity()};
    std::mutex map_to_odom_mtx_;
    std::mutex modified_map_publish_mtx_;
    // Cached last-assembled /modified_map (+ timed variant), for cheap
    // periodic republish when nothing changed since the last real rebuild.
    // cached_modified_map_msg_, cached_modified_map_timed_msg_,
    // cached_map_valid_, cached_map_submap_count_, and
    // cached_map_loop_edge_count_ are read/written ONLY while holding
    // modified_map_publish_mtx_ (doPoseAdjustment() already does for its
    // whole body; publishMapAndPose() takes it explicitly for the
    // decide-and-maybe-cheap-publish step). graph_dirty_ is the exception:
    // upsertLoopEdge() sets it from the search worker under mtx_, a
    // different mutex, so it must stay atomic to be read safely here.
    sensor_msgs::msg::PointCloud2 cached_modified_map_msg_;
    sensor_msgs::msg::PointCloud2 cached_modified_map_timed_msg_;
    std::chrono::steady_clock::time_point last_periodic_output_publish_time_ {};
    bool cached_map_valid_ {false};
    int cached_map_submap_count_ {-1};
    std::size_t cached_map_loop_edge_count_ {0};
    std::atomic<bool> graph_dirty_ {true};

    // Per-submap incremental assembly cache for doPoseAdjustment()'s
    // modified_map build loop. Reloading + retransforming + reserializing
    // every submap from disk on every rebuild scales linearly with total
    // mission length; most submaps' optimized pose doesn't change between
    // consecutive rebuilds (no loop edges touch them), so this reuses the
    // previously computed contribution for any submap whose pose hasn't
    // moved beyond the epsilon below, and only redoes the expensive work for
    // submaps that are brand new or whose pose actually changed (e.g. after
    // an accepted loop edge). Only ever accessed from inside
    // doPoseAdjustment(), which already holds modified_map_publish_mtx_ for
    // its entire body -- no separate lock needed. Unbounded (one entry per
    // submap ever created): evicting entries would defeat the point, since
    // every submap's contribution is needed on every full-map republish
    // regardless of how "old" it is. This roughly doubles steady-state RAM
    // for the assembled-map data (cached transformed cloud + serialized
    // message per submap) in exchange for O(changed submaps) instead of
    // O(all submaps) rebuild cost.
    struct SubmapAssemblyCacheEntry
    {
      Eigen::Isometry3d pose {Eigen::Isometry3d::Identity()};
      pcl::PointCloud < pcl::PointXYZI > ::Ptr transformed_cloud;
      // Serialized local-coordinate cloud for the modified MapArray. The
      // transformed cloud above is used only for the assembled global map.
      sensor_msgs::msg::PointCloud2 local_cloud_msg;
    };
    std::unordered_map < int, SubmapAssemblyCacheEntry > submap_assembly_cache_;
    // Persistent aggregate products. When optimization leaves every existing
    // submap pose unchanged and only appends new submaps, doPoseAdjustment()
    // extends these instead of walking and copying the whole mission again.
    pcl::PointCloud < pcl::PointXYZI > ::Ptr assembled_map_cloud_ {
      new pcl::PointCloud < pcl::PointXYZI > ()
    };
    std::vector < TimedMapPoint > assembled_timed_map_points_;
    std::unordered_map <
      TimedVoxelKey, TimedVoxelAccumulator, TimedVoxelKeyHash > assembled_timed_voxels_;
    std::size_t assembled_timed_input_point_count_ {0};
    double assembled_timed_voxel_leaf_size_ {-1.0};
    lidarslam_msgs::msg::MapArray assembled_map_array_msg_;
    nav_msgs::msg::Path assembled_path_msg_;
    std::vector < Eigen::Isometry3d > assembled_submap_poses_;
    int assembled_submap_count_ {0};
    bool assembled_products_valid_ {false};
    double submap_assembly_reuse_translation_eps_m_ {1.0e-3};
    double submap_assembly_reuse_rotation_eps_deg_ {0.05};
    rclcpp::Subscription < nav_msgs::msg::Odometry > ::SharedPtr odom_sub_;
    rclcpp::Subscription < sensor_msgs::msg::PointCloud2 > ::SharedPtr cloud_sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    Eigen::Vector3d last_submap_position_ {0, 0, 0};
    bool last_submap_position_valid_ {false};
    double accumulated_distance_ {0.0};
    /*
    Summary:
    Accepts the newest frontend odometry sample for direct-input submap creation.
    */
    void receiveOdometry(const nav_msgs::msg::Odometry & msg);

    /*
    Summary:
    Accepts the newest frontend point cloud for direct-input submap creation.
    */
    void receiveCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /*
    Summary:
    Creates and publishes a submap when synchronized inputs meet travel criteria.
    */
    void tryCreateSubmap();
    nav_msgs::msg::Odometry latest_odom_;
    bool latest_odom_valid_ {false};
    rclcpp::Time latest_cloud_stamp_ {0, 0, RCL_ROS_TIME};

    // GNSS constraints for georeferenced mapping
    bool use_gnss_ {false};
    std::string gnss_topic_ {"/gnss/fix"};
    double gnss_info_weight_ {1.0};
    bool gnss_use_covariance_weighting_ {true};
    double gnss_covariance_min_variance_m2_ {0.01};
    double gnss_covariance_max_variance_m2_ {25.0};
    double gnss_rtk_fix_max_horizontal_stddev_m_ {0.3};
    double gnss_rtk_fix_weight_scale_ {3.0};
    double gnss_non_rtk_weight_scale_ {1.0};
    double gnss_header_stamp_max_skew_sec_ {30.0};
    int gnss_origin_min_samples_ {3};
    double gnss_origin_consistency_threshold_m_ {20.0};
    double gnss_max_horizontal_residual_m_ {10.0};
    rclcpp::Subscription < sensor_msgs::msg::NavSatFix > ::SharedPtr gnss_sub_;
    struct GnssEnu
    {
      double stamp;
      double x;
      double y;
      double z;  // ENU coordinates relative to origin
      double info_x;
      double info_y;
      double info_z;
      bool covariance_valid;
      bool rtk_like;
      double horizontal_stddev_m;
    };
    struct GnssOriginSample
    {
      double lat;
      double lon;
      double alt;
    };
    std::vector < GnssEnu > gnss_buffer_;
    std::vector < GnssOriginSample > gnss_origin_candidates_;
    std::mutex gnss_mtx_;
    std::atomic<bool> gnss_origin_set_ {false};
    double gnss_origin_lat_ {0.0};
    double gnss_origin_lon_ {0.0};
    double gnss_origin_alt_ {0.0};
    /*
    Summary:
    Validates, weights, converts, and buffers an incoming GNSS measurement.
    */
    void receiveNavSatFix(const sensor_msgs::msg::NavSatFix & msg);

    /*
    Summary:
    Reports whether a GNSS fix can safely contribute to the graph.
    */
    bool isUsableGnssFix(const sensor_msgs::msg::NavSatFix & msg) const;

    /*
    Summary:
    Establishes a consistent ENU origin from a configured number of fixes.
    */
    void tryInitializeGnssOrigin(double lat, double lon, double alt);

    /*
    Summary:
    Approximates horizontal geodetic separation in meters.
    */
    double approximateGeodeticDistanceMeters(
      double lat0,
      double lon0,
      double lat1,
      double lon1) const;
    /*
    Summary:
    Converts latitude, longitude, and altitude into the active local ENU frame.
    */
    Eigen::Vector3d geodeticToEnu(double lat, double lon, double alt) const;

    // IMU preintegration
    bool use_imu_preintegration_ {false};
    double imu_rotation_info_roll_pitch_ {100.0};
    double imu_rotation_info_yaw_ {10.0};
    rclcpp::Subscription < sensor_msgs::msg::Imu > ::SharedPtr imu_sub_;
    struct StampedImu
    {
      double stamp;
      double ax;
      double ay;
      double az;
      double gx;
      double gy;
      double gz;
      double qx;
      double qy;
      double qz;
      double qw;
    };
    std::vector < StampedImu > imu_buffer_;
    std::mutex imu_mtx_;
    static constexpr size_t kMaxImuBufferSize = 50000;
    /*
    Summary:
    Buffers an IMU sample for optional adjacent-edge rotation preintegration.
    */
    void receiveImu(const sensor_msgs::msg::Imu & msg);

    /*
    Summary:
    Integrates buffered gyroscope samples over a submap time interval.
    */
    Eigen::Quaterniond integrateImuRotation(double t0, double t1) const;
  };
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GRAPH_BASED_SLAM_COMPONENT_H_
