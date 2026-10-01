/*
 * MIT License
 *
 * Copyright (c) 2025 Meher V.R. Malladi.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file lio.hpp
 * Core LIO class and utilities for RKO-LIO.
 */

#pragma once
#include "sparse_voxel_grid.hpp"
#include "util.hpp"
#include <optional>
#include <string>

/** Core namespace containing LIO data structures and state definitions. */
namespace rko_lio::core {
/** Core LiDAR-inertial odometry algorithm class. */
class LIO {
public:
  /** Per-scan registration quality metrics for ROS diagnostics and gating. */
  struct RegistrationDiagnostics {
    bool valid = false;
    int keypoints = 0;
    int correspondences = 0;
    double inlier_ratio = 0.0;
    double mean_error = 0.0;
    double hessian_min_eigenvalue = 0.0;
    double hessian_max_eigenvalue = 0.0;
    double hessian_condition = 0.0;
    int consecutive_registration_failures = 0;
    bool coarse_to_fine_used = false;
    double degeneracy_damping_alpha_applied = 1.0;
    double eigen_degeneracy_min_scale_applied = 1.0;
    double adaptive_imu_pose_prior_translation_weight_applied = 0.0;
    double adaptive_imu_pose_prior_rotation_weight_applied = 0.0;
    bool rover_degeneracy_motion_constraint_applied = false;
  };

  /** Configuration parameters for odometry. */
  struct Config {
    /** Enable scan deskewing. */
    bool deskew = true;

    /** Maximum number of ICP iterations. */
    size_t max_iterations = 100;

    /** Size of voxel grid (m). */
    double voxel_size = 1.0;

    /** Max points per voxel. */
    int max_points_per_voxel = 20;

    /** Maximum lidar range (m). */
    double max_range = 100.0;

    /** Minimum lidar range (m). */
    double min_range = 1.0;

    /** ICP convergence threshold. */
    double convergence_criterion = 1e-5;

    /** Max distance for correspondences (m). */
    double max_correspondance_distance = 0.5;

    /** Registration residual model: "point_to_point" or "point_to_plane". */
    std::string registration_error_model = "point_to_point";

    /** Cap voxel neighborhood search to avoid large correspondence cubes. */
    int max_voxel_search_radius = 3;

    /** Number of local map neighbors used for point-to-plane fitting. */
    int plane_fit_neighbors = 8;

    /** Minimum local map neighbors required to fit a plane. */
    int plane_fit_min_neighbors = 5;

    /** Maximum point-to-plane neighbor search distance. */
    double plane_fit_max_distance = 1.0;

    /** Maximum accepted smallest plane covariance eigenvalue. */
    double plane_fit_max_eigenvalue = 0.05;

    /** Use only every Nth keypoint for point-to-plane residuals. */
    int point_to_plane_keypoint_stride = 1;

    /** Seed fine ICP with a coarse, more forgiving ICP pass. */
    bool coarse_to_fine_registration = false;

    /** Coarse ICP voxel size for keypoints (m). */
    double coarse_voxel_size = 1.0;

    /** Coarse ICP correspondence distance (m). */
    double coarse_max_correspondance_distance = 2.0;

    /** Maximum iterations for the coarse ICP seed. */
    size_t coarse_max_iterations = 20;

    /** Add an IMU-propagated pose prior directly into each registration iteration. */
    bool enable_imu_pose_prior = false;

    /** Translation weight for the IMU-propagated pose prior. */
    double imu_pose_prior_translation_weight = 0.0;

    /** Rotation weight for the IMU-propagated pose prior. */
    double imu_pose_prior_rotation_weight = 0.0;

    /** Increase the IMU pose prior when LiDAR geometry becomes weak. */
    bool enable_adaptive_imu_pose_prior = true;

    /** Hessian condition where adaptive IMU prior starts increasing. */
    double adaptive_imu_pose_prior_condition = 1500.0;

    /** Hessian condition ratio where adaptive IMU prior reaches max weights. */
    double adaptive_imu_pose_prior_max_condition_ratio = 8.0;

    /** Maximum adaptive translation prior weight under severe degeneracy. */
    double adaptive_imu_pose_prior_max_translation_weight = 0.05;

    /** Maximum adaptive rotation prior weight under severe degeneracy. */
    double adaptive_imu_pose_prior_max_rotation_weight = 2.0;

    /** Damp scan-matching updates when the Hessian indicates degenerate geometry. */
    bool enable_degeneracy_damping = false;

    /** Hessian condition number above which degeneracy damping is applied. */
    double degeneracy_damping_condition = 1000.0;

    /** Fraction of the scan-matching update to keep under degeneracy damping. */
    double degeneracy_damping_alpha = 0.35;

    /** Scale damping by condition severity instead of using a fixed alpha. */
    bool adaptive_degeneracy_damping = true;

    /** Lower bound for adaptive degeneracy damping update scale. */
    double degeneracy_damping_min_alpha = 0.08;

    /** Project ICP updates through Hessian eigenvectors and damp only weak directions. */
    bool enable_eigen_degeneracy_projection = true;

    /** Hessian condition where weak eigen-directions are damped. */
    double eigen_degeneracy_projection_condition = 1500.0;

    /** Minimum update scale for severely weak eigen-directions. */
    double eigen_degeneracy_projection_min_scale = 0.05;

    /** Hold pose when IMU and LiDAR both indicate the rover is stationary. */
    bool enable_stationary_hold = false;

    /** Stationary angular velocity threshold in rad/s. */
    double stationary_angular_velocity_threshold = 0.015;

    /** Stationary body acceleration threshold in m/s^2 after gravity compensation. */
    double stationary_linear_acceleration_threshold = 0.08;

    /** Maximum accepted scan-to-scan translation for stationary hold. */
    double stationary_max_translation_delta = 0.006;

    /** Maximum accepted scan-to-scan rotation for stationary hold. */
    double stationary_max_rotation_delta = 0.004;

    /** Clamp sudden scan-to-scan Z jumps caused by weak vertical constraints. */
    bool enable_vertical_spike_filter = false;

    /** Maximum allowed scan-to-scan Z update before clamping (m). */
    double max_vertical_update_m = 0.08;

    /** Shape weak-geometry motion using rover kinematics instead of freezing all motion. */
    bool enable_rover_degeneracy_motion_constraint = true;

    /** Hessian condition where rover motion shaping becomes active. */
    double rover_degeneracy_motion_condition = 6000.0;

    /** Scale for local forward translation under degeneracy. */
    double rover_degeneracy_forward_scale = 1.0;

    /** Scale for local lateral translation under degeneracy. */
    double rover_degeneracy_lateral_scale = 0.25;

    /** Scale for local vertical translation under degeneracy. */
    double rover_degeneracy_vertical_scale = 0.10;

    /** Scale for local roll/pitch under degeneracy. */
    double rover_degeneracy_roll_pitch_scale = 0.20;

    /** Scale for local yaw under degeneracy. */
    double rover_degeneracy_yaw_scale = 1.0;

    /** Thread count for data association (0 = automatic). */
    int max_num_threads = 0;

    /** Enable initialization phase. */
    bool initialization_phase = false;

    /** Maximum expected jerk (m/s³). */
    double max_expected_jerk = 3;

    /** Gravity magnitude in m/s^2. Earth=9.8107, Moon~=1.625. */
    double gravity_magnitude = GRAVITY_MAG;

    /** Enable double downsampling. */
    bool double_downsample = true;

    /** Minimum weight for orientation regularization. */
    double min_beta = 200;

    /** Maximum delta between adjacent LiDAR scan timestamps (s).
     *  Frames whose stamp is further than this from the previous LiDAR
     *  state time are dropped. Default 1.0 preserves the historic check;
     *  raise it to tolerate kidnap-style recordings with longer scan gaps. */
    double max_scan_delta_sec = 1.0;

    /** Enable recovery after kidnap-style ICP failures. */
    bool enable_kidnap_relocalization = false;

    /** If relocalization fails, start a new local map at the last known pose. */
    bool reset_on_registration_failure = false;

    /** Consecutive registration failures required before recovery is attempted. */
    int recovery_min_failures = 1;

    /** Try global relocalization at the first valid scan after dropped scans. */
    bool relocalize_after_scan_gap = false;

    /** Minimum correspondences required for a relocalization candidate. */
    int relocalization_min_correspondences = 30;

    /** Minimum inlier ratio required for a relocalization candidate. */
    double relocalization_min_inlier_ratio = 0.10;

    /** Maximum accepted mean nearest-neighbor error for relocalization. */
    double relocalization_max_mean_error = 1.5;

    /** ICP correspondence distance used only during global relocalization. */
    double relocalization_max_correspondance_distance = 2.0;

    /** Number of coarse yaw hypotheses to evaluate around each historical pose. */
    int relocalization_yaw_samples = 24;

    /** Historical pose stride for global relocalization candidates. */
    int relocalization_pose_stride = 10;

    /** Recent historical poses to skip when relocalizing. */
    int relocalization_min_pose_separation = 50;

    /** Maximum ICP iterations for each relocalization hypothesis. */
    int relocalization_max_iterations = 15;
  };

  /** Configuration parameters. */
  Config config;

  /** Local map as sparse voxel grid (Bonxai). */
  SparseVoxelGrid map;

  /** Global sparse map used for kidnap relocalization. This map is never pruned. */
  SparseVoxelGrid relocalization_map;

  /** Current LiDAR state estimate. */
  State lidar_state;

  /** IMU bias estimates when initialization is enabled. */
  ImuBias imu_bias;

  /** Mean body acceleration estimate. */
  Eigen::Vector3d mean_body_acceleration = Eigen::Vector3d::Zero();

  /** Covariance of body acceleration estimate. */
  Eigen::Matrix3d body_acceleration_covariance = Eigen::Matrix3d::Identity();

  /** IMU measurement statistics since last LiDAR frame. */
  IntervalStats interval_stats;

  /** Quality metrics from the latest registration attempt. */
  RegistrationDiagnostics last_registration_diagnostics;

  explicit LIO(const Config& config_)
      : config(config_),
        map(config_.voxel_size, config_.max_range, config_.max_points_per_voxel),
        relocalization_map(config_.voxel_size, config_.max_range, config_.max_points_per_voxel) {}

  /** Add an IMU measurement expressed in the base frame. */
  void add_imu_measurement(const ImuControl& base_imu);

  /**
   * Add an IMU measurement expressed in the IMU frame and transform it
   * to the base frame using the given extrinsic calibration.
   * @param extrinsic_imu2base Extrinsic transform from IMU to base frame.
   * @param raw_imu Raw IMU measurement.
   */
  void add_imu_measurement(const Sophus::SE3d& extrinsic_imu2base, const ImuControl& raw_imu);

  /**
   * Register a LiDAR scan, applying deskewing based on the initial motion guess
   * and clipping points beyond valid range.
   * @param scan Input raw point cloud.
   * @param timestamps Absolute timestamps corresponding to each scan point.
   * @return Deskewed and clipped point cloud.
   */
  Vector3dVector register_scan(const Vector3dVector& scan, const TimestampVector& timestamps);

  /**
   * Register a LiDAR scan for which the extrinsic calibration from lidar to base
   * has already been applied.
   * @param extrinsic_lidar2base Extrinsic from lidar to base frame.
   * @param scan Input raw point cloud.
   * @param timestamps Absolute timestamps corresponding to each scan point.
   * @return Deskewed and clipped scan in the original lidar frame.
   */
  Vector3dVector register_scan(const Sophus::SE3d& extrinsic_lidar2base,
                               const Vector3dVector& scan,
                               const TimestampVector& timestamps);

  /** Sequence of registered scan poses with corresponding timestamps. */
  std::vector<std::pair<Secondsd, Sophus::SE3d>> poses_with_timestamps;

private:
  /**
   * Initialize internal odometry state using the given lidar timestamp.
   * @param lidar_time Current lidar timestamp.
   */
  void initialize(const Secondsd lidar_time);

  /** get the convenience struct with accel mag variance and local gravity estimate. */
  std::optional<AccelInfo> get_accel_info(const Sophus::SO3d& rotation_estimate, const Secondsd& time);

  /** Register a recovery scan at a chosen pose and start a fresh local map. */
  Vector3dVector recover_with_scan(const Vector3dVector& filtered_frame,
                                   const Vector3dVector& map_update_frame,
                                   const Secondsd& current_lidar_time,
                                   const Sophus::SE3d& recovery_pose,
                                   const std::string& reason);

  /** Drop an unusable scan while advancing the internal LiDAR timestamp. */
  Vector3dVector drop_failed_scan(const Secondsd& current_lidar_time, const std::string& reason);

  /** Try to align the current scan against the unpruned relocalization map. */
  std::optional<Sophus::SE3d> try_global_relocalization(const Vector3dVector& keypoints) const;

  /** Update both the sliding local map and the unpruned relocalization map. */
  void update_maps(const Vector3dVector& map_update_frame, const Sophus::SE3d& pose);

  /** True if odometry initialization has been completed. */
  bool _initialized = false;

  /** Latest IMU orientation used for gravity compensation. This is ahead of the rotation in the state. */
  Sophus::SO3d _imu_local_rotation;

  /** Timestamp of the latest IMU orientation. Once a scan is registered, this is reset to the lidar state orientation.
   */
  Secondsd _imu_local_rotation_time = Secondsd{0.0};

  /** Timestamp of the most recent real IMU measurement. */
  Secondsd _last_real_imu_time = Secondsd{0.0};

  /** Angular velocity of last true IMU measurement expressed in base frame. */
  Eigen::Vector3d _last_real_base_imu_ang_vel = Eigen::Vector3d::Zero();

  /** Consecutive scan registration failures since the last accepted scan. */
  int _consecutive_registration_failures = 0;
};
} // namespace rko_lio::core
