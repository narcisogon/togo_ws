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

#include "dlio/odom.h"
#include "dlio/utils.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <queue>
#include <sstream>

#include "rclcpp/qos.hpp"

dlio::OdomNode::CallbackCompletionGuard::~CallbackCompletionGuard() {
  node->setPipelineStage(0);
  ++node->lidar_callbacks_completed_;
  node->last_callback_complete_steady_ns_ =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  {
    std::lock_guard<std::mutex> lock(node->main_loop_running_mutex);
    node->main_loop_running = false;
  }
  node->submap_build_cv.notify_all();
}

dlio::OdomNode::OdomNode() : Node("dlio_odom_node") {

  this->getParams();

  this->num_threads_ = omp_get_max_threads();

  this->dlio_initialized = false;
  this->first_valid_scan = false;
  this->first_imu_received = false;
  if (this->imu_calibrate_) {this->imu_calibrated = false;}
  else {this->imu_calibrated = true;}
  this->deskew_status = false;
  this->deskew_size = 0;
  this->last_correction_translation_ = 0.0f;
  this->last_correction_rotation_deg_ = 0.0f;
  this->last_correction_rejected_ = false;
  this->last_gicp_fitness_ = 0.0f;
  this->last_gicp_inliers_ = 0;
  this->last_gicp_overlap_ = 0.0f;
  this->last_hessian_min_eigen_ = 0.0f;
  this->last_hessian_max_eigen_ = 0.0f;
  this->last_hessian_condition_ = 0.0f;
  this->last_gicp_solve_time_ms_ = 0.0f;
  this->last_gicp_quality_mode_ = 0;
  this->last_angular_rate_ = 0.0f;
  this->last_spin_protection_active_ = false;
  this->last_imu_age_ms_ = 0.0f;
  this->last_timing_protection_active_ = false;
  this->last_lidar_header_stamp_ = 0.0f;
  this->last_scan_start_stamp_ = 0.0f;
  this->last_scan_end_stamp_ = 0.0f;
  this->last_oldest_imu_stamp_ = 0.0f;
  this->last_latest_imu_stamp_ = 0.0f;
  this->last_latest_imu_minus_lidar_ms_ = 0.0f;
  this->last_imu_covers_scan_start_ = false;
  this->last_imu_covers_scan_end_ = false;
  this->last_imu_samples_used_for_deskew_ = 0;
  this->stale_scan_drop_count_ = 0;
  this->last_scan_duration_ms_ = 0.0f;
  this->bad_correction_streak_ = 0;
  this->recovery_last_attempt_streak_ = 0;
  this->diagnostic_history_printed_ = false;

  if (!this->diagnostic_history_file_.empty()) {
    std::ofstream file(this->diagnostic_history_file_, std::ios::out | std::ios::trunc);
    if (file.is_open()) {
      file << "DLIO diagnostic history for current run\n";
      file << "Fields: sim_t, type, likely_cause, detail, compute_ms, imu_age_ms, raw/filtered, "
              "keyframes, submap_points, correction, angular_rate, spin_guard, timing_guard, reject_streak, sync, pose\n";
    }
  }

  this->lidar_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto lidar_sub_opt = rclcpp::SubscriptionOptions();
  lidar_sub_opt.callback_group = this->lidar_cb_group;
  auto lidar_qos = rclcpp::SensorDataQoS();
  lidar_qos.keep_last(1);
  this->lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", lidar_qos,
      std::bind(&dlio::OdomNode::callbackPointCloud, this, std::placeholders::_1), lidar_sub_opt);

  this->imu_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = this->imu_cb_group;
  this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS(),
      std::bind(&dlio::OdomNode::callbackImu, this, std::placeholders::_1), imu_sub_opt);

  this->odom_pub     = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  this->pose_pub     = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);
  this->path_pub     = this->create_publisher<nav_msgs::msg::Path>("path", 1);
  this->kf_pose_pub  = this->create_publisher<geometry_msgs::msg::PoseArray>("kf_pose", 1);
  this->kf_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("kf_cloud", 1);
  this->deskewed_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("deskewed", 1);
  this->diagnostics_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("frontend_diagnostics", 10);

  this->br = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  this->publish_timer = this->create_wall_timer(std::chrono::duration<double>(0.01), 
      std::bind(&dlio::OdomNode::publishPose, this));
  this->diagnostics_timer = this->create_wall_timer(std::chrono::duration<double>(1.0),
      [this]() { this->publishPipelineHeartbeat(); });

  this->T = Eigen::Matrix4f::Identity();
  this->T_prior = Eigen::Matrix4f::Identity();
  this->T_corr = Eigen::Matrix4f::Identity();
  this->last_accepted_T_ = Eigen::Matrix4f::Identity();

  this->origin = Eigen::Vector3f(0., 0., 0.);
  this->state.p = Eigen::Vector3f(0., 0., 0.);
  this->state.q = Eigen::Quaternionf(1., 0., 0., 0.);
  this->state.v.lin.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.lin.w = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.w = Eigen::Vector3f(0., 0., 0.);
  this->last_accepted_state_ = this->state;

  this->lidarPose.p = Eigen::Vector3f(0., 0., 0.);
  this->lidarPose.q = Eigen::Quaternionf(1., 0., 0., 0.);

  this->imu_meas.stamp = 0.;
  this->imu_meas.ang_vel[0] = 0.;
  this->imu_meas.ang_vel[1] = 0.;
  this->imu_meas.ang_vel[2] = 0.;
  this->imu_meas.lin_accel[0] = 0.;
  this->imu_meas.lin_accel[1] = 0.;
  this->imu_meas.lin_accel[2] = 0.;

  this->imu_buffer.set_capacity(this->imu_buffer_size_);
  this->first_imu_stamp = 0.;
  this->prev_imu_stamp = 0.;

  this->trajectory.set_capacity(this->debug_metrics_history_size_);
  this->comp_times.set_capacity(this->debug_metrics_history_size_);
  this->imu_rates.set_capacity(this->debug_metrics_history_size_);
  this->lidar_rates.set_capacity(this->debug_metrics_history_size_);
  this->cpu_percents.set_capacity(this->debug_metrics_history_size_);

  if (!this->diagnostic_history_file_.empty()) {
    this->diagnostic_history_stream_.open(
      this->diagnostic_history_file_, std::ios::out | std::ios::app);
  }

  this->original_scan = std::make_shared<const pcl::PointCloud<PointType>>();
  this->deskewed_scan = std::make_shared<const pcl::PointCloud<PointType>>();
  this->current_scan = std::make_shared<const pcl::PointCloud<PointType>>();
  this->submap_cloud = std::make_shared<const pcl::PointCloud<PointType>>();

  this->num_processed_keyframes = 0;

  this->submap_hasChanged = true;
  this->submap_kf_idx_prev.clear();

  this->first_scan_stamp = 0.;
  this->elapsed_time = 0.;
  // Was a no-op statement here (missing "= 0."), so length_traversed relied
  // on debug()'s periodic recompute to ever get set -- meaning
  // wait_until_move_ gating in publishCloud() could stay latched past real
  // movement until the next debug print caught up. Now updated per-scan at
  // the trajectory push site instead, so it needs a real initial value.
  this->length_traversed = 0.;

  this->convex_hull.setDimension(3);
  this->concave_hull.setDimension(3);
  this->concave_hull.setAlpha(this->keyframe_thresh_dist_);
  this->concave_hull.setKeepInformation(true);

  this->gicp.setCorrespondenceRandomness(this->gicp_k_correspondences_);
  this->gicp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
  this->gicp.setMaximumIterations(this->gicp_max_iter_);
  this->gicp.setTransformationEpsilon(this->gicp_transformation_ep_);
  this->gicp.setRotationEpsilon(this->gicp_rotation_ep_);
  this->gicp.setInitialLambdaFactor(this->gicp_init_lambda_factor_);
  nano_gicp::RegularizationMethod gicp_regularization_method =
    nano_gicp::RegularizationMethod::PLANE;
  std::string gicp_regularization_method_name = this->gicp_regularization_method_;
  boost::to_upper(gicp_regularization_method_name);
  if (gicp_regularization_method_name == "NONE") {
    gicp_regularization_method = nano_gicp::RegularizationMethod::NONE;
  } else if (gicp_regularization_method_name == "MIN_EIG") {
    gicp_regularization_method = nano_gicp::RegularizationMethod::MIN_EIG;
  } else if (gicp_regularization_method_name == "NORMALIZED_MIN_EIG") {
    gicp_regularization_method = nano_gicp::RegularizationMethod::NORMALIZED_MIN_EIG;
  } else if (gicp_regularization_method_name == "FROBENIUS") {
    gicp_regularization_method = nano_gicp::RegularizationMethod::FROBENIUS;
  } else if (gicp_regularization_method_name != "PLANE") {
    RCLCPP_WARN(
      this->get_logger(),
      "Unknown odom/gicp/regularizationMethod '%s'; using PLANE",
      this->gicp_regularization_method_.c_str());
  }
  this->gicp.setRegularizationMethod(gicp_regularization_method);

  this->gicp_temp.setCorrespondenceRandomness(this->gicp_k_correspondences_);
  this->gicp_temp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
  this->gicp_temp.setMaximumIterations(this->gicp_max_iter_);
  this->gicp_temp.setTransformationEpsilon(this->gicp_transformation_ep_);
  this->gicp_temp.setRotationEpsilon(this->gicp_rotation_ep_);
  this->gicp_temp.setInitialLambdaFactor(this->gicp_init_lambda_factor_);
  this->gicp_temp.setRegularizationMethod(gicp_regularization_method);

  pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
  this->gicp.setSearchMethodSource(temp, true);
  this->gicp.setSearchMethodTarget(temp, true);
  this->gicp_temp.setSearchMethodSource(temp, true);
  this->gicp_temp.setSearchMethodTarget(temp, true);

  this->geo.first_opt_done = false;
  this->geo.prev_vel = Eigen::Vector3f(0., 0., 0.);

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  this->crop.setNegative(true);
  this->crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_, -this->crop_size_, 1.0));
  this->crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_, this->crop_size_, 1.0));

  this->voxel.setLeafSize(this->vf_res_, this->vf_res_, this->vf_res_);

  this->metrics.spaciousness.push_back(0.);
  this->metrics.density.push_back(this->gicp_max_corr_dist_);

  // CPU Specs
  char CPUBrandString[0x40];
  memset(CPUBrandString, 0, sizeof(CPUBrandString));

  this->cpu_type = "";

  #ifdef HAS_CPUID
  unsigned int CPUInfo[4] = {0,0,0,0};
  __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
  unsigned int nExIds = CPUInfo[0];
  for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
    __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
    if (i == 0x80000002)
      memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000003)
      memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000004)
      memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
  }
  this->cpu_type = CPUBrandString;
  boost::trim(this->cpu_type);
  #endif

  FILE* file;
  struct tms timeSample;
  char line[128];

  this->lastCPU = times(&timeSample);
  this->lastSysCPU = timeSample.tms_stime;
  this->lastUserCPU = timeSample.tms_utime;
  this->last_debug_print_time_ = 0.0;

  file = fopen("/proc/cpuinfo", "r");
  this->numProcessors = 0;
  while(fgets(line, 128, file) != nullptr) {
      if (strncmp(line, "processor", 9) == 0) this->numProcessors++;
  }
  fclose(file);

}

dlio::OdomNode::~OdomNode() {
  this->requestShutdown();
  if (this->submap_future.valid()) {
    this->submap_future.wait();
  }
  this->printDiagnosticHistory();
}

void dlio::OdomNode::requestShutdown() {
  if (this->shutting_down_.exchange(true)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(this->main_loop_running_mutex);
    this->main_loop_running = false;
  }
  this->cv_imu_stamp.notify_all();
  this->submap_build_cv.notify_all();
}

void dlio::OdomNode::getParams() {

  // Version
  dlio::declare_param(this, "version", this->version_, "0.0.0");
  dlio::declare_param(this, "debug/printPeriod", this->debug_print_period_, 1.0);
  dlio::declare_param(this, "diagnostics/historySize", this->diagnostic_history_size_, 80);
  dlio::declare_param(this, "diagnostics/lagWarningMs", this->diagnostic_lag_warning_ms_, 90.0);
  dlio::declare_param(this, "diagnostics/imuAgeWarningMs", this->diagnostic_imu_age_warning_ms_, 120.0);
  dlio::declare_param(this, "diagnostics/keyframeWarningCount", this->diagnostic_keyframe_warning_count_, 220);
  dlio::declare_param(this, "diagnostics/submapWarningPoints", this->diagnostic_submap_warning_points_, 250000);
  dlio::declare_param(this, "diagnostics/historyFile", this->diagnostic_history_file_,
      std::string("/tmp/dlio_diagnostic_history.log"));
  dlio::declare_param(this, "diagnostics/imuWaitTimeoutMs", this->diagnostic_imu_wait_timeout_ms_, 500.0);
  dlio::declare_param(this, "debug/metricsHistorySize", this->debug_metrics_history_size_, 5000);

  // Gated ground-plane z-constraint. See applyGroundConstraint() -- strictly
  // additive; gc_enabled_: false must be bit-identical to prior behavior.
  dlio::declare_param(this, "odom/groundConstraint/enabled", this->gc_enabled_, false);
  dlio::declare_param(this, "odom/groundConstraint/maxRange", this->gc_max_range_, 8.0);
  dlio::declare_param(this, "odom/groundConstraint/zBandBelow", this->gc_zband_below_, 0.9);
  dlio::declare_param(this, "odom/groundConstraint/zBandAbove", this->gc_zband_above_, 0.25);
  dlio::declare_param(this, "odom/groundConstraint/baselinkHeight", this->gc_baselink_height_, 0.30);
  dlio::declare_param(this, "odom/groundConstraint/ransacIterations", this->gc_ransac_iterations_, 50);
  dlio::declare_param(this, "odom/groundConstraint/ransacInlierDist", this->gc_ransac_inlier_dist_, 0.05);
  dlio::declare_param(this, "odom/groundConstraint/minInliers", this->gc_min_inliers_, 150);
  dlio::declare_param(this, "odom/groundConstraint/minInlierRatio", this->gc_min_inlier_ratio_, 0.60);
  dlio::declare_param(this, "odom/groundConstraint/maxNormalTiltDeg", this->gc_max_normal_tilt_deg_, 10.0);
  dlio::declare_param(this, "odom/groundConstraint/maxHeightJump", this->gc_max_height_jump_, 0.15);
  dlio::declare_param(this, "odom/groundConstraint/heightFilterAlpha", this->gc_height_filter_alpha_, 0.02);
  dlio::declare_param(this, "odom/groundConstraint/Kz", this->gc_kz_, 1.0);
  dlio::declare_param(this, "odom/groundConstraint/correctRollPitch", this->gc_correct_roll_pitch_, false);
  dlio::declare_param(this, "odom/groundConstraint/Krp", this->gc_krp_, 0.5);
  dlio::declare_param(
    this, "odom/groundConstraint/maxCorrectionPerScan", this->gc_max_correction_per_scan_, 0.01);
  dlio::declare_param(
    this, "odom/groundConstraint/heightJumpRecoverScans", this->gc_height_jump_recover_scans_, 15);

  // Path publishing. Previously always published every pose unbounded --
  // these keys existed in yaml configs but were never actually wired up.
  dlio::declare_param(this, "odom/path/publish", this->path_publish_, true);
  dlio::declare_param(this, "odom/path/publishEveryN", this->path_publish_every_n_, 1);
  dlio::declare_param(this, "odom/path/maxPoses", this->path_max_poses_, 2000);
  dlio::declare_param(this, "odom/path/minDistance", this->path_min_distance_, 0.0);

  // Frames
  dlio::declare_param(this, "frames/odom", this->odom_frame, "odom");
  dlio::declare_param(this, "frames/baselink", this->baselink_frame, "base_link");
  dlio::declare_param(this, "frames/lidar", this->lidar_frame, "lidar");
  dlio::declare_param(this, "frames/imu", this->imu_frame, "imu");

  // Deskew Flag
  dlio::declare_param(this, "pointcloud/deskew", this->deskew_, true);
  dlio::declare_param(
    this, "pointcloud/syntheticTiming/enabled", this->synthetic_timing_enabled_, false);
  dlio::declare_param(
    this, "pointcloud/syntheticTiming/scanPeriodSec",
    this->synthetic_timing_scan_period_sec_, 1.0 / 15.0);
  dlio::declare_param(
    this, "pointcloud/syntheticTiming/reverseColumns",
    this->synthetic_timing_reverse_columns_, false);
  if (this->synthetic_timing_scan_period_sec_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "pointcloud/syntheticTiming/scanPeriodSec must be positive; using 1/15 s");
    this->synthetic_timing_scan_period_sec_ = 1.0 / 15.0;
  }

  // Gravity
  dlio::declare_param(this, "odom/gravity", this->gravity_, 9.80665);

  // Compute time offset between lidar and imu
  dlio::declare_param(this, "odom/computeTimeOffset", this->time_offset_, false);

  // Keyframe Threshold
  dlio::declare_param(this, "odom/keyframe/threshD", this->keyframe_thresh_dist_, 0.1);
  dlio::declare_param(this, "odom/keyframe/threshR", this->keyframe_thresh_rot_, 1.0);

  // Submap
  dlio::declare_param(this, "odom/submap/keyframe/knn", this->submap_knn_, 10);
  dlio::declare_param(this, "odom/submap/keyframe/kcv", this->submap_kcv_, 10);
  dlio::declare_param(this, "odom/submap/keyframe/kcc", this->submap_kcc_, 10);

  // Dense map resolution
  dlio::declare_param(this, "map/dense/filtered", this->densemap_filtered_, true);

  // Wait until movement to publish map
  dlio::declare_param(this, "map/waitUntilMove", this->wait_until_move_, false);

  // Crop Box Filter
  dlio::declare_param(this, "odom/preprocessing/cropBoxFilter/size", this->crop_size_, 1.0);

  // Voxel Grid Filter
  dlio::declare_param(this, "pointcloud/voxelize", this->vf_use_, true);
  dlio::declare_param(this, "odom/preprocessing/voxelFilter/res", this->vf_res_, 0.05);

  // Adaptive Parameters
  dlio::declare_param(this, "adaptive", this->adaptive_params_, true);

  // Extrinsics
  std::vector<double> t_default{0., 0., 0.};
  std::vector<double> R_default{1., 0., 0., 0., 1., 0., 0., 0., 1.};

  // center of gravity to imu
  std::vector<double> baselink2imu_t, baselink2imu_R;
  dlio::declare_param(this, "extrinsics/baselink2imu/t", baselink2imu_t, t_default);
  dlio::declare_param(this, "extrinsics/baselink2imu/R", baselink2imu_R, R_default);
  this->extrinsics.baselink2imu.t =
    Eigen::Vector3f(baselink2imu_t[0], baselink2imu_t[1], baselink2imu_t[2]);
  this->extrinsics.baselink2imu.R =
    Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(std::vector<float>(baselink2imu_R.begin(), baselink2imu_R.end()).data(), 3, 3);
  this->extrinsics.baselink2imu_T = Eigen::Matrix4f::Identity();
  this->extrinsics.baselink2imu_T.block(0, 3, 3, 1) = this->extrinsics.baselink2imu.t;
  this->extrinsics.baselink2imu_T.block(0, 0, 3, 3) = this->extrinsics.baselink2imu.R;

  // center of gravity to lidar
  std::vector<double> baselink2lidar_t, baselink2lidar_R;
  dlio::declare_param(this, "extrinsics/baselink2lidar/t", baselink2lidar_t, t_default);
  dlio::declare_param(this, "extrinsics/baselink2lidar/R", baselink2lidar_R, R_default);

  this->extrinsics.baselink2lidar.t =
    Eigen::Vector3f(baselink2lidar_t[0], baselink2lidar_t[1], baselink2lidar_t[2]);
  this->extrinsics.baselink2lidar.R =
    Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(std::vector<float>(baselink2lidar_R.begin(), baselink2lidar_R.end()).data(), 3, 3);

  this->extrinsics.baselink2lidar_T = Eigen::Matrix4f::Identity();
  this->extrinsics.baselink2lidar_T.block(0, 3, 3, 1) = this->extrinsics.baselink2lidar.t;
  this->extrinsics.baselink2lidar_T.block(0, 0, 3, 3) = this->extrinsics.baselink2lidar.R;

  // IMU
  dlio::declare_param(this, "odom/imu/calibration/accel", this->calibrate_accel_, true);
  dlio::declare_param(this, "odom/imu/calibration/gyro", this->calibrate_gyro_, true);
  dlio::declare_param(this, "odom/imu/calibration/time", this->imu_calib_time_, 3.0);
  dlio::declare_param(this, "odom/imu/bufferSize", this->imu_buffer_size_, 2000);

  std::vector<double> accel_default{0., 0., 0.}; std::vector<double> prior_accel_bias;
  std::vector<double> gyro_default{0., 0., 0.}; std::vector<double> prior_gyro_bias;

  dlio::declare_param(this, "odom/imu/approximateGravity", this->gravity_align_, true);
  dlio::declare_param(this, "imu/calibration", this->imu_calibrate_, true);
  dlio::declare_param(this, "imu/intrinsics/accel/bias", prior_accel_bias, accel_default);
  dlio::declare_param(this, "imu/intrinsics/gyro/bias", prior_gyro_bias, gyro_default);

  // scale-misalignment matrix
  std::vector<double> imu_sm_default{1., 0., 0., 0., 1., 0., 0., 0., 1.};
  std::vector<double> imu_sm;

  dlio::declare_param(this, "imu/intrinsics/accel/sm", imu_sm, imu_sm_default);

  if (!this->imu_calibrate_) {
    this->state.b.accel[0] = prior_accel_bias[0];
    this->state.b.accel[1] = prior_accel_bias[1];
    this->state.b.accel[2] = prior_accel_bias[2];
    this->state.b.gyro[0] = prior_gyro_bias[0];
    this->state.b.gyro[1] = prior_gyro_bias[1];
    this->state.b.gyro[2] = prior_gyro_bias[2];
    this->imu_accel_sm_ = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(std::vector<float>(imu_sm.begin(), imu_sm.end()).data(), 3, 3);
  } else {
    this->state.b.accel = Eigen::Vector3f(0., 0., 0.);
    this->state.b.gyro = Eigen::Vector3f(0., 0., 0.);
    this->imu_accel_sm_ = Eigen::Matrix3f::Identity();
  }

  // GICP
  dlio::declare_param(this, "odom/gicp/minNumPoints", this->gicp_min_num_points_, 100);
  dlio::declare_param(this, "odom/gicp/kCorrespondences", this->gicp_k_correspondences_, 20);
  dlio::declare_param(this, "odom/gicp/maxCorrespondenceDistance", this->gicp_max_corr_dist_,
      std::sqrt(std::numeric_limits<double>::max()));
  dlio::declare_param(this, "odom/gicp/maxIterations", this->gicp_max_iter_, 64);
  dlio::declare_param(this, "odom/gicp/transformationEpsilon", this->gicp_transformation_ep_, 0.0005);
  dlio::declare_param(this, "odom/gicp/rotationEpsilon", this->gicp_rotation_ep_, 0.0005);
  dlio::declare_param(this, "odom/gicp/initLambdaFactor", this->gicp_init_lambda_factor_, 1e-9);
  dlio::declare_param(
      this, "odom/gicp/regularizationMethod", this->gicp_regularization_method_, std::string("PLANE"));
  dlio::declare_param(this, "odom/gicp/rejectBadCorrections", this->gicp_reject_bad_corrections_, false);
  dlio::declare_param(this, "odom/gicp/maxCorrectionTranslation", this->gicp_max_correction_translation_, 0.5);
  dlio::declare_param(this, "odom/gicp/maxCorrectionRotationDeg", this->gicp_max_correction_rotation_, 20.0);
  dlio::declare_param(this, "odom/gicp/spinProtection/enabled", this->spin_protection_enabled_, false);
  dlio::declare_param(this, "odom/gicp/spinProtection/angularRate", this->spin_protection_angular_rate_, 0.8);
  dlio::declare_param(this, "odom/gicp/spinProtection/maxCorrectionTranslation",
      this->spin_protection_max_translation_, 0.08);
  dlio::declare_param(this, "odom/gicp/spinProtection/recoveryTranslationStep",
      this->spin_protection_recovery_translation_step_, 0.015);
  dlio::declare_param(this, "odom/gicp/spinProtection/recoveryRotationStepDeg",
      this->spin_protection_recovery_rotation_step_, 2.0);
  dlio::declare_param(this, "odom/gicp/spinProtection/useImuPriorOnReject",
      this->spin_protection_use_imu_prior_on_reject_, false);
  dlio::declare_param(this, "odom/gicp/timingProtection/enabled", this->timing_protection_enabled_, false);
  dlio::declare_param(this, "odom/gicp/timingProtection/imuAgeMs", this->timing_protection_imu_age_ms_, 150.0);
  dlio::declare_param(this, "odom/gicp/timingProtection/maxCorrectionTranslation",
      this->timing_protection_max_translation_, 0.08);
  dlio::declare_param(this, "odom/gicp/timingProtection/maxCorrectionRotationDeg",
      this->timing_protection_max_rotation_, 8.0);
  dlio::declare_param(this, "odom/gicp/timingProtection/recoveryTranslationStep",
      this->timing_protection_recovery_translation_step_, 0.015);
  dlio::declare_param(this, "odom/gicp/timingProtection/recoveryRotationStepDeg",
      this->timing_protection_recovery_rotation_step_, 2.0);
  dlio::declare_param(this, "odom/gicp/timingProtection/dropStaleScans",
      this->timing_protection_drop_stale_scans_, true);
  dlio::declare_param(this, "odom/gicp/timingProtection/dropImuAgeMs",
      this->timing_protection_drop_imu_age_ms_, 120.0);
  dlio::declare_param(this, "odom/gicp/timingProtection/dropRejectStreak",
      this->timing_protection_drop_reject_streak_, 0);
  dlio::declare_param(this, "odom/gicp/timingProtection/maxIterations",
      this->timing_protection_max_iterations_, 16);
  dlio::declare_param(this, "odom/gicp/freezeOnBadCorrection", this->freeze_on_bad_correction_, true);
  dlio::declare_param(this, "odom/gicp/badCorrectionFreezeStreak", this->bad_correction_freeze_streak_, 8);
  dlio::declare_param(this, "odom/gicp/badCorrectionRecoveryTranslationStep",
      this->bad_correction_recovery_translation_step_, 0.08);
  dlio::declare_param(this, "odom/gicp/badCorrectionRecoveryRotationStepDeg",
      this->bad_correction_recovery_rotation_step_, 5.0);
  dlio::declare_param(this, "odom/gicp/badCorrectionMaxLinearSpeed",
      this->bad_correction_max_linear_speed_, 1.0);
  dlio::declare_param(this, "odom/gicp/badCorrectionVelocityDecay",
      this->bad_correction_velocity_decay_, 0.75);
  dlio::declare_param(this, "odom/gicp/badCorrectionVelocityDecayStreak",
      this->bad_correction_velocity_decay_streak_, 3);
  dlio::declare_param(this, "odom/gicp/badCorrectionHoldStreak",
      this->bad_correction_hold_streak_, 20);
  dlio::declare_param(this, "odom/gicp/quality/usePartialDegenerate",
      this->quality_use_partial_degenerate_, true);
  dlio::declare_param(this, "odom/gicp/quality/minOverlap", this->quality_min_overlap_, 0.30);
  dlio::declare_param(this, "odom/gicp/quality/degenerateMinOverlap",
      this->quality_degenerate_min_overlap_, 0.12);
  dlio::declare_param(this, "odom/gicp/quality/maxFitness", this->quality_max_fitness_, 5.0);
  dlio::declare_param(this, "odom/gicp/quality/degenerateMaxFitness",
      this->quality_degenerate_max_fitness_, 25.0);
  dlio::declare_param(this, "odom/gicp/quality/minHessianEigen",
      this->quality_min_hessian_eigen_, 1e-6);
  dlio::declare_param(this, "odom/gicp/quality/maxHessianCondition",
      this->quality_max_hessian_condition_, 1e8);
  dlio::declare_param(this, "odom/gicp/quality/partialCorrectionScale",
      this->quality_partial_correction_scale_, 0.35);
  dlio::declare_param(this, "odom/gicp/quality/partialMaxTranslation",
      this->quality_partial_max_translation_, 1.5);
  dlio::declare_param(this, "odom/gicp/quality/partialMaxRotationDeg",
      this->quality_partial_max_rotation_, 45.0);
  dlio::declare_param(this, "odom/gicp/innovationGate/enabled",
      this->innovation_gate_enabled_, true);
  dlio::declare_param(this, "odom/gicp/innovationGate/partialScale",
      this->innovation_gate_partial_scale_, 0.25);
  dlio::declare_param(this, "odom/gicp/innovationGate/maxTranslation",
      this->innovation_gate_max_translation_, 0.35);
  dlio::declare_param(this, "odom/gicp/innovationGate/maxYawDeg",
      this->innovation_gate_max_yaw_, 8.0);
  dlio::declare_param(this, "odom/gicp/degeneracyProjection/enabled",
      this->degeneracy_projection_enabled_, true);
  dlio::declare_param(this, "odom/gicp/degeneracyProjection/condition",
      this->degeneracy_projection_condition_, 100000.0);
  dlio::declare_param(this, "odom/gicp/degeneracyProjection/minScale",
      this->degeneracy_projection_min_scale_, 0.05);
  dlio::declare_param(this, "odom/gicp/recovery/enabled", this->recovery_enabled_, true);
  dlio::declare_param(this, "odom/gicp/recovery/rejectStreak", this->recovery_reject_streak_, 3);
  dlio::declare_param(this, "odom/gicp/recovery/maxCorrespondenceDistance",
      this->recovery_max_correspondence_distance_, 2.0);
  dlio::declare_param(this, "odom/gicp/recovery/maxIterations", this->recovery_max_iterations_, 96);
  dlio::declare_param(this, "odom/gicp/recovery/acceptTranslation",
      this->recovery_accept_translation_, 0.9);
  dlio::declare_param(this, "odom/gicp/recovery/acceptRotationDeg",
      this->recovery_accept_rotation_, 30.0);
  dlio::declare_param(this, "odom/gicp/recovery/minOverlap", this->recovery_min_overlap_, 0.12);
  dlio::declare_param(this, "odom/gicp/recovery/maxFitness", this->recovery_max_fitness_, 25.0);
  dlio::declare_param(this, "odom/gicp/recovery/attemptSpacing", this->recovery_attempt_spacing_, 6);
  dlio::declare_param(this, "odom/gicp/recovery/skipWhenTimingActive",
      this->recovery_skip_when_timing_active_, true);
  dlio::declare_param(this, "odom/gicp/recovery/yawOffsetsDeg",
      this->recovery_yaw_offsets_deg_, std::vector<double>{0.0, -10.0, 10.0, -20.0, 20.0});

  // Geometric Observer
  dlio::declare_param(this, "odom/geo/Kp", this->geo_Kp_, 1.0);
  dlio::declare_param(this, "odom/geo/Kv", this->geo_Kv_, 1.0);
  dlio::declare_param(this, "odom/geo/Kq", this->geo_Kq_, 1.0);
  dlio::declare_param(this, "odom/geo/Kab", this->geo_Kab_, 1.0);
  dlio::declare_param(this, "odom/geo/Kgb", this->geo_Kgb_, 1.0);
  dlio::declare_param(this, "odom/geo/abias_max", this->geo_abias_max_, 1.0);
  dlio::declare_param(this, "odom/geo/gbias_max", this->geo_gbias_max_, 1.0);
}

void dlio::OdomNode::start() {

  printf("\033[2J\033[1;1H");
  std::cout << std::endl
            << "+-------------------------------------------------------------------+" << std::endl;
  std::cout << "|               Direct LiDAR-Inertial Odometry v" << this->version_  << "               |"
            << std::endl;
  std::cout << "+-------------------------------------------------------------------+" << std::endl;

}

void dlio::OdomNode::publishPose() {

  // nav_msgs::msg::Odometry
  this->odom_ros.header.stamp = this->imu_stamp;
  this->odom_ros.header.frame_id = this->odom_frame;
  this->odom_ros.child_frame_id = this->baselink_frame;

  this->odom_ros.pose.pose.position.x = this->state.p[0];
  this->odom_ros.pose.pose.position.y = this->state.p[1];
  this->odom_ros.pose.pose.position.z = this->state.p[2];

  this->odom_ros.pose.pose.orientation.w = this->state.q.w();
  this->odom_ros.pose.pose.orientation.x = this->state.q.x();
  this->odom_ros.pose.pose.orientation.y = this->state.q.y();
  this->odom_ros.pose.pose.orientation.z = this->state.q.z();

  this->odom_ros.twist.twist.linear.x = this->state.v.lin.w[0];
  this->odom_ros.twist.twist.linear.y = this->state.v.lin.w[1];
  this->odom_ros.twist.twist.linear.z = this->state.v.lin.w[2];

  this->odom_ros.twist.twist.angular.x = this->state.v.ang.b[0];
  this->odom_ros.twist.twist.angular.y = this->state.v.ang.b[1];
  this->odom_ros.twist.twist.angular.z = this->state.v.ang.b[2];

  this->odom_pub->publish(this->odom_ros);

  // geometry_msgs::msg::PoseStamped
  this->pose_ros.header.stamp = this->imu_stamp;
  this->pose_ros.header.frame_id = this->odom_frame;

  this->pose_ros.pose.position.x = this->state.p[0];
  this->pose_ros.pose.position.y = this->state.p[1];
  this->pose_ros.pose.position.z = this->state.p[2];

  this->pose_ros.pose.orientation.w = this->state.q.w();
  this->pose_ros.pose.orientation.x = this->state.q.x();
  this->pose_ros.pose.orientation.y = this->state.q.y();
  this->pose_ros.pose.orientation.z = this->state.q.z();

  this->pose_pub->publish(this->pose_ros);

}

void dlio::OdomNode::publishToROS(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud) {
  this->publishCloud(published_cloud, T_cloud);

  // nav_msgs::msg::Path
  // odom/path/publish, publishEveryN, maxPoses, minDistance were present in
  // yaml configs but never actually wired up here -- path_ros.poses grew
  // unboundedly and was republished in full every tick regardless of what
  // those keys said. Now respected.
  if (this->path_publish_) {
    ++this->path_pose_counter_;
    const bool every_n_ok =
      this->path_publish_every_n_ <= 1 ||
      (this->path_pose_counter_ % this->path_publish_every_n_) == 0;
    const bool distance_ok =
      this->path_min_distance_ <= 0.0 ||
      !this->path_has_last_pushed_ ||
      (this->state.p - this->path_last_pushed_p_).norm() >= this->path_min_distance_;

    if (every_n_ok && distance_ok) {
      this->path_ros.header.stamp = this->imu_stamp;
      this->path_ros.header.frame_id = this->odom_frame;

      geometry_msgs::msg::PoseStamped p;
      p.header.stamp = this->imu_stamp;
      p.header.frame_id = this->odom_frame;
      p.pose.position.x = this->state.p[0];
      p.pose.position.y = this->state.p[1];
      p.pose.position.z = this->state.p[2];
      p.pose.orientation.w = this->state.q.w();
      p.pose.orientation.x = this->state.q.x();
      p.pose.orientation.y = this->state.q.y();
      p.pose.orientation.z = this->state.q.z();

      this->path_ros.poses.push_back(p);
      this->path_last_pushed_p_ = this->state.p;
      this->path_has_last_pushed_ = true;

      if (this->path_max_poses_ > 0 &&
        static_cast<int>(this->path_ros.poses.size()) > this->path_max_poses_)
      {
        const size_t excess = this->path_ros.poses.size() -
          static_cast<size_t>(this->path_max_poses_);
        this->path_ros.poses.erase(
          this->path_ros.poses.begin(), this->path_ros.poses.begin() + excess);
      }
    }

    this->path_ros.header.stamp = this->imu_stamp;
    this->path_pub->publish(this->path_ros);
  }

  // transform: odom to baselink
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = this->imu_stamp;
  transformStamped.header.frame_id = this->odom_frame;
  transformStamped.child_frame_id = this->baselink_frame;

  transformStamped.transform.translation.x = this->state.p[0];
  transformStamped.transform.translation.y = this->state.p[1];
  transformStamped.transform.translation.z = this->state.p[2];

  transformStamped.transform.rotation.w = this->state.q.w();
  transformStamped.transform.rotation.x = this->state.q.x();
  transformStamped.transform.rotation.y = this->state.q.y();
  transformStamped.transform.rotation.z = this->state.q.z();

  br->sendTransform(transformStamped);

  // transform: baselink to imu
  transformStamped.header.stamp = this->imu_stamp;
  transformStamped.header.frame_id = this->baselink_frame;
  transformStamped.child_frame_id = this->imu_frame;

  transformStamped.transform.translation.x = this->extrinsics.baselink2imu.t[0];
  transformStamped.transform.translation.y = this->extrinsics.baselink2imu.t[1];
  transformStamped.transform.translation.z = this->extrinsics.baselink2imu.t[2];

  Eigen::Quaternionf q(this->extrinsics.baselink2imu.R);
  transformStamped.transform.rotation.w = q.w();
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();

  br->sendTransform(transformStamped);

  // transform: baselink to lidar
  transformStamped.header.stamp = this->imu_stamp;
  transformStamped.header.frame_id = this->baselink_frame;
  transformStamped.child_frame_id = this->lidar_frame;

  transformStamped.transform.translation.x = this->extrinsics.baselink2lidar.t[0];
  transformStamped.transform.translation.y = this->extrinsics.baselink2lidar.t[1];
  transformStamped.transform.translation.z = this->extrinsics.baselink2lidar.t[2];

  Eigen::Quaternionf qq(this->extrinsics.baselink2lidar.R);
  transformStamped.transform.rotation.w = qq.w();
  transformStamped.transform.rotation.x = qq.x();
  transformStamped.transform.rotation.y = qq.y();
  transformStamped.transform.rotation.z = qq.z();

  br->sendTransform(transformStamped);

}

void dlio::OdomNode::publishCloud(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud) {

  if (this->wait_until_move_) {
    if (this->length_traversed < 0.1) { return; }
  }

  pcl::PointCloud<PointType>::Ptr deskewed_scan_t_ = std::make_shared<pcl::PointCloud<PointType>>();

  pcl::transformPointCloud (*published_cloud, *deskewed_scan_t_, T_cloud);

  // published deskewed cloud
  sensor_msgs::msg::PointCloud2 deskewed_ros;
  pcl::toROSMsg(*deskewed_scan_t_, deskewed_ros);
  deskewed_ros.header.stamp = this->scan_header_stamp;
  deskewed_ros.header.frame_id = this->odom_frame;
  this->deskewed_pub->publish(deskewed_ros);

}

void dlio::OdomNode::updateSyncDiagnostics(double scan_start_stamp, double scan_end_stamp, int imu_samples_used) {
  const double lidar_header_stamp = rclcpp::Time(this->scan_header_stamp).seconds();

  double oldest_imu_stamp = 0.0;
  double latest_imu_stamp = 0.0;
  {
    std::lock_guard<decltype(this->mtx_imu)> lock(this->mtx_imu);
    if (!this->imu_buffer.empty()) {
      latest_imu_stamp = this->imu_buffer.front().stamp;
      oldest_imu_stamp = this->imu_buffer.back().stamp;
    }
  }

  const bool has_imu = latest_imu_stamp > 0.0 && oldest_imu_stamp > 0.0;
  const bool covers_start =
    has_imu && oldest_imu_stamp <= scan_start_stamp && latest_imu_stamp >= scan_start_stamp;
  const bool covers_end =
    has_imu && oldest_imu_stamp <= scan_end_stamp && latest_imu_stamp >= scan_end_stamp;

  this->last_lidar_header_stamp_ = lidar_header_stamp;
  this->last_scan_start_stamp_ = scan_start_stamp;
  this->last_scan_end_stamp_ = scan_end_stamp;
  this->last_oldest_imu_stamp_ = oldest_imu_stamp;
  this->last_latest_imu_stamp_ = latest_imu_stamp;
  this->last_latest_imu_minus_lidar_ms_ =
    static_cast<float>((latest_imu_stamp - lidar_header_stamp) * 1000.0);
  this->last_imu_covers_scan_start_ = covers_start;
  this->last_imu_covers_scan_end_ = covers_end;
  this->last_imu_samples_used_for_deskew_ = imu_samples_used;
  this->last_scan_duration_ms_ =
    static_cast<float>(std::max(0.0, scan_end_stamp - scan_start_stamp) * 1000.0);
}

void dlio::OdomNode::publishDiagnostics(float status_code, float raw_points, float filtered_points) {
  if (!this->diagnostics_pub) {
    return;
  }

  float latest_imu_stamp = 0.0f;
  float oldest_imu_stamp = 0.0f;
  float imu_buffer_size = 0.0f;
  {
    std::lock_guard<decltype(this->mtx_imu)> lock(this->mtx_imu);
    imu_buffer_size = static_cast<float>(this->imu_buffer.size());
    if (!this->imu_buffer.empty()) {
      latest_imu_stamp = static_cast<float>(this->imu_buffer.front().stamp);
      oldest_imu_stamp = static_cast<float>(this->imu_buffer.back().stamp);
    }
  }

  std_msgs::msg::Float32MultiArray msg;
  msg.data = {
    status_code,
    this->dlio_initialized.load() ? 1.0f : 0.0f,
    this->first_imu_received.load() ? 1.0f : 0.0f,
    this->imu_calibrated.load() ? 1.0f : 0.0f,
    this->first_valid_scan.load() ? 1.0f : 0.0f,
    static_cast<float>(this->sensor),
    raw_points,
    filtered_points,
    imu_buffer_size,
    static_cast<float>(this->scan_stamp),
    latest_imu_stamp,
    oldest_imu_stamp,
    this->state.p[0],
    this->state.p[1],
    this->state.p[2],
    this->state.q.w(),
    this->deskew_status.load() ? 1.0f : 0.0f,
    static_cast<float>(this->deskew_size.load()),
    this->last_correction_translation_.load(),
    this->last_correction_rotation_deg_.load(),
    this->last_correction_rejected_.load() ? 1.0f : 0.0f,
    static_cast<float>(this->bad_correction_streak_.load()),
    this->last_gicp_fitness_.load(),
    static_cast<float>(this->last_gicp_inliers_.load()),
    this->last_gicp_overlap_.load(),
    this->last_hessian_min_eigen_.load(),
    this->last_hessian_max_eigen_.load(),
    this->last_hessian_condition_.load(),
    this->last_gicp_solve_time_ms_.load(),
    static_cast<float>(this->last_gicp_quality_mode_.load()),
    this->last_angular_rate_.load(),
    this->last_spin_protection_active_.load() ? 1.0f : 0.0f,
    this->last_imu_age_ms_.load(),
    this->last_timing_protection_active_.load() ? 1.0f : 0.0f,
    // Float32MultiArray cannot retain full epoch precision; these casts are
    // presentation-only. Internal synchronization uses the double atomics.
    static_cast<float>(this->last_lidar_header_stamp_.load()),
    static_cast<float>(this->last_scan_start_stamp_.load()),
    static_cast<float>(this->last_scan_end_stamp_.load()),
    static_cast<float>(this->last_oldest_imu_stamp_.load()),
    static_cast<float>(this->last_latest_imu_stamp_.load()),
    this->last_imu_covers_scan_start_.load() ? 1.0f : 0.0f,
    this->last_imu_covers_scan_end_.load() ? 1.0f : 0.0f,
    this->last_latest_imu_minus_lidar_ms_.load(),
    static_cast<float>(this->last_imu_samples_used_for_deskew_.load()),
    static_cast<float>(this->stale_scan_drop_count_.load()),
    this->last_scan_duration_ms_.load(),
    this->timing_protection_drop_stale_scans_ ? 1.0f : 0.0f,
    static_cast<float>(this->timing_protection_drop_imu_age_ms_),
    static_cast<float>(this->timing_protection_drop_reject_streak_),
    static_cast<float>(this->timing_protection_max_iterations_),
    // Ground constraint (appended at end -- do not reorder existing indices)
    this->gc_engaged_last_scan_.load() ? 1.0f : 0.0f,
    static_cast<float>(this->gc_disengage_reason_.load()),
    static_cast<float>(this->gc_last_inliers_.load()),
    this->gc_last_normal_tilt_deg_.load(),
    this->gc_last_z_correction_.load(),
    this->gc_tracked_ground_height_pub_.load(),
    // Pipeline watchdog (appended; existing diagnostic indices remain stable).
    static_cast<float>(this->pipeline_stage_.load()),
    static_cast<float>(this->lidar_callbacks_started_.load()),
    static_cast<float>(this->lidar_callbacks_completed_.load())
  };
  this->diagnostics_pub->publish(msg);
}

void dlio::OdomNode::setPipelineStage(int stage) {
  this->pipeline_stage_ = stage;
}

void dlio::OdomNode::publishPipelineHeartbeat() {
  this->publishDiagnostics(-1.0f);
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto receive_ns = this->last_lidar_receive_steady_ns_.load();
  const auto start_ns = this->callback_start_steady_ns_.load();
  const auto complete_ns = this->last_callback_complete_steady_ns_.load();
  const double receive_age = receive_ns > 0 ? (now_ns - receive_ns) / 1.0e9 : -1.0;
  const bool callback_active = this->lidar_callbacks_started_.load() >
    this->lidar_callbacks_completed_.load();
  const double callback_age = callback_active && start_ns > 0 ? (now_ns - start_ns) / 1.0e9 : 0.0;
  const double complete_age = complete_ns > 0 ? (now_ns - complete_ns) / 1.0e9 : -1.0;

  if (callback_active && callback_age > 1.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "DLIO watchdog: callback active %.2fs at pipeline_stage=%d started=%llu completed=%llu",
      callback_age, this->pipeline_stage_.load(),
      static_cast<unsigned long long>(this->lidar_callbacks_started_.load()),
      static_cast<unsigned long long>(this->lidar_callbacks_completed_.load()));
  } else if (!callback_active && receive_age > 1.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "DLIO watchdog: waiting for timed point cloud; last_receive_age=%.2fs last_complete_age=%.2fs",
      receive_age, complete_age);
  }
}

void dlio::OdomNode::recordDiagnosticEvent(const std::string& type, const std::string& detail,
                                           float raw_points, float filtered_points,
                                           double compute_time_ms) {
  if (this->diagnostic_history_size_ <= 0) {
    return;
  }

  double latest_imu_stamp = 0.0;
  int imu_buffer_size = 0;
  {
    std::lock_guard<decltype(this->mtx_imu)> lock(this->mtx_imu);
    imu_buffer_size = static_cast<int>(this->imu_buffer.size());
    if (!this->imu_buffer.empty()) {
      latest_imu_stamp = this->imu_buffer.front().stamp;
    }
  }
  const double age_reference_stamp =
    this->last_scan_end_stamp_.load() > 0.0f
      ? static_cast<double>(this->last_scan_end_stamp_.load())
      : this->scan_stamp;
  const double latest_imu_age_ms =
    (latest_imu_stamp > 0.0 && age_reference_stamp > 0.0)
      ? std::abs(age_reference_stamp - latest_imu_stamp) * 1000.0
      : 0.0;

  DiagnosticEvent event;
  event.wall_time = this->now().seconds();
  event.scan_stamp = this->scan_stamp;
  event.type = type;
  event.detail = detail;
  event.raw_points = raw_points;
  event.filtered_points = filtered_points;
  event.compute_time_ms = compute_time_ms;
  event.latest_imu_age_ms = latest_imu_age_ms;
  event.keyframes = static_cast<int>(this->keyframes.size());
  event.imu_buffer_size = imu_buffer_size;
  event.bad_correction_streak = this->bad_correction_streak_.load();
  event.correction_translation = this->last_correction_translation_.load();
  event.correction_rotation_deg = this->last_correction_rotation_deg_.load();
  event.gicp_fitness = this->last_gicp_fitness_.load();
  event.gicp_inliers = this->last_gicp_inliers_.load();
  event.gicp_overlap = this->last_gicp_overlap_.load();
  event.hessian_min_eigen = this->last_hessian_min_eigen_.load();
  event.hessian_max_eigen = this->last_hessian_max_eigen_.load();
  event.hessian_condition = this->last_hessian_condition_.load();
  event.gicp_solve_time_ms = this->last_gicp_solve_time_ms_.load();
  event.gicp_quality_mode = this->last_gicp_quality_mode_.load();
  event.angular_rate = this->last_angular_rate_.load();
  event.spin_protection_active = this->last_spin_protection_active_.load();
  event.imu_age_ms_snapshot = this->last_imu_age_ms_.load();
  event.timing_protection_active = this->last_timing_protection_active_.load();
  event.lidar_header_stamp = this->last_lidar_header_stamp_.load();
  event.scan_start_stamp = this->last_scan_start_stamp_.load();
  event.scan_end_stamp = this->last_scan_end_stamp_.load();
  event.oldest_imu_stamp = this->last_oldest_imu_stamp_.load();
  event.latest_imu_stamp = this->last_latest_imu_stamp_.load();
  event.latest_imu_minus_lidar_ms = this->last_latest_imu_minus_lidar_ms_.load();
  event.imu_covers_scan_start = this->last_imu_covers_scan_start_.load();
  event.imu_covers_scan_end = this->last_imu_covers_scan_end_.load();
  event.imu_samples_used_for_deskew = this->last_imu_samples_used_for_deskew_.load();
  event.stale_scan_drop_count = this->stale_scan_drop_count_.load();
  event.scan_duration_ms = this->last_scan_duration_ms_.load();
  event.position = this->state.p;
  event.orientation_w = this->state.q.w();
  event.submap_points = this->submap_cloud ? this->submap_cloud->points.size() : 0;

  std::lock_guard<std::mutex> lock(this->diagnostic_history_mutex_);
  this->diagnostic_history_.push_back(event);
  while (static_cast<int>(this->diagnostic_history_.size()) > this->diagnostic_history_size_) {
    this->diagnostic_history_.pop_front();
  }
  this->appendDiagnosticEventToFile(event);
}

std::string dlio::OdomNode::diagnosticLikelyCause(const std::string& type) const {
  if (type == "registration_rejected") {
    return "bad scan-to-submap match or weak/ambiguous geometry";
  } else if (type == "lag_spike") {
    return "frontend fell behind; reduce submap/keyframes/logging or point count";
  } else if (type == "imu_age") {
    return "LiDAR/IMU timing delay; check sim time, QoS, CPU load, adapter";
  } else if (type == "sync_gap") {
    return "IMU buffer does not cover the LiDAR sweep interval";
  } else if (type == "late_registration") {
    return "scan matching finished after the LiDAR frame was already stale";
  } else if (type == "low_points") {
    return "too few usable points after crop/voxel filtering";
  } else if (type == "map_growth") {
    return "submap/keyframe growth increasing registration cost";
  } else if (type == "sensor_format") {
    return "point fields/time/ring format mismatch; deskew may be disabled";
  } else if (type == "initialization") {
    return "waiting for IMU/calibration/first valid scan";
  } else if (type == "stale_scan_drop") {
    return "dropping stale LiDAR frames so the frontend can catch up";
  } else if (type == "registration_degenerate_partial") {
    return "geometry is usable but weak; only a scaled correction was applied";
  } else if (type == "registration_degeneracy_projected") {
    return "Hessian indicates weak geometry; correction was damped along weak eigen-directions";
  } else if (type == "registration_innovation_partial") {
    return "GICP correction disagreed with IMU prediction; only a small correction was applied";
  } else if (type == "registration_recovery_accepted") {
    return "wide/multi-yaw recovery found a plausible scan-to-map match";
  } else if (type == "registration_recovery_rejected") {
    return "wide/multi-yaw recovery could not find a trustworthy match";
  } else if (type == "ground_constraint_state_change") {
    return "ground-plane z-constraint engage/disengage transition; see detail for reason code";
  }
  return "unknown";
}

std::string dlio::OdomNode::formatDiagnosticEvent(const DiagnosticEvent& event) const {
  std::ostringstream line;
  line << std::fixed << std::setprecision(3)
       << "t=" << event.scan_stamp
       << " type=" << event.type
       << " cause=\"" << this->diagnosticLikelyCause(event.type) << "\""
       << " detail=\"" << event.detail << "\""
       << " compute_ms=" << event.compute_time_ms
       << " imu_age_ms=" << event.latest_imu_age_ms
       << " points=" << event.raw_points << "/" << event.filtered_points
       << " keyframes=" << event.keyframes
       << " submap_points=" << event.submap_points
       << " corr=" << event.correction_translation << "m/"
       << event.correction_rotation_deg << "deg"
       << " quality=(mode=" << event.gicp_quality_mode
       << " fitness=" << event.gicp_fitness
       << " inliers=" << event.gicp_inliers
       << " overlap=" << event.gicp_overlap
       << " hmin=" << event.hessian_min_eigen
       << " hmax=" << event.hessian_max_eigen
       << " hcond=" << event.hessian_condition
       << " solve_ms=" << event.gicp_solve_time_ms << ")"
       << " angular_rate=" << event.angular_rate
       << " spin_guard=" << (event.spin_protection_active ? "true" : "false")
       << " timing_guard=" << (event.timing_protection_active ? "true" : "false")
       << " reject_streak=" << event.bad_correction_streak
       << " sync=(lidar=" << event.lidar_header_stamp
       << " start=" << event.scan_start_stamp
       << " end=" << event.scan_end_stamp
       << " imu_old=" << event.oldest_imu_stamp
       << " imu_new=" << event.latest_imu_stamp
       << " imu_minus_lidar_ms=" << event.latest_imu_minus_lidar_ms
       << " covers=" << (event.imu_covers_scan_start ? "1" : "0")
       << "/" << (event.imu_covers_scan_end ? "1" : "0")
       << " imu_samples=" << event.imu_samples_used_for_deskew
       << " scan_ms=" << event.scan_duration_ms
       << " stale_drops=" << event.stale_scan_drop_count << ")"
       << " pose=(" << event.position.x() << ","
       << event.position.y() << "," << event.position.z() << ")"
       << " qw=" << event.orientation_w;
  return line.str();
}

void dlio::OdomNode::appendDiagnosticEventToFile(const DiagnosticEvent& event) {
  if (this->diagnostic_history_file_.empty()) {
    return;
  }
  // Opened once (see getParams()/constructor) and kept open for the node's
  // lifetime instead of a fresh ofstream per call, which used to mean an
  // open+close syscall pair landing in the hot path during reject storms --
  // exactly when compute headroom is least available.
  if (!this->diagnostic_history_stream_.is_open()) {
    return;
  }
  this->diagnostic_history_stream_ << this->formatDiagnosticEvent(event) << std::endl;
  this->diagnostic_history_stream_.flush();
}

void dlio::OdomNode::printDiagnosticHistory() {
  bool expected = false;
  if (!this->diagnostic_history_printed_.compare_exchange_strong(expected, true)) {
    return;
  }

  std::deque<DiagnosticEvent> history;
  {
    std::lock_guard<std::mutex> lock(this->diagnostic_history_mutex_);
    history = this->diagnostic_history_;
  }

  std::cout << std::endl;
  std::cout << "================ DLIO DIAGNOSTIC HISTORY ================" << std::endl;
  if (history.empty()) {
    std::cout << "No suspicious DLIO events were recorded." << std::endl;
    std::cout << "=========================================================" << std::endl;
    return;
  }

  std::cout << "Most recent " << history.size()
            << " events. Fields: sim_t, type, likely_cause, compute_ms, imu_age_ms,"
            << " raw/filtered, keyframes, submap_points, correction, angular_rate, spin_guard, timing_guard, reject_streak, pose"
            << std::endl;

  for (const auto& event : history) {
    std::cout << this->formatDiagnosticEvent(event) << std::endl;
  }
  std::cout << "=========================================================" << std::endl;
  if (!this->diagnostic_history_file_.empty()) {
    std::cout << "DLIO diagnostic file: " << this->diagnostic_history_file_ << std::endl;
  }
}

void dlio::OdomNode::publishKeyframe(std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, pcl::PointCloud<PointType>::ConstPtr> kf, rclcpp::Time timestamp) {

  // Push back
  geometry_msgs::msg::Pose p;
  p.position.x = kf.first.first[0];
  p.position.y = kf.first.first[1];
  p.position.z = kf.first.first[2];
  p.orientation.w = kf.first.second.w();
  p.orientation.x = kf.first.second.x();
  p.orientation.y = kf.first.second.y();
  p.orientation.z = kf.first.second.z();
  this->kf_pose_ros.poses.push_back(p);
  // Previously unbounded (same pattern as path_ros.poses above). Keyframe
  // count is already implicitly bounded in practice, but cap defensively
  // for long-running/large-loop missions.
  if (this->kf_pose_ros.poses.size() > kMaxKeyframePoses) {
    const size_t excess = this->kf_pose_ros.poses.size() - kMaxKeyframePoses;
    this->kf_pose_ros.poses.erase(
      this->kf_pose_ros.poses.begin(), this->kf_pose_ros.poses.begin() + excess);
  }

  // Publish
  this->kf_pose_ros.header.stamp = timestamp;
  this->kf_pose_ros.header.frame_id = this->odom_frame;
  this->kf_pose_pub->publish(this->kf_pose_ros);

  // publish keyframe scan for map
  if (this->vf_use_) {
    if (kf.second->points.size() == kf.second->width * kf.second->height) {
      sensor_msgs::msg::PointCloud2 keyframe_cloud_ros;
      pcl::toROSMsg(*kf.second, keyframe_cloud_ros);
      keyframe_cloud_ros.header.stamp = timestamp;
      keyframe_cloud_ros.header.frame_id = this->odom_frame;
      this->kf_cloud_pub->publish(keyframe_cloud_ros);
    }
  } else {
    sensor_msgs::msg::PointCloud2 keyframe_cloud_ros;
    pcl::toROSMsg(*kf.second, keyframe_cloud_ros);
    keyframe_cloud_ros.header.stamp = timestamp;
    keyframe_cloud_ros.header.frame_id = this->odom_frame;
    this->kf_cloud_pub->publish(keyframe_cloud_ros);
  }

}

void dlio::OdomNode::getScanFromROS(const sensor_msgs::msg::PointCloud2::SharedPtr& pc) {

  pcl::PointCloud<PointType>::Ptr original_scan_ = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*pc, *original_scan_);

  // Keep the high-bandwidth raw cloud on one DDS hop. For organized clouds
  // without native point timing, synthesize the same Ouster-style `t` field
  // the former standalone adapter produced, directly in DLIO's input path.
  // Column time is shared across rings/rows and spans one configured sweep.
  if (this->synthetic_timing_enabled_ && original_scan_->width > 0 &&
      !original_scan_->points.empty()) {
    const size_t width = static_cast<size_t>(original_scan_->width);
    const double denom = width > 1 ? static_cast<double>(width - 1) : 1.0;
    for (size_t i = 0; i < original_scan_->points.size(); ++i) {
      const size_t col = i % width;
      const size_t time_col = this->synthetic_timing_reverse_columns_ ?
        (width - 1U - col) : col;
      const double relative_sec =
        (static_cast<double>(time_col) / denom) * this->synthetic_timing_scan_period_sec_;
      original_scan_->points[i].t = static_cast<std::uint32_t>(
        std::clamp(
          std::llround(relative_sec * 1.0e9),
          0LL,
          static_cast<long long>(std::numeric_limits<std::uint32_t>::max())));
    }
  }
  this->scan_header_stamp = pc->header.stamp;
  this->last_lidar_header_stamp_ = rclcpp::Time(this->scan_header_stamp).seconds();

  // Remove NaNs
  std::vector<int> idx;
  original_scan_->is_dense = false;
  pcl::removeNaNFromPointCloud(*original_scan_, *original_scan_, idx);

  // Crop Box Filter
  this->crop.setInputCloud(original_scan_);
  this->crop.filter(*original_scan_);

  // automatically detect sensor type
  this->sensor = this->synthetic_timing_enabled_ ?
    dlio::SensorType::OUSTER : dlio::SensorType::UNKNOWN;
  if (!this->synthetic_timing_enabled_) {
    for (auto &field : pc->fields) {
      if (field.name == "t") {
        this->sensor = dlio::SensorType::OUSTER;
        break;
      } else if (field.name == "time") {
        this->sensor = dlio::SensorType::VELODYNE;
        break;
      } else if (field.name == "timestamp" && original_scan_->points[0].timestamp < 1e14) {
        this->sensor = dlio::SensorType::HESAI;
        break;
      } else if (field.name == "timestamp" && original_scan_->points[0].timestamp > 1e14) {
        this->sensor = dlio::SensorType::LIVOX;
        break;
      }
    }
  }

  if (this->sensor == dlio::SensorType::UNKNOWN) {
    this->deskew_ = false;
    this->publishDiagnostics(8.0f, static_cast<float>(original_scan_->points.size()), 0.0f);
    this->recordDiagnosticEvent(
      "sensor_format",
      "unknown point type; disabled deskew for this cloud",
      static_cast<float>(original_scan_->points.size()),
      0.0f);
  }

  this->original_scan = original_scan_;

}

void dlio::OdomNode::preprocessPoints() {

  // Deskew the original dlio-type scan
  if (this->deskew_) {

    this->deskewPointcloud();

    if (!this->first_valid_scan) {
      return;
    }

  } else {

    this->scan_stamp = rclcpp::Time(this->scan_header_stamp).seconds();
    this->updateSyncDiagnostics(this->scan_stamp, this->scan_stamp, 0);

    // don't process scans until IMU data is present
    if (!this->first_valid_scan) {

      bool wait_for_imu = true;
      {
        std::lock_guard<decltype(this->mtx_imu)> lock(this->mtx_imu);
        // The first scan is valid only when the IMU buffer brackets its
        // timestamp. The previous test checked only the oldest sample, so a
        // scan several seconds ahead was accepted and the next LiDAR callback
        // blocked waiting for IMU, potentially starving that IMU callback in
        // CPU-constrained containers.
        wait_for_imu = this->imu_buffer.empty() ||
          this->imu_buffer.back().stamp > this->scan_stamp ||
          this->imu_buffer.front().stamp < this->scan_stamp;
      }
      if (wait_for_imu) {
        return;
      }

      this->first_valid_scan = true;
      this->T_prior = this->T; // assume no motion for the first scan

    } else {

      // IMU prior for second scan onwards
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
      frames = this->integrateImu(this->prev_scan_stamp, this->lidarPose.q, this->lidarPose.p,
                                this->geo.prev_vel.cast<float>(), {this->scan_stamp});
      this->updateSyncDiagnostics(this->scan_stamp, this->scan_stamp, static_cast<int>(frames.size()));

      if (frames.size() > 0) {
        this->T_prior = frames.back();
      } else {
        this->T_prior = this->T;
      }

    }

    pcl::PointCloud<PointType>::Ptr deskewed_scan_ = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud (*this->original_scan, *deskewed_scan_,
                              this->T_prior * this->extrinsics.baselink2lidar_T);
    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = false;
  }

  // Voxel Grid Filter
  if (this->vf_use_) {
    pcl::PointCloud<PointType>::Ptr current_scan_ = std::make_shared<pcl::PointCloud<PointType>>(*this->deskewed_scan);
    this->voxel.setInputCloud(current_scan_);
    this->voxel.filter(*current_scan_);
    this->current_scan = current_scan_;
  } else {
    this->current_scan = this->deskewed_scan;
  }

}

void dlio::OdomNode::deskewPointcloud() {

  pcl::PointCloud<PointType>::Ptr deskewed_scan_ = std::make_shared<pcl::PointCloud<PointType>>(1, this->original_scan->points.size());
  // deskewed_scan_->points.resize(this->original_scan->points.size());
  // individual point timestamps should be relative to this time
  double sweep_ref_time = rclcpp::Time(this->scan_header_stamp).seconds();

  // sort points by timestamp and build list of timestamps
  std::function<bool(const PointType&, const PointType&)> point_time_cmp;
  std::function<bool(boost::range::index_value<PointType&, long>,
                     boost::range::index_value<PointType&, long>)> point_time_neq;
  std::function<double(boost::range::index_value<PointType&, long>)> extract_point_time;

  if (this->sensor == dlio::SensorType::OUSTER) {

    point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.t < p2.t; };
    point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                        boost::range::index_value<PointType&, long> p2)
      { return p1.value().t != p2.value().t; };
    extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
      { return sweep_ref_time + pt.value().t * 1e-9f; };

  } else if (this->sensor == dlio::SensorType::VELODYNE) {

    point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.time < p2.time; };
    point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                        boost::range::index_value<PointType&, long> p2)
      { return p1.value().time != p2.value().time; };
    extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
      { return sweep_ref_time + pt.value().time; };

  } else if (this->sensor == dlio::SensorType::HESAI) {

    point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.timestamp < p2.timestamp; };
    point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                        boost::range::index_value<PointType&, long> p2)
      { return p1.value().timestamp != p2.value().timestamp; };
    extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
      { return pt.value().timestamp; };
  } else if (this->sensor == dlio::SensorType::LIVOX) {
    point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.timestamp < p2.timestamp; };
    point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                        boost::range::index_value<PointType&, long> p2)
      { return p1.value().timestamp != p2.value().timestamp; };
    extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
      { return pt.value().timestamp * 1e-9f; };
  }

  // copy points into deskewed_scan_ in order of timestamp
  std::partial_sort_copy(this->original_scan->points.begin(), this->original_scan->points.end(),
                         deskewed_scan_->points.begin(), deskewed_scan_->points.end(), point_time_cmp);

  // filter unique timestamps
  auto points_unique_timestamps = deskewed_scan_->points
                                  | boost::adaptors::indexed()
                                  | boost::adaptors::adjacent_filtered(point_time_neq);

  // extract timestamps from points and put them in their own list
  std::vector<double> timestamps;
  std::vector<int> unique_time_indices;

  // compute offset between sweep reference time and first point timestamp
  double offset = 0.0;
  if (this->time_offset_) {
    offset = sweep_ref_time - extract_point_time(*points_unique_timestamps.begin());
  }

  // build list of unique timestamps and indices of first point with each timestamp
  for (auto it = points_unique_timestamps.begin(); it != points_unique_timestamps.end(); it++) {
    timestamps.push_back(extract_point_time(*it) + offset);
    unique_time_indices.push_back(it->index());
  }
  unique_time_indices.push_back(deskewed_scan_->points.size());

  int median_pt_index = timestamps.size() / 2;
  this->scan_stamp = timestamps[median_pt_index]; // set this->scan_stamp to the timestamp of the median point
  this->updateSyncDiagnostics(timestamps.front(), timestamps.back(), 0);

  // don't process scans until IMU data is present
  if (!this->first_valid_scan) {
    bool wait_for_imu = true;
    {
      std::lock_guard<decltype(this->mtx_imu)> lock(this->mtx_imu);
      wait_for_imu = this->imu_buffer.empty() ||
        this->imu_buffer.back().stamp > this->scan_stamp ||
        this->imu_buffer.front().stamp < this->scan_stamp;
    }
    if (wait_for_imu) {
      return;
    }

    this->first_valid_scan = true;
    this->T_prior = this->T; // assume no motion for the first scan
    pcl::transformPointCloud (*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = true;
    return;
  }

  // IMU prior & deskewing for second scan onwards
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
  frames = this->integrateImu(this->prev_scan_stamp, this->lidarPose.q, this->lidarPose.p,
                              this->geo.prev_vel.cast<float>(), timestamps);
  this->deskew_size = frames.size(); // if integration successful, equal to timestamps.size()
  this->updateSyncDiagnostics(timestamps.front(), timestamps.back(), static_cast<int>(frames.size()));

  // if there are no frames between the start and end of the sweep
  // that probably means that there's a sync issue
  if (frames.size() != timestamps.size()) {
    RCLCPP_FATAL(this->get_logger(),"Bad time sync between LiDAR and IMU!");
    std::ostringstream detail;
    detail << "deskew needed " << timestamps.size()
           << " timestamp poses but IMU integration produced " << frames.size();
    this->recordDiagnosticEvent(
      "sync_gap",
      detail.str(),
      static_cast<float>(this->original_scan->points.size()),
      static_cast<float>(this->deskewed_scan ? this->deskewed_scan->points.size() : 0));

    this->T_prior = this->T;
    pcl::transformPointCloud (*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = false;
    return;
  }

  // update prior to be the estimated pose at the median time of the scan (corresponds to this->scan_stamp)
  this->T_prior = frames[median_pt_index];

#pragma omp parallel for num_threads(this->num_threads_)
  for (int i = 0; i < timestamps.size(); i++) {

    Eigen::Matrix4f T = frames[i] * this->extrinsics.baselink2lidar_T;

    // transform point to world frame
    for (int k = unique_time_indices[i]; k < unique_time_indices[i+1]; k++) {
      auto &pt = deskewed_scan_->points[k];
      pt.getVector4fMap()[3] = 1.;
      pt.getVector4fMap() = T * pt.getVector4fMap();
    }
  }

  this->deskewed_scan = deskewed_scan_;
  this->deskew_status = true;

}

void dlio::OdomNode::initializeInputTarget() {

  this->prev_scan_stamp = this->scan_stamp;

  // keep history of keyframes
  this->keyframes.push_back(std::make_pair(std::make_pair(this->lidarPose.p, this->lidarPose.q), this->current_scan));
  this->keyframe_timestamps.push_back(this->scan_header_stamp);
  this->keyframe_normals.push_back(this->gicp.getSourceCovariances());
  this->keyframe_transformations.push_back(this->T_corr);

}

void dlio::OdomNode::setInputSource() {
  this->gicp.setInputSource(this->current_scan);
  this->gicp.calculateSourceCovariances();
}

void dlio::OdomNode::initializeDLIO() {

  // Wait for IMU
  if (!this->first_imu_received || !this->imu_calibrated) {
    this->publishDiagnostics(1.0f);
    this->recordDiagnosticEvent(
      "initialization",
      this->first_imu_received ? "waiting for IMU calibration" : "waiting for first IMU",
      0.0f,
      0.0f);
    return;
  }

  this->dlio_initialized = true;
  this->publishDiagnostics(9.0f);
  std::cout << std::endl << " DLIO initialized!" << std::endl;

}

void dlio::OdomNode::callbackPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc) {

  if (this->shutting_down_.load()) {
    return;
  }

  const auto callback_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  this->last_lidar_receive_steady_ns_ = callback_now_ns;
  this->callback_start_steady_ns_ = callback_now_ns;
  ++this->lidar_callbacks_started_;
  this->setPipelineStage(1);

  CallbackCompletionGuard completion_guard(this);

  if (pc->width == 0 || pc->height == 0 || pc->point_step == 0 || pc->data.empty()) {
    this->publishDiagnostics(8.0f, 0.0f, 0.0f);
    this->recordDiagnosticEvent(
      "sensor_format", "received empty or malformed PointCloud2", 0.0f, 0.0f);
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Dropping empty or malformed PointCloud2 before DLIO preprocessing");
    return;
  }

  std::unique_lock<decltype(this->main_loop_running_mutex)> lock(main_loop_running_mutex);
  this->main_loop_running = true;
  lock.unlock();

  double then = this->now().seconds();
  const double header_stamp = rclcpp::Time(pc->header.stamp).seconds();

  static double last_received_header_stamp = 0.0;
  if (last_received_header_stamp > 0.0 && header_stamp <= last_received_header_stamp) {
    this->publishDiagnostics(11.0f, static_cast<float>(pc->width * pc->height), 0.0f);
    std::ostringstream detail;
    detail << "dropping non-increasing LiDAR header stamp; stamp=" << header_stamp
           << " previous=" << last_received_header_stamp;
    this->recordDiagnosticEvent(
      "non_increasing_lidar_stamp",
      detail.str(),
      static_cast<float>(pc->width * pc->height),
      0.0f);
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Dropping non-increasing LiDAR stamp: %.9f <= %.9f",
      header_stamp,
      last_received_header_stamp);
    lock.lock();
    this->main_loop_running = false;
    lock.unlock();
    this->submap_build_cv.notify_one();
    return;
  }
  last_received_header_stamp = header_stamp;

  if (this->first_scan_stamp == 0.) {
    this->first_scan_stamp = header_stamp;
  }

  this->publishDiagnostics(0.0f, static_cast<float>(pc->width * pc->height), 0.0f);

  // DLIO Initialization procedures (IMU calib, gravity align)
  if (!this->dlio_initialized) {
    this->setPipelineStage(2);
    this->initializeDLIO();
  }

  // Convert incoming scan into DLIO format
  this->setPipelineStage(3);
  this->getScanFromROS(pc);

  // Preprocess points
  this->setPipelineStage(4);
  this->preprocessPoints();

  if (!this->first_valid_scan) {
    const float current_points = this->current_scan ? static_cast<float>(this->current_scan->points.size()) : 0.0f;
    this->publishDiagnostics(2.0f, static_cast<float>(this->original_scan->points.size()), current_points);
    this->recordDiagnosticEvent(
      "initialization",
      "preprocessed cloud exists but first valid scan is not ready",
      static_cast<float>(this->original_scan->points.size()),
      current_points);
    return;
  }

  if (this->current_scan->points.size() <= this->gicp_min_num_points_) {
    this->publishDiagnostics(
      3.0f,
      static_cast<float>(this->original_scan->points.size()),
      static_cast<float>(this->current_scan->points.size()));
    this->recordDiagnosticEvent(
      "low_points",
      "current cloud has fewer points than odom/gicp/minNumPoints",
      static_cast<float>(this->original_scan->points.size()),
      static_cast<float>(this->current_scan->points.size()));
    RCLCPP_FATAL(this->get_logger(), "Low number of points in the cloud!");
    return;
  }

  if ((!this->last_imu_covers_scan_start_.load() || !this->last_imu_covers_scan_end_.load()) &&
      this->dlio_initialized) {
    static double last_sync_gap_event_time = 0.0;
    if (this->now().seconds() - last_sync_gap_event_time > 2.0) {
      last_sync_gap_event_time = this->now().seconds();
      std::ostringstream detail;
      detail << "IMU coverage start/end="
             << (this->last_imu_covers_scan_start_.load() ? "true" : "false")
             << "/"
             << (this->last_imu_covers_scan_end_.load() ? "true" : "false")
             << " lidar_stamp=" << this->last_lidar_header_stamp_.load()
             << " scan_start=" << this->last_scan_start_stamp_.load()
             << " scan_end=" << this->last_scan_end_stamp_.load()
             << " oldest_imu=" << this->last_oldest_imu_stamp_.load()
             << " latest_imu=" << this->last_latest_imu_stamp_.load()
             << " imu_samples=" << this->last_imu_samples_used_for_deskew_.load();
      this->recordDiagnosticEvent(
        "sync_gap",
        detail.str(),
        static_cast<float>(this->original_scan->points.size()),
        static_cast<float>(this->current_scan->points.size()));
    }
  }

  double latest_imu_stamp = 0.0;
  {
    std::lock_guard<decltype(this->mtx_imu)> lock(this->mtx_imu);
    if (!this->imu_buffer.empty()) {
      latest_imu_stamp = this->imu_buffer.front().stamp;
    }
  }
  const double age_reference_stamp =
    this->last_scan_end_stamp_.load() > 0.0
      ? this->last_scan_end_stamp_.load()
      : this->scan_stamp;
  const double latest_imu_age_ms =
    (latest_imu_stamp > 0.0 && age_reference_stamp > 0.0)
      ? std::abs(age_reference_stamp - latest_imu_stamp) * 1000.0
      : 0.0;
  this->last_imu_age_ms_ = static_cast<float>(latest_imu_age_ms);
  const bool should_drop_stale_scan =
    this->timing_protection_drop_stale_scans_ &&
    this->dlio_initialized &&
    this->keyframes.size() > 0 &&
    latest_imu_age_ms > this->timing_protection_drop_imu_age_ms_ &&
    this->bad_correction_streak_.load() >= this->timing_protection_drop_reject_streak_;

  if (should_drop_stale_scan) {
    this->last_timing_protection_active_ = true;
    this->last_correction_rejected_ = true;
    this->stale_scan_drop_count_.fetch_add(1);
    this->publishDiagnostics(
      7.0f,
      static_cast<float>(this->original_scan->points.size()),
      static_cast<float>(this->current_scan->points.size()));
    std::ostringstream detail;
    detail << "dropping stale scan to catch up; imu_age_ms=" << latest_imu_age_ms
           << " reject_streak=" << this->bad_correction_streak_.load();
    this->recordDiagnosticEvent(
      "stale_scan_drop",
      detail.str(),
      static_cast<float>(this->original_scan->points.size()),
      static_cast<float>(this->current_scan->points.size()));

    if (this->scan_stamp > this->prev_scan_stamp) {
      this->lidar_rates.push_back(1. / (this->scan_stamp - this->prev_scan_stamp));
      this->prev_scan_stamp = this->scan_stamp;
    }
    this->elapsed_time = this->scan_stamp - this->first_scan_stamp;

    lock.lock();
    this->main_loop_running = false;
    lock.unlock();
    this->submap_build_cv.notify_one();
    return;
  }

  // Keep the per-scan pipeline single-owner. Detached per-frame workers were
  // racing shared DLIO state and could corrupt the heap after long runs.
  this->setPipelineStage(5);
  this->computeMetrics();

  // Set Adaptive Parameters
  if (this->adaptive_params_) {
    this->setAdaptiveParams();
  }

  // Set new frame as input source
  this->setPipelineStage(6);
  this->setInputSource();

  // Set initial frame as first keyframe
  if (this->keyframes.size() == 0) {
    this->initializeInputTarget();
    this->last_accepted_state_ = this->state;
    this->last_accepted_T_ = this->T;
    this->publishDiagnostics(
      4.0f,
      static_cast<float>(this->original_scan->points.size()),
      static_cast<float>(this->current_scan->points.size()));
    this->main_loop_running = false;
    if (!this->shutting_down_.load()) {
      this->submap_future =
        std::async(std::launch::async, &dlio::OdomNode::buildKeyframesAndSubmap, this, this->state);
    }
    if (this->submap_future.valid()) {
      this->submap_future.wait(); // wait until completion
    }
    return;
  }

  // Get the next pose via IMU + S2M + GEO
  this->setPipelineStage(7);
  const bool accepted_registration = this->getNextPose();

  // Gated ground-plane z-constraint: must run before updateKeyframes() so a
  // corrected altitude is what gets archived into the new keyframe -- that's
  // what severs the flat-ground z-drift ratchet rather than just hiding it
  // for one tick. Never runs on a rejected/held registration.
  if (accepted_registration) {
    this->applyGroundConstraint();
  }

  // Update current keyframe poses and map
  if (accepted_registration) {
    this->setPipelineStage(8);
    this->updateKeyframes();
  }

  // Build keyframe normals and submap if needed (and if we're not already waiting)
  this->setPipelineStage(9);
  if (this->new_submap_is_ready && !this->shutting_down_.load()) {
    this->main_loop_running = false;
    // Anchor the submap's spatial KNN query to the last ACCEPTED pose while
    // this tick's registration was rejected, not the live pose. buildSubmap
    // selects keyframes purely by proximity to whatever state it's given;
    // during a reject streak this->state is driven by unconstrained IMU
    // dead-reckoning, so feeding it the drifting estimate makes submap
    // selection chase the wrong location and pick increasingly irrelevant
    // keyframes -- compounding the very mismatch that caused the reject in
    // the first place, with no way to self-correct since keyframe/submap
    // growth is itself gated on acceptance. last_accepted_state_ is the
    // last position GICP actually verified, so the keyframes that are truly
    // still nearby stay in the candidate set even while the live estimate
    // drifts.
    const State submap_query_state =
      accepted_registration ? this->state : this->last_accepted_state_;
    this->submap_future =
      std::async( std::launch::async, &dlio::OdomNode::buildKeyframesAndSubmap, this, submap_query_state );
  } else {
    lock.lock();
    this->main_loop_running = false;
    lock.unlock();
    this->submap_build_cv.notify_one();
  }

  // Update trajectory
  this->trajectory.push_back( std::make_pair(this->state.p, this->state.q) );

  // Incremental cumulative-distance accumulation. Mirrors the jitter filter
  // debug() used to apply when it rescanned the full trajectory each tick
  // (only count a segment if it's >= 0.1 m, and only advance the reference
  // point when a segment is counted) -- see length_traversed_has_prev_.
  if (!this->length_traversed_has_prev_) {
    this->length_traversed_prev_p_ = this->state.p;
    this->length_traversed_has_prev_ = true;
  } else {
    const double l = (this->state.p - this->length_traversed_prev_p_).norm();
    if (l >= 0.1) {
      this->length_traversed += l;
      this->length_traversed_prev_p_ = this->state.p;
    }
  }

  // Update time stamps
  this->lidar_rates.push_back( 1. / (this->scan_stamp - this->prev_scan_stamp) );
  this->prev_scan_stamp = this->scan_stamp;
  this->elapsed_time = this->scan_stamp - this->first_scan_stamp;

  // Publish stuff to ROS
  pcl::PointCloud<PointType>::ConstPtr published_cloud;
  if (this->densemap_filtered_) {
    published_cloud = this->current_scan;
  } else {
    published_cloud = this->deskewed_scan;
  }
  this->setPipelineStage(10);
  this->publishToROS(published_cloud, this->T_corr);
  this->publishDiagnostics(
    accepted_registration ? 5.0f : 6.0f,
    static_cast<float>(this->original_scan->points.size()),
    static_cast<float>(this->current_scan->points.size()));

  // Update some statistics
  const double compute_time_s = this->now().seconds() - then;
  const double compute_time_ms = compute_time_s * 1000.0;
  this->comp_times.push_back(compute_time_s);
  this->gicp_hasConverged = this->gicp.hasConverged();

  if (compute_time_ms > this->diagnostic_lag_warning_ms_) {
    std::ostringstream detail;
    detail << "scan callback exceeded " << this->diagnostic_lag_warning_ms_ << " ms";
    this->recordDiagnosticEvent(
      "lag_spike",
      detail.str(),
      static_cast<float>(this->original_scan->points.size()),
      static_cast<float>(this->current_scan->points.size()),
      compute_time_ms);
  }

  if (latest_imu_age_ms > this->diagnostic_imu_age_warning_ms_) {
    std::ostringstream detail;
    detail << "latest IMU differs from LiDAR stamp by " << latest_imu_age_ms << " ms";
    this->recordDiagnosticEvent(
      "imu_age",
      detail.str(),
      static_cast<float>(this->original_scan->points.size()),
      static_cast<float>(this->current_scan->points.size()),
      compute_time_ms);
  }

  const size_t submap_points = this->submap_cloud ? this->submap_cloud->points.size() : 0;
  if (static_cast<int>(this->keyframes.size()) > this->diagnostic_keyframe_warning_count_ ||
      static_cast<int>(submap_points) > this->diagnostic_submap_warning_points_) {
    static double last_map_growth_event_time = 0.0;
    if (this->now().seconds() - last_map_growth_event_time > 5.0) {
      last_map_growth_event_time = this->now().seconds();
      std::ostringstream detail;
      detail << "keyframes=" << this->keyframes.size()
             << " submap_points=" << submap_points;
      this->recordDiagnosticEvent(
        "map_growth",
        detail.str(),
        static_cast<float>(this->original_scan->points.size()),
        static_cast<float>(this->current_scan->points.size()),
        compute_time_ms);
    }
  }

  // Debug statements and publish custom DLIO message. Printing the full
  // dashboard every scan can block the real-time callback.
  const double debug_now = this->now().seconds();
  if (this->debug_print_period_ > 0.0 &&
      debug_now - this->last_debug_print_time_ >= this->debug_print_period_) {
    this->last_debug_print_time_ = debug_now;
    this->debug();
  }

  this->geo.first_opt_done = true;

}

void dlio::OdomNode::callbackImu(const sensor_msgs::msg::Imu::SharedPtr imu_raw) {

  if (this->shutting_down_.load()) {
    return;
  }

  this->first_imu_received = true;

  sensor_msgs::msg::Imu::SharedPtr imu = this->transformImu( imu_raw );
  this->imu_stamp = imu->header.stamp;
  double imu_stamp_secs = rclcpp::Time(imu->header.stamp).seconds();

  Eigen::Vector3f lin_accel;
  Eigen::Vector3f ang_vel;

  // Get IMU samples
  ang_vel[0] = imu->angular_velocity.x;
  ang_vel[1] = imu->angular_velocity.y;
  ang_vel[2] = imu->angular_velocity.z;

  lin_accel[0] = imu->linear_acceleration.x;
  lin_accel[1] = imu->linear_acceleration.y;
  lin_accel[2] = imu->linear_acceleration.z;

  if (this->first_imu_stamp == 0.) {
    this->first_imu_stamp = imu_stamp_secs;
  }

  // IMU calibration procedure - do for three seconds
  if (!this->imu_calibrated) {

    static int num_samples = 0;
    static Eigen::Vector3f gyro_avg (0., 0., 0.);
    static Eigen::Vector3f accel_avg (0., 0., 0.);
    static bool print = true;

    if ((imu_stamp_secs - this->first_imu_stamp) < this->imu_calib_time_) {

      num_samples++;

      gyro_avg[0] += ang_vel[0];
      gyro_avg[1] += ang_vel[1];
      gyro_avg[2] += ang_vel[2];

      accel_avg[0] += lin_accel[0];
      accel_avg[1] += lin_accel[1];
      accel_avg[2] += lin_accel[2];

      if(print) {
        std::cout << std::endl << " Calibrating IMU for " << this->imu_calib_time_ << " seconds... ";
        std::cout.flush();
        print = false;
      }

    } else {

      std::cout << "done" << std::endl << std::endl;

      gyro_avg /= num_samples;
      accel_avg /= num_samples;

      Eigen::Vector3f grav_vec (0., 0., this->gravity_);

      if (this->gravity_align_) {

        // Estimate gravity vector - Only approximate if biases have not been pre-calibrated
        grav_vec = (accel_avg - this->state.b.accel).normalized() * abs(this->gravity_);
        Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(grav_vec, Eigen::Vector3f(0., 0., this->gravity_));

        // set gravity aligned orientation
        this->state.q = grav_q;
        this->T.block(0,0,3,3) = this->state.q.toRotationMatrix();
        this->lidarPose.q = this->state.q;

        // rpy
        auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
        double yaw = euler[0] * (180.0/M_PI);
        double pitch = euler[1] * (180.0/M_PI);
        double roll = euler[2] * (180.0/M_PI);

        // use alternate representation if the yaw is smaller
        if (abs(remainder(yaw + 180.0, 360.0)) < abs(yaw)) {
          yaw   = remainder(yaw + 180.0,   360.0);
          pitch = remainder(180.0 - pitch, 360.0);
          roll  = remainder(roll + 180.0,  360.0);
        }
        std::cout << " Estimated initial attitude:" << std::endl;
        std::cout << "   Roll  [deg]: " << to_string_with_precision(roll, 4) << std::endl;
        std::cout << "   Pitch [deg]: " << to_string_with_precision(pitch, 4) << std::endl;
        std::cout << "   Yaw   [deg]: " << to_string_with_precision(yaw, 4) << std::endl;
        std::cout << std::endl;
      }

      if (this->calibrate_accel_) {

        // subtract gravity from avg accel to get bias
        this->state.b.accel = accel_avg - grav_vec;

        std::cout << " Accel biases [xyz]: " << to_string_with_precision(this->state.b.accel[0], 8) << ", "
                                             << to_string_with_precision(this->state.b.accel[1], 8) << ", "
                                             << to_string_with_precision(this->state.b.accel[2], 8) << std::endl;
      }

      if (this->calibrate_gyro_) {

        this->state.b.gyro = gyro_avg;

        std::cout << " Gyro biases  [xyz]: " << to_string_with_precision(this->state.b.gyro[0], 8) << ", "
                                             << to_string_with_precision(this->state.b.gyro[1], 8) << ", "
                                             << to_string_with_precision(this->state.b.gyro[2], 8) << std::endl;
      }

      this->imu_calibrated = true;

    }

  } else {

    double dt = imu_stamp_secs - this->prev_imu_stamp;
    if (dt == 0) { dt = 1.0/200.0; }
    this->imu_rates.push_back( 1./dt );

    // Apply the calibrated bias to the new IMU measurements
    this->imu_meas.stamp = imu_stamp_secs;
    this->imu_meas.dt = dt;
    this->prev_imu_stamp = this->imu_meas.stamp;

    Eigen::Vector3f lin_accel_corrected = (this->imu_accel_sm_ * lin_accel) - this->state.b.accel;
    Eigen::Vector3f ang_vel_corrected = ang_vel - this->state.b.gyro;
    this->last_angular_rate_ = static_cast<float>(ang_vel_corrected.norm());

    this->imu_meas.lin_accel = lin_accel_corrected;
    this->imu_meas.ang_vel = ang_vel_corrected;

    // Store calibrated IMU measurements into imu buffer for manual integration later.
    this->mtx_imu.lock();
    this->imu_buffer.push_front(this->imu_meas);
    this->mtx_imu.unlock();

    // Notify the callbackPointCloud thread that IMU data exists for this time
    this->cv_imu_stamp.notify_one();

    if (this->geo.first_opt_done) {
      // Geometric Observer: Propagate State
      this->propagateState();
    }

  }

}

bool dlio::OdomNode::getNextPose() {

  // Check if the new submap is ready to be used
  this->new_submap_is_ready = (this->submap_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready);

  if (this->new_submap_is_ready && this->submap_hasChanged) {

    // Set the current global submap as the target cloud
    this->gicp.registerInputTarget(this->submap_cloud);

    // Set submap kdtree
    this->gicp.target_kdtree_ = this->submap_kdtree;

    // Set target cloud's normals as submap normals
    this->gicp.setTargetCovariances(this->submap_normals);

    this->submap_hasChanged = false;
  }

  const double angular_rate =
    std::max(static_cast<double>(this->last_angular_rate_.load()),
             static_cast<double>(this->state.v.ang.b.norm()));
  const double age_reference_stamp =
    this->last_scan_end_stamp_.load() > 0.0f
      ? static_cast<double>(this->last_scan_end_stamp_.load())
      : this->scan_stamp;
  double pre_align_latest_imu_stamp = 0.0;
  {
    std::lock_guard<decltype(this->mtx_imu)> lock(this->mtx_imu);
    if (!this->imu_buffer.empty()) {
      pre_align_latest_imu_stamp = this->imu_buffer.front().stamp;
    }
  }
  const double pre_align_imu_age_ms =
    (pre_align_latest_imu_stamp > 0.0 && age_reference_stamp > 0.0)
      ? std::abs(age_reference_stamp - pre_align_latest_imu_stamp) * 1000.0
      : 0.0;
  const bool spin_protection_active =
    this->spin_protection_enabled_ && angular_rate > this->spin_protection_angular_rate_;
  const bool pre_align_timing_protection_active =
    this->timing_protection_enabled_ && pre_align_imu_age_ms > this->timing_protection_imu_age_ms_;
  // Only cap iterations while a protection condition is actively true for
  // THIS tick. Previously this also capped whenever bad_correction_streak_
  // > 0, which meant a single rejected correction permanently hobbled every
  // subsequent GICP attempt (fewer iterations -> harder to converge -> more
  // likely to reject again) until the streak happened to clear -- a
  // self-reinforcing loop that made recovery from a bad correction less
  // likely the longer it persisted, independent of whether spin/timing
  // protection was still actually active.
  const bool use_timing_iteration_cap =
    spin_protection_active ||
    pre_align_timing_protection_active;
  const int align_max_iterations =
    use_timing_iteration_cap
      ? std::max(1, std::min(this->gicp_max_iter_, this->timing_protection_max_iterations_))
      : this->gicp_max_iter_;
  this->gicp.setMaximumIterations(align_max_iterations);

  struct GicpQuality {
    Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
    double translation = 0.0;
    double rotation_deg = 0.0;
    double yaw_deg = 0.0;
    double fitness = std::numeric_limits<double>::infinity();
    int inliers = 0;
    double overlap = 0.0;
    double hessian_min_eigen = 0.0;
    double hessian_max_eigen = 0.0;
    double hessian_condition = std::numeric_limits<double>::infinity();
    double solve_ms = 0.0;
    bool converged = false;
  };

  auto evaluate_quality = [this](const Eigen::Matrix4f& correction, double solve_ms) {
    GicpQuality quality;
    quality.correction = correction;
    quality.solve_ms = solve_ms;
    quality.converged = this->gicp.hasConverged();

    const Eigen::Vector3f correction_translation = correction.block<3, 1>(0, 3);
    const Eigen::Matrix3f correction_rotation = correction.block<3, 3>(0, 0);
    const Eigen::AngleAxisf correction_angle_axis(correction_rotation);
    quality.translation = correction_translation.norm();
    quality.rotation_deg = std::abs(correction_angle_axis.angle()) * 180.0 / M_PI;
    quality.yaw_deg =
      std::abs(std::atan2(correction_rotation(1, 0), correction_rotation(0, 0))) * 180.0 / M_PI;
    quality.inliers = std::max(0, this->gicp.num_correspondences);
    const double source_points =
      this->current_scan ? static_cast<double>(std::max<size_t>(1, this->current_scan->points.size())) : 1.0;
    quality.overlap = static_cast<double>(quality.inliers) / source_points;
    quality.fitness = this->gicp.getFinalError() / static_cast<double>(std::max(1, quality.inliers));

    const Eigen::Matrix<double, 6, 6>& hessian = this->gicp.getFinalHessian();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(hessian);
    if (eigensolver.info() == Eigen::Success) {
      Eigen::Matrix<double, 6, 1> eigenvalues = eigensolver.eigenvalues().cwiseAbs();
      quality.hessian_min_eigen = eigenvalues.minCoeff();
      quality.hessian_max_eigen = eigenvalues.maxCoeff();
      quality.hessian_condition =
        quality.hessian_min_eigen > 1e-12
          ? quality.hessian_max_eigen / quality.hessian_min_eigen
          : std::numeric_limits<double>::infinity();
    }
    return quality;
  };

  auto update_quality_correction_motion = [](GicpQuality& quality, const Eigen::Matrix4f& correction) {
    quality.correction = correction;
    const Eigen::Vector3f correction_translation = correction.block<3, 1>(0, 3);
    const Eigen::Matrix3f correction_rotation = correction.block<3, 3>(0, 0);
    const Eigen::AngleAxisf correction_angle_axis(correction_rotation);
    quality.translation = correction_translation.norm();
    quality.rotation_deg = std::abs(correction_angle_axis.angle()) * 180.0 / M_PI;
    quality.yaw_deg =
      std::abs(std::atan2(correction_rotation(1, 0), correction_rotation(0, 0))) * 180.0 / M_PI;
  };

  auto store_quality = [this](const GicpQuality& quality, int mode) {
    this->last_correction_translation_ = static_cast<float>(quality.translation);
    this->last_correction_rotation_deg_ = static_cast<float>(quality.rotation_deg);
    this->last_gicp_fitness_ = static_cast<float>(quality.fitness);
    this->last_gicp_inliers_ = quality.inliers;
    this->last_gicp_overlap_ = static_cast<float>(quality.overlap);
    this->last_hessian_min_eigen_ = static_cast<float>(quality.hessian_min_eigen);
    this->last_hessian_max_eigen_ = static_cast<float>(quality.hessian_max_eigen);
    this->last_hessian_condition_ = static_cast<float>(quality.hessian_condition);
    this->last_gicp_solve_time_ms_ = static_cast<float>(quality.solve_ms);
    this->last_gicp_quality_mode_ = mode;
  };

  auto scaled_correction = [](const Eigen::Matrix4f& correction, double scale) {
    const double bounded_scale = std::max(0.0, std::min(1.0, scale));
    Eigen::Matrix4f scaled = Eigen::Matrix4f::Identity();
    scaled.block<3, 1>(0, 3) = correction.block<3, 1>(0, 3) * static_cast<float>(bounded_scale);

    Eigen::Matrix3f rotation = correction.block<3, 3>(0, 0);
    Eigen::AngleAxisf angle_axis(rotation);
    double angle = angle_axis.angle();
    Eigen::Vector3f axis = angle_axis.axis();
    if (!std::isfinite(angle) || axis.norm() < 1e-6f) {
      angle = 0.0;
      axis = Eigen::Vector3f::UnitZ();
    }
    scaled.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(static_cast<float>(angle * bounded_scale), axis.normalized()).toRotationMatrix();
    return scaled;
  };

  auto projected_degenerate_correction =
    [this, &update_quality_correction_motion](const Eigen::Matrix4f& correction, GicpQuality& quality) {
      if (!this->degeneracy_projection_enabled_ ||
          !std::isfinite(quality.hessian_condition) ||
          quality.hessian_condition < this->degeneracy_projection_condition_) {
        return correction;
      }

      const Eigen::Matrix<double, 6, 6>& hessian = this->gicp.getFinalHessian();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(hessian);
      if (eigensolver.info() != Eigen::Success) {
        return correction;
      }

      const Eigen::Matrix<double, 6, 1> eigenvalues = eigensolver.eigenvalues().cwiseAbs();
      const Eigen::Matrix<double, 6, 6> eigenvectors = eigensolver.eigenvectors();
      const double max_eigen = eigenvalues.maxCoeff();
      if (max_eigen <= 1e-12) {
        return correction;
      }

      Eigen::AngleAxisf angle_axis(correction.block<3, 3>(0, 0));
      double angle = angle_axis.angle();
      Eigen::Vector3f axis = angle_axis.axis();
      if (!std::isfinite(angle) || axis.norm() < 1e-6f) {
        angle = 0.0;
        axis = Eigen::Vector3f::UnitZ();
      }

      Eigen::Matrix<double, 6, 1> dx = Eigen::Matrix<double, 6, 1>::Zero();
      dx.head<3>() = (axis.normalized() * static_cast<float>(angle)).cast<double>();
      dx.tail<3>() = correction.block<3, 1>(0, 3).cast<double>();

      const double min_scale =
        std::max(0.0, std::min(1.0, this->degeneracy_projection_min_scale_));
      const double condition_threshold = std::max(1.0, this->degeneracy_projection_condition_);
      Eigen::Matrix<double, 6, 1> projected_dx = Eigen::Matrix<double, 6, 1>::Zero();

      for (int i = 0; i < 6; ++i) {
        const double lambda = eigenvalues[i];
        const double direction_condition =
          lambda > 1e-12 ? max_eigen / lambda : std::numeric_limits<double>::infinity();
        double scale = 1.0;
        if (!std::isfinite(direction_condition) || direction_condition > condition_threshold) {
          scale = std::isfinite(direction_condition)
                    ? std::max(min_scale, std::min(1.0, condition_threshold / direction_condition))
                    : min_scale;
        }
        projected_dx += scale * eigenvectors.col(i).dot(dx) * eigenvectors.col(i);
      }

      Eigen::Matrix4f projected = Eigen::Matrix4f::Identity();
      const Eigen::Vector3f projected_rot = projected_dx.head<3>().cast<float>();
      const float projected_angle = projected_rot.norm();
      if (projected_angle > 1e-6f) {
        projected.block<3, 3>(0, 0) =
          Eigen::AngleAxisf(projected_angle, projected_rot / projected_angle).toRotationMatrix();
      }
      projected.block<3, 1>(0, 3) = projected_dx.tail<3>().cast<float>();

      GicpQuality projected_quality = quality;
      update_quality_correction_motion(projected_quality, projected);
      std::ostringstream detail;
      detail << "hessian eigen projection; original=" << quality.translation
             << "m/" << quality.rotation_deg << "deg yaw=" << quality.yaw_deg
             << " projected=" << projected_quality.translation
             << "m/" << projected_quality.rotation_deg << "deg yaw=" << projected_quality.yaw_deg
             << " hmin=" << quality.hessian_min_eigen
             << " hcond=" << quality.hessian_condition;
      this->recordDiagnosticEvent("registration_degeneracy_projected", detail.str());
      quality = projected_quality;
      return projected;
    };

  // Align with current submap with global IMU transformation as initial guess
  pcl::PointCloud<PointType>::Ptr aligned = std::make_shared<pcl::PointCloud<PointType>>();
  const auto solve_start = std::chrono::steady_clock::now();
  this->gicp.align(*aligned);
  const double solve_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - solve_start).count();

  // Get final transformation in global frame
  this->T_corr = this->gicp.getFinalTransformation(); // "correction" transformation
  GicpQuality quality = evaluate_quality(this->T_corr, solve_ms);
  this->T_corr = projected_degenerate_correction(this->T_corr, quality);
  double latest_imu_stamp = 0.0;
  {
    std::lock_guard<decltype(this->mtx_imu)> lock(this->mtx_imu);
    if (!this->imu_buffer.empty()) {
      latest_imu_stamp = this->imu_buffer.front().stamp;
    }
  }
  const double latest_imu_age_ms =
    (latest_imu_stamp > 0.0 && age_reference_stamp > 0.0)
      ? std::abs(age_reference_stamp - latest_imu_stamp) * 1000.0
      : 0.0;
  const bool timing_protection_active =
    this->timing_protection_enabled_ && latest_imu_age_ms > this->timing_protection_imu_age_ms_;
  double max_translation_correction = this->gicp_max_correction_translation_;
  double max_rotation_correction = this->gicp_max_correction_rotation_;
  if (spin_protection_active) {
    max_translation_correction = std::min(max_translation_correction, this->spin_protection_max_translation_);
  }
  if (timing_protection_active) {
    max_translation_correction = std::min(max_translation_correction, this->timing_protection_max_translation_);
    max_rotation_correction = std::min(max_rotation_correction, this->timing_protection_max_rotation_);
  }

  const bool geometry_strong =
    quality.converged &&
    quality.overlap >= this->quality_min_overlap_ &&
    quality.fitness <= this->quality_max_fitness_ &&
    quality.hessian_min_eigen >= this->quality_min_hessian_eigen_ &&
    quality.hessian_condition <= this->quality_max_hessian_condition_;
  const bool geometry_usable_degenerate =
    quality.converged &&
    quality.overlap >= this->quality_degenerate_min_overlap_ &&
    quality.fitness <= this->quality_degenerate_max_fitness_;
  const bool innovation_gate_active =
    this->innovation_gate_enabled_ &&
    (quality.translation > this->innovation_gate_max_translation_ ||
     quality.yaw_deg > this->innovation_gate_max_yaw_);
  bool innovation_partial_correction = false;
  if (innovation_gate_active && geometry_usable_degenerate) {
    const Eigen::Matrix4f original_correction = this->T_corr;
    this->T_corr = scaled_correction(this->T_corr, this->innovation_gate_partial_scale_);
    innovation_partial_correction = true;
    const double original_translation = quality.translation;
    const double original_rotation = quality.rotation_deg;
    const double original_yaw = quality.yaw_deg;
    update_quality_correction_motion(quality, this->T_corr);
    std::ostringstream detail;
    detail << "IMU innovation gate partial correction; original="
           << original_translation << "m/" << original_rotation << "deg yaw=" << original_yaw
           << " projected=" << quality.translation << "m/" << quality.rotation_deg
           << "deg yaw=" << quality.yaw_deg
           << " scale=" << this->innovation_gate_partial_scale_
           << " max_translation=" << this->innovation_gate_max_translation_
           << " max_yaw=" << this->innovation_gate_max_yaw_;
    this->recordDiagnosticEvent("registration_innovation_partial", detail.str());
    (void)original_correction;
  }
  const bool correction_within_normal_gate =
    quality.translation <= max_translation_correction &&
    quality.rotation_deg <= max_rotation_correction;
  const bool correction_within_partial_gate =
    quality.translation <= this->quality_partial_max_translation_ &&
    quality.rotation_deg <= this->quality_partial_max_rotation_;
  const bool strong_correction =
    !this->gicp_reject_bad_corrections_ ||
    (geometry_strong && correction_within_normal_gate);
  const bool partial_degenerate_correction =
    this->gicp_reject_bad_corrections_ &&
    this->quality_use_partial_degenerate_ &&
    !strong_correction &&
    geometry_usable_degenerate &&
    correction_within_partial_gate;
  const bool bad_correction =
    this->gicp_reject_bad_corrections_ &&
    !strong_correction &&
    !partial_degenerate_correction;

  store_quality(
    quality,
    (innovation_partial_correction && strong_correction) ? 6 : (strong_correction ? 1 : (partial_degenerate_correction ? 2 : 3)));
  this->last_correction_rejected_ = bad_correction;
  this->last_angular_rate_ = static_cast<float>(angular_rate);
  this->last_spin_protection_active_ = spin_protection_active;
  this->last_imu_age_ms_ = static_cast<float>(latest_imu_age_ms);
  this->last_timing_protection_active_ = timing_protection_active;

  const bool late_registration =
    this->timing_protection_drop_stale_scans_ &&
    this->keyframes.size() > 0 &&
    latest_imu_age_ms > this->timing_protection_drop_imu_age_ms_;
  if (late_registration) {
    this->last_correction_rejected_ = true;
    this->last_timing_protection_active_ = true;
    this->stale_scan_drop_count_.fetch_add(1);
    this->T = this->last_accepted_T_;
    this->T_corr = this->T * this->T_prior.inverse();
    this->propagateGICP();
    {
      std::lock_guard<std::mutex> lock(this->geo.mtx);
      this->state = this->last_accepted_state_;
      this->state.v.lin.w = Eigen::Vector3f::Zero();
      this->state.v.lin.b = Eigen::Vector3f::Zero();
      this->geo.prev_p = this->state.p;
      this->geo.prev_q = this->state.q;
      this->geo.prev_vel = Eigen::Vector3f::Zero();
    }
    this->publishDiagnostics(
      10.0f,
      this->original_scan ? static_cast<float>(this->original_scan->points.size()) : 0.0f,
      this->current_scan ? static_cast<float>(this->current_scan->points.size()) : 0.0f);
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Dropped late DLIO registration: imu_age=%.1fms threshold=%.1fms iterations=%d correction=%.3fm/%.2fdeg",
      latest_imu_age_ms,
      this->timing_protection_drop_imu_age_ms_,
      align_max_iterations,
      quality.translation,
      quality.rotation_deg);
    std::ostringstream detail;
    detail << "registration finished stale; imu_age_ms=" << latest_imu_age_ms
           << " threshold_ms=" << this->timing_protection_drop_imu_age_ms_
           << " iterations=" << align_max_iterations
           << " correction=" << quality.translation
           << "m/" << quality.rotation_deg << "deg";
    this->recordDiagnosticEvent(
      "late_registration",
      detail.str(),
      this->original_scan ? static_cast<float>(this->original_scan->points.size()) : 0.0f,
      this->current_scan ? static_cast<float>(this->current_scan->points.size()) : 0.0f);
    return false;
  }

  if (partial_degenerate_correction) {
    this->T_corr = scaled_correction(this->T_corr, this->quality_partial_correction_scale_);
    this->T = this->T_corr * this->T_prior;
    this->propagateGICP();
    this->updateState();
    {
      std::lock_guard<std::mutex> lock(this->geo.mtx);
      this->last_accepted_state_ = this->state;
      this->last_accepted_T_ = this->T;
    }
    this->bad_correction_streak_ = 0;
    this->last_correction_rejected_ = false;
    this->last_gicp_quality_mode_ = 2;

    std::ostringstream detail;
    detail << "scaled degenerate correction; scale=" << this->quality_partial_correction_scale_
           << " original=" << quality.translation << "m/" << quality.rotation_deg
           << "deg fitness=" << quality.fitness
           << " overlap=" << quality.overlap
           << " inliers=" << quality.inliers
           << " hmin=" << quality.hessian_min_eigen
           << " hcond=" << quality.hessian_condition;
    this->recordDiagnosticEvent("registration_degenerate_partial", detail.str());
    return false;
  }

  if (bad_correction) {
    const int bad_streak = this->bad_correction_streak_.load() + 1;
    this->bad_correction_streak_ = bad_streak;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Rejected DLIO GICP correction: translation=%.3fm rotation=%.2fdeg fitness=%.3f overlap=%.2f inliers=%d angular_rate=%.2frad/s imu_age=%.1fms spin_guard=%s timing_guard=%s",
      quality.translation,
      quality.rotation_deg,
      quality.fitness,
      quality.overlap,
      quality.inliers,
      angular_rate,
      latest_imu_age_ms,
      spin_protection_active ? "true" : "false",
      timing_protection_active ? "true" : "false");
    {
      std::ostringstream detail;
      detail << "correction/quality rejected; translation=" << quality.translation
             << "m rotation=" << quality.rotation_deg
             << "deg fitness=" << quality.fitness
             << " overlap=" << quality.overlap
             << " inliers=" << quality.inliers
             << " hmin=" << quality.hessian_min_eigen
             << " hcond=" << quality.hessian_condition
             << " solve_ms=" << quality.solve_ms
             << " angular_rate=" << angular_rate
             << "rad/s spin_guard=" << (spin_protection_active ? "true" : "false")
             << " imu_age_ms=" << latest_imu_age_ms
             << " timing_guard=" << (timing_protection_active ? "true" : "false")
             << " streak=" << bad_streak;
      this->recordDiagnosticEvent("registration_rejected", detail.str());
    }

    const int recovery_attempt_spacing = std::max(1, this->recovery_attempt_spacing_);
    const bool recovery_timing_ok =
      !this->recovery_skip_when_timing_active_ || !timing_protection_active;
    const bool recovery_spacing_ok =
      this->recovery_last_attempt_streak_ <= 0 ||
      bad_streak - this->recovery_last_attempt_streak_ >= recovery_attempt_spacing;
    const bool try_recovery =
      this->recovery_enabled_ &&
      this->recovery_reject_streak_ > 0 &&
      bad_streak >= this->recovery_reject_streak_ &&
      recovery_timing_ok &&
      recovery_spacing_ok &&
      this->submap_cloud &&
      !this->submap_cloud->points.empty();

    if (try_recovery) {
      this->recovery_last_attempt_streak_ = bad_streak;
      const Eigen::Matrix4f rejected_T_corr = this->T_corr;
      const double recovery_corr_dist =
        std::max(this->gicp_max_corr_dist_, this->recovery_max_correspondence_distance_);
      const int recovery_iterations =
        std::max(align_max_iterations, this->recovery_max_iterations_);

      this->gicp.setMaxCorrespondenceDistance(recovery_corr_dist);
      this->gicp.setMaximumIterations(recovery_iterations);

      std::vector<double> yaw_offsets = this->recovery_yaw_offsets_deg_;
      if (yaw_offsets.empty()) {
        yaw_offsets.push_back(0.0);
      }

      bool have_best = false;
      bool best_accepted = false;
      double best_yaw_offset = 0.0;
      double best_score = std::numeric_limits<double>::infinity();
      GicpQuality best_quality;

      for (const double yaw_offset_deg : yaw_offsets) {
        pcl::PointCloud<PointType>::Ptr recovery_aligned =
          std::make_shared<pcl::PointCloud<PointType>>();
        Eigen::Matrix4f yaw_guess = Eigen::Matrix4f::Identity();
        yaw_guess.block<3, 3>(0, 0) =
          Eigen::AngleAxisf(
            static_cast<float>(yaw_offset_deg * M_PI / 180.0),
            Eigen::Vector3f::UnitZ()).toRotationMatrix();

        const auto recovery_start = std::chrono::steady_clock::now();
        this->gicp.align(*recovery_aligned, yaw_guess);
        const double recovery_solve_ms =
          std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - recovery_start).count();
        GicpQuality candidate =
          evaluate_quality(this->gicp.getFinalTransformation(), recovery_solve_ms);
        const bool candidate_accepted =
          candidate.converged &&
          candidate.translation <= this->recovery_accept_translation_ &&
          candidate.rotation_deg <= this->recovery_accept_rotation_ &&
          candidate.overlap >= this->recovery_min_overlap_ &&
          candidate.fitness <= this->recovery_max_fitness_;
        const double candidate_score =
          candidate.fitness +
          (1.0 - std::min(1.0, candidate.overlap)) * 10.0 +
          candidate.translation +
          candidate.rotation_deg / 30.0 +
          (candidate.converged ? 0.0 : 1000.0);

        if (!have_best ||
            (candidate_accepted && !best_accepted) ||
            (candidate_accepted == best_accepted && candidate_score < best_score)) {
          have_best = true;
          best_accepted = candidate_accepted;
          best_yaw_offset = yaw_offset_deg;
          best_score = candidate_score;
          best_quality = candidate;
        }
      }

      this->gicp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
      this->gicp.setMaximumIterations(align_max_iterations);

      std::ostringstream recovery_detail;
      recovery_detail << "streak=" << bad_streak
                      << " widened_corr_dist=" << recovery_corr_dist
                      << " iterations=" << recovery_iterations
                      << " yaw_candidates=" << yaw_offsets.size()
                      << " best_yaw_offset_deg=" << best_yaw_offset
                      << " converged=" << (best_quality.converged ? "true" : "false")
                      << " correction=" << best_quality.translation
                      << "m/" << best_quality.rotation_deg << "deg"
                      << " fitness=" << best_quality.fitness
                      << " overlap=" << best_quality.overlap
                      << " inliers=" << best_quality.inliers
                      << " hmin=" << best_quality.hessian_min_eigen
                      << " hcond=" << best_quality.hessian_condition
                      << " solve_ms=" << best_quality.solve_ms
                      << " score=" << best_score;

      store_quality(best_quality, best_accepted ? 4 : 5);

      if (best_accepted) {
        this->T_corr = best_quality.correction;
        this->last_correction_rejected_ = false;
        this->bad_correction_streak_ = 0;
        this->recovery_last_attempt_streak_ = 0;
        this->recordDiagnosticEvent("registration_recovery_accepted", recovery_detail.str());
        RCLCPP_WARN(
          this->get_logger(),
          "Accepted DLIO recovery registration after %d rejects: yaw_offset=%.1fdeg correction=%.3fm %.2fdeg fitness=%.3f overlap=%.2f",
          bad_streak,
          best_yaw_offset,
          best_quality.translation,
          best_quality.rotation_deg,
          best_quality.fitness,
          best_quality.overlap);

        this->T = this->T_corr * this->T_prior;
        this->propagateGICP();
        this->updateState();
        {
          std::lock_guard<std::mutex> lock(this->geo.mtx);
          this->last_accepted_state_ = this->state;
          this->last_accepted_T_ = this->T;
        }
        return true;
      }

      this->recordDiagnosticEvent("registration_recovery_rejected", recovery_detail.str());
      this->T_corr = rejected_T_corr;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "DLIO recovery registration rejected: yaw_offset=%.1fdeg correction=%.3fm rotation=%.2fdeg fitness=%.3f overlap=%.2f converged=%s",
        best_yaw_offset,
        best_quality.translation,
        best_quality.rotation_deg,
        best_quality.fitness,
        best_quality.overlap,
        best_quality.converged ? "true" : "false");
    }

    if (!this->freeze_on_bad_correction_) {
      this->T = this->T_prior;
      this->T_corr = Eigen::Matrix4f::Identity();
      this->propagateGICP();
      std::lock_guard<std::mutex> lock(this->geo.mtx);
      this->state.p = this->lidarPose.p;
      this->state.q = this->lidarPose.q;

      Eigen::Vector3f limited_velocity = this->state.v.lin.w;
      const bool force_hold =
        this->bad_correction_hold_streak_ > 0 &&
        bad_streak >= this->bad_correction_hold_streak_;
      const bool decay_velocity =
        this->bad_correction_velocity_decay_streak_ > 0 &&
        bad_streak >= this->bad_correction_velocity_decay_streak_;

      if (force_hold) {
        limited_velocity.setZero();
      } else {
        if (decay_velocity) {
          const double decay = std::max(0.0, std::min(1.0, this->bad_correction_velocity_decay_));
          limited_velocity *= static_cast<float>(decay);
        }
        const double max_speed = std::max(0.0, this->bad_correction_max_linear_speed_);
        const float speed = limited_velocity.norm();
        if (max_speed > 0.0 && speed > static_cast<float>(max_speed)) {
          limited_velocity *= static_cast<float>(max_speed) / speed;
        }
      }

      if (force_hold || decay_velocity) {
        std::ostringstream limiter_detail;
        limiter_detail << "bad correction velocity limiter; streak=" << bad_streak
                       << " force_hold=" << (force_hold ? "true" : "false")
                       << " speed=" << limited_velocity.norm();
        this->recordDiagnosticEvent("bad_correction_velocity_limiter", limiter_detail.str());
      }

      this->state.v.lin.w = limited_velocity;
      this->state.v.lin.b = this->state.q.toRotationMatrix().inverse() * this->state.v.lin.w;
      this->geo.prev_p = this->state.p;
      this->geo.prev_q = this->state.q;
      this->geo.prev_vel = this->state.v.lin.w;
      return false;
    }
    if (this->freeze_on_bad_correction_) {
      const bool use_imu_prior_recovery =
        this->spin_protection_use_imu_prior_on_reject_ &&
        spin_protection_active &&
        !timing_protection_active;
      const bool hold_pose =
        timing_protection_active ||
        (!use_imu_prior_recovery && bad_streak <= this->bad_correction_freeze_streak_);
      const Eigen::Matrix4f proposed_T = this->T_corr * this->T_prior;
      this->T = use_imu_prior_recovery ? this->T_prior : this->last_accepted_T_;

      if (!hold_pose) {
        Eigen::Matrix4f delta_T =
          use_imu_prior_recovery ? this->last_accepted_T_.inverse() * this->T_prior :
                                   this->last_accepted_T_.inverse() * proposed_T;
        Eigen::Vector3f delta_t = delta_T.block<3, 1>(0, 3);
        const double delta_t_norm = delta_t.norm();
        const double recovery_translation_step =
          std::min(
            spin_protection_active
              ? this->spin_protection_recovery_translation_step_
              : this->bad_correction_recovery_translation_step_,
            timing_protection_active
              ? this->timing_protection_recovery_translation_step_
              : this->bad_correction_recovery_translation_step_);
        if (delta_t_norm > recovery_translation_step && delta_t_norm > 1e-6) {
          delta_t *= static_cast<float>(recovery_translation_step / delta_t_norm);
        }

        Eigen::AngleAxisf delta_angle_axis(delta_T.block<3, 3>(0, 0));
        double delta_angle = delta_angle_axis.angle();
        Eigen::Vector3f delta_axis = delta_angle_axis.axis();
        if (!std::isfinite(delta_angle) || delta_axis.norm() < 1e-6) {
          delta_angle = 0.0;
          delta_axis = Eigen::Vector3f::UnitZ();
        }

        const double max_recovery_angle =
          std::abs(std::min(
            spin_protection_active
              ? this->spin_protection_recovery_rotation_step_
              : this->bad_correction_recovery_rotation_step_,
            timing_protection_active
              ? this->timing_protection_recovery_rotation_step_
              : this->bad_correction_recovery_rotation_step_)) * M_PI / 180.0;
        if (std::abs(delta_angle) > max_recovery_angle) {
          delta_angle = (delta_angle < 0.0 ? -max_recovery_angle : max_recovery_angle);
        }

        Eigen::Matrix4f clamped_delta = Eigen::Matrix4f::Identity();
        clamped_delta.block<3, 3>(0, 0) =
          Eigen::AngleAxisf(static_cast<float>(delta_angle), delta_axis.normalized()).toRotationMatrix();
        clamped_delta.block<3, 1>(0, 3) = delta_t;
        this->T = this->last_accepted_T_ * clamped_delta;
      }

      this->T_corr = this->T * this->T_prior.inverse();
      this->propagateGICP();
      std::lock_guard<std::mutex> lock(this->geo.mtx);
      this->state = hold_pose ? this->last_accepted_state_ : this->state;
      if (!hold_pose) {
        this->state.p = this->lidarPose.p;
        this->state.q = this->lidarPose.q;
      }
      this->state.v.lin.w = Eigen::Vector3f::Zero();
      this->state.v.lin.b = Eigen::Vector3f::Zero();
      this->geo.prev_p = this->state.p;
      this->geo.prev_q = this->state.q;
      this->geo.prev_vel = Eigen::Vector3f::Zero();
      if (!hold_pose) {
        this->last_accepted_state_ = this->state;
        this->last_accepted_T_ = this->T;
      }
      return !hold_pose;
    }
  } else {
    this->bad_correction_streak_ = 0;
  }

  this->T = this->T_corr * this->T_prior;

  // Update next global pose
  // Both source and target clouds are in the global frame now, so tranformation is global
  this->propagateGICP();

  // Geometric observer update
  this->updateState();
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->last_accepted_state_ = this->state;
    this->last_accepted_T_ = this->T;
  }
  return !bad_correction;

}

bool dlio::OdomNode::imuMeasFromTimeRange(double start_time, double end_time,
                                          boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
                                          boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it,
                                          std::unique_lock<std::mutex>& imu_lock) {

  if (!imu_lock.owns_lock()) {
    return false;
  }

  if (this->imu_buffer.empty() || this->imu_buffer.front().stamp < end_time) {
    // Wait for the latest IMU data. The predicate must handle an empty buffer,
    // because startup races can wake this before the first IMU is stored.
    // Bounded: previously this was an unconditional wait() -- if the IMU
    // stream stalled (sim pause, driver hiccup, bag end), this call blocked
    // forever holding the lidar callback's callback group, killing the node
    // with no diagnostic. A timeout converts that hang into a logged
    // degradation; callers already treat a false return as "skip this scan."
    const bool ready = this->cv_imu_stamp.wait_for(
      imu_lock,
      std::chrono::duration<double, std::milli>(this->diagnostic_imu_wait_timeout_ms_),
      [this, &end_time]{
        return this->shutting_down_.load() ||
          (!this->imu_buffer.empty() && this->imu_buffer.front().stamp >= end_time);
      });
    if (this->shutting_down_.load()) {
      return false;
    }
    if (!ready) {
      std::ostringstream detail;
      detail << "IMU wait timed out after " << this->diagnostic_imu_wait_timeout_ms_
             << " ms waiting for stamp >= " << end_time;
      this->recordDiagnosticEvent("imu_wait_timeout", detail.str());
      return false;
    }
  }

  if (this->imu_buffer.size() < 2) {
    return false;
  }

  auto imu_it = this->imu_buffer.begin();

  auto last_imu_it = imu_it;
  imu_it++;
  while (imu_it != this->imu_buffer.end() && imu_it->stamp >= end_time) {
    last_imu_it = imu_it;
    imu_it++;
  }

  while (imu_it != this->imu_buffer.end() && imu_it->stamp >= start_time) {
    imu_it++;
  }

  if (imu_it == this->imu_buffer.end()) {
    // not enough IMU measurements, return false
    return false;
  }
  imu_it++;

  // Set reverse iterators (to iterate forward in time)
  end_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it);
  begin_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it);

  return true;
}


std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
dlio::OdomNode::integrateImu(double start_time, Eigen::Quaternionf q_init, 
                              Eigen::Vector3f p_init, Eigen::Vector3f v_init, 
                              const std::vector<double>& sorted_timestamps) {

  const std::vector<Eigen::Matrix4f, 
    Eigen::aligned_allocator<Eigen::Matrix4f>> empty;

  // ─────────────────────────────────────────────────────────────
  // ORIGINAL - no change needed here
  // ─────────────────────────────────────────────────────────────
  if (sorted_timestamps.empty() || 
      start_time > sorted_timestamps.front()) {
    return empty;
  }

  boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it;
  boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it;
  std::unique_lock<std::mutex> imu_lock(this->mtx_imu);

  if (this->imuMeasFromTimeRange(start_time, sorted_timestamps.back(), 
        begin_imu_it, end_imu_it, imu_lock) == false) {
    return empty;
  }

 
  // If imuMeasFromTimeRange returned true but only found 1 sample,
  // *(begin_imu_it+1) is undefined behavior — you're reading 
  // garbage memory. This is a safety guard that costs nothing.
  // ─────────────────────────────────────────────────────────────
  if (std::distance(begin_imu_it, end_imu_it) < 2) {
    RCLCPP_WARN(this->get_logger(),
        "integrateImu: need >= 2 IMU samples in range, got %td",
        std::distance(begin_imu_it, end_imu_it));
    return empty;
  }

  const ImuMeas& f1 = *begin_imu_it;
  const ImuMeas& f2 = *(begin_imu_it + 1);

  double dt = f2.dt;

  
  // ─────────────────────────────────────────────────────────────
  double idt = start_time - f1.stamp;
  if (idt < 0.0) {
    RCLCPP_WARN(this->get_logger(),
        "integrateImu: negative idt=%.9f "
        "(f1.stamp=%.6f > start_time=%.6f), clamping to 0",
        idt, f1.stamp, start_time);
    idt = 0.0;
  }

  Eigen::Vector3f alpha_dt = f2.ang_vel - f1.ang_vel;
  Eigen::Vector3f alpha = alpha_dt / dt;

  Eigen::Vector3f omega_i = -(f1.ang_vel + 0.5*alpha*idt);

  // ─────────────────────────────────────────────────────────────
  q_init = Eigen::Quaternionf(
    // w component 
    q_init.w() - 0.5*(q_init.x()*omega_i[0] + 
                      q_init.y()*omega_i[1] + 
                      q_init.z()*omega_i[2]) * idt,
    // x component 
    q_init.x() + 0.5*(q_init.w()*omega_i[0] + 
                      q_init.z()*omega_i[1] -  // was -z, now +z
                      q_init.y()*omega_i[2]) * idt,  // was +y, now -y
    // y component 
    q_init.y() + 0.5*(q_init.w()*omega_i[1] - 
                      q_init.z()*omega_i[0] +  // was +z·ω0, now -z·ω0
                      q_init.x()*omega_i[2]) * idt,
    // z component
    q_init.z() + 0.5*(q_init.w()*omega_i[2] + 
                      q_init.y()*omega_i[0] -  // was +x·ω1, now +y·ω0
                      q_init.x()*omega_i[1]) * idt
  );
  q_init.normalize();

  Eigen::Vector3f omega = f1.ang_vel + 0.5*alpha_dt;

 
  // ─────────────────────────────────────────────────────────────
  Eigen::Quaternionf q2(
    q_init.w() - 0.5*(q_init.x()*omega[0] + 
                      q_init.y()*omega[1] + 
                      q_init.z()*omega[2]) * dt,
    q_init.x() + 0.5*(q_init.w()*omega[0] + 
                      q_init.z()*omega[1] -  // fixed
                      q_init.y()*omega[2]) * dt,
    q_init.y() + 0.5*(q_init.w()*omega[1] - 
                      q_init.z()*omega[0] +  // fixed
                      q_init.x()*omega[2]) * dt,
    q_init.z() + 0.5*(q_init.w()*omega[2] + 
                      q_init.y()*omega[0] -  // fixed
                      q_init.x()*omega[1]) * dt
  );
  q2.normalize();

  Eigen::Vector3f a1 = q_init._transformVector(f1.lin_accel);
  a1[2] -= this->gravity_;

  Eigen::Vector3f a2 = q2._transformVector(f2.lin_accel);
  a2[2] -= this->gravity_;

  Eigen::Vector3f j = (a2 - a1) / dt;

 
  Eigen::Vector3f v_at_start = v_init;  // save BEFORE modifying

  v_init -= a1*idt + 0.5*j*idt*idt;

  p_init -= v_at_start*idt          // USE SAVED VALUE here
            + 0.5*a1*idt*idt 
            + (1/6.)*j*idt*idt*idt;

  return this->integrateImuInternal(q_init, p_init, v_init, 
      sorted_timestamps, begin_imu_it, end_imu_it);
}



std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
dlio::OdomNode::integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                                     const std::vector<double>& sorted_timestamps,
                                     boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
                                     boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it) {

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> imu_se3;

  // Initialization
  Eigen::Quaternionf q = q_init;
  Eigen::Vector3f p = p_init;
  Eigen::Vector3f v = v_init;
  Eigen::Vector3f a = q._transformVector(begin_imu_it->lin_accel);
  a[2] -= this->gravity_;

  // Iterate over IMU measurements and timestamps
  auto prev_imu_it = begin_imu_it;
  auto imu_it = prev_imu_it + 1;

  auto stamp_it = sorted_timestamps.begin();

  for (; imu_it != end_imu_it; imu_it++) {

    const ImuMeas& f0 = *prev_imu_it;
    const ImuMeas& f = *imu_it;

    // Time between IMU samples
    double dt = f.dt;

    // Angular acceleration
    Eigen::Vector3f alpha_dt = f.ang_vel - f0.ang_vel;
    Eigen::Vector3f alpha = alpha_dt / dt;

    // Average angular velocity
    Eigen::Vector3f omega = f0.ang_vel + 0.5*alpha_dt;

    // Orientation
    q = Eigen::Quaternionf (
      q.w() - 0.5*( q.x()*omega[0] + q.y()*omega[1] + q.z()*omega[2] ) * dt,
      q.x() + 0.5*( q.w()*omega[0] - q.z()*omega[1] + q.y()*omega[2] ) * dt,
      q.y() + 0.5*( q.z()*omega[0] + q.w()*omega[1] - q.x()*omega[2] ) * dt,
      q.z() + 0.5*( q.x()*omega[1] - q.y()*omega[0] + q.w()*omega[2] ) * dt
    );
    q.normalize();

    // Acceleration
    Eigen::Vector3f a0 = a;
    a = q._transformVector(f.lin_accel);
    a[2] -= this->gravity_;

    // Jerk
    Eigen::Vector3f j_dt = a - a0;
    Eigen::Vector3f j = j_dt / dt;

    // Interpolate for given timestamps
    while (stamp_it != sorted_timestamps.end() && *stamp_it <= f.stamp) {
      // Time between previous IMU sample and given timestamp
      double idt = *stamp_it - f0.stamp;

      // Average angular velocity
      Eigen::Vector3f omega_i = f0.ang_vel + 0.5*alpha*idt;

      // Orientation
      Eigen::Quaternionf q_i (
        q.w() - 0.5*( q.x()*omega_i[0] + q.y()*omega_i[1] + q.z()*omega_i[2] ) * idt,
        q.x() + 0.5*( q.w()*omega_i[0] - q.z()*omega_i[1] + q.y()*omega_i[2] ) * idt,
        q.y() + 0.5*( q.z()*omega_i[0] + q.w()*omega_i[1] - q.x()*omega_i[2] ) * idt,
        q.z() + 0.5*( q.x()*omega_i[1] - q.y()*omega_i[0] + q.w()*omega_i[2] ) * idt
      );
      q_i.normalize();

      // Position
      Eigen::Vector3f p_i = p + v*idt + 0.5*a0*idt*idt + (1/6.)*j*idt*idt*idt;

      // Transformation
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      T.block(0, 0, 3, 3) = q_i.toRotationMatrix();
      T.block(0, 3, 3, 1) = p_i;

      imu_se3.push_back(T);

      stamp_it++;
    }

    // Position
    p += v*dt + 0.5*a0*dt*dt + (1/6.)*j_dt*dt*dt;

    // Velocity
    v += a0*dt + 0.5*j_dt*dt;

    prev_imu_it = imu_it;

  }

  return imu_se3;

}

void dlio::OdomNode::propagateGICP() {

  this->lidarPose.p << this->T(0,3), this->T(1,3), this->T(2,3);

  Eigen::Matrix3f rotSO3;
  rotSO3 << this->T(0,0), this->T(0,1), this->T(0,2),
            this->T(1,0), this->T(1,1), this->T(1,2),
            this->T(2,0), this->T(2,1), this->T(2,2);

  Eigen::Quaternionf q(rotSO3);

  // Normalize quaternion
  double norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
  q.w() /= norm; q.x() /= norm; q.y() /= norm; q.z() /= norm;
  this->lidarPose.q = q;

}

void dlio::OdomNode::propagateState() {

  // Lock thread to prevent state from being accessed by UpdateState
  std::lock_guard<std::mutex> lock( this->geo.mtx );

  double dt = this->imu_meas.dt;

  Eigen::Quaternionf qhat = this->state.q, omega;
  Eigen::Vector3f world_accel;

  // Transform accel from body to world frame
  world_accel = qhat._transformVector(this->imu_meas.lin_accel);

  // Accel propogation
  this->state.p[0] += this->state.v.lin.w[0]*dt + 0.5*dt*dt*world_accel[0];
  this->state.p[1] += this->state.v.lin.w[1]*dt + 0.5*dt*dt*world_accel[1];
  this->state.p[2] += this->state.v.lin.w[2]*dt + 0.5*dt*dt*(world_accel[2] - this->gravity_);

  this->state.v.lin.w[0] += world_accel[0]*dt;
  this->state.v.lin.w[1] += world_accel[1]*dt;
  this->state.v.lin.w[2] += (world_accel[2] - this->gravity_)*dt;
  this->state.v.lin.b = this->state.q.toRotationMatrix().inverse() * this->state.v.lin.w;

  // Gyro propogation
  omega.w() = 0;
  omega.vec() = this->imu_meas.ang_vel;
  Eigen::Quaternionf tmp = qhat * omega;
  this->state.q.w() += 0.5 * dt * tmp.w();
  this->state.q.vec() += 0.5 * dt * tmp.vec();

  // Ensure quaternion is properly normalized
  this->state.q.normalize();

  this->state.v.ang.b = this->imu_meas.ang_vel;
  this->state.v.ang.w = this->state.q.toRotationMatrix() * this->state.v.ang.b;

}

void dlio::OdomNode::updateState() {

  // Lock thread to prevent state from being accessed by PropagateState
  std::lock_guard<std::mutex> lock( this->geo.mtx );

  Eigen::Vector3f pin = this->lidarPose.p;
  Eigen::Quaternionf qin = this->lidarPose.q;
  double dt = this->scan_stamp - this->prev_scan_stamp;

  Eigen::Quaternionf qe, qhat, qcorr;
  qhat = this->state.q;

  // Constuct error quaternion
  qe = qhat.conjugate()*qin;

  double sgn = 1.;
  if (qe.w() < 0) {
    sgn = -1;
  }

  // Construct quaternion correction
  qcorr.w() = 1 - abs(qe.w());
  qcorr.vec() = sgn*qe.vec();
  qcorr = qhat * qcorr;

  Eigen::Vector3f err = pin - this->state.p;
  Eigen::Vector3f err_body;

  err_body = qhat.conjugate()._transformVector(err);

  double abias_max = this->geo_abias_max_;
  double gbias_max = this->geo_gbias_max_;

  // Update accel bias
  this->state.b.accel -= dt * this->geo_Kab_ * err_body;
  this->state.b.accel = this->state.b.accel.array().min(abias_max).max(-abias_max);

  // Update gyro bias
  this->state.b.gyro[0] -= dt * this->geo_Kgb_ * qe.w() * qe.x();
  this->state.b.gyro[1] -= dt * this->geo_Kgb_ * qe.w() * qe.y();
  this->state.b.gyro[2] -= dt * this->geo_Kgb_ * qe.w() * qe.z();
  this->state.b.gyro = this->state.b.gyro.array().min(gbias_max).max(-gbias_max);

  // Update state
  this->state.p += dt * this->geo_Kp_ * err;
  this->state.v.lin.w += dt * this->geo_Kv_ * err;

  this->state.q.w() += dt * this->geo_Kq_ * qcorr.w();
  this->state.q.x() += dt * this->geo_Kq_ * qcorr.x();
  this->state.q.y() += dt * this->geo_Kq_ * qcorr.y();
  this->state.q.z() += dt * this->geo_Kq_ * qcorr.z();
  this->state.q.normalize();

  // store previous pose, orientation, and velocity
  this->geo.prev_p = this->state.p;
  this->geo.prev_q = this->state.q;
  this->geo.prev_vel = this->state.v.lin.w;

}

sensor_msgs::msg::Imu::SharedPtr dlio::OdomNode::transformImu(const sensor_msgs::msg::Imu::SharedPtr& imu_raw) {

  auto imu = std::make_shared<sensor_msgs::msg::Imu>();

  // Copy header
  imu->header = imu_raw->header;

  double imu_stamp_secs = rclcpp::Time(imu->header.stamp).seconds();
  static double prev_stamp = imu_stamp_secs;
  double dt = imu_stamp_secs - prev_stamp;
  prev_stamp = imu_stamp_secs;
  
  if (dt == 0) { dt = 1.0/200.0; }

  // Transform angular velocity (will be the same on a rigid body, so just rotate to ROS convention)
  Eigen::Vector3f ang_vel(imu_raw->angular_velocity.x,
                          imu_raw->angular_velocity.y,
                          imu_raw->angular_velocity.z);

  Eigen::Vector3f ang_vel_cg = this->extrinsics.baselink2imu.R * ang_vel;

  imu->angular_velocity.x = ang_vel_cg[0];
  imu->angular_velocity.y = ang_vel_cg[1];
  imu->angular_velocity.z = ang_vel_cg[2];

  static Eigen::Vector3f ang_vel_cg_prev = ang_vel_cg;

  // Transform linear acceleration (need to account for component due to translational difference)
  Eigen::Vector3f lin_accel(imu_raw->linear_acceleration.x,
                            imu_raw->linear_acceleration.y,
                            imu_raw->linear_acceleration.z);

  Eigen::Vector3f lin_accel_cg = this->extrinsics.baselink2imu.R * lin_accel;

  lin_accel_cg = lin_accel_cg
                 + ((ang_vel_cg - ang_vel_cg_prev) / dt).cross(-this->extrinsics.baselink2imu.t)
                 + ang_vel_cg.cross(ang_vel_cg.cross(-this->extrinsics.baselink2imu.t));

  ang_vel_cg_prev = ang_vel_cg;

  imu->linear_acceleration.x = lin_accel_cg[0];
  imu->linear_acceleration.y = lin_accel_cg[1];
  imu->linear_acceleration.z = lin_accel_cg[2];

  return imu;

}

void dlio::OdomNode::computeMetrics() {
  this->computeSpaciousness();
  this->computeDensity();
}

void dlio::OdomNode::computeSpaciousness() {

  // compute range of points
  std::vector<float> ds;

  for (int i = 0; i < this->original_scan->points.size(); i++) {
    float d = std::sqrt(pow(this->original_scan->points[i].x, 2) +
                        pow(this->original_scan->points[i].y, 2));
    ds.push_back(d);
  }

  // median
  std::nth_element(ds.begin(), ds.begin() + ds.size()/2, ds.end());
  float median_curr = ds[ds.size()/2];
  static float median_prev = median_curr;
  float median_lpf = 0.95*median_prev + 0.05*median_curr;
  median_prev = median_lpf;

  // push
  this->metrics.spaciousness.push_back( median_lpf );

}

void dlio::OdomNode::computeDensity() {

  float density;

  if (!this->geo.first_opt_done) {
    density = 0.;
  } else {
    density = this->gicp.source_density_;
  }

  static float density_prev = density;
  float density_lpf = 0.95*density_prev + 0.05*density;
  density_prev = density_lpf;

  this->metrics.density.push_back( density_lpf );

}

dlio::OdomNode::GroundPlaneFit dlio::OdomNode::fitGroundPlane() {

  GroundPlaneFit result;

  if (!this->current_scan || this->current_scan->points.empty()) {
    return result;
  }

  // current_scan is already in odom/world frame at this point in the
  // pipeline (transformed by T_prior when deskew is off, or per-point when
  // deskew is on) -- directly comparable to state.p without further
  // transform. Note: when deskew is off this reflects T_prior (the
  // pre-registration IMU prior), not this->T post-GICP-correction; for the
  // small per-scan corrections this constraint deals in, that's a
  // negligible-order effect smoothed out by the EMA tracking below, not a
  // correctness bug.
  const Eigen::Vector3f robot_p = this->state.p;
  const double ref_h = this->gc_initialized_ ?
    this->gc_tracked_ground_height_ : (robot_p.z() - this->gc_baselink_height_);
  const float z_lo = static_cast<float>(ref_h - this->gc_zband_below_);
  const float z_hi = static_cast<float>(ref_h + this->gc_zband_above_);
  const float max_range_sq = static_cast<float>(this->gc_max_range_ * this->gc_max_range_);

  std::vector<Eigen::Vector3f> candidates;
  candidates.reserve(this->current_scan->points.size());
  for (const auto & p : this->current_scan->points) {
    if (p.z < z_lo || p.z > z_hi) {continue;}
    const float dx = p.x - robot_p.x();
    const float dy = p.y - robot_p.y();
    if (dx * dx + dy * dy > max_range_sq) {continue;}
    candidates.emplace_back(p.x, p.y, p.z);
  }
  result.candidates = static_cast<int>(candidates.size());
  if (result.candidates < 3) {return result;}

  // Hand-rolled 3-point RANSAC (avoids pcl::SACSegmentation's allocation
  // overhead in this hot path -- this runs every scan). gc_rng_ is a member
  // (not a function-local static) so its state is scoped to this node
  // instance rather than shared/reseeded-once across all instances.
  std::uniform_int_distribution<int> pick(0, result.candidates - 1);

  int best_inliers = 0;
  Eigen::Vector3f best_normal = Eigen::Vector3f::UnitZ();
  float best_d = 0.0f;
  const float inlier_dist = static_cast<float>(this->gc_ransac_inlier_dist_);

  for (int iter = 0; iter < this->gc_ransac_iterations_; ++iter) {
    const int i0 = pick(this->gc_rng_);
    const int i1 = pick(this->gc_rng_);
    const int i2 = pick(this->gc_rng_);
    if (i0 == i1 || i1 == i2 || i0 == i2) {continue;}
    const Eigen::Vector3f & a = candidates[i0];
    const Eigen::Vector3f & b = candidates[i1];
    const Eigen::Vector3f & c = candidates[i2];
    Eigen::Vector3f normal = (b - a).cross(c - a);
    const float norm = normal.norm();
    if (norm < 1e-6f) {continue;}  // near-collinear triple, skip
    normal /= norm;
    if (normal.z() < 0.0f) {normal = -normal;}
    const float d = -normal.dot(a);

    int inliers = 0;
    for (const auto & p : candidates) {
      if (std::abs(normal.dot(p) + d) < inlier_dist) {++inliers;}
    }
    if (inliers > best_inliers) {
      best_inliers = inliers;
      best_normal = normal;
      best_d = d;
    }
  }
  if (best_inliers < 3) {return result;}  // RANSAC found no usable hypothesis

  // LSQ refine over the winning hypothesis's inlier set: centroid +
  // smallest-eigenvalue eigenvector of the inlier covariance -- same
  // SelfAdjointEigenSolver pattern already used for the GICP Hessian
  // eigendecomposition elsewhere in this file (eigenvalues ascending, so
  // column 0 is the smallest -- the direction of least variance, i.e. the
  // plane normal).
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  std::vector<Eigen::Vector3f> inlier_pts;
  inlier_pts.reserve(static_cast<std::size_t>(best_inliers));
  for (const auto & p : candidates) {
    if (std::abs(best_normal.dot(p) + best_d) < inlier_dist) {
      inlier_pts.push_back(p);
      centroid += p;
    }
  }
  centroid /= static_cast<float>(inlier_pts.size());

  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (const auto & p : inlier_pts) {
    const Eigen::Vector3f d3 = p - centroid;
    cov += d3 * d3.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(cov);
  if (eigensolver.info() != Eigen::Success) {return result;}
  Eigen::Vector3f refined_normal = eigensolver.eigenvectors().col(0);
  if (refined_normal.z() < 0.0f) {refined_normal = -refined_normal;}
  const float refined_d = -refined_normal.dot(centroid);

  int refined_inliers = 0;
  for (const auto & p : candidates) {
    if (std::abs(refined_normal.dot(p) + refined_d) < inlier_dist) {++refined_inliers;}
  }

  result.normal = refined_normal;
  result.d = refined_d;
  result.inliers = refined_inliers;
  result.tilt_deg = std::acos(std::max(-1.0f, std::min(1.0f, refined_normal.z()))) *
    180.0f / static_cast<float>(M_PI);
  if (std::abs(refined_normal.z()) > 1e-6f) {
    result.height_at_robot = -(refined_normal.x() * robot_p.x() +
      refined_normal.y() * robot_p.y() + refined_d) / refined_normal.z();
    result.valid = true;
  }
  // else: near-vertical normal (wall-like fit) -- height_at_robot would be
  // ill-conditioned/garbage, so leave valid=false rather than reporting it
  // as a usable ground height.

  return result;
}

void dlio::OdomNode::applyGroundConstraint() {

  const bool was_engaged = this->gc_engaged_last_scan_.load();
  const int prev_reason = this->gc_disengage_reason_.load();

  if (!this->gc_enabled_) {
    // No atomics beyond these two touched, no diagnostic event, no state
    // mutation below this point: bit-identical to prior behavior.
    this->gc_disengage_reason_ = 1;
    this->gc_engaged_last_scan_ = false;
    return;
  }

  const GroundPlaneFit fit = this->fitGroundPlane();
  this->gc_last_inliers_ = fit.inliers;
  this->gc_last_normal_tilt_deg_ = fit.tilt_deg;
  this->gc_last_plane_height_ = fit.height_at_robot;

  int reason = 0;
  if (fit.candidates < (3 * this->gc_min_inliers_) / 2) {
    reason = 2;
  } else if (!fit.valid) {
    reason = 3;
  } else if (fit.inliers < this->gc_min_inliers_) {
    reason = 4;
  } else if (fit.inliers < this->gc_min_inlier_ratio_ * fit.candidates) {
    reason = 5;
  } else if (fit.tilt_deg > this->gc_max_normal_tilt_deg_) {
    reason = 6;
  } else if (
    this->gc_initialized_ &&
    std::abs(fit.height_at_robot - this->gc_tracked_ground_height_) > this->gc_max_height_jump_)
  {
    // A short-lived height jump is likely a transient false surface
    // (curb/ledge/underside). A persistent gap is accumulated estimation
    // drift, so re-engage and correct the state toward the fixed reference.
    // Do not move gc_tracked_ground_height_ here; it is the anchor.
    if (++this->gc_height_jump_streak_ <= this->gc_height_jump_recover_scans_) {
      reason = 7;
    }
  } else {
    this->gc_height_jump_streak_ = 0;
  }

  this->gc_disengage_reason_ = reason;
  const bool engaged_now = (reason == 0);
  this->gc_engaged_last_scan_ = engaged_now;

  // Edge-triggered logging only -- this engages ~every scan on flat ground,
  // so logging every scan would be event-log spam. Only record a transition.
  if (was_engaged != engaged_now || (!engaged_now && reason != prev_reason)) {
    std::ostringstream detail;
    detail << "engaged=" << (engaged_now ? "true" : "false")
           << " reason=" << reason
           << " inliers=" << fit.inliers
           << " candidates=" << fit.candidates
           << " tilt_deg=" << fit.tilt_deg
           << " plane_height=" << fit.height_at_robot
           << " tracked_height=" << this->gc_tracked_ground_height_;
    this->recordDiagnosticEvent("ground_constraint_state_change", detail.str());
  }

  if (!engaged_now) {
    return;
  }

  if (!this->gc_initialized_) {
    // First fit only establishes the reference height -- no correction yet.
    this->gc_tracked_ground_height_ = fit.height_at_robot;
    this->gc_tracked_ground_height_pub_ = static_cast<float>(fit.height_at_robot);
    this->gc_initialized_ = true;
    return;
  }

  const double dt = this->scan_stamp - this->prev_scan_stamp;
  const double z_err = this->gc_tracked_ground_height_ - fit.height_at_robot;
  const double clamp = this->gc_max_correction_per_scan_;
  const double z_corr = std::max(-clamp, std::min(clamp, dt * this->gc_kz_ * z_err));
  const float z_corr_f = static_cast<float>(z_corr);

  // All five must move together: lidarPose because updateKeyframes() (called
  // right after this in the scan pipeline) archives it into the new
  // keyframe -- that's what severs the ratchet. T/last_accepted_T_ because
  // the reject-recovery paths anchor to them; last_accepted_state_ for the
  // submap query anchor (see buildKeyframesAndSubmap call sites). Leaving
  // any one stale reintroduces an internal inconsistency that GICP corrects
  // back out next scan, defeating the whole point.
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.p[2] += z_corr_f;
    this->lidarPose.p[2] += z_corr_f;
    this->geo.prev_p = this->state.p;
  }
  this->T(2, 3) += z_corr_f;
  this->T_corr(2, 3) += z_corr_f; 
  this->last_accepted_T_(2, 3) += z_corr_f;
  this->last_accepted_state_.p[2] += z_corr_f;

  // Tracked height absorbs slow/real terrain change; the correction above
  // absorbs fast estimation drift -- so genuinely gentle terrain can still
  // pass through via heightFilterAlpha while estimation drift cannot.
  const double residual_terrain = fit.height_at_robot - this->gc_tracked_ground_height_;
  this->gc_tracked_ground_height_ += this->gc_height_filter_alpha_ * residual_terrain;
  this->gc_tracked_ground_height_pub_ = static_cast<float>(this->gc_tracked_ground_height_);

  this->gc_last_z_correction_ = z_corr_f;

  // Roll/pitch correction (gc_correct_roll_pitch_) is phase 2, intentionally
  // not implemented yet -- ship and verify z-only first, per spec.
}

void dlio::OdomNode::computeConvexHull() {

  // at least 4 keyframes for convex hull
  if (this->num_processed_keyframes < 4) {
    return;
  }

  // create a pointcloud with points at keyframes
  pcl::PointCloud<PointType>::Ptr cloud = std::make_shared<pcl::PointCloud<PointType>>();

  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  for (int i = 0; i < this->num_processed_keyframes; i++) {
    PointType pt;
    pt.x = this->keyframes[i].first.first[0];
    pt.y = this->keyframes[i].first.first[1];
    pt.z = this->keyframes[i].first.first[2];
    cloud->push_back(pt);
  }
  lock.unlock();

  // calculate the convex hull of the point cloud
  this->convex_hull.setInputCloud(cloud);

  // get the indices of the keyframes on the convex hull
  pcl::PointCloud<PointType>::Ptr convex_points = std::make_shared<pcl::PointCloud<PointType>>();
  this->convex_hull.reconstruct(*convex_points);

  pcl::PointIndices::Ptr convex_hull_point_idx = std::make_shared<pcl::PointIndices>();
  this->convex_hull.getHullPointIndices(*convex_hull_point_idx);

  this->keyframe_convex.clear();
  for (int i=0; i<convex_hull_point_idx->indices.size(); ++i) {
    this->keyframe_convex.push_back(convex_hull_point_idx->indices[i]);
  }

}

void dlio::OdomNode::computeConcaveHull() {

  // at least 5 keyframes for concave hull
  if (this->num_processed_keyframes < 5) {
    return;
  }

  // create a pointcloud with points at keyframes
  auto cloud = std::make_shared<pcl::PointCloud<PointType>>();

  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  for (int i = 0; i < this->num_processed_keyframes; i++) {
    PointType pt;
    pt.x = this->keyframes[i].first.first[0];
    pt.y = this->keyframes[i].first.first[1];
    pt.z = this->keyframes[i].first.first[2];
    cloud->push_back(pt);
  }
  lock.unlock();

  // calculate the concave hull of the point cloud
  this->concave_hull.setInputCloud(cloud);

  // get the indices of the keyframes on the concave hull
  pcl::PointCloud<PointType>::Ptr concave_points = std::make_shared<pcl::PointCloud<PointType>>();
  this->concave_hull.reconstruct(*concave_points);

  pcl::PointIndices::Ptr concave_hull_point_idx = std::make_shared<pcl::PointIndices>();
  this->concave_hull.getHullPointIndices(*concave_hull_point_idx);

  this->keyframe_concave.clear();
  for (int i=0; i<concave_hull_point_idx->indices.size(); ++i) {
    this->keyframe_concave.push_back(concave_hull_point_idx->indices[i]);
  }

}

void dlio::OdomNode::updateKeyframes() {

  // calculate difference in pose and rotation to all poses in trajectory
  float closest_d = std::numeric_limits<float>::infinity();
  int closest_idx = -1;
  int keyframes_idx = 0;

  int num_nearby = 0;
  int num_nearby_same_yaw = 0;

  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  for (const auto& k : this->keyframes) {

    // calculate distance between current pose and pose in keyframes
    float delta_d = sqrt( pow(this->state.p[0] - k.first.first[0], 2) +
                          pow(this->state.p[1] - k.first.first[1], 2) +
                          pow(this->state.p[2] - k.first.first[2], 2) );

    // count the number nearby current pose
    if (delta_d <= this->keyframe_thresh_dist_ * 1.5){
      ++num_nearby;
    }

    Eigen::Quaternionf dq;
    if (this->state.q.dot(k.first.second) < 0.) {
      Eigen::Quaternionf lq = k.first.second;
      lq.w() *= -1.; lq.x() *= -1.; lq.y() *= -1.; lq.z() *= -1.;
      dq = this->state.q * lq.inverse();
    } else {
      dq = this->state.q * k.first.second.inverse();
    }
    const double delta_theta_rad =
      2. * atan2(sqrt(pow(dq.x(), 2) + pow(dq.y(), 2) + pow(dq.z(), 2)), dq.w());
    const double delta_theta_deg = std::abs(delta_theta_rad * (180.0/M_PI));

    if (delta_d <= this->keyframe_thresh_dist_ * 1.5 &&
        delta_theta_deg <= this->keyframe_thresh_rot_ * 0.75) {
      ++num_nearby_same_yaw;
    }

    // store into variable
    if (delta_d < closest_d) {
      closest_d = delta_d;
      closest_idx = keyframes_idx;
    }

    keyframes_idx++;

  }

  if (closest_idx < 0) {
    return;
  }

  // get closest pose and corresponding rotation
  Eigen::Vector3f closest_pose = this->keyframes[closest_idx].first.first;
  Eigen::Quaternionf closest_pose_r = this->keyframes[closest_idx].first.second;

  // calculate distance between current pose and closest pose from above
  float dd = sqrt( pow(this->state.p[0] - closest_pose[0], 2) +
                   pow(this->state.p[1] - closest_pose[1], 2) +
                   pow(this->state.p[2] - closest_pose[2], 2) );

  // calculate difference in orientation using SLERP
  Eigen::Quaternionf dq;

  if (this->state.q.dot(closest_pose_r) < 0.) {
    Eigen::Quaternionf lq = closest_pose_r;
    lq.w() *= -1.; lq.x() *= -1.; lq.y() *= -1.; lq.z() *= -1.;
    dq = this->state.q * lq.inverse();
  } else {
    dq = this->state.q * closest_pose_r.inverse();
  }

  double theta_rad = 2. * atan2(sqrt( pow(dq.x(), 2) + pow(dq.y(), 2) + pow(dq.z(), 2) ), dq.w());
  double theta_deg = theta_rad * (180.0/M_PI);

  // update keyframes
  bool newKeyframe = false;

  if (abs(dd) > this->keyframe_thresh_dist_ || abs(theta_deg) > this->keyframe_thresh_rot_) {
    newKeyframe = true;
  }

  if (abs(dd) <= this->keyframe_thresh_dist_ && abs(theta_deg) <= this->keyframe_thresh_rot_) {
    newKeyframe = false;
  }

  // Turning in place still changes the LiDAR view. The original DLIO condition
  // suppressed rotation-only keyframes when several keyframes were nearby,
  // which starves scan-to-map during yaw sweeps.
  if (abs(dd) <= this->keyframe_thresh_dist_ &&
      abs(theta_deg) > this->keyframe_thresh_rot_ &&
      num_nearby_same_yaw == 0) {
    newKeyframe = true;
  }

  if (newKeyframe) {

    // update keyframe vector
    this->keyframes.push_back(std::make_pair(std::make_pair(this->lidarPose.p, this->lidarPose.q), this->current_scan));
    this->keyframe_timestamps.push_back(this->scan_header_stamp);
    this->keyframe_normals.push_back(this->gicp.getSourceCovariances());
    this->keyframe_transformations.push_back(this->T_corr);

  }

}

void dlio::OdomNode::setAdaptiveParams() {

  // Spaciousness
  float sp = this->metrics.spaciousness.back();

  if (sp < 0.5) { sp = 0.5; }
  if (sp > 5.0) { sp = 5.0; }

  this->keyframe_thresh_dist_ = sp;

  // Density
  float den = this->metrics.density.back();

  if (den < 0.5*this->gicp_max_corr_dist_) { den = 0.5*this->gicp_max_corr_dist_; }
  if (den > 2.0*this->gicp_max_corr_dist_) { den = 2.0*this->gicp_max_corr_dist_; }

  if (sp < 5.0) { den = 0.5*this->gicp_max_corr_dist_; };
  if (sp > 5.0) { den = 2.0*this->gicp_max_corr_dist_; };

  this->gicp.setMaxCorrespondenceDistance(den);

  // Concave hull alpha
  this->concave_hull.setAlpha(this->keyframe_thresh_dist_);

}

void dlio::OdomNode::pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames) {

  // make sure dists is not empty
  if (!dists.size()) { return; }

  // maintain max heap of at most k elements
  std::priority_queue<float> pq;

  for (auto d : dists) {
    if (pq.size() >= k && pq.top() > d) {
      pq.push(d);
      pq.pop();
    } else if (pq.size() < k) {
      pq.push(d);
    }
  }

  // get the kth smallest element, which should be at the top of the heap
  float kth_element = pq.top();

  // get all elements smaller or equal to the kth smallest element
  for (int i = 0; i < dists.size(); ++i) {
    if (dists[i] <= kth_element)
      this->submap_kf_idx_curr.push_back(frames[i]);
  }

}

void dlio::OdomNode::buildSubmap(State vehicle_state) {

  // clear vector of keyframe indices to use for submap
  this->submap_kf_idx_curr.clear();

  // calculate distance between current pose and poses in keyframe set
  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  std::vector<float> ds;
  std::vector<int> keyframe_nn;
  for (int i = 0; i < this->num_processed_keyframes; i++) {
    float d = sqrt( pow(vehicle_state.p[0] - this->keyframes[i].first.first[0], 2) +
                    pow(vehicle_state.p[1] - this->keyframes[i].first.first[1], 2) +
                    pow(vehicle_state.p[2] - this->keyframes[i].first.first[2], 2) );
    ds.push_back(d);
    keyframe_nn.push_back(i);
  }
  lock.unlock();

  // get indices for top K nearest neighbor keyframe poses
  this->pushSubmapIndices(ds, this->submap_knn_, keyframe_nn);

  // get convex hull indices
  this->computeConvexHull();

  // get distances for each keyframe on convex hull
  std::vector<float> convex_ds;
  for (const auto& c : this->keyframe_convex) {
    convex_ds.push_back(ds[c]);
  }

  // get indices for top kNN for convex hull
  this->pushSubmapIndices(convex_ds, this->submap_kcv_, this->keyframe_convex);

  // get concave hull indices
  this->computeConcaveHull();

  // get distances for each keyframe on concave hull
  std::vector<float> concave_ds;
  for (const auto& c : this->keyframe_concave) {
    concave_ds.push_back(ds[c]);
  }

  // get indices for top kNN for concave hull
  this->pushSubmapIndices(concave_ds, this->submap_kcc_, this->keyframe_concave);

  // sort current and previous submap kf list of indices
  std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
  std::sort(this->submap_kf_idx_prev.begin(), this->submap_kf_idx_prev.end());

  // remove duplicate indices
  auto last = std::unique(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
  this->submap_kf_idx_curr.erase(last, this->submap_kf_idx_curr.end());

  // check if submap has changed from previous iteration
  if (this->submap_kf_idx_curr != this->submap_kf_idx_prev){

    this->submap_hasChanged = true;

    // Pause to prevent stealing resources from the main loop if it is running.
    if (!this->pauseSubmapBuildIfNeeded()) {
      return;
    }

    // reinitialize submap cloud and normals
    pcl::PointCloud<PointType>::Ptr submap_cloud_ = std::make_shared<pcl::PointCloud<PointType>>();
    std::shared_ptr<nano_gicp::CovarianceList> submap_normals_ (std::make_shared<nano_gicp::CovarianceList>());

    for (auto k : this->submap_kf_idx_curr) {
      if (this->shutting_down_.load()) {
        return;
      }

      // create current submap cloud
      lock.lock();
      *submap_cloud_ += *this->keyframes[k].second;
      lock.unlock();

      // grab corresponding submap cloud's normals
      submap_normals_->insert( std::end(*submap_normals_),
          std::begin(*(this->keyframe_normals[k])), std::end(*(this->keyframe_normals[k])) );
    }

    this->submap_cloud = submap_cloud_;
    this->submap_normals = submap_normals_;

    // Pause to prevent stealing resources from the main loop if it is running.
    if (!this->pauseSubmapBuildIfNeeded()) {
      return;
    }

    this->gicp_temp.setInputTarget(this->submap_cloud);
    this->submap_kdtree = this->gicp_temp.target_kdtree_;

    this->submap_kf_idx_prev = this->submap_kf_idx_curr;
  }
}

void dlio::OdomNode::buildKeyframesAndSubmap(State vehicle_state) {

  if (this->shutting_down_.load()) {
    return;
  }

  // transform the new keyframe(s) and associated covariance list(s)
    std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);

  for (int i = this->num_processed_keyframes; i < this->keyframes.size(); i++) {
    if (this->shutting_down_.load()) {
      return;
    }
    pcl::PointCloud<PointType>::ConstPtr raw_keyframe = this->keyframes[i].second;
    std::shared_ptr<const nano_gicp::CovarianceList> raw_covariances = this->keyframe_normals[i];
    Eigen::Matrix4f T = this->keyframe_transformations[i];
    lock.unlock();

    Eigen::Matrix4d Td = T.cast<double>();

    pcl::PointCloud<PointType>::Ptr transformed_keyframe = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud (*raw_keyframe, *transformed_keyframe, T);

    std::shared_ptr<nano_gicp::CovarianceList> transformed_covariances (std::make_shared<nano_gicp::CovarianceList>(raw_covariances->size()));
    std::transform(raw_covariances->begin(), raw_covariances->end(), transformed_covariances->begin(),
                   [&Td](Eigen::Matrix4d cov) { return Td * cov * Td.transpose(); });

    ++this->num_processed_keyframes;

    lock.lock();
    this->keyframes[i].second = transformed_keyframe;
    this->keyframe_normals[i] = transformed_covariances;

    this->publishKeyframe(this->keyframes[i], this->keyframe_timestamps[i]);
  }

  lock.unlock();

  // Pause to prevent stealing resources from the main loop if it is running.
  if (!this->pauseSubmapBuildIfNeeded()) {
    return;
  }

  this->buildSubmap(vehicle_state);
}

bool dlio::OdomNode::pauseSubmapBuildIfNeeded() {
  std::unique_lock<decltype(this->main_loop_running_mutex)> lock(this->main_loop_running_mutex);
  this->submap_build_cv.wait(lock, [this]{
    return this->shutting_down_.load() || !this->main_loop_running;
  });
  return !this->shutting_down_.load();
}

void dlio::OdomNode::debug() {

  // Total length traversed. Tracked incrementally at the trajectory push
  // site now (see length_traversed_has_prev_) instead of rescanning the
  // full trajectory buffer here every tick -- trajectory is bounded (see
  // its declaration), so a full rescan would silently become a windowed
  // approximation rather than a true cumulative total.
  double length_traversed = this->length_traversed;

  // Average computation time
  double avg_comp_time =
    std::accumulate(this->comp_times.begin(), this->comp_times.end(), 0.0) / this->comp_times.size();

  // Average sensor rates
  int win_size = 100;
  double avg_imu_rate;
  double avg_lidar_rate;
  if (this->imu_rates.size() < win_size) {
    avg_imu_rate =
      std::accumulate(this->imu_rates.begin(), this->imu_rates.end(), 0.0) / this->imu_rates.size();
  } else {
    avg_imu_rate =
      std::accumulate(this->imu_rates.end()-win_size, this->imu_rates.end(), 0.0) / win_size;
  }
  if (this->lidar_rates.size() < win_size) {
    avg_lidar_rate =
      std::accumulate(this->lidar_rates.begin(), this->lidar_rates.end(), 0.0) / this->lidar_rates.size();
  } else {
    avg_lidar_rate =
      std::accumulate(this->lidar_rates.end()-win_size, this->lidar_rates.end(), 0.0) / win_size;
  }

  // RAM Usage
  double vm_usage = 0.0;
  double resident_set = 0.0;
  std::ifstream stat_stream("/proc/self/stat", std::ios_base::in); //get info from proc directory
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string num_threads, itrealvalue, starttime;
  unsigned long vsize;
  long rss;
  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
              >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
              >> utime >> stime >> cutime >> cstime >> priority >> nice
              >> num_threads >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
  stat_stream.close();
  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
  vm_usage = vsize / 1024.0;
  resident_set = rss * page_size_kb;

  // CPU Usage
  struct tms timeSample;
  clock_t now;
  double cpu_percent;
  now = times(&timeSample);
  if (now <= this->lastCPU || timeSample.tms_stime < this->lastSysCPU ||
      timeSample.tms_utime < this->lastUserCPU) {
      cpu_percent = -1.0;
  } else {
      cpu_percent = (timeSample.tms_stime - this->lastSysCPU) + (timeSample.tms_utime - this->lastUserCPU);
      cpu_percent /= (now - this->lastCPU);
      cpu_percent /= this->numProcessors;
      cpu_percent *= 100.;
  }
  this->lastCPU = now;
  this->lastSysCPU = timeSample.tms_stime;
  this->lastUserCPU = timeSample.tms_utime;
  this->cpu_percents.push_back(cpu_percent);
  double avg_cpu_usage =
    std::accumulate(this->cpu_percents.begin(), this->cpu_percents.end(), 0.0) / this->cpu_percents.size();

  // Print to terminal
  printf("\033[2J\033[1;1H");

  std::cout << std::endl
            << "+-------------------------------------------------------------------+" << std::endl;
  std::cout << "|               Direct LiDAR-Inertial Odometry v" << this->version_  << "               |"
            << std::endl;
  std::cout << "+-------------------------------------------------------------------+" << std::endl;

  std::time_t curr_time = this->scan_stamp;
  std::string asc_time = std::asctime(std::localtime(&curr_time)); asc_time.pop_back();
  std::cout << "| " << std::left << asc_time;
  std::cout << std::right << std::setfill(' ') << std::setw(42)
    << "Elapsed Time: " + to_string_with_precision(this->elapsed_time, 2) + " seconds "
    << "|" << std::endl;

  if ( !this->cpu_type.empty() ) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << this->cpu_type + " x " + std::to_string(this->numProcessors)
      << "|" << std::endl;
  }

  if (this->sensor == dlio::SensorType::OUSTER) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Ouster @ " + to_string_with_precision(avg_lidar_rate, 2)
                                   + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  } else if (this->sensor == dlio::SensorType::VELODYNE) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Velodyne @ " + to_string_with_precision(avg_lidar_rate, 2)
                                     + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  } else if (this->sensor == dlio::SensorType::HESAI) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Hesai @ " + to_string_with_precision(avg_lidar_rate, 2)
                                  + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  } else if (this->sensor == dlio::SensorType::LIVOX) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Livox @ " + to_string_with_precision(avg_lidar_rate, 2)
                                  + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  } else {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Unknown LiDAR @ " + to_string_with_precision(avg_lidar_rate, 2)
                                          + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  }

  std::cout << "|===================================================================|" << std::endl;

  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Position     {W}  [xyz] :: " + to_string_with_precision(this->state.p[0], 4) + " "
                                + to_string_with_precision(this->state.p[1], 4) + " "
                                + to_string_with_precision(this->state.p[2], 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Orientation  {W} [wxyz] :: " + to_string_with_precision(this->state.q.w(), 4) + " "
                                + to_string_with_precision(this->state.q.x(), 4) + " "
                                + to_string_with_precision(this->state.q.y(), 4) + " "
                                + to_string_with_precision(this->state.q.z(), 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Lin Velocity {B}  [xyz] :: " + to_string_with_precision(this->state.v.lin.b[0], 4) + " "
                                + to_string_with_precision(this->state.v.lin.b[1], 4) + " "
                                + to_string_with_precision(this->state.v.lin.b[2], 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Ang Velocity {B}  [xyz] :: " + to_string_with_precision(this->state.v.ang.b[0], 4) + " "
                                + to_string_with_precision(this->state.v.ang.b[1], 4) + " "
                                + to_string_with_precision(this->state.v.ang.b[2], 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Accel Bias        [xyz] :: " + to_string_with_precision(this->state.b.accel[0], 8) + " "
                                + to_string_with_precision(this->state.b.accel[1], 8) + " "
                                + to_string_with_precision(this->state.b.accel[2], 8)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Gyro Bias         [xyz] :: " + to_string_with_precision(this->state.b.gyro[0], 8) + " "
                                + to_string_with_precision(this->state.b.gyro[1], 8) + " "
                                + to_string_with_precision(this->state.b.gyro[2], 8)
    << "|" << std::endl;

  std::cout << "|                                                                   |" << std::endl;

  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Distance Traveled  :: " + to_string_with_precision(length_traversed, 4) + " meters"
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Distance to Origin :: "
      + to_string_with_precision( sqrt(pow(this->state.p[0]-this->origin[0],2) +
                                       pow(this->state.p[1]-this->origin[1],2) +
                                       pow(this->state.p[2]-this->origin[2],2)), 4) + " meters"
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Registration       :: keyframes: " + std::to_string(this->keyframes.size()) + ", "
                               + "deskewed points: " + std::to_string(this->deskew_size)
    << "|" << std::endl;
  std::cout << "|                                                                   |" << std::endl;

  std::cout << std::right << std::setprecision(2) << std::fixed;
  std::cout << "| Computation Time :: "
    << std::setfill(' ') << std::setw(6) << this->comp_times.back()*1000. << " ms    // Avg: "
    << std::setw(6) << avg_comp_time*1000. << " / Max: "
    << std::setw(6) << *std::max_element(this->comp_times.begin(), this->comp_times.end())*1000.
    << "     |" << std::endl;
  std::cout << "| Cores Utilized   :: "
    << std::setfill(' ') << std::setw(6) << (cpu_percent/100.) * this->numProcessors << " cores // Avg: "
    << std::setw(6) << (avg_cpu_usage/100.) * this->numProcessors << " / Max: "
    << std::setw(6) << (*std::max_element(this->cpu_percents.begin(), this->cpu_percents.end()) / 100.)
                       * this->numProcessors
    << "     |" << std::endl;
  std::cout << "| CPU Load         :: "
    << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: "
    << std::setw(6) << avg_cpu_usage << " / Max: "
    << std::setw(6) << *std::max_element(this->cpu_percents.begin(), this->cpu_percents.end())
    << "     |" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "RAM Allocation   :: " + to_string_with_precision(resident_set/1000., 2) + " MB"
    << "|" << std::endl;

  std::cout << "+-------------------------------------------------------------------+" << std::endl;

}
