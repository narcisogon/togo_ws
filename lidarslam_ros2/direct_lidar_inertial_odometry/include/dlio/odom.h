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
Declares the DLIO frontend node that converts synchronized LiDAR and IMU data
into continuous local odometry, motion-corrected clouds, keyframes, and a local
registration submap.

Frame ownership:
DLIO owns smooth local motion in the odom frame and publishes odom -> base_link.
The graph backend consumes DLIO outputs and owns the global map -> odom correction.

Concurrency:
LiDAR and IMU callbacks use separate mutually-exclusive callback groups. Scan
processing, publication, metrics, and submap construction also use coordinated
threads and futures, so shutdown and shared state require explicit synchronization.
*/

#include "dlio/dlio.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include <deque>
#include <fstream>
#include <random>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

// BOOST
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/adjacent_filtered.hpp>

// PCL
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

/*
Summary:
Owns the complete Direct LiDAR-Inertial Odometry frontend pipeline.
*/
class dlio::OdomNode: public rclcpp::Node {

public:

  /*
  Summary:
  Loads parameters, creates ROS interfaces, and initializes frontend state.
  */
  OdomNode();

  /*
  Summary:
  Requests shutdown and releases worker, file, and registration resources.
  */
  ~OdomNode();

  /*
  Summary:
  Prints startup information after the node has been constructed.
  */
  void start();

  /*
  Summary:
  Marks the pipeline as stopping and wakes work that may be waiting on data.
  */
  void requestShutdown();

private:

  struct State;
  struct ImuMeas;
  struct DiagnosticEvent;

  /*
  Summary:
  Guarantees LiDAR callback bookkeeping and waiter notification on every exit path.
  */
  struct CallbackCompletionGuard {
    /*
    Summary:
    Associates callback cleanup with its owning DLIO node.
    */
    explicit CallbackCompletionGuard(OdomNode * owner) : node(owner) {}

    /*
    Summary:
    Marks the LiDAR callback complete and wakes paused submap construction.
    */
    ~CallbackCompletionGuard();
    OdomNode * node;
  };

  /*
  Summary:
  Declares and reads all sensor, timing, registration, protection, and output parameters.
  */
  void getParams();

  /*
  Summary:
  Runs the full per-scan pipeline from cloud conversion through odometry publication.
  */
  void callbackPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc);

  /*
  Summary:
  Calibrates, transforms, buffers, and propagates each incoming IMU measurement.
  */
  void callbackImu(const sensor_msgs::msg::Imu::SharedPtr imu);

  /*
  Summary:
  Publishes the latest high-rate odometry pose, path, and odom -> base_link transform.
  */
  void publishPose();

  /*
  Summary:
  Publishes scan-rate odometry products after registration and state correction.
  */
  void publishToROS(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud);

  /*
  Summary:
  Transforms and publishes a deskewed cloud in the configured odom frame.
  */
  void publishCloud(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud);

  /*
  Summary:
  Publishes one accepted keyframe cloud with its odom-frame pose and timestamp.
  */
  void publishKeyframe(std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
                       pcl::PointCloud<PointType>::ConstPtr> kf, rclcpp::Time timestamp);

  /*
  Summary:
  Publishes the current frontend health, registration, timing, and filtering metrics.
  */
  void publishDiagnostics(float status_code, float raw_points = 0.0f, float filtered_points = 0.0f);

  /*
  Summary:
  Records whether buffered IMU data covers the current LiDAR scan interval.
  */
  void updateSyncDiagnostics(double scan_start_stamp, double scan_end_stamp, int imu_samples_used);

  /*
  Summary:
  Adds a structured event to the bounded diagnostic history and optional log file.
  */
  void recordDiagnosticEvent(const std::string& type, const std::string& detail,
                             float raw_points = 0.0f, float filtered_points = 0.0f,
                             double compute_time_ms = 0.0);

  /*
  Summary:
  Prints the diagnostic history once when shutdown or a fatal path requests it.
  */
  void printDiagnosticHistory();

  /*
  Summary:
  Formats one diagnostic event as a human-readable log record.
  */
  std::string formatDiagnosticEvent(const DiagnosticEvent& event) const;

  /*
  Summary:
  Maps a diagnostic event type to a likely operator-facing cause.
  */
  std::string diagnosticLikelyCause(const std::string& type) const;

  /*
  Summary:
  Appends one event to the configured persistent diagnostic-history file.
  */
  void appendDiagnosticEventToFile(const DiagnosticEvent& event);

  /*
  Summary:
  Updates the atomic stage identifier used by frontend heartbeat diagnostics.
  */
  void setPipelineStage(int stage);

  /*
  Summary:
  Publishes periodic liveness and detects a LiDAR callback stalled in one stage.
  */
  void publishPipelineHeartbeat();

  /*
  Summary:
  Converts PointCloud2 fields into DLIO points and establishes per-point scan timing.
  */
  void getScanFromROS(const sensor_msgs::msg::PointCloud2::SharedPtr& pc);

  /*
  Summary:
  Applies crop, dust rejection, voxelization, and optional normal-space sampling.
  */
  void preprocessPoints();

  /*
  Summary:
  Uses the continuous IMU trajectory to compensate motion within the current scan.
  */
  void deskewPointcloud();

  /*
  Summary:
  Seeds NanoGICP's first local target from the initialized scan.
  */
  void initializeInputTarget();

  /*
  Summary:
  Supplies the current processed scan and covariances to NanoGICP.
  */
  void setInputSource();

  /*
  Summary:
  Completes IMU calibration, gravity alignment, and first-scan initialization.
  */
  void initializeDLIO();

  /*
  Summary:
  Predicts, registers, validates, and commits the next scan pose.

  Returns:
  True when a usable pose update is available for publication and mapping.
  */
  bool getNextPose();

  /*
  Summary:
  Locates IMU samples spanning a requested time interval, waiting within configured limits.
  */
  bool imuMeasFromTimeRange(double start_time, double end_time,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it,
                            std::unique_lock<std::mutex>& imu_lock);

  /*
  Summary:
  Integrates buffered IMU motion at requested timestamps for scan deskewing.
  */
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                 const std::vector<double>& sorted_timestamps);

  /*
  Summary:
  Performs the interval integration once the required IMU iterator range is known.
  */
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                         const std::vector<double>& sorted_timestamps,
                         boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
                         boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it);

  /*
  Summary:
  Converts the IMU-predicted state into NanoGICP's initial transformation guess.
  */
  void propagateGICP();

  /*
  Summary:
  Propagates the local state continuously from the newest transformed IMU measurement.
  */
  void propagateState();

  /*
  Summary:
  Corrects pose, velocity, and sensor biases using the registered LiDAR pose.
  */
  void updateState();

  // Gated ground-plane z-constraint: fits the dominant ground plane from
  // near-ground points each scan and, when confidence gates pass, softly
  // pulls state.p.z() toward consistency with a slowly-tracked ground
  // height. Strictly additive -- see applyGroundConstraint() call site for
  // the guarantee that gc_enabled_: false is bit-identical to prior behavior.
  struct GroundPlaneFit
  {
    bool valid {false};
    Eigen::Vector3f normal {Eigen::Vector3f::UnitZ()};  // unit, oriented +z up
    float d {0.0f};                                     // plane: n.p + d = 0, odom frame
    float height_at_robot {0.0f};
    int inliers {0};
    int candidates {0};
    float tilt_deg {0.0f};
  };

  /*
  Summary:
  Robustly estimates the current dominant ground plane and confidence metrics.
  */
  GroundPlaneFit fitGroundPlane();

  /*
  Summary:
  Applies a gated soft height and optional roll/pitch correction from the ground plane.
  */
  void applyGroundConstraint();

  /*
  Summary:
  Adapts keyframe and registration settings using current scene metrics.
  */
  void setAdaptiveParams();

  /*
  Summary:
  Legacy keyframe-cloud preparation hook retained by the class interface.
  */
  void setKeyframeCloud();

  /*
  Summary:
  Updates environmental metrics used by adaptive parameter selection.
  */
  void computeMetrics();

  /*
  Summary:
  Estimates open-space scale from ranges in the current scan.
  */
  void computeSpaciousness();

  /*
  Summary:
  Estimates local point density for adaptive registration behavior.
  */
  void computeDensity();

  /*
  Summary:
  Applies configured IMU intrinsics and base-link extrinsics to a raw measurement.
  */
  sensor_msgs::msg::Imu::SharedPtr transformImu(const sensor_msgs::msg::Imu::SharedPtr& imu);

  /*
  Summary:
  Accepts keyframes based on translation or rotation change and updates stored poses.
  */
  void updateKeyframes();

  /*
  Summary:
  Selects spatially important keyframes on the convex hull around the current pose.
  */
  void computeConvexHull();

  /*
  Summary:
  Selects additional nearby keyframes from a concave hull representation.
  */
  void computeConcaveHull();

  /*
  Summary:
  Inserts the nearest eligible keyframe indices into the next local submap selection.
  */
  void pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames);

  /*
  Summary:
  Builds a registration target by transforming and combining selected keyframes.
  */
  void buildSubmap(State vehicle_state);

  /*
  Summary:
  Adds a new keyframe when required and asynchronously refreshes the local submap.
  */
  void buildKeyframesAndSubmap(State vehicle_state);

  /*
  Summary:
  Waits when necessary to prevent submap mutation from overlapping scan registration.
  */
  bool pauseSubmapBuildIfNeeded();

  /*
  Summary:
  Emits verbose runtime state used for frontend development and troubleshooting.
  */
  void debug();

  rclcpp::TimerBase::SharedPtr publish_timer;
  rclcpp::TimerBase::SharedPtr diagnostics_timer;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::CallbackGroup::SharedPtr lidar_cb_group, imu_cb_group;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr kf_pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr kf_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr diagnostics_pub;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> br;

  // ROS Msgs
  nav_msgs::msg::Odometry odom_ros;
  geometry_msgs::msg::PoseStamped pose_ros;
  nav_msgs::msg::Path path_ros;
  geometry_msgs::msg::PoseArray kf_pose_ros;

  // path_ros.poses/kf_pose_ros.poses previously grew unboundedly and were
  // republished in full every tick. odom/path/publish, publishEveryN,
  // maxPoses, minDistance were present in yaml configs but never actually
  // declared/read anywhere in this code -- they did nothing. Now wired up.
  bool path_publish_ {true};
  int path_publish_every_n_ {1};
  int path_max_poses_ {2000};
  double path_min_distance_ {0.0};
  int path_pose_counter_ {0};
  Eigen::Vector3f path_last_pushed_p_ {Eigen::Vector3f::Zero()};
  bool path_has_last_pushed_ {false};
  static constexpr size_t kMaxKeyframePoses = 5000;

  // Flags
  std::atomic<bool> dlio_initialized;
  std::atomic<bool> first_valid_scan;
  std::atomic<bool> first_imu_received;
  std::atomic<bool> imu_calibrated;
  std::atomic<bool> submap_hasChanged;
  std::atomic<bool> gicp_hasConverged;
  std::atomic<bool> deskew_status;
  std::atomic<int> deskew_size;
  // Pipeline stage: 0=waiting, 1=callback entry, 2=initialize, 3=convert,
  // 4=preprocess/deskew, 5=metrics, 6=input source, 7=registration,
  // 8=keyframes, 9=submap scheduling, 10=publishing.
  std::atomic<int> pipeline_stage_ {0};
  std::atomic<uint64_t> lidar_callbacks_started_ {0};
  std::atomic<uint64_t> lidar_callbacks_completed_ {0};
  std::atomic<bool> shutting_down_ {false};
  std::atomic<int64_t> last_lidar_receive_steady_ns_ {0};
  std::atomic<int64_t> callback_start_steady_ns_ {0};
  std::atomic<int64_t> last_callback_complete_steady_ns_ {0};

  // Threads
  std::thread publish_thread;
  std::thread publish_keyframe_thread;
  std::thread metrics_thread;
  std::thread debug_thread;

  // Trajectory
  // Bounded so long runs don't grow this without limit; the debug() report
  // used to rescan the entire vector every tick to derive length_traversed.
  // That total is now tracked incrementally at the push site instead (see
  // length_traversed_prev_p_/length_traversed_has_prev_ below), so bounding
  // this buffer doesn't turn "distance traveled" into a windowed
  // approximation -- it stays a true cumulative total.
  boost::circular_buffer<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
  double length_traversed;
  Eigen::Vector3f length_traversed_prev_p_ {Eigen::Vector3f::Zero()};
  bool length_traversed_has_prev_ {false};

  // Keyframes
  std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
                        pcl::PointCloud<PointType>::ConstPtr>> keyframes;
  std::vector<rclcpp::Time> keyframe_timestamps;
  std::vector<std::shared_ptr<const nano_gicp::CovarianceList>> keyframe_normals;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> keyframe_transformations;
  std::mutex keyframes_mutex;

  // Sensor Type
  dlio::SensorType sensor;

  // Frames
  std::string odom_frame;
  std::string baselink_frame;
  std::string lidar_frame;
  std::string imu_frame;

  // Preprocessing
  pcl::CropBox<PointType> crop;
  pcl::VoxelGrid<PointType> voxel;

  // Point Clouds
  pcl::PointCloud<PointType>::ConstPtr original_scan;
  pcl::PointCloud<PointType>::ConstPtr deskewed_scan;
  pcl::PointCloud<PointType>::ConstPtr current_scan;

  // Keyframes
  pcl::PointCloud<PointType>::ConstPtr keyframe_cloud;
  int num_processed_keyframes;

  pcl::ConvexHull<PointType> convex_hull;
  pcl::ConcaveHull<PointType> concave_hull;
  std::vector<int> keyframe_convex;
  std::vector<int> keyframe_concave;

  // Submap
  pcl::PointCloud<PointType>::ConstPtr submap_cloud;
  std::shared_ptr<const nano_gicp::CovarianceList> submap_normals;
  std::shared_ptr<const nanoflann::KdTreeFLANN<PointType>> submap_kdtree;

  std::vector<int> submap_kf_idx_curr;
  std::vector<int> submap_kf_idx_prev;

  bool new_submap_is_ready;
  std::future<void> submap_future;
  std::condition_variable submap_build_cv;
  bool main_loop_running;
  std::mutex main_loop_running_mutex;

  // Timestamps
  rclcpp::Time scan_header_stamp;
  double scan_stamp;
  double prev_scan_stamp;
  double scan_dt;
  // Bounded (previously unbounded std::vector): debug() reads these with
  // std::accumulate/std::max_element every tick, so unbounded growth meant
  // O(n) work per debug print that got slower forever on long runs.
  boost::circular_buffer<double> comp_times;
  boost::circular_buffer<double> imu_rates;
  boost::circular_buffer<double> lidar_rates;

  double first_scan_stamp;
  double elapsed_time;

  // GICP
  nano_gicp::NanoGICP<PointType, PointType> gicp;
  nano_gicp::NanoGICP<PointType, PointType> gicp_temp;

  // Transformations
  Eigen::Matrix4f T, T_prior, T_corr;
  Eigen::Quaternionf q_final;

  Eigen::Vector3f origin;

  struct Extrinsics {
    struct SE3 {
      Eigen::Vector3f t;
      Eigen::Matrix3f R;
    };
    SE3 baselink2imu;
    SE3 baselink2lidar;
    Eigen::Matrix4f baselink2imu_T;
    Eigen::Matrix4f baselink2lidar_T;
  }; Extrinsics extrinsics;

  // IMU
  rclcpp::Time imu_stamp;
  double first_imu_stamp;
  double prev_imu_stamp;
  double imu_dp, imu_dq_deg;

  struct ImuMeas {
    double stamp;
    double dt; // defined as the difference between the current and the previous measurement
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f lin_accel;
  }; ImuMeas imu_meas;

  boost::circular_buffer<ImuMeas> imu_buffer;
  std::mutex mtx_imu;
  std::condition_variable cv_imu_stamp;

  /*
  Summary:
  Orders buffered IMU measurements by timestamp for interval integration.
  */
  static bool comparatorImu(ImuMeas m1, ImuMeas m2) {
    return (m1.stamp < m2.stamp);
  };

  // Geometric Observer
  struct Geo {
    bool first_opt_done;
    std::mutex mtx;
    double dp;
    double dq_deg;
    Eigen::Vector3f prev_p;
    Eigen::Quaternionf prev_q;
    Eigen::Vector3f prev_vel;
  }; Geo geo;

  // State Vector
  struct ImuBias {
    Eigen::Vector3f gyro;
    Eigen::Vector3f accel;
  };

  struct Frames {
    Eigen::Vector3f b;
    Eigen::Vector3f w;
  };

  struct Velocity {
    Frames lin;
    Frames ang;
  };

  struct State {
    Eigen::Vector3f p; // position in world frame
    Eigen::Quaternionf q; // orientation in world frame
    Velocity v;
    ImuBias b; // imu biases in body frame
  }; State state;

  struct Pose {
    Eigen::Vector3f p; // position in world frame
    Eigen::Quaternionf q; // orientation in world frame
  };
  Pose lidarPose;
  Pose imuPose;

  // Metrics
  struct Metrics {
    std::vector<float> spaciousness;
    std::vector<float> density;
  }; Metrics metrics;

  std::string cpu_type;
  boost::circular_buffer<double> cpu_percents;  // bounded, see comp_times comment
  clock_t lastCPU, lastSysCPU, lastUserCPU;
  int numProcessors;
  double last_debug_print_time_;

  struct DiagnosticEvent {
    double wall_time;
    double scan_stamp;
    std::string type;
    std::string detail;
    float raw_points;
    float filtered_points;
    double compute_time_ms;
    double latest_imu_age_ms;
    int keyframes;
    int imu_buffer_size;
    int bad_correction_streak;
    float correction_translation;
    float correction_rotation_deg;
    float gicp_fitness;
    int gicp_inliers;
    float gicp_overlap;
    float hessian_min_eigen;
    float hessian_max_eigen;
    float hessian_condition;
    float gicp_solve_time_ms;
    int gicp_quality_mode;
    float angular_rate;
    bool spin_protection_active;
    float imu_age_ms_snapshot;
    bool timing_protection_active;
    double lidar_header_stamp;
    double scan_start_stamp;
    double scan_end_stamp;
    double oldest_imu_stamp;
    double latest_imu_stamp;
    float latest_imu_minus_lidar_ms;
    bool imu_covers_scan_start;
    bool imu_covers_scan_end;
    int imu_samples_used_for_deskew;
    int stale_scan_drop_count;
    float scan_duration_ms;
    Eigen::Vector3f position;
    float orientation_w;
    size_t submap_points;
  };
  std::deque<DiagnosticEvent> diagnostic_history_;
  std::mutex diagnostic_history_mutex_;
  std::string diagnostic_history_file_;
  std::atomic<bool> diagnostic_history_printed_;
  // Kept open for the node's lifetime instead of opening/closing per event
  // (appendDiagnosticEventToFile used to construct a fresh ofstream on every
  // call -- a syscall-heavy operation landing in the hot path during exactly
  // the reject storms where compute headroom matters most).
  std::ofstream diagnostic_history_stream_;

  // Parameters
  std::string version_;
  int num_threads_;
  double debug_print_period_;
  int diagnostic_history_size_;
  double diagnostic_lag_warning_ms_;
  double diagnostic_imu_age_warning_ms_;
  int diagnostic_keyframe_warning_count_;
  int diagnostic_submap_warning_points_;
  // imuMeasFromTimeRange's cv_imu_stamp.wait() previously had no timeout --
  // if the IMU stream stalled (sim pause, driver hiccup, bag end), the lidar
  // callback would block forever holding its callback group, killing the
  // node with no diagnostic. Now bounded by this and logged on timeout.
  double diagnostic_imu_wait_timeout_ms_;
  // Bounds trajectory/comp_times/imu_rates/lidar_rates/cpu_percents (see
  // their declarations) -- previously unbounded std::vector.
  int debug_metrics_history_size_;

  // Gated ground-plane z-constraint (see applyGroundConstraint()).
  bool gc_enabled_;
  double gc_max_range_, gc_zband_below_, gc_zband_above_;
  double gc_baselink_height_;
  int gc_ransac_iterations_;
  double gc_ransac_inlier_dist_;
  int gc_min_inliers_;
  double gc_min_inlier_ratio_;
  double gc_max_normal_tilt_deg_;
  double gc_max_height_jump_;
  double gc_height_filter_alpha_;
  double gc_kz_, gc_krp_;
  bool gc_correct_roll_pitch_;
  double gc_max_correction_per_scan_;
  // Consecutive scans a height-jump condition must persist before it's
  // treated as real drift (re-baseline) rather than a transient false
  // surface (curb/ledge, stay disengaged) -- see applyGroundConstraint().
  int gc_height_jump_recover_scans_;

  // Ground constraint state -- all touched only from the lidar callback
  // thread (applyGroundConstraint()/fitGroundPlane()), so plain types are
  // fine here. gc_tracked_ground_height_pub_ mirrors
  // gc_tracked_ground_height_ into an atomic for publishDiagnostics(),
  // which runs on a separate 1Hz wall-timer thread.
  bool gc_initialized_ {false};
  double gc_tracked_ground_height_ {0.0};  // ground plane height in odom frame
  std::atomic<float> gc_tracked_ground_height_pub_ {0.0f};
  int gc_height_jump_streak_ {0};
  std::mt19937 gc_rng_ {12345};
  std::atomic<bool> gc_engaged_last_scan_ {false};
  std::atomic<float> gc_last_plane_height_ {0.0f};
  std::atomic<float> gc_last_normal_tilt_deg_ {0.0f};
  std::atomic<int> gc_last_inliers_ {0};
  std::atomic<float> gc_last_z_correction_ {0.0f};
  // 0=engaged,1=disabled,2=few candidates,3=ransac fail,4=few inliers,
  // 5=ratio,6=tilt,7=height jump
  std::atomic<int> gc_disengage_reason_ {0};

  bool deskew_;
  // Optional synthetic per-column timing for organized clouds that do not
  // carry a native per-point time field (for example the simulated Seyond).
  bool synthetic_timing_enabled_ {false};
  double synthetic_timing_scan_period_sec_ {1.0 / 15.0};
  bool synthetic_timing_reverse_columns_ {false};

  double gravity_;

  bool time_offset_;

  bool adaptive_params_;

  double obs_submap_thresh_;
  double obs_keyframe_thresh_;
  double obs_keyframe_lag_;

  double keyframe_thresh_dist_;
  double keyframe_thresh_rot_;

  int submap_knn_;
  int submap_kcv_;
  int submap_kcc_;
  double submap_concave_alpha_;

  bool densemap_filtered_;
  bool wait_until_move_;

  double crop_size_;

  bool vf_use_;
  double vf_res_;

  bool imu_calibrate_;
  bool calibrate_gyro_;
  bool calibrate_accel_;
  bool gravity_align_;
  double imu_calib_time_;
  int imu_buffer_size_;
  Eigen::Matrix3f imu_accel_sm_;

  int gicp_min_num_points_;
  int gicp_k_correspondences_;
  double gicp_max_corr_dist_;
  int gicp_max_iter_;
  double gicp_transformation_ep_;
  double gicp_rotation_ep_;
  double gicp_init_lambda_factor_;
  std::string gicp_regularization_method_;
  bool gicp_reject_bad_corrections_;
  double gicp_max_correction_translation_;
  double gicp_max_correction_rotation_;
  bool spin_protection_enabled_;
  double spin_protection_angular_rate_;
  double spin_protection_max_translation_;
  double spin_protection_recovery_translation_step_;
  double spin_protection_recovery_rotation_step_;
  bool spin_protection_use_imu_prior_on_reject_;
  bool timing_protection_enabled_;
  double timing_protection_imu_age_ms_;
  double timing_protection_max_translation_;
  double timing_protection_max_rotation_;
  double timing_protection_recovery_translation_step_;
  double timing_protection_recovery_rotation_step_;
  bool timing_protection_drop_stale_scans_;
  double timing_protection_drop_imu_age_ms_;
  int timing_protection_drop_reject_streak_;
  int timing_protection_max_iterations_;
  std::atomic<float> last_correction_translation_;
  std::atomic<float> last_correction_rotation_deg_;
  std::atomic<bool> last_correction_rejected_;
  std::atomic<float> last_gicp_fitness_;
  std::atomic<int> last_gicp_inliers_;
  std::atomic<float> last_gicp_overlap_;
  std::atomic<float> last_hessian_min_eigen_;
  std::atomic<float> last_hessian_max_eigen_;
  std::atomic<float> last_hessian_condition_;
  std::atomic<float> last_gicp_solve_time_ms_;
  std::atomic<int> last_gicp_quality_mode_;
  std::atomic<float> last_angular_rate_;
  std::atomic<bool> last_spin_protection_active_;
  std::atomic<float> last_imu_age_ms_;
  std::atomic<bool> last_timing_protection_active_;
  // Epoch timestamps must remain double precision. A float near 1.78e9 has
  // ~128-second resolution and previously created false IMU-age errors.
  std::atomic<double> last_lidar_header_stamp_;
  std::atomic<double> last_scan_start_stamp_;
  std::atomic<double> last_scan_end_stamp_;
  std::atomic<double> last_oldest_imu_stamp_;
  std::atomic<double> last_latest_imu_stamp_;
  std::atomic<float> last_latest_imu_minus_lidar_ms_;
  std::atomic<bool> last_imu_covers_scan_start_;
  std::atomic<bool> last_imu_covers_scan_end_;
  std::atomic<int> last_imu_samples_used_for_deskew_;
  std::atomic<int> stale_scan_drop_count_;
  std::atomic<float> last_scan_duration_ms_;
  std::atomic<int> bad_correction_streak_;
  bool freeze_on_bad_correction_;
  int bad_correction_freeze_streak_;
  double bad_correction_recovery_translation_step_;
  double bad_correction_recovery_rotation_step_;
  double bad_correction_max_linear_speed_;
  double bad_correction_velocity_decay_;
  int bad_correction_velocity_decay_streak_;
  int bad_correction_hold_streak_;
  bool quality_use_partial_degenerate_;
  double quality_min_overlap_;
  double quality_degenerate_min_overlap_;
  double quality_max_fitness_;
  double quality_degenerate_max_fitness_;
  double quality_min_hessian_eigen_;
  double quality_max_hessian_condition_;
  double quality_partial_correction_scale_;
  double quality_partial_max_translation_;
  double quality_partial_max_rotation_;
  bool innovation_gate_enabled_;
  double innovation_gate_partial_scale_;
  double innovation_gate_max_translation_;
  double innovation_gate_max_yaw_;
  bool degeneracy_projection_enabled_;
  double degeneracy_projection_condition_;
  double degeneracy_projection_min_scale_;
  bool recovery_enabled_;
  int recovery_reject_streak_;
  double recovery_max_correspondence_distance_;
  int recovery_max_iterations_;
  double recovery_accept_translation_;
  double recovery_accept_rotation_;
  double recovery_min_overlap_;
  double recovery_max_fitness_;
  int recovery_attempt_spacing_;
  bool recovery_skip_when_timing_active_;
  int recovery_last_attempt_streak_;
  std::vector<double> recovery_yaw_offsets_deg_;
  State last_accepted_state_;
  Eigen::Matrix4f last_accepted_T_;

  double geo_Kp_;
  double geo_Kv_;
  double geo_Kq_;
  double geo_Kab_;
  double geo_Kgb_;
  double geo_abias_max_;
  double geo_gbias_max_;

};
