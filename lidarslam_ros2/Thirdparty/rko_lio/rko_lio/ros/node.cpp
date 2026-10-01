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

#include "node.hpp"
#include "rko_lio/core/process_timestamps.hpp"
#include "rko_lio/core/profiler.hpp"
#include "rko_lio/ros/utils/utils.hpp"
// other
#include <chrono>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <rclcpp/serialization.hpp>
#include <stdexcept>

namespace {
using namespace std::literals;

rko_lio::core::ImuControl imu_msg_to_imu_data(const sensor_msgs::msg::Imu& imu_msg) {
  rko_lio::core::ImuControl imu_data;
  imu_data.time = rko_lio::ros::utils::ros_time_to_seconds(imu_msg.header.stamp);
  imu_data.angular_velocity = rko_lio::ros::utils::ros_xyz_to_eigen_vector3d(imu_msg.angular_velocity);
  imu_data.acceleration = rko_lio::ros::utils::ros_xyz_to_eigen_vector3d(imu_msg.linear_acceleration);
  return imu_data;
}

} // namespace

namespace rko_lio::core {
// necessary for serializing the config, including the namespacing
void to_json(nlohmann::json& j, const LIO::Config& c) {
  j = {
      {"deskew", c.deskew},
      {"max_iterations", c.max_iterations},
      {"voxel_size", c.voxel_size},
      {"max_points_per_voxel", c.max_points_per_voxel},
      {"max_range", c.max_range},
      {"min_range", c.min_range},
      {"convergence_criterion", c.convergence_criterion},
      {"max_correspondance_distance", c.max_correspondance_distance},
      {"registration_error_model", c.registration_error_model},
      {"max_voxel_search_radius", c.max_voxel_search_radius},
      {"plane_fit_neighbors", c.plane_fit_neighbors},
      {"plane_fit_min_neighbors", c.plane_fit_min_neighbors},
      {"plane_fit_max_distance", c.plane_fit_max_distance},
      {"plane_fit_max_eigenvalue", c.plane_fit_max_eigenvalue},
      {"point_to_plane_keypoint_stride", c.point_to_plane_keypoint_stride},
      {"coarse_to_fine_registration", c.coarse_to_fine_registration},
      {"coarse_voxel_size", c.coarse_voxel_size},
      {"coarse_max_correspondance_distance", c.coarse_max_correspondance_distance},
      {"coarse_max_iterations", c.coarse_max_iterations},
      {"enable_imu_pose_prior", c.enable_imu_pose_prior},
      {"imu_pose_prior_translation_weight", c.imu_pose_prior_translation_weight},
      {"imu_pose_prior_rotation_weight", c.imu_pose_prior_rotation_weight},
      {"enable_adaptive_imu_pose_prior", c.enable_adaptive_imu_pose_prior},
      {"adaptive_imu_pose_prior_condition", c.adaptive_imu_pose_prior_condition},
      {"adaptive_imu_pose_prior_max_condition_ratio", c.adaptive_imu_pose_prior_max_condition_ratio},
      {"adaptive_imu_pose_prior_max_translation_weight", c.adaptive_imu_pose_prior_max_translation_weight},
      {"adaptive_imu_pose_prior_max_rotation_weight", c.adaptive_imu_pose_prior_max_rotation_weight},
      {"enable_degeneracy_damping", c.enable_degeneracy_damping},
      {"degeneracy_damping_condition", c.degeneracy_damping_condition},
      {"degeneracy_damping_alpha", c.degeneracy_damping_alpha},
      {"adaptive_degeneracy_damping", c.adaptive_degeneracy_damping},
      {"degeneracy_damping_min_alpha", c.degeneracy_damping_min_alpha},
      {"enable_eigen_degeneracy_projection", c.enable_eigen_degeneracy_projection},
      {"eigen_degeneracy_projection_condition", c.eigen_degeneracy_projection_condition},
      {"eigen_degeneracy_projection_min_scale", c.eigen_degeneracy_projection_min_scale},
      {"enable_stationary_hold", c.enable_stationary_hold},
      {"stationary_angular_velocity_threshold", c.stationary_angular_velocity_threshold},
      {"stationary_linear_acceleration_threshold", c.stationary_linear_acceleration_threshold},
      {"stationary_max_translation_delta", c.stationary_max_translation_delta},
      {"stationary_max_rotation_delta", c.stationary_max_rotation_delta},
      {"enable_vertical_spike_filter", c.enable_vertical_spike_filter},
      {"max_vertical_update_m", c.max_vertical_update_m},
      {"enable_rover_degeneracy_motion_constraint", c.enable_rover_degeneracy_motion_constraint},
      {"rover_degeneracy_motion_condition", c.rover_degeneracy_motion_condition},
      {"rover_degeneracy_forward_scale", c.rover_degeneracy_forward_scale},
      {"rover_degeneracy_lateral_scale", c.rover_degeneracy_lateral_scale},
      {"rover_degeneracy_vertical_scale", c.rover_degeneracy_vertical_scale},
      {"rover_degeneracy_roll_pitch_scale", c.rover_degeneracy_roll_pitch_scale},
      {"rover_degeneracy_yaw_scale", c.rover_degeneracy_yaw_scale},
      {"max_num_threads", c.max_num_threads},
      {"initialization_phase", c.initialization_phase},
      {"max_expected_jerk", c.max_expected_jerk},
      {"gravity_magnitude", c.gravity_magnitude},
      {"double_downsample", c.double_downsample},
      {"min_beta", c.min_beta},
      {"max_scan_delta_sec", c.max_scan_delta_sec},
      {"enable_kidnap_relocalization", c.enable_kidnap_relocalization},
      {"reset_on_registration_failure", c.reset_on_registration_failure},
      {"recovery_min_failures", c.recovery_min_failures},
      {"relocalize_after_scan_gap", c.relocalize_after_scan_gap},
      {"relocalization_min_correspondences", c.relocalization_min_correspondences},
      {"relocalization_min_inlier_ratio", c.relocalization_min_inlier_ratio},
      {"relocalization_max_mean_error", c.relocalization_max_mean_error},
      {"relocalization_max_correspondance_distance", c.relocalization_max_correspondance_distance},
      {"relocalization_yaw_samples", c.relocalization_yaw_samples},
      {"relocalization_pose_stride", c.relocalization_pose_stride},
      {"relocalization_min_pose_separation", c.relocalization_min_pose_separation},
      {"relocalization_max_iterations", c.relocalization_max_iterations},
  };
}
} // namespace rko_lio::core

namespace rko_lio::ros {

Node::Node(const std::string& node_name, const rclcpp::NodeOptions& options) {
  node = rclcpp::Node::make_shared(node_name, options);
  imu_topic = node->declare_parameter<std::string>("imu_topic");     // required
  lidar_topic = node->declare_parameter<std::string>("lidar_topic"); // required
  base_frame = node->declare_parameter<std::string>("base_frame");   // required
  imu_frame = node->declare_parameter<std::string>("imu_frame", imu_frame);
  lidar_frame = node->declare_parameter<std::string>("lidar_frame", lidar_frame);
  odom_frame = node->declare_parameter<std::string>("odom_frame", odom_frame);
  odom_topic = node->declare_parameter<std::string>("odom_topic", odom_topic);
  const int configured_lidar_buffer_size =
      node->declare_parameter<int>("max_lidar_buffer_size", static_cast<int>(max_lidar_buffer_size));
  max_lidar_buffer_size = static_cast<size_t>(configured_lidar_buffer_size < 1 ? 1 : configured_lidar_buffer_size);
  drop_oldest_lidar_when_full =
      node->declare_parameter<bool>("drop_oldest_lidar_when_full", drop_oldest_lidar_when_full);

  // tf
  invert_odom_tf = node->declare_parameter<bool>("invert_odom_tf", invert_odom_tf);
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

  // publishing
  const rclcpp::QoS publisher_qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
  odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>(odom_topic, publisher_qos);
  publish_registration_diagnostics =
      node->declare_parameter<bool>("publish_registration_diagnostics", publish_registration_diagnostics);
  if (publish_registration_diagnostics) {
    registration_diagnostics_topic =
        node->declare_parameter<std::string>("registration_diagnostics_topic", registration_diagnostics_topic);
    registration_diagnostics_publisher =
        node->create_publisher<std_msgs::msg::Float32MultiArray>(registration_diagnostics_topic, publisher_qos);
  }
  publish_runtime_diagnostics =
      node->declare_parameter<bool>("publish_runtime_diagnostics", publish_runtime_diagnostics);
  if (publish_runtime_diagnostics) {
    runtime_diagnostics_topic =
        node->declare_parameter<std::string>("runtime_diagnostics_topic", runtime_diagnostics_topic);
    runtime_diagnostics_publisher =
        node->create_publisher<std_msgs::msg::Float32MultiArray>(runtime_diagnostics_topic, publisher_qos);
  }

  publish_lidar_acceleration = node->declare_parameter<bool>("publish_lidar_acceleration", publish_lidar_acceleration);
  if (publish_lidar_acceleration) {
    lidar_accel_publisher =
        node->create_publisher<geometry_msgs::msg::AccelStamped>("rko_lio/lidar_acceleration", publisher_qos);
  }

  publish_deskewed_scan = node->declare_parameter<bool>("publish_deskewed_scan", publish_deskewed_scan);
  if (publish_deskewed_scan) {
    deskewed_scan_topic = node->declare_parameter<std::string>("deskewed_scan_topic", deskewed_scan_topic);
    frame_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(deskewed_scan_topic, publisher_qos);
  }

  publish_local_map = node->declare_parameter<bool>("publish_local_map", publish_local_map);
  if (publish_local_map) {
    map_topic = node->declare_parameter<std::string>("map_topic", map_topic);
    publish_map_after = core::Secondsd(node->declare_parameter<double>("publish_map_after", publish_map_after.count()));
    map_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, publisher_qos);
    map_publish_thead = std::jthread([this]() { publish_map_loop(); });
  }

  // lio params
  core::LIO::Config lio_config{};
  lio_config.deskew = node->declare_parameter<bool>("deskew", lio_config.deskew);
  lio_config.max_iterations =
      static_cast<size_t>(node->declare_parameter<int>("max_iterations", static_cast<int>(lio_config.max_iterations)));
  lio_config.voxel_size = node->declare_parameter<double>("voxel_size", lio_config.voxel_size);
  lio_config.max_points_per_voxel =
      static_cast<int>(node->declare_parameter<int>("max_points_per_voxel", lio_config.max_points_per_voxel));
  lio_config.max_range = node->declare_parameter<double>("max_range", lio_config.max_range);
  lio_config.min_range = node->declare_parameter<double>("min_range", lio_config.min_range);
  lio_config.convergence_criterion =
      node->declare_parameter<double>("convergence_criterion", lio_config.convergence_criterion);
  lio_config.max_correspondance_distance =
      node->declare_parameter<double>("max_correspondance_distance", lio_config.max_correspondance_distance);
  lio_config.registration_error_model =
      node->declare_parameter<std::string>("registration_error_model", lio_config.registration_error_model);
  if (lio_config.registration_error_model != "point_to_point" &&
      lio_config.registration_error_model != "point_to_plane") {
    RCLCPP_WARN_STREAM(node->get_logger(),
                       "Unknown registration_error_model '" << lio_config.registration_error_model
                                                           << "'. Falling back to point_to_point.");
    lio_config.registration_error_model = "point_to_point";
  }
  lio_config.max_voxel_search_radius =
      node->declare_parameter<int>("max_voxel_search_radius", lio_config.max_voxel_search_radius);
  lio_config.plane_fit_neighbors =
      node->declare_parameter<int>("plane_fit_neighbors", lio_config.plane_fit_neighbors);
  lio_config.plane_fit_min_neighbors =
      node->declare_parameter<int>("plane_fit_min_neighbors", lio_config.plane_fit_min_neighbors);
  lio_config.plane_fit_max_distance =
      node->declare_parameter<double>("plane_fit_max_distance", lio_config.plane_fit_max_distance);
  lio_config.plane_fit_max_eigenvalue =
      node->declare_parameter<double>("plane_fit_max_eigenvalue", lio_config.plane_fit_max_eigenvalue);
  lio_config.point_to_plane_keypoint_stride =
      node->declare_parameter<int>("point_to_plane_keypoint_stride", lio_config.point_to_plane_keypoint_stride);
  if (lio_config.point_to_plane_keypoint_stride < 1) {
    lio_config.point_to_plane_keypoint_stride = 1;
  }
  lio_config.coarse_to_fine_registration =
      node->declare_parameter<bool>("coarse_to_fine_registration", lio_config.coarse_to_fine_registration);
  lio_config.coarse_voxel_size = node->declare_parameter<double>("coarse_voxel_size", lio_config.coarse_voxel_size);
  lio_config.coarse_max_correspondance_distance = node->declare_parameter<double>(
      "coarse_max_correspondance_distance", lio_config.coarse_max_correspondance_distance);
  lio_config.coarse_max_iterations = static_cast<size_t>(
      node->declare_parameter<int>("coarse_max_iterations", static_cast<int>(lio_config.coarse_max_iterations)));
  lio_config.enable_imu_pose_prior =
      node->declare_parameter<bool>("enable_imu_pose_prior", lio_config.enable_imu_pose_prior);
  lio_config.imu_pose_prior_translation_weight =
      node->declare_parameter<double>("imu_pose_prior_translation_weight", lio_config.imu_pose_prior_translation_weight);
  lio_config.imu_pose_prior_rotation_weight =
      node->declare_parameter<double>("imu_pose_prior_rotation_weight", lio_config.imu_pose_prior_rotation_weight);
  lio_config.enable_adaptive_imu_pose_prior =
      node->declare_parameter<bool>("enable_adaptive_imu_pose_prior", lio_config.enable_adaptive_imu_pose_prior);
  lio_config.adaptive_imu_pose_prior_condition =
      node->declare_parameter<double>("adaptive_imu_pose_prior_condition",
                                      lio_config.adaptive_imu_pose_prior_condition);
  lio_config.adaptive_imu_pose_prior_max_condition_ratio =
      node->declare_parameter<double>("adaptive_imu_pose_prior_max_condition_ratio",
                                      lio_config.adaptive_imu_pose_prior_max_condition_ratio);
  lio_config.adaptive_imu_pose_prior_max_translation_weight =
      node->declare_parameter<double>("adaptive_imu_pose_prior_max_translation_weight",
                                      lio_config.adaptive_imu_pose_prior_max_translation_weight);
  lio_config.adaptive_imu_pose_prior_max_rotation_weight =
      node->declare_parameter<double>("adaptive_imu_pose_prior_max_rotation_weight",
                                      lio_config.adaptive_imu_pose_prior_max_rotation_weight);
  lio_config.enable_degeneracy_damping =
      node->declare_parameter<bool>("enable_degeneracy_damping", lio_config.enable_degeneracy_damping);
  lio_config.degeneracy_damping_condition =
      node->declare_parameter<double>("degeneracy_damping_condition", lio_config.degeneracy_damping_condition);
  lio_config.degeneracy_damping_alpha =
      node->declare_parameter<double>("degeneracy_damping_alpha", lio_config.degeneracy_damping_alpha);
  lio_config.adaptive_degeneracy_damping =
      node->declare_parameter<bool>("adaptive_degeneracy_damping", lio_config.adaptive_degeneracy_damping);
  lio_config.degeneracy_damping_min_alpha =
      node->declare_parameter<double>("degeneracy_damping_min_alpha", lio_config.degeneracy_damping_min_alpha);
  lio_config.enable_eigen_degeneracy_projection =
      node->declare_parameter<bool>("enable_eigen_degeneracy_projection",
                                    lio_config.enable_eigen_degeneracy_projection);
  lio_config.eigen_degeneracy_projection_condition =
      node->declare_parameter<double>("eigen_degeneracy_projection_condition",
                                      lio_config.eigen_degeneracy_projection_condition);
  lio_config.eigen_degeneracy_projection_min_scale =
      node->declare_parameter<double>("eigen_degeneracy_projection_min_scale",
                                      lio_config.eigen_degeneracy_projection_min_scale);
  lio_config.enable_stationary_hold =
      node->declare_parameter<bool>("enable_stationary_hold", lio_config.enable_stationary_hold);
  lio_config.stationary_angular_velocity_threshold =
      node->declare_parameter<double>("stationary_angular_velocity_threshold",
                                      lio_config.stationary_angular_velocity_threshold);
  lio_config.stationary_linear_acceleration_threshold =
      node->declare_parameter<double>("stationary_linear_acceleration_threshold",
                                      lio_config.stationary_linear_acceleration_threshold);
  lio_config.stationary_max_translation_delta =
      node->declare_parameter<double>("stationary_max_translation_delta", lio_config.stationary_max_translation_delta);
  lio_config.stationary_max_rotation_delta =
      node->declare_parameter<double>("stationary_max_rotation_delta", lio_config.stationary_max_rotation_delta);
  lio_config.enable_vertical_spike_filter =
      node->declare_parameter<bool>("enable_vertical_spike_filter", lio_config.enable_vertical_spike_filter);
  lio_config.max_vertical_update_m =
      node->declare_parameter<double>("max_vertical_update_m", lio_config.max_vertical_update_m);
  lio_config.enable_rover_degeneracy_motion_constraint =
      node->declare_parameter<bool>("enable_rover_degeneracy_motion_constraint",
                                    lio_config.enable_rover_degeneracy_motion_constraint);
  lio_config.rover_degeneracy_motion_condition =
      node->declare_parameter<double>("rover_degeneracy_motion_condition",
                                      lio_config.rover_degeneracy_motion_condition);
  lio_config.rover_degeneracy_forward_scale =
      node->declare_parameter<double>("rover_degeneracy_forward_scale", lio_config.rover_degeneracy_forward_scale);
  lio_config.rover_degeneracy_lateral_scale =
      node->declare_parameter<double>("rover_degeneracy_lateral_scale", lio_config.rover_degeneracy_lateral_scale);
  lio_config.rover_degeneracy_vertical_scale =
      node->declare_parameter<double>("rover_degeneracy_vertical_scale", lio_config.rover_degeneracy_vertical_scale);
  lio_config.rover_degeneracy_roll_pitch_scale =
      node->declare_parameter<double>("rover_degeneracy_roll_pitch_scale",
                                      lio_config.rover_degeneracy_roll_pitch_scale);
  lio_config.rover_degeneracy_yaw_scale =
      node->declare_parameter<double>("rover_degeneracy_yaw_scale", lio_config.rover_degeneracy_yaw_scale);
  lio_config.max_num_threads =
      static_cast<int>(node->declare_parameter<int>("max_num_threads", lio_config.max_num_threads));
  lio_config.initialization_phase =
      node->declare_parameter<bool>("initialization_phase", lio_config.initialization_phase);
  lio_config.max_expected_jerk = node->declare_parameter<double>("max_expected_jerk", lio_config.max_expected_jerk);
  lio_config.gravity_magnitude = node->declare_parameter<double>("gravity_magnitude", lio_config.gravity_magnitude);
  lio_config.double_downsample = node->declare_parameter<bool>("double_downsample", lio_config.double_downsample);
  lio_config.min_beta = node->declare_parameter<double>("min_beta", lio_config.min_beta);
  lio_config.max_scan_delta_sec =
      node->declare_parameter<double>("max_scan_delta_sec", lio_config.max_scan_delta_sec);
  lio_config.enable_kidnap_relocalization =
      node->declare_parameter<bool>("enable_kidnap_relocalization", lio_config.enable_kidnap_relocalization);
  lio_config.reset_on_registration_failure =
      node->declare_parameter<bool>("reset_on_registration_failure", lio_config.reset_on_registration_failure);
  lio_config.recovery_min_failures =
      node->declare_parameter<int>("recovery_min_failures", lio_config.recovery_min_failures);
  lio_config.relocalize_after_scan_gap =
      node->declare_parameter<bool>("relocalize_after_scan_gap", lio_config.relocalize_after_scan_gap);
  lio_config.relocalization_min_correspondences =
      node->declare_parameter<int>("relocalization_min_correspondences", lio_config.relocalization_min_correspondences);
  lio_config.relocalization_min_inlier_ratio =
      node->declare_parameter<double>("relocalization_min_inlier_ratio", lio_config.relocalization_min_inlier_ratio);
  lio_config.relocalization_max_mean_error =
      node->declare_parameter<double>("relocalization_max_mean_error", lio_config.relocalization_max_mean_error);
  lio_config.relocalization_max_correspondance_distance = node->declare_parameter<double>(
      "relocalization_max_correspondance_distance", lio_config.relocalization_max_correspondance_distance);
  lio_config.relocalization_yaw_samples =
      node->declare_parameter<int>("relocalization_yaw_samples", lio_config.relocalization_yaw_samples);
  lio_config.relocalization_pose_stride =
      node->declare_parameter<int>("relocalization_pose_stride", lio_config.relocalization_pose_stride);
  lio_config.relocalization_min_pose_separation =
      node->declare_parameter<int>("relocalization_min_pose_separation", lio_config.relocalization_min_pose_separation);
  lio_config.relocalization_max_iterations =
      node->declare_parameter<int>("relocalization_max_iterations", lio_config.relocalization_max_iterations);
  lio = std::make_unique<core::LIO>(lio_config);

  // Timestamp processing params - lts for lidar time stamps, without having 100 char param names
  timestamp_proc_config.multiplier_to_seconds =
      node->declare_parameter<double>("lts_multiplier_to_seconds", timestamp_proc_config.multiplier_to_seconds);
  timestamp_proc_config.force_absolute =
      node->declare_parameter<bool>("lts_force_absolute", timestamp_proc_config.force_absolute);
  timestamp_proc_config.force_relative =
      node->declare_parameter<bool>("lts_force_relative", timestamp_proc_config.force_relative);

  // manually, if, define extrinsics
  parse_cli_extrinsics();

  RCLCPP_INFO_STREAM(node->get_logger(),
                     "Subscribed to IMU: "
                         << imu_topic << (!imu_frame.empty() ? " (frame " + imu_frame + ")" : "") << " and LiDAR: "
                         << lidar_topic << (!lidar_frame.empty() ? " (frame " + lidar_frame + ")" : "")
                         << ". Max number of threads: " << lio_config.max_num_threads << ". Publishing odometry to "
                         << odom_topic << " ( " << odom_frame
                         << " ) and acceleration "
                            "estimates to rko_lio/lidar_acceleration. Deskewing is "
                         << (lio->config.deskew ? "enabled" : "disabled") << "."
                         << (publish_deskewed_scan ? (" Publishing deskewed_cloud to " + deskewed_scan_topic + ".")
                                                   : ""));

  // disk logging
  dump_results = node->declare_parameter<bool>("dump_results", dump_results);
  results_dir = node->declare_parameter<std::string>("results_dir", results_dir);
  run_name = node->declare_parameter<std::string>("run_name", run_name);
  rclcpp::on_shutdown([this]() {
    // i'll need to look into rclcpp::Context a bit more, but for now i think this callback should be called before
    // anything gets destroyed.
    if (dump_results) {
      // it is probably still a veery good idea to make dump_results_to_disk noexcept
      dump_results_to_disk(results_dir, run_name);
    }
  });

  registration_thread = std::jthread([this]() { registration_loop(); });

  RCLCPP_INFO(node->get_logger(), "RKO LIO Node is up!");
}

void Node::parse_cli_extrinsics() {
  auto parse_extrinsic = [this](const std::string& name, Sophus::SE3d& extrinsic) {
    const std::string param_name = "extrinsic_" + name + "2base_quat_xyzw_xyz";
    const std::vector<double> vec = node->declare_parameter<std::vector<double>>(param_name, std::vector<double>{});

    if (vec.size() != 7) {
      if (!vec.empty()) {
        RCLCPP_WARN_STREAM(node->get_logger(),
                           "Parameter 'extrinsic_"
                               << name << "2base_quat_xyzw_xyz' is set but has wrong size: " << vec.size()
                               << ". Expected 7 (qx, qy, qz, qw, x, y, z). check the value: "
                               << Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size()).transpose());
      }
      return false;
    }
    Eigen::Quaterniond q(vec[3], vec[0], vec[1], vec[2]); // qw, qx, qy, qz
    if (q.norm() < 1e-6) {
      throw std::runtime_error(name + " extrinsic quaternion has zero norm");
    }
    extrinsic = Sophus::SE3d(q, Eigen::Vector3d(vec[4], vec[5], vec[6]));
    RCLCPP_INFO_STREAM(node->get_logger(), "Parsed " << name << " extrinsic as: " << extrinsic.log().transpose());
    return true;
  };
  const bool imu_ok = parse_extrinsic("imu", extrinsic_imu2base);
  const bool lidar_ok = parse_extrinsic("lidar", extrinsic_lidar2base);
  extrinsics_set = imu_ok && lidar_ok;
}

bool Node::check_and_set_extrinsics() {
  if (extrinsics_set) {
    return true;
  }
  const std::optional<Sophus::SE3d> imu_transform = utils::get_transform(tf_buffer, imu_frame, base_frame, 0s);
  if (!imu_transform) {
    return false;
  }
  const std::optional<Sophus::SE3d> lidar_transform = utils::get_transform(tf_buffer, lidar_frame, base_frame, 0s);
  if (!lidar_transform) {
    return false;
  }
  extrinsic_imu2base = imu_transform.value();
  extrinsic_lidar2base = lidar_transform.value();
  extrinsics_set = true;
  return true;
}

void Node::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) {
  if (imu_frame.empty()) {
    if (imu_msg->header.frame_id.empty() && !extrinsics_set) {
      throw std::runtime_error("IMU message header has no frame id and we need it to query TF for the extrinsics. "
                               "Either specify the frame id or the extrinsic manually.");
    }
    imu_frame = imu_msg->header.frame_id;
    RCLCPP_INFO_STREAM(node->get_logger(), "Parsed the imu frame id as: " << imu_frame);
  }
  if (!check_and_set_extrinsics()) {
    // we assume that extrinsics are static. if they change, its better to query the tf directly in the registration
    // loop for each message being processed asynchronously.
    return;
  }
  {
    std::lock_guard lock(buffer_mutex);
    imu_buffer.emplace(imu_msg_to_imu_data(*imu_msg));
    atomic_can_process = !lidar_buffer.empty() && imu_buffer.back().time > lidar_buffer.front().timestamps.max;
  }
  if (atomic_can_process) {
    sync_condition_variable.notify_one();
  }
}

void Node::lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg) {
  if (lidar_frame.empty()) {
    if (lidar_msg->header.frame_id.empty() && !extrinsics_set) {
      throw std::runtime_error("LiDAR message header has no frame id and we need it to query TF for the extrinsics. "
                               "Either specify the frame id or the extrinsic manually.");
    }
    lidar_frame = lidar_msg->header.frame_id;
    RCLCPP_INFO_STREAM(node->get_logger(), "Parsed the lidar frame id as: " << lidar_frame);
  }
  if (!check_and_set_extrinsics()) {
    return;
  }
  {
    std::lock_guard lock(buffer_mutex);
    if (lidar_buffer.size() >= max_lidar_buffer_size) {
      if (drop_oldest_lidar_when_full) {
        lidar_buffer.pop();
        ++dropped_lidar_frames;
        RCLCPP_WARN_STREAM_THROTTLE(
            node->get_logger(), *node->get_clock(), 1000,
            "Registration lidar buffer limit reached. Dropping oldest queued frame to keep frontend fresh.");
      } else {
        RCLCPP_WARN_STREAM_THROTTLE(
            node->get_logger(), *node->get_clock(), 1000,
            "Registration lidar buffer limit reached. Dropping newest frame.");
        ++dropped_lidar_frames;
        sync_condition_variable.notify_one();
        return;
      }
    }
  }
  try {
    const auto& [timestamps, scan] = std::invoke([&]() -> std::tuple<core::Timestamps, core::Vector3dVector> {
      const core::Secondsd& header_stamp = utils::ros_time_to_seconds(lidar_msg->header.stamp);
      if (lio->config.deskew) {
        const auto& [scan, raw_timestamps] = utils::point_cloud2_to_eigen_with_timestamps(lidar_msg);
        const core::Timestamps& timestamps =
            core::process_timestamps(raw_timestamps, header_stamp, timestamp_proc_config);
        return {timestamps, scan};
      } else {
        RCLCPP_WARN_STREAM_ONCE(node->get_logger(),
                                "Deskewing is disabled. Populating timestamps with static header time.");
        const core::Vector3dVector scan = utils::point_cloud2_to_eigen(lidar_msg);
        return {{.min = header_stamp, .max = header_stamp, .times = core::TimestampVector(scan.size(), header_stamp)},
                scan};
      }
    });

    {
      std::lock_guard lock(buffer_mutex);
      lidar_buffer.emplace(timestamps, scan);
      atomic_can_process = !imu_buffer.empty() && imu_buffer.back().time > lidar_buffer.front().timestamps.max;
    }
    if (atomic_can_process) {
      sync_condition_variable.notify_one();
    }
  } catch (const std::invalid_argument& ex) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Encountered error, dropping frame: Error. " << ex.what());
  }
}

void Node::registration_loop() {
  while (rclcpp::ok() && atomic_node_running) {
    SCOPED_PROFILER("ROS Registration Loop");
    std::unique_lock buffer_lock(buffer_mutex);
    sync_condition_variable.wait(buffer_lock, [this]() { return !atomic_node_running || atomic_can_process; });
    if (!atomic_node_running) {
      // node could have been killed after waiting on the cv
      break;
    }
    core::LidarFrame frame = std::move(lidar_buffer.front());
    lidar_buffer.pop();
    const size_t queued_lidar_frames = lidar_buffer.size();
    const size_t dropped_frames = dropped_lidar_frames;
    atomic_registration_active = true;
    const auto& [timestamps, scan] = frame;
    const auto& [start_stamp, end_stamp, time_vector] = timestamps;
    for (; !imu_buffer.empty() && imu_buffer.front().time < end_stamp; imu_buffer.pop()) {
      const core::ImuControl& imu_data = imu_buffer.front();
      lio->add_imu_measurement(extrinsic_imu2base, imu_data);
    }
    // check if there are more messages buffered already
    atomic_can_process =
        !imu_buffer.empty() && !lidar_buffer.empty() && imu_buffer.back().time > lidar_buffer.front().timestamps.max;
    buffer_lock.unlock(); // we dont touch the buffers anymore

    const double scan_age_before_sec = node->now().seconds() - end_stamp.count();
    const auto processing_start = std::chrono::steady_clock::now();
    size_t deskewed_points = 0;
    bool registration_success = false;
    try {
      const core::Vector3dVector deskewed_frame = std::invoke([&]() {
        if (publish_local_map) {
          std::lock_guard lock(local_map_mutex); // publish_map thread might access simultaneously
          return lio->register_scan(extrinsic_lidar2base, scan, time_vector);
        } else {
          return lio->register_scan(extrinsic_lidar2base, scan, time_vector);
        }
      });
      deskewed_points = deskewed_frame.size();

      if (!deskewed_frame.empty()) {
        registration_success = true;
        // TODO: first frame is skipped and an empty frame is returned. improve how we handle this
        if (publish_deskewed_scan) {
          std_msgs::msg::Header header;
          header.frame_id = lidar_frame;
          header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(end_stamp).count());
          frame_publisher->publish(utils::eigen_to_point_cloud2(deskewed_frame, header));
        }
        publish_odometry(lio->lidar_state, end_stamp);
        if (publish_registration_diagnostics) {
          publish_registration_metrics(lio->last_registration_diagnostics);
        }
        if (publish_lidar_acceleration) {
          publish_lidar_accel(lio->lidar_state.linear_acceleration, end_stamp);
        }
      }
    } catch (const std::exception& ex) {
      // Catch both std::invalid_argument (Keypoints=0 / Δt) and std::runtime_error
      // (Number of correspondences=0). Both are recoverable on kidnap-style bags.
      RCLCPP_ERROR_STREAM(node->get_logger(), "Encountered error, dropping frame. Error: " << ex.what());
    }
    const double processing_time_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - processing_start).count();
    const double scan_age_after_sec = node->now().seconds() - end_stamp.count();
    publish_runtime_metrics(processing_time_sec,
                            scan_age_before_sec,
                            scan_age_after_sec,
                            queued_lidar_frames,
                            scan.size(),
                            deskewed_points,
                            dropped_frames,
                            registration_success);
    RCLCPP_INFO_STREAM_THROTTLE(node->get_logger(),
                                *node->get_clock(),
                                1000,
                                "RKO runtime: process=" << processing_time_sec << "s, scan_age_before="
                                                        << scan_age_before_sec << "s, scan_age_after="
                                                        << scan_age_after_sec << "s, queued_lidar="
                                                        << queued_lidar_frames << ", raw_points=" << scan.size()
                                                        << ", deskewed_points=" << deskewed_points
                                                        << ", dropped_lidar=" << dropped_frames
                                                        << ", success=" << registration_success);
    atomic_registration_active = false;
  }
  atomic_registration_active = false;
  atomic_node_running = false;
}

void Node::publish_odometry(const core::State& state, const core::Secondsd& stamp) const {
  const std::string_view from_frame = base_frame;
  const std::string_view to_frame = odom_frame;
  // tf message
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count());
  if (invert_odom_tf) {
    transform_msg.header.frame_id = from_frame;
    transform_msg.child_frame_id = to_frame;
    transform_msg.transform = utils::sophus_to_transform(state.pose.inverse());
  } else {
    transform_msg.header.frame_id = to_frame;
    transform_msg.child_frame_id = from_frame;
    transform_msg.transform = utils::sophus_to_transform(state.pose);
  }
  tf_broadcaster->sendTransform(transform_msg);

  // odometry msg
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count());
  odom_msg.header.frame_id = to_frame;
  odom_msg.child_frame_id = from_frame;
  odom_msg.pose.pose = utils::sophus_to_pose(state.pose);
  utils::eigen_vector3d_to_ros_xyz(state.velocity, odom_msg.twist.twist.linear);
  utils::eigen_vector3d_to_ros_xyz(state.angular_velocity, odom_msg.twist.twist.angular);
  odom_publisher->publish(odom_msg);
}

void Node::publish_registration_metrics(const core::LIO::RegistrationDiagnostics& diagnostics) const {
  if (!registration_diagnostics_publisher) {
    return;
  }
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {
      diagnostics.valid ? 1.0f : 0.0f,
      static_cast<float>(diagnostics.keypoints),
      static_cast<float>(diagnostics.correspondences),
      static_cast<float>(diagnostics.inlier_ratio),
      static_cast<float>(diagnostics.mean_error),
      static_cast<float>(diagnostics.hessian_min_eigenvalue),
      static_cast<float>(diagnostics.hessian_max_eigenvalue),
      static_cast<float>(diagnostics.hessian_condition),
      static_cast<float>(diagnostics.consecutive_registration_failures),
      diagnostics.coarse_to_fine_used ? 1.0f : 0.0f,
      static_cast<float>(diagnostics.degeneracy_damping_alpha_applied),
      static_cast<float>(diagnostics.eigen_degeneracy_min_scale_applied),
      static_cast<float>(diagnostics.adaptive_imu_pose_prior_translation_weight_applied),
      static_cast<float>(diagnostics.adaptive_imu_pose_prior_rotation_weight_applied),
      diagnostics.rover_degeneracy_motion_constraint_applied ? 1.0f : 0.0f,
  };
  registration_diagnostics_publisher->publish(msg);
}

void Node::publish_runtime_metrics(double processing_time_sec,
                                   double scan_age_before_sec,
                                   double scan_age_after_sec,
                                   size_t queued_lidar_frames,
                                   size_t raw_points,
                                   size_t deskewed_points,
                                   size_t dropped_frames,
                                   bool success) const {
  if (!runtime_diagnostics_publisher) {
    return;
  }
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {
      static_cast<float>(processing_time_sec),
      static_cast<float>(scan_age_before_sec),
      static_cast<float>(scan_age_after_sec),
      static_cast<float>(queued_lidar_frames),
      static_cast<float>(raw_points),
      static_cast<float>(deskewed_points),
      static_cast<float>(dropped_frames),
      success ? 1.0f : 0.0f,
  };
  runtime_diagnostics_publisher->publish(msg);
}

void Node::publish_lidar_accel(const Eigen::Vector3d& acceleration, const core::Secondsd& stamp) const {
  auto accel_msg = geometry_msgs::msg::AccelStamped();
  accel_msg.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count());
  accel_msg.header.frame_id = base_frame;
  utils::eigen_vector3d_to_ros_xyz(acceleration, accel_msg.accel.linear);
  lidar_accel_publisher->publish(accel_msg);
}

void Node::publish_map_loop() {
  while (atomic_node_running) {
    std::this_thread::sleep_for(publish_map_after);
    std::unique_lock lock(local_map_mutex);
    if (lio->map.Empty()) {
      RCLCPP_WARN_ONCE(node->get_logger(), "Local map publish thread: Local map is empty.");
      continue;
    }
    const core::Vector3dVector map_points = lio->map.Pointcloud();
    lock.unlock(); // we don't access the local map anymore
    std_msgs::msg::Header map_header;
    map_header.stamp = node->now();
    map_header.frame_id = odom_frame;
    map_publisher->publish(utils::eigen_to_point_cloud2(map_points, map_header));
  }
}

Node::~Node() {
  atomic_node_running = false;
  sync_condition_variable.notify_all();
}

void Node::dump_results_to_disk(const std::filesystem::path& results_dir, const std::string& run_name) const {
  try {
    std::filesystem::create_directories(results_dir); // no error if exists
    int index = 0;
    std::filesystem::path output_dir = results_dir / (run_name + "_" + std::to_string(index));
    while (std::filesystem::exists(output_dir)) {
      ++index;
      output_dir = results_dir / (run_name + "_" + std::to_string(index));
    }
    std::filesystem::create_directory(output_dir);
    const std::filesystem::path output_file = output_dir / (run_name + "_tum_" + std::to_string(index) + ".txt");
    // dump poses
    if (std::ofstream file(output_file); file.is_open()) {
      for (const auto& [timestamp, pose] : lio->poses_with_timestamps) {
        const Eigen::Vector3d& translation = pose.translation();
        const Eigen::Quaterniond& quaternion = pose.so3().unit_quaternion();
        file << std::fixed << std::setprecision(6) << timestamp.count() << " " << translation.x() << " "
             << translation.y() << " " << translation.z() << " " << quaternion.x() << " " << quaternion.y() << " "
             << quaternion.z() << " " << quaternion.w() << "\n";
      }
      std::cout << "Poses written to " << std::filesystem::absolute(output_file) << "\n";
    }
    // dump config
    const nlohmann::json json_config = {{"config", lio->config}};
    const std::filesystem::path config_file = output_dir / "config.json";
    if (std::ofstream file(config_file); file.is_open()) {
      file << json_config.dump(4);
      std::cout << "Configuration written to " << config_file << "\n";
    }
  } catch (const std::filesystem::filesystem_error& ex) {
    std::cerr << "[WARNING] Cannot write files to disk, encountered filesystem error: " << ex.what() << "\n";
  }
}

} // namespace rko_lio::ros
