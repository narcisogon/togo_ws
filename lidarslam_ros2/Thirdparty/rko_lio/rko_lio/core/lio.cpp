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

#include "lio.hpp"
#include "preprocess_scan.hpp"
#include "profiler.hpp"
#include "util.hpp"
#include "voxel_down_sample.hpp"
// other
#include <sophus/se3.hpp>
// tbb
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>
// stl
#include <algorithm>
#include <cmath>
#include <Eigen/Eigenvalues>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace {
constexpr double EPSILON = 1e-8;
constexpr auto EPSILON_TIME = std::chrono::nanoseconds(10);
using namespace rko_lio::core;

double clamp01(const double value);

inline void transform_points(const Sophus::SE3d& T, Vector3dVector& points) {
  std::transform(points.begin(), points.end(), points.begin(), [&](const auto& point) { return T * point; });
}

inline Eigen::Vector3d gravity_vector(const double magnitude) {
  return {0.0, 0.0, -std::max(0.0, magnitude)};
}

using LinearSystem = std::tuple<Eigen::Matrix6d, Eigen::Vector6d, double>;

struct PlaneFit {
  bool valid = false;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double smallest_eigenvalue = std::numeric_limits<double>::max();
};

PlaneFit fit_local_plane(const Vector3dVector& neighbors, const double max_smallest_eigenvalue) {
  if (neighbors.size() < 3) {
    return {};
  }
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d& point : neighbors) {
    centroid += point;
  }
  centroid /= static_cast<double>(neighbors.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const Eigen::Vector3d& point : neighbors) {
    const Eigen::Vector3d centered = point - centroid;
    covariance += centered * centered.transpose();
  }
  covariance /= static_cast<double>(neighbors.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return {};
  }
  const Eigen::Vector3d eigenvalues = solver.eigenvalues();
  const double smallest = eigenvalues.x();
  if (smallest > max_smallest_eigenvalue) {
    return {};
  }
  Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
  return PlaneFit{
      .valid = true,
      .centroid = centroid,
      .normal = normal,
      .smallest_eigenvalue = smallest,
  };
}

LinearSystem build_icp_linear_system(const Sophus::SE3d& current_pose,
                                     const rko_lio::core::Vector3dVector& frame,
                                     const rko_lio::core::SparseVoxelGrid& voxel_map,
                                     const LIO::Config& config,
                                     const int voxel_search_radius = 1) {
  auto linear_system_reduce = [](LinearSystem lhs, const LinearSystem& rhs) {
    auto& [lhs_H, lhs_b, lhs_chi] = lhs;
    const auto& [rhs_H, rhs_b, rhs_chi] = rhs;
    lhs_H += rhs_H;
    lhs_b += rhs_b;
    lhs_chi += rhs_chi;
    return lhs;
  };

  auto linear_system_for_one_point = [](const Eigen::Vector3d& source, const Eigen::Vector3d& target) {
    Eigen::Matrix3_6d J_r;
    J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source);
    const Eigen::Vector3d residual = source - target;
    return LinearSystem(J_r.transpose() * J_r,      // JTJ
                        J_r.transpose() * residual, // JTr
                        residual.squaredNorm());    // chi
  };

  auto linear_system_for_one_plane = [](const Eigen::Vector3d& source, const PlaneFit& plane) {
    const Eigen::RowVector3d n_t = plane.normal.transpose();
    Eigen::Matrix<double, 1, 6> J_r;
    J_r.block<1, 3>(0, 0) = n_t;
    J_r.block<1, 3>(0, 3) = n_t * (-1.0 * Sophus::SO3d::hat(source).matrix());
    const double residual = plane.normal.dot(source - plane.centroid);
    return LinearSystem(J_r.transpose() * J_r,      // JTJ
                        J_r.transpose() * residual, // JTr
                        residual * residual);       // chi
  };

  // The only parallel part
  using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
  std::atomic<int> correspondances_counter = 0;
  const int point_to_plane_stride = std::max(1, config.point_to_plane_keypoint_stride);
  const auto& [H_icp, b_icp, chi_icp] = tbb::parallel_reduce(
      // Range
      tbb::blocked_range<points_iterator>{frame.cbegin(), frame.cend()},
      // Identity
      LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero(), 0.0),
      // 1st Lambda: Parallel computation
      [&](const tbb::blocked_range<points_iterator>& r, LinearSystem J) -> LinearSystem {
        return std::transform_reduce(r.begin(), r.end(), J, linear_system_reduce, [&](const auto& point) {
          // Compute data association and linear system
          const Eigen::Vector3d transformed_point = current_pose * point;
          if (config.registration_error_model == "point_to_plane") {
            const auto point_index = static_cast<std::ptrdiff_t>(&point - frame.data());
            if (point_to_plane_stride > 1 && (point_index % point_to_plane_stride) != 0) {
              return LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero(), 0.0);
            }
            const Vector3dVector neighbors = voxel_map.GetNearestNeighbors(
                transformed_point,
                voxel_search_radius,
                config.plane_fit_max_distance,
                config.plane_fit_neighbors);
            if (neighbors.size() >= static_cast<size_t>(config.plane_fit_min_neighbors)) {
              PlaneFit plane = fit_local_plane(neighbors, config.plane_fit_max_eigenvalue);
              if (plane.valid) {
                if (plane.normal.dot(transformed_point - plane.centroid) < 0.0) {
                  plane.normal *= -1.0;
                }
                correspondances_counter++;
                return linear_system_for_one_plane(transformed_point, plane);
              }
            }
          } else {
            const auto& [closest_neighbor, distance] =
                voxel_map.GetClosestNeighbor(transformed_point, voxel_search_radius);
            if (distance < config.max_correspondance_distance) {
              correspondances_counter++;
              return linear_system_for_one_point(transformed_point, closest_neighbor);
            }
          }
          // TODO (meher): additional 0 add flops, which may hurt single threaded perf slightly
          return LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero(), 0.0);
        });
      },
      // 2nd Lambda: Parallel reduction of the private Jacobians
      linear_system_reduce);

  if (correspondances_counter == 0) {
    throw std::runtime_error("Number of correspondences are 0.");
  }

  return {H_icp / correspondances_counter, b_icp / correspondances_counter, 0.5 * chi_icp};
}

LinearSystem build_orientation_linear_system(const Sophus::SE3d& current_pose,
                                             const Eigen::Vector3d& local_gravity_estimate,
                                             const Eigen::Vector3d& gravity) {
  const Sophus::SO3d& current_rotation = current_pose.so3();
  const Eigen::Vector3d predicted_gravity =
      current_rotation.inverse() * (-1 * gravity); // points upwards, same as local_gravity_estimate
  const Eigen::Vector3d residual = predicted_gravity - local_gravity_estimate;

  Eigen::Matrix3_6d J_ori = Eigen::Matrix3_6d::Zero();
  J_ori.block<3, 3>(0, 3) = current_rotation.inverse().matrix() * Sophus::SO3d::hat(-1 * gravity).matrix();

  return LinearSystem{J_ori.transpose() * J_ori, J_ori.transpose() * residual, 0.5 * residual.squaredNorm()};
}

LinearSystem build_pose_prior_linear_system(const Sophus::SE3d& current_pose,
                                            const Sophus::SE3d& prior_pose,
                                            const double translation_weight,
                                            const double rotation_weight) {
  Eigen::Vector6d residual = (current_pose * prior_pose.inverse()).log();
  Eigen::Matrix6d H = Eigen::Matrix6d::Zero();
  H.topLeftCorner<3, 3>().diagonal().setConstant(std::max(0.0, translation_weight));
  H.bottomRightCorner<3, 3>().diagonal().setConstant(std::max(0.0, rotation_weight));
  const double chi = 0.5 * (residual.transpose() * H * residual)(0, 0);
  return LinearSystem{H, H * residual, chi};
}

struct HessianAnalysis {
  bool valid = false;
  double min_eigenvalue = 0.0;
  double max_eigenvalue = 0.0;
  double condition = std::numeric_limits<double>::infinity();
  Eigen::Vector6d eigenvalues = Eigen::Vector6d::Zero();
  Eigen::Matrix6d eigenvectors = Eigen::Matrix6d::Identity();
};

HessianAnalysis analyze_hessian(const Eigen::Matrix6d& H) {
  HessianAnalysis analysis;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix6d> solver(H);
  if (solver.info() != Eigen::Success) {
    return analysis;
  }
  analysis.valid = true;
  analysis.eigenvalues = solver.eigenvalues();
  analysis.eigenvectors = solver.eigenvectors();
  analysis.min_eigenvalue = analysis.eigenvalues.minCoeff();
  analysis.max_eigenvalue = analysis.eigenvalues.maxCoeff();
  analysis.condition = analysis.min_eigenvalue > 1e-9
                           ? analysis.max_eigenvalue / analysis.min_eigenvalue
                           : std::numeric_limits<double>::infinity();
  return analysis;
}

double degeneracy_severity(const double condition, const double start_condition, const double max_ratio) {
  if (!std::isfinite(condition) || start_condition <= 0.0) {
    return 1.0;
  }
  const double ratio = condition / start_condition;
  if (ratio <= 1.0) {
    return 0.0;
  }
  const double denom = std::max(1e-6, max_ratio - 1.0);
  return std::clamp((ratio - 1.0) / denom, 0.0, 1.0);
}

std::pair<double, double> adaptive_pose_prior_weights(const LIO::Config& config,
                                                      const HessianAnalysis& analysis) {
  double translation_weight = config.imu_pose_prior_translation_weight;
  double rotation_weight = config.imu_pose_prior_rotation_weight;
  if (!config.enable_adaptive_imu_pose_prior) {
    return {translation_weight, rotation_weight};
  }

  const double start_condition = config.adaptive_imu_pose_prior_condition > 0.0
                                     ? config.adaptive_imu_pose_prior_condition
                                     : config.degeneracy_damping_condition;
  const double severity =
      degeneracy_severity(analysis.condition, start_condition, config.adaptive_imu_pose_prior_max_condition_ratio);
  translation_weight += severity *
                        (std::max(translation_weight, config.adaptive_imu_pose_prior_max_translation_weight) -
                         translation_weight);
  rotation_weight += severity *
                     (std::max(rotation_weight, config.adaptive_imu_pose_prior_max_rotation_weight) -
                      rotation_weight);
  return {translation_weight, rotation_weight};
}

Eigen::Vector6d project_degenerate_update(const Eigen::Vector6d& dx,
                                          const LIO::Config& config,
                                          const HessianAnalysis& analysis,
                                          double& min_scale_applied) {
  min_scale_applied = 1.0;
  if (!config.enable_eigen_degeneracy_projection || !analysis.valid || analysis.max_eigenvalue <= 1e-9) {
    return dx;
  }

  const double condition_threshold = config.eigen_degeneracy_projection_condition > 0.0
                                         ? config.eigen_degeneracy_projection_condition
                                         : config.degeneracy_damping_condition;
  if (condition_threshold <= 0.0 || analysis.condition < condition_threshold) {
    return dx;
  }

  const double min_scale = clamp01(config.eigen_degeneracy_projection_min_scale);
  Eigen::Vector6d projected = Eigen::Vector6d::Zero();
  for (int i = 0; i < 6; ++i) {
    const double lambda = analysis.eigenvalues[i];
    const double direction_condition =
        lambda > 1e-9 ? analysis.max_eigenvalue / lambda : std::numeric_limits<double>::infinity();
    double scale = 1.0;
    if (!std::isfinite(direction_condition) || direction_condition >= condition_threshold) {
      scale = std::isfinite(direction_condition)
                  ? std::clamp(condition_threshold / direction_condition, min_scale, 1.0)
                  : min_scale;
    }
    min_scale_applied = std::min(min_scale_applied, scale);
    projected += scale * analysis.eigenvectors.col(i).dot(dx) * analysis.eigenvectors.col(i);
  }
  return projected;
}

Sophus::SE3d icp(const Vector3dVector& frame,
                 const SparseVoxelGrid& voxel_map,
                 const Sophus::SE3d& initial_guess,
                 const LIO::Config& config,
                 const std::optional<AccelInfo>& optional_accel_info,
                 const std::optional<Sophus::SE3d>& optional_pose_prior = std::nullopt,
                 const int voxel_search_radius = 1,
                 LIO::RegistrationDiagnostics* diagnostics = nullptr) {
  // in case config disables it, or we don't have valid IMU information for this icp loop, beta is -1
  const double beta = (config.min_beta > 0 && optional_accel_info.has_value())
                          ? (config.min_beta * (1 + optional_accel_info->accel_mag_variance))
                          : -1;

  Sophus::SE3d current_pose = initial_guess;

  for (size_t i = 0; i < config.max_iterations; ++i) {
    const auto& [H_icp, b_icp, chi_icp] =
        build_icp_linear_system(current_pose, frame, voxel_map, config, voxel_search_radius);
    const HessianAnalysis hessian_analysis = analyze_hessian(H_icp);

    const auto& [H, b, chi] = std::invoke([&]() -> LinearSystem {
      if (beta >= 0) {
        const auto& [H_ori, b_ori, chi_ori] =
            build_orientation_linear_system(
                current_pose,
                optional_accel_info->local_gravity_estimate,
                gravity_vector(config.gravity_magnitude));
        Eigen::Matrix6d H = H_icp + H_ori / beta;
        Eigen::Vector6d b = b_icp + b_ori / beta;
        double chi = chi_icp + chi_ori / beta;
        if (config.enable_imu_pose_prior && optional_pose_prior.has_value()) {
          const auto [translation_weight, rotation_weight] =
              adaptive_pose_prior_weights(config, hessian_analysis);
          const auto& [H_prior, b_prior, chi_prior] =
              build_pose_prior_linear_system(current_pose,
                                             optional_pose_prior.value(),
                                             translation_weight,
                                             rotation_weight);
          H += H_prior;
          b += b_prior;
          chi += chi_prior;
          if (diagnostics != nullptr) {
            diagnostics->adaptive_imu_pose_prior_translation_weight_applied = translation_weight;
            diagnostics->adaptive_imu_pose_prior_rotation_weight_applied = rotation_weight;
          }
        }
        return {H, b, chi};
      }
      if (config.enable_imu_pose_prior && optional_pose_prior.has_value()) {
        const auto [translation_weight, rotation_weight] =
            adaptive_pose_prior_weights(config, hessian_analysis);
        const auto& [H_prior, b_prior, chi_prior] =
            build_pose_prior_linear_system(current_pose,
                                           optional_pose_prior.value(),
                                           translation_weight,
                                           rotation_weight);
        if (diagnostics != nullptr) {
          diagnostics->adaptive_imu_pose_prior_translation_weight_applied = translation_weight;
          diagnostics->adaptive_imu_pose_prior_rotation_weight_applied = rotation_weight;
        }
        return {H_icp + H_prior, b_icp + b_prior, chi_icp + chi_prior};
      }
      return {H_icp, b_icp, chi_icp};
    });

    const Eigen::Vector6d raw_dx = H.ldlt().solve(-b);
    double min_projection_scale = 1.0;
    const Eigen::Vector6d dx = project_degenerate_update(raw_dx, config, hessian_analysis, min_projection_scale);
    if (diagnostics != nullptr) {
      diagnostics->eigen_degeneracy_min_scale_applied = min_projection_scale;
    }
    current_pose = Sophus::SE3d::exp(dx) * current_pose;

    if (dx.norm() < config.convergence_criterion || i == (config.max_iterations - 1)) {
      // TODO: proper debug logging
      // std::cout << "iter " << i << ", beta: " << beta << ", chi: " << chi << ", num_assoc: " <<
      // correspondences.size() << "\n";
      break;
    }
  }
  return current_pose;
}

struct AlignmentStats {
  int correspondences = 0;
  double inlier_ratio = 0.0;
  double mean_error = std::numeric_limits<double>::max();
};

AlignmentStats evaluate_alignment(const Sophus::SE3d& pose,
                                  const Vector3dVector& frame,
                                  const SparseVoxelGrid& voxel_map,
                                  const double max_correspondance_distance,
                                  const int voxel_search_radius) {
  if (frame.empty()) {
    return {};
  }
  int correspondences = 0;
  double error_sum = 0.0;
  for (const Eigen::Vector3d& point : frame) {
    const Eigen::Vector3d transformed_point = pose * point;
    const auto& [closest_neighbor, distance] = voxel_map.GetClosestNeighbor(transformed_point, voxel_search_radius);
    (void)closest_neighbor;
    if (distance < max_correspondance_distance) {
      ++correspondences;
      error_sum += distance;
    }
  }
  if (correspondences == 0) {
    return {};
  }
  return {
      .correspondences = correspondences,
      .inlier_ratio = static_cast<double>(correspondences) / static_cast<double>(frame.size()),
      .mean_error = error_sum / static_cast<double>(correspondences),
  };
}

int voxel_search_radius_for_distance(const SparseVoxelGrid& voxel_map,
                                     const double max_correspondance_distance,
                                     const int max_voxel_search_radius) {
  const double voxel_size = std::max(1e-6, voxel_map.voxel_size_);
  const int radius = std::max(1, static_cast<int>(std::ceil(max_correspondance_distance / voxel_size)) + 1);
  return max_voxel_search_radius > 0 ? std::min(radius, max_voxel_search_radius) : radius;
}

Sophus::SO3d yaw_rotation(const double yaw_rad) {
  return Sophus::SO3d::exp(Eigen::Vector3d(0.0, 0.0, yaw_rad));
}

inline Sophus::SO3d align_accel_to_z_world(const Eigen::Vector3d& accel) {
  //  unobservable in the gravity direction, and the z in R.log() will always be 0
  const Eigen::Vector3d z_world = {0.0, 0.0, 1.0};
  const Eigen::Quaterniond quat_accel = Eigen::Quaterniond::FromTwoVectors(accel, z_world);
  return Sophus::SO3d(quat_accel);
}

double clamp01(const double value) {
  return std::clamp(value, 0.0, 1.0);
}
} // namespace

// ==========================
//   actual LIO class stuff
// ==========================

namespace rko_lio::core {

// ==========================
//          private
// ==========================

void LIO::initialize(const Secondsd lidar_time) {
  if (interval_stats.imu_count == 0) {
    std::cerr << "[WARNING] Cannot initialize. No imu measurements received.\n";
    // lidar_state.time has the time from the previous lidar, which we didn't log if init_phase was on
    poses_with_timestamps.emplace_back(lidar_state.time, lidar_state.pose);
    _initialized = true;
    return;
  }

  const Eigen::Vector3d avg_accel = interval_stats.imu_acceleration_sum / interval_stats.imu_count;
  const Eigen::Vector3d avg_gyro = interval_stats.angular_velocity_sum / interval_stats.imu_count;

  _imu_local_rotation = align_accel_to_z_world(avg_accel);
  _imu_local_rotation_time = lidar_time;
  lidar_state.pose.so3() = _imu_local_rotation;

  // lidar_state.time has the time from the previous lidar, which we didn't log if init_phase was on
  poses_with_timestamps.emplace_back(lidar_state.time, lidar_state.pose);

  // the pose for the current time gets logged at the end of register_scan in the typical fashion
  lidar_state.time = lidar_time;

  const Eigen::Vector3d local_gravity =
      _imu_local_rotation.inverse() * gravity_vector(config.gravity_magnitude);
  imu_bias.accelerometer = avg_accel + local_gravity;
  imu_bias.gyroscope = avg_gyro;

  _initialized = true;
  std::cout << "[INFO] Odometry map frame initialized using " << interval_stats.imu_count
            << " IMU measurements. Estimated initial rotation [se(3)] is " << _imu_local_rotation.log().transpose()
            << "\n";
  std::cout << "[INFO] Estimated accel bias: " << imu_bias.accelerometer.transpose()
            << ", gyro bias: " << imu_bias.gyroscope.transpose() << "\n";
}

// use the acceleration kalman filter to compute the two values we need for ori. reg.
std::optional<AccelInfo> LIO::get_accel_info(const Sophus::SO3d& rotation_estimate, const Secondsd& time) {
  if (interval_stats.imu_count <= 1) {
    std::cerr << "[WARNING] " << interval_stats.imu_count
              << " IMU message(s) in interval between two lidar scans. Cannot compute "
                 "acceleration statistics for orientation regularisation. Please check your data and its "
                 "timestamping as likely there should not be so few IMU measurements between two LiDAR scans.\n";
    return std::nullopt;
  }

  const Eigen::Vector3d avg_imu_accel = interval_stats.imu_acceleration_sum / interval_stats.imu_count;
  const double accel_mag_variance = interval_stats.welford_sum_of_squares / (interval_stats.imu_count - 1);
  const double dt = (time - lidar_state.time).count();

  const Eigen::Vector3d body_accel_measurement =
      avg_imu_accel + rotation_estimate.inverse() * gravity_vector(config.gravity_magnitude);

  const double max_acceleration_change = config.max_expected_jerk * dt;
  // assume [j, -j] range for uniform dist. on jerk. variance is (2j)^2 / 12 = j^2/3. multiply by dt^2 for accel
  const Eigen::Matrix3d process_noise = square(max_acceleration_change) / 3 * Eigen::Matrix3d::Identity();
  body_acceleration_covariance += process_noise;

  // isotropic accel mag variance
  const Eigen::Matrix3d measurement_noise = accel_mag_variance / 3 * Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d S = body_acceleration_covariance + measurement_noise;
  const Eigen::Matrix3d kalman_gain = body_acceleration_covariance * S.inverse();

  const Eigen::Vector3d innovation = kalman_gain * (body_accel_measurement - mean_body_acceleration);
  mean_body_acceleration += innovation;
  body_acceleration_covariance -= kalman_gain * body_acceleration_covariance;

  const Eigen::Vector3d local_gravity_estimate = avg_imu_accel - mean_body_acceleration; // points upwards

  return AccelInfo{.accel_mag_variance = accel_mag_variance, .local_gravity_estimate = local_gravity_estimate};
}

void LIO::update_maps(const Vector3dVector& map_update_frame, const Sophus::SE3d& pose) {
  map.Update(map_update_frame, pose);

  Vector3dVector points_transformed(map_update_frame.size());
  std::transform(map_update_frame.cbegin(), map_update_frame.cend(), points_transformed.begin(),
                 [&](const auto& point) { return pose * point; });
  relocalization_map.AddPoints(points_transformed);
}

Vector3dVector LIO::recover_with_scan(const Vector3dVector& filtered_frame,
                                      const Vector3dVector& map_update_frame,
                                      const Secondsd& current_lidar_time,
                                      const Sophus::SE3d& recovery_pose,
                                      const std::string& reason) {
  map.Clear();
  lidar_state.pose = recovery_pose;
  lidar_state.time = current_lidar_time;
  lidar_state.velocity.setZero();
  lidar_state.angular_velocity.setZero();
  lidar_state.linear_acceleration.setZero();
  _imu_local_rotation = recovery_pose.so3();
  _imu_local_rotation_time = current_lidar_time;
  interval_stats.reset();
  update_maps(map_update_frame, lidar_state.pose);
  poses_with_timestamps.emplace_back(lidar_state.time, lidar_state.pose);
  _consecutive_registration_failures = 0;
  std::cout << "[INFO] Kidnap recovery accepted scan at " << current_lidar_time.count() << "s via " << reason
            << ".\n";
  return filtered_frame;
}

Vector3dVector LIO::drop_failed_scan(const Secondsd& current_lidar_time, const std::string& reason) {
  lidar_state.time = current_lidar_time;
  _imu_local_rotation_time = current_lidar_time;
  interval_stats.reset();
  std::cerr << "[WARNING] Dropping scan during kidnap recovery: " << reason << "\n";
  return {};
}

std::optional<Sophus::SE3d> LIO::try_global_relocalization(const Vector3dVector& keypoints) const {
  if (!config.enable_kidnap_relocalization || relocalization_map.Empty() || keypoints.empty()) {
    return std::nullopt;
  }
  const int usable_pose_count =
      static_cast<int>(poses_with_timestamps.size()) - std::max(0, config.relocalization_min_pose_separation);
  if (usable_pose_count <= 0) {
    return std::nullopt;
  }

  LIO::Config relocalization_config = config;
  relocalization_config.max_iterations = static_cast<size_t>(std::max(1, config.relocalization_max_iterations));
  relocalization_config.max_correspondance_distance = config.relocalization_max_correspondance_distance;
  relocalization_config.min_beta = -1.0;
  const int pose_stride = std::max(1, config.relocalization_pose_stride);
  const int yaw_samples = std::max(1, config.relocalization_yaw_samples);
  const int voxel_search_radius =
      voxel_search_radius_for_distance(relocalization_map,
                                       relocalization_config.max_correspondance_distance,
                                       relocalization_config.max_voxel_search_radius);
  const double pi = std::acos(-1.0);

  bool found = false;
  Sophus::SE3d best_pose;
  AlignmentStats best_stats;
  for (int pose_index = 0; pose_index < usable_pose_count; pose_index += pose_stride) {
    const Sophus::SE3d& historical_pose = poses_with_timestamps[static_cast<size_t>(pose_index)].second;
    for (int yaw_index = 0; yaw_index < yaw_samples; ++yaw_index) {
      const double yaw = (2.0 * pi * static_cast<double>(yaw_index)) / static_cast<double>(yaw_samples);
      const Sophus::SE3d initial_guess(
          yaw_rotation(yaw) * historical_pose.so3(),
          historical_pose.translation());
      Sophus::SE3d optimized_pose;
      try {
        optimized_pose = icp(
            keypoints,
            relocalization_map,
            initial_guess,
            relocalization_config,
            std::nullopt,
            std::nullopt,
            voxel_search_radius);
      } catch (const std::exception&) {
        continue;
      }
      const AlignmentStats stats = evaluate_alignment(
          optimized_pose,
          keypoints,
          relocalization_map,
          relocalization_config.max_correspondance_distance,
          voxel_search_radius);
      if (stats.correspondences < config.relocalization_min_correspondences ||
          stats.inlier_ratio < config.relocalization_min_inlier_ratio ||
          stats.mean_error > config.relocalization_max_mean_error) {
        continue;
      }
      if (!found || stats.mean_error < best_stats.mean_error ||
          (std::abs(stats.mean_error - best_stats.mean_error) < 1e-6 &&
           stats.correspondences > best_stats.correspondences)) {
        found = true;
        best_pose = optimized_pose;
        best_stats = stats;
      }
    }
  }

  if (!found) {
    return std::nullopt;
  }
  std::cout << "[INFO] Kidnap relocalization matched " << best_stats.correspondences << "/" << keypoints.size()
            << " keypoints, mean error " << best_stats.mean_error << " m.\n";
  return best_pose;
}

// ==========================
//          public
// ==========================

// ============================ imu ===============================

void LIO::add_imu_measurement(const ImuControl& base_imu) {
  if (lidar_state.time < EPSILON_TIME) {
    static bool warning_skip_till_first_lidar = false;
    if (!warning_skip_till_first_lidar) {
      std::cerr << "[WARNING - ONCE] Skipping IMU, waiting for first LiDAR message.\n";
      warning_skip_till_first_lidar = true;
    }
    _last_real_imu_time = base_imu.time;
    _last_real_base_imu_ang_vel = base_imu.angular_velocity;
    return;
  }

  if (_imu_local_rotation_time < EPSILON_TIME) {
    _imu_local_rotation_time = lidar_state.time;
  }

  const double dt = (base_imu.time - _imu_local_rotation_time).count();

  if (dt < 0.0) {
    // messages are out of sync. thats a problem, since we integrate gyro from last lidar time onwards
    std::cerr << "[WARNING] Received IMU message from the past. Can result in errors.\n";
    // maybe skip this imu reading?
  }

  const Eigen::Vector3d unbiased_ang_vel = base_imu.angular_velocity - imu_bias.gyroscope;
  const Eigen::Vector3d unbiased_accel = base_imu.acceleration - imu_bias.accelerometer;

  _imu_local_rotation = _imu_local_rotation * Sophus::SO3d::exp(unbiased_ang_vel * dt);
  _imu_local_rotation_time = base_imu.time;

  const Eigen::Vector3d local_gravity =
      _imu_local_rotation.inverse() * gravity_vector(config.gravity_magnitude);
  const Eigen::Vector3d compensated_accel = unbiased_accel + local_gravity;

  interval_stats.update(unbiased_ang_vel, unbiased_accel, compensated_accel);

  _last_real_imu_time = base_imu.time;
  _last_real_base_imu_ang_vel = base_imu.angular_velocity;
}

void LIO::add_imu_measurement(const Sophus::SE3d& extrinsic_imu2base, const ImuControl& raw_imu) {
  if (extrinsic_imu2base.log().norm() < EPSILON) {
    add_imu_measurement(raw_imu);
    return;
  }

  if (_last_real_imu_time < EPSILON_TIME) {
    // skip IMU message as we need a previous imu time for extrinsic compensation
    _last_real_imu_time = raw_imu.time;
    return;
  }

  // accounting for the transport-rate
  ImuControl base_imu = raw_imu;
  const Sophus::SO3d& extrinsic_rotation = extrinsic_imu2base.so3();
  base_imu.angular_velocity = extrinsic_rotation * raw_imu.angular_velocity;

  const Eigen::Vector3d& lever_arm = -1 * extrinsic_imu2base.translation();
  const Secondsd dt = raw_imu.time - _last_real_imu_time;

  const Eigen::Vector3d angular_acceleration = std::invoke([&]() -> Eigen::Vector3d {
    if (std::chrono::abs(dt) < Secondsd(1.0 / 5000.0)) {
      // if dt is less than the equivalent of a 5000 Hz imu, assuming zero ang accel,
      // causes numerical issues otherwise
      static bool warning_imu_too_close = false;
      if (!warning_imu_too_close) {
        std::cerr << "[WARNING - ONCE] Received IMU message with a very short delta to previous IMU message. Ignoring "
                     "all such messages.\n";
        warning_imu_too_close = true;
      }
      return Eigen::Vector3d::Zero();
    } else {
      const Eigen::Vector3d angular_acceleration =
          (base_imu.angular_velocity - _last_real_base_imu_ang_vel) / dt.count();
      return angular_acceleration;
    }
  });

  base_imu.acceleration = extrinsic_rotation * raw_imu.acceleration + angular_acceleration.cross(lever_arm) +
                          base_imu.angular_velocity.cross(base_imu.angular_velocity.cross(lever_arm));

  this->add_imu_measurement(base_imu);
}

// ============================ lidar ===============================

Vector3dVector LIO::register_scan(const Vector3dVector& scan, const TimestampVector& timestamps) {
  // TODO: redundant max compute as its available after process_timestamps
  const auto max = std::max_element(timestamps.cbegin(), timestamps.cend());
  const Secondsd current_lidar_time = *max;

  if (lidar_state.time < EPSILON_TIME) {
    lidar_state.time = current_lidar_time;
    const auto& preproc_result = preprocess_scan(scan, config);
    if (!config.initialization_phase) {
      // use the first frame for the map only if we're not initializing
      update_maps(preproc_result.map_update_frame(), lidar_state.pose);
      poses_with_timestamps.emplace_back(lidar_state.time, lidar_state.pose);
      std::cout << "[INFO] Odometry map frame initialized with first lidar scan.\n";
    }
    return preproc_result.filtered_frame;
  }

  if (std::chrono::abs(current_lidar_time - lidar_state.time).count() > config.max_scan_delta_sec) {
    const double diff_seconds = (current_lidar_time - lidar_state.time).count();
    return drop_failed_scan(
        current_lidar_time,
        "Received LiDAR scan with " + std::to_string(diff_seconds) +
            " seconds delta to previous scan.");
  }

  const auto& [avg_body_accel, avg_ang_vel] = std::invoke([&]() -> std::pair<Eigen::Vector3d, Eigen::Vector3d> {
    if (config.initialization_phase && !_initialized) {
      // assume static and
      initialize(current_lidar_time);
      return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    }
    if (interval_stats.imu_count == 0) {
      std::cerr << "[WARNING] No Imu measurements in interval to average. Assuming constant velocity motion.\n";
      return {Eigen::Vector3d::Zero(), lidar_state.angular_velocity};
    }
    const Eigen::Vector3d avg_body_accel = interval_stats.body_acceleration_sum / interval_stats.imu_count;
    const Eigen::Vector3d avg_ang_vel = interval_stats.angular_velocity_sum / interval_stats.imu_count;
    if (avg_body_accel.norm() > 50.0) {
      std::cerr << "[WARNING] Erratic body acceleration computed, norm > 50 m/s2. Either IMU data is corrupted, or you "
                   "should report an issue.";
    }
    return {avg_body_accel, avg_ang_vel};
  });

  // compute relative motion using controls
  auto relative_pose_at_time = [&](const Secondsd time) -> Sophus::SE3d {
    const double dt = (time - lidar_state.time).count();
    Eigen::Matrix<double, 6, 1> tau;
    tau.head<3>() = lidar_state.velocity * dt + (avg_body_accel * square(dt) / 2);
    tau.tail<3>() = avg_ang_vel * dt;
    return Sophus::SE3d::exp(tau);
  };

  const Sophus::SE3d initial_guess = lidar_state.pose * relative_pose_at_time(current_lidar_time);

  // body acceleration filter
  const auto& accel_filter_info = get_accel_info(initial_guess.so3(), current_lidar_time);

  const auto& preproc_result = preprocess_scan(scan, timestamps, current_lidar_time, relative_pose_at_time, config);
  last_registration_diagnostics = {};
  last_registration_diagnostics.keypoints = static_cast<int>(preproc_result.keypoints.size());
  last_registration_diagnostics.consecutive_registration_failures = _consecutive_registration_failures;

  if (preproc_result.keypoints.size() < 10) {
    const std::string error_msg =
        "Keypoints for ICP registration = " + std::to_string(preproc_result.keypoints.size()) +
        ", this is too little for ICP and likely unintended. Input scan size = " + std::to_string(scan.size()) +
        ". Config voxel size = " + std::to_string(config.voxel_size) +
        ". Either the input scan is corrupt (empty) or the downsampling is too aggressive.";
    ++_consecutive_registration_failures;
    if (config.reset_on_registration_failure &&
        _consecutive_registration_failures >= std::max(1, config.recovery_min_failures)) {
      return drop_failed_scan(current_lidar_time, error_msg);
    }
    throw std::invalid_argument(error_msg);
  }

  if (config.enable_kidnap_relocalization && config.relocalize_after_scan_gap &&
      _consecutive_registration_failures >= std::max(1, config.recovery_min_failures)) {
    if (const auto relocalized_pose = try_global_relocalization(preproc_result.keypoints)) {
      return recover_with_scan(preproc_result.filtered_frame,
                               preproc_result.map_update_frame(),
                               current_lidar_time,
                               relocalized_pose.value(),
                               "global relocalization after scan gap");
    }
  }

  if (!map.Empty()) {
    SCOPED_PROFILER("ICP");
    Sophus::SE3d optimized_pose;
    try {
      Sophus::SE3d fine_initial_guess = initial_guess;
      if (config.coarse_to_fine_registration) {
        const Vector3dVector coarse_keypoints =
            voxel_down_sample(preproc_result.keypoints, std::max(config.coarse_voxel_size, config.voxel_size));
        if (coarse_keypoints.size() >= 10) {
          LIO::Config coarse_config = config;
          coarse_config.max_iterations = std::max<size_t>(1, config.coarse_max_iterations);
          coarse_config.max_correspondance_distance =
              std::max(config.max_correspondance_distance, config.coarse_max_correspondance_distance);
          fine_initial_guess = icp(coarse_keypoints, map, initial_guess, coarse_config, accel_filter_info, initial_guess);
          last_registration_diagnostics.coarse_to_fine_used = true;
        }
      }
      optimized_pose = icp(preproc_result.keypoints,
                           map,
                           fine_initial_guess,
                           config,
                           accel_filter_info,
                           fine_initial_guess,
                           1,
                           &last_registration_diagnostics);
    } catch (const std::exception&) {
      last_registration_diagnostics.consecutive_registration_failures =
          _consecutive_registration_failures + 1;
      ++_consecutive_registration_failures;
      if (_consecutive_registration_failures < std::max(1, config.recovery_min_failures)) {
        throw;
      }
      if (const auto relocalized_pose = try_global_relocalization(preproc_result.keypoints)) {
        return recover_with_scan(preproc_result.filtered_frame,
                                 preproc_result.map_update_frame(),
                                 current_lidar_time,
                                 relocalized_pose.value(),
                                 "global relocalization");
      }
      if (config.reset_on_registration_failure) {
        return recover_with_scan(preproc_result.filtered_frame,
                                 preproc_result.map_update_frame(),
                                 current_lidar_time,
                                 lidar_state.pose,
                                 "local reset");
      }
      throw;
    }

    const int voxel_search_radius =
        voxel_search_radius_for_distance(map, config.max_correspondance_distance, config.max_voxel_search_radius);
    const AlignmentStats stats = evaluate_alignment(
        optimized_pose, preproc_result.keypoints, map, config.max_correspondance_distance, voxel_search_radius);
    last_registration_diagnostics.valid = stats.correspondences > 0;
    last_registration_diagnostics.correspondences = stats.correspondences;
    last_registration_diagnostics.inlier_ratio = stats.inlier_ratio;
    last_registration_diagnostics.mean_error =
        std::isfinite(stats.mean_error) ? stats.mean_error : 0.0;

    try {
      const auto& [H, b, chi] = build_icp_linear_system(
          optimized_pose, preproc_result.keypoints, map, config, voxel_search_radius);
      (void)b;
      (void)chi;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix6d> solver(H);
      if (solver.info() == Eigen::Success) {
        const Eigen::Vector6d eigenvalues = solver.eigenvalues();
        const double min_eigen = eigenvalues.minCoeff();
        const double max_eigen = eigenvalues.maxCoeff();
        last_registration_diagnostics.hessian_min_eigenvalue = min_eigen;
        last_registration_diagnostics.hessian_max_eigenvalue = max_eigen;
        last_registration_diagnostics.hessian_condition =
            min_eigen > 1e-9 ? max_eigen / min_eigen : std::numeric_limits<double>::infinity();
      }
    } catch (const std::exception&) {
      last_registration_diagnostics.hessian_condition = std::numeric_limits<double>::infinity();
    }

    const Sophus::SE3d candidate_motion = lidar_state.pose.inverse() * optimized_pose;
    const Eigen::Vector6d candidate_delta = candidate_motion.log();
    const bool stationary_interval =
        config.enable_stationary_hold &&
        avg_ang_vel.norm() <= config.stationary_angular_velocity_threshold &&
        avg_body_accel.norm() <= config.stationary_linear_acceleration_threshold &&
        candidate_motion.translation().norm() <= config.stationary_max_translation_delta &&
        candidate_motion.so3().log().norm() <= config.stationary_max_rotation_delta;

    if (stationary_interval) {
      optimized_pose = lidar_state.pose;
    } else if (config.enable_degeneracy_damping) {
      const double condition = last_registration_diagnostics.hessian_condition;
      if (!std::isfinite(condition) || condition >= config.degeneracy_damping_condition) {
        const double max_alpha = clamp01(config.degeneracy_damping_alpha);
        double alpha = max_alpha;
        if (config.adaptive_degeneracy_damping && std::isfinite(condition) && condition > 0.0) {
          const double ratio = std::max(0.0, config.degeneracy_damping_condition) / condition;
          const double min_alpha = std::min(max_alpha, clamp01(config.degeneracy_damping_min_alpha));
          alpha = std::clamp(max_alpha * ratio, min_alpha, max_alpha);
        }
        last_registration_diagnostics.degeneracy_damping_alpha_applied = alpha;
        optimized_pose = lidar_state.pose * Sophus::SE3d::exp(alpha * candidate_delta);
      }
    }

    if (config.enable_rover_degeneracy_motion_constraint) {
      const double condition = last_registration_diagnostics.hessian_condition;
      if (!std::isfinite(condition) || condition >= config.rover_degeneracy_motion_condition) {
        last_registration_diagnostics.rover_degeneracy_motion_constraint_applied = true;
        Eigen::Vector6d rover_delta = (lidar_state.pose.inverse() * optimized_pose).log();
        rover_delta[0] *= clamp01(config.rover_degeneracy_forward_scale);
        rover_delta[1] *= clamp01(config.rover_degeneracy_lateral_scale);
        rover_delta[2] *= clamp01(config.rover_degeneracy_vertical_scale);
        rover_delta[3] *= clamp01(config.rover_degeneracy_roll_pitch_scale);
        rover_delta[4] *= clamp01(config.rover_degeneracy_roll_pitch_scale);
        rover_delta[5] *= clamp01(config.rover_degeneracy_yaw_scale);
        optimized_pose = lidar_state.pose * Sophus::SE3d::exp(rover_delta);
      }
    }

    if (config.enable_vertical_spike_filter && config.max_vertical_update_m > 0.0) {
      const double previous_z = lidar_state.pose.translation().z();
      const double dz = optimized_pose.translation().z() - previous_z;
      const double max_dz = std::abs(config.max_vertical_update_m);
      if (std::isfinite(dz) && std::abs(dz) > max_dz) {
        optimized_pose.translation().z() = previous_z + std::copysign(max_dz, dz);
      }
    }

    // estimate velocities and accelerations from the new pose
    const double dt = (current_lidar_time - lidar_state.time).count();
    const Sophus::SE3d motion = lidar_state.pose.inverse() * optimized_pose;
    const Eigen::Vector6d local_velocity = motion.log() / dt;
    const Eigen::Vector3d local_linear_acceleration =
        (local_velocity.head<3>() - motion.so3().inverse() * lidar_state.velocity) / dt;

    // update
    lidar_state.pose = optimized_pose;
    lidar_state.velocity = local_velocity.head<3>();
    lidar_state.angular_velocity = local_velocity.tail<3>();
    lidar_state.linear_acceleration = local_linear_acceleration;

    _imu_local_rotation = optimized_pose.so3(); // correct the drift in imu integration
  }
  // even if map is empty, time should still update
  lidar_state.time = current_lidar_time;
  _imu_local_rotation_time = current_lidar_time;

  // reset imu averages
  interval_stats.reset();

  update_maps(preproc_result.map_update_frame(), lidar_state.pose);

  poses_with_timestamps.emplace_back(lidar_state.time, lidar_state.pose);
  _consecutive_registration_failures = 0;
  last_registration_diagnostics.consecutive_registration_failures = 0;

  return preproc_result.filtered_frame;
}

Vector3dVector LIO::register_scan(const Sophus::SE3d& extrinsic_lidar2base,
                                  const Vector3dVector& scan,
                                  const TimestampVector& timestamps) {
  if (extrinsic_lidar2base.log().norm() < EPSILON) {
    return register_scan(scan, timestamps);
  }

  Vector3dVector transformed_scan = scan;
  transform_points(extrinsic_lidar2base, transformed_scan);
  Vector3dVector frame = register_scan(transformed_scan, timestamps);
  transform_points(extrinsic_lidar2base.inverse(), frame);
  return frame;
}
} // namespace rko_lio::core
