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
Implements the stateless batch GTSAM pose-graph backend. Every request builds a
new NonlinearFactorGraph, initializes all poses, applies all current factors,
and solves the complete graph with Levenberg-Marquardt.

Primary use:
Batch mode provides a simple reference solve for comparison, debugging, and
workloads where retaining incremental state is not required.
*/

#include "pose_graph_optimizer_internal.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <memory>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <vector>

namespace graphslam
{
namespace optimization
{
namespace detail
{
namespace
{
using PoseBetweenFactor = gtsam::BetweenFactor<gtsam::Pose3>;

/*
Summary:
Returns elapsed wall-clock time in milliseconds for backend diagnostics.
*/
double elapsedMillis(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
         .count();
}

/*
Summary:
Converts the backend-neutral Eigen SE(3) pose into GTSAM Pose3 form.
*/
gtsam::Pose3 toGtsamPose(const Eigen::Isometry3d & pose)
{
  return gtsam::Pose3(gtsam::Rot3(pose.rotation()),
                      gtsam::Point3(pose.translation().x(),
                                    pose.translation().y(),
                                    pose.translation().z()));
}

/*
Summary:
Converts a solved GTSAM Pose3 back into the backend-neutral Eigen form.
*/
Eigen::Isometry3d fromGtsamPose(const gtsam::Pose3 & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = pose.rotation().matrix();
  result.translation() = pose.translation();
  return result;
}

/*
Summary:
Builds a GTSAM 6x6 precision matrix from separate physical information blocks.

Important behavior:
GTSAM Pose3 tangent order is rotation followed by translation. Numerically
negligible diagonal precision keeps intentionally unconstrained axes factorizable.
*/
gtsam::Matrix6 gtsamInformation(const BetweenConstraint & constraint)
{
  gtsam::Matrix6 information = gtsam::Matrix6::Zero();
  // Pose3 tangent order is [rotation, translation].
  information.topLeftCorner<3, 3>() = constraint.rotation_information;
  information.bottomRightCorner<3, 3>() = constraint.translation_information;

  // A rotation-only IMU factor is intentionally rank deficient in physical
  // terms. GTSAM's Gaussian noise model expects a factorizable matrix, so use
  // a numerically negligible precision in unconstrained axes. At 1e-12 this
  // cannot influence metre-scale poses but keeps Cholesky/whitening defined.
  for (int i = 0; i < 6; ++i) {
    information(i, i) = std::max(information(i, i), 1.0e-12);
  }
  return information;
}

/*
Summary:
Creates the GTSAM robust-loss estimator requested for a loop constraint.
*/
gtsam::noiseModel::mEstimator::Base::shared_ptr
makeMEstimator(robust::LoopEdgeKernelType type, double delta)
{
  using gtsam::noiseModel::mEstimator::Base;
  switch (type) {
    case robust::LoopEdgeKernelType::DCS:
      return gtsam::noiseModel::mEstimator::DCS::Create(delta, Base::Block);
    case robust::LoopEdgeKernelType::Cauchy:
      return gtsam::noiseModel::mEstimator::Cauchy::Create(delta, Base::Block);
    case robust::LoopEdgeKernelType::Huber:
    default:
      return gtsam::noiseModel::mEstimator::Huber::Create(delta, Base::Block);
  }
}

/*
Summary:
Creates a Gaussian factor noise model and optionally wraps it in a robust loss.
*/
gtsam::SharedNoiseModel makeNoiseModel(const BetweenConstraint & constraint)
{
  const auto gaussian =
    gtsam::noiseModel::Gaussian::Information(gtsamInformation(constraint));
  if (!constraint.robust) {
    return gaussian;
  }
  return gtsam::noiseModel::Robust::Create(
      makeMEstimator(constraint.robust_kernel, constraint.robust_delta),
      gaussian);
}

/*
Summary:
Checks that a signed pose index addresses the current pose snapshot.
*/
bool validPoseIndex(int index, std::size_t pose_count)
{
  return index >= 0 && static_cast<std::size_t>(index) < pose_count;
}
} // namespace

/*
Summary:
Builds and solves the complete pose graph with batch GTSAM.

Input:
- problem: Full current pose snapshot, all relative factors, all absolute
  position factors, the fixed-pose index, and optional graph save path.

Returns:
Corrected poses, per-between-factor residuals, iteration counts, timings, and
graph serialization status.

Important behavior:
The fixed pose is represented by a tight prior. A non-empty save path writes the
solved graph in the interoperable g2o text format through GTSAM.
*/
PoseGraphResult optimizePoseGraphGtsam(const PoseGraphProblem & problem)
{
  PoseGraphResult result;
  result.poses = problem.initial_poses;
  result.constraint_residuals.resize(problem.between_constraints.size());
  result.graph_save_requested = !problem.save_path.empty();

  if (problem.initial_poses.empty()) {
    result.error_message = "pose graph has no poses";
    return result;
  }
  if (!validPoseIndex(problem.fixed_pose_index, problem.initial_poses.size())) {
    result.error_message = "fixed pose index is out of range";
    return result;
  }

  try {
    auto stage_start = std::chrono::steady_clock::now();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    for (std::size_t i = 0; i < problem.initial_poses.size(); ++i) {
      initial.insert(static_cast<gtsam::Key>(i),
                     toGtsamPose(problem.initial_poses[i]));
    }

    // A tight prior holds the map origin to sub-micron precision without
    // relying on backend-internal constrained ordering.
    const gtsam::Vector6 anchor_sigmas = gtsam::Vector6::Constant(1.0e-6);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        static_cast<gtsam::Key>(problem.fixed_pose_index),
        toGtsamPose(problem.initial_poses[static_cast<std::size_t>(
        problem.fixed_pose_index)]),
        gtsam::noiseModel::Diagonal::Sigmas(anchor_sigmas));

    std::vector<PoseBetweenFactor::shared_ptr> between_factors;
    between_factors.reserve(problem.between_constraints.size());
    std::vector<gtsam::Matrix6> between_information;
    between_information.reserve(problem.between_constraints.size());
    for (const auto & constraint : problem.between_constraints) {
      if (!validPoseIndex(constraint.from, problem.initial_poses.size()) ||
        !validPoseIndex(constraint.to, problem.initial_poses.size()))
      {
        result.error_message = "between-constraint pose index is out of range";
        return result;
      }
      auto factor = std::make_shared<PoseBetweenFactor>(
          static_cast<gtsam::Key>(constraint.from),
          static_cast<gtsam::Key>(constraint.to),
          toGtsamPose(constraint.measurement), makeNoiseModel(constraint));
      graph.push_back(factor);
      between_factors.push_back(factor);
      between_information.push_back(gtsamInformation(constraint));
    }

    for (const auto & constraint : problem.position_constraints) {
      if (!validPoseIndex(constraint.pose_index,
                          problem.initial_poses.size()))
      {
        result.error_message = "position-constraint pose index is out of range";
        return result;
      }
      graph.emplace_shared<gtsam::GPSFactor>(
          static_cast<gtsam::Key>(constraint.pose_index),
          gtsam::Point3(constraint.position.x(), constraint.position.y(),
                        constraint.position.z()),
          gtsam::noiseModel::Gaussian::Information(constraint.information));
    }
    result.graph_build_ms = elapsedMillis(stage_start);

    stage_start = std::chrono::steady_clock::now();
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(std::max(1, problem.max_iterations));
    params.setVerbosity("SILENT");
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    const gtsam::Values optimized = optimizer.optimize();
    result.iterations = static_cast<int>(optimizer.iterations());
    result.optimize_ms = elapsedMillis(stage_start);

    for (std::size_t i = 0; i < problem.initial_poses.size(); ++i) {
      result.poses[i] =
        fromGtsamPose(optimized.at<gtsam::Pose3>(static_cast<gtsam::Key>(i)));
    }

    for (std::size_t i = 0; i < between_factors.size(); ++i) {
      const gtsam::Vector error =
        between_factors[i]->unwhitenedError(optimized);
      if (error.size() != 6) {
        result.error_message = "GTSAM returned a non-6D Pose3 residual";
        return result;
      }
      result.constraint_residuals[i].rotation = error.head<3>();
      result.constraint_residuals[i].translation = error.tail<3>();
      result.constraint_residuals[i].chi2 =
        (error.transpose() * between_information[i] * error)(0, 0);
    }

    if (result.graph_save_requested) {
      stage_start = std::chrono::steady_clock::now();
      try {
        gtsam::writeG2o(graph, optimized, problem.save_path);
        result.graph_saved = true;
      } catch (const std::exception & ex) {
        result.graph_save_error =
          std::string("GTSAM failed to save graph: ") + ex.what();
      }
      result.graph_save_ms = elapsedMillis(stage_start);
    }

    result.success = true;
    return result;
  } catch (const std::exception & ex) {
    result.error_message = std::string("GTSAM exception: ") + ex.what();
    return result;
  }
}

} // namespace detail
} // namespace optimization
} // namespace graphslam
