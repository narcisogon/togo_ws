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
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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
Implements the persistent GTSAM iSAM2 pose-graph backend used for live mapping.
The caller supplies a complete graph snapshot on every request, while this file
identifies and appends only new variables and factors whenever graph history is
unchanged.

State lifecycle:
The iSAM2 Bayes tree, estimate, factor graph, and factor identity maps survive
between optimize calls. Removing or modifying historical graph data, shrinking
the pose list, or changing the anchor triggers a controlled full rebuild.

Update policy:
Routine adjacent odometry additions use one iSAM2 update. New loop, IMU, or
absolute-position constraints may run additional empty updates so large
nonlinear corrections can relinearize and propagate through the graph.
*/

#include "pose_graph_optimizer_internal.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
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
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
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
    std::chrono::steady_clock::now() - start).count();
}

/*
Summary:
Converts the backend-neutral Eigen SE(3) pose into GTSAM Pose3 form.
*/
gtsam::Pose3 toGtsamPose(const Eigen::Isometry3d & pose)
{
  return gtsam::Pose3(
    gtsam::Rot3(pose.rotation()),
    gtsam::Point3(
      pose.translation().x(), pose.translation().y(), pose.translation().z()));
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
Builds GTSAM's rotation-first 6x6 precision matrix from separate physical blocks.
*/
gtsam::Matrix6 gtsamInformation(const BetweenConstraint & constraint)
{
  gtsam::Matrix6 information = gtsam::Matrix6::Zero();
  // Pose3 tangent order is [rotation, translation].
  information.topLeftCorner<3, 3>() = constraint.rotation_information;
  information.bottomRightCorner<3, 3>() = constraint.translation_information;
  for (int i = 0; i < 6; ++i) {
    information(i, i) = std::max(information(i, i), 1.0e-12);
  }
  return information;
}

/*
Summary:
Creates the GTSAM robust-loss estimator requested for a loop constraint.
*/
gtsam::noiseModel::mEstimator::Base::shared_ptr makeMEstimator(
  robust::LoopEdgeKernelType type, double delta)
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
    makeMEstimator(constraint.robust_kernel, constraint.robust_delta), gaussian);
}

/*
Summary:
Checks that a signed pose index addresses the current pose snapshot.
*/
bool validPoseIndex(int index, std::size_t pose_count)
{
  return index >= 0 && static_cast<std::size_t>(index) < pose_count;
}

/*
Summary:
Compares two poses tightly enough to detect a changed anchor or factor history.
*/
bool posesEquivalent(
  const Eigen::Isometry3d & lhs, const Eigen::Isometry3d & rhs,
  double tolerance = 1.0e-9)
{
  return (lhs.translation() - rhs.translation()).norm() <= tolerance &&
         Eigen::AngleAxisd(lhs.rotation().transpose() * rhs.rotation()).angle() <= tolerance;
}

/*
Summary:
Checks whether a previously stored relative-pose constraint is unchanged.
*/
bool constraintsEquivalent(
  const BetweenConstraint & lhs, const BetweenConstraint & rhs)
{
  return lhs.from == rhs.from && lhs.to == rhs.to && lhs.kind == rhs.kind &&
         posesEquivalent(lhs.measurement, rhs.measurement) &&
         lhs.translation_information.isApprox(rhs.translation_information, 1.0e-12) &&
         lhs.rotation_information.isApprox(rhs.rotation_information, 1.0e-12) &&
         lhs.robust == rhs.robust && lhs.robust_kernel == rhs.robust_kernel &&
         std::abs(lhs.robust_delta - rhs.robust_delta) <= 1.0e-12;
}

/*
Summary:
Checks whether a previously stored absolute-position constraint is unchanged.
*/
bool constraintsEquivalent(
  const PositionConstraint & lhs, const PositionConstraint & rhs)
{
  return lhs.pose_index == rhs.pose_index &&
         lhs.position.isApprox(rhs.position, 1.0e-12) &&
         lhs.information.isApprox(rhs.information, 1.0e-12);
}

/*
Summary:
Uniquely identifies a relative factor by endpoints, semantic kind, and repeated
occurrence number within the complete graph snapshot.
*/
struct BetweenFactorKey
{
  int from{-1};
  int to{-1};
  ConstraintKind kind{ConstraintKind::ADJACENT};
  int occurrence{0};

  /*
  Summary:
  Provides deterministic ordering for use as a std::map key.
  */
  bool operator<(const BetweenFactorKey & other) const
  {
    return std::tie(from, to, kind, occurrence) <
           std::tie(other.from, other.to, other.kind, other.occurrence);
  }
};

/*
Summary:
Uniquely identifies an absolute-position factor, including repeated factors on
the same pose.
*/
struct PositionFactorKey
{
  int pose_index{-1};
  int occurrence{0};

  /*
  Summary:
  Provides deterministic ordering for use as a std::map key.
  */
  bool operator<(const PositionFactorKey & other) const
  {
    return std::tie(pose_index, occurrence) <
           std::tie(other.pose_index, other.occurrence);
  }
};

/*
Summary:
Pairs a stable relative-factor key with its constraint in the current snapshot.
*/
struct CurrentBetweenConstraint
{
  BetweenFactorKey key;
  const BetweenConstraint * constraint{nullptr};
};

/*
Summary:
Pairs a stable absolute-position key with its constraint in the current snapshot.
*/
struct CurrentPositionConstraint
{
  PositionFactorKey key;
  const PositionConstraint * constraint{nullptr};
};

/*
Summary:
Assigns stable occurrence-aware keys to all current relative-pose constraints.

Important behavior:
The caller must keep the PoseGraphProblem alive while the returned pointers are
used because this helper does not copy constraints.
*/
std::vector<CurrentBetweenConstraint> keyedBetweenConstraints(
  const PoseGraphProblem & problem)
{
  std::map<std::tuple<int, int, ConstraintKind>, int> occurrences;
  std::vector<CurrentBetweenConstraint> result;
  result.reserve(problem.between_constraints.size());
  for (const auto & constraint : problem.between_constraints) {
    const auto base = std::make_tuple(constraint.from, constraint.to, constraint.kind);
    const int occurrence = occurrences[base]++;
    result.push_back({
              BetweenFactorKey{constraint.from, constraint.to, constraint.kind, occurrence},
              &constraint});
  }
  return result;
}

/*
Summary:
Assigns stable occurrence-aware keys to all current absolute-position constraints.
*/
std::vector<CurrentPositionConstraint> keyedPositionConstraints(
  const PoseGraphProblem & problem)
{
  std::map<int, int> occurrences;
  std::vector<CurrentPositionConstraint> result;
  result.reserve(problem.position_constraints.size());
  for (const auto & constraint : problem.position_constraints) {
    const int occurrence = occurrences[constraint.pose_index]++;
    result.push_back({PositionFactorKey{constraint.pose_index, occurrence}, &constraint});
  }
  return result;
}

/*
Summary:
Owns the persistent iSAM2 state and reconciles complete snapshots against the
factors already incorporated into that state.
*/
class IncrementalGtsamOptimizerImpl final : public IncrementalGtsamOptimizer
{
public:
  /*
  Summary:
  Updates the persistent iSAM2 graph from a complete current graph snapshot.

  Returns:
  Corrected poses, residuals, timing, serialization status, and counts showing
  whether state was reused, extended, or rebuilt.

  Important behavior:
  New graph elements are appended. Missing or changed historical elements,
  pose-count reduction, fixed-pose changes, and anchor movement trigger rebuilds.
  */
  PoseGraphResult optimize(const PoseGraphProblem & problem) override
  {
    PoseGraphResult result;
    result.incremental_backend = true;
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
    for (const auto & constraint : problem.between_constraints) {
      if (!validPoseIndex(constraint.from, problem.initial_poses.size()) ||
        !validPoseIndex(constraint.to, problem.initial_poses.size()))
      {
        result.error_message = "between-constraint pose index is out of range";
        return result;
      }
    }
    for (const auto & constraint : problem.position_constraints) {
      if (!validPoseIndex(constraint.pose_index, problem.initial_poses.size())) {
        result.error_message = "position-constraint pose index is out of range";
        return result;
      }
    }

    try {
      const auto stage_start = std::chrono::steady_clock::now();
      const auto current_between = keyedBetweenConstraints(problem);
      const auto current_positions = keyedPositionConstraints(problem);

      std::string rebuild_reason;
      if (!initialized_) {
        rebuild_reason = "initialization";
      } else if (problem.initial_poses.size() < pose_count_) {
        rebuild_reason = "pose_count_decreased";
      } else if (problem.fixed_pose_index != fixed_pose_index_) {
        rebuild_reason = "fixed_pose_changed";
      } else if (!posesEquivalent(
          problem.initial_poses[static_cast<std::size_t>(problem.fixed_pose_index)],
          anchor_pose_))
      {
        rebuild_reason = "anchor_pose_changed";
      } else {
        std::map<BetweenFactorKey, const BetweenConstraint *> current_between_map;
        for (const auto & item : current_between) {
          current_between_map.emplace(item.key, item.constraint);
        }
        for (const auto & stored : between_constraints_) {
          const auto current = current_between_map.find(stored.first);
          if (current == current_between_map.end()) {
            rebuild_reason = "between_factor_removed";
            break;
          }
          if (!constraintsEquivalent(stored.second, *current->second)) {
            rebuild_reason = "between_factor_changed";
            break;
          }
        }

        if (rebuild_reason.empty()) {
          std::map<PositionFactorKey, const PositionConstraint *> current_position_map;
          for (const auto & item : current_positions) {
            current_position_map.emplace(item.key, item.constraint);
          }
          for (const auto & stored : position_constraints_) {
            const auto current = current_position_map.find(stored.first);
            if (current == current_position_map.end()) {
              rebuild_reason = "position_factor_removed";
              break;
            }
            if (!constraintsEquivalent(stored.second, *current->second)) {
              rebuild_reason = "position_factor_changed";
              break;
            }
          }
        }
      }

      gtsam::NonlinearFactorGraph new_factors;
      gtsam::Values new_values;
      bool added_non_adjacent_factor = false;

      if (!rebuild_reason.empty()) {
        resetState();
        initializeIsam();
        result.state_rebuilt = true;
        result.state_rebuild_reason = rebuild_reason;

        for (std::size_t i = 0; i < problem.initial_poses.size(); ++i) {
          new_values.insert(static_cast<gtsam::Key>(i), toGtsamPose(problem.initial_poses[i]));
        }

        const gtsam::Vector6 anchor_sigmas = gtsam::Vector6::Constant(1.0e-6);
        auto anchor = std::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          static_cast<gtsam::Key>(problem.fixed_pose_index),
          toGtsamPose(problem.initial_poses[static_cast<std::size_t>(
            problem.fixed_pose_index)]),
          gtsam::noiseModel::Diagonal::Sigmas(anchor_sigmas));
        new_factors.push_back(anchor);
        full_graph_.push_back(anchor);

        for (const auto & item : current_between) {
          addBetweenFactor(item, new_factors);
          added_non_adjacent_factor =
            added_non_adjacent_factor ||
            item.constraint->kind != ConstraintKind::ADJACENT;
        }
        for (const auto & item : current_positions) {
          addPositionFactor(item, new_factors);
          added_non_adjacent_factor = true;
        }
      } else {
        for (std::size_t i = pose_count_; i < problem.initial_poses.size(); ++i) {
          new_values.insert(static_cast<gtsam::Key>(i), toGtsamPose(problem.initial_poses[i]));
        }
        for (const auto & item : current_between) {
          if (between_constraints_.find(item.key) == between_constraints_.end()) {
            addBetweenFactor(item, new_factors);
            added_non_adjacent_factor =
              added_non_adjacent_factor ||
              item.constraint->kind != ConstraintKind::ADJACENT;
          }
        }
        for (const auto & item : current_positions) {
          if (position_constraints_.find(item.key) == position_constraints_.end()) {
            addPositionFactor(item, new_factors);
            added_non_adjacent_factor = true;
          }
        }
      }

      result.variables_added = new_values.size();
      result.factors_added = new_factors.size();
      result.graph_build_ms = elapsedMillis(stage_start);

      const auto optimize_start = std::chrono::steady_clock::now();
      if (!new_factors.empty() || !new_values.empty()) {
        isam_->update(new_factors, new_values);
        result.iterations = 1;

        // A newly introduced loop/IMU/GNSS constraint can create a large
        // nonlinear correction. Extra empty updates are deliberately limited
        // to those rare events; ordinary odometry-only appends stay one-shot.
        if (added_non_adjacent_factor) {
          const int max_updates = std::max(1, problem.max_iterations);
          const gtsam::NonlinearFactorGraph empty_factors;
          const gtsam::Values empty_values;
          while (result.iterations < max_updates) {
            isam_->update(empty_factors, empty_values);
            ++result.iterations;
          }
        }
      }
      const gtsam::Values optimized = isam_->calculateEstimate();
      result.optimize_ms = elapsedMillis(optimize_start);

      pose_count_ = problem.initial_poses.size();
      fixed_pose_index_ = problem.fixed_pose_index;
      anchor_pose_ = problem.initial_poses[static_cast<std::size_t>(
            problem.fixed_pose_index)];
      initialized_ = true;
      result.total_variables = pose_count_;
      result.total_factors = full_graph_.size();

      for (std::size_t i = 0; i < problem.initial_poses.size(); ++i) {
        result.poses[i] =
          fromGtsamPose(optimized.at<gtsam::Pose3>(static_cast<gtsam::Key>(i)));
      }

      for (std::size_t i = 0; i < current_between.size(); ++i) {
        const auto factor_it = between_factors_.find(current_between[i].key);
        if (factor_it == between_factors_.end()) {
          result.error_message = "iSAM2 state is missing a between factor";
          resetState();
          return result;
        }
        const gtsam::Vector error = factor_it->second->unwhitenedError(optimized);
        if (error.size() != 6) {
          result.error_message = "GTSAM returned a non-6D Pose3 residual";
          resetState();
          return result;
        }
        result.constraint_residuals[i].rotation = error.head<3>();
        result.constraint_residuals[i].translation = error.tail<3>();
        const gtsam::Matrix6 information =
          gtsamInformation(*current_between[i].constraint);
        result.constraint_residuals[i].chi2 =
          (error.transpose() * information * error)(0, 0);
      }

      if (result.graph_save_requested) {
        const auto save_start = std::chrono::steady_clock::now();
        try {
          gtsam::writeG2o(full_graph_, optimized, problem.save_path);
          result.graph_saved = true;
        } catch (const std::exception & ex) {
          result.graph_save_error =
            std::string("GTSAM iSAM2 failed to save graph: ") + ex.what();
        }
        result.graph_save_ms = elapsedMillis(save_start);
      }

      result.success = true;
      return result;
    } catch (const std::exception & ex) {
      result.error_message = std::string("GTSAM iSAM2 exception: ") + ex.what();
      resetState();
      return result;
    }
  }

  /*
  Summary:
  Discards all persistent graph, factor-identity, estimate, and iSAM2 state.
  */
  void reset() override
  {
    resetState();
  }

private:
  /*
  Summary:
  Creates a fresh iSAM2 engine with the live mapping relinearization policy.
  */
  void initializeIsam()
  {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(params);
  }

  /*
  Summary:
  Returns the implementation to its pre-initialization state.
  */
  void resetState()
  {
    initialized_ = false;
    pose_count_ = 0;
    fixed_pose_index_ = -1;
    anchor_pose_ = Eigen::Isometry3d::Identity();
    between_constraints_.clear();
    position_constraints_.clear();
    between_factors_.clear();
    full_graph_.resize(0);
    isam_.reset();
  }

  /*
  Summary:
  Converts and appends one relative-pose constraint to both the pending update
  and the retained full graph, then records its identity and value.
  */
  void addBetweenFactor(
    const CurrentBetweenConstraint & item,
    gtsam::NonlinearFactorGraph & new_factors)
  {
    auto factor = std::make_shared<PoseBetweenFactor>(
      static_cast<gtsam::Key>(item.constraint->from),
      static_cast<gtsam::Key>(item.constraint->to),
      toGtsamPose(item.constraint->measurement), makeNoiseModel(*item.constraint));
    new_factors.push_back(factor);
    full_graph_.push_back(factor);
    between_constraints_.emplace(item.key, *item.constraint);
    between_factors_.emplace(item.key, std::move(factor));
  }

  /*
  Summary:
  Converts and appends one absolute-position constraint to the pending update
  and retained full graph, then records its identity and value.
  */
  void addPositionFactor(
    const CurrentPositionConstraint & item,
    gtsam::NonlinearFactorGraph & new_factors)
  {
    auto factor = std::make_shared<gtsam::GPSFactor>(
      static_cast<gtsam::Key>(item.constraint->pose_index),
      gtsam::Point3(
        item.constraint->position.x(), item.constraint->position.y(),
        item.constraint->position.z()),
      gtsam::noiseModel::Gaussian::Information(item.constraint->information));
    new_factors.push_back(factor);
    full_graph_.push_back(factor);
    position_constraints_.emplace(item.key, *item.constraint);
  }

  bool initialized_{false};
  std::size_t pose_count_{0};
  int fixed_pose_index_{-1};
  Eigen::Isometry3d anchor_pose_{Eigen::Isometry3d::Identity()};
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::NonlinearFactorGraph full_graph_;
  std::map<BetweenFactorKey, BetweenConstraint> between_constraints_;
  std::map<PositionFactorKey, PositionConstraint> position_constraints_;
  std::map<BetweenFactorKey, PoseBetweenFactor::shared_ptr> between_factors_;
};
} // namespace

/*
Summary:
Creates a new private persistent iSAM2 implementation for the public dispatcher.
*/
std::unique_ptr<IncrementalGtsamOptimizer> makeIncrementalGtsamOptimizer()
{
  return std::make_unique<IncrementalGtsamOptimizerImpl>();
}

} // namespace detail
} // namespace optimization
} // namespace graphslam
