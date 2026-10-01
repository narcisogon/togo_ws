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
Defines the backend-neutral pose-graph optimization API used by graph-based
SLAM. The ROS component converts submaps and constraints into these data
structures, then selects either batch GTSAM or persistent GTSAM iSAM2.

Ownership:
PoseGraphProblem and PoseGraphResult own their snapshots. PoseGraphOptimizer
owns backend state; an iSAM2 instance therefore remains valid only while its
PoseGraphOptimizer session remains alive.

Coordinate and matrix conventions:
Poses are Eigen SE(3) transforms in the graph's map frame. Translation and
rotation information are kept in separate 3x3 matrices so backend-specific
tangent-vector ordering cannot leak into the public API.
*/

#ifndef GRAPH_BASED_SLAM__POSE_GRAPH_OPTIMIZER_HPP_
#define GRAPH_BASED_SLAM__POSE_GRAPH_OPTIMIZER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "graph_based_slam/loop_edge_robustifier.hpp"

namespace graphslam
{
namespace optimization
{

/*
Summary:
Selects the optimization strategy. GTSAM solves the full graph in one batch;
GTSAM_ISAM2 preserves incremental state between optimize calls.
*/
enum class PoseGraphBackend
{
  GTSAM,
  GTSAM_ISAM2,
};

/*
Summary:
Identifies why a relative-pose factor exists. The iSAM2 backend uses this to
distinguish routine adjacent additions from constraints that may need extra
nonlinear update passes.
*/
enum class ConstraintKind
{
  ADJACENT,
  LOOP,
  IMU_ROTATION,
};

/*
Summary:
Describes one relative SE(3) constraint between two pose indices.

Important behavior:
Information matrices contain precision, not covariance. Robust loss settings
are normally enabled for loop closures and left disabled for adjacent factors.
*/
struct BetweenConstraint
{
  int from{-1};
  int to{-1};
  Eigen::Isometry3d measurement{Eigen::Isometry3d::Identity()};
  // Store named physical blocks instead of a backend-native 6x6 matrix.
  // GTSAM Pose3 uses [rotation, translation] tangent ordering, so keeping the
  // blocks separate prevents a silent axis/order swap at the API boundary.
  Eigen::Matrix3d translation_information{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d rotation_information{Eigen::Matrix3d::Zero()};
  ConstraintKind kind{ConstraintKind::ADJACENT};
  bool robust{false};
  robust::LoopEdgeKernelType robust_kernel{robust::LoopEdgeKernelType::Huber};
  double robust_delta{1.0};
};

/*
Summary:
Constrains the translation of one pose to an absolute map-frame position.
This is the backend-neutral representation used for GNSS position factors.
*/
struct PositionConstraint
{
  int pose_index{-1};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d information{Eigen::Matrix3d::Zero()};
};

/*
Summary:
Contains the complete current pose graph supplied to an optimizer.

Important behavior:
This object is a full snapshot, not an incremental delta. Pose indices used by
all constraints must address initial_poses. save_path requests serialization
when non-empty; it does not select an optimization backend.
*/
struct PoseGraphProblem
{
  std::vector<Eigen::Isometry3d> initial_poses;
  std::vector<BetweenConstraint> between_constraints;
  std::vector<PositionConstraint> position_constraints;
  int fixed_pose_index{0};
  int max_iterations{10};
  std::string save_path;
};

/*
Summary:
Contains the post-solve error of one relative-pose constraint, separated into
physical translation and rotation blocks.
*/
struct ConstraintResidual
{
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rotation{Eigen::Vector3d::Zero()};
  double chi2{0.0};
};

/*
Summary:
Reports the optimized poses, residuals, timing, serialization outcome, and
incremental-backend activity for one optimization request.

Important behavior:
constraint_residuals follows the same ordering as the input between constraints.
Incremental fields remain at their defaults when the batch backend is used.
*/
struct PoseGraphResult
{
  bool success{false};
  std::string error_message;
  std::vector<Eigen::Isometry3d> poses;
  // One entry per PoseGraphProblem::between_constraints item, in the same
  // order. These are native post-solve residuals split into physical blocks.
  std::vector<ConstraintResidual> constraint_residuals;
  int iterations{0};
  double graph_build_ms{0.0};
  double optimize_ms{0.0};
  double graph_save_ms{0.0};
  bool graph_save_requested{false};
  bool graph_saved{false};
  std::string graph_save_error;
  // Incremental-backend observability. Batch backends leave these at their
  // defaults. An iSAM2 update can still be successful with zero additions
  // when the caller only requests graph serialization.
  bool incremental_backend{false};
  bool state_rebuilt{false};
  std::string state_rebuild_reason;
  std::size_t variables_added{0};
  std::size_t factors_added{0};
  std::size_t total_variables{0};
  std::size_t total_factors{0};
};

/*
Summary:
Owns one selected pose-graph backend and presents a common optimization API.

Ownership and threading:
Batch GTSAM performs a new solve per request. GTSAM iSAM2 retains its Bayes
tree, factors, and estimates across requests. The graph component owns one
session for its lifetime and serializes calls into it.
*/
class PoseGraphOptimizer
{
public:
  /*
  Summary:
  Creates an optimizer session for the selected backend.
  */
  explicit PoseGraphOptimizer(PoseGraphBackend backend);

  /*
  Summary:
  Releases the selected backend and any persistent incremental state.
  */
  ~PoseGraphOptimizer();

  /*
  Summary:
  Prevents copying because an optimizer session uniquely owns backend state.
  */
  PoseGraphOptimizer(const PoseGraphOptimizer &) = delete;

  /*
  Summary:
  Prevents copy assignment because backend state has one owning session.
  */
  PoseGraphOptimizer & operator=(const PoseGraphOptimizer &) = delete;

  /*
  Summary:
  Transfers ownership of the selected backend and any incremental state.
  */
  PoseGraphOptimizer(PoseGraphOptimizer &&) noexcept;

  /*
  Summary:
  Replaces this session by moving ownership from another optimizer.
  */
  PoseGraphOptimizer & operator=(PoseGraphOptimizer &&) noexcept;

  /*
  Summary:
  Solves the supplied complete graph snapshot using this session's backend.

  Returns:
  Corrected poses plus residual, timing, save, and incremental diagnostics.
  */
  PoseGraphResult optimize(const PoseGraphProblem & problem);

  /*
  Summary:
  Discards persistent backend state without changing the selected backend.
  */
  void reset();

  /*
  Summary:
  Returns the backend selected when this session was constructed.
  */
  PoseGraphBackend backend() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

/*
Summary:
Parses a case-insensitive backend name. Unknown values resolve to the live
default, GTSAM_ISAM2; callers that need validation should call
isPoseGraphBackendName first.
*/
PoseGraphBackend parsePoseGraphBackend(const std::string & value);

/*
Summary:
Returns the canonical ROS parameter spelling for a backend value.
*/
const char * poseGraphBackendName(PoseGraphBackend backend);

/*
Summary:
Checks whether a string names a supported optimizer backend or alias.
*/
bool isPoseGraphBackendName(const std::string & value);

/*
Summary:
Runs one optimization request using a temporary optimizer session.

Important behavior:
This helper is appropriate for batch solves and tests. A temporary iSAM2
session cannot preserve incremental state after the function returns.
*/
PoseGraphResult optimizePoseGraph(
  const PoseGraphProblem & problem,
  PoseGraphBackend backend);

} // namespace optimization
} // namespace graphslam

#endif // GRAPH_BASED_SLAM__POSE_GRAPH_OPTIMIZER_HPP_
