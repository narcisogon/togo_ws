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
Verifies the backend-neutral optimizer contract and the behavioral agreement
between batch GTSAM and persistent GTSAM iSAM2.

Coverage:
The tests protect backend parsing, rotation/translation information ordering,
graph serialization, append-only incremental updates, loop-factor propagation,
and safe rebuilding after historical factor changes.
*/

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <filesystem>
#include <string>

#include "graph_based_slam/pose_graph_optimizer.hpp"

namespace
{
using graphslam::optimization::BetweenConstraint;
using graphslam::optimization::ConstraintKind;
using graphslam::optimization::PoseGraphBackend;
using graphslam::optimization::PoseGraphOptimizer;
using graphslam::optimization::PoseGraphProblem;
using graphslam::optimization::PositionConstraint;

/*
Summary:
Builds a simple map-frame pose from translation and yaw for test graphs.
*/
Eigen::Isometry3d pose(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() = Eigen::Vector3d(x, y, z);
  value.linear() =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return value;
}

/*
Summary:
Builds a relative-pose constraint with isotropic translation and rotation precision.
*/
BetweenConstraint between(
  int from, int to,
  const Eigen::Isometry3d & measurement,
  double translation_weight, double rotation_weight)
{
  BetweenConstraint constraint;
  constraint.from = from;
  constraint.to = to;
  constraint.measurement = measurement;
  constraint.translation_information =
    Eigen::Matrix3d::Identity() * translation_weight;
  constraint.rotation_information =
    Eigen::Matrix3d::Identity() * rotation_weight;
  return constraint;
}

/*
Summary:
Extracts planar yaw from a test pose for rotation-order assertions.
*/
double yawOf(const Eigen::Isometry3d & value)
{
  return std::atan2(value.rotation()(1, 0), value.rotation()(0, 0));
}

/*
Summary:
Verifies accepted aliases, case handling, canonical names, and safe invalid-name fallback.
*/
TEST(PoseGraphOptimizer, ParsesBackendNamesCaseInsensitively) {
  using graphslam::optimization::isPoseGraphBackendName;
  using graphslam::optimization::parsePoseGraphBackend;
  EXPECT_FALSE(isPoseGraphBackendName("g2o"));
  EXPECT_TRUE(isPoseGraphBackendName("GTSAM"));
  EXPECT_TRUE(isPoseGraphBackendName("isam2"));
  EXPECT_TRUE(isPoseGraphBackendName("GTSAM_ISAM2"));
  EXPECT_EQ(parsePoseGraphBackend("invalid"), PoseGraphBackend::GTSAM_ISAM2);
  EXPECT_EQ(parsePoseGraphBackend("gtsam"), PoseGraphBackend::GTSAM);
  EXPECT_EQ(parsePoseGraphBackend("isam2"), PoseGraphBackend::GTSAM_ISAM2);
  EXPECT_STREQ(
    graphslam::optimization::poseGraphBackendName(
      parsePoseGraphBackend("GTSAM_ISAM2")),
    "gtsam_isam2");
}

/*
Summary:
Verifies that batch GTSAM and one-session iSAM2 converge to the same loop-corrected graph.
*/
TEST(PoseGraphOptimizer, BatchGtsamAndIsam2AgreeOnTheSameGraph) {
  PoseGraphProblem problem;
  problem.initial_poses = {
    pose(0.0, 0.0, 0.0, 0.0),
    pose(1.15, 0.08, 0.0, 0.03),
    pose(2.30, 0.12, 0.0, 0.05),
  };
  problem.between_constraints.push_back(
      between(0, 1, pose(1.0, 0.0, 0.0, 0.0), 100.0, 80.0));
  problem.between_constraints.push_back(
      between(1, 2, pose(1.0, 0.0, 0.0, 0.0), 100.0, 80.0));
  auto loop = between(0, 2, pose(1.8, 0.0, 0.0, 0.0), 20.0, 20.0);
  loop.kind = ConstraintKind::LOOP;
  problem.between_constraints.push_back(loop);
  problem.max_iterations = 50;

  const auto batch_result = graphslam::optimization::optimizePoseGraph(
      problem, PoseGraphBackend::GTSAM);
  const auto isam2_result = graphslam::optimization::optimizePoseGraph(
      problem, PoseGraphBackend::GTSAM_ISAM2);

  ASSERT_TRUE(batch_result.success) << batch_result.error_message;
  ASSERT_TRUE(isam2_result.success) << isam2_result.error_message;
  ASSERT_EQ(batch_result.poses.size(), isam2_result.poses.size());
  for (std::size_t i = 0; i < batch_result.poses.size(); ++i) {
    EXPECT_LT((batch_result.poses[i].translation() -
      isam2_result.poses[i].translation())
      .norm(),
              1.0e-4);
    EXPECT_LT(Eigen::AngleAxisd(batch_result.poses[i].rotation().transpose() *
                                isam2_result.poses[i].rotation())
      .angle(),
              1.0e-4);
  }
}

/*
Summary:
Protects the public translation/rotation information blocks from a tangent-order swap.
*/
TEST(PoseGraphOptimizer, TranslationAndRotationInformationAreNotSwapped) {
  for (const auto backend : std::array<PoseGraphBackend, 2>{
      PoseGraphBackend::GTSAM, PoseGraphBackend::GTSAM_ISAM2})
  {
    PoseGraphProblem problem;
    problem.initial_poses = {
      pose(0.0, 0.0, 0.0, 0.0),
      pose(1.0, 0.0, 0.0, 0.25),
    };
    problem.between_constraints.push_back(
        between(0, 1, pose(1.0, 0.0, 0.0, 0.25), 1.0, 100.0));

    PositionConstraint position_constraint;
    position_constraint.pose_index = 1;
    position_constraint.position = Eigen::Vector3d(4.0, 0.0, 0.0);
    position_constraint.information = Eigen::Matrix3d::Identity() * 1000.0;
    problem.position_constraints.push_back(position_constraint);
    problem.max_iterations = 50;

    const auto result =
      graphslam::optimization::optimizePoseGraph(problem, backend);
    ASSERT_TRUE(result.success) << result.error_message;
    ASSERT_EQ(result.poses.size(), 2U);
    EXPECT_GT(result.poses[1].translation().x(), 3.9);
    EXPECT_NEAR(yawOf(result.poses[1]), 0.25, 1.0e-3);
  }
}

/*
Summary:
Verifies that both GTSAM modes emit a non-empty reusable pose-graph artifact.
*/
TEST(PoseGraphOptimizer, GtsamModesSerializeAReusableGraphArtifact) {
  for (const auto backend : std::array<PoseGraphBackend, 2>{
      PoseGraphBackend::GTSAM, PoseGraphBackend::GTSAM_ISAM2})
  {
    PoseGraphProblem problem;
    problem.initial_poses = {
      pose(0.0, 0.0, 0.0, 0.0),
      pose(1.1, 0.0, 0.0, 0.0),
    };
    problem.between_constraints.push_back(
      between(0, 1, pose(1.0, 0.0, 0.0, 0.0), 100.0, 100.0));
    const std::string backend_name =
      graphslam::optimization::poseGraphBackendName(backend);
    const auto graph_path = std::filesystem::temp_directory_path() /
      ("graph_based_slam_" + backend_name + "_test.g2o");
    std::error_code remove_error;
    std::filesystem::remove(graph_path, remove_error);
    problem.save_path = graph_path.string();

    const auto result =
      graphslam::optimization::optimizePoseGraph(problem, backend);
    EXPECT_TRUE(result.success) << result.error_message;
    EXPECT_TRUE(result.graph_save_requested);
    EXPECT_TRUE(result.graph_saved) << result.graph_save_error;
    EXPECT_TRUE(std::filesystem::exists(graph_path));
    if (std::filesystem::exists(graph_path)) {
      EXPECT_GT(std::filesystem::file_size(graph_path), 0U);
    }
    std::filesystem::remove(graph_path, remove_error);
  }
}

/*
Summary:
Verifies that new adjacent poses and factors extend iSAM2 without rebuilding history.
*/
TEST(PoseGraphOptimizer, Isam2AppendsNewPosesAndFactorsWithoutRebuilding) {
  PoseGraphOptimizer optimizer(PoseGraphBackend::GTSAM_ISAM2);
  PoseGraphProblem problem;
  problem.initial_poses = {
    pose(0.0, 0.0, 0.0, 0.0),
    pose(1.0, 0.0, 0.0, 0.0),
  };
  problem.between_constraints.push_back(
    between(0, 1, pose(1.0, 0.0, 0.0, 0.0), 100.0, 100.0));

  const auto first = optimizer.optimize(problem);
  ASSERT_TRUE(first.success) << first.error_message;
  EXPECT_TRUE(first.incremental_backend);
  EXPECT_TRUE(first.state_rebuilt);
  EXPECT_EQ(first.state_rebuild_reason, "initialization");
  EXPECT_EQ(first.variables_added, 2U);
  EXPECT_EQ(first.factors_added, 2U);  // anchor + odometry

  problem.initial_poses.push_back(pose(2.0, 0.0, 0.0, 0.0));
  problem.between_constraints.push_back(
    between(1, 2, pose(1.0, 0.0, 0.0, 0.0), 100.0, 100.0));
  const auto second = optimizer.optimize(problem);
  ASSERT_TRUE(second.success) << second.error_message;
  EXPECT_FALSE(second.state_rebuilt);
  EXPECT_EQ(second.variables_added, 1U);
  EXPECT_EQ(second.factors_added, 1U);
  EXPECT_EQ(second.total_variables, 3U);
  EXPECT_EQ(second.total_factors, 3U);
  ASSERT_EQ(second.poses.size(), 3U);
  EXPECT_NEAR(second.poses[2].translation().x(), 2.0, 1.0e-6);
}

/*
Summary:
Verifies that a new loop factor is applied incrementally and agrees with batch GTSAM.
*/
TEST(PoseGraphOptimizer, Isam2IncrementallyAppliesANewLoopConstraint) {
  PoseGraphOptimizer optimizer(PoseGraphBackend::GTSAM_ISAM2);
  PoseGraphProblem problem;
  problem.initial_poses = {
    pose(0.0, 0.0, 0.0, 0.0),
    pose(1.0, 0.0, 0.0, 0.0),
    pose(2.0, 0.0, 0.0, 0.0),
  };
  problem.between_constraints.push_back(
    between(0, 1, pose(1.0, 0.0, 0.0, 0.0), 100.0, 100.0));
  problem.between_constraints.push_back(
    between(1, 2, pose(1.0, 0.0, 0.0, 0.0), 100.0, 100.0));
  ASSERT_TRUE(optimizer.optimize(problem).success);

  auto loop = between(0, 2, pose(1.8, 0.0, 0.0, 0.0), 20.0, 20.0);
  loop.kind = ConstraintKind::LOOP;
  problem.between_constraints.push_back(loop);
  problem.max_iterations = 10;

  const auto incremental = optimizer.optimize(problem);
  const auto batch = graphslam::optimization::optimizePoseGraph(
    problem, PoseGraphBackend::GTSAM);
  ASSERT_TRUE(incremental.success) << incremental.error_message;
  ASSERT_TRUE(batch.success) << batch.error_message;
  EXPECT_FALSE(incremental.state_rebuilt);
  EXPECT_EQ(incremental.variables_added, 0U);
  EXPECT_EQ(incremental.factors_added, 1U);
  EXPECT_GT(incremental.iterations, 1);
  ASSERT_EQ(incremental.poses.size(), batch.poses.size());
  for (std::size_t i = 0; i < incremental.poses.size(); ++i) {
    EXPECT_LT(
      (incremental.poses[i].translation() - batch.poses[i].translation()).norm(),
      1.0e-4);
  }
}

/*
Summary:
Verifies safe iSAM2 reconstruction when an already-incorporated factor changes.
*/
TEST(PoseGraphOptimizer, Isam2RebuildsWhenAHistoricalFactorChanges) {
  PoseGraphOptimizer optimizer(PoseGraphBackend::GTSAM_ISAM2);
  PoseGraphProblem problem;
  problem.initial_poses = {
    pose(0.0, 0.0, 0.0, 0.0),
    pose(1.0, 0.0, 0.0, 0.0),
  };
  problem.between_constraints.push_back(
    between(0, 1, pose(1.0, 0.0, 0.0, 0.0), 100.0, 100.0));
  ASSERT_TRUE(optimizer.optimize(problem).success);

  problem.between_constraints[0].measurement = pose(0.9, 0.0, 0.0, 0.0);
  const auto changed = optimizer.optimize(problem);
  ASSERT_TRUE(changed.success) << changed.error_message;
  EXPECT_TRUE(changed.state_rebuilt);
  EXPECT_EQ(changed.state_rebuild_reason, "between_factor_changed");
  EXPECT_EQ(changed.variables_added, 2U);
  EXPECT_EQ(changed.factors_added, 2U);
}
} // namespace
