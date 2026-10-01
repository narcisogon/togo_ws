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
Declares the private boundary between the public pose-graph dispatcher and the
GTSAM implementations. It lives in src because these symbols are implementation
details and are not installed as part of the package's public API.
*/

#ifndef GRAPH_BASED_SLAM__POSE_GRAPH_OPTIMIZER_INTERNAL_HPP_
#define GRAPH_BASED_SLAM__POSE_GRAPH_OPTIMIZER_INTERNAL_HPP_

#include <memory>

#include "graph_based_slam/pose_graph_optimizer.hpp"

namespace graphslam
{
namespace optimization
{
namespace detail
{

/*
Summary:
Builds and solves a complete graph with GTSAM Levenberg-Marquardt.
*/
PoseGraphResult optimizePoseGraphGtsam(const PoseGraphProblem & problem);

/*
Summary:
Defines the private stateful interface implemented by the GTSAM iSAM2 adapter.
*/
class IncrementalGtsamOptimizer
{
public:
  /*
  Summary:
  Releases the incremental graph, estimate, and Bayes-tree state.
  */
  virtual ~IncrementalGtsamOptimizer() = default;

  /*
  Summary:
  Applies a complete graph snapshot to the persistent incremental state.
  */
  virtual PoseGraphResult optimize(const PoseGraphProblem & problem) = 0;

  /*
  Summary:
  Clears all retained state so the next optimization starts from initialization.
  */
  virtual void reset() = 0;
};

/*
Summary:
Creates the private concrete GTSAM iSAM2 implementation.
*/
std::unique_ptr<IncrementalGtsamOptimizer> makeIncrementalGtsamOptimizer();

} // namespace detail
} // namespace optimization
} // namespace graphslam

#endif // GRAPH_BASED_SLAM__POSE_GRAPH_OPTIMIZER_INTERNAL_HPP_
