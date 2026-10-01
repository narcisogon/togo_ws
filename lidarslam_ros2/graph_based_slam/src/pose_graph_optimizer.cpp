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
Implements backend-name handling and the public PoseGraphOptimizer dispatcher.
The dispatcher keeps ROS-facing code independent of GTSAM headers and owns the
persistent iSAM2 adapter only when the incremental backend is selected.
*/

#include "graph_based_slam/pose_graph_optimizer.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <utility>

#include "pose_graph_optimizer_internal.hpp"

namespace graphslam
{
namespace optimization
{
namespace
{
/*
Summary:
Normalizes an ASCII parameter value for case-insensitive backend matching.
*/
std::string lowerAscii(const std::string & value)
{
  std::string result;
  result.reserve(value.size());
  for (const char ch : value) {
    result.push_back(
        static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
  }
  return result;
}
} // namespace

/*
Summary:
Converts a supported backend name or alias into its enum representation.

Important behavior:
Unknown input falls back to GTSAM_ISAM2. The graph component validates names
before parsing so configuration mistakes still produce a warning.
*/
PoseGraphBackend parsePoseGraphBackend(const std::string & value)
{
  const std::string normalized = lowerAscii(value);
  if (normalized == "gtsam_isam2" || normalized == "isam2") {
    return PoseGraphBackend::GTSAM_ISAM2;
  }
  return normalized == "gtsam" ? PoseGraphBackend::GTSAM :
         PoseGraphBackend::GTSAM_ISAM2;
}

/*
Summary:
Returns the stable parameter and diagnostic name for a backend enum value.
*/
const char * poseGraphBackendName(PoseGraphBackend backend)
{
  switch (backend) {
    case PoseGraphBackend::GTSAM_ISAM2:
      return "gtsam_isam2";
    case PoseGraphBackend::GTSAM:
      return "gtsam";
    default:
      return "gtsam_isam2";
  }
}

/*
Summary:
Reports whether a case-insensitive string is a supported backend name or alias.
*/
bool isPoseGraphBackendName(const std::string & value)
{
  const std::string normalized = lowerAscii(value);
  return normalized == "gtsam" || normalized == "gtsam_isam2" ||
         normalized == "isam2";
}

/*
Summary:
Hides backend-specific state from the installed public header. Only iSAM2 needs
retained state; batch GTSAM dispatches directly to its one-shot implementation.
*/
class PoseGraphOptimizer::Impl
{
public:
  /*
  Summary:
  Stores the selected backend and creates persistent iSAM2 state when needed.
  */
  explicit Impl(PoseGraphBackend backend_in)
  : backend(backend_in)
  {
    if (backend == PoseGraphBackend::GTSAM_ISAM2) {
      incremental_gtsam = detail::makeIncrementalGtsamOptimizer();
    }
  }

  /*
  Summary:
  Routes a graph snapshot to the selected batch or incremental implementation.
  */
  PoseGraphResult optimize(const PoseGraphProblem & problem)
  {
    switch (backend) {
      case PoseGraphBackend::GTSAM_ISAM2:
        return incremental_gtsam->optimize(problem);
      case PoseGraphBackend::GTSAM:
        return detail::optimizePoseGraphGtsam(problem);
      default:
        return incremental_gtsam->optimize(problem);
    }
  }

  /*
  Summary:
  Clears retained iSAM2 state; batch GTSAM has no session state to clear.
  */
  void reset()
  {
    if (incremental_gtsam) {
      incremental_gtsam->reset();
    }
  }

  PoseGraphBackend backend;
  std::unique_ptr<detail::IncrementalGtsamOptimizer> incremental_gtsam;
};

/*
Summary:
Constructs the private dispatcher for the selected backend.
*/
PoseGraphOptimizer::PoseGraphOptimizer(PoseGraphBackend backend)
: impl_(std::make_unique<Impl>(backend))
{
}

/*
Summary:
Releases the private backend implementation and retained state.
*/
PoseGraphOptimizer::~PoseGraphOptimizer() = default;

/*
Summary:
Transfers ownership of a private backend implementation into a new session.
*/
PoseGraphOptimizer::PoseGraphOptimizer(PoseGraphOptimizer &&) noexcept = default;

/*
Summary:
Replaces this private backend implementation by move assignment.
*/
PoseGraphOptimizer & PoseGraphOptimizer::operator=(PoseGraphOptimizer &&) noexcept = default;

/*
Summary:
Optimizes one complete graph snapshot through this session's backend.
*/
PoseGraphResult PoseGraphOptimizer::optimize(const PoseGraphProblem & problem)
{
  return impl_->optimize(problem);
}

/*
Summary:
Forgets retained backend state without replacing the selected backend.
*/
void PoseGraphOptimizer::reset()
{
  impl_->reset();
}

/*
Summary:
Returns the backend owned by this optimizer session.
*/
PoseGraphBackend PoseGraphOptimizer::backend() const
{
  return impl_->backend;
}

/*
Summary:
Runs a one-shot solve by constructing a temporary backend session.
*/
PoseGraphResult optimizePoseGraph(
  const PoseGraphProblem & problem,
  PoseGraphBackend backend)
{
  PoseGraphOptimizer optimizer(backend);
  return optimizer.optimize(problem);
}

} // namespace optimization
} // namespace graphslam
