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
Defines backend-neutral robust-loss selection for loop-closure constraints.
Optimizer adapters and ROS parameter validation share this vocabulary without
exposing GTSAM headers to the graph component.

Kernel behavior:
Huber clips influence above a threshold but does not drive it to zero. Dynamic
Covariance Scaling strongly suppresses outlying loops without extra variables.
Cauchy provides a smooth middle ground.
*/

#ifndef GRAPH_BASED_SLAM__LOOP_EDGE_ROBUSTIFIER_HPP_
#define GRAPH_BASED_SLAM__LOOP_EDGE_ROBUSTIFIER_HPP_

#include <algorithm>
#include <cctype>
#include <string>

namespace graphslam
{
namespace robust
{

/*
Summary:
Lists robust losses supported for loop-closure factors.
*/
enum class LoopEdgeKernelType
{
  Huber,
  DCS,
  Cauchy,
};

/*
Summary:
Normalizes an ASCII parameter value for case-insensitive kernel matching.
*/
inline std::string toLowerAscii(const std::string & input)
{
  std::string out;
  out.reserve(input.size());
  for (char ch : input) {
    out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
  }
  return out;
}

/*
Summary:
Parses a ROS parameter string into a kernel enum. Unknown values safely fall
back to Huber.
*/
inline LoopEdgeKernelType parseLoopEdgeKernelType(const std::string & raw)
{
  const std::string lower = toLowerAscii(raw);
  if (lower == "dcs") {return LoopEdgeKernelType::DCS;}
  if (lower == "cauchy") {return LoopEdgeKernelType::Cauchy;}
  return LoopEdgeKernelType::Huber;
}

/*
Summary:
Returns the canonical parameter and diagnostic name for a kernel type.
*/
inline const char * loopEdgeKernelTypeName(LoopEdgeKernelType type)
{
  switch (type) {
    case LoopEdgeKernelType::DCS:
      return "dcs";
    case LoopEdgeKernelType::Cauchy:
      return "cauchy";
    default:
      return "huber";
  }
}

}  // namespace robust
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__LOOP_EDGE_ROBUSTIFIER_HPP_
