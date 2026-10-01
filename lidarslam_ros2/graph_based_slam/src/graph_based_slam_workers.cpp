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
Implements graph-backend scheduling and shared-state snapshots. Separate search
and publication workers keep expensive loop detection, optimization, map
publication, and save requests off ROS subscription callbacks.
*/

#include "graph_based_slam/graph_based_slam_component.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <numeric>
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "graph_based_slam/adjacent_edge_auto_scale.hpp"
#include "graph_based_slam/bev_mutual_visibility.hpp"
#include "graph_based_slam/dynamic_object_filter.hpp"
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace graphslam
{
/*
Summary:
Publishes one structured loop-closure diagnostic payload when subscribers exist.
*/
void GraphBasedSlamComponent::publishLoopDiagnostic(const std::string & payload)
{
  if (!loop_diagnostics_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = payload;
  loop_diagnostics_pub_->publish(msg);
}

/*
Summary:
Publishes one structured backend timing payload when subscribers exist.
*/
void GraphBasedSlamComponent::publishBackendTimingDiagnostic(const std::string & payload)
{
  if (!backend_timing_diagnostics_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = payload;
  backend_timing_diagnostics_pub_->publish(msg);
}

/*
Summary:
Converts a ROS map-save service call into an asynchronous publication-worker
request so disk I/O does not block the executor.
*/
void GraphBasedSlamComponent::handleMapSaveRequest(
  const MapSaveRequestHeader request_header,
  const MapSaveRequest request,
  const MapSaveResponse response)
{
  static_cast<void>(request_header);
  static_cast<void>(request);
  static_cast<void>(response);

  std::cout << "Received a request to save the map" << std::endl;
  save_map_requested_ = true;
  publish_worker_cv_.notify_one();
  std::cout << "Map save requested; will complete asynchronously on backend worker" << std::endl;
}

/*
Summary:
Publishes cached corrected products when valid or rebuilds them from a fresh
graph snapshot when the graph is dirty.
*/
void GraphBasedSlamComponent::publishMapAndPose()
{
  lidarslam_msgs::msg::MapArray map_array_msg;
  LoopEdges loop_edges;
  if (!snapshotGraphState(map_array_msg, loop_edges, false)) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for initial map before periodic modified_map publish");
    return;
  }

  // Decide whether the cached /modified_map from the last real rebuild is
  // still valid, or whether a full doPoseAdjustment() is needed. This whole
  // decision (and the cheap-republish path below) runs under
  // modified_map_publish_mtx_ so it's serialized against doPoseAdjustment()
  // writing the same cached_* fields from either worker thread. The lock is
  // released (block scope ends) before falling through to doPoseAdjustment()
  // below -- std::mutex isn't recursive, and doPoseAdjustment() takes the
  // same lock itself.
  bool need_full_rebuild = true;
  {
    std::lock_guard<std::mutex> lock(modified_map_publish_mtx_);
    const bool unchanged =
      !graph_dirty_.load() &&
      cached_map_valid_ &&
      static_cast<int>(map_array_msg.submaps.size()) == cached_map_submap_count_ &&
      loop_edges.size() == cached_map_loop_edge_count_;
    if (unchanged) {
      need_full_rebuild = false;
      // Heartbeat republish: identical points, fresh stamp. Keeps the
      // periodic publish contract for downstream consumers without paying
      // for pose-graph optimization, PCD reloads, transforms, or voxelization.
      const auto stamp = this->now();
      if (publish_modified_map_ && modified_map_pub_) {
        cached_modified_map_msg_.header.stamp = stamp;
        modified_map_pub_->publish(cached_modified_map_msg_);
      }
      if (publish_modified_map_timed_ && modified_map_timed_pub_) {
        cached_modified_map_timed_msg_.header.stamp = stamp;
        modified_map_timed_pub_->publish(cached_modified_map_timed_msg_);
      }
      publishMapToOdomTf(stamp);
      last_periodic_output_publish_time_ = std::chrono::steady_clock::now();
    }
  }
  if (!need_full_rebuild) {
    return;
  }

  doPoseAdjustment(map_array_msg, loop_edges, false);
}

/*
Summary:
Runs periodic loop search and triggers optimization after newly accepted loop
constraints until shutdown is requested.
*/
void GraphBasedSlamComponent::searchWorkerLoop()
{
  // Local timing helper: elapsedMillis() (used by searchLoop/doPoseAdjustment
  // below) is defined later in this translation unit, after this function.
  const auto elapsed_ms = [](const std::chrono::steady_clock::time_point & t) {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t).count();
    };

  while (!shutting_down_) {
    {
      std::unique_lock<std::mutex> lk(search_worker_cv_mtx_);
      search_worker_cv_.wait_for(
        lk, std::chrono::milliseconds(loop_detection_period_),
        [this] {return shutting_down_.load();});
    }
    if (shutting_down_) {
      break;
    }

    const auto iteration_start = std::chrono::steady_clock::now();

    // Loop search and descriptor DB maintenance. No longer triggers
    // doPoseAdjustment directly -- it only sets optimize_requested_.
    auto stage_start = std::chrono::steady_clock::now();
    searchLoop();
    const double search_loop_ms = elapsed_ms(stage_start);

    const bool do_optimize = optimize_requested_.exchange(false);
    double pose_adjustment_ms = 0.0;
    if (do_optimize) {
      lidarslam_msgs::msg::MapArray snapshot;
      LoopEdges edges;
      stage_start = std::chrono::steady_clock::now();
      if (snapshotGraphState(snapshot, edges, false)) {
        doPoseAdjustment(snapshot, edges, use_save_map_in_loop_);
      }
      pose_adjustment_ms = elapsed_ms(stage_start);
    }

    {
      std::ostringstream diag;
      diag << std::fixed << std::setprecision(3)
           << "{\"event\":\"search_worker_iteration_timing\""
           << ",\"search_loop_ms\":" << search_loop_ms
           << ",\"pose_adjustment_triggered\":" << (do_optimize ? "true" : "false")
           << ",\"pose_adjustment_ms\":" << pose_adjustment_ms
           << ",\"total_ms\":" << elapsed_ms(iteration_start)
           << "}";
      publishBackendTimingDiagnostic(diag.str());
    }
  }
}

/*
Summary:
Services periodic corrected-map publication and queued map-save requests on a
worker independent from loop search.
*/
void GraphBasedSlamComponent::publishWorkerLoop()
{
  // Runs independently of searchWorkerLoop() so a slow loop-search catch-up
  // (which can take well over a second while ingesting many new submaps)
  // never delays periodic /modified_map publication or /map_save.
  const auto elapsed_ms = [](const std::chrono::steady_clock::time_point & t) {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t).count();
    };

  while (!shutting_down_) {
    // Check more frequently than the heartbeat deadline. Sleeping for the
    // full period could turn a two-second maximum-silence contract into
    // almost four seconds when a real publish occurs just after a check.
    const auto wait_ms = modified_map_publish_period_sec_ > 0.0 ?
      std::clamp(
      static_cast<int>(modified_map_publish_period_sec_ * 250.0), 50, 500) :
      1000;
    {
      std::unique_lock<std::mutex> lk(publish_worker_cv_mtx_);
      publish_worker_cv_.wait_for(
        lk, std::chrono::milliseconds(wait_ms),
        [this] {return shutting_down_.load() || save_map_requested_.load();});
    }
    if (shutting_down_) {
      break;
    }

    const auto iteration_start = std::chrono::steady_clock::now();

    const bool save_triggered = save_map_requested_.exchange(false);
    double save_ms = 0.0;
    if (save_triggered) {
      lidarslam_msgs::msg::MapArray snapshot;
      LoopEdges edges;
      const auto stage_start = std::chrono::steady_clock::now();
      if (snapshotGraphState(snapshot, edges, false)) {
        doPoseAdjustment(snapshot, edges, true);
      }
      save_ms = elapsed_ms(stage_start);
    }

    bool periodic_publish_triggered = false;
    double periodic_publish_ms = 0.0;
    double since_last_periodic_publish_sec = -1.0;
    // Skip the periodic check this tick if a save just ran -- it already
    // published an equivalent up-to-date map.
    if (modified_map_publish_period_sec_ > 0.0 && !save_triggered) {
      const auto now = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(modified_map_publish_mtx_);
        since_last_periodic_publish_sec =
          last_periodic_output_publish_time_.time_since_epoch().count() == 0 ?
          modified_map_publish_period_sec_ :
          std::chrono::duration<double>(
          now - last_periodic_output_publish_time_).count();
      }
      if (since_last_periodic_publish_sec >= modified_map_publish_period_sec_) {
        const auto stage_start = std::chrono::steady_clock::now();
        publishMapAndPose();
        periodic_publish_ms = elapsed_ms(stage_start);
        periodic_publish_triggered = true;
      }
    }

    {
      std::ostringstream diag;
      diag << std::fixed << std::setprecision(3)
           << "{\"event\":\"publish_worker_iteration_timing\""
           << ",\"save_triggered\":" << (save_triggered ? "true" : "false")
           << ",\"save_ms\":" << save_ms
           << ",\"periodic_publish_triggered\":" << (periodic_publish_triggered ? "true" : "false")
           << ",\"periodic_publish_ms\":" << periodic_publish_ms
           // Actual measured gap since the last successful periodic publish,
           // vs. the configured modified_map_publish_period_sec_ target --
           // this is the number that directly answers "the modified map
           // took a while." -1 when periodic publishing is disabled.
           << ",\"since_last_periodic_publish_sec\":" << since_last_periodic_publish_sec
           << ",\"total_ms\":" << elapsed_ms(iteration_start)
           << "}";
      publishBackendTimingDiagnostic(diag.str());
    }
  }
}

/*
Summary:
Copies the current map array and loop edges under lock and optionally consumes
the map-updated flag.
*/
bool GraphBasedSlamComponent::snapshotGraphState(
  lidarslam_msgs::msg::MapArray & map_array_msg,
  LoopEdges & loop_edges,
  bool consume_map_update)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!initial_map_array_received_) {
    return false;
  }
  if (consume_map_update && !is_map_array_updated_) {
    return false;
  }

  map_array_msg = map_array_msg_;
  loop_edges = loop_edges_;
  if (consume_map_update) {
    is_map_array_updated_ = false;
  }
  return true;
}

/*
Summary:
Copies the accepted loop-edge set under the graph-state mutex.
*/
void GraphBasedSlamComponent::snapshotLoopEdges(LoopEdges & loop_edges)
{
  std::lock_guard<std::mutex> lock(mtx_);
  loop_edges = loop_edges_;
}

/*
Summary:
Inserts or improves a loop constraint, marks graph products dirty, and reports
whether the stored edge set changed.
*/
bool GraphBasedSlamComponent::upsertLoopEdge(const LoopEdge & loop_edge)
{
  if (loop_edge.pair_id.first < 0 || loop_edge.pair_id.second < 0) {
    return false;
  }

  LoopEdge normalized = loop_edge;
  if (normalized.pair_id.first > normalized.pair_id.second) {
    std::swap(normalized.pair_id.first, normalized.pair_id.second);
    normalized.relative_pose = normalized.relative_pose.inverse();
  }
  if (normalized.pair_id.first == normalized.pair_id.second) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mtx_);
  auto is_nearby_pair = [this](const LoopEdge & lhs, const LoopEdge & rhs) {
      return std::abs(lhs.pair_id.first - rhs.pair_id.first) <= loop_edge_dedup_index_window_ &&
             std::abs(lhs.pair_id.second - rhs.pair_id.second) <= loop_edge_dedup_index_window_;
    };
  for (auto & existing : loop_edges_) {
    if (!is_nearby_pair(existing, normalized)) {
      continue;
    }
    if (existing.fitness_score > 0.0 &&
      normalized.fitness_score >= existing.fitness_score)
    {
      return false;
    }
    existing = normalized;
    graph_dirty_ = true;
    return true;
  }

  loop_edges_.push_back(normalized);
  graph_dirty_ = true;
  return true;
}
}  // namespace graphslam
