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
Manages temporary per-run PCD storage and persistent corrected-map export.
Temporary cache sessions reduce long-run process memory while file locks prevent
one live graph process from deleting another process's submaps.

Storage boundary:
The PCD cache is disposable working state, commonly placed in /dev/shm.
Grid-divided maps, map.pcd, and projector metadata are persistent products under
map_save_dir.
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
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <system_error>
#include <unordered_map>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include "graph_based_slam/adjacent_edge_auto_scale.hpp"
#include "graph_based_slam/bev_mutual_visibility.hpp"
#include "graph_based_slam/dynamic_object_filter.hpp"
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace graphslam
{
namespace
{
constexpr char kPcdCacheRunPrefix[] = "run_";
constexpr char kPcdCacheLockFile[] = ".lock";
constexpr char kPcdCacheCleanupLockFile[] = ".cleanup.lock";

/*
Summary:
Checks whether a string begins with the cache-session naming prefix.
*/
bool startsWith(const std::string & value, const std::string & prefix)
{
  return value.size() >= prefix.size() &&
         value.compare(0, prefix.size(), prefix) == 0;
}
}  // namespace

/*
Summary:
Creates and locks a private PCD cache directory for this graph process.

Returns:
True when the session is ready; false when the component must fall back to
in-memory submap storage.

Important behavior:
Startup cleanup removes only abandoned run directories with an acquirable lock.
Unknown directories and cache sessions owned by live processes are preserved.
*/
bool GraphBasedSlamComponent::initializePcdCacheSession()
{
  namespace fs = std::filesystem;

  std::error_code error;
  fs::create_directories(pcd_cache_root_dir_, error);
  if (error) {
    RCLCPP_ERROR(
      get_logger(), "Could not create PCD cache root '%s': %s",
      pcd_cache_root_dir_.c_str(), error.message().c_str());
    return false;
  }

  const fs::path root(pcd_cache_root_dir_);
  const fs::path cleanup_lock_path = root / kPcdCacheCleanupLockFile;
  const int cleanup_lock_fd = open(
    cleanup_lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
  if (cleanup_lock_fd < 0 || flock(cleanup_lock_fd, LOCK_EX) != 0) {
    RCLCPP_ERROR(
      get_logger(), "Could not lock PCD cache root '%s' for session initialization",
      pcd_cache_root_dir_.c_str());
    if (cleanup_lock_fd >= 0) {
      close(cleanup_lock_fd);
    }
    return false;
  }

  std::vector<fs::path> existing_run_paths;
  for (fs::directory_iterator it(root, error), end; !error && it != end; it.increment(error)) {
    std::error_code type_error;
    if (it->is_directory(type_error) && !type_error &&
      startsWith(it->path().filename().string(), kPcdCacheRunPrefix))
    {
      existing_run_paths.push_back(it->path());
    }
  }
  if (error) {
    RCLCPP_WARN(
      get_logger(), "Could not completely scan PCD cache root '%s': %s",
      pcd_cache_root_dir_.c_str(), error.message().c_str());
  }

  std::size_t removed_run_count = 0;
  for (const auto & run_path : existing_run_paths) {
    const fs::path run_lock_path = run_path / kPcdCacheLockFile;
    std::error_code lock_file_error;
    if (!fs::is_regular_file(run_lock_path, lock_file_error) || lock_file_error) {
      // A run_* directory without our lock marker is not known to be owned by
      // this component, so leave it alone rather than guessing it is stale.
      continue;
    }
    const int run_lock_fd = open(run_lock_path.c_str(), O_RDWR | O_CLOEXEC);
    if (run_lock_fd < 0) {
      RCLCPP_WARN(
        get_logger(), "Could not inspect PCD cache session '%s'",
        run_path.string().c_str());
      continue;
    }

    if (flock(run_lock_fd, LOCK_EX | LOCK_NB) == 0) {
      std::error_code remove_error;
      fs::remove_all(run_path, remove_error);
      if (remove_error) {
        RCLCPP_WARN(
          get_logger(), "Could not remove abandoned PCD cache session '%s': %s",
          run_path.string().c_str(), remove_error.message().c_str());
      } else {
        ++removed_run_count;
      }
      flock(run_lock_fd, LOCK_UN);
    }
    close(run_lock_fd);
  }

  error.clear();
  const auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const fs::path run_path = root /
    (std::string(kPcdCacheRunPrefix) + std::to_string(getpid()) + "_" +
    std::to_string(timestamp_ns));
  fs::create_directory(run_path, error);
  if (error) {
    RCLCPP_ERROR(
      get_logger(), "Could not create PCD cache session '%s': %s",
      run_path.string().c_str(), error.message().c_str());
    flock(cleanup_lock_fd, LOCK_UN);
    close(cleanup_lock_fd);
    return false;
  }

  const fs::path run_lock_path = run_path / kPcdCacheLockFile;
  pcd_cache_lock_fd_ = open(
    run_lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
  if (pcd_cache_lock_fd_ < 0 || flock(pcd_cache_lock_fd_, LOCK_EX | LOCK_NB) != 0) {
    RCLCPP_ERROR(
      get_logger(), "Could not lock new PCD cache session '%s'",
      run_path.string().c_str());
    if (pcd_cache_lock_fd_ >= 0) {
      close(pcd_cache_lock_fd_);
      pcd_cache_lock_fd_ = -1;
    }
    std::error_code remove_error;
    fs::remove_all(run_path, remove_error);
    flock(cleanup_lock_fd, LOCK_UN);
    close(cleanup_lock_fd);
    return false;
  }

  pcd_cache_dir_ = run_path.string();
  flock(cleanup_lock_fd, LOCK_UN);
  close(cleanup_lock_fd);

  RCLCPP_INFO(
    get_logger(), "PCD cache session ready: root=%s run=%s abandoned_runs_removed=%zu",
    pcd_cache_root_dir_.c_str(), pcd_cache_dir_.c_str(), removed_run_count);
  return true;
}

/*
Summary:
Removes this process's temporary PCD cache directory and releases its lock.

Important behavior:
The component calls this only after background workers have joined so no worker
can access a submap file during removal.
*/
void GraphBasedSlamComponent::cleanupPcdCacheSession()
{
  if (pcd_cache_lock_fd_ < 0) {
    return;
  }

  std::error_code error;
  const std::uintmax_t removed = std::filesystem::remove_all(pcd_cache_dir_, error);
  if (error) {
    RCLCPP_WARN(
      get_logger(), "Could not remove PCD cache session '%s': %s",
      pcd_cache_dir_.c_str(), error.message().c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "Removed PCD cache session '%s' (%llu filesystem entries)",
      pcd_cache_dir_.c_str(), static_cast<unsigned long long>(removed));
  }

  flock(pcd_cache_lock_fd_, LOCK_UN);
  close(pcd_cache_lock_fd_);
  pcd_cache_lock_fd_ = -1;
  pcd_cache_dir_.clear();
}

/*
Summary:
Writes one submap cloud into the current cache session as compressed PCD.
*/
void GraphBasedSlamComponent::saveSubmapToPCD(
  int idx,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  std::string path = pcd_cache_dir_ + "/submap_" + std::to_string(idx) + ".pcd";
  pcl::io::savePCDFileBinaryCompressed(path, *cloud);
}

/*
Summary:
Loads one submap cloud from the current cache session.
*/
pcl::PointCloud<pcl::PointXYZI>::Ptr GraphBasedSlamComponent::loadSubmapFromPCD(int idx)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  std::string path = pcd_cache_dir_ + "/submap_" + std::to_string(idx) + ".pcd";
  if (pcl::io::loadPCDFile(path, *cloud) == -1) {
    RCLCPP_WARN(get_logger(), "Failed to load PCD: %s", path.c_str());
  }
  return cloud;
}

/*
Summary:
Returns a submap through the bounded in-process read cache, loading it from PCD
only on a cache miss.
*/
pcl::PointCloud<pcl::PointXYZI>::Ptr GraphBasedSlamComponent::loadSubmapFromPCDCached(int idx)
{
  const auto it = submap_cloud_cache_.find(idx);
  if (it != submap_cloud_cache_.end()) {
    return it->second;
  }
  auto cloud = loadSubmapFromPCD(idx);
  submap_cloud_cache_.emplace(idx, cloud);
  submap_cloud_cache_order_.push_back(idx);
  if (submap_cloud_cache_order_.size() > kSubmapCloudCacheMaxEntries) {
    const int oldest = submap_cloud_cache_order_.front();
    submap_cloud_cache_order_.pop_front();
    submap_cloud_cache_.erase(oldest);
  }
  return cloud;
}

/*
Summary:
Downsamples and writes the corrected map as grid-divided PCD files, a complete
map.pcd, grid metadata, and map projector metadata beneath map_save_dir.
*/
void GraphBasedSlamComponent::saveGridDividedMap(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & map)
{
  if (map->empty()) {
    std::cout << "Map is empty, skipping save." << std::endl;
    return;
  }

  // Create output directory (clean existing PCD files to prevent orphans)
  std::string out_dir = map_save_dir_ + "/pointcloud_map";
  if (std::filesystem::exists(out_dir)) {
    for (auto & entry : std::filesystem::directory_iterator(out_dir)) {
      if (entry.path().extension() == ".pcd" || entry.path().extension() == ".yaml") {
        std::filesystem::remove(entry.path());
      }
    }
  }
  std::filesystem::create_directories(out_dir);

  // Downsample the map
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(map);
  vg.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
  vg.filter(*downsampled);

  std::cout << "Map points: " << map->size() << " -> " << downsampled->size()
            << " (leaf=" << map_leaf_size_ << "m)" << std::endl;

  // Compute bounding box
  pcl::PointXYZI min_pt, max_pt;
  pcl::getMinMax3D(*downsampled, min_pt, max_pt);

  // Compute grid bounds (align to grid)
  double x_min = std::floor(min_pt.x / map_grid_size_x_) * map_grid_size_x_;
  double y_min = std::floor(min_pt.y / map_grid_size_y_) * map_grid_size_y_;
  double x_max = std::ceil(max_pt.x / map_grid_size_x_) * map_grid_size_x_;
  double y_max = std::ceil(max_pt.y / map_grid_size_y_) * map_grid_size_y_;

  int nx = static_cast<int>((x_max - x_min) / map_grid_size_x_);
  int ny = static_cast<int>((y_max - y_min) / map_grid_size_y_);
  if (nx <= 0) {nx = 1;}
  if (ny <= 0) {ny = 1;}

  // Assign points to grid cells
  std::map<std::pair<int, int>, pcl::PointCloud<pcl::PointXYZI>::Ptr> grid_cells;
  for (const auto & pt : downsampled->points) {
    int gx = static_cast<int>(std::floor((pt.x - x_min) / map_grid_size_x_));
    int gy = static_cast<int>(std::floor((pt.y - y_min) / map_grid_size_y_));
    auto key = std::make_pair(gx, gy);
    if (grid_cells.find(key) == grid_cells.end()) {
      grid_cells[key] = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    }
    grid_cells[key]->push_back(pt);
  }

  // Save each grid cell as PCD and build metadata
  // Format: Autoware pointcloud_map_loader expects:
  //   x_resolution: 20.0
  //   y_resolution: 20.0
  //   filename.pcd: [x, y]   (lower-left corner of grid cell)
  std::ofstream meta(out_dir + "/pointcloud_map_metadata.yaml");
  meta << std::fixed;
  meta << "x_resolution: " << std::setprecision(1) << map_grid_size_x_ << std::endl;
  meta << "y_resolution: " << std::setprecision(1) << map_grid_size_y_ << std::endl;

  int saved = 0;
  for (auto & [key, cloud] : grid_cells) {
    if (cloud->empty()) {continue;}
    double cell_x = x_min + key.first * map_grid_size_x_;
    double cell_y = y_min + key.second * map_grid_size_y_;

    std::ostringstream filename;
    filename << static_cast<int>(cell_x) << "_"
             << static_cast<int>(cell_y) << ".pcd";
    std::string filepath = out_dir + "/" + filename.str();
    pcl::io::savePCDFileBinaryCompressed(filepath, *cloud);

    meta << filename.str() << ": ["
         << static_cast<int>(cell_x) << ", "
         << static_cast<int>(cell_y) << "]" << std::endl;
    saved++;
  }

  meta.close();

  // Also save the full map as a single PCD for convenience
  pcl::io::savePCDFileBinaryCompressed(map_save_dir_ + "/map.pcd", *downsampled);

  std::cout << "Saved grid-divided map: " << saved << " cells ("
            << map_grid_size_x_ << "x" << map_grid_size_y_ << "m) to " << out_dir
            << std::endl;
  std::cout << "Total points: " << downsampled->size() << std::endl;
  std::cout << "Metadata: " << out_dir << "/pointcloud_map_metadata.yaml" << std::endl;

  // Always emit map_projector_info.yaml so Autoware can load pointcloud-only maps.
  std::string proj_file = map_save_dir_ + "/map_projector_info.yaml";
  std::ofstream proj(proj_file);
  proj << std::fixed << std::setprecision(10);
  if (gnss_origin_set_) {
    double origin_lat;
    double origin_lon;
    {
      std::lock_guard<std::mutex> gnss_lock(gnss_mtx_);
      origin_lat = gnss_origin_lat_;
      origin_lon = gnss_origin_lon_;
    }
    proj << "projector_type: LocalCartesian" << std::endl;
    proj << "vertical_datum: WGS84" << std::endl;
    proj << "map_origin:" << std::endl;
    proj << "  latitude: " << origin_lat << std::endl;
    proj << "  longitude: " << origin_lon << std::endl;
    std::cout << "Saved Autoware map projector info (LocalCartesian): " << proj_file
              << std::endl;
  } else {
    proj << "projector_type: Local" << std::endl;
    std::cout << "Saved Autoware map projector info (Local): " << proj_file << std::endl;
  }
  proj.close();
}
}  // namespace graphslam
