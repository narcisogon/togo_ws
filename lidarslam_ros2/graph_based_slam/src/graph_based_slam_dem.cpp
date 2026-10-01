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
Builds, repairs, persists, and previews a terrain digital elevation model from
immutable corrected-map snapshots. All rasterization and disk work runs on a
dedicated worker so navigation and graph processing remain responsive.
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
namespace
{
constexpr float kDemNoData = -9999.0F;

struct DemGrid
{
  double resolution {0.1};
  double origin_x {0.0};
  double origin_y {0.0};
  int width {0};
  int height {0};
  std::vector<float> terrain;
  std::vector<float> surface;
  std::vector<float> interpolated_terrain;
  std::vector<float> variance;
  std::vector<float> min_z;
  std::vector<float> max_z;
  std::vector<uint32_t> count;
  std::vector<uint8_t> valid;
  std::vector<uint8_t> interpolated;
};

/*
Summary:
Rasterizes a corrected point cloud into percentile-based terrain heights and
rejects invalid dimensions or grids above the configured cell limit.
*/
bool buildDemGrid(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  double resolution,
  int min_points,
  double percentile,
  int max_cells,
  DemGrid & grid,
  std::string & error)
{
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  std::size_t finite_points = 0;
  for (const auto & p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    min_x = std::min(min_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    max_x = std::max(max_x, static_cast<double>(p.x));
    max_y = std::max(max_y, static_cast<double>(p.y));
    ++finite_points;
  }
  if (finite_points == 0 || resolution <= 0.0) {
    error = "no finite points or invalid resolution";
    return false;
  }

  grid.resolution = resolution;
  grid.origin_x = std::floor(min_x / resolution) * resolution;
  grid.origin_y = std::floor(min_y / resolution) * resolution;
  grid.width = static_cast<int>(std::floor((max_x - grid.origin_x) / resolution)) + 1;
  grid.height = static_cast<int>(std::floor((max_y - grid.origin_y) / resolution)) + 1;
  const int64_t cell_count = static_cast<int64_t>(grid.width) * grid.height;
  if (grid.width <= 0 || grid.height <= 0 || cell_count <= 0 || cell_count > max_cells) {
    std::ostringstream ss;
    ss << "DEM bounds require " << cell_count << " cells, maximum is " << max_cells;
    error = ss.str();
    return false;
  }

  std::unordered_map<std::size_t, std::vector<float>> z_values;
  z_values.reserve(std::min<std::size_t>(finite_points, static_cast<std::size_t>(cell_count)));
  for (const auto & p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const int x = static_cast<int>(std::floor((p.x - grid.origin_x) / resolution));
    const int y = static_cast<int>(std::floor((p.y - grid.origin_y) / resolution));
    if (x < 0 || y < 0 || x >= grid.width || y >= grid.height) {
      continue;
    }
    z_values[static_cast<std::size_t>(y) * grid.width + x].push_back(p.z);
  }

  const std::size_t n = static_cast<std::size_t>(cell_count);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  grid.terrain.assign(n, nan);
  grid.surface.assign(n, nan);
  grid.interpolated_terrain.assign(n, nan);
  grid.variance.assign(n, nan);
  grid.min_z.assign(n, nan);
  grid.max_z.assign(n, nan);
  grid.count.assign(n, 0U);
  grid.valid.assign(n, 0U);
  grid.interpolated.assign(n, 0U);
  percentile = std::clamp(percentile, 0.0, 1.0);
  for (auto & entry : z_values) {
    auto & values = entry.second;
    const std::size_t idx = entry.first;
    grid.count[idx] = static_cast<uint32_t>(values.size());
    if (static_cast<int>(values.size()) < min_points) {
      continue;
    }
    const auto minmax = std::minmax_element(values.begin(), values.end());
    const float cell_min = *minmax.first;
    const float cell_max = *minmax.second;
    const std::size_t q = static_cast<std::size_t>(
      std::floor(percentile * static_cast<double>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + q, values.end());
    const float terrain = values[q];
    const double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    double sum_sq = 0.0;
    for (const float z : values) {
      const double d = static_cast<double>(z) - mean;
      sum_sq += d * d;
    }
    grid.terrain[idx] = terrain;
    grid.interpolated_terrain[idx] = terrain;
    grid.surface[idx] = cell_max;
    grid.min_z[idx] = cell_min;
    grid.max_z[idx] = cell_max;
    grid.variance[idx] = static_cast<float>(sum_sq / values.size());
    grid.valid[idx] = 1U;
  }
  return true;
}

/*
Summary:
Fills small enclosed invalid-cell components when neighboring terrain supports
a sufficiently consistent local plane estimate.
*/
void interpolateSmallDemHoles(
  DemGrid & grid,
  int max_component_cells,
  int search_radius,
  int min_neighbors,
  double max_height_range,
  double max_plane_residual)
{
  if (max_component_cells <= 0 || grid.width <= 2 || grid.height <= 2) {
    return;
  }
  const std::size_t total = grid.valid.size();
  std::vector<uint8_t> visited(total, 0U);
  const int dx4[4] = {1, -1, 0, 0};
  const int dy4[4] = {0, 0, 1, -1};
  for (int sy = 0; sy < grid.height; ++sy) {
    for (int sx = 0; sx < grid.width; ++sx) {
      const std::size_t start = static_cast<std::size_t>(sy) * grid.width + sx;
      if (grid.valid[start] || visited[start]) {
        continue;
      }
      std::queue<std::pair<int, int>> queue;
      std::vector<std::pair<int, int>> component;
      bool touches_boundary = false;
      visited[start] = 1U;
      queue.emplace(sx, sy);
      while (!queue.empty()) {
        const auto cell = queue.front();
        queue.pop();
        component.push_back(cell);
        touches_boundary = touches_boundary || cell.first == 0 || cell.second == 0 ||
          cell.first == grid.width - 1 || cell.second == grid.height - 1;
        for (int k = 0; k < 4; ++k) {
          const int nx = cell.first + dx4[k];
          const int ny = cell.second + dy4[k];
          if (nx < 0 || ny < 0 || nx >= grid.width || ny >= grid.height) {
            continue;
          }
          const std::size_t ni = static_cast<std::size_t>(ny) * grid.width + nx;
          if (!grid.valid[ni] && !visited[ni]) {
            visited[ni] = 1U;
            queue.emplace(nx, ny);
          }
        }
      }
      if (touches_boundary || static_cast<int>(component.size()) > max_component_cells) {
        continue;
      }

      std::set<std::size_t> neighbor_indices;
      for (const auto & cell : component) {
        for (int dy = -search_radius; dy <= search_radius; ++dy) {
          for (int dx = -search_radius; dx <= search_radius; ++dx) {
            const int nx = cell.first + dx;
            const int ny = cell.second + dy;
            if (nx < 0 || ny < 0 || nx >= grid.width || ny >= grid.height) {
              continue;
            }
            const std::size_t ni = static_cast<std::size_t>(ny) * grid.width + nx;
            if (grid.valid[ni]) {
              neighbor_indices.insert(ni);
            }
          }
        }
      }
      if (static_cast<int>(neighbor_indices.size()) < min_neighbors) {
        continue;
      }

      Eigen::Matrix3d ata = Eigen::Matrix3d::Zero();
      Eigen::Vector3d atz = Eigen::Vector3d::Zero();
      float min_height = std::numeric_limits<float>::infinity();
      float max_height = -std::numeric_limits<float>::infinity();
      for (const std::size_t ni : neighbor_indices) {
        const int x = static_cast<int>(ni % grid.width);
        const int y = static_cast<int>(ni / grid.width);
        const double px = grid.origin_x + (x + 0.5) * grid.resolution;
        const double py = grid.origin_y + (y + 0.5) * grid.resolution;
        const double z = grid.terrain[ni];
        const Eigen::Vector3d a(px, py, 1.0);
        ata += a * a.transpose();
        atz += a * z;
        min_height = std::min(min_height, static_cast<float>(z));
        max_height = std::max(max_height, static_cast<float>(z));
      }
      if (max_height - min_height > max_height_range ||
          std::abs(ata.determinant()) < 1.0e-12) {
        continue;
      }

      // Prefer bilinear interpolation when the entire hole is enclosed by a
      // valid rectangular frame. Validate the interpolant against all nearby
      // measured terrain cells so that a four-corner match cannot bridge a
      // rough or discontinuous surface. Irregular holes and rejected bilinear
      // fits fall through to the existing least-squares plane interpolation.
      int hole_min_x = grid.width;
      int hole_min_y = grid.height;
      int hole_max_x = -1;
      int hole_max_y = -1;
      for (const auto & cell : component) {
        hole_min_x = std::min(hole_min_x, cell.first);
        hole_min_y = std::min(hole_min_y, cell.second);
        hole_max_x = std::max(hole_max_x, cell.first);
        hole_max_y = std::max(hole_max_y, cell.second);
      }
      const int left = hole_min_x - 1;
      const int right = hole_max_x + 1;
      const int bottom = hole_min_y - 1;
      const int top = hole_max_y + 1;
      const auto cell_index = [&grid](int x, int y) {
        return static_cast<std::size_t>(y) * grid.width + x;
      };
      const bool corners_in_bounds =
        left >= 0 && bottom >= 0 && right < grid.width && top < grid.height;
      const bool corners_valid = corners_in_bounds &&
        grid.valid[cell_index(left, bottom)] &&
        grid.valid[cell_index(right, bottom)] &&
        grid.valid[cell_index(left, top)] &&
        grid.valid[cell_index(right, top)];
      if (corners_valid) {
        const double z00 = grid.terrain[cell_index(left, bottom)];
        const double z10 = grid.terrain[cell_index(right, bottom)];
        const double z01 = grid.terrain[cell_index(left, top)];
        const double z11 = grid.terrain[cell_index(right, top)];
        const auto bilinear_height =
          [left, right, bottom, top, z00, z10, z01, z11](double x, double y) {
            const double u = (x - left) / static_cast<double>(right - left);
            const double v = (y - bottom) / static_cast<double>(top - bottom);
            return (1.0 - u) * (1.0 - v) * z00 +
                   u * (1.0 - v) * z10 +
                   (1.0 - u) * v * z01 +
                   u * v * z11;
          };
        double bilinear_squared_error = 0.0;
        for (const std::size_t ni : neighbor_indices) {
          const int x = static_cast<int>(ni % grid.width);
          const int y = static_cast<int>(ni / grid.width);
          const double residual = grid.terrain[ni] - bilinear_height(x, y);
          bilinear_squared_error += residual * residual;
        }
        const double bilinear_rms =
          std::sqrt(bilinear_squared_error / neighbor_indices.size());
        if (std::isfinite(bilinear_rms) && bilinear_rms <= max_plane_residual) {
          for (const auto & cell : component) {
            const std::size_t idx = cell_index(cell.first, cell.second);
            grid.interpolated_terrain[idx] = static_cast<float>(
              bilinear_height(cell.first, cell.second));
            grid.interpolated[idx] = 1U;
          }
          continue;
        }
      }

      const Eigen::Vector3d plane = ata.ldlt().solve(atz);
      double squared_error = 0.0;
      for (const std::size_t ni : neighbor_indices) {
        const int x = static_cast<int>(ni % grid.width);
        const int y = static_cast<int>(ni / grid.width);
        const double px = grid.origin_x + (x + 0.5) * grid.resolution;
        const double py = grid.origin_y + (y + 0.5) * grid.resolution;
        const double residual = grid.terrain[ni] - (plane.x() * px + plane.y() * py + plane.z());
        squared_error += residual * residual;
      }
      const double rms = std::sqrt(squared_error / neighbor_indices.size());
      if (!std::isfinite(rms) || rms > max_plane_residual) {
        continue;
      }
      for (const auto & cell : component) {
        const std::size_t idx = static_cast<std::size_t>(cell.second) * grid.width + cell.first;
        const double px = grid.origin_x + (cell.first + 0.5) * grid.resolution;
        const double py = grid.origin_y + (cell.second + 0.5) * grid.resolution;
        grid.interpolated_terrain[idx] = static_cast<float>(
          plane.x() * px + plane.y() * py + plane.z());
        grid.interpolated[idx] = 1U;
      }
    }
  }
}

template<typename ValueFn>
/*
Summary:
Writes an ESRI-style ASCII elevation grid through a temporary file and atomically
replaces the completed destination.
*/
bool writeAsciiGridAtomic(
  const std::filesystem::path & path,
  const DemGrid & grid,
  ValueFn value_fn)
{
  const auto temp = path.string() + ".tmp";
  std::ofstream out(temp, std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    return false;
  }
  out << "ncols " << grid.width << '\n'
      << "nrows " << grid.height << '\n'
      << std::setprecision(12)
      << "xllcorner " << grid.origin_x << '\n'
      << "yllcorner " << grid.origin_y << '\n'
      << "cellsize " << grid.resolution << '\n'
      << "NODATA_value " << kDemNoData << '\n';
  out << std::setprecision(7);
  for (int y = grid.height - 1; y >= 0; --y) {
    for (int x = 0; x < grid.width; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * grid.width + x;
      const float value = value_fn(idx);
      out << (std::isfinite(value) ? value : kDemNoData);
      if (x + 1 < grid.width) {
        out << ' ';
      }
    }
    out << '\n';
  }
  out.close();
  if (!out) {
    return false;
  }
  std::error_code ec;
  std::filesystem::remove(path, ec);
  ec.clear();
  std::filesystem::rename(temp, path, ec);
  return !ec;
}
}  // namespace

/*
Summary:
Publishes a structured DEM state and detail message for operational monitoring.
*/
void GraphBasedSlamComponent::publishDemStatus(
  const std::string & state, const std::string & detail)
{
  if (!dem_status_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  std::ostringstream ss;
  ss << "{\"state\":\"" << state << "\",\"detail\":\"" << detail
     << "\",\"last_completed_revision\":" << dem_last_completed_revision_.load() << "}";
  msg.data = ss.str();
  dem_status_pub_->publish(msg);
}

/*
Summary:
Queues the newest corrected-map snapshot when the DEM period has elapsed or a
forced save requires an immediate update.
*/
void GraphBasedSlamComponent::scheduleDemJob(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud,
  const rclcpp::Time & stamp,
  bool force)
{
  if (!dem_enabled_ || !cloud || cloud->empty()) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (!force && dem_last_scheduled_time_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - dem_last_scheduled_time_).count() <
      dem_update_period_sec_) {
    return;
  }
  dem_last_scheduled_time_ = now;
  auto job = std::make_unique<DemJob>();
  // Aggregate map storage is persistent and may be extended by a later
  // append-only publish while this worker is rasterizing. Give the DEM an
  // immutable snapshot so the background read cannot race that append.
  job->cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(*cloud));
  job->revision = dem_revision_counter_.fetch_add(1) + 1;
  job->stamp = stamp;
  {
    std::lock_guard<std::mutex> lock(dem_worker_mtx_);
    // Queue depth one: a fresher corrected-map snapshot supersedes an older
    // pending job. The active job, if any, is allowed to finish safely.
    pending_dem_job_ = std::move(job);
  }
  dem_worker_cv_.notify_one();
}

/*
Summary:
Waits for pending DEM snapshots and processes only the newest queued job until
component shutdown.
*/
void GraphBasedSlamComponent::demWorkerLoop()
{
  while (!shutting_down_) {
    std::unique_ptr<DemJob> job;
    {
      std::unique_lock<std::mutex> lock(dem_worker_mtx_);
      dem_worker_cv_.wait(lock, [this]() {
        return shutting_down_.load() || static_cast<bool>(pending_dem_job_);
      });
      if (shutting_down_) {
        break;
      }
      job = std::move(pending_dem_job_);
    }
    if (!job) {
      continue;
    }
    try {
      processDemJob(*job);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "DEM worker revision %llu failed: %s",
        static_cast<unsigned long long>(job->revision), e.what());
      publishDemStatus("error", e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "DEM worker revision %llu failed with unknown exception",
        static_cast<unsigned long long>(job->revision));
      publishDemStatus("error", "unknown exception");
    }
  }
}

/*
Summary:
Generates full-resolution and preview DEM products, writes the snapshot, and
publishes status, timing, and optional preview data.
*/
void GraphBasedSlamComponent::processDemJob(const DemJob & job)
{
  const auto started = std::chrono::steady_clock::now();
  publishDemStatus("busy", "rasterizing corrected map");
  DemGrid grid;
  std::string error;
  if (!buildDemGrid(
      *job.cloud, dem_storage_resolution_m_, dem_min_points_per_cell_,
      dem_terrain_percentile_, dem_max_raster_cells_, grid, error)) {
    throw std::runtime_error(error);
  }
  interpolateSmallDemHoles(
    grid, dem_interpolation_max_component_cells_, dem_interpolation_search_radius_cells_,
    dem_interpolation_min_valid_neighbors_, dem_interpolation_max_neighbor_height_range_m_,
    dem_interpolation_max_plane_residual_m_);

  const std::filesystem::path output_dir(dem_output_dir_);
  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  if (ec) {
    throw std::runtime_error("failed to create DEM output directory: " + ec.message());
  }

  bool ok = true;
  ok = ok && writeAsciiGridAtomic(
    output_dir / "measured_terrain.asc", grid,
    [&grid](std::size_t i) {return grid.valid[i] ? grid.terrain[i] : kDemNoData;});
  ok = ok && writeAsciiGridAtomic(
    output_dir / "measured_surface.asc", grid,
    [&grid](std::size_t i) {return grid.valid[i] ? grid.surface[i] : kDemNoData;});
  ok = ok && writeAsciiGridAtomic(
    output_dir / "interpolated_terrain.asc", grid,
    [&grid](std::size_t i) {return grid.interpolated_terrain[i];});
  ok = ok && writeAsciiGridAtomic(
    output_dir / "elevation_variance.asc", grid,
    [&grid](std::size_t i) {return grid.valid[i] ? grid.variance[i] : kDemNoData;});
  ok = ok && writeAsciiGridAtomic(
    output_dir / "observation_count.asc", grid,
    [&grid](std::size_t i) {return static_cast<float>(grid.count[i]);});
  ok = ok && writeAsciiGridAtomic(
    output_dir / "valid_mask.asc", grid,
    [&grid](std::size_t i) {return grid.valid[i] ? 1.0F : 0.0F;});
  ok = ok && writeAsciiGridAtomic(
    output_dir / "interpolated_mask.asc", grid,
    [&grid](std::size_t i) {return grid.interpolated[i] ? 1.0F : 0.0F;});
  ok = ok && writeAsciiGridAtomic(
    output_dir / "minimum_elevation.asc", grid,
    [&grid](std::size_t i) {return grid.valid[i] ? grid.min_z[i] : kDemNoData;});
  ok = ok && writeAsciiGridAtomic(
    output_dir / "maximum_elevation.asc", grid,
    [&grid](std::size_t i) {return grid.valid[i] ? grid.max_z[i] : kDemNoData;});
  if (!ok) {
    throw std::runtime_error("failed to atomically write one or more DEM raster layers");
  }

  const auto metadata_temp = output_dir / "metadata.yaml.tmp";
  const auto metadata_path = output_dir / "metadata.yaml";
  {
    std::ofstream meta(metadata_temp, std::ios::out | std::ios::trunc);
    if (!meta.is_open()) {
      throw std::runtime_error("failed to open DEM metadata temporary file");
    }
    meta << "schema_version: 1\n"
         << "map_state: preliminary_corrected\n"
         << "graph_revision: " << job.revision << "\n"
         << "frame_id: " << global_frame_id_ << "\n"
         << "coordinate_type: local_cartesian\n"
         << "horizontal_units: meters\n"
         << "vertical_units: meters\n"
         << "axis_convention: {x: map_positive_x, y: map_positive_y, z: up}\n"
         << "resolution_m: " << std::setprecision(12) << grid.resolution << "\n"
         << "width: " << grid.width << "\n"
         << "height: " << grid.height << "\n"
         << "origin_x_m: " << grid.origin_x << "\n"
         << "origin_y_m: " << grid.origin_y << "\n"
         << "pixel_registration: cell_center\n"
         << "row_order: north_to_south\n"
         << "nodata: " << kDemNoData << "\n"
         << "terrain_estimator: lower_percentile\n"
         << "terrain_percentile: " << dem_terrain_percentile_ << "\n"
         << "min_points_per_cell: " << dem_min_points_per_cell_ << "\n"
         << "source_point_count: " << job.cloud->size() << "\n"
         << "source_stamp_sec: " << std::setprecision(15) << job.stamp.seconds() << "\n"
         << "global_anchor: {available: false}\n";
  }
  std::filesystem::remove(metadata_path, ec);
  ec.clear();
  std::filesystem::rename(metadata_temp, metadata_path, ec);
  if (ec) {
    throw std::runtime_error("failed to publish DEM metadata: " + ec.message());
  }

  std::size_t valid_cells = 0;
  std::size_t interpolated_cells = 0;
  for (std::size_t i = 0; i < grid.valid.size(); ++i) {
    valid_cells += grid.valid[i] ? 1U : 0U;
    interpolated_cells += grid.interpolated[i] ? 1U : 0U;
  }

  if (dem_publish_preview_ && dem_preview_pub_ && valid_cells > 0) {
    const int factor = std::max(
      1, static_cast<int>(std::lround(dem_preview_resolution_m_ / grid.resolution)));
    struct PreviewAccumulator {double x {0}; double y {0}; double z {0}; double confidence {0}; int n {0};};
    std::unordered_map<int64_t, PreviewAccumulator> preview_cells;
    preview_cells.reserve(valid_cells / static_cast<std::size_t>(factor * factor) + 1);
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const std::size_t idx = static_cast<std::size_t>(y) * grid.width + x;
        if (!grid.valid[idx]) {
          continue;
        }
        const int px = x / factor;
        const int py = y / factor;
        const int64_t key = (static_cast<int64_t>(py) << 32) | static_cast<uint32_t>(px);
        auto & acc = preview_cells[key];
        acc.x += grid.origin_x + (x + 0.5) * grid.resolution;
        acc.y += grid.origin_y + (y + 0.5) * grid.resolution;
        acc.z += grid.terrain[idx];
        acc.confidence += std::min(1.0, static_cast<double>(grid.count[idx]) /
          std::max(1, dem_min_points_per_cell_ * 3));
        ++acc.n;
      }
    }
    pcl::PointCloud<pcl::PointXYZI> preview;
    const std::size_t stride = std::max<std::size_t>(
      1, (preview_cells.size() + static_cast<std::size_t>(dem_preview_max_cells_) - 1) /
      static_cast<std::size_t>(dem_preview_max_cells_));
    preview.reserve(std::min<std::size_t>(preview_cells.size(), dem_preview_max_cells_));
    std::size_t ordinal = 0;
    for (const auto & entry : preview_cells) {
      if (ordinal++ % stride != 0 || entry.second.n <= 0) {
        continue;
      }
      pcl::PointXYZI point;
      point.x = static_cast<float>(entry.second.x / entry.second.n);
      point.y = static_cast<float>(entry.second.y / entry.second.n);
      point.z = static_cast<float>(entry.second.z / entry.second.n);
      point.intensity = static_cast<float>(entry.second.confidence / entry.second.n);
      preview.push_back(point);
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(preview, msg);
    msg.header.frame_id = global_frame_id_;
    msg.header.stamp = job.stamp;
    dem_preview_pub_->publish(msg);
  }

  dem_last_completed_revision_ = job.revision;
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  std::ostringstream detail;
  detail << "revision=" << job.revision << " points=" << job.cloud->size()
         << " valid_cells=" << valid_cells << " interpolated_cells=" << interpolated_cells
         << " raster=" << grid.width << "x" << grid.height
         << " elapsed_ms=" << std::fixed << std::setprecision(1) << elapsed_ms;
  RCLCPP_INFO(get_logger(), "DEM snapshot complete: %s", detail.str().c_str());
  publishDemStatus("idle", detail.str());
}

}  // namespace graphslam
