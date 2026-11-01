// Incremental DEM rasterisation for the graph backend (2026-11-01).
//
// The DEM used to be rebuilt from the whole corrected map on every pass (6 million points, 14 s per pass by the
// end of a lap). It is now an accumulator: every cell keeps the heights it has seen, a job adds only the points
// appended to the map since the previous job, the statistics are recomputed for the cells that received points,
// and the raster is laid out from the cells' absolute indices so it can grow in any direction. A rebuild of the
// assembled map (poses changed) starts the accumulator over. The results are, cell for cell, what the one-shot
// rasteriser produced; test_dem_raster.cpp checks that against an independent reference.
#ifndef GRAPH_BASED_SLAM__DEM_RASTER_HPP_
#define GRAPH_BASED_SLAM__DEM_RASTER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace graphslam
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

// One cell of the accumulator: the heights seen so far and the statistics derived from them.
struct DemCell
{
  std::vector<float> z;
  float terrain {0.0F};
  float surface {0.0F};
  float variance {0.0F};
  float min_z {0.0F};
  float max_z {0.0F};
  bool valid {false};
  bool dirty {true};
};

using DemCells = std::unordered_map<int64_t, DemCell>;

inline int64_t demCellKey(int32_t ix, int32_t iy)
{
  return (static_cast<int64_t>(ix) << 32) | static_cast<uint32_t>(iy);
}

inline int32_t demCellX(int64_t key) {return static_cast<int32_t>(key >> 32);}
inline int32_t demCellY(int64_t key) {return static_cast<int32_t>(key & 0xFFFFFFFFLL);}

// Adds the finite points of a cloud to their cells (absolute cell indices); returns the number added.
inline std::size_t accumulateDemPoints(
  DemCells & cells, const pcl::PointCloud<pcl::PointXYZI> & cloud, double resolution)
{
  std::size_t added = 0;
  for (const auto & p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const int32_t ix = static_cast<int32_t>(std::floor(p.x / resolution));
    const int32_t iy = static_cast<int32_t>(std::floor(p.y / resolution));
    auto & cell = cells[demCellKey(ix, iy)];
    cell.z.push_back(p.z);
    cell.dirty = true;
    ++added;
  }
  return added;
}

// Recomputes the statistics of the cells that received points since the last call; returns how many.
// terrain = the lower `percentile` of the cell's heights (index floor(p * (n - 1)) of the sorted heights),
// surface = the highest, plus min, max and the population variance; a cell with fewer than `min_points`
// heights is not valid.
inline std::size_t refreshDemCells(DemCells & cells, int min_points, double percentile)
{
  percentile = std::clamp(percentile, 0.0, 1.0);
  std::size_t recomputed = 0;
  for (auto & entry : cells) {
    auto & cell = entry.second;
    if (!cell.dirty) {
      continue;
    }
    cell.dirty = false;
    ++recomputed;
    auto & values = cell.z;
    cell.valid = static_cast<int>(values.size()) >= min_points;
    if (!cell.valid) {
      continue;
    }
    const auto minmax = std::minmax_element(values.begin(), values.end());
    cell.min_z = *minmax.first;
    cell.max_z = *minmax.second;
    cell.surface = cell.max_z;
    const std::size_t q = static_cast<std::size_t>(
      std::floor(percentile * static_cast<double>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(q), values.end());
    cell.terrain = values[q];
    const double mean = std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
    double sum_sq = 0.0;
    for (const float z : values) {
      const double d = static_cast<double>(z) - mean;
      sum_sq += d * d;
    }
    cell.variance = static_cast<float>(sum_sq / static_cast<double>(values.size()));
  }
  return recomputed;
}

// Lays the raster out over the cells' extent and fills it from their statistics. False (with `error`) when
// there are no cells or the raster would exceed `max_cells`.
inline bool rasterizeDemCells(
  const DemCells & cells, double resolution, int max_cells, DemGrid & grid, std::string & error)
{
  if (cells.empty() || resolution <= 0.0) {
    error = "no finite points or invalid resolution";
    return false;
  }
  int32_t min_ix = INT32_MAX, min_iy = INT32_MAX, max_ix = INT32_MIN, max_iy = INT32_MIN;
  for (const auto & entry : cells) {
    const int32_t ix = demCellX(entry.first);
    const int32_t iy = demCellY(entry.first);
    min_ix = std::min(min_ix, ix); max_ix = std::max(max_ix, ix);
    min_iy = std::min(min_iy, iy); max_iy = std::max(max_iy, iy);
  }
  grid.resolution = resolution;
  grid.origin_x = static_cast<double>(min_ix) * resolution;
  grid.origin_y = static_cast<double>(min_iy) * resolution;
  grid.width = max_ix - min_ix + 1;
  grid.height = max_iy - min_iy + 1;
  const int64_t cell_count = static_cast<int64_t>(grid.width) * grid.height;
  if (grid.width <= 0 || grid.height <= 0 || cell_count <= 0 || cell_count > max_cells) {
    std::ostringstream ss;
    ss << "DEM bounds require " << cell_count << " cells, maximum is " << max_cells;
    error = ss.str();
    return false;
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
  for (const auto & entry : cells) {
    const std::size_t idx = static_cast<std::size_t>(demCellY(entry.first) - min_iy) * grid.width +
      static_cast<std::size_t>(demCellX(entry.first) - min_ix);
    const auto & cell = entry.second;
    grid.count[idx] = static_cast<uint32_t>(cell.z.size());
    if (!cell.valid) {
      continue;
    }
    grid.terrain[idx] = cell.terrain;
    grid.interpolated_terrain[idx] = cell.terrain;
    grid.surface[idx] = cell.surface;
    grid.min_z[idx] = cell.min_z;
    grid.max_z[idx] = cell.max_z;
    grid.variance[idx] = cell.variance;
    grid.valid[idx] = 1U;
  }
  return true;
}

}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__DEM_RASTER_HPP_
