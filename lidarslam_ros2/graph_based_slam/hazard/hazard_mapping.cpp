#include "hazard_mapping.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace graphslam::hazard
{
void Config::validate() const
{
  const double values[] = {resolution, min_height, max_height, max_range,
    slope_radius, max_slope_deg, lethal_radius, soft_radius, footprint_radius,
    terrain_percentile, height_noise, min_plane_coverage, max_plane_error, clear_slope_margin,
    obstacle_min_height, obstacle_max_height, obstacle_voxel_height, obstacle_hit_probability,
    obstacle_miss_probability, obstacle_mark_probability, obstacle_clear_probability,
    obstacle_max_probability};
  for (double value : values) {
    if (!std::isfinite(value)) {throw std::invalid_argument("hazard values must be finite");}
  }
  if (resolution < 0.01 || max_range <= 0.0 || min_height >= max_height ||
    slope_radius < resolution || max_slope_deg <= 0.0 || max_slope_deg >= 90.0 ||
    lethal_radius < 0.0 || soft_radius <= lethal_radius || footprint_radius < 0.0 ||
    unknown_cost < -1 || unknown_cost > 100 ||
    min_points < 1 || min_neighbors < 3 || terrain_percentile < 0.0 ||
    terrain_percentile > 1.0 || max_cells < 100 || height_noise <= 0 ||
    min_plane_coverage <= 0 || min_plane_coverage > 0.5 || max_plane_error <= 0 ||
    clear_confirmations < 1 || clear_confirmations > 32 || clear_slope_margin < 0 ||
    clear_slope_margin >= max_slope_deg || obstacle_min_height <= 0 ||
    obstacle_max_height <= obstacle_min_height || obstacle_voxel_height < 0.05 ||
    obstacle_voxel_height > 1.0 || obstacle_min_points < 1 || obstacle_min_points > 128 ||
    obstacle_clear_probability <= 0 || obstacle_clear_probability >= 0.5 ||
    obstacle_mark_probability <= 0.5 || obstacle_mark_probability >= obstacle_max_probability ||
    obstacle_max_probability >= 1 || obstacle_hit_probability <= 0.5 ||
    obstacle_hit_probability >= 1 || obstacle_miss_probability <= 0 ||
    obstacle_miss_probability >= 0.5 || max_rays < 1 || max_rays > 20000 ||
    max_obstacle_voxels < 1 || max_obstacle_voxels > 4000000)
  {
    throw std::invalid_argument("invalid hazard configuration: require valid height/range, "
      "0 < slope < 90, 0 <= lethal radius < soft radius, and positive support/budget");
  }
  if (slope_radius / resolution > 100.0 || max_range / resolution > 10000.0 ||
    std::max(std::abs(min_height),std::abs(max_height)) /
    std::min(resolution,obstacle_voxel_height)>10000.0 ||
    (footprint_radius + soft_radius + slope_radius) / resolution > 10000.0)
  {
    throw std::invalid_argument("hazard range/neighborhood exceeds bounded work limits");
  }
}

bool Config::sameTerrain(const Config & other) const
{
  return resolution == other.resolution && min_height == other.min_height &&
         max_height == other.max_height && max_range == other.max_range &&
         terrain_percentile == other.terrain_percentile && min_points == other.min_points &&
         height_noise == other.height_noise && obstacle_min_height == other.obstacle_min_height &&
         obstacle_max_height == other.obstacle_max_height &&
         obstacle_voxel_height == other.obstacle_voxel_height;
}

std::size_t KeyHash::operator()(const Key & key) const
{
  const auto packed = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.x)) << 32) |
    static_cast<std::uint32_t>(key.y);
  return std::hash<std::uint64_t>{}(packed);
}

namespace
{
struct Samples
{
  std::vector<double> heights;
  Eigen::Vector3d up {Eigen::Vector3d::Zero()};
  double range_sum {0};
};
using SampleMap = std::unordered_map<Key, Samples, KeyHash>;
void appendSamples(SampleMap & samples,
  const std::vector<Eigen::Vector3d> & points,
  const Eigen::Isometry3d & pose, const Eigen::Vector3d & up_in_body,
  const Config & config)
{
  if (!pose.matrix().allFinite() || !up_in_body.allFinite() || up_in_body.norm() < 0.5) {
    throw std::invalid_argument("hazard pose/gravity direction is invalid");
  }
  const Eigen::Vector3d acquisition_up = up_in_body.normalized();
  const Eigen::Vector3d corrected_up = (pose.linear() * acquisition_up).normalized();
  for (const auto & point : points) {
    if (!point.allFinite()) {continue;}
    const double relative_height = point.dot(acquisition_up);
    const double horizontal_sq = std::max(0.0, point.squaredNorm() -
      relative_height * relative_height);
    if (relative_height < config.min_height || relative_height > config.max_height ||
      horizontal_sq > config.max_range * config.max_range)
    {continue;}
    const Eigen::Vector3d world = pose * point;
    const double gx = std::floor(world.x() / config.resolution);
    const double gy = std::floor(world.y() / config.resolution);
    if (std::abs(gx) > 100000000.0 || std::abs(gy) > 100000000.0) {
      throw std::runtime_error("hazard coordinates exceed map lattice limits");
    }
    auto & cell = samples[{static_cast<int>(gx), static_cast<int>(gy)}];
    cell.heights.push_back(world.z());
    cell.up += corrected_up;
    cell.range_sum += std::sqrt(horizontal_sq);
  }
}

Contribution terrainFromSamples(SampleMap & samples, const Config & config)
{
  Contribution result;
  result.reserve(samples.size());
  for (auto & [key, cell] : samples) {
    auto & heights = cell.heights;
    if (heights.size() < static_cast<std::size_t>(config.min_points)) {continue;}
    // A sparse lower percentile is just the minimum; interpolate its median
    // instead. Retain the ground percentile when there is enough support.
    const double percentile = heights.size() < 5 ? 0.5 : config.terrain_percentile;
    const double rank = percentile * (heights.size() - 1);
    const auto lower = static_cast<std::size_t>(std::floor(rank));
    const auto upper = static_cast<std::size_t>(std::ceil(rank));
    std::nth_element(heights.begin(), heights.begin() + lower, heights.end());
    const double low = heights[lower];
    std::nth_element(heights.begin(), heights.begin() + upper, heights.end());
    const double height = low + (rank - lower) * (heights[upper] - low);
    const double range = cell.range_sum / heights.size();
    const double sigma = config.height_noise * (1.0 + 2.0 * range / config.max_range);
    double scatter = 0;
    for (const double z : heights) {scatter += std::min((z-height)*(z-height), 0.01);}
    // Cap density benefit: repeated/correlated lidar returns are not unlimited
    // independent evidence. This is an uncertainty proxy, not sensor calibration.
    const double variance = (sigma*sigma + scatter / heights.size()) /
      std::min<std::size_t>(heights.size(), 8);
    result.emplace(key, Cell{height, cell.up.normalized(), heights.size(), variance});
  }
  return result;
}
}  // namespace

Contribution project(
  const std::vector<Eigen::Vector3d> & points, const Eigen::Isometry3d & pose,
  const Eigen::Vector3d & up_in_body, const Config & config)
{
  SampleMap samples;
  appendSamples(samples, points, pose, up_in_body, config);
  return terrainFromSamples(samples, config);
}

Contribution projectBatch(const std::vector<CloudObservation> & clouds, const Config & config)
{
  SampleMap samples;
  for (const auto & cloud : clouds) {
    appendSamples(samples, cloud.body_points, cloud.pose, cloud.up_in_body, config);
  }
  return terrainFromSamples(samples, config);
}

Grid rasterize(const std::vector<const Contribution *> & sources, const Config & config)
{
  config.validate();
  struct Aggregate {double z {0.0}; Eigen::Vector3d up {Eigen::Vector3d::Zero()}; int count {0};};
  std::unordered_map<Key, Aggregate, KeyHash> cells;
  for (const auto * source : sources) {
    for (const auto & [key, cell] : *source) {
      auto & combined = cells[key];
      // Each observation contributes one terrain estimate, independent of
      // LiDAR point density. This is height fusion, not hazard voting.
      combined.z += cell.height;
      combined.up += cell.up;
      ++combined.count;
    }
  }
  Grid grid;
  grid.resolution = config.resolution;
  int min_x = 0, max_x = 0, min_y = 0, max_y = 0;
  bool first = true;
  for (auto & [key, cell] : cells) {
    cell.z /= cell.count;
    cell.up.normalize();
    if (first) {min_x = max_x = key.x; min_y = max_y = key.y; first = false;}
    min_x = std::min(min_x, key.x); max_x = std::max(max_x, key.x);
    min_y = std::min(min_y, key.y); max_y = std::max(max_y, key.y);
  }
  const int padding = static_cast<int>(std::ceil(
    (config.footprint_radius + config.soft_radius + config.slope_radius) / config.resolution)) + 1;
  grid.origin_x = min_x - padding;
  grid.origin_y = min_y - padding;
  grid.width = static_cast<std::uint32_t>(max_x - min_x + 1 + 2 * padding);
  grid.height = static_cast<std::uint32_t>(max_y - min_y + 1 + 2 * padding);
  const auto size = static_cast<std::size_t>(grid.width) * grid.height;
  if (size > config.max_cells) {throw std::runtime_error("hazard grid exceeds max_cells");}
  grid.raw.assign(size, -1);
  grid.slope_deg.assign(size, std::numeric_limits<float>::quiet_NaN());
  const int radius_cells = static_cast<int>(std::ceil(config.slope_radius / config.resolution));
  struct Neighbor {Eigen::Vector3d row; double z;};
  std::vector<Neighbor> neighbors;
  for (const auto & [key, cell] : cells) {
    neighbors.clear();
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const double x = dx * config.resolution, y = dy * config.resolution;
        if (x * x + y * y > config.slope_radius * config.slope_radius) {continue;}
        const auto it = cells.find({key.x + dx, key.y + dy});
        if (it != cells.end()) {
          neighbors.push_back({Eigen::Vector3d(x, y, 1.0), it->second.z - cell.z});
        }
      }
    }
    if (neighbors.size() < static_cast<std::size_t>(config.min_neighbors)) {continue;}
    Eigen::Vector3d coefficients = Eigen::Vector3d::Zero();
    bool valid = true;
    // Three robust least-squares passes; outliers are downweighted, not a
    // second hazard rule. A collinear neighborhood cannot establish slope.
    for (int pass = 0; pass < 3; ++pass) {
      Eigen::Matrix3d lhs = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      for (const auto & neighbor : neighbors) {
        const double residual = std::abs(neighbor.z - neighbor.row.dot(coefficients));
        const double weight = pass == 0 ? 1.0 : std::min(1.0, 0.05 / std::max(1e-9, residual));
        lhs.noalias() += weight * neighbor.row * neighbor.row.transpose();
        rhs.noalias() += weight * neighbor.row * neighbor.z;
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> conditioning(lhs, Eigen::EigenvaluesOnly);
      if (conditioning.info() != Eigen::Success || conditioning.eigenvalues().minCoeff() < 1e-6) {
        valid = false; break;
      }
      coefficients = lhs.ldlt().solve(rhs);
      if (!coefficients.allFinite()) {valid = false; break;}
    }
    if (!valid) {continue;}
    const Eigen::Vector3d normal = Eigen::Vector3d(-coefficients.x(), -coefficients.y(), 1.0).normalized();
    const double slope = std::acos(std::clamp(std::abs(normal.dot(cell.up)), 0.0, 1.0)) *
      180.0 / 3.14159265358979323846;
    const auto idx = static_cast<std::size_t>(key.y - grid.origin_y) * grid.width +
      static_cast<std::size_t>(key.x - grid.origin_x);
    grid.slope_deg[idx] = static_cast<float>(slope);
    grid.raw[idx] = slope >= config.max_slope_deg ? 100 : 0;
  }
  inflate(grid, config);
  return grid;
}

int inflationCost(double distance, const Config & config)
{
  const double clearance = distance - config.footprint_radius;
  if (clearance <= config.lethal_radius) {return 100;}
  if (clearance >= config.soft_radius || !std::isfinite(distance)) {return 0;}
  return std::clamp(static_cast<int>(std::lround(99.0 *
      (config.soft_radius - clearance) / (config.soft_radius - config.lethal_radius))), 0, 99);
}

namespace
{
// Exact squared Euclidean distance transform (lower envelope of parabolas).
// Linear work per row/column; avoids hazard_count * inflation_area loops.
void distanceTransform(const std::vector<double> & f, std::vector<double> & d)
{
  const int size = static_cast<int>(f.size());
  std::vector<int> vertices(size);
  std::vector<double> boundaries(size + 1);
  int last = 0;
  vertices[0] = 0;
  boundaries[0] = -std::numeric_limits<double>::infinity();
  boundaries[1] = std::numeric_limits<double>::infinity();
  for (int q = 1; q < size; ++q) {
    double split;
    do {
      const int v = vertices[last];
      split = ((f[q] + static_cast<double>(q) * q) -
        (f[v] + static_cast<double>(v) * v)) / (2.0 * (q - v));
      if (split > boundaries[last]) {break;}
      --last;
    } while (last >= 0);
    ++last;
    vertices[last] = q;
    boundaries[last] = split;
    boundaries[last + 1] = std::numeric_limits<double>::infinity();
  }
  int envelope = 0;
  d.resize(size);
  for (int q = 0; q < size; ++q) {
    while (boundaries[envelope + 1] < q) {++envelope;}
    const double delta = q - vertices[envelope];
    d[q] = delta * delta + f[vertices[envelope]];
  }
}
}  // namespace

void inflate(Grid & grid, const Config & config)
{
  grid.costs = grid.raw;
  // Keep evidence unknown in raw, while independently pricing it for planning.
  // Unknown-cost 100 is not a measured hazard and must not seed inflation.
  if (config.unknown_cost >= 0) {
    for (auto & cost : grid.costs) {
      if (cost < 0) {cost = static_cast<std::int8_t>(config.unknown_cost);}
    }
  }
  if (std::find(grid.raw.begin(), grid.raw.end(), 100) == grid.raw.end()) {return;}
  const double far = 1e12;
  std::vector<double> distances(grid.raw.size(), far), input, output;
  input.resize(grid.width);
  for (std::uint32_t y = 0; y < grid.height; ++y) {
    for (std::uint32_t x = 0; x < grid.width; ++x) {
      input[x] = grid.raw[static_cast<std::size_t>(y) * grid.width + x] == 100 ? 0.0 : far;
    }
    distanceTransform(input, output);
    std::copy(output.begin(), output.end(), distances.begin() + static_cast<std::size_t>(y) * grid.width);
  }
  input.resize(grid.height);
  for (std::uint32_t x = 0; x < grid.width; ++x) {
    for (std::uint32_t y = 0; y < grid.height; ++y) {
      input[y] = distances[static_cast<std::size_t>(y) * grid.width + x];
    }
    distanceTransform(input, output);
    for (std::uint32_t y = 0; y < grid.height; ++y) {
      const auto idx = static_cast<std::size_t>(y) * grid.width + x;
      // Cover the source cell's square boundary conservatively, not only its center.
      const double distance = std::max(0.0, std::sqrt(output[y]) * grid.resolution -
        grid.resolution * std::sqrt(0.5));
      const auto cost = inflationCost(distance, config);
      if (cost == 100 || grid.costs[idx] >= 0) {
        grid.costs[idx] = static_cast<std::int8_t>(std::max<int>(grid.costs[idx], cost));
      }
      // Hard exclusions always win. Soft halos price unknown only when the
      // operator assigns it a numeric planning cost; raw evidence stays unknown.
    }
  }
}
void IncrementalGrid::reset(const Config & config)
{
  config.validate();
  config_ = config;
  grid_ = rasterize({}, config);
  terrain_.assign(grid_.raw.size(), TerrainState{});
  fresh_stamps_.assign(grid_.raw.size(), std::numeric_limits<int64_t>::min());
  dirty_.assign(grid_.raw.size(), 0);
  changed_cells_ = fitted_cells_ = inflated_cells_ = observed_cells_ = 0;
  full_inflation_ = true;
  finished_ = false;
  batch_stamp_ = std::numeric_limits<int64_t>::min();
  revised_.clear();
  held_hazards_ = cleared_hazards_ = rejected_heights_ = 0;
  obstacles_.clear(); occupied_columns_.clear(); obstacle_columns_.clear();
  revised_obstacles_.clear();
  marked_obstacles_ = cleared_obstacles_ = traced_rays_ = skipped_rays_ = 0;
}

void IncrementalGrid::extend(const Contribution & observation)
{
  if (observation.empty()) {return;}
  int min_x = observation.begin()->first.x, max_x = min_x;
  int min_y = observation.begin()->first.y, max_y = min_y;
  for (const auto & [key, cell] : observation) {
    if (!std::isfinite(cell.height) || !cell.up.allFinite() || cell.up.norm() < 0.5 ||
      cell.count == 0 || !std::isfinite(cell.variance) || cell.variance <= 0 ||
      std::abs(static_cast<int64_t>(key.x)) > 100000000 ||
      std::abs(static_cast<int64_t>(key.y)) > 100000000)
    {throw std::invalid_argument("invalid terrain observation");}
    min_x = std::min(min_x, key.x); max_x = std::max(max_x, key.x);
    min_y = std::min(min_y, key.y); max_y = std::max(max_y, key.y);
  }
  const int padding = static_cast<int>(std::ceil((config_.footprint_radius +
    config_.soft_radius + config_.slope_radius) / config_.resolution)) + 1;
  min_x -= padding; max_x += padding; min_y -= padding; max_y += padding;
  if (observed_cells_ > 0 || !obstacles_.empty()) {
    min_x = std::min(min_x, grid_.origin_x);
    min_y = std::min(min_y, grid_.origin_y);
    max_x = std::max(max_x, grid_.origin_x + static_cast<int>(grid_.width) - 1);
    max_y = std::max(max_y, grid_.origin_y + static_cast<int>(grid_.height) - 1);
    if (min_x == grid_.origin_x && min_y == grid_.origin_y &&
      max_x == grid_.origin_x + static_cast<int>(grid_.width) - 1 &&
      max_y == grid_.origin_y + static_cast<int>(grid_.height) - 1) {return;}
  }
  Grid next;
  next.origin_x = min_x; next.origin_y = min_y; next.resolution = config_.resolution;
  next.width = max_x - min_x + 1; next.height = max_y - min_y + 1;
  const auto size = static_cast<std::size_t>(next.width) * next.height;
  if (size > config_.max_cells) {throw std::runtime_error("hazard grid exceeds max_cells");}
  next.raw.assign(size, -1); next.costs.assign(size, -1);
  next.slope_deg.assign(size, std::numeric_limits<float>::quiet_NaN());
  std::vector<TerrainState> terrain(size);
  std::vector<int64_t> fresh_stamps(size, std::numeric_limits<int64_t>::min());
  std::vector<std::uint8_t> dirty(size, 0);
  if (observed_cells_ > 0 || !obstacles_.empty()) {
    for (std::uint32_t y = 0; y < grid_.height; ++y) {
      const auto old = static_cast<std::size_t>(y) * grid_.width;
      const auto dst = static_cast<std::size_t>(grid_.origin_y + y - min_y) * next.width +
        grid_.origin_x - min_x;
      std::copy_n(grid_.raw.begin() + old, grid_.width, next.raw.begin() + dst);
      std::copy_n(grid_.costs.begin() + old, grid_.width, next.costs.begin() + dst);
      std::copy_n(grid_.slope_deg.begin() + old, grid_.width, next.slope_deg.begin() + dst);
      std::copy_n(terrain_.begin() + old, grid_.width, terrain.begin() + dst);
      std::copy_n(fresh_stamps_.begin() + old, grid_.width, fresh_stamps.begin() + dst);
      std::copy_n(dirty_.begin() + old, grid_.width, dirty.begin() + dst);
    }
  }
  grid_ = std::move(next); terrain_ = std::move(terrain);
  fresh_stamps_ = std::move(fresh_stamps); dirty_ = std::move(dirty);
  full_inflation_ = true;
}

const TerrainState * IncrementalGrid::state(const Key & key) const
{
  const int x = key.x - grid_.origin_x, y = key.y - grid_.origin_y;
  if (x < 0 || y < 0 || x >= static_cast<int>(grid_.width) ||
    y >= static_cast<int>(grid_.height)) {return nullptr;}
  const auto & value = terrain_[static_cast<std::size_t>(y) * grid_.width + x];
  return value.estimate.count ? &value : nullptr;
}

void IncrementalGrid::dirtyNeighborhood(const Key & key, bool height_changed, bool confidence_changed)
{
  if (!height_changed && !confidence_changed) {return;}
  const int x = key.x - grid_.origin_x, y = key.y - grid_.origin_y;
  const int radius = static_cast<int>(std::ceil(config_.slope_radius / config_.resolution));
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      if ((dx * dx + dy * dy) * config_.resolution * config_.resolution >
        config_.slope_radius * config_.slope_radius) {continue;}
      const int nx = x + dx, ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(grid_.width) ||
        ny >= static_cast<int>(grid_.height)) {continue;}
      const auto idx = static_cast<std::size_t>(ny) * grid_.width + nx;
      const bool uncertain = terrain_[idx].raw == -1 ||
        (terrain_[idx].raw == 100 && !std::isfinite(grid_.slope_deg[idx]));
      if (height_changed || (confidence_changed && uncertain &&
        terrain_[idx].estimate.count)) {
        dirty_[idx] = 1;
      }
    }
  }
}

void IncrementalGrid::restore(const Key & key, const TerrainState & value)
{
  extend({{key, value.estimate}});
  const auto idx = static_cast<std::size_t>(key.y - grid_.origin_y) * grid_.width +
    key.x - grid_.origin_x;
  if (value.stamp < terrain_[idx].stamp) {return;}
  if (!terrain_[idx].estimate.count) {++observed_cells_;}
  terrain_[idx] = value;
  grid_.raw[idx] = value.raw;
  fresh_stamps_[idx] = std::numeric_limits<int64_t>::min();
  revised_.push_back(key);
  dirtyNeighborhood(key, true);
}

void IncrementalGrid::update(const Contribution & observation, int64_t stamp, bool fresh)
{
  if (finished_) {
    changed_cells_ = rejected_heights_ = 0; revised_.clear(); finished_ = false;
    revised_obstacles_.clear(); obstacle_columns_.clear();
    marked_obstacles_ = cleared_obstacles_ = traced_rays_ = skipped_rays_ = 0;
  }
  extend(observation);
  if (fresh) {batch_stamp_ = std::max(batch_stamp_, stamp);}
  const double floor = config_.height_noise * config_.height_noise / 8.0;
  const double reliable_variance = 4.0 * config_.height_noise * config_.height_noise;
  for (const auto & [key, cell] : observation) {
    const int x = key.x - grid_.origin_x, y = key.y - grid_.origin_y;
    const auto idx = static_cast<std::size_t>(y) * grid_.width + x;
    auto & value = terrain_[idx];
    if (stamp <= value.stamp) {continue;}  // Replay is not confirmation.
    const Cell before = value.estimate;
    bool accepted = true;
    if (!before.count || !fresh) {
      if (!before.count) {++observed_cells_;}
      value.estimate = cell; value.candidate_votes = 0;
    } else {
      const double delta = cell.height - before.height;
      const double gate = 3.0 * std::sqrt(before.variance + cell.variance);
      if (std::abs(delta) <= gate) {
        const double gain = before.variance / (before.variance + cell.variance);
        // Acquisition transforms introduce sub-nanometer roundoff. Do not let
        // fusion chase that noise and refit unchanged stationary terrain.
        if (std::abs(delta) > 1e-9) {value.estimate.height += gain * delta;}
        value.estimate.variance = std::max(floor, (1.0 - gain) * before.variance);
        value.estimate.up = cell.up;
        value.estimate.count = cell.count;
        value.candidate_votes = 0;
      } else {
        accepted = false; ++rejected_heights_;
        // Keep different height modes separate; averaging across a discontinuity
        // can invent a gentle ramp. Weak contradictory returns cannot replace it.
        if (cell.variance <= reliable_variance) {
          if (!value.candidate_votes || std::abs(cell.height-value.candidate.height) >
            3.0 * std::sqrt(cell.variance + value.candidate.variance)) {
            value.candidate = cell; value.candidate_votes = 1;
          } else {
            const double gain = value.candidate.variance /
              (value.candidate.variance + cell.variance);
            value.candidate.height += gain * (cell.height-value.candidate.height);
            value.candidate.variance = std::max(floor,
              (1.0-gain)*value.candidate.variance);
            value.candidate.up = cell.up; value.candidate.count = cell.count;
            value.candidate_votes = std::min(config_.clear_confirmations,
              value.candidate_votes + 1);
          }
          if (value.candidate_votes >= config_.clear_confirmations) {
            value.estimate = value.candidate;
            value.candidate_votes = 0; accepted = true;
          }
        }
      }
    }
    value.stamp = stamp;
    fresh_stamps_[idx] = fresh && accepted && cell.variance <= reliable_variance ? stamp :
      std::numeric_limits<int64_t>::min();
    const bool changed = !before.count || before.height != value.estimate.height ||
      !before.up.isApprox(value.estimate.up, 1e-12);
    revised_.push_back(key);
    if (changed) {++changed_cells_;}
    dirtyNeighborhood(key, changed, before.variance != value.estimate.variance);
  }
}

std::optional<Cell> IncrementalGrid::groundAt(const Key & key) const
{
  const auto trusted = [&](const Key & at) -> const TerrainState * {
    const auto * value = state(at);
    if (!value || value->raw < 0 || value->estimate.variance >
      4*config_.height_noise*config_.height_noise) {return nullptr;}
    const auto idx = static_cast<std::size_t>(at.y-grid_.origin_y)*grid_.width+
      at.x-grid_.origin_x;
    return std::isfinite(grid_.slope_deg[idx]) ? value : nullptr;
  };
  if (const auto * value = trusted(key)) {return value->estimate;}
  // Predict ground only from a well-supported surrounding retained plane.
  // Unknown surfaces and obstacle tops cannot become a ground reference here.
  struct Support {Eigen::Vector3d row; double z;};
  std::vector<Support> support;
  Eigen::Vector3d up = Eigen::Vector3d::Zero();
  const int radius = static_cast<int>(std::ceil(config_.slope_radius/config_.resolution));
  double xx=0, xy=0, yy=0, sx=0, sy=0;
  for (int dy=-radius; dy<=radius; ++dy) {
    for (int dx=-radius; dx<=radius; ++dx) {
      const double x=dx*config_.resolution, y=dy*config_.resolution;
      if (x*x+y*y > config_.slope_radius*config_.slope_radius) {continue;}
      const auto * value = trusted({key.x+dx, key.y+dy});
      if (!value) {continue;}
      support.push_back({Eigen::Vector3d(x,y,1), value->estimate.height});
      up += value->estimate.up;
      xx+=x*x; xy+=x*y; yy+=y*y; sx+=x; sy+=y;
    }
  }
  const double n = support.size();
  if (n < config_.min_neighbors || up.norm() < 0.5) {return std::nullopt;}
  const double a=xx/n-sx*sx/(n*n), b=xy/n-sx*sy/(n*n), d=yy/n-sy*sy/(n*n);
  const double coverage = std::sqrt(std::max(0.0, .5*(a+d-std::hypot(a-d,2*b)))) /
    config_.slope_radius;
  if (coverage < config_.min_plane_coverage) {return std::nullopt;}
  Eigen::Vector3d coefficients = Eigen::Vector3d::Zero();
  Eigen::Matrix3d lhs;
  for (int pass=0; pass<3; ++pass) {
    lhs.setZero(); Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (const auto & p : support) {
      const double w = pass == 0 ? 1.0 : std::min(1.0,
        config_.max_plane_error/std::max(1e-9,std::abs(p.z-p.row.dot(coefficients))));
      lhs.noalias() += w*p.row*p.row.transpose(); rhs.noalias() += w*p.row*p.z;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(lhs,Eigen::EigenvaluesOnly);
    if (eig.info()!=Eigen::Success || eig.eigenvalues().minCoeff()<1e-6) {return std::nullopt;}
    coefficients=lhs.ldlt().solve(rhs);
    if (!coefficients.allFinite()) {return std::nullopt;}
  }
  double error=0; int inliers=0;
  for (const auto & p : support) {
    const double residual=p.z-p.row.dot(coefficients); error+=residual*residual;
    inliers += std::abs(residual)<=config_.max_plane_error;
  }
  if (std::sqrt(error/n)>config_.max_plane_error || inliers<.75*n) {return std::nullopt;}
  return Cell{coefficients.z(),up.normalized(),support.size(),
    config_.height_noise*config_.height_noise};
}

void IncrementalGrid::setObstacle(const VoxelKey & key, const ObstacleState & value)
{
  const auto old=obstacles_.find(key);
  if (old!=obstacles_.end() && value.stamp<old->second.stamp) {return;}
  if (old==obstacles_.end() && obstacles_.size()>=static_cast<std::size_t>(config_.max_obstacle_voxels))
  {throw std::runtime_error("hazard obstacle voxels exceed max_obstacle_voxels");}
  extend({{{key.x,key.y},Cell{(key.z+.5)*config_.obstacle_voxel_height,
    Eigen::Vector3d::UnitZ(),1,config_.height_noise*config_.height_noise}}});
  const bool was_lethal=old!=obstacles_.end() && old->second.lethal;
  if (value.lethal!=was_lethal) {
    const Key column{key.x,key.y};
    if (value.lethal) {++occupied_columns_[column]; ++marked_obstacles_;}
    else {
      auto it=occupied_columns_.find(column);
      if (--it->second==0) {occupied_columns_.erase(it);}
      ++cleared_obstacles_;
    }
    obstacle_columns_.push_back(column);
  }
  obstacles_[key]=value; revised_obstacles_.push_back(key);
}

void IncrementalGrid::restoreObstacle(const VoxelKey & key, const ObstacleState & value)
{
  if (!std::isfinite(value.log_odds) || std::abs(static_cast<int64_t>(key.z))>100000000)
  {throw std::invalid_argument("invalid obstacle state");}
  const auto logit=[](double p) {return std::log(p/(1-p));};
  auto restored=value;
  // Resampling corrected voxel centers can merge columns. A collision is not
  // free-space evidence; retain the strongest state until actually observed.
  if (const auto old=obstacles_.find(key); old!=obstacles_.end()) {
    restored.log_odds=std::max(restored.log_odds,old->second.log_odds);
    restored.lethal=restored.lethal || old->second.lethal;
    restored.stamp=std::max(restored.stamp,old->second.stamp);
  }
  restored.log_odds=std::clamp(restored.log_odds,logit(config_.obstacle_clear_probability),
    logit(config_.obstacle_max_probability));
  setObstacle(key,restored);
}

void IncrementalGrid::observe(const std::vector<CloudObservation> & clouds, int64_t stamp, bool fresh,
  bool update_ground)
{
  struct Return {Eigen::Vector3d point, origin; bool trace;};
  struct Evidence {int hits {0}, misses {0};};
  std::vector<Return> returns;
  std::unordered_map<VoxelKey,Evidence,VoxelHash> evidence;
  std::unordered_map<Key,std::optional<Cell>,KeyHash> ground;
  SampleMap terrain;
  const auto voxel=[&](const Eigen::Vector3d & p) {
    return VoxelKey{static_cast<int>(std::floor(p.x()/config_.resolution)),
      static_cast<int>(std::floor(p.y()/config_.resolution)),
      static_cast<int>(std::floor(p.z()/config_.obstacle_voxel_height))};
  };
  for (const auto & cloud : clouds) {
    if (!cloud.pose.matrix().allFinite() || !cloud.up_in_body.allFinite() ||
      cloud.up_in_body.norm()<.5 || !cloud.sensor_origin.allFinite() || cloud.sensor_origin.norm()>10)
    {throw std::invalid_argument("invalid hazard observation origin/pose");}
    const auto up_body=cloud.up_in_body.normalized();
    const Eigen::Vector3d up=(cloud.pose.linear()*up_body).normalized();
    const Eigen::Vector3d origin=cloud.pose*cloud.sensor_origin;
    for (const auto & point : cloud.body_points) {
      if (!point.allFinite()) {continue;}
      const double h=point.dot(up_body), r2=std::max(0.0,point.squaredNorm()-h*h);
      if (h<config_.min_height || h>config_.max_height || r2>config_.max_range*config_.max_range)
      {continue;}
      const Eigen::Vector3d world=cloud.pose*point;
      if ((world.array().abs()>1e6).any()) {throw std::runtime_error("hazard coordinates exceed limits");}
      const auto vk=voxel(world); const Key key{vk.x,vk.y};
      auto [g,inserted]=ground.emplace(key,std::nullopt);
      if (inserted) {g->second=groundAt(key);}
      bool include=true;
      if (g->second) {
        const double above=(world.z()-g->second->height)*g->second->up.z();
        if (above>=config_.obstacle_min_height) {
          include=false;
          if (above<=config_.obstacle_max_height) {++evidence[vk].hits;}
        } else if (std::abs(above)<=config_.max_plane_error) {
          // A confirmed ground return can clear that SAME ground-height voxel;
          // it does not clear the unobserved space above it.
          ++evidence[vk].misses;
        }
      } else if (h>0) {
        // Bootstrap conservatively: an unsupported surface above the acquisition
        // body must not establish traversable ground from an obstacle's flat top.
        include=false;
      }
      if (include && update_ground) {
        auto & samples=terrain[key]; samples.heights.push_back(world.z());
        samples.up+=up; samples.range_sum+=std::sqrt(r2);
      }
      returns.push_back({world,origin,cloud.clear_rays});
    }
  }
  if (update_ground) {update(terrainFromSamples(terrain,config_),stamp,fresh);}
  if (!fresh) {return;}  // Historical replay never creates occupancy evidence.
  // Trace a bounded, evenly sampled subset. Missing/skipped rays cannot clear.
  const auto budget=static_cast<std::size_t>(config_.max_rays)-
    std::min(traced_rays_,static_cast<std::size_t>(config_.max_rays));
  const auto stride=budget ? std::max<std::size_t>(1,(returns.size()+budget-1)/budget) : 1;
  const double guard=std::max(config_.resolution,config_.obstacle_voxel_height)*.5+.02;
  for (std::size_t ri=0; ri<returns.size(); ++ri) {
    const auto & ray=returns[ri];
    if (!budget || ri%stride || !ray.trace || traced_rays_>=static_cast<std::size_t>(config_.max_rays))
    {++skipped_rays_; continue;}
    const Eigen::Vector3d delta=ray.point-ray.origin;
    const double length=delta.norm();
    if (length<=guard) {++skipped_rays_; continue;}
    ++traced_rays_;
    VoxelKey key=voxel(ray.origin);
    const double scale[]={config_.resolution,config_.resolution,config_.obstacle_voxel_height};
    int step[3]; double next[3], increment[3];
    int * coord[]={&key.x,&key.y,&key.z};
    for (int axis=0; axis<3; ++axis) {
      step[axis]=delta[axis]>0 ? 1 : -1;
      if (std::abs(delta[axis])<1e-12) {
        next[axis]=increment[axis]=std::numeric_limits<double>::infinity();
      } else {
        const double boundary=(*coord[axis]+(step[axis]>0 ? 1 : 0))*scale[axis];
        next[axis]=(boundary-ray.origin[axis])/delta[axis];
        increment[axis]=scale[axis]/std::abs(delta[axis]);
      }
    }
    const double end=1-guard/length;
    double entered=0;
    const int max_steps=static_cast<int>(std::ceil(delta.cwiseAbs().sum()/
      std::min(config_.resolution,config_.obstacle_voxel_height)))+4;
    for (int iteration=0; entered<end && iteration<max_steps; ++iteration) {
      const int axis=next[0]<=next[1] && next[0]<=next[2] ? 0 : (next[1]<=next[2] ? 1 : 2);
      const double exited=std::min(end,next[axis]);
      if ((exited-entered)*length>config_.resolution*.25 && obstacles_.count(key))
      {++evidence[key].misses;}
      entered=next[axis]; *coord[axis]+=step[axis]; next[axis]+=increment[axis];
    }
  }
  const auto logit=[](double p) {return std::log(p/(1-p));};
  const double low=logit(config_.obstacle_clear_probability), high=logit(config_.obstacle_max_probability);
  for (const auto & [key,observed] : evidence) {
    const auto previous=obstacles_.find(key);
    if (previous!=obstacles_.end() && stamp<=previous->second.stamp) {continue;}
    const bool hit=observed.hits>=config_.obstacle_min_points;
    if (!update_ground && hit) {continue;}  // Deferred rays only clear; old endpoints never mark anew.
    if (update_ground && observed.hits>0 && !hit && previous!=obstacles_.end()) {
      // Even a sparse newer return disproves an older clearing ray. Preserve
      // its acquisition ordering without pretending it is a strong new hit.
      auto held=previous->second; held.stamp=stamp; setObstacle(key,held); continue;
    }
    // Any occupied return vetoes clearing, even if too sparse to mark a new voxel.
    const bool miss=observed.hits==0 && observed.misses>=config_.obstacle_min_points;
    if (!hit && (!miss || previous==obstacles_.end())) {continue;}
    auto state=previous==obstacles_.end() ? ObstacleState{} : previous->second;
    state.log_odds=std::clamp(state.log_odds+logit(hit ? config_.obstacle_hit_probability :
      config_.obstacle_miss_probability),low,high);
    state.stamp=stamp;
    if (hit && state.log_odds>=logit(config_.obstacle_mark_probability)) {state.lethal=true;}
    if (miss && state.log_odds<=low+1e-12) {state.lethal=false;}
    setObstacle(key,state);
    if (miss && !state.lethal && state.log_odds<=low+1e-12) {obstacles_.erase(key);}
  }
}

const Grid & IncrementalGrid::finish()
{
  fitted_cells_ = inflated_cells_ = held_hazards_ = cleared_hazards_ = 0;
  struct Offset {int dx, dy; double x, y;};
  std::vector<Offset> offsets;
  const int radius = static_cast<int>(std::ceil(config_.slope_radius / config_.resolution));
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const double x = dx * config_.resolution, y = dy * config_.resolution;
      if (x*x + y*y <= config_.slope_radius*config_.slope_radius)
      {offsets.push_back({dx, dy, x, y});}
    }
  }
  struct Neighbor {double x, y, z, weight; bool fresh;};
  std::vector<Neighbor> neighbors; neighbors.reserve(offsets.size());
  const double noise_variance = config_.height_noise*config_.height_noise;
  const auto coverage = [&](double xx, double xy, double yy, double sx, double sy, double sw) {
    if (sw <= 0) {return 0.0;}
    const double a = xx/sw - (sx/sw)*(sx/sw);
    const double b = xy/sw - (sx/sw)*(sy/sw);
    const double d = yy/sw - (sy/sw)*(sy/sw);
    const double minor = 0.5*(a+d-std::hypot(a-d, 2*b));
    return std::sqrt(std::max(0.0, minor))/config_.slope_radius;
  };
  int min_x = grid_.width, min_y = grid_.height, max_x = -1, max_y = -1;
  for (std::size_t idx = 0; idx < dirty_.size(); ++idx) {
    // A retained hazard whose cached slope is already clearly safe still needs
    // confirmations even when fresh heights are identical. Check its center
    // once here, rather than walking every unchanged cell's neighborhood.
    const bool clearing = terrain_[idx].raw == 100 &&
      grid_.slope_deg[idx] <= config_.max_slope_deg-config_.clear_slope_margin &&
      batch_stamp_ != std::numeric_limits<int64_t>::min() &&
      fresh_stamps_[idx] == batch_stamp_ && batch_stamp_ > terrain_[idx].last_vote;
    if (!dirty_[idx] && !clearing) {continue;}
    dirty_[idx] = 0;
    auto & value = terrain_[idx];
    if (!value.estimate.count) {continue;}
    ++fitted_cells_;
    const int x = idx % grid_.width, y = idx / grid_.width;
    const auto & cell = value.estimate;
    neighbors.clear();
    double fresh_xx=0, fresh_xy=0, fresh_yy=0, fresh_x=0, fresh_y=0;
    int fresh_count=0;
    for (const auto & offset : offsets) {
      const int nx = x+offset.dx, ny = y+offset.dy;
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(grid_.width) ||
        ny >= static_cast<int>(grid_.height)) {continue;}
      const auto ni = static_cast<std::size_t>(ny)*grid_.width+nx;
      const auto & neighbor = terrain_[ni].estimate;
      if (!neighbor.count) {continue;}
      const bool fresh = batch_stamp_ != std::numeric_limits<int64_t>::min() &&
        fresh_stamps_[ni] == batch_stamp_;
      neighbors.push_back({offset.x, offset.y, neighbor.height-cell.height,
        std::clamp(noise_variance/neighbor.variance, 0.05, 8.0), fresh});
      if (fresh) {
        ++fresh_count; fresh_xx+=offset.x*offset.x; fresh_xy+=offset.x*offset.y;
        fresh_yy+=offset.y*offset.y; fresh_x+=offset.x; fresh_y+=offset.y;
      }
    }
    Eigen::Vector3d coefficients = Eigen::Vector3d::Zero();
    Eigen::Matrix3d lhs = Eigen::Matrix3d::Zero();
    bool valid = neighbors.size() >= static_cast<std::size_t>(config_.min_neighbors);
    for (int pass = 0; valid && pass < 3; ++pass) {
      double xx=0, xy=0, yy=0, sx=0, sy=0, sw=0, xz=0, yz=0, sz=0;
      for (const auto & p : neighbors) {
        const double residual = std::abs(p.z -
          (p.x*coefficients.x()+p.y*coefficients.y()+coefficients.z()));
        const double w = p.weight * (pass == 0 ? 1.0 :
          std::min(1.0, config_.max_plane_error/std::max(1e-9, residual)));
        xx+=w*p.x*p.x; xy+=w*p.x*p.y; yy+=w*p.y*p.y;
        sx+=w*p.x; sy+=w*p.y; sw+=w;
        xz+=w*p.x*p.z; yz+=w*p.y*p.z; sz+=w*p.z;
      }
      lhs << xx, xy, sx, xy, yy, sy, sx, sy, sw;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> conditioning(lhs, Eigen::EigenvaluesOnly);
      valid = conditioning.info() == Eigen::Success &&
        conditioning.eigenvalues().minCoeff() >= 1e-6 &&
        coverage(xx, xy, yy, sx, sy, sw) >= config_.min_plane_coverage;
      if (valid) {
        coefficients = lhs.ldlt().solve(Eigen::Vector3d(xz, yz, sz));
        valid = coefficients.allFinite();
      }
    }
    double error=0, weight=0;
    if (valid) {
      int inliers=0;
      for (const auto & p : neighbors) {
        const double residual = p.z -
          (p.x*coefficients.x()+p.y*coefficients.y()+coefficients.z());
        error+=p.weight*residual*residual; weight+=p.weight;
        if (std::abs(residual) <= config_.max_plane_error) {++inliers;}
      }
      valid = std::sqrt(error/weight) <= config_.max_plane_error &&
        inliers >= 0.75*neighbors.size();
    }
    double slope = std::numeric_limits<double>::quiet_NaN();
    double slope_error = std::numeric_limits<double>::infinity();
    int8_t raw = -1;
    if (valid) {
      const Eigen::Vector3d normal =
        Eigen::Vector3d(-coefficients.x(), -coefficients.y(), 1).normalized();
      slope = std::acos(std::clamp(std::abs(normal.dot(cell.up)), 0.0, 1.0)) *
        180.0/3.14159265358979323846;
      const Eigen::Matrix3d covariance = lhs.ldlt().solve(Eigen::Matrix3d::Identity()) *
        std::max(noise_variance, error/std::max(1.0, weight-3.0));
      // Conservative fit uncertainty proxy. Pose/gravity covariance is not
      // propagated; corrected up defines the slope reference above.
      slope_error = std::sqrt(std::max(0.0, covariance(0,0)+covariance(1,1))) *
        180.0/3.14159265358979323846;
      raw = slope >= config_.max_slope_deg ? 100 :
        (slope+2*slope_error < config_.max_slope_deg ? 0 : -1);
    }
    const bool new_evidence = fresh_stamps_[idx] == batch_stamp_ &&
      batch_stamp_ > value.last_vote;
    const bool fresh_support = fresh_count >= config_.min_neighbors &&
      fresh_count >= 0.60*neighbors.size() &&
      coverage(fresh_xx, fresh_xy, fresh_yy, fresh_x, fresh_y, fresh_count) >=
      config_.min_plane_coverage;
    if (value.raw == 100) {
      raw = 100;  // Inconclusive/unknown is not evidence that a hazard disappeared.
      if (new_evidence && fresh_support) {
        value.last_vote = batch_stamp_;
        if (valid && slope+2*slope_error <= config_.max_slope_deg-config_.clear_slope_margin) {
          value.clear_votes = std::min(config_.clear_confirmations, value.clear_votes+1);
          if (value.clear_votes >= config_.clear_confirmations) {
            raw = 0; value.clear_votes = 0; ++cleared_hazards_;
          }
        } else {value.clear_votes = 0;}
      }
      if (raw == 100) {++held_hazards_;}
    } else if (raw == 100) {value.clear_votes = 0;}
    value.raw = raw;
    revised_.push_back({x+grid_.origin_x, y+grid_.origin_y});
    grid_.slope_deg[idx] = static_cast<float>(slope);
    const Key key{x+grid_.origin_x, y+grid_.origin_y};
    const int8_t combined = occupied_columns_.count(key) ? 100 : raw;
    if (grid_.raw[idx] != combined) {
      grid_.raw[idx] = combined;
      min_x=std::min(min_x,x); min_y=std::min(min_y,y);
      max_x=std::max(max_x,x); max_y=std::max(max_y,y);
    }
  }
  for (const auto & key : obstacle_columns_) {
    const int x = key.x-grid_.origin_x, y = key.y-grid_.origin_y;
    const auto idx = static_cast<std::size_t>(y)*grid_.width+x;
    const int8_t combined = occupied_columns_.count(key) ? 100 : terrain_[idx].raw;
    if (grid_.raw[idx] != combined) {
      grid_.raw[idx] = combined;
      min_x=std::min(min_x,x); min_y=std::min(min_y,y);
      max_x=std::max(max_x,x); max_y=std::max(max_y,y);
    }
  }
  if (full_inflation_) {
    inflate(grid_, config_); inflated_cells_ = grid_.raw.size(); full_inflation_ = false;
  } else if (max_x >= 0) {
    // Changed raw cells affect costs only inside the outer halo. A second
    // halo around that output rectangle includes all competing hazard sources,
    // including unchanged hazards across the rectangle's edge.
    const int halo = static_cast<int>(std::ceil((config_.footprint_radius +
      config_.soft_radius) / config_.resolution + std::sqrt(0.5)));
    const int out_x0 = std::max(0, min_x - halo), out_y0 = std::max(0, min_y - halo);
    const int out_x1 = std::min(static_cast<int>(grid_.width) - 1, max_x + halo);
    const int out_y1 = std::min(static_cast<int>(grid_.height) - 1, max_y + halo);
    const int in_x0 = std::max(0, out_x0 - halo), in_y0 = std::max(0, out_y0 - halo);
    const int in_x1 = std::min(static_cast<int>(grid_.width) - 1, out_x1 + halo);
    const int in_y1 = std::min(static_cast<int>(grid_.height) - 1, out_y1 + halo);
    Grid patch;
    patch.resolution = grid_.resolution;
    patch.width = in_x1 - in_x0 + 1; patch.height = in_y1 - in_y0 + 1;
    patch.raw.resize(static_cast<std::size_t>(patch.width) * patch.height);
    for (int y = in_y0; y <= in_y1; ++y) {
      std::copy_n(grid_.raw.begin() + static_cast<std::size_t>(y) * grid_.width + in_x0,
        patch.width, patch.raw.begin() + static_cast<std::size_t>(y - in_y0) * patch.width);
    }
    inflate(patch, config_);
    inflated_cells_ = patch.raw.size();
    for (int y = out_y0; y <= out_y1; ++y) {
      std::copy_n(patch.costs.begin() + static_cast<std::size_t>(y - in_y0) * patch.width +
        out_x0 - in_x0, out_x1 - out_x0 + 1,
        grid_.costs.begin() + static_cast<std::size_t>(y) * grid_.width + out_x0);
    }
  }
  finished_ = true;
  return grid_;
}
}  // namespace graphslam::hazard
