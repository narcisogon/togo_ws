// Backend-owned terrain processing. Compiled into graph_based_slam_component;
// this directory does not define another ROS node or a library target.
#ifndef TOGO_GRAPH_HAZARD_MAPPING_HPP_
#define TOGO_GRAPH_HAZARD_MAPPING_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

namespace graphslam::hazard
{
struct Config
{
  double resolution {0.05};
  double min_height {-3.0};
  double max_height {2.0};
  double max_range {12.0};
  double slope_radius {0.40};
  double max_slope_deg {25.0};
  double lethal_radius {0.10};
  double soft_radius {0.30};
  int unknown_cost {-1};  // Published planning cost; raw unknown remains -1.
  // The grid describes allowed ROBOT CENTER positions. These two inflation
  // radii are extra margins outside this conservative physical footprint.
  double footprint_radius {0.6685057966};
  int min_points {1};
  int min_neighbors {6};
  double terrain_percentile {0.25};
  double height_noise {0.01};
  double min_plane_coverage {0.20};
  double max_plane_error {0.05};
  int clear_confirmations {3};
  double clear_slope_margin {3.0};
  double obstacle_min_height {0.15};
  double obstacle_max_height {1.8};
  double obstacle_voxel_height {0.10};
  int obstacle_min_points {3};
  double obstacle_hit_probability {0.85};
  double obstacle_miss_probability {0.25};
  double obstacle_mark_probability {0.70};
  double obstacle_clear_probability {0.30};
  double obstacle_max_probability {0.90};
  int max_rays {3000};
  int max_obstacle_voxels {200000};
  std::size_t max_cells {4000000};
  void validate() const;
  bool sameTerrain(const Config & other) const;
};

struct Key
{
  int x {0};
  int y {0};
  bool operator==(const Key & other) const {return x == other.x && y == other.y;}
};
struct KeyHash
{
  std::size_t operator()(const Key & key) const;
};
struct Cell
{
  double height {0.0};
  Eigen::Vector3d up {Eigen::Vector3d::UnitZ()};
  std::size_t count {0};
  double variance {0.0001};  // Conservative height uncertainty, in square meters.
};
using Contribution = std::unordered_map<Key, Cell, KeyHash>;

struct VoxelKey
{
  int x {0}, y {0}, z {0};
  bool operator==(const VoxelKey & other) const
  {return x == other.x && y == other.y && z == other.z;}
};
struct VoxelHash
{
  std::size_t operator()(const VoxelKey & key) const
  {return KeyHash{}({key.x, key.y}) ^ (std::hash<int>{}(key.z) * 0x9e3779b9U);}
};
struct ObstacleState
{
  double log_odds {0.0};
  bool lethal {false};
  int64_t stamp {std::numeric_limits<int64_t>::min()};
};
using Obstacles = std::unordered_map<VoxelKey, ObstacleState, VoxelHash>;

// Retained through graph reprojection, including the alternative height mode.
struct TerrainState
{
  Cell estimate;
  Cell candidate;
  int candidate_votes {0};
  int clear_votes {0};
  int8_t raw {-1};
  int64_t stamp {std::numeric_limits<int64_t>::min()};
  int64_t last_vote {std::numeric_limits<int64_t>::min()};
};

struct Grid
{
  int origin_x {0};  // integer cells in the fixed map lattice
  int origin_y {0};
  std::uint32_t width {0};
  std::uint32_t height {0};
  double resolution {0.05};
  std::vector<std::int8_t> raw;  // -1 unknown, 0 measured traversable, 100 hazard
  std::vector<std::int8_t> costs;
  std::vector<float> slope_deg;
};

// up_in_body is acquisition-pose gravity-up, NOT the body's own Z axis.
// A full corrected SE(3) pose projects observations and that gravity direction.
Contribution project(
  const std::vector<Eigen::Vector3d> & body_points,
  const Eigen::Isometry3d & corrected_pose, const Eigen::Vector3d & up_in_body,
  const Config & config);

struct CloudObservation
{
  std::vector<Eigen::Vector3d> body_points;
  Eigen::Isometry3d pose {Eigen::Isometry3d::Identity()};
  Eigen::Vector3d up_in_body {Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d sensor_origin {Eigen::Vector3d::Zero()};  // acquisition body coordinates
  bool clear_rays {false};  // true only when the ray origin is trustworthy
};
// Filter each scan at its own acquisition pose, then estimate terrain from the batch.
Contribution projectBatch(const std::vector<CloudObservation> & clouds, const Config & config);

// Persistent confidence-aware terrain, compiled into the backend.
class IncrementalGrid
{
public:
  void reset(const Config & config);
  void update(const Contribution & observation, int64_t stamp, bool fresh = true);
  void observe(const std::vector<CloudObservation> & clouds, int64_t stamp, bool fresh = true,
    bool update_ground = true);
  void restore(const Key & key, const TerrainState & state);
  void restoreObstacle(const VoxelKey & key, const ObstacleState & state);
  const Obstacles & obstacles() const {return obstacles_;}
  const std::vector<VoxelKey> & revisedObstacles() const {return revised_obstacles_;}
  const TerrainState * state(const Key & key) const;
  const std::vector<Key> & revisedCells() const {return revised_;}
  const Grid & finish();
  std::size_t changedCells() const {return changed_cells_;}
  std::size_t fittedCells() const {return fitted_cells_;}
  std::size_t inflatedCells() const {return inflated_cells_;}
  std::size_t observedCells() const {return observed_cells_;}
  std::size_t heldHazards() const {return held_hazards_;}
  std::size_t clearedHazards() const {return cleared_hazards_;}
  std::size_t rejectedHeights() const {return rejected_heights_;}
  std::size_t markedObstacles() const {return marked_obstacles_;}
  std::size_t clearedObstacles() const {return cleared_obstacles_;}
  std::size_t tracedRays() const {return traced_rays_;}
  std::size_t skippedRays() const {return skipped_rays_;}

private:
  void extend(const Contribution & observation);
  void dirtyNeighborhood(const Key & key, bool height_changed, bool confidence_changed = false);
  std::optional<Cell> groundAt(const Key & key) const;
  void setObstacle(const VoxelKey & key, const ObstacleState & state);
  Config config_;
  Grid grid_;
  std::vector<TerrainState> terrain_;
  std::vector<int64_t> fresh_stamps_;
  std::vector<std::uint8_t> dirty_;
  std::size_t changed_cells_ {0}, fitted_cells_ {0}, inflated_cells_ {0}, observed_cells_ {0};
  bool full_inflation_ {true};
  bool finished_ {false};
  int64_t batch_stamp_ {std::numeric_limits<int64_t>::min()};
  std::vector<Key> revised_;
  std::size_t held_hazards_ {0}, cleared_hazards_ {0}, rejected_heights_ {0};
  Obstacles obstacles_;
  std::unordered_map<Key, int, KeyHash> occupied_columns_;
  std::vector<Key> obstacle_columns_;
  std::vector<VoxelKey> revised_obstacles_;
  std::size_t marked_obstacles_ {0}, cleared_obstacles_ {0}, traced_rays_ {0}, skipped_rays_ {0};
};

Grid rasterize(const std::vector<const Contribution *> & sources, const Config & config);
void inflate(Grid & grid, const Config & config);
int inflationCost(double hazard_distance, const Config & config);
}  // namespace graphslam::hazard
#endif
