#include "hazard/hazard_mapping.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <random>

using namespace graphslam::hazard;

namespace
{
std::vector<Eigen::Vector3d> plane(double degrees)
{
  std::vector<Eigen::Vector3d> points;
  for (int y = -20; y < 20; ++y) {
    for (int x = -20; x < 20; ++x) {
      const double px = (x + 0.5) * 0.05, py = (y + 0.5) * 0.05;
      points.emplace_back(px, py, -0.3 + std::tan(degrees * M_PI / 180.0) * px);
    }
  }
  return points;
}

Grid observedPlane(double terrain_slope, const Eigen::Isometry3d & robot,
  const Eigen::Isometry3d & correction = Eigen::Isometry3d::Identity())
{
  Config config;
  std::vector<Eigen::Vector3d> body;
  for (const auto & point : plane(terrain_slope)) {body.push_back(robot.inverse() * point);}
  const auto source = project(body, correction * robot,
    robot.linear().transpose() * Eigen::Vector3d::UnitZ(), config);
  IncrementalGrid map; map.reset(config); map.update(source, 1);
  return map.finish();
}

double meanSlope(const Grid & grid)
{
  double total = 0.0; std::size_t count = 0;
  for (auto value : grid.slope_deg) {if (std::isfinite(value)) {total += value; ++count;}}
  return count ? total / count : -1.0;
}
}  // namespace

TEST(HazardMapping, TerrainSlopeIsIndependentOfRoverPitchAndRoll)
{
  const auto upright = observedPlane(20.0, Eigen::Isometry3d::Identity());
  Eigen::Isometry3d tilted = Eigen::Isometry3d::Identity();
  tilted.linear() = (Eigen::AngleAxisd(-20.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(12.0 * M_PI / 180.0, Eigen::Vector3d::UnitX())).toRotationMatrix();
  const auto downhill = observedPlane(20.0, tilted);
  EXPECT_NEAR(meanSlope(upright), 20.0, 1e-3);
  EXPECT_NEAR(meanSlope(downhill), 20.0, 1e-3);
  EXPECT_EQ(upright.raw, downhill.raw);
  EXPECT_EQ(upright.costs, downhill.costs);
}

TEST(HazardMapping, SteepSlopeStaysHazardousWhenRoverMatchesIt)
{
  Eigen::Isometry3d robot = Eigen::Isometry3d::Identity();
  robot.linear() = Eigen::AngleAxisd(-30.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const auto grid = observedPlane(30.0, robot);
  EXPECT_NEAR(meanSlope(grid), 30.0, 1e-3);
  EXPECT_GT(std::count(grid.raw.begin(), grid.raw.end(), 100), 1000);
}

TEST(HazardMapping, GraphRotationPreservesSlopeAgainstCorrectedGravity)
{
  Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  correction.linear() = Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
  correction.translation().x() = 2.0;
  const auto grid = observedPlane(20.0, Eigen::Isometry3d::Identity(), correction);
  EXPECT_NEAR(meanSlope(grid), 20.0, 0.5);
  EXPECT_EQ(std::count(grid.raw.begin(), grid.raw.end(), 100), 0);
}

TEST(HazardMapping, InputHeightUsesAcquisitionGravityRatherThanBodyZ)
{
  Config config; config.min_height = -0.5; config.max_height = -0.1;
  Eigen::Isometry3d robot = Eigen::Isometry3d::Identity();
  robot.linear() = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
  std::vector<Eigen::Vector3d> body;
  for (const auto & p : plane(0.0)) {body.push_back(robot.inverse() * p);}
  const auto source = project(body, robot, robot.linear().transpose() * Eigen::Vector3d::UnitZ(), config);
  EXPECT_EQ(source.size(), 1600u);
  IncrementalGrid map; map.reset(config); map.update(source, 1);
  const auto grid = map.finish();
  EXPECT_NEAR(meanSlope(grid), 0.0, 1e-3);
}

TEST(HazardMapping, MissingOrCollinearSupportRemainsUnknown)
{
  Config config;
  const auto empty = rasterize({}, config);
  EXPECT_TRUE(std::all_of(empty.costs.begin(), empty.costs.end(), [](auto cost) {return cost == -1;}));
  std::vector<Eigen::Vector3d> line;
  for (int i = 0; i < 20; ++i) {line.emplace_back(i * 0.05 + 0.025, 0.025, -0.3);}
  const auto source = project(line, Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config);
  const auto grid = rasterize({&source}, config);
  EXPECT_TRUE(std::all_of(grid.raw.begin(), grid.raw.end(), [](auto cost) {return cost == -1;}));
}

TEST(HazardMapping, InflationAdaptsToBothRadiiAndNeverMakesSoftCostsLethal)
{
  Config config; config.footprint_radius = 0.0;
  EXPECT_EQ(inflationCost(0.10, config), 100);
  EXPECT_EQ(inflationCost(0.15, config), 74);
  EXPECT_EQ(inflationCost(0.20, config), 50);
  EXPECT_EQ(inflationCost(0.25, config), 25);
  EXPECT_EQ(inflationCost(0.30, config), 0);
  config.soft_radius = 0.22;
  EXPECT_EQ(inflationCost(0.16, config), 50);
  EXPECT_EQ(inflationCost(0.22, config), 0);
  EXPECT_EQ(inflationCost(0.10001, config), 99);
}

TEST(HazardMapping, PhysicalFootprintPrecedesAdditionalInflation)
{
  Config config; config.footprint_radius = 0.67;
  EXPECT_EQ(inflationCost(0.77, config), 100);
  EXPECT_EQ(inflationCost(0.87, config), 50);
  EXPECT_EQ(inflationCost(0.97, config), 0);
}

TEST(HazardMapping, OverlappingHalosUseNearestHazardNotSum)
{
  Config config; config.resolution = 0.02; config.footprint_radius = 0.0;
  Grid grid; grid.width = 51; grid.height = 51; grid.resolution = 0.02;
  grid.raw.assign(51 * 51, 0);
  grid.raw[25 * 51 + 15] = 100; grid.raw[25 * 51 + 35] = 100;
  inflate(grid, config);
  const int expected = inflationCost(0.20 - 0.02 * std::sqrt(0.5), config);
  EXPECT_EQ(grid.costs[25 * 51 + 25], expected);
  EXPECT_LT(grid.costs[25 * 51 + 25], 100);
}

TEST(HazardMapping, SoftInflationDoesNotTurnUnknownIntoTraversableTerrain)
{
  Config config; config.resolution = 0.02; config.footprint_radius = 0.0;
  Grid grid; grid.width = 51; grid.height = 51; grid.resolution = 0.02;
  grid.raw.assign(51 * 51, -1); grid.raw[25 * 51 + 25] = 100;
  inflate(grid, config);
  EXPECT_EQ(grid.costs[25 * 51 + 28], 100);
  EXPECT_EQ(grid.costs[25 * 51 + 35], -1);
}

TEST(HazardMapping, InvalidParametersAndExcessiveGridAreRejected)
{
  Config config; config.soft_radius = config.lethal_radius;
  EXPECT_THROW(config.validate(), std::invalid_argument);
  config = Config{}; config.max_cells = 100;
  const auto source = project(plane(0.0), Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config);
  EXPECT_THROW(rasterize({&source}, config), std::runtime_error);
  config = Config{}; config.soft_radius = 1e100;
  EXPECT_THROW(config.validate(), std::invalid_argument);
}

TEST(HazardMapping, DistanceTransformMatchesBruteForceClearance)
{
  Config config; config.resolution = 0.05; config.footprint_radius = 0.0;
  Grid grid; grid.width = 37; grid.height = 29; grid.resolution = config.resolution;
  grid.raw.assign(grid.width * grid.height, 0);
  const std::vector<Key> hazards{{1, 1}, {12, 20}, {35, 27}, {18, 13}};
  for (const auto & h : hazards) {grid.raw[h.y * grid.width + h.x] = 100;}
  inflate(grid, config);
  for (int y = 0; y < static_cast<int>(grid.height); ++y) {
    for (int x = 0; x < static_cast<int>(grid.width); ++x) {
      double distance = 1e10;
      for (const auto & h : hazards) {
        distance = std::min(distance, std::hypot(x - h.x, y - h.y));
      }
      const auto expected = inflationCost(std::max(0.0,
        (distance - std::sqrt(0.5)) * config.resolution), config);
      EXPECT_EQ(grid.costs[y * grid.width + x], expected);
    }
  }
}

namespace
{
void expectSameGrid(const Grid & actual, const Grid & expected)
{
  ASSERT_EQ(actual.origin_x, expected.origin_x);
  ASSERT_EQ(actual.origin_y, expected.origin_y);
  ASSERT_EQ(actual.width, expected.width);
  ASSERT_EQ(actual.height, expected.height);
  EXPECT_EQ(actual.raw, expected.raw);
  EXPECT_EQ(actual.costs, expected.costs);
  for (std::size_t i = 0; i < actual.raw.size(); ++i) {
    if (std::isfinite(expected.slope_deg[i])) {
      EXPECT_NEAR(actual.slope_deg[i], expected.slope_deg[i], 1e-4);
    } else {EXPECT_FALSE(std::isfinite(actual.slope_deg[i]));}
  }
}
}

TEST(HazardMapping, BatchCombinesSupportAndFiltersEachAcquisitionPose)
{
  Config config; config.min_points = 3; config.min_height = -0.5; config.max_height = -0.1;
  std::vector<CloudObservation> clouds;
  for (int i = 0; i < 3; ++i) {
    CloudObservation cloud;
    cloud.pose.linear() = Eigen::AngleAxisd(i * 0.15, Eigen::Vector3d::UnitY()).toRotationMatrix();
    cloud.pose.translation().x() = i * 0.1;
    cloud.up_in_body = cloud.pose.linear().transpose() * Eigen::Vector3d::UnitZ();
    cloud.body_points.push_back(cloud.pose.inverse() * Eigen::Vector3d(0.025, 0.025, -0.3));
    cloud.body_points.push_back(cloud.pose.inverse() * Eigen::Vector3d(0.025, 0.025, 2.0));
    EXPECT_TRUE(project(cloud.body_points, cloud.pose, cloud.up_in_body, config).empty());
    clouds.push_back(std::move(cloud));
  }
  const auto cells = projectBatch(clouds, config);
  ASSERT_EQ(cells.size(), 1u);
  EXPECT_EQ(cells.at({0, 0}).count, 3u);
  EXPECT_NEAR(cells.at({0, 0}).height, -0.3, 1e-9);
}

TEST(HazardMapping, PartialInflationMatchesFullDistanceTransformAfterConfirmedChanges)
{
  Config config; config.footprint_radius = 0;
  auto all = project(plane(0), Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config);
  IncrementalGrid map; map.reset(config); map.update(all, 1);
  expectSameGrid(map.finish(), rasterize({&all}, config));
  std::mt19937 random(17);
  int64_t stamp = 1;
  for (int tick = 2; tick <= 10; ++tick) {
    Contribution patch;
    for (auto & [key, cell] : all) {
      if (key.x >= -3 && key.x <= 3 && key.y >= -3 && key.y <= 3) {
        cell.height = tick % 2 ? -0.3 : -0.3 + (random() % 100) / 100.0;
        patch.emplace(key, cell);
      }
    }
    map.update(patch, ++stamp);
    const auto & actual = map.finish();
    auto full = actual; inflate(full, config);
    EXPECT_TRUE(actual.costs == full.costs);
    EXPECT_LT(map.fittedCells(), all.size());
    if (tick % 2) {
      for (int confirmation = 0; confirmation < 6; ++confirmation) {
        map.update(all, ++stamp); map.finish();
      }
      EXPECT_EQ(std::count(map.finish().costs.begin(), map.finish().costs.end(), 100), 0);
    }
  }
}

TEST(HazardMapping, UnchangedAndOlderObservationsDoNotRefitOrRestoreHazards)
{
  Config config; config.footprint_radius = 0;
  const auto flat = project(plane(0), Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config);
  const auto steep = project(plane(30), Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config);
  IncrementalGrid map; map.reset(config); map.update(flat, 10); map.finish();
  map.update(flat, 11); map.finish();
  EXPECT_EQ(map.changedCells(), 0u); EXPECT_EQ(map.fittedCells(), 0u); EXPECT_EQ(map.inflatedCells(), 0u);
  map.update(steep, 9);
  expectSameGrid(map.finish(), rasterize({&flat}, config));
  EXPECT_EQ(map.fittedCells(), 0u);
}

TEST(HazardMapping, PartialCoverageAndExpansionPreserveEarlierTerrain)
{
  Config config; config.footprint_radius = 0;
  auto all = project(plane(0), Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config);
  IncrementalGrid map; map.reset(config); map.update(all, 1); map.finish();
  auto distant_points = plane(30);
  for (auto & p : distant_points) {p.x() += 5;}
  const auto distant = project(distant_points, Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config);
  map.update(distant, 2);
  all.insert(distant.begin(), distant.end());
  expectSameGrid(map.finish(), rasterize({&all}, config));
  EXPECT_EQ(map.observedCells(), all.size());
}

TEST(HazardMapping, PartialInflationKeepsUnchangedHazardsOutsideTheUpdatedArea)
{
  Config config; config.footprint_radius = 0;
  auto all = project(plane(30), Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config);
  IncrementalGrid map; map.reset(config); map.update(all, 1); map.finish();
  Contribution patch;
  for (auto & [key, cell] : all) {
    if (std::abs(key.x) <= 4 && std::abs(key.y) <= 4) {
      cell.height = -0.3; patch.emplace(key, cell);
    }
  }
  map.update(patch, 2);
  auto actual = map.finish(); auto full = actual; inflate(full, config);
  EXPECT_TRUE(actual.costs == full.costs);
  EXPECT_EQ(map.state({15, 15})->raw, 100);
}

namespace
{
Contribution measurement(double degrees, double variance = 0.000025)
{
  Config config;
  auto cells = project(plane(degrees), Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config);
  for (auto & [key, cell] : cells) {cell.variance = variance; cell.count = 4;}
  return cells;
}

int rawAt(const Grid & grid, int x = 0, int y = 0)
{
  return grid.raw[static_cast<std::size_t>(y-grid.origin_y)*grid.width+x-grid.origin_x];
}
}

TEST(HazardMapping, SparseDistantContradictionsHoldHazardsAndReliableSafeScansClear)
{
  Config config; config.max_slope_deg = 15; config.footprint_radius = 0;
  IncrementalGrid map; map.reset(config);
  auto steep = measurement(30), weak = measurement(0, 0.001), safe = measurement(0);
  map.update(steep, 1); ASSERT_EQ(rawAt(map.finish()), 100);
  for (int i = 2; i < 9; ++i) {
    map.update(weak, i); EXPECT_EQ(rawAt(map.finish()), 100);
    EXPECT_GT(map.rejectedHeights(), 0u);
    EXPECT_EQ(map.clearedHazards(), 0u);
  }
  map.update(safe, 10); EXPECT_EQ(rawAt(map.finish()), 100);
  for (int i = 11; i < 17; ++i) {map.update(safe, i); map.finish();}
  EXPECT_EQ(rawAt(map.finish()), 0);
  EXPECT_EQ(std::count(map.finish().costs.begin(), map.finish().costs.end(), 100), 0);
}

TEST(HazardMapping, MissingInconclusiveAndRepeatedDataNeverVoteToClear)
{
  Config config; config.max_slope_deg = 15;
  IncrementalGrid map; map.reset(config); map.update(measurement(30), 1); map.finish();
  for (int i = 2; i < 7; ++i) {
    map.update({}, i); EXPECT_EQ(rawAt(map.finish()), 100);
  }
  auto bad = measurement(0);
  for (auto & [key, cell] : bad) {cell.height = ((key.x+key.y)%2 ? .5 : -.5);}
  for (int i = 7; i < 13; ++i) {
    map.update(bad, i); EXPECT_EQ(rawAt(map.finish()), 100);
  }
  const auto safe = measurement(0);
  map.update(safe, 20); map.finish();
  const auto votes = map.state({0, 0})->candidate_votes;
  for (int i = 0; i < 8; ++i) {map.update(safe, 20); EXPECT_EQ(rawAt(map.finish()), 100);}
  EXPECT_EQ(map.state({0, 0})->candidate_votes, votes);
}

TEST(HazardMapping, IsolatedFreshCellsCannotClearCachedHazardNeighborhoods)
{
  Config config; config.max_slope_deg = 15;
  IncrementalGrid map; map.reset(config); map.update(measurement(30), 1); map.finish();
  auto safe = measurement(0);
  for (int i = 2; i < 15; ++i) {
    map.update({{{0, 0}, safe.at({0, 0})}}, i);
    EXPECT_EQ(rawAt(map.finish()), 100);
    EXPECT_EQ(map.state({0, 0})->clear_votes, 0);
  }
}

TEST(HazardMapping, HysteresisRetainsBorderlineSlopesUntilClearlySafe)
{
  Config config; config.max_slope_deg = 15;
  IncrementalGrid map; map.reset(config); map.update(measurement(20), 1); map.finish();
  for (int i = 2; i < 14; ++i) {
    map.update(measurement(14), i); EXPECT_EQ(rawAt(map.finish()), 100);
  }
  for (int i = 14; i < 25; ++i) {map.update(measurement(5), i); map.finish();}
  EXPECT_EQ(rawAt(map.finish()), 0);
}

TEST(HazardMapping, ConfidenceWeightsFusionAndDiscontinuitiesUseConfirmedModes)
{
  Config config;
  IncrementalGrid map; map.reset(config);
  auto initial = measurement(0), weak = measurement(0, 0.001);
  map.update(initial, 1); map.finish();
  weak.at({0, 0}).height += .03;
  map.update(weak, 2); map.finish();
  EXPECT_LT(map.state({0, 0})->estimate.height-initial.at({0, 0}).height, .002);
  auto moved = initial; moved.at({0, 0}).height += .5;
  map.update(moved, 3); map.finish();
  EXPECT_LT(map.state({0, 0})->estimate.height, 0);
  map.update(moved, 4); map.finish();
  EXPECT_LT(map.state({0, 0})->estimate.height, 0);
  map.update(moved, 5); map.finish();
  EXPECT_NEAR(map.state({0, 0})->estimate.height, .2, 1e-9);
}

TEST(HazardMapping, SparseSamplesUseMedianAndRangeIncreasesUncertainty)
{
  Config config;
  const auto near = project({{.025, .025, -.4}, {.025, .025, -.2}},
    Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config);
  EXPECT_NEAR(near.at({0, 0}).height, -.3, 1e-9);
  const auto a = project({{.025, .025, -.3}}, Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config);
  const auto b = project({{10.025, .025, -.3}}, Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config);
  EXPECT_GT(b.at({200, 0}).variance, a.at({0, 0}).variance);
  config.clear_confirmations = 0; EXPECT_THROW(config.validate(), std::invalid_argument);
  config.clear_confirmations = 3; config.clear_slope_margin = config.max_slope_deg;
  EXPECT_THROW(config.validate(), std::invalid_argument);
}

TEST(HazardMapping, NumerousClusteredPointsCannotEstablishAPlane)
{
  Config config;
  Contribution cluster;
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 2; ++y) {
      cluster[{x, y}] = Cell{-.3, Eigen::Vector3d::UnitZ(), 100, .000025};
    }
  }
  IncrementalGrid map; map.reset(config); map.update(cluster, 1);
  EXPECT_EQ(rawAt(map.finish()), -1);
}

TEST(HazardMapping, AcquisitionRoundoffDoesNotRefitStationaryTerrain)
{
  Config config;
  auto flat = measurement(0);
  IncrementalGrid map; map.reset(config); map.update(flat, 1); map.finish();
  for (auto & [key, cell] : flat) {cell.height += 3e-10;}
  map.update(flat, 2); map.finish();
  EXPECT_EQ(map.changedCells(), 0u);
  EXPECT_EQ(map.fittedCells(), 0u);
}

TEST(HazardMapping, UnknownPlanningCostPreservesEvidenceAndDoesNotSeedInflation)
{
  Config config; config.footprint_radius = 0;
  for (const int cost : {-1, 0, 50, 99, 100}) {
    config.unknown_cost = cost;
    Grid grid; grid.resolution = .05; grid.width = 3; grid.height = 1;
    grid.raw = {-1, 0, -1};
    inflate(grid, config);
    EXPECT_EQ(grid.raw, (std::vector<int8_t>{-1, 0, -1}));
    EXPECT_EQ(grid.costs, (std::vector<int8_t>{static_cast<int8_t>(cost), 0,
      static_cast<int8_t>(cost)}));
    const auto empty = rasterize({}, config);
    EXPECT_TRUE(std::all_of(empty.raw.begin(), empty.raw.end(),
      [](auto value) {return value == -1;}));
    EXPECT_TRUE(std::all_of(empty.costs.begin(), empty.costs.end(),
      [cost](auto value) {return value == cost;}));
  }
  config.unknown_cost = -2; EXPECT_THROW(config.validate(), std::invalid_argument);
  config.unknown_cost = 101; EXPECT_THROW(config.validate(), std::invalid_argument);
}

TEST(HazardMapping, UnknownCostCombinesWithInflationAndSurvivesIncrementalClearing)
{
  Config config; config.footprint_radius = 0; config.unknown_cost = 50;
  IncrementalGrid map; map.reset(config);
  map.update(measurement(30), 1);
  const auto hazardous = map.finish();
  bool hard_unknown = false, soft_unknown = false;
  for (std::size_t i = 0; i < hazardous.raw.size(); ++i) {
    if (hazardous.raw[i] < 0) {
      EXPECT_GE(hazardous.costs[i], 50);
      hard_unknown |= hazardous.costs[i] == 100;
      soft_unknown |= hazardous.costs[i] > 50 && hazardous.costs[i] < 100;
    }
  }
  EXPECT_TRUE(hard_unknown); EXPECT_TRUE(soft_unknown);
  for (int stamp = 2; stamp <= 10; ++stamp) {
    map.update(measurement(0), stamp);
    const auto actual = map.finish(); auto full = actual; inflate(full, config);
    EXPECT_EQ(actual.costs, full.costs);
  }
  const auto cleared = map.finish();
  ASSERT_EQ(rawAt(cleared), 0);
  for (std::size_t i = 0; i < cleared.raw.size(); ++i) {
    if (cleared.raw[i] < 0) {EXPECT_EQ(cleared.costs[i], 50);}
  }
}
