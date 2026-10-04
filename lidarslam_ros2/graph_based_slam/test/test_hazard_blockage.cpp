#include "hazard/hazard_mapping.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>

using namespace graphslam::hazard;

namespace
{
constexpr Key target_cell {20, 0};
constexpr VoxelKey target_voxel {20, 0, 3};

Config testConfig()
{
  Config config;
  config.footprint_radius = 0.0;
  config.max_slope_deg = 15.0;
  return config;
}

void seedGround(IncrementalGrid & map, const Config & config, double slope = 0.0,
  bool leave_target_unknown = false)
{
  std::vector<Eigen::Vector3d> points;
  for (int y = -20; y <= 20; ++y) {
    for (int x = -20; x <= 60; ++x) {
      if (leave_target_unknown && x == target_cell.x && y == target_cell.y) {continue;}
      const double px = (x + 0.5) * config.resolution;
      const double py = (y + 0.5) * config.resolution;
      // The slope runs across the ray, rather than up its direction of travel.
      points.emplace_back(px, py, std::tan(slope * M_PI / 180.0) * py);
    }
  }
  map.reset(config);
  map.update(project(points, Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(), config), 1);
  map.finish();
}

CloudObservation obstacle(double height = 0.35)
{
  CloudObservation scan;
  scan.sensor_origin = {0.025, 0.025, height};
  scan.clear_rays = true;
  // Distinct returns meet the density gate without crossing a voxel boundary.
  scan.body_points = {{1.021, 0.022, height - 0.004},
    {1.025, 0.025, height}, {1.029, 0.028, height + 0.004}};
  return scan;
}

CloudObservation clearRay(double height = 0.35, double endpoint_x = 2.025)
{
  CloudObservation scan;
  scan.sensor_origin = {0.025, 0.025, height};
  scan.clear_rays = true;
  scan.body_points = {{endpoint_x - 0.004, 0.023, height},
    {endpoint_x, 0.025, height}, {endpoint_x + 0.004, 0.027, height}};
  return scan;
}

int rawAt(const Grid & grid, Key key)
{
  const int x = key.x - grid.origin_x, y = key.y - grid.origin_y;
  if (x < 0 || y < 0 || x >= static_cast<int>(grid.width) ||
    y >= static_cast<int>(grid.height)) {return -1;}
  return grid.raw[static_cast<std::size_t>(y) * grid.width + x];
}

void observe(IncrementalGrid & map, const CloudObservation & scan, int64_t stamp,
  bool fresh = true)
{
  map.observe({scan}, stamp, fresh);
  map.finish();
}

int batchesToClear(IncrementalGrid & map, int64_t first_stamp)
{
  for (int batch = 1; batch <= 10; ++batch) {
    observe(map, clearRay(), first_stamp + batch);
    if (rawAt(map.finish(), target_cell) != 100) {return batch;}
  }
  return 11;
}
}  // namespace

TEST(HazardBlockage, SaturatedLongStandingObstacleHasBoundedClearingDebt)
{
  const auto config = testConfig();
  IncrementalGrid short_dwell, long_dwell;
  seedGround(short_dwell, config); seedGround(long_dwell, config);
  observe(short_dwell, obstacle(), 2);
  for (int64_t stamp = 2; stamp <= 1001; ++stamp) {observe(long_dwell, obstacle(), stamp);}
  ASSERT_EQ(rawAt(short_dwell.finish(), target_cell), 100);
  ASSERT_EQ(rawAt(long_dwell.finish(), target_cell), 100);
  const int short_batches = batchesToClear(short_dwell, 2000);
  const int long_batches = batchesToClear(long_dwell, 2000);
  EXPECT_EQ(short_batches, 3);
  EXPECT_EQ(long_batches, short_batches);
  EXPECT_EQ(rawAt(long_dwell.finish(), target_cell), 0);
}

TEST(HazardBlockage, MissingOverheadAndOccludedObservationsDoNotClear)
{
  IncrementalGrid map; seedGround(map, testConfig());
  observe(map, obstacle(), 2);
  for (int64_t stamp = 3; stamp <= 8; ++stamp) {
    observe(map, clearRay(0.55), stamp);  // A beam above the occupied volume.
    EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  }
  for (int64_t stamp = 9; stamp <= 14; ++stamp) {
    observe(map, clearRay(0.35, 0.825), stamp);  // Something nearer hides the target.
    EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  }
  map.observe({}, 10000); map.finish();
  EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  EXPECT_EQ(batchesToClear(map, 10001), 3);
}

TEST(HazardBlockage, AReturnAtTheObstacleIsNotFreeSpace)
{
  IncrementalGrid map; seedGround(map, testConfig());
  observe(map, obstacle(), 2);
  // Even when too sparse to add occupied evidence, the endpoint is not a clear ray.
  auto endpoint = clearRay(0.35, 1.025); endpoint.body_points.resize(1);
  for (int64_t stamp = 3; stamp <= 9; ++stamp) {
    observe(map, endpoint, stamp);
    EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  }
  EXPECT_EQ(batchesToClear(map, 10), 3);
}

TEST(HazardBlockage, ClearingOneHeightCannotClearAnotherOccupiedHeight)
{
  IncrementalGrid map; seedGround(map, testConfig());
  auto scan = obstacle(0.25);
  const auto upper = obstacle(0.45);
  scan.body_points.insert(scan.body_points.end(), upper.body_points.begin(), upper.body_points.end());
  observe(map, scan, 2);
  ASSERT_EQ(rawAt(map.finish(), target_cell), 100);
  for (int64_t stamp = 3; stamp <= 6; ++stamp) {observe(map, clearRay(0.25), stamp);}
  EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  for (int64_t stamp = 7; stamp <= 10; ++stamp) {observe(map, clearRay(0.45), stamp);}
  EXPECT_EQ(rawAt(map.finish(), target_cell), 0);
}

TEST(HazardBlockage, DuplicateHistoricalAndUntrustedRaysNeverAccumulateClearance)
{
  IncrementalGrid map; seedGround(map, testConfig());
  observe(map, obstacle(), 10);
  for (int i = 0; i < 8; ++i) {observe(map, clearRay(), 11);}
  EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  for (int64_t stamp = 12; stamp <= 18; ++stamp) {observe(map, clearRay(), stamp, false);}
  EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  auto untrusted = clearRay(); untrusted.clear_rays = false;
  for (int64_t stamp = 19; stamp <= 25; ++stamp) {observe(map, untrusted, stamp);}
  EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  // Only the first fresh ray counted; two more qualified observations finish clearing.
  observe(map, clearRay(), 26); EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  observe(map, clearRay(), 27); EXPECT_EQ(rawAt(map.finish(), target_cell), 0);
}

TEST(HazardBlockage, OccupiedReturnsVetoClearingWithinTheSameBatch)
{
  IncrementalGrid map; seedGround(map, testConfig());
  observe(map, obstacle(), 2);
  for (int64_t stamp = 3; stamp <= 10; ++stamp) {
    // A second scan passing through the cell cannot overrule actual returns there.
    map.observe({clearRay(), obstacle()}, stamp); map.finish();
    EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  }
  EXPECT_EQ(batchesToClear(map, 11), 3);
}

TEST(HazardBlockage, ManyPointsAndScansContributeOnlyOneBoundedHitPerBatch)
{
  IncrementalGrid one_scan, many_scans;
  seedGround(one_scan, testConfig()); seedGround(many_scans, testConfig());
  observe(one_scan, obstacle(), 2);
  std::vector<CloudObservation> scans(50, obstacle());
  many_scans.observe(scans, 2); many_scans.finish();
  const auto one = one_scan.obstacles().find(target_voxel);
  const auto many = many_scans.obstacles().find(target_voxel);
  ASSERT_NE(one, one_scan.obstacles().end());
  ASSERT_NE(many, many_scans.obstacles().end());
  EXPECT_NEAR(one->second.log_odds, many->second.log_odds, 1e-12);
  EXPECT_EQ(batchesToClear(one_scan, 3), batchesToClear(many_scans, 3));
}

TEST(HazardBlockage, SparseAboveGroundReturnsCannotMarkOrReplaceStrongTerrain)
{
  IncrementalGrid map; seedGround(map, testConfig());
  auto sparse = obstacle(); sparse.body_points.resize(1);
  for (int64_t stamp = 2; stamp <= 12; ++stamp) {observe(map, sparse, stamp);}
  EXPECT_EQ(rawAt(map.finish(), target_cell), 0);
  ASSERT_NE(map.state(target_cell), nullptr);
  EXPECT_NEAR(map.state(target_cell)->estimate.height, 0.0, 1e-9);
}

TEST(HazardBlockage, NegativeWorldCoordinatesUseTheSameVoxelClearingRules)
{
  IncrementalGrid map; seedGround(map, testConfig());
  auto target = obstacle();
  target.sensor_origin = {0.025, -0.025, 0.35};
  for (auto & point : target.body_points) {point.x() -= 1.55; point.y() -= 0.05;}
  const Key negative {-11, -1};
  observe(map, target, 2);
  ASSERT_EQ(rawAt(map.finish(), negative), 100);
  auto ray = clearRay();
  ray.sensor_origin = target.sensor_origin;
  ray.body_points = {{-0.929, -0.023, 0.35},
    {-0.925, -0.025, 0.35}, {-0.921, -0.027, 0.35}};
  for (int64_t stamp = 3; stamp <= 5; ++stamp) {observe(map, ray, stamp);}
  EXPECT_EQ(rawAt(map.finish(), negative), 0);
}

TEST(HazardBlockage, APersonAboveGroundDoesNotReplaceTheGroundEstimate)
{
  IncrementalGrid map; seedGround(map, testConfig());
  auto scan = obstacle();
  scan.body_points.emplace_back(1.025, 0.025, 0.0);
  for (int64_t stamp = 2; stamp <= 12; ++stamp) {observe(map, scan, stamp);}
  ASSERT_NE(map.state(target_cell), nullptr);
  EXPECT_NEAR(map.state(target_cell)->estimate.height, 0.0, 1e-9);
  EXPECT_EQ(map.state(target_cell)->raw, 0);
  EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  EXPECT_EQ(batchesToClear(map, 20), 3);
  EXPECT_EQ(rawAt(map.finish(), target_cell), 0);
}

TEST(HazardBlockage, ClearingTemporaryBlockageCannotClearAnUnsafeSlope)
{
  IncrementalGrid map; seedGround(map, testConfig(), 30.0);
  ASSERT_NE(map.state(target_cell), nullptr);
  ASSERT_EQ(map.state(target_cell)->raw, 100);
  observe(map, obstacle(), 2);
  ASSERT_NE(map.obstacles().find(target_voxel), map.obstacles().end());
  for (int64_t stamp = 3; stamp <= 8; ++stamp) {observe(map, clearRay(), stamp);}
  const auto found = map.obstacles().find(target_voxel);
  EXPECT_TRUE(found == map.obstacles().end() || !found->second.lethal);
  EXPECT_EQ(map.state(target_cell)->raw, 100);
  EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
}

TEST(HazardBlockage, CorrectedAcquisitionPoseTransformsReturnsAndRayOriginTogether)
{
  IncrementalGrid map; seedGround(map, testConfig());
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = (Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitZ())).toRotationMatrix();
  pose.translation() = Eigen::Vector3d(0.25, -0.10, 0.15);
  const auto inBody = [&pose](CloudObservation scan) {
      scan.pose = pose;
      scan.up_in_body = pose.linear().transpose() * Eigen::Vector3d::UnitZ();
      scan.sensor_origin = pose.inverse() * scan.sensor_origin;
      for (auto & point : scan.body_points) {point = pose.inverse() * point;}
      return scan;
    };
  observe(map, inBody(obstacle()), 2);
  EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  for (int64_t stamp = 3; stamp <= 5; ++stamp) {observe(map, inBody(clearRay()), stamp);}
  EXPECT_EQ(rawAt(map.finish(), target_cell), 0);
}

TEST(HazardBlockage, UnsupportedPositiveHeightReturnsCannotBootstrapSafeTerrain)
{
  IncrementalGrid map; map.reset(testConfig());
  for (int64_t stamp = 1; stamp <= 8; ++stamp) {observe(map, obstacle(), stamp);}
  EXPECT_TRUE(map.obstacles().empty());
  EXPECT_TRUE(std::all_of(map.finish().raw.begin(), map.finish().raw.end(),
    [](int8_t value) {return value == -1;}));
}

TEST(HazardBlockage, SurroundingTrustedPlaneSupportsBlockageOverAnUnmeasuredCell)
{
  IncrementalGrid map; seedGround(map, testConfig(), 0.0, true);
  ASSERT_EQ(rawAt(map.finish(), target_cell), -1);
  observe(map, obstacle(), 2);
  ASSERT_EQ(rawAt(map.finish(), target_cell), 100);
  EXPECT_EQ(batchesToClear(map, 3), 3);
  // Clearance of the volume is not a direct measurement of the ground cell.
  EXPECT_EQ(rawAt(map.finish(), target_cell), -1);
}

TEST(HazardBlockage, DeferredVisibilityClearsWithoutRecreatingObjectsOrChangingTerrain)
{
  IncrementalGrid map; seedGround(map, testConfig());
  observe(map, obstacle(), 2);
  ASSERT_EQ(rawAt(map.finish(), target_cell), 100);
  // The current map batch has no rays; an older acquisition has acquired a
  // trustworthy sensor transform and can now contribute visibility only.
  for (int batch = 0; batch < 3; ++batch) {
    map.observe({}, 100 + batch);
    map.observe({clearRay()}, 10 + batch, true, false);
    map.finish();
  }
  ASSERT_EQ(rawAt(map.finish(), target_cell), 0);
  auto old_endpoints = obstacle();
  for (int i = 0; i < 8; ++i) {
    old_endpoints.body_points.emplace_back(1.025, 0.025, 0.04);
  }
  for (int64_t stamp = 13; stamp <= 18; ++stamp) {
    map.observe({}, 100 + stamp);
    map.observe({old_endpoints}, stamp, true, false); map.finish();
    EXPECT_EQ(rawAt(map.finish(), target_cell), 0);
    ASSERT_NE(map.state(target_cell), nullptr);
    EXPECT_NEAR(map.state(target_cell)->estimate.height, 0.0, 1e-9);
  }
  // Deferring visibility does not suppress genuinely fresh obstacle marking.
  observe(map, obstacle(), 200);
  EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
}

TEST(HazardBlockage, NewerSparseOccupiedReturnsVetoOlderDeferredClearance)
{
  IncrementalGrid map; seedGround(map, testConfig());
  observe(map, obstacle(), 2);
  observe(map, clearRay(), 3); observe(map, clearRay(), 4);
  ASSERT_EQ(rawAt(map.finish(), target_cell), 100);
  auto sparse = obstacle(); sparse.body_points.resize(1);
  // There are too few occupied returns to raise confidence, but they are still
  // newer evidence that the old free-space acquisition must not overrule.
  map.observe({sparse}, 20);
  map.observe({clearRay()}, 19, true, false); map.finish();
  EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  observe(map, clearRay(), 21);
  EXPECT_EQ(rawAt(map.finish(), target_cell), 0);
}

TEST(HazardBlockage, LatestAndDeferredObservationsShareOneRayBudget)
{
  auto config = testConfig(); config.max_rays = 5;
  IncrementalGrid map; seedGround(map, config);
  observe(map, obstacle(), 2);
  map.observe({clearRay()}, 12);
  ASSERT_EQ(map.tracedRays(), 3u);
  map.observe({clearRay()}, 11, true, false);
  EXPECT_EQ(map.tracedRays(), 5u);
  map.observe({clearRay()}, 10, true, false);
  EXPECT_EQ(map.tracedRays(), 5u);
  EXPECT_EQ(map.skippedRays(), 4u);
  map.finish();
  EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  // A completed cycle starts a fresh bounded budget, rather than exhausting
  // ray tracing permanently or giving every deferred scan another full budget.
  map.observe({clearRay()}, 13);
  EXPECT_EQ(map.tracedRays(), 3u);
  map.finish();
  EXPECT_EQ(rawAt(map.finish(), target_cell), 100);
  observe(map, clearRay(), 14);
  EXPECT_EQ(rawAt(map.finish(), target_cell), 0);
}

TEST(HazardBlockage, ClearingRestoredBlockageLeavesUnobservedTerrainUnknown)
{
  IncrementalGrid map; map.reset(testConfig());
  const auto config = testConfig();
  ObstacleState retained;
  retained.log_odds = std::log(config.obstacle_max_probability /
    (1.0 - config.obstacle_max_probability));
  retained.lethal = true; retained.stamp = 1;
  map.restoreObstacle(target_voxel, retained); map.finish();
  ASSERT_EQ(rawAt(map.finish(), target_cell), 100);
  for (int64_t stamp = 2; stamp <= 4; ++stamp) {observe(map, clearRay(), stamp);}
  EXPECT_EQ(rawAt(map.finish(), target_cell), -1);
}
