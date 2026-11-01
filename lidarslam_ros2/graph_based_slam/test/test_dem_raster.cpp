// The incremental DEM (dem_raster.hpp) against an independent one-shot reference: a synthetic terrain with
// rocks and holes, the points fed in several chunks (some chunks extend the bounds), compared cell for cell.
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <random>
#include <vector>

#include "graph_based_slam/dem_raster.hpp"

namespace
{

struct Reference
{
  double origin_x, origin_y;
  int width, height;
  std::vector<float> terrain, surface, variance, min_z, max_z;
  std::vector<uint32_t> count;
  std::vector<uint8_t> valid;
};

// The reference: bin every point by floor(x / res), floor(y / res); terrain = sorted heights[floor(p * (n - 1))].
Reference oneShot(
  const pcl::PointCloud<pcl::PointXYZI> & cloud, double res, int min_points, double percentile)
{
  std::map<std::pair<int, int>, std::vector<float>> bins;
  int min_ix = INT32_MAX, min_iy = INT32_MAX, max_ix = INT32_MIN, max_iy = INT32_MIN;
  for (const auto & p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    const int ix = static_cast<int>(std::floor(p.x / res));
    const int iy = static_cast<int>(std::floor(p.y / res));
    bins[{ix, iy}].push_back(p.z);
    min_ix = std::min(min_ix, ix); max_ix = std::max(max_ix, ix);
    min_iy = std::min(min_iy, iy); max_iy = std::max(max_iy, iy);
  }
  Reference r;
  r.origin_x = min_ix * res;
  r.origin_y = min_iy * res;
  r.width = max_ix - min_ix + 1;
  r.height = max_iy - min_iy + 1;
  const std::size_t n = static_cast<std::size_t>(r.width) * r.height;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  r.terrain.assign(n, nan); r.surface.assign(n, nan); r.variance.assign(n, nan);
  r.min_z.assign(n, nan); r.max_z.assign(n, nan); r.count.assign(n, 0U); r.valid.assign(n, 0U);
  for (auto & [key, zs] : bins) {
    const std::size_t idx = static_cast<std::size_t>(key.second - min_iy) * r.width + (key.first - min_ix);
    r.count[idx] = static_cast<uint32_t>(zs.size());
    if (static_cast<int>(zs.size()) < min_points) {continue;}
    std::sort(zs.begin(), zs.end());
    const std::size_t q = static_cast<std::size_t>(std::floor(percentile * (zs.size() - 1)));
    r.terrain[idx] = zs[q];
    r.min_z[idx] = zs.front();
    r.max_z[idx] = zs.back();
    r.surface[idx] = zs.back();
    double mean = 0.0;
    for (float z : zs) {mean += z;}
    mean /= zs.size();
    double ss = 0.0;
    for (float z : zs) {ss += (z - mean) * (z - mean);}
    r.variance[idx] = static_cast<float>(ss / zs.size());
    r.valid[idx] = 1U;
  }
  return r;
}

pcl::PointCloud<pcl::PointXYZI> syntheticChunk(std::mt19937 & rng, double cx, double cy, double radius, int n)
{
  std::uniform_real_distribution<double> u(-radius, radius);
  std::normal_distribution<double> noise(0.0, 0.02);
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int i = 0; i < n; ++i) {
    pcl::PointXYZI p;
    p.x = static_cast<float>(cx + u(rng));
    p.y = static_cast<float>(cy + u(rng));
    // a gentle slope, a crater, a few rocks, and noise
    double z = 0.05 * p.x - 0.03 * p.y + 0.5 * std::exp(-((p.x - 3) * (p.x - 3) + (p.y + 2) * (p.y + 2)) / 4.0);
    if (std::fmod(std::abs(p.x * 7.3 + p.y * 3.1), 5.0) < 0.3) {z += 0.4;}
    p.z = static_cast<float>(z + noise(rng));
    p.intensity = 1.0F;
    cloud.push_back(p);
  }
  return cloud;
}

void expectSame(const graphslam::DemGrid & grid, const Reference & ref)
{
  ASSERT_EQ(grid.width, ref.width);
  ASSERT_EQ(grid.height, ref.height);
  EXPECT_DOUBLE_EQ(grid.origin_x, ref.origin_x);
  EXPECT_DOUBLE_EQ(grid.origin_y, ref.origin_y);
  const std::size_t n = static_cast<std::size_t>(ref.width) * ref.height;
  std::size_t valid = 0;
  for (std::size_t i = 0; i < n; ++i) {
    ASSERT_EQ(grid.count[i], ref.count[i]) << "cell " << i;
    ASSERT_EQ(grid.valid[i], ref.valid[i]) << "cell " << i;
    if (!ref.valid[i]) {continue;}
    ++valid;
    EXPECT_FLOAT_EQ(grid.terrain[i], ref.terrain[i]) << "cell " << i;
    EXPECT_FLOAT_EQ(grid.surface[i], ref.surface[i]) << "cell " << i;
    EXPECT_FLOAT_EQ(grid.min_z[i], ref.min_z[i]) << "cell " << i;
    EXPECT_FLOAT_EQ(grid.max_z[i], ref.max_z[i]) << "cell " << i;
    EXPECT_NEAR(grid.variance[i], ref.variance[i], 1e-6F) << "cell " << i;
  }
  EXPECT_GT(valid, 100U);
}

}  // namespace

TEST(DemRaster, ChunkedAccumulationMatchesOneShot)
{
  std::mt19937 rng(42);
  const double res = 0.2;
  const int min_points = 3;
  const double percentile = 0.25;
  graphslam::DemCells cells;
  pcl::PointCloud<pcl::PointXYZI> everything;
  // five chunks around a path that extends the bounds to the north-east, then back south-west
  const double centres[5][2] = {{0, 0}, {4, 1}, {8, 3}, {-3, -2}, {2, -6}};
  for (int c = 0; c < 5; ++c) {
    auto chunk = syntheticChunk(rng, centres[c][0], centres[c][1], 4.0, 20000);
    everything += chunk;
    EXPECT_EQ(graphslam::accumulateDemPoints(cells, chunk, res), chunk.size());
    const std::size_t recomputed = graphslam::refreshDemCells(cells, min_points, percentile);
    EXPECT_GT(recomputed, 0U);
    graphslam::DemGrid grid;
    std::string error;
    ASSERT_TRUE(graphslam::rasterizeDemCells(cells, res, 4000000, grid, error)) << error;
    expectSame(grid, oneShot(everything, res, min_points, percentile));
  }
  // a second refresh recomputes nothing and changes nothing
  EXPECT_EQ(graphslam::refreshDemCells(cells, min_points, percentile), 0U);
  graphslam::DemGrid grid;
  std::string error;
  ASSERT_TRUE(graphslam::rasterizeDemCells(cells, res, 4000000, grid, error)) << error;
  expectSame(grid, oneShot(everything, res, min_points, percentile));
}

TEST(DemRaster, NonFinitePointsAreSkippedAndSparseCellsStayInvalid)
{
  graphslam::DemCells cells;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI p;
  p.x = 0.1F; p.y = 0.1F; p.z = 1.0F; cloud.push_back(p);
  p.z = 2.0F; cloud.push_back(p);
  p.x = std::numeric_limits<float>::quiet_NaN(); cloud.push_back(p);
  EXPECT_EQ(graphslam::accumulateDemPoints(cells, cloud, 0.5), 2U);
  EXPECT_EQ(graphslam::refreshDemCells(cells, 3, 0.25), 1U);
  graphslam::DemGrid grid;
  std::string error;
  ASSERT_TRUE(graphslam::rasterizeDemCells(cells, 0.5, 100, grid, error)) << error;
  ASSERT_EQ(grid.width, 1);
  ASSERT_EQ(grid.height, 1);
  EXPECT_EQ(grid.count[0], 2U);
  EXPECT_EQ(grid.valid[0], 0U);
  EXPECT_TRUE(std::isnan(grid.terrain[0]));
}

TEST(DemRaster, RasterLimitIsEnforced)
{
  graphslam::DemCells cells;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI p;
  p.x = 0.0F; p.y = 0.0F; p.z = 0.0F; cloud.push_back(p);
  p.x = 100.0F; p.y = 100.0F; cloud.push_back(p);
  graphslam::accumulateDemPoints(cells, cloud, 0.1);
  graphslam::refreshDemCells(cells, 1, 0.25);
  graphslam::DemGrid grid;
  std::string error;
  EXPECT_FALSE(graphslam::rasterizeDemCells(cells, 0.1, 1000, grid, error));
  EXPECT_FALSE(error.empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
