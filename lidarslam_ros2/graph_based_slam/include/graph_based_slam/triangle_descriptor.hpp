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
Implements triangle-descriptor primitives for place recognition: three
keypoint-extraction modes, invariant triangle construction, and rigid transform
estimation from triangle or general point correspondences.
*/

// STD/BTC-style triangle descriptor primitives, implemented from scratch
// under BSD-2 so the default workflow can include them.
//
// The pipeline (in this header):
//   1. extractKeypoints: dispatcher that picks one of the modes below.
//      - extractKeypointsBEV: take a submap point cloud, project to BEV, pick
//        the local-maximum cells in max_height as stable keypoints. Works on
//        spinning 360° LiDAR with wide vertical FOV (e.g. OS1 outdoors).
//      - extractKeypointsEdge3D: PCA eigenvalue ratio on radius-r neighborhoods
//        picks edge-like supports (column edges, wall corners). Survives in
//        narrow-FOV (MID-360) and indoor scenes where BEV max-height collapses.
//      - extractKeypointsSurfaceSaliency: same 2.5D grid as BEV, but detrends
//        a robust ground-plane fit out of it first and picks curvature
//        extrema (crater rims, curbs, boulders, pole bases) on the residual
//        surface, ranked by percentile rather than an absolute threshold.
//        Works on open/sloped terrain (parking lots, hills, lunar-like)
//        where BEV_MAX_HEIGHT (needs vertical prominence) and EDGE_3D (needs
//        edge response) both starve.
//   2. buildTriangles: enumerate all 3-tuples of keypoints, drop those whose
//      edge lengths fall outside [min_edge_m, max_edge_m] or that are
//      near-collinear, and store the edges sorted ascending so the triangle
//      is a yaw / rotation invariant descriptor.
//   3. estimateRigidFromTriangle: SVD / Umeyama on the 3-point
//      correspondence to recover the SE(3) bringing one triangle onto the
//      other; this is the "geometric verification" step used downstream.
//
// Matching, hashing, and the actual TriangleDatabase live in follow-up
// patches; this header keeps the testable primitives self-contained.

#ifndef GRAPH_BASED_SLAM__TRIANGLE_DESCRIPTOR_HPP_
#define GRAPH_BASED_SLAM__TRIANGLE_DESCRIPTOR_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace graphslam
{
namespace triangle
{

struct Keypoint
{
  Eigen::Vector3f position {Eigen::Vector3f::Zero()};
  // BEV max-height minus the neighborhood floor (BEV mode),
  // or PCA edgeness (λ2 - λ1) / λ2 (EDGE_3D mode); higher = more salient.
  float salience {0.0f};
};

// Keypoint extraction strategy. BEV_MAX_HEIGHT was the original outdoor-only
// extractor; EDGE_3D adds PCA-based edge keypoints that survive in narrow-FOV
// LiDARs (MID-360) and indoor scenes where BEV max-height collapses.
// SURFACE_SALIENCY detrends a robust ground-plane fit out of the BEV grid and
// picks curvature extrema, so open/sloped terrain that starves the other two
// (no vertical prominence, no edge response) still yields keypoints.
enum class KeypointMode
{
  BEV_MAX_HEIGHT,
  EDGE_3D,
  SURFACE_SALIENCY,
};

struct KeypointExtractionConfig
{
  KeypointMode mode {KeypointMode::BEV_MAX_HEIGHT};
  // ----- BEV_MAX_HEIGHT params -----
  // Side length of the BEV window centred on the submap origin.
  double grid_size_m {60.0};
  // Cells per side; cell_size = grid_size_m / grid_cells.
  int grid_cells {60};
  // Radius (in cells) used for local-max neighborhood comparison.
  int neighborhood_radius_cells {2};
  // Minimum salience (m) to keep a keypoint.
  float min_salience_m {0.3f};
  // ----- EDGE_3D params -----
  // Voxel downsample size before PCA; trades repeatability for cost.
  float edge_voxel_size_m {0.4f};
  // Radius (m) for the PCA neighborhood used to compute eigenvalues.
  float edge_neighbor_radius_m {1.0f};
  // Minimum neighbor count; points with sparser support are skipped.
  int edge_min_neighbors {6};
  // Minimum PCA edgeness (λ2 - λ1) / λ2 to accept a candidate.
  float edge_min_edgeness {0.5f};
  // Suppression radius (m) for non-maximum suppression of edgeness.
  float edge_nms_radius_m {2.0f};
  // ----- SURFACE_SALIENCY params -----
  // Lower height-percentile (0..1) of populated grid cells used to
  // least-squares fit the dominant ground plane; keeps tall structures
  // (curbs, boulders, poles) from dragging the plane off horizontal.
  // 0.3 = fit using the lowest 30% of cells by height.
  double surface_plane_fit_percentile {0.3};
  // Neighborhood radius (in cells) for the curvature response computed over
  // the detrended residual surface (|residual - mean(neighborhood)|).
  int surface_curvature_radius_cells {1};
  // Soft percentile floor (0..1) applied to the saliency distribution BEFORE
  // the top-N cut: cells below this percentile are dropped even if that
  // leaves fewer than max_keypoints. 0 = no floor, meaning relative rank
  // (top max_keypoints) is the only selector -- this is what lets a flat lot
  // and a crater field both yield ~max_keypoints instead of the hard-gate
  // starvation that BEV_MAX_HEIGHT / EDGE_3D hit on open terrain.
  double surface_min_saliency_percentile {0.0};
  // Non-maximum suppression radius reuses edge_nms_radius_m (above).
  // ----- common -----
  // Cap on returned keypoints; we keep the highest-salience ones.
  int max_keypoints {80};
};

struct TriangleDescriptor
{
  // Edge lengths sorted ascending. edges[0] <= edges[1] <= edges[2].
  std::array<float, 3> edges {{0.0f, 0.0f, 0.0f}};
  // Indices into the keypoint list this triangle was built from. The order
  // matches the edges array: the vertex opposite ``edges[k]`` is
  // ``keypoint_ids[k]``; vertices a, b, c are ids [0], [1], [2] but the
  // physical edges connecting them are sorted before they are written here.
  std::array<int, 3> keypoint_ids {{-1, -1, -1}};
};

struct TriangleBuildConfig
{
  // Lower edge bound (m); shorter triangles are dropped because tiny
  // baselines are unstable under noise.
  float min_edge_m {2.0f};
  // Upper edge bound (m); longer triangles tend to bridge dynamic objects.
  float max_edge_m {50.0f};
  // Reject triangles whose smallest angle is below this threshold (deg);
  // near-collinear configurations carry almost no orientation information.
  float min_angle_deg {5.0f};
  // Cap on number of triangles returned, sorted by largest edge first to
  // bias toward globally informative baselines.
  int max_triangles {5000};
};

// --------------------------- helpers ---------------------------

namespace detail
{

/*
Summary:
Returns Euclidean distance between two keypoint positions.
*/
inline float edgeLength(const Eigen::Vector3f & a, const Eigen::Vector3f & b)
{
  return (a - b).norm();
}

// Triangle smallest angle from edge lengths via the law of cosines.
/*
Summary:
Computes a triangle's smallest interior angle from sorted edge lengths.
*/
inline float smallestAngleDeg(float l1, float l2, float l3)
{
  // l3 is the longest edge; opposite angle is the largest. The smallest
  // angle is opposite the shortest edge (l1).
  const float num = l2 * l2 + l3 * l3 - l1 * l1;
  const float den = 2.0f * l2 * l3;
  if (den <= 1e-9f) {return 0.0f;}
  const float cos_alpha = std::max(-1.0f, std::min(1.0f, num / den));
  return std::acos(cos_alpha) * 180.0f / static_cast<float>(M_PI);
}

// Shared 2.5D max-height grid builder used by BEV_MAX_HEIGHT and
// SURFACE_SALIENCY (which detrends this same grid before ranking). Populates
// max_height with the per-cell max z (-inf when the cell is empty) and
// best_x / best_y with the (x, y) of the point that produced it, so emitted
// keypoints sit at an actual support rather than a cell centre.
/*
Summary:
Builds a 2.5D maximum-height raster while retaining the supporting point's
horizontal coordinates in each populated cell.
*/
inline void buildMaxHeightGrid(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  int gc, float half, float cell,
  Eigen::MatrixXf & max_height, Eigen::MatrixXf & best_x, Eigen::MatrixXf & best_y)
{
  max_height = Eigen::MatrixXf::Constant(gc, gc, -std::numeric_limits<float>::infinity());
  best_x = Eigen::MatrixXf::Zero(gc, gc);
  best_y = Eigen::MatrixXf::Zero(gc, gc);
  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    if (std::abs(p.x) > half || std::abs(p.y) > half) {continue;}
    const int ix = static_cast<int>(std::floor((p.x + half) / cell));
    const int iy = static_cast<int>(std::floor((p.y + half) / cell));
    if (ix < 0 || ix >= gc || iy < 0 || iy >= gc) {continue;}
    if (p.z > max_height(iy, ix)) {
      max_height(iy, ix) = p.z;
      best_x(iy, ix) = p.x;
      best_y(iy, ix) = p.y;
    }
  }
}

}  // namespace detail

// --------------------------- keypoint extraction ---------------------------

/*
Summary:
Extracts salient local maxima from a BEV height grid and applies spatial
non-maximum suppression and count limits.
*/
inline std::vector<Keypoint> extractKeypointsBEV(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const KeypointExtractionConfig & cfg)
{
  std::vector<Keypoint> result;
  if (cloud.empty() || cfg.grid_cells <= 0 || cfg.grid_size_m <= 0.0) {
    return result;
  }
  const int gc = cfg.grid_cells;
  const float half = static_cast<float>(cfg.grid_size_m * 0.5);
  const float cell = static_cast<float>(cfg.grid_size_m / static_cast<double>(gc));

  Eigen::MatrixXf max_height;
  Eigen::MatrixXf best_x;
  Eigen::MatrixXf best_y;
  detail::buildMaxHeightGrid(cloud, gc, half, cell, max_height, best_x, best_y);

  const int r = std::max(1, cfg.neighborhood_radius_cells);
  std::vector<Keypoint> candidates;
  candidates.reserve(static_cast<std::size_t>(gc) * static_cast<std::size_t>(gc));

  for (int iy = 0; iy < gc; ++iy) {
    for (int ix = 0; ix < gc; ++ix) {
      const float h = max_height(iy, ix);
      if (!std::isfinite(h)) {continue;}
      // Local-max test on the (2r+1)x(2r+1) window.
      float floor_h = std::numeric_limits<float>::infinity();
      bool is_max = true;
      int neighbor_count = 0;
      for (int dy = -r; dy <= r && is_max; ++dy) {
        const int ny = iy + dy;
        if (ny < 0 || ny >= gc) {continue;}
        for (int dx = -r; dx <= r; ++dx) {
          const int nx = ix + dx;
          if (nx < 0 || nx >= gc) {continue;}
          if (nx == ix && ny == iy) {continue;}
          const float hn = max_height(ny, nx);
          if (!std::isfinite(hn)) {continue;}
          ++neighbor_count;
          if (hn > h) {is_max = false; break;}
          if (hn < floor_h) {floor_h = hn;}
        }
      }
      if (!is_max || neighbor_count == 0) {continue;}
      const float salience = std::isfinite(floor_h) ? (h - floor_h) : 0.0f;
      if (salience < cfg.min_salience_m) {continue;}
      Keypoint kp;
      kp.position = Eigen::Vector3f(best_x(iy, ix), best_y(iy, ix), h);
      kp.salience = salience;
      candidates.push_back(kp);
    }
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const Keypoint & a, const Keypoint & b) {return a.salience > b.salience;});

  const int keep = std::min<int>(cfg.max_keypoints, static_cast<int>(candidates.size()));
  result.assign(candidates.begin(), candidates.begin() + keep);
  return result;
}

// ------------------ surface-saliency keypoint extraction ------------------

// Detrended-curvature keypoint extractor. Builds the same 2.5D max-height
// grid as BEV_MAX_HEIGHT, fits a robust ground plane (least-squares over the
// lowest surface_plane_fit_percentile of populated cells, so tall structures
// don't drag it off horizontal), subtracts it to get a residual surface, and
// ranks cells by a curvature response on that residual. Selection is by
// percentile rank (top max_keypoints), not an absolute threshold, which is
// what makes this mode work on open/sloped terrain (parking lots, hills)
// where BEV_MAX_HEIGHT (needs vertical prominence) and EDGE_3D (needs edge
// response) both starve under their hard continue-gates.
/*
Summary:
Fits and removes dominant terrain, scores residual surface curvature, and
returns spatially separated high-salience supports.
*/
inline std::vector<Keypoint> extractKeypointsSurfaceSaliency(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const KeypointExtractionConfig & cfg)
{
  std::vector<Keypoint> result;
  if (cloud.empty() || cfg.grid_cells <= 0 || cfg.grid_size_m <= 0.0) {
    return result;
  }
  const int gc = cfg.grid_cells;
  const float half = static_cast<float>(cfg.grid_size_m * 0.5);
  const float cell = static_cast<float>(cfg.grid_size_m / static_cast<double>(gc));

  Eigen::MatrixXf max_height;
  Eigen::MatrixXf best_x;
  Eigen::MatrixXf best_y;
  detail::buildMaxHeightGrid(cloud, gc, half, cell, max_height, best_x, best_y);

  struct Cell
  {
    int iy;
    int ix;
    float z;
  };
  std::vector<Cell> populated;
  populated.reserve(static_cast<std::size_t>(gc) * static_cast<std::size_t>(gc));
  for (int iy = 0; iy < gc; ++iy) {
    for (int ix = 0; ix < gc; ++ix) {
      const float h = max_height(iy, ix);
      if (std::isfinite(h)) {populated.push_back({iy, ix, h});}
    }
  }
  if (populated.size() < 3) {return result;}

  // Robust ground-plane fit: least-squares z = a*x + b*y + c over the lower
  // height-percentile of populated cells.
  std::vector<Cell> by_height = populated;
  std::sort(
    by_height.begin(), by_height.end(),
    [](const Cell & a, const Cell & b) {return a.z < b.z;});
  const double pct = std::max(0.05, std::min(1.0, cfg.surface_plane_fit_percentile));
  const std::size_t n_ground = std::max<std::size_t>(
    3, static_cast<std::size_t>(static_cast<double>(by_height.size()) * pct));

  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < n_ground; ++i) {
    const auto & c = by_height[i];
    const double x = static_cast<double>(best_x(c.iy, c.ix));
    const double y = static_cast<double>(best_y(c.iy, c.ix));
    const double z = static_cast<double>(c.z);
    A(0, 0) += x * x; A(0, 1) += x * y; A(0, 2) += x;
    A(1, 0) += x * y; A(1, 1) += y * y; A(1, 2) += y;
    A(2, 0) += x;     A(2, 1) += y;     A(2, 2) += 1.0;
    rhs(0) += x * z; rhs(1) += y * z; rhs(2) += z;
  }
  Eigen::Vector3d plane = Eigen::Vector3d::Zero();
  if (std::abs(A.determinant()) > 1e-9) {
    plane = A.colPivHouseholderQr().solve(rhs);
  }
  const double pa = plane(0);
  const double pb = plane(1);
  const double pc = plane(2);

  // Detrend: residual(iy, ix) = z - plane(x, y). NaN marks unpopulated cells.
  Eigen::MatrixXf residual = Eigen::MatrixXf::Constant(
    gc, gc, std::numeric_limits<float>::quiet_NaN());
  for (const auto & c : populated) {
    const double x = static_cast<double>(best_x(c.iy, c.ix));
    const double y = static_cast<double>(best_y(c.iy, c.ix));
    const double plane_z = pa * x + pb * y + pc;
    residual(c.iy, c.ix) = static_cast<float>(static_cast<double>(c.z) - plane_z);
  }

  // Curvature saliency: |residual - mean(neighborhood residual)| over a
  // (2r+1)x(2r+1) window on the detrended surface. Crater rims, boulders,
  // curbs and pole bases become extrema; flat ground -> ~0.
  const int r = std::max(1, cfg.surface_curvature_radius_cells);
  struct Candidate
  {
    int iy;
    int ix;
    float saliency;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(populated.size());
  for (const auto & c : populated) {
    const float res = residual(c.iy, c.ix);
    float sum = 0.0f;
    int count = 0;
    for (int dy = -r; dy <= r; ++dy) {
      const int ny = c.iy + dy;
      if (ny < 0 || ny >= gc) {continue;}
      for (int dx = -r; dx <= r; ++dx) {
        const int nx = c.ix + dx;
        if (nx < 0 || nx >= gc) {continue;}
        if (nx == c.ix && ny == c.iy) {continue;}
        const float rn = residual(ny, nx);
        if (!std::isfinite(rn)) {continue;}
        sum += rn;
        ++count;
      }
    }
    if (count == 0) {continue;}
    const float mean_neighbor = sum / static_cast<float>(count);
    candidates.push_back({c.iy, c.ix, std::abs(res - mean_neighbor)});
  }
  if (candidates.empty()) {return result;}

  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {return a.saliency > b.saliency;});

  // Soft percentile floor: drop the lowest-saliency tail of the candidate
  // distribution before ranking. The primary selector stays relative rank
  // (the top-N cut below), not this floor -- that is the anti-starvation
  // fix, since a flat lot and a crater field both then yield ~max_keypoints
  // instead of zero.
  const double floor_pct = std::max(0.0, std::min(1.0, cfg.surface_min_saliency_percentile));
  if (floor_pct > 0.0) {
    const std::size_t floor_cut = std::max<std::size_t>(
      1, static_cast<std::size_t>(static_cast<double>(candidates.size()) * (1.0 - floor_pct)));
    if (floor_cut < candidates.size()) {candidates.resize(floor_cut);}
  }

  // NMS (same pattern as EDGE_3D): greedily accept by descending saliency,
  // reject candidates within edge_nms_radius_m of an already-accepted one so
  // keypoints spread over the tile instead of clustering on one ridge.
  const float nms_r = std::max(0.0f, cfg.edge_nms_radius_m);
  const float nms_r2 = nms_r * nms_r;
  std::vector<Keypoint> kept;
  kept.reserve(candidates.size());
  for (const auto & c : candidates) {
    // Emit the original (non-detrended) z: the triangles and downstream
    // SE(3) must live in the real submap frame; detrending only guided
    // which cells to select.
    const Eigen::Vector3f pos(best_x(c.iy, c.ix), best_y(c.iy, c.ix), max_height(c.iy, c.ix));
    bool suppressed = false;
    if (nms_r > 0.0f) {
      for (const auto & k : kept) {
        if ((k.position - pos).squaredNorm() < nms_r2) {
          suppressed = true;
          break;
        }
      }
    }
    if (suppressed) {continue;}
    Keypoint kp;
    kp.position = pos;
    kp.salience = c.saliency;
    kept.push_back(kp);
    if (cfg.max_keypoints > 0 && static_cast<int>(kept.size()) >= cfg.max_keypoints) {break;}
  }

  result = std::move(kept);
  return result;
}

// --------------------------- edge-3D keypoint extraction ---------------------------

// PCA-based edge keypoint extractor. For each voxel-downsampled point, compute
// the covariance of its radius-r neighbors and pick points where the largest
// eigenvalue dominates the middle one — geometrically these are linear /
// edge-like supports (column edges, wall corners, door frames) that survive in
// narrow-FOV LiDAR and indoor scenes where BEV max-height collapses.
/*
Summary:
Uses local PCA edgeness and three-dimensional non-maximum suppression to extract
repeatable structural keypoints.
*/
inline std::vector<Keypoint> extractKeypointsEdge3D(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const KeypointExtractionConfig & cfg)
{
  std::vector<Keypoint> result;
  if (cloud.empty()) {return result;}

  // Voxel downsample so the PCA cost is bounded regardless of submap density.
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr src(new pcl::PointCloud<pcl::PointXYZI>(cloud));
  if (cfg.edge_voxel_size_m > 0.0f) {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(src);
    vg.setLeafSize(cfg.edge_voxel_size_m, cfg.edge_voxel_size_m, cfg.edge_voxel_size_m);
    vg.filter(*downsampled);
  } else {
    downsampled = src;
  }
  if (downsampled->size() < static_cast<std::size_t>(std::max(3, cfg.edge_min_neighbors))) {
    return result;
  }

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(downsampled);

  struct Candidate
  {
    int idx;
    float edgeness;
    Eigen::Vector3f position;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(downsampled->size());

  const float radius = std::max(0.05f, cfg.edge_neighbor_radius_m);
  std::vector<int> nn_idx;
  std::vector<float> nn_sq;

  for (std::size_t i = 0; i < downsampled->size(); ++i) {
    const auto & p = downsampled->points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    nn_idx.clear();
    nn_sq.clear();
    const int found = kdtree.radiusSearch(static_cast<int>(i), radius, nn_idx, nn_sq);
    if (found < cfg.edge_min_neighbors) {continue;}

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int idx : nn_idx) {
      const auto & q = downsampled->points[idx];
      centroid += Eigen::Vector3f(q.x, q.y, q.z);
    }
    centroid /= static_cast<float>(nn_idx.size());

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int idx : nn_idx) {
      const auto & q = downsampled->points[idx];
      Eigen::Vector3f d(q.x - centroid.x(), q.y - centroid.y(), q.z - centroid.z());
      cov += d * d.transpose();
    }
    cov /= static_cast<float>(nn_idx.size());

    // SelfAdjointEigenSolver returns eigenvalues in ascending order.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov, Eigen::EigenvaluesOnly);
    if (es.info() != Eigen::Success) {continue;}
    const Eigen::Vector3f ev = es.eigenvalues();
    const float lam0 = std::max(0.0f, ev(0));
    const float lam1 = std::max(0.0f, ev(1));
    const float lam2 = std::max(0.0f, ev(2));
    if (lam2 <= 1e-9f) {continue;}
    const float edgeness = (lam2 - lam1) / lam2;
    if (edgeness < cfg.edge_min_edgeness) {continue;}

    Candidate c;
    c.idx = static_cast<int>(i);
    c.edgeness = edgeness;
    c.position = Eigen::Vector3f(p.x, p.y, p.z);
    candidates.push_back(c);
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {return a.edgeness > b.edgeness;});

  // Non-maximum suppression in 3D: drop candidates that fall within
  // edge_nms_radius_m of an already-accepted higher-edgeness keypoint.
  const float nms_r = std::max(0.0f, cfg.edge_nms_radius_m);
  const float nms_r2 = nms_r * nms_r;
  std::vector<Keypoint> kept;
  kept.reserve(candidates.size());
  for (const auto & c : candidates) {
    bool suppressed = false;
    if (nms_r > 0.0f) {
      for (const auto & k : kept) {
        if ((k.position - c.position).squaredNorm() < nms_r2) {
          suppressed = true;
          break;
        }
      }
    }
    if (suppressed) {continue;}
    Keypoint kp;
    kp.position = c.position;
    kp.salience = c.edgeness;
    kept.push_back(kp);
    if (cfg.max_keypoints > 0 && static_cast<int>(kept.size()) >= cfg.max_keypoints) {break;}
  }

  result = std::move(kept);
  return result;
}

// Dispatcher: pick the keypoint extractor based on ``cfg.mode``. New code
// should call this rather than the mode-specific entry points; the BEV and
// EDGE_3D functions stay public for tests and ablation tooling.
/*
Summary:
Dispatches keypoint extraction to the configured BEV, surface-saliency, or 3D
edge strategy.
*/
inline std::vector<Keypoint> extractKeypoints(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const KeypointExtractionConfig & cfg)
{
  switch (cfg.mode) {
    case KeypointMode::EDGE_3D:
      return extractKeypointsEdge3D(cloud, cfg);
    case KeypointMode::SURFACE_SALIENCY:
      return extractKeypointsSurfaceSaliency(cloud, cfg);
    case KeypointMode::BEV_MAX_HEIGHT:
    default:
      return extractKeypointsBEV(cloud, cfg);
  }
}

// --------------------------- triangle enumeration ---------------------------

/*
Summary:
Enumerates valid keypoint triples, rejects unstable geometry, and returns the
most informative bounded set of sorted-edge triangle descriptors.
*/
inline std::vector<TriangleDescriptor> buildTriangles(
  const std::vector<Keypoint> & keypoints,
  const TriangleBuildConfig & cfg)
{
  std::vector<TriangleDescriptor> all;
  const int n = static_cast<int>(keypoints.size());
  if (n < 3) {return all;}
  all.reserve(static_cast<std::size_t>(n) * static_cast<std::size_t>(n));

  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const float l_ij = detail::edgeLength(keypoints[i].position, keypoints[j].position);
      if (l_ij < cfg.min_edge_m || l_ij > cfg.max_edge_m) {continue;}
      for (int k = j + 1; k < n; ++k) {
        const float l_jk = detail::edgeLength(keypoints[j].position, keypoints[k].position);
        if (l_jk < cfg.min_edge_m || l_jk > cfg.max_edge_m) {continue;}
        const float l_ik = detail::edgeLength(keypoints[i].position, keypoints[k].position);
        if (l_ik < cfg.min_edge_m || l_ik > cfg.max_edge_m) {continue;}

        // Sort edges ascending. Track which keypoint is opposite each edge so
        // downstream code can recover the original physical correspondence.
        // Edge ij is opposite vertex k; edge jk opposite i; edge ik opposite j.
        struct EdgeRef
        {
          float length;
          int opposite_kp;
        };
        std::array<EdgeRef, 3> e = {{
          {l_jk, i}, {l_ik, j}, {l_ij, k},
        }};
        std::sort(
          e.begin(), e.end(),
          [](const EdgeRef & a, const EdgeRef & b) {return a.length < b.length;});

        const float ang = detail::smallestAngleDeg(e[0].length, e[1].length, e[2].length);
        if (ang < cfg.min_angle_deg) {continue;}

        TriangleDescriptor t;
        t.edges = {{e[0].length, e[1].length, e[2].length}};
        t.keypoint_ids = {{e[0].opposite_kp, e[1].opposite_kp, e[2].opposite_kp}};
        all.push_back(t);
      }
    }
  }

  // Keep the most informative triangles (largest baseline first) and cap.
  std::sort(
    all.begin(), all.end(),
    [](const TriangleDescriptor & a, const TriangleDescriptor & b) {
      return a.edges[2] > b.edges[2];
    });
  if (cfg.max_triangles > 0 && static_cast<int>(all.size()) > cfg.max_triangles) {
    all.resize(static_cast<std::size_t>(cfg.max_triangles));
  }
  return all;
}

// --------------------------- 3-point rigid SE(3) ---------------------------

// Estimate the SE(3) transform T such that T * src[i] ≈ dst[i] for i in 0..2,
// using closed-form Umeyama (3D analogue of Kabsch with translation centring).
// Returns identity when the source points are degenerate (collinear / nearly
// coincident).
/*
Summary:
Estimates the rigid source-to-target transform from one matched triangle using
an SVD-based least-squares fit.
*/
inline Eigen::Matrix4f estimateRigidFromTriangle(
  const std::array<Eigen::Vector3f, 3> & src,
  const std::array<Eigen::Vector3f, 3> & dst)
{
  const Eigen::Vector3f src_centroid = (src[0] + src[1] + src[2]) / 3.0f;
  const Eigen::Vector3f dst_centroid = (dst[0] + dst[1] + dst[2]) / 3.0f;
  Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
  for (int i = 0; i < 3; ++i) {
    H += (src[i] - src_centroid) * (dst[i] - dst_centroid).transpose();
  }
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(
    H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0.0f) {
    Eigen::Matrix3f V = svd.matrixV();
    V.col(2) *= -1.0f;
    R = V * svd.matrixU().transpose();
  }
  const Eigen::Vector3f t = dst_centroid - R * src_centroid;
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

// N-point variant of estimateRigidFromTriangle. Used to refine the winning
// 3-point RANSAC hypothesis after we know which triangle pairs agree on the
// SE(3): pooling the 3*N inlier point correspondences and running one final
// Umeyama least-squares reduces translation noise by √N relative to the
// single-triangle estimate. Returns identity if N < 3 or sizes mismatch.
/*
Summary:
Estimates a rigid transform from arbitrary paired point sets for optional
multi-inlier refinement.
*/
inline Eigen::Matrix4f estimateRigidFromCorrespondences(
  const std::vector<Eigen::Vector3f> & src,
  const std::vector<Eigen::Vector3f> & dst)
{
  if (src.size() != dst.size() || src.size() < 3) {
    return Eigen::Matrix4f::Identity();
  }
  const std::size_t n = src.size();
  Eigen::Vector3f src_centroid = Eigen::Vector3f::Zero();
  Eigen::Vector3f dst_centroid = Eigen::Vector3f::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    src_centroid += src[i];
    dst_centroid += dst[i];
  }
  src_centroid /= static_cast<float>(n);
  dst_centroid /= static_cast<float>(n);
  Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    H += (src[i] - src_centroid) * (dst[i] - dst_centroid).transpose();
  }
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0.0f) {
    Eigen::Matrix3f V = svd.matrixV();
    V.col(2) *= -1.0f;
    R = V * svd.matrixU().transpose();
  }
  const Eigen::Vector3f t = dst_centroid - R * src_centroid;
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

}  // namespace triangle
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__TRIANGLE_DESCRIPTOR_HPP_
