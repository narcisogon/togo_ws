#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <hazard_mapping_msgs/msg/hazard_patch.hpp>
#include <lidarslam_msgs/msg/map_array.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
double elapsedMillis(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

struct Pose2D
{
  double x {0.0};
  double y {0.0};
  double yaw {0.0};
};

struct AABB
{
  double min_x {0.0};
  double min_y {0.0};
  double max_x {0.0};
  double max_y {0.0};
};

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

Pose2D poseToPose2D(const geometry_msgs::msg::Pose & pose)
{
  return {pose.position.x, pose.position.y, yawFromQuaternion(pose.orientation)};
}

AABB unionAABB(const AABB & a, const AABB & b)
{
  return {
    std::min(a.min_x, b.min_x), std::min(a.min_y, b.min_y),
    std::max(a.max_x, b.max_x), std::max(a.max_y, b.max_y)};
}

AABB expandAABB(const AABB & box, double margin)
{
  return {
    box.min_x - margin, box.min_y - margin,
    box.max_x + margin, box.max_y + margin};
}

bool aabbIntersects(const AABB & a, const AABB & b)
{
  return a.min_x <= b.max_x && a.max_x >= b.min_x && a.min_y <= b.max_y && a.max_y >= b.min_y;
}
}  // namespace

// Composes per-submap 2D hazard patches into a growing global OccupancyGrid,
// published directly on /map (+/map_updates) to match Nav2's existing
// static_layer subscription (togo_navigation/config/nav2_slam_params.yaml)
// -- no Nav2 YAML changes needed. Projection uses x/y/yaw only (z/roll/pitch
// ignored, per the design rationale that a 0.1m grid doesn't need full 3D
// projection and this keeps z-drift from ever coupling into the composed
// costmap). On a pose change, only the affected region is cleared and
// re-projected from stored patches -- never a global re-analysis.
class GlobalCostmapComposer : public rclcpp::Node
{
public:
  GlobalCostmapComposer()
  : Node("global_costmap_composer")
  {
    resolution_ = declare_parameter<double>("global_resolution", 0.10);
    initial_size_m_ = declare_parameter<double>("initial_size_m", 200.0);
    growth_margin_m_ = declare_parameter<double>("growth_margin_m", 50.0);
    pose_change_translation_thresh_m_ =
      declare_parameter<double>("pose_change_translation_thresh", 0.05);
    pose_change_yaw_thresh_deg_ = declare_parameter<double>("pose_change_yaw_thresh_deg", 0.5);
    full_republish_period_sec_ = declare_parameter<double>("full_republish_period_sec", 10.0);
    hazard_probability_threshold_ =
      declare_parameter<double>("hazard_probability_threshold", 0.60);
    soft_probability_threshold_ =
      declare_parameter<double>("soft_probability_threshold", 0.20);
    soft_min_cost_ = declare_parameter<int>("evidence_soft_min_cost", 5);
    soft_max_cost_ = declare_parameter<int>("evidence_soft_max_cost", 60);
    lethal_cost_ = declare_parameter<int>("evidence_lethal_cost", 100);
    minimum_unsafe_evidence_for_lethal_ =
      declare_parameter<int>("minimum_unsafe_evidence_for_lethal", 2);
    max_effective_evidence_ = declare_parameter<int>("max_effective_evidence", 40);
    evidence_gradient_radius_m_ =
      declare_parameter<double>("evidence_gradient_radius_m", 1.2);
    evidence_gradient_min_cost_ =
      declare_parameter<int>("evidence_gradient_min_cost", 5);
    evidence_gradient_max_cost_ =
      declare_parameter<int>("evidence_gradient_max_cost", 60);

    if (resolution_ <= 0.0) {
      throw std::runtime_error("global_resolution must be > 0");
    }
    hazard_probability_threshold_ = std::clamp(hazard_probability_threshold_, 0.01, 1.0);
    soft_probability_threshold_ = std::clamp(
      soft_probability_threshold_, 0.0, hazard_probability_threshold_ - 0.01);
    soft_min_cost_ = std::clamp(soft_min_cost_, 0, 99);
    soft_max_cost_ = std::clamp(soft_max_cost_, soft_min_cost_, 99);
    lethal_cost_ = std::clamp(lethal_cost_, soft_max_cost_ + 1, 100);
    minimum_unsafe_evidence_for_lethal_ =
      std::clamp(minimum_unsafe_evidence_for_lethal_, 1, 255);
    max_effective_evidence_ = std::clamp(max_effective_evidence_, 2, 65535);
    evidence_gradient_radius_m_ = std::max(0.0, evidence_gradient_radius_m_);
    evidence_gradient_min_cost_ = std::clamp(evidence_gradient_min_cost_, 0, 99);
    evidence_gradient_max_cost_ = std::clamp(
      evidence_gradient_max_cost_, evidence_gradient_min_cost_, lethal_cost_ - 1);

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable().transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos);
    update_pub_ = create_publisher<map_msgs::msg::OccupancyGridUpdate>(
      "map_updates", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
      "composer_timing_diagnostics", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());

    patch_sub_ = create_subscription<hazard_mapping_msgs::msg::HazardPatch>(
      "hazard_patch", rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      std::bind(&GlobalCostmapComposer::onHazardPatch, this, std::placeholders::_1));
    map_array_sub_ = create_subscription<lidarslam_msgs::msg::MapArray>(
      "modified_map_array",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&GlobalCostmapComposer::onMapArray, this, std::placeholders::_1));

    // Publish a transient-local unknown map immediately.  Nav2 can now
    // configure its static layer before the first hazard patch arrives,
    // instead of appearing map-less until the rover creates a new submap.
    initializeGrid(AABB {});
    global_grid_.header.stamp = this->now();
    map_pub_->publish(global_grid_);

    full_republish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(full_republish_period_sec_)),
      [this]() {
        if (grid_initialized_) {
          global_grid_.header.stamp = this->now();
          map_pub_->publish(global_grid_);
        }
      });

    RCLCPP_INFO(
      get_logger(), "global_costmap_composer: %.2fm/cell, publishing /map (+ /map_updates)",
      resolution_);
  }

private:
  void onHazardPatch(const hazard_mapping_msgs::msg::HazardPatch::SharedPtr msg)
  {
    const auto start = std::chrono::steady_clock::now();
    if (std::abs(static_cast<double>(msg->resolution) - resolution_) > 1e-6) {
      RCLCPP_FATAL(
        get_logger(),
        "hazard patch resolution %.4f != global_resolution %.4f (submap %u) -- dropping patch",
        msg->resolution, resolution_, msg->submap_index);
      return;
    }
    patches_[msg->submap_index] = *msg;
    current_pose_[msg->submap_index] = msg->pose;

    const Pose2D pose2d = poseToPose2D(msg->pose);
    const AABB affected = expandAABB(
      patchWorldAABB(*msg, pose2d), evidence_gradient_radius_m_);
    const bool resized = ensureGridCovers(affected);
    clearAndReproject(affected);
    publishAffected(affected, resized);

    std::ostringstream diag;
    diag << std::fixed << std::setprecision(3)
         << "{\"event\":\"hazard_patch_event\""
         << ",\"submap_index\":" << msg->submap_index
         << ",\"total_patches\":" << patches_.size()
         << ",\"grid_resized\":" << (resized ? "true" : "false")
         << ",\"safe_votes\":" << last_safe_votes_
         << ",\"unsafe_votes\":" << last_unsafe_votes_
         << ",\"known_cells\":" << last_known_cells_
         << ",\"lethal_cells\":" << last_lethal_cells_
         << ",\"gradient_ms\":" << last_gradient_ms_
         << ",\"total_ms\":" << elapsedMillis(start)
         << "}";
    publishDiagnostic(diag.str());
  }

  void onMapArray(const lidarslam_msgs::msg::MapArray::SharedPtr msg)
  {
    const auto start = std::chrono::steady_clock::now();
    bool any_dirty = false;
    AABB merged {};
    for (size_t i = 0; i < msg->submaps.size(); ++i) {
      const uint32_t idx = static_cast<uint32_t>(i);
      const auto patch_it = patches_.find(idx);
      if (patch_it == patches_.end()) {
        continue;  // no patch computed for this submap yet
      }
      const auto pose_it = current_pose_.find(idx);
      const geometry_msgs::msg::Pose & new_pose = msg->submaps[i].pose;
      if (pose_it != current_pose_.end() && poseChangeBelowThreshold(pose_it->second, new_pose)) {
        continue;  // nothing meaningful moved
      }

      AABB dirty = patchWorldAABB(patch_it->second, poseToPose2D(new_pose));
      if (pose_it != current_pose_.end()) {
        dirty = unionAABB(dirty, patchWorldAABB(patch_it->second, poseToPose2D(pose_it->second)));
      }
      current_pose_[idx] = new_pose;

      merged = any_dirty ? unionAABB(merged, dirty) : dirty;
      any_dirty = true;
    }
    if (!any_dirty) {
      // No diagnostic here -- this fires on essentially every
      // /modified_map_array tick regardless of whether anything moved, and
      // would be pure noise for the case we actually care about timing.
      return;
    }
    merged = expandAABB(merged, evidence_gradient_radius_m_);
    const bool resized = ensureGridCovers(merged);
    const auto reproject_start = std::chrono::steady_clock::now();
    clearAndReproject(merged);
    const double reproject_ms = elapsedMillis(reproject_start);
    publishAffected(merged, resized);

    std::ostringstream diag;
    diag << std::fixed << std::setprecision(3)
         << "{\"event\":\"map_array_event\""
         << ",\"num_submaps\":" << msg->submaps.size()
         << ",\"total_patches\":" << patches_.size()
         // reproject_ms is the linear scan over every stored patch to find
         // which ones intersect the dirty region -- this is the one part of
         // the composer that scales with total trip length/patch count
         // rather than staying bounded to the affected region.
         << ",\"reproject_ms\":" << reproject_ms
         << ",\"total_ms\":" << elapsedMillis(start)
         << "}";
    publishDiagnostic(diag.str());
  }

  bool poseChangeBelowThreshold(
    const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b) const
  {
    const double dx = a.position.x - b.position.x;
    const double dy = a.position.y - b.position.y;
    const double dz = a.position.z - b.position.z;
    const double translation = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (translation > pose_change_translation_thresh_m_) {
      return false;
    }
    double dyaw = yawFromQuaternion(a.orientation) - yawFromQuaternion(b.orientation);
    while (dyaw > M_PI) {dyaw -= 2.0 * M_PI;}
    while (dyaw < -M_PI) {dyaw += 2.0 * M_PI;}
    return std::abs(dyaw) * 180.0 / M_PI <= pose_change_yaw_thresh_deg_;
  }

  AABB patchWorldAABB(const hazard_mapping_msgs::msg::HazardPatch & patch, const Pose2D & pose2d) const
  {
    const double lx0 = patch.origin_x;
    const double ly0 = patch.origin_y;
    const double lx1 = patch.origin_x + patch.width * patch.resolution;
    const double ly1 = patch.origin_y + patch.height * patch.resolution;
    const double cos_yaw = std::cos(pose2d.yaw);
    const double sin_yaw = std::sin(pose2d.yaw);
    const auto transform = [&](double lx, double ly) {
        return std::make_pair(
          pose2d.x + cos_yaw * lx - sin_yaw * ly, pose2d.y + sin_yaw * lx + cos_yaw * ly);
      };
    const auto c0 = transform(lx0, ly0);
    const auto c1 = transform(lx1, ly0);
    const auto c2 = transform(lx0, ly1);
    const auto c3 = transform(lx1, ly1);
    AABB box;
    box.min_x = std::min({c0.first, c1.first, c2.first, c3.first});
    box.max_x = std::max({c0.first, c1.first, c2.first, c3.first});
    box.min_y = std::min({c0.second, c1.second, c2.second, c3.second});
    box.max_y = std::max({c0.second, c1.second, c2.second, c3.second});
    return box;
  }

  // Returns true if the grid geometry (origin/size) changed -- callers must
  // publish the full grid rather than an incremental update in that case,
  // since subscribers' cached grid dimensions would otherwise be stale.
  bool ensureGridCovers(const AABB & required)
  {
    if (!grid_initialized_) {
      initializeGrid(required);
      return true;
    }
    const double grid_min_x = global_grid_.info.origin.position.x;
    const double grid_min_y = global_grid_.info.origin.position.y;
    const double grid_max_x = grid_min_x + global_grid_.info.width * resolution_;
    const double grid_max_y = grid_min_y + global_grid_.info.height * resolution_;
    if (required.min_x >= grid_min_x && required.min_y >= grid_min_y &&
      required.max_x <= grid_max_x && required.max_y <= grid_max_y)
    {
      return false;
    }
    growGrid(required);
    return true;
  }

  void initializeGrid(const AABB & required)
  {
    const double origin_x = required.min_x - growth_margin_m_;
    const double origin_y = required.min_y - growth_margin_m_;
    const double size_x = std::max(
      initial_size_m_, (required.max_x - required.min_x) + 2.0 * growth_margin_m_);
    const double size_y = std::max(
      initial_size_m_, (required.max_y - required.min_y) + 2.0 * growth_margin_m_);

    global_grid_.header.frame_id = "map";
    global_grid_.info.resolution = static_cast<float>(resolution_);
    global_grid_.info.origin.position.x = origin_x;
    global_grid_.info.origin.position.y = origin_y;
    global_grid_.info.origin.orientation.w = 1.0;
    global_grid_.info.width = static_cast<uint32_t>(std::ceil(size_x / resolution_));
    global_grid_.info.height = static_cast<uint32_t>(std::ceil(size_y / resolution_));
    global_grid_.data.assign(
      static_cast<size_t>(global_grid_.info.width) * global_grid_.info.height, -1);
    safe_evidence_.assign(global_grid_.data.size(), 0);
    unsafe_evidence_.assign(global_grid_.data.size(), 0);
    grid_initialized_ = true;
  }

  void growGrid(const AABB & required)
  {
    const double old_origin_x = global_grid_.info.origin.position.x;
    const double old_origin_y = global_grid_.info.origin.position.y;
    const double old_max_x = old_origin_x + global_grid_.info.width * resolution_;
    const double old_max_y = old_origin_y + global_grid_.info.height * resolution_;

    const double new_min_x = std::min(old_origin_x, required.min_x - growth_margin_m_);
    const double new_min_y = std::min(old_origin_y, required.min_y - growth_margin_m_);
    const double new_max_x = std::max(old_max_x, required.max_x + growth_margin_m_);
    const double new_max_y = std::max(old_max_y, required.max_y + growth_margin_m_);

    const auto new_width = static_cast<uint32_t>(std::ceil((new_max_x - new_min_x) / resolution_));
    const auto new_height = static_cast<uint32_t>(std::ceil((new_max_y - new_min_y) / resolution_));

    std::vector<int8_t> new_data(static_cast<size_t>(new_width) * new_height, -1);
    std::vector<uint16_t> new_safe(static_cast<size_t>(new_width) * new_height, 0);
    std::vector<uint16_t> new_unsafe(static_cast<size_t>(new_width) * new_height, 0);
    const int offset_x = static_cast<int>(std::round((old_origin_x - new_min_x) / resolution_));
    const int offset_y = static_cast<int>(std::round((old_origin_y - new_min_y) / resolution_));

    for (uint32_t y = 0; y < global_grid_.info.height; ++y) {
      for (uint32_t x = 0; x < global_grid_.info.width; ++x) {
        const int8_t v =
          global_grid_.data[static_cast<size_t>(y) * global_grid_.info.width + x];
        const int nx = offset_x + static_cast<int>(x);
        const int ny = offset_y + static_cast<int>(y);
        if (nx >= 0 && ny >= 0 && nx < static_cast<int>(new_width) &&
          ny < static_cast<int>(new_height))
        {
          new_data[static_cast<size_t>(ny) * new_width + static_cast<size_t>(nx)] = v;
          const size_t old_idx = static_cast<size_t>(y) * global_grid_.info.width + x;
          const size_t new_idx = static_cast<size_t>(ny) * new_width + static_cast<size_t>(nx);
          new_safe[new_idx] = safe_evidence_[old_idx];
          new_unsafe[new_idx] = unsafe_evidence_[old_idx];
        }
      }
    }

    global_grid_.info.origin.position.x = new_min_x;
    global_grid_.info.origin.position.y = new_min_y;
    global_grid_.info.width = new_width;
    global_grid_.info.height = new_height;
    global_grid_.data = std::move(new_data);
    safe_evidence_ = std::move(new_safe);
    unsafe_evidence_ = std::move(new_unsafe);
  }

  void projectPatchIntoRegion(
    const hazard_mapping_msgs::msg::HazardPatch & patch, const Pose2D & pose2d, const AABB & region)
  {
    const int gx0 = std::max(
      0, static_cast<int>(
        std::floor((region.min_x - global_grid_.info.origin.position.x) / resolution_)));
    const int gy0 = std::max(
      0, static_cast<int>(
        std::floor((region.min_y - global_grid_.info.origin.position.y) / resolution_)));
    const int gx1 = std::min(
      static_cast<int>(global_grid_.info.width) - 1,
      static_cast<int>(std::ceil((region.max_x - global_grid_.info.origin.position.x) / resolution_)));
    const int gy1 = std::min(
      static_cast<int>(global_grid_.info.height) - 1,
      static_cast<int>(std::ceil((region.max_y - global_grid_.info.origin.position.y) / resolution_)));
    if (gx1 < gx0 || gy1 < gy0) {
      return;
    }

    const double cos_yaw = std::cos(pose2d.yaw);
    const double sin_yaw = std::sin(pose2d.yaw);
    for (int gy = gy0; gy <= gy1; ++gy) {
      for (int gx = gx0; gx <= gx1; ++gx) {
        const double wx = global_grid_.info.origin.position.x + (gx + 0.5) * resolution_;
        const double wy = global_grid_.info.origin.position.y + (gy + 0.5) * resolution_;
        const double dx = wx - pose2d.x;
        const double dy = wy - pose2d.y;
        const double lx = cos_yaw * dx + sin_yaw * dy;
        const double ly = -sin_yaw * dx + cos_yaw * dy;
        const int px = static_cast<int>(std::floor((lx - patch.origin_x) / patch.resolution));
        const int py = static_cast<int>(std::floor((ly - patch.origin_y) / patch.resolution));
        if (px < 0 || py < 0 || px >= static_cast<int>(patch.width) ||
          py >= static_cast<int>(patch.height))
        {
          continue;
        }
        const size_t pidx =
          static_cast<size_t>(py) * patch.width + static_cast<size_t>(px);
        const int8_t pval = patch.cost[pidx];
        if (pval < 0) {
          continue;  // unknown never overrides known
        }
        const size_t gidx = static_cast<size_t>(gy) * global_grid_.info.width + static_cast<size_t>(gx);
        // New patches carry independent safe/unsafe evidence. One submap
        // contributes at most one vote of each kind to a destination cell,
        // regardless of how many raw LiDAR points happened to land there.
        if (patch.safe_evidence.size() == patch.cost.size() &&
          patch.unsafe_evidence.size() == patch.cost.size())
        {
          safe_evidence_[gidx] = static_cast<uint16_t>(std::min<uint32_t>(
            65535U, static_cast<uint32_t>(safe_evidence_[gidx]) +
            patch.safe_evidence[pidx]));
          unsafe_evidence_[gidx] = static_cast<uint16_t>(std::min<uint32_t>(
            65535U, static_cast<uint32_t>(unsafe_evidence_[gidx]) +
            patch.unsafe_evidence[pidx]));
        } else {
          // Compatibility with recorded/old HazardPatch messages.
          if (pval == 0) {
            safe_evidence_[gidx] = static_cast<uint16_t>(
              std::min<int>(65535, safe_evidence_[gidx] + 1));
          } else if (pval >= lethal_cost_) {
            unsafe_evidence_[gidx] = static_cast<uint16_t>(
              std::min<int>(65535, unsafe_evidence_[gidx] + 1));
          }
        }
      }
    }
  }

  void clearAndReproject(const AABB & region)
  {
    const int gx0 = std::max(
      0, static_cast<int>(
        std::floor((region.min_x - global_grid_.info.origin.position.x) / resolution_)));
    const int gy0 = std::max(
      0, static_cast<int>(
        std::floor((region.min_y - global_grid_.info.origin.position.y) / resolution_)));
    const int gx1 = std::min(
      static_cast<int>(global_grid_.info.width) - 1,
      static_cast<int>(std::ceil((region.max_x - global_grid_.info.origin.position.x) / resolution_)));
    const int gy1 = std::min(
      static_cast<int>(global_grid_.info.height) - 1,
      static_cast<int>(std::ceil((region.max_y - global_grid_.info.origin.position.y) / resolution_)));
    for (int gy = gy0; gy <= gy1; ++gy) {
      for (int gx = gx0; gx <= gx1; ++gx) {
        global_grid_.data[static_cast<size_t>(gy) * global_grid_.info.width + static_cast<size_t>(gx)] = -1;
        safe_evidence_[static_cast<size_t>(gy) * global_grid_.info.width +
          static_cast<size_t>(gx)] = 0;
        unsafe_evidence_[static_cast<size_t>(gy) * global_grid_.info.width +
          static_cast<size_t>(gx)] = 0;
      }
    }
    // Re-project every stored patch whose current footprint intersects the
    // cleared region -- not just the one(s) that moved -- since other
    // patches may still contribute coverage there.
    for (const auto & [idx, patch] : patches_) {
      const auto pose_it = current_pose_.find(idx);
      if (pose_it == current_pose_.end()) {
        continue;
      }
      const Pose2D pose2d = poseToPose2D(pose_it->second);
      const AABB patch_box = patchWorldAABB(patch, pose2d);
      if (!aabbIntersects(patch_box, region)) {
        continue;
      }
      projectPatchIntoRegion(patch, pose2d, region);
    }
    finalizeEvidence(region);
    const auto gradient_start = std::chrono::steady_clock::now();
    applyEvidenceGradient(region);
    last_gradient_ms_ = elapsedMillis(gradient_start);
  }

  void finalizeEvidence(const AABB & region)
  {
    const int gx0 = std::max(
      0, static_cast<int>(
        std::floor((region.min_x - global_grid_.info.origin.position.x) / resolution_)));
    const int gy0 = std::max(
      0, static_cast<int>(
        std::floor((region.min_y - global_grid_.info.origin.position.y) / resolution_)));
    const int gx1 = std::min(
      static_cast<int>(global_grid_.info.width) - 1,
      static_cast<int>(std::ceil((region.max_x - global_grid_.info.origin.position.x) / resolution_)));
    const int gy1 = std::min(
      static_cast<int>(global_grid_.info.height) - 1,
      static_cast<int>(std::ceil((region.max_y - global_grid_.info.origin.position.y) / resolution_)));

    last_safe_votes_ = 0;
    last_unsafe_votes_ = 0;
    last_known_cells_ = 0;
    last_lethal_cells_ = 0;
    for (int gy = gy0; gy <= gy1; ++gy) {
      for (int gx = gx0; gx <= gx1; ++gx) {
        const size_t idx =
          static_cast<size_t>(gy) * global_grid_.info.width + static_cast<size_t>(gx);
        double safe = safe_evidence_[idx];
        double unsafe = unsafe_evidence_[idx];
        const double total = safe + unsafe;
        if (total <= 0.0) {
          global_grid_.data[idx] = -1;
          continue;
        }
        if (total > max_effective_evidence_) {
          const double scale = static_cast<double>(max_effective_evidence_) / total;
          safe *= scale;
          unsafe *= scale;
          safe_evidence_[idx] = static_cast<uint16_t>(std::round(safe));
          unsafe_evidence_[idx] = static_cast<uint16_t>(std::round(unsafe));
        }
        last_safe_votes_ += safe_evidence_[idx];
        last_unsafe_votes_ += unsafe_evidence_[idx];
        ++last_known_cells_;
        const double probability = unsafe / (safe + unsafe);
        if (unsafe >= minimum_unsafe_evidence_for_lethal_ &&
          probability >= hazard_probability_threshold_)
        {
          global_grid_.data[idx] = static_cast<int8_t>(lethal_cost_);
          ++last_lethal_cells_;
        } else if (probability <= soft_probability_threshold_) {
          global_grid_.data[idx] = 0;
        } else {
          const double alpha = std::clamp(
            (probability - soft_probability_threshold_) /
            (hazard_probability_threshold_ - soft_probability_threshold_), 0.0, 1.0);
          global_grid_.data[idx] = static_cast<int8_t>(
            std::clamp(
              static_cast<int>(std::round(
                soft_min_cost_ + alpha * static_cast<double>(soft_max_cost_ - soft_min_cost_))),
              soft_min_cost_, soft_max_cost_));
        }
      }
    }
  }

  void applyEvidenceGradient(const AABB & region)
  {
    const int radius_cells =
      static_cast<int>(std::ceil(evidence_gradient_radius_m_ / resolution_));
    if (radius_cells <= 0) {
      return;
    }

    // Include a source margin so a lethal cell just outside the dirty
    // region can restore its halo inside the region after recomposition.
    const AABB source_region = expandAABB(region, evidence_gradient_radius_m_);
    const int sx0 = std::max(
      0, static_cast<int>(
        std::floor((source_region.min_x - global_grid_.info.origin.position.x) / resolution_)));
    const int sy0 = std::max(
      0, static_cast<int>(
        std::floor((source_region.min_y - global_grid_.info.origin.position.y) / resolution_)));
    const int sx1 = std::min(
      static_cast<int>(global_grid_.info.width) - 1,
      static_cast<int>(
        std::ceil((source_region.max_x - global_grid_.info.origin.position.x) / resolution_)));
    const int sy1 = std::min(
      static_cast<int>(global_grid_.info.height) - 1,
      static_cast<int>(
        std::ceil((source_region.max_y - global_grid_.info.origin.position.y) / resolution_)));
    if (sx1 < sx0 || sy1 < sy0) {
      return;
    }

    const int width = sx1 - sx0 + 1;
    const int height = sy1 - sy0 + 1;
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> distance(static_cast<size_t>(width) * height, infinity);
    using QueueItem = std::pair<double, size_t>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;

    for (int gy = sy0; gy <= sy1; ++gy) {
      for (int gx = sx0; gx <= sx1; ++gx) {
        const size_t global_idx =
          static_cast<size_t>(gy) * global_grid_.info.width + static_cast<size_t>(gx);
        if (global_grid_.data[global_idx] != lethal_cost_) {
          continue;
        }
        const size_t local_idx =
          static_cast<size_t>(gy - sy0) * width + static_cast<size_t>(gx - sx0);
        distance[local_idx] = 0.0;
        queue.emplace(0.0, local_idx);
      }
    }

    constexpr int kDx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    constexpr int kDy[8] = {0, 0, -1, 1, -1, 1, -1, 1};
    const double diagonal = std::sqrt(2.0);
    while (!queue.empty()) {
      const auto [current_distance, local_idx] = queue.top();
      queue.pop();
      if (current_distance != distance[local_idx] || current_distance >= radius_cells) {
        continue;
      }
      const int x = static_cast<int>(local_idx % width);
      const int y = static_cast<int>(local_idx / width);
      for (int direction = 0; direction < 8; ++direction) {
        const int nx = x + kDx[direction];
        const int ny = y + kDy[direction];
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
          continue;
        }
        const double next_distance =
          current_distance + (direction < 4 ? 1.0 : diagonal);
        if (next_distance > radius_cells) {
          continue;
        }
        const size_t next_idx = static_cast<size_t>(ny) * width + static_cast<size_t>(nx);
        if (next_distance < distance[next_idx]) {
          distance[next_idx] = next_distance;
          queue.emplace(next_distance, next_idx);
        }
      }
    }

    const int gx0 = std::max(
      sx0, static_cast<int>(
        std::floor((region.min_x - global_grid_.info.origin.position.x) / resolution_)));
    const int gy0 = std::max(
      sy0, static_cast<int>(
        std::floor((region.min_y - global_grid_.info.origin.position.y) / resolution_)));
    const int gx1 = std::min(
      sx1, static_cast<int>(
        std::ceil((region.max_x - global_grid_.info.origin.position.x) / resolution_)));
    const int gy1 = std::min(
      sy1, static_cast<int>(
        std::ceil((region.max_y - global_grid_.info.origin.position.y) / resolution_)));
    for (int gy = gy0; gy <= gy1; ++gy) {
      for (int gx = gx0; gx <= gx1; ++gx) {
        const size_t global_idx =
          static_cast<size_t>(gy) * global_grid_.info.width + static_cast<size_t>(gx);
        if (global_grid_.data[global_idx] < 0 ||
          global_grid_.data[global_idx] >= lethal_cost_)
        {
          continue;
        }
        const size_t local_idx =
          static_cast<size_t>(gy - sy0) * width + static_cast<size_t>(gx - sx0);
        if (!std::isfinite(distance[local_idx]) || distance[local_idx] > radius_cells) {
          continue;
        }
        const double falloff = 1.0 - distance[local_idx] / radius_cells;
        const int gradient_cost = evidence_gradient_min_cost_ + static_cast<int>(std::round(
          falloff * (evidence_gradient_max_cost_ - evidence_gradient_min_cost_)));
        global_grid_.data[global_idx] = static_cast<int8_t>(std::max(
          static_cast<int>(global_grid_.data[global_idx]),
          std::clamp(
            gradient_cost, evidence_gradient_min_cost_, evidence_gradient_max_cost_)));
      }
    }
  }

  void publishAffected(const AABB & region, bool grid_resized)
  {
    global_grid_.header.stamp = this->now();
    if (grid_resized) {
      map_pub_->publish(global_grid_);
      return;
    }
    publishIncrementalUpdate(region);
  }

  void publishIncrementalUpdate(const AABB & region)
  {
    const int gx0 = std::max(
      0, static_cast<int>(
        std::floor((region.min_x - global_grid_.info.origin.position.x) / resolution_)));
    const int gy0 = std::max(
      0, static_cast<int>(
        std::floor((region.min_y - global_grid_.info.origin.position.y) / resolution_)));
    const int gx1 = std::min(
      static_cast<int>(global_grid_.info.width) - 1,
      static_cast<int>(std::ceil((region.max_x - global_grid_.info.origin.position.x) / resolution_)));
    const int gy1 = std::min(
      static_cast<int>(global_grid_.info.height) - 1,
      static_cast<int>(std::ceil((region.max_y - global_grid_.info.origin.position.y) / resolution_)));
    if (gx1 < gx0 || gy1 < gy0) {
      return;
    }
    map_msgs::msg::OccupancyGridUpdate update;
    update.header = global_grid_.header;
    update.x = gx0;
    update.y = gy0;
    update.width = static_cast<uint32_t>(gx1 - gx0 + 1);
    update.height = static_cast<uint32_t>(gy1 - gy0 + 1);
    update.data.resize(static_cast<size_t>(update.width) * update.height);
    for (int gy = gy0; gy <= gy1; ++gy) {
      for (int gx = gx0; gx <= gx1; ++gx) {
        update.data[static_cast<size_t>(gy - gy0) * update.width + static_cast<size_t>(gx - gx0)] =
          global_grid_.data[static_cast<size_t>(gy) * global_grid_.info.width + static_cast<size_t>(gx)];
      }
    }
    update_pub_->publish(update);
  }

  void publishDiagnostic(const std::string & payload)
  {
    std_msgs::msg::String msg;
    msg.data = payload;
    diagnostics_pub_->publish(msg);
  }

  double resolution_ {};
  double initial_size_m_ {};
  double growth_margin_m_ {};
  double pose_change_translation_thresh_m_ {};
  double pose_change_yaw_thresh_deg_ {};
  double full_republish_period_sec_ {};
  double hazard_probability_threshold_ {};
  double soft_probability_threshold_ {};
  int soft_min_cost_ {};
  int soft_max_cost_ {};
  int lethal_cost_ {};
  int minimum_unsafe_evidence_for_lethal_ {};
  int max_effective_evidence_ {};
  double evidence_gradient_radius_m_ {};
  int evidence_gradient_min_cost_ {};
  int evidence_gradient_max_cost_ {};
  double last_gradient_ms_ {0.0};

  nav_msgs::msg::OccupancyGrid global_grid_;
  std::vector<uint16_t> safe_evidence_;
  std::vector<uint16_t> unsafe_evidence_;
  uint64_t last_safe_votes_ {0};
  uint64_t last_unsafe_votes_ {0};
  uint64_t last_known_cells_ {0};
  uint64_t last_lethal_cells_ {0};
  bool grid_initialized_ {false};
  std::unordered_map<uint32_t, hazard_mapping_msgs::msg::HazardPatch> patches_;
  std::unordered_map<uint32_t, geometry_msgs::msg::Pose> current_pose_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr update_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<hazard_mapping_msgs::msg::HazardPatch>::SharedPtr patch_sub_;
  rclcpp::Subscription<lidarslam_msgs::msg::MapArray>::SharedPtr map_array_sub_;
  rclcpp::TimerBase::SharedPtr full_republish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalCostmapComposer>());
  rclcpp::shutdown();
  return 0;
}
