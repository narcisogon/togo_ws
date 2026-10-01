#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <hazard_mapping_msgs/msg/hazard_patch.hpp>
#include <lidarslam_msgs/msg/new_submap.hpp>
#include <lidarslam_msgs/msg/map_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Computes one 2D hazard cost patch per submap, from that submap's own
// local-frame cloud only -- never from aggregated/global clouds. Reuses the
// per-cell hazard math already proven in togo_navigation's local_hazard_grid
// (mean height per cell, slope via tan()-threshold neighbor comparison, step
// via max-min height, binary hazard + gradient falloff) rather than a new
// heuristic, so the local and global hazard maps behave consistently.
class HazardPatchNode : public rclcpp::Node
{
public:
  HazardPatchNode()
  : Node("hazard_patch_node")
  {
    patch_resolution_ = declare_parameter<double>("patch_resolution", 0.10);
    patch_half_extent_ = declare_parameter<double>("patch_half_extent", 15.0);
    min_points_per_cell_ = declare_parameter<int>("min_points_per_cell", 3);
    min_obstacle_height_ = declare_parameter<double>("min_obstacle_height", 0.25);
    max_obstacle_height_ = declare_parameter<double>("max_obstacle_height", 1.20);
    terrain_min_height_ = declare_parameter<double>("terrain_min_height", -0.75);
    terrain_max_height_ = declare_parameter<double>("terrain_max_height", 1.20);
    terrain_slope_hazard_deg_ = declare_parameter<double>("terrain_slope_hazard_deg", 22.0);
    severe_slope_deg_ = declare_parameter<double>("severe_slope_deg", 45.0);
    terrain_step_hazard_m_ = declare_parameter<double>("terrain_step_hazard_m", 0.25);
    terrain_neighbor_radius_cells_ = declare_parameter<int>("terrain_neighbor_radius_cells", 2);
    min_points_per_obstacle_cell_ = declare_parameter<int>("min_points_per_obstacle_cell", 1);
    obstacle_unsafe_weight_ = declare_parameter<int>("obstacle_unsafe_weight", 3);
    step_unsafe_weight_ = declare_parameter<int>("step_unsafe_weight", 3);
    max_slope_unsafe_weight_ = declare_parameter<int>("max_slope_unsafe_weight", 5);
    gradient_radius_m_ = declare_parameter<double>("gradient_radius_m", 0.20);
    // Within this inner radius of a detected hazard, cost is unconditionally
    // lethal_cost_ -- no falloff, no exceptions. This is what makes "don't
    // get this close" an actual hard constraint the planner cannot route
    // through (regardless of cost_travel_multiplier or distance savings),
    // rather than just a strong-but-negotiable cost preference. The
    // existing soft gradient only applies beyond this radius, out to
    // gradient_radius_m.
    hard_lethal_radius_m_ = declare_parameter<double>("hard_lethal_radius_m", 0.3);
    gradient_min_cost_ = declare_parameter<int>("gradient_min_cost", 8);
    // Ceiling for the SOFT part of the gradient -- must stay below
    // lethal_cost_ (and below the static layer's lethal_cost_threshold in
    // nav2_slam_params.yaml) so preference never becomes an accidental
    // second lethal wall. Only hard_lethal_radius_m_ should ever be able to
    // block a plan; everything past it is a cost the planner can climb.
    soft_max_cost_ = declare_parameter<int>("soft_max_cost", 60);
    lethal_cost_ = declare_parameter<int>("lethal_cost", 100);

    if (patch_resolution_ <= 0.0) {
      throw std::runtime_error("patch_resolution must be > 0");
    }
    patch_cells_ = std::max(
      1, static_cast<int>(std::ceil(2.0 * patch_half_extent_ / patch_resolution_)));
    terrain_slope_hazard_deg_ = std::clamp(terrain_slope_hazard_deg_, 0.0, 89.0);
    severe_slope_deg_ = std::clamp(
      severe_slope_deg_, terrain_slope_hazard_deg_ + 0.1, 89.0);
    terrain_step_hazard_m_ = std::max(0.0, terrain_step_hazard_m_);
    terrain_neighbor_radius_cells_ = std::max(1, terrain_neighbor_radius_cells_);
    min_points_per_cell_ = std::max(1, min_points_per_cell_);
    min_points_per_obstacle_cell_ = std::max(1, min_points_per_obstacle_cell_);
    obstacle_unsafe_weight_ = std::clamp(obstacle_unsafe_weight_, 1, 255);
    step_unsafe_weight_ = std::clamp(step_unsafe_weight_, 1, 255);
    max_slope_unsafe_weight_ = std::clamp(max_slope_unsafe_weight_, 1, 255);
    gradient_radius_m_ = std::max(0.0, gradient_radius_m_);
    hard_lethal_radius_m_ = std::clamp(hard_lethal_radius_m_, 0.0, gradient_radius_m_);
    gradient_min_cost_ = std::clamp(gradient_min_cost_, 0, 99);
    lethal_cost_ = std::clamp(lethal_cost_, 1, 100);
    soft_max_cost_ = std::clamp(soft_max_cost_, gradient_min_cost_, lethal_cost_ - 1);

    patch_pub_ = create_publisher<hazard_mapping_msgs::msg::HazardPatch>(
      "hazard_patch", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());

    submap_sub_ = create_subscription<lidarslam_msgs::msg::NewSubmap>(
      "submap_created", rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      std::bind(&HazardPatchNode::receiveSubmap, this, std::placeholders::_1));

    // /submap_created is an event stream, so a Nav2 process started after
    // SLAM used to miss every existing submap and could not publish /map
    // until rover motion created another one.  The corrected MapArray is a
    // transient-local snapshot; use it to backfill any missing patches.
    map_array_sub_ = create_subscription<lidarslam_msgs::msg::MapArray>(
      "modified_map_array",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&HazardPatchNode::receiveMapArray, this, std::placeholders::_1));

    worker_ = std::thread(&HazardPatchNode::workerLoop, this);

    RCLCPP_INFO(
      get_logger(), "hazard_patch_node: %.1fx%.1fm patch @ %.2fm/cell, slope=%.1fdeg step=%.2fm",
      2.0 * patch_half_extent_, 2.0 * patch_half_extent_, patch_resolution_,
      terrain_slope_hazard_deg_, terrain_step_hazard_m_);
  }

  ~HazardPatchNode() override
  {
    {
      std::lock_guard<std::mutex> lock(queue_mtx_);
      shutting_down_ = true;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void receiveMapArray(const lidarslam_msgs::msg::MapArray::SharedPtr msg)
  {
    if (msg->cloud_coordinate != msg->LOCAL) {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot backfill hazard patches: /modified_map_array clouds are not local");
      return;
    }
    for (size_t i = 0; i < msg->submaps.size(); ++i) {
      auto submap = std::make_shared<lidarslam_msgs::msg::NewSubmap>();
      submap->header = msg->submaps[i].header;
      submap->submap_index = static_cast<uint32_t>(i);
      submap->pose = msg->submaps[i].pose;
      submap->cloud = msg->submaps[i].cloud;
      receiveSubmap(submap);
    }
  }

  void receiveSubmap(const lidarslam_msgs::msg::NewSubmap::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mtx_);
      pending_.push_back({msg, std::chrono::steady_clock::now()});
    }
    queue_cv_.notify_one();
  }

  void workerLoop()
  {
    while (true) {
      lidarslam_msgs::msg::NewSubmap::SharedPtr submap;
      std::chrono::steady_clock::time_point enqueued_at;
      {
        std::unique_lock<std::mutex> lock(queue_mtx_);
        queue_cv_.wait(lock, [this] {return shutting_down_ || !pending_.empty();});
        if (pending_.empty()) {
          // Only true when shutting_down_ and nothing left to drain.
          return;
        }
        submap = pending_.front().first;
        enqueued_at = pending_.front().second;
        pending_.pop_front();
      }
      // Idle waiting here is normal (no work pending). But once notified,
      // the worker should start almost instantly -- a large gap between
      // enqueue and dequeue-start means this thread was starved of CPU by
      // the OS scheduler, not that it was legitimately busy or idle.
      const double wait_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - enqueued_at).count();
      if (wait_sec > kQueueLatencyWarnSec) {
        RCLCPP_WARN(
          get_logger(),
          "hazard_patch_node worker was starved: submap %u sat queued for %.2fs before "
          "processing started -- likely CPU/executor starvation, not hazard logic",
          submap->submap_index, wait_sec);
      }
      // Idempotent: never recompute a patch for the same submap_index.
      if (processed_.insert(submap->submap_index).second) {
        buildAndPublishPatch(*submap);
      }
    }
  }

  static constexpr double kQueueLatencyWarnSec = 1.0;

  void buildAndPublishPatch(const lidarslam_msgs::msg::NewSubmap & submap)
  {
    const size_t grid_size =
      static_cast<size_t>(patch_cells_) * static_cast<size_t>(patch_cells_);
    std::vector<int> terrain_count(grid_size, 0);
    std::vector<double> terrain_sum_z(grid_size, 0.0);
    std::vector<double> terrain_min_z(grid_size, std::numeric_limits<double>::infinity());
    std::vector<double> terrain_max_z(grid_size, -std::numeric_limits<double>::infinity());
    std::vector<int> obstacle_count(grid_size, 0);

    const double origin = -patch_half_extent_;  // patch covers [-half_extent, +half_extent]

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(submap.cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(submap.cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(submap.cloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const float px = *iter_x;
        const float py = *iter_y;
        const float pz = *iter_z;
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
          continue;
        }
        const int cx = static_cast<int>(std::floor((px - origin) / patch_resolution_));
        const int cy = static_cast<int>(std::floor((py - origin) / patch_resolution_));
        if (cx < 0 || cy < 0 || cx >= patch_cells_ || cy >= patch_cells_) {
          continue;
        }
        const size_t idx = static_cast<size_t>(cy) * patch_cells_ + static_cast<size_t>(cx);
        if (pz >= terrain_min_height_ && pz <= terrain_max_height_) {
          terrain_count[idx] += 1;
          terrain_sum_z[idx] += pz;
          terrain_min_z[idx] = std::min(terrain_min_z[idx], static_cast<double>(pz));
          terrain_max_z[idx] = std::max(terrain_max_z[idx], static_cast<double>(pz));
        }
        if (pz >= min_obstacle_height_ && pz <= max_obstacle_height_) {
          obstacle_count[idx] += 1;
        }
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR(
        get_logger(), "submap %u cloud missing x/y/z fields: %s", submap.submap_index, ex.what());
      return;
    }

    // -1 = unknown (no data), matching OccupancyGrid convention -- required
    // for the composer's "unknown never overrides known" composition rule.
    std::vector<int8_t> cost(grid_size, -1);
    std::vector<uint8_t> safe_evidence(grid_size, 0);
    std::vector<uint8_t> unsafe_evidence(grid_size, 0);
    std::vector<double> mean_z(grid_size, std::numeric_limits<double>::quiet_NaN());
    for (int cy = 0; cy < patch_cells_; ++cy) {
      for (int cx = 0; cx < patch_cells_; ++cx) {
        const size_t idx = static_cast<size_t>(cy) * patch_cells_ + static_cast<size_t>(cx);
        if (terrain_count[idx] >= min_points_per_cell_) {
          mean_z[idx] = terrain_sum_z[idx] / static_cast<double>(terrain_count[idx]);
          cost[idx] = 0;  // known, provisionally free
          safe_evidence[idx] = 1;
        }
      }
    }

    std::vector<std::pair<int, int>> hazard_cells;

    // Obstacle-height hazard (mirrors local_hazard_grid's markHazard pass).
    for (int cy = 0; cy < patch_cells_; ++cy) {
      for (int cx = 0; cx < patch_cells_; ++cx) {
        const size_t idx = static_cast<size_t>(cy) * patch_cells_ + static_cast<size_t>(cx);
        if (obstacle_count[idx] >= min_points_per_obstacle_cell_) {
          safe_evidence[idx] = 0;
          unsafe_evidence[idx] = static_cast<uint8_t>(obstacle_unsafe_weight_);
          markHazard(cost, cx, cy, hazard_cells);
        }
      }
    }

    // Terrain slope/step hazard (mirrors local_hazard_grid's markTerrainHazards).
    for (int cy = 0; cy < patch_cells_; ++cy) {
      for (int cx = 0; cx < patch_cells_; ++cx) {
        const size_t idx = static_cast<size_t>(cy) * patch_cells_ + static_cast<size_t>(cx);
        if (!std::isfinite(mean_z[idx])) {
          continue;
        }
        const bool step_hazard =
          (terrain_max_z[idx] - terrain_min_z[idx]) >= terrain_step_hazard_m_;
        double max_slope_deg = 0.0;
        for (int dy = -terrain_neighbor_radius_cells_;
          dy <= terrain_neighbor_radius_cells_; ++dy)
        {
          for (int dx = -terrain_neighbor_radius_cells_;
            dx <= terrain_neighbor_radius_cells_; ++dx)
          {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = cx + dx;
            const int ny = cy + dy;
            if (nx < 0 || ny < 0 || nx >= patch_cells_ || ny >= patch_cells_) {
              continue;
            }
            const size_t nidx = static_cast<size_t>(ny) * patch_cells_ + static_cast<size_t>(nx);
            if (!std::isfinite(mean_z[nidx])) {
              continue;
            }
            const double horizontal_distance =
              patch_resolution_ * std::hypot(static_cast<double>(dx), static_cast<double>(dy));
            const double slope = std::abs(mean_z[idx] - mean_z[nidx]) / horizontal_distance;
            max_slope_deg = std::max(max_slope_deg, std::atan(slope) * 180.0 / M_PI);
          }
        }
        const bool slope_hazard = max_slope_deg >= terrain_slope_hazard_deg_;
        const bool hazard = step_hazard || slope_hazard;
        if (hazard) {
          int weight = step_hazard ? step_unsafe_weight_ : 1;
          if (slope_hazard) {
            const double severity = std::clamp(
              (max_slope_deg - terrain_slope_hazard_deg_) /
              (severe_slope_deg_ - terrain_slope_hazard_deg_), 0.0, 1.0);
            weight = std::max(
              weight, 1 + static_cast<int>(
                std::round(severity * static_cast<double>(max_slope_unsafe_weight_ - 1))));
          }
          safe_evidence[idx] = 0;
          unsafe_evidence[idx] = static_cast<uint8_t>(
            std::max(static_cast<int>(unsafe_evidence[idx]), weight));
          markHazard(cost, cx, cy, hazard_cells);
        }
      }
    }

    applyGradient(cost, hazard_cells);

    hazard_mapping_msgs::msg::HazardPatch patch_msg;
    patch_msg.header.stamp = submap.header.stamp;
    patch_msg.header.frame_id = "map";
    patch_msg.submap_index = submap.submap_index;
    patch_msg.pose = submap.pose;
    patch_msg.resolution = static_cast<float>(patch_resolution_);
    patch_msg.width = static_cast<uint32_t>(patch_cells_);
    patch_msg.height = static_cast<uint32_t>(patch_cells_);
    patch_msg.origin_x = static_cast<float>(origin);
    patch_msg.origin_y = static_cast<float>(origin);
    patch_msg.cost.assign(cost.begin(), cost.end());
    patch_msg.safe_evidence = std::move(safe_evidence);
    patch_msg.unsafe_evidence = std::move(unsafe_evidence);
    patch_pub_->publish(patch_msg);
  }

  void markHazard(
    std::vector<int8_t> & cost, int cx, int cy,
    std::vector<std::pair<int, int>> & hazard_cells) const
  {
    const size_t idx = static_cast<size_t>(cy) * patch_cells_ + static_cast<size_t>(cx);
    if (cost[idx] < lethal_cost_) {
      cost[idx] = static_cast<int8_t>(lethal_cost_);
      hazard_cells.emplace_back(cx, cy);
    }
  }

  void applyGradient(
    std::vector<int8_t> & cost,
    const std::vector<std::pair<int, int>> & hazard_cells) const
  {
    const int radius_cells = static_cast<int>(std::ceil(gradient_radius_m_ / patch_resolution_));
    if (radius_cells <= 0) {
      return;
    }
    // Cells within hard_lethal_radius_cells get unconditional lethal cost --
    // no falloff. This is what makes "stay this far from a hazard" an actual
    // hard constraint the planner can't route through, rather than just a
    // strong cost preference it can trade off against a shorter path. The
    // soft gradient below only shapes the region beyond this inner ring.
    const int hard_lethal_radius_cells =
      static_cast<int>(std::ceil(hard_lethal_radius_m_ / patch_resolution_));
    const int soft_span_cells = std::max(1, radius_cells - hard_lethal_radius_cells);
    for (const auto & hazard : hazard_cells) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const double distance = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
          if (distance > radius_cells) {
            continue;
          }
          const int nx = hazard.first + dx;
          const int ny = hazard.second + dy;
          if (nx < 0 || ny < 0 || nx >= patch_cells_ || ny >= patch_cells_) {
            continue;
          }
          const size_t idx = static_cast<size_t>(ny) * patch_cells_ + static_cast<size_t>(nx);
          // Unknown cells (no points) stay unknown -- gradient only refines
          // cells we actually have terrain data for, never invents data.
          if (cost[idx] < 0 || cost[idx] >= lethal_cost_) {
            continue;
          }
          int c;
          int c_max;
          if (distance <= hard_lethal_radius_cells) {
            c = lethal_cost_;
            c_max = lethal_cost_;
          } else {
            // Soft band never reaches lethal_cost_ -- it tops out at
            // soft_max_cost_, which must stay below the static layer's
            // lethal_cost_threshold (nav2_slam_params.yaml). Otherwise the
            // "preference" halo silently becomes a second lethal wall the
            // planner can't route through, defeating the point of having a
            // soft band at all.
            const double soft_distance = distance - hard_lethal_radius_cells;
            const double falloff = 1.0 - soft_distance / static_cast<double>(soft_span_cells);
            c = gradient_min_cost_ +
              static_cast<int>(std::round((soft_max_cost_ - gradient_min_cost_) * falloff));
            c_max = soft_max_cost_;
          }
          cost[idx] = static_cast<int8_t>(
            std::max(static_cast<int>(cost[idx]), std::clamp(c, gradient_min_cost_, c_max)));
        }
      }
    }
  }

  // params
  double patch_resolution_ {};
  double patch_half_extent_ {};
  int patch_cells_ {};
  int min_points_per_cell_ {};
  double min_obstacle_height_ {};
  double max_obstacle_height_ {};
  double terrain_min_height_ {};
  double terrain_max_height_ {};
  double terrain_slope_hazard_deg_ {};
  double severe_slope_deg_ {};
  double terrain_step_hazard_m_ {};
  int terrain_neighbor_radius_cells_ {};
  int min_points_per_obstacle_cell_ {};
  int obstacle_unsafe_weight_ {};
  int step_unsafe_weight_ {};
  int max_slope_unsafe_weight_ {};
  double gradient_radius_m_ {};
  double hard_lethal_radius_m_ {};
  int gradient_min_cost_ {};
  int soft_max_cost_ {};
  int lethal_cost_ {};

  rclcpp::Publisher<hazard_mapping_msgs::msg::HazardPatch>::SharedPtr patch_pub_;
  rclcpp::Subscription<lidarslam_msgs::msg::NewSubmap>::SharedPtr submap_sub_;
  rclcpp::Subscription<lidarslam_msgs::msg::MapArray>::SharedPtr map_array_sub_;

  std::thread worker_;
  std::mutex queue_mtx_;
  std::condition_variable queue_cv_;
  std::deque<std::pair<lidarslam_msgs::msg::NewSubmap::SharedPtr, std::chrono::steady_clock::time_point>>
    pending_;
  bool shutting_down_ {false};
  std::set<uint32_t> processed_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HazardPatchNode>());
  rclcpp::shutdown();
  return 0;
}
