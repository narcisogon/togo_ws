#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

struct TimedPoint
{
  double x {};
  double y {};
  double z {};
  rclcpp::Time stamp;
};

struct Point3
{
  double x {};
  double y {};
  double z {};
};

Point3 transformPoint(const Point3 & point, const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Quaternion q;
  tf2::fromMsg(transform.transform.rotation, q);
  q.normalize();
  const tf2::Vector3 p(point.x, point.y, point.z);
  const tf2::Vector3 t(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z);
  const tf2::Vector3 out = tf2::quatRotate(q, p) + t;
  return {out.x(), out.y(), out.z()};
}

double elapsedMillis(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

}  // namespace

class LocalHazardGrid : public rclcpp::Node
{
public:
  LocalHazardGrid()
  : Node("local_hazard_grid"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/dlio/deskewed");
    output_map_topic_ = declare_parameter<std::string>("output_map_topic", "/local_hazard_map");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    resolution_ = declare_parameter<double>("resolution", 0.10);
    width_m_ = declare_parameter<double>("width_m", 10.0);
    height_m_ = declare_parameter<double>("height_m", 10.0);
    history_duration_sec_ = declare_parameter<double>("history_duration_sec", 2.0);
    max_history_points_ = declare_parameter<int>("max_history_points", 250000);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    min_obstacle_height_ = declare_parameter<double>("min_obstacle_height", 0.25);
    max_obstacle_height_ = declare_parameter<double>("max_obstacle_height", 1.20);
    terrain_min_height_ = declare_parameter<double>("terrain_min_height", -0.75);
    terrain_max_height_ = declare_parameter<double>("terrain_max_height", 1.20);
    terrain_slope_hazard_deg_ = declare_parameter<double>("terrain_slope_hazard_deg", 22.0);
    terrain_step_hazard_m_ = declare_parameter<double>("terrain_step_hazard_m", 0.25);
    terrain_min_points_per_cell_ = declare_parameter<int>("terrain_min_points_per_cell", 5);
    terrain_neighbor_radius_cells_ = declare_parameter<int>("terrain_neighbor_radius_cells", 2);
    min_points_per_obstacle_cell_ = declare_parameter<int>("min_points_per_obstacle_cell", 1);
    // A cell only confirms as hazard once points landing in it span at
    // least this much time -- damps single-scan noise blips (which only
    // ever span one instant) without meaningfully delaying detection of a
    // real, persistent hazard (which keeps getting returns scan after scan).
    hazard_confirm_sec_ = declare_parameter<double>("hazard_confirm_sec", 0.3);
    gradient_radius_m_ = declare_parameter<double>("gradient_radius_m", 0.20);
    // Inner ring around a hazard cell that is unconditionally lethal, no
    // falloff -- mirrors hazard_patch_node's hard_lethal_radius_m so the
    // local and global hazard costmaps agree on where the hard boundary is.
    hard_lethal_radius_m_ = declare_parameter<double>("hard_lethal_radius_m", 0.2);
    gradient_min_cost_ = declare_parameter<int>("gradient_min_cost", 8);
    // Ceiling for the soft part of the gradient -- see hazard_patch_node's
    // matching param for the rationale: must stay below occupied_value_ so
    // preference cost can never become a second lethal wall.
    soft_max_cost_ = declare_parameter<int>("soft_max_cost", 60);
    occupied_value_ = declare_parameter<int>("occupied_value", 100);
    free_value_ = declare_parameter<int>("free_value", 0);
    clear_robot_radius_m_ = declare_parameter<double>("clear_robot_radius_m", 1.0);
    max_input_range_m_ = declare_parameter<double>("max_input_range_m", 6.0);

    if (resolution_ <= 0.0) {
      throw std::runtime_error("resolution must be > 0");
    }
    width_cells_ = std::max(1, static_cast<int>(std::ceil(width_m_ / resolution_)));
    height_cells_ = std::max(1, static_cast<int>(std::ceil(height_m_ / resolution_)));
    history_duration_sec_ = std::max(0.1, history_duration_sec_);
    max_history_points_ = std::max(1000, max_history_points_);
    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
    terrain_slope_hazard_deg_ = std::clamp(terrain_slope_hazard_deg_, 0.0, 89.0);
    terrain_step_hazard_m_ = std::max(0.0, terrain_step_hazard_m_);
    terrain_min_points_per_cell_ = std::max(1, terrain_min_points_per_cell_);
    terrain_neighbor_radius_cells_ = std::max(1, terrain_neighbor_radius_cells_);
    min_points_per_obstacle_cell_ = std::max(1, min_points_per_obstacle_cell_);
    hazard_confirm_sec_ = std::clamp(hazard_confirm_sec_, 0.0, history_duration_sec_);
    gradient_radius_m_ = std::max(0.0, gradient_radius_m_);
    hard_lethal_radius_m_ = std::clamp(hard_lethal_radius_m_, 0.0, gradient_radius_m_);
    gradient_min_cost_ = std::clamp(gradient_min_cost_, 0, 99);
    occupied_value_ = std::clamp(occupied_value_, 1, 100);
    soft_max_cost_ = std::clamp(soft_max_cost_, gradient_min_cost_, occupied_value_ - 1);
    free_value_ = std::clamp(free_value_, 0, 100);

    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    // transient_local: occupancy_grid_to_points subscribes with transient_local
    // (needed for its /map use case), and a volatile publisher here is
    // incompatible with that -- DDS silently never connects them, which is
    // why /local_hazard_debug_points never gets data even though this node
    // is publishing fine. Upgrading to transient_local is always safe (it's
    // compatible with both transient_local and volatile subscribers).
    map_qos.transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_map_topic_, map_qos);
    diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
      "local_hazard_grid_timing_diagnostics", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalHazardGrid::receiveCloud, this, std::placeholders::_1));

    // publishGrid() (terrain-hazard neighbor search + gradient falloff) can
    // get genuinely expensive on complex terrain -- cost scales with hazard
    // cell count, which can spike well above the typical case. Running it
    // on its own thread (rather than a wall timer on the executor) means
    // that spike can never block receiveCloud() or anything else, matching
    // the pattern used everywhere else in this project.
    worker_ = std::thread(&LocalHazardGrid::workerLoop, this);

    RCLCPP_INFO(
      get_logger(), "Local hazard grid: %s -> %s frame=%s %.1fx%.1fm %.2fm/cell history=%.1fs",
      input_cloud_topic_.c_str(), output_map_topic_.c_str(), target_frame_.c_str(),
      width_m_, height_m_, resolution_, history_duration_sec_);
    RCLCPP_INFO(
      get_logger(),
      "max_input_range_m=%.1f is measured from the robot's pose in '%s' after transforming "
      "each point -- frame-independent regardless of whether '%s' publishes body-relative or "
      "world-frame clouds",
      max_input_range_m_, target_frame_.c_str(), input_cloud_topic_.c_str());
  }

  ~LocalHazardGrid() override
  {
    shutting_down_ = true;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void workerLoop()
  {
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    auto last_tick = std::chrono::steady_clock::now();
    while (!shutting_down_) {
      std::this_thread::sleep_for(period);
      if (shutting_down_) {
        break;
      }
      // This loop should tick every `period`. A gap much larger than that
      // can only mean the OS scheduler starved this thread (system-wide CPU
      // contention) -- report it immediately instead of leaving it to be
      // inferred later from timestamp gaps in a log file.
      const auto now = std::chrono::steady_clock::now();
      const double gap_sec = std::chrono::duration<double>(now - last_tick).count();
      last_tick = now;
      const double expected_sec = std::chrono::duration<double>(period).count();
      if (gap_sec > kStallWarnMultiplier * expected_sec) {
        RCLCPP_WARN(
          get_logger(),
          "local_hazard_grid worker loop stalled: expected ~%.2fs between ticks, actual "
          "gap was %.2fs -- likely CPU/executor starvation, not hazard logic",
          expected_sec, gap_sec);
      }
      publishGrid();
    }
  }

  static constexpr double kStallWarnMultiplier = 3.0;
  static constexpr double kMaxInputStalenessMultiplier = 3.0;

  void receiveCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    if (cloud->width == 0 || cloud->height == 0) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        target_frame_, cloud->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for transform %s -> %s: %s",
        target_frame_.c_str(), cloud->header.frame_id.c_str(), ex.what());
      return;
    }

    const rclcpp::Time stamp = cloud->header.stamp.sec == 0 && cloud->header.stamp.nanosec == 0 ?
      now() : rclcpp::Time(cloud->header.stamp);

    // max_input_range_m_ means "distance from the robot," which only equals
    // hypot(point.x, point.y) if the input cloud is body/sensor-relative.
    // Some sources (e.g. a LIO frontend's deskewed output) instead publish
    // already in a fixed world frame (cloud->header.frame_id == target_frame_,
    // e.g. "odom") -- in that case raw point coordinates are offsets from the
    // world origin, not the robot, and the pre-transform filter below would
    // silently become "distance from world origin." Transforming first and
    // filtering against the robot's actual pose makes this correct
    // regardless of which convention the input topic uses.
    const auto robot = lookupRobot();

    // Build locally (no lock needed) before taking points_mtx_ once for the
    // bulk append + prune -- keeps the lock held only briefly, since the
    // worker thread may be waiting on it to snapshot for publishGrid().
    std::vector<TimedPoint> new_points;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
      new_points.reserve(cloud->width * cloud->height);
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        Point3 point{*iter_x, *iter_y, *iter_z};
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
          continue;
        }
        point = transformPoint(point, transform);
        // If the robot's pose isn't available this callback, skip the range
        // filter rather than dropping the whole cloud -- a briefly oversized
        // deque is harmless (max_history_points_ bounds it), whereas
        // dropping data recreates the blindness this fix is for.
        if (max_input_range_m_ > 0.0 && robot.has_value() &&
          std::hypot(point.x - robot->x, point.y - robot->y) > max_input_range_m_)
        {
          continue;
        }
        new_points.push_back({point.x, point.y, point.z, stamp});
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input cloud is missing x/y/z fields: %s", ex.what());
      return;
    }

    const uint32_t raw_count = cloud->width * cloud->height;
    if (new_points.empty() && raw_count > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cloud in: raw=%u kept=0 -- filtering out every point is always pathological, check "
        "max_input_range_m_/frame assumptions",
        raw_count);
    }

    std::lock_guard<std::mutex> lock(points_mtx_);
    for (auto & point : new_points) {
      points_.push_back(std::move(point));
    }
    // Prune relative to the newest buffered stamp, not wall/sim now() --
    // keeps a genuine "last history_duration_sec of DATA" window that
    // survives pipeline latency spikes instead of evicting everything the
    // instant now() - stamp happens to exceed history_duration_sec. See
    // publishGrid() for the separate absolute-staleness guard that still
    // clears the buffer if input actually stops arriving.
    if (!points_.empty()) {
      pruneOldPoints(points_.back().stamp);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "cloud in: raw=%u kept=%zu deque=%zu stamp_age=%.2fs",
      raw_count, new_points.size(), points_.size(), (now() - stamp).seconds());
  }

  void publishGrid()
  {
    const auto total_start = std::chrono::steady_clock::now();
    const auto robot = lookupRobot();
    if (!robot.has_value()) {
      return;
    }

    std::deque<TimedPoint> points_snapshot;
    {
      std::lock_guard<std::mutex> lock(points_mtx_);
      if (!points_.empty()) {
        // Same sliding-window rationale as receiveCloud().
        pruneOldPoints(points_.back().stamp);
        // The sliding window alone would happily keep serving a frozen
        // snapshot forever if input actually stopped arriving (not just
        // arrived late). Since this grid re-projects relative to the
        // robot's CURRENT pose on every publish, stale points would
        // silently paint terrain that's no longer near the robot. Clear
        // the buffer once the newest data is old enough that it can only
        // mean the input pipeline stalled, not just network/DDS jitter.
        const double staleness_sec = (now() - points_.back().stamp).seconds();
        const double staleness_limit_sec = kMaxInputStalenessMultiplier * history_duration_sec_;
        if (staleness_sec > staleness_limit_sec) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "No fresh input for %.2fs (> %.2fs) -- clearing local hazard map",
            staleness_sec, staleness_limit_sec);
          points_.clear();
        }
      }
      points_snapshot = points_;
    }

    const double origin_x = robot->x - 0.5 * width_cells_ * resolution_;
    const double origin_y = robot->y - 0.5 * height_cells_ * resolution_;
    const size_t grid_size = static_cast<size_t>(width_cells_ * height_cells_);
    std::vector<int> obstacle_counts(grid_size, 0);
    std::vector<int> terrain_counts(grid_size, 0);
    std::vector<double> terrain_sum_z(grid_size, 0.0);
    std::vector<double> terrain_min_z(grid_size, std::numeric_limits<double>::infinity());
    std::vector<double> terrain_max_z(grid_size, -std::numeric_limits<double>::infinity());
    // Every publish recomputes hazards from scratch against whatever points
    // currently sit in the rolling window -- with no cross-publish state, a
    // single-scan cluster of spurious/noisy returns can paint a hazard cell
    // for one publish and vanish the next, and the controller reacts to
    // that instantaneous blip as a real obstacle. Track the observed time
    // span per cell so a cell only confirms as hazard once it's been seen
    // across multiple scans (hazard_confirm_sec_), not from one instant.
    std::vector<double> cell_min_stamp_sec(grid_size, std::numeric_limits<double>::infinity());
    std::vector<double> cell_max_stamp_sec(grid_size, -std::numeric_limits<double>::infinity());
    std::vector<std::pair<int, int>> hazard_cells;

    // Height bands are measured relative to the robot's current Z, not the
    // target_frame_'s absolute Z=0. odom's Z reference drifts over a session
    // (least-observable axis for a ground vehicle), but the robot's own Z
    // estimate and nearby terrain points drift together on the same
    // odometry chain -- so point.z - robot->z stays accurate regardless of
    // how far the absolute reference has wandered, without needing to
    // correct the drift itself.
    const auto binning_start = std::chrono::steady_clock::now();
    for (const auto & point : points_snapshot) {
      const int x = static_cast<int>(std::floor((point.x - origin_x) / resolution_));
      const int y = static_cast<int>(std::floor((point.y - origin_y) / resolution_));
      if (x < 0 || y < 0 || x >= width_cells_ || y >= height_cells_) {
        continue;
      }
      const size_t idx = static_cast<size_t>(y * width_cells_ + x);
      const double relative_z = point.z - robot->z;
      if (relative_z >= terrain_min_height_ && relative_z <= terrain_max_height_) {
        terrain_counts[idx] += 1;
        terrain_sum_z[idx] += relative_z;
        terrain_min_z[idx] = std::min(terrain_min_z[idx], relative_z);
        terrain_max_z[idx] = std::max(terrain_max_z[idx], relative_z);
      }
      if (relative_z >= min_obstacle_height_ && relative_z <= max_obstacle_height_) {
        obstacle_counts[idx] += 1;
      }
      const double point_stamp_sec = point.stamp.seconds();
      cell_min_stamp_sec[idx] = std::min(cell_min_stamp_sec[idx], point_stamp_sec);
      cell_max_stamp_sec[idx] = std::max(cell_max_stamp_sec[idx], point_stamp_sec);
    }
    const double binning_ms = elapsedMillis(binning_start);

    nav_msgs::msg::OccupancyGrid map;
    stampMap(map);
    map.info.resolution = static_cast<float>(resolution_);
    map.info.width = static_cast<uint32_t>(width_cells_);
    map.info.height = static_cast<uint32_t>(height_cells_);
    map.info.origin.position.x = origin_x;
    map.info.origin.position.y = origin_y;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(grid_size, static_cast<int8_t>(free_value_));

    const auto mark_hazard_start = std::chrono::steady_clock::now();
    for (int y = 0; y < height_cells_; ++y) {
      for (int x = 0; x < width_cells_; ++x) {
        const size_t idx = static_cast<size_t>(y * width_cells_ + x);
        const bool confirmed =
          cell_max_stamp_sec[idx] - cell_min_stamp_sec[idx] >= hazard_confirm_sec_;
        if (obstacle_counts[idx] >= min_points_per_obstacle_cell_ && confirmed) {
          markHazard(map, x, y, hazard_cells);
        }
      }
    }
    const double mark_hazard_ms = elapsedMillis(mark_hazard_start);

    const auto terrain_start = std::chrono::steady_clock::now();
    markTerrainHazards(
      map, terrain_counts, terrain_sum_z, terrain_min_z, terrain_max_z, cell_min_stamp_sec,
      cell_max_stamp_sec, hazard_cells);
    const double terrain_ms = elapsedMillis(terrain_start);

    const auto gradient_start = std::chrono::steady_clock::now();
    applyGradient(map, hazard_cells);
    const double gradient_ms = elapsedMillis(gradient_start);

    clearRobot(map, origin_x, origin_y, robot->x, robot->y);
    map_pub_->publish(map);

    {
      std::ostringstream diag;
      diag << std::fixed << std::setprecision(3)
           << "{\"event\":\"local_hazard_grid_timing\""
           << ",\"num_points\":" << points_snapshot.size()
           << ",\"hazards\":" << hazard_cells.size()
           << ",\"binning_ms\":" << binning_ms
           << ",\"mark_hazard_ms\":" << mark_hazard_ms
           // terrain_ms is the neighbor-slope search -- cost scales with
           // terrain_neighbor_radius_cells^2 per cell.
           << ",\"terrain_ms\":" << terrain_ms
           // gradient_ms scales with hazard_cells.size() * gradient_radius^2
           // -- this is the one most likely to spike on complex terrain.
           << ",\"gradient_ms\":" << gradient_ms
           << ",\"total_ms\":" << elapsedMillis(total_start)
           << "}";
      std_msgs::msg::String diag_msg;
      diag_msg.data = diag.str();
      diagnostics_pub_->publish(diag_msg);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Published /local_hazard_map from %zu recent points; hazards=%zu",
      points_snapshot.size(), hazard_cells.size());
  }

  std::optional<Point3> lookupRobot()
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(target_frame_, robot_frame_, tf2::TimePointZero);
      return Point3{
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z};
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for robot transform %s -> %s: %s",
        target_frame_.c_str(), robot_frame_.c_str(), ex.what());
      return std::nullopt;
    }
  }

  void pruneOldPoints(const rclcpp::Time & stamp)
  {
    const auto cutoff = stamp - rclcpp::Duration::from_seconds(history_duration_sec_);
    while (!points_.empty() && points_.front().stamp < cutoff) {
      points_.pop_front();
    }
    while (static_cast<int>(points_.size()) > max_history_points_) {
      points_.pop_front();
    }
  }

  void markHazard(
    nav_msgs::msg::OccupancyGrid & map,
    const int x,
    const int y,
    std::vector<std::pair<int, int>> & hazard_cells) const
  {
    if (x < 0 || y < 0 || x >= width_cells_ || y >= height_cells_) {
      return;
    }
    const size_t idx = static_cast<size_t>(y * width_cells_ + x);
    if (map.data[idx] < occupied_value_) {
      map.data[idx] = static_cast<int8_t>(occupied_value_);
      hazard_cells.emplace_back(x, y);
    }
  }

  void markTerrainHazards(
    nav_msgs::msg::OccupancyGrid & map,
    const std::vector<int> & terrain_counts,
    const std::vector<double> & terrain_sum_z,
    const std::vector<double> & terrain_min_z,
    const std::vector<double> & terrain_max_z,
    const std::vector<double> & cell_min_stamp_sec,
    const std::vector<double> & cell_max_stamp_sec,
    std::vector<std::pair<int, int>> & hazard_cells) const
  {
    std::vector<double> mean_z(
      static_cast<size_t>(width_cells_ * height_cells_),
      std::numeric_limits<double>::quiet_NaN());
    for (int y = 0; y < height_cells_; ++y) {
      for (int x = 0; x < width_cells_; ++x) {
        const size_t idx = static_cast<size_t>(y * width_cells_ + x);
        if (terrain_counts[idx] >= terrain_min_points_per_cell_) {
          mean_z[idx] = terrain_sum_z[idx] / static_cast<double>(terrain_counts[idx]);
        }
      }
    }

    constexpr double pi = 3.14159265358979323846;
    const double slope_threshold = std::tan(terrain_slope_hazard_deg_ * pi / 180.0);
    for (int y = 0; y < height_cells_; ++y) {
      for (int x = 0; x < width_cells_; ++x) {
        const size_t idx = static_cast<size_t>(y * width_cells_ + x);
        if (!std::isfinite(mean_z[idx])) {
          continue;
        }
        bool hazard = terrain_max_z[idx] - terrain_min_z[idx] >= terrain_step_hazard_m_;
        for (int dy = -terrain_neighbor_radius_cells_; !hazard && dy <= terrain_neighbor_radius_cells_; ++dy) {
          for (int dx = -terrain_neighbor_radius_cells_; dx <= terrain_neighbor_radius_cells_; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= width_cells_ || ny >= height_cells_) {
              continue;
            }
            const size_t neighbor_idx = static_cast<size_t>(ny * width_cells_ + nx);
            if (!std::isfinite(mean_z[neighbor_idx])) {
              continue;
            }
            const double horizontal_distance =
              resolution_ * std::hypot(static_cast<double>(dx), static_cast<double>(dy));
            const double slope = std::abs(mean_z[idx] - mean_z[neighbor_idx]) / horizontal_distance;
            if (slope >= slope_threshold) {
              hazard = true;
              break;
            }
          }
        }
        const bool confirmed =
          cell_max_stamp_sec[idx] - cell_min_stamp_sec[idx] >= hazard_confirm_sec_;
        if (hazard && confirmed) {
          markHazard(map, x, y, hazard_cells);
        }
      }
    }
  }

  void applyGradient(
    nav_msgs::msg::OccupancyGrid & map,
    const std::vector<std::pair<int, int>> & hazard_cells) const
  {
    const int radius_cells = static_cast<int>(std::ceil(gradient_radius_m_ / resolution_));
    if (radius_cells <= 0) {
      return;
    }
    // See hazard_patch_node's applyGradient for the rationale: cells within
    // hard_lethal_radius_cells are unconditionally lethal (a hard boundary
    // the planner cannot route through), and the soft gradient only shapes
    // the remaining span out to gradient_radius_m.
    const int hard_lethal_radius_cells =
      static_cast<int>(std::ceil(hard_lethal_radius_m_ / resolution_));
    const int soft_span_cells = std::max(1, radius_cells - hard_lethal_radius_cells);
    for (const auto & hazard : hazard_cells) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const double distance = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
          if (distance > radius_cells) {
            continue;
          }
          const int x = hazard.first + dx;
          const int y = hazard.second + dy;
          if (x < 0 || y < 0 || x >= width_cells_ || y >= height_cells_) {
            continue;
          }
          const size_t idx = static_cast<size_t>(y * width_cells_ + x);
          if (map.data[idx] >= occupied_value_) {
            continue;
          }
          int cost;
          int cost_max;
          if (distance <= hard_lethal_radius_cells) {
            cost = occupied_value_;
            cost_max = occupied_value_;
          } else {
            // Soft band tops out at soft_max_cost_, not occupied_value_ --
            // see hazard_patch_node's applyGradient for why: preference cost
            // must stay strictly below what the local costmap treats as
            // occupied, or it silently becomes a second lethal wall.
            const double soft_distance = distance - hard_lethal_radius_cells;
            const double falloff = 1.0 - soft_distance / static_cast<double>(soft_span_cells);
            cost = gradient_min_cost_ +
              static_cast<int>(std::round((soft_max_cost_ - gradient_min_cost_) * falloff));
            cost_max = soft_max_cost_;
          }
          map.data[idx] = static_cast<int8_t>(
            std::max(static_cast<int>(map.data[idx]), std::clamp(cost, gradient_min_cost_, cost_max)));
        }
      }
    }
  }

  void clearRobot(
    nav_msgs::msg::OccupancyGrid & map,
    const double origin_x,
    const double origin_y,
    const double robot_x,
    const double robot_y) const
  {
    const int radius_cells = static_cast<int>(std::ceil(clear_robot_radius_m_ / resolution_));
    const int center_x = static_cast<int>(std::floor((robot_x - origin_x) / resolution_));
    const int center_y = static_cast<int>(std::floor((robot_y - origin_y) / resolution_));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int x = center_x + dx;
        const int y = center_y + dy;
        if (x < 0 || y < 0 || x >= width_cells_ || y >= height_cells_) {
          continue;
        }
        map.data[static_cast<size_t>(y * width_cells_ + x)] =
          static_cast<int8_t>(free_value_);
      }
    }
  }

  void stampMap(nav_msgs::msg::OccupancyGrid & map)
  {
    const int64_t now_ns = now().nanoseconds();
    map.header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
    map.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
    map.header.frame_id = target_frame_;
    map.info.map_load_time = map.header.stamp;
  }

  std::string input_cloud_topic_;
  std::string output_map_topic_;
  std::string target_frame_;
  std::string robot_frame_;
  double resolution_ {};
  double width_m_ {};
  double height_m_ {};
  double history_duration_sec_ {};
  int max_history_points_ {};
  double publish_rate_hz_ {};
  double min_obstacle_height_ {};
  double max_obstacle_height_ {};
  double terrain_min_height_ {};
  double terrain_max_height_ {};
  double terrain_slope_hazard_deg_ {};
  double terrain_step_hazard_m_ {};
  int terrain_min_points_per_cell_ {};
  int terrain_neighbor_radius_cells_ {};
  int min_points_per_obstacle_cell_ {};
  double hazard_confirm_sec_ {};
  double gradient_radius_m_ {};
  double hard_lethal_radius_m_ {};
  int gradient_min_cost_ {};
  int soft_max_cost_ {};
  int occupied_value_ {};
  int free_value_ {};
  double clear_robot_radius_m_ {};
  double max_input_range_m_ {};
  int width_cells_ {};
  int height_cells_ {};
  std::deque<TimedPoint> points_;
  std::mutex points_mtx_;

  std::thread worker_;
  std::atomic<bool> shutting_down_ {false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalHazardGrid>());
  rclcpp::shutdown();
  return 0;
}
