#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

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

}  // namespace

class SlamToOccupancyGrid : public rclcpp::Node
{
public:
  SlamToOccupancyGrid()
  : Node("slam_to_occupancy_grid"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/modified_map");
    output_map_topic_ = declare_parameter<std::string>("output_map_topic", "/map");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    resolution_ = declare_parameter<double>("resolution", 0.20);
    width_m_ = declare_parameter<double>("width_m", 80.0);
    height_m_ = declare_parameter<double>("height_m", 80.0);
    origin_x_ = declare_parameter<double>("origin_x", -0.5 * width_m_);
    origin_y_ = declare_parameter<double>("origin_y", -0.5 * height_m_);
    min_obstacle_height_ = declare_parameter<double>("min_obstacle_height", -0.15);
    max_obstacle_height_ = declare_parameter<double>("max_obstacle_height", 1.20);
    min_points_per_cell_ = declare_parameter<int>("min_points_per_cell", 1);
    occupied_value_ = declare_parameter<int>("occupied_value", 100);
    free_value_ = declare_parameter<int>("free_value", 0);
    unknown_value_ = declare_parameter<int>("unknown_value", -1);
    initialize_as_free_ = declare_parameter<bool>("initialize_as_free", true);
    obstacle_dilation_cells_ = declare_parameter<int>("obstacle_dilation_cells", 1);
    enable_gradient_costs_ = declare_parameter<bool>("enable_gradient_costs", false);
    gradient_radius_m_ = declare_parameter<double>("gradient_radius_m", 0.8);
    gradient_min_cost_ = declare_parameter<int>("gradient_min_cost", 5);
    gradient_power_ = declare_parameter<double>("gradient_power", 1.0);
    enable_terrain_hazards_ = declare_parameter<bool>("enable_terrain_hazards", false);
    terrain_min_height_ = declare_parameter<double>("terrain_min_height", -1.0);
    terrain_max_height_ = declare_parameter<double>("terrain_max_height", 1.2);
    terrain_slope_hazard_deg_ = declare_parameter<double>("terrain_slope_hazard_deg", 15.0);
    terrain_step_hazard_m_ = declare_parameter<double>("terrain_step_hazard_m", 0.18);
    terrain_min_points_per_cell_ = declare_parameter<int>("terrain_min_points_per_cell", 3);
    terrain_neighbor_radius_cells_ = declare_parameter<int>("terrain_neighbor_radius_cells", 1);
    terrain_hazard_dilation_cells_ = declare_parameter<int>("terrain_hazard_dilation_cells", 0);
    max_input_range_m_ = declare_parameter<double>("max_input_range_m", 120.0);
    clear_robot_radius_m_ = declare_parameter<double>("clear_robot_radius_m", 1.0);
    enable_robot_trail_clearing_ = declare_parameter<bool>("enable_robot_trail_clearing", false);
    robot_trail_clear_radius_m_ = declare_parameter<double>("robot_trail_clear_radius_m", 0.9);
    robot_trail_max_poses_ = declare_parameter<int>("robot_trail_max_poses", 300);
    robot_trail_min_distance_m_ = declare_parameter<double>("robot_trail_min_distance_m", 0.15);
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    center_map_on_robot_ = declare_parameter<bool>("center_map_on_robot", true);
    publish_empty_map_until_first_cloud_ =
      declare_parameter<bool>("publish_empty_map_until_first_cloud", true);

    if (resolution_ <= 0.0) {
      throw std::runtime_error("resolution must be > 0");
    }
    width_cells_ = std::max(1, static_cast<int>(std::ceil(width_m_ / resolution_)));
    height_cells_ = std::max(1, static_cast<int>(std::ceil(height_m_ / resolution_)));
    min_points_per_cell_ = std::max(1, min_points_per_cell_);
    obstacle_dilation_cells_ = std::max(0, obstacle_dilation_cells_);
    gradient_radius_m_ = std::max(0.0, gradient_radius_m_);
    gradient_min_cost_ = std::clamp(gradient_min_cost_, 0, 99);
    gradient_power_ = std::max(0.1, gradient_power_);
    terrain_slope_hazard_deg_ = std::clamp(terrain_slope_hazard_deg_, 0.0, 89.0);
    terrain_step_hazard_m_ = std::max(0.0, terrain_step_hazard_m_);
    terrain_min_points_per_cell_ = std::max(1, terrain_min_points_per_cell_);
    terrain_neighbor_radius_cells_ = std::max(1, terrain_neighbor_radius_cells_);
    terrain_hazard_dilation_cells_ = std::max(0, terrain_hazard_dilation_cells_);
    occupied_value_ = std::clamp(occupied_value_, 1, 100);
    free_value_ = std::clamp(free_value_, 0, 100);
    clear_robot_radius_m_ = std::max(0.0, clear_robot_radius_m_);
    robot_trail_clear_radius_m_ = std::max(0.0, robot_trail_clear_radius_m_);
    robot_trail_max_poses_ = std::max(1, robot_trail_max_poses_);
    robot_trail_min_distance_m_ = std::max(0.0, robot_trail_min_distance_m_);

    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    map_qos.transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_map_topic_, map_qos);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&SlamToOccupancyGrid::receiveCloud, this, std::placeholders::_1));
    empty_map_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&SlamToOccupancyGrid::publishEmptyMapIfNeeded, this));

    RCLCPP_INFO(
      get_logger(),
      "SLAM-to-OccupancyGrid: %s -> %s frame=%s size=%dx%d resolution=%.3f origin=(%.2f, %.2f)",
      input_cloud_topic_.c_str(), output_map_topic_.c_str(), target_frame_.c_str(),
      width_cells_, height_cells_, resolution_, origin_x_, origin_y_);
  }

private:
  void receiveCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    if (cloud->width == 0 || cloud->height == 0) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    const bool needs_transform =
      !cloud->header.frame_id.empty() && cloud->header.frame_id != target_frame_;
    if (needs_transform) {
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
    }

    auto [map_origin_x, map_origin_y] = getMapOrigin();

    std::vector<int> counts(static_cast<size_t>(width_cells_ * height_cells_), 0);
    std::vector<int> terrain_counts(static_cast<size_t>(width_cells_ * height_cells_), 0);
    std::vector<double> terrain_sum_z(static_cast<size_t>(width_cells_ * height_cells_), 0.0);
    std::vector<double> terrain_min_z(
      static_cast<size_t>(width_cells_ * height_cells_),
      std::numeric_limits<double>::infinity());
    std::vector<double> terrain_max_z(
      static_cast<size_t>(width_cells_ * height_cells_),
      -std::numeric_limits<double>::infinity());
    std::vector<std::pair<int, int>> obstacle_cells;
    size_t accepted_points = 0;
    size_t terrain_points = 0;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        Point3 point{*iter_x, *iter_y, *iter_z};
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
          continue;
        }
        if (needs_transform) {
          point = transformPoint(point, transform);
        }
        if (max_input_range_m_ > 0.0 && std::hypot(point.x, point.y) > max_input_range_m_) {
          continue;
        }

        const int x = static_cast<int>(std::floor((point.x - map_origin_x) / resolution_));
        const int y = static_cast<int>(std::floor((point.y - map_origin_y) / resolution_));
        if (x < 0 || y < 0 || x >= width_cells_ || y >= height_cells_) {
          continue;
        }
        const size_t idx = static_cast<size_t>(y * width_cells_ + x);

        if (enable_terrain_hazards_ &&
          point.z >= terrain_min_height_ && point.z <= terrain_max_height_)
        {
          terrain_counts[idx] += 1;
          terrain_sum_z[idx] += point.z;
          terrain_min_z[idx] = std::min(terrain_min_z[idx], point.z);
          terrain_max_z[idx] = std::max(terrain_max_z[idx], point.z);
          ++terrain_points;
        }

        if (point.z < min_obstacle_height_ || point.z > max_obstacle_height_) {
          continue;
        }
        counts[idx] += 1;
        ++accepted_points;
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input cloud is missing x/y/z fields: %s", ex.what());
      return;
    }

    nav_msgs::msg::OccupancyGrid map;
    stampMap(map);
    map.info.resolution = static_cast<float>(resolution_);
    map.info.width = static_cast<uint32_t>(width_cells_);
    map.info.height = static_cast<uint32_t>(height_cells_);
    map.info.origin.position.x = map_origin_x;
    map.info.origin.position.y = map_origin_y;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(
      static_cast<size_t>(width_cells_ * height_cells_),
      static_cast<int8_t>(initialize_as_free_ ? free_value_ : unknown_value_));

    size_t occupied_cells = 0;
    for (int y = 0; y < height_cells_; ++y) {
      for (int x = 0; x < width_cells_; ++x) {
        const size_t idx = static_cast<size_t>(y * width_cells_ + x);
        if (counts[idx] < min_points_per_cell_) {
          continue;
        }
        markHazardCore(map, x, y, obstacle_cells, obstacle_dilation_cells_);
        ++occupied_cells;
      }
    }

    const size_t terrain_hazard_cells = enable_terrain_hazards_ ?
      markTerrainHazards(
        map, terrain_counts, terrain_sum_z, terrain_min_z, terrain_max_z, obstacle_cells) :
      0;

    if (enable_gradient_costs_) {
      applyGradientCosts(map, obstacle_cells);
    }
    updateRobotTrail();
    clearRobotTrail(map, map_origin_x, map_origin_y);
    clearRobotFootprint(map, map_origin_x, map_origin_y);
    map_pub_->publish(map);
    received_cloud_ = true;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Published /map from %zu obstacle points and %zu terrain points; occupied_cells=%zu terrain_hazards=%zu",
      accepted_points, terrain_points, occupied_cells, terrain_hazard_cells);
  }

  void markHazardCore(
    nav_msgs::msg::OccupancyGrid & map,
    const int center_x,
    const int center_y,
    std::vector<std::pair<int, int>> & obstacle_cells,
    const int dilation_cells) const
  {
    for (int dy = -dilation_cells; dy <= dilation_cells; ++dy) {
      for (int dx = -dilation_cells; dx <= dilation_cells; ++dx) {
        if (dx * dx + dy * dy >
          dilation_cells * dilation_cells)
        {
          continue;
        }
        const int x = center_x + dx;
        const int y = center_y + dy;
        if (x < 0 || y < 0 || x >= width_cells_ || y >= height_cells_) {
          continue;
        }
        const size_t idx = static_cast<size_t>(y * width_cells_ + x);
        if (map.data[idx] < occupied_value_) {
          map.data[idx] = static_cast<int8_t>(occupied_value_);
          obstacle_cells.emplace_back(x, y);
        }
      }
    }
  }

  size_t markTerrainHazards(
    nav_msgs::msg::OccupancyGrid & map,
    const std::vector<int> & terrain_counts,
    const std::vector<double> & terrain_sum_z,
    const std::vector<double> & terrain_min_z,
    const std::vector<double> & terrain_max_z,
    std::vector<std::pair<int, int>> & obstacle_cells) const
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
    size_t hazard_cells = 0;
    for (int y = 0; y < height_cells_; ++y) {
      for (int x = 0; x < width_cells_; ++x) {
        const size_t idx = static_cast<size_t>(y * width_cells_ + x);
        if (!std::isfinite(mean_z[idx])) {
          continue;
        }

        bool hazard = false;
        if (terrain_max_z[idx] - terrain_min_z[idx] >= terrain_step_hazard_m_) {
          hazard = true;
        }

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
            if (horizontal_distance <= 0.0) {
              continue;
            }
            const double slope = std::abs(mean_z[idx] - mean_z[neighbor_idx]) / horizontal_distance;
            if (slope >= slope_threshold) {
              hazard = true;
              break;
            }
          }
        }

        if (hazard) {
          markHazardCore(map, x, y, obstacle_cells, terrain_hazard_dilation_cells_);
          ++hazard_cells;
        }
      }
    }
    return hazard_cells;
  }

  void applyGradientCosts(
    nav_msgs::msg::OccupancyGrid & map,
    const std::vector<std::pair<int, int>> & obstacle_cells) const
  {
    const int radius_cells = static_cast<int>(std::ceil(gradient_radius_m_ / resolution_));
    if (radius_cells <= 0 || obstacle_cells.empty()) {
      return;
    }

    for (const auto & [center_x, center_y] : obstacle_cells) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const double distance_cells = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
          if (distance_cells > radius_cells) {
            continue;
          }

          const int x = center_x + dx;
          const int y = center_y + dy;
          if (x < 0 || y < 0 || x >= width_cells_ || y >= height_cells_) {
            continue;
          }

          const size_t idx = static_cast<size_t>(y * width_cells_ + x);
          if (map.data[idx] < 0 || map.data[idx] >= occupied_value_) {
            continue;
          }

          const double normalized_distance = distance_cells / static_cast<double>(radius_cells);
          const double falloff = std::pow(1.0 - normalized_distance, gradient_power_);
          const int cost = gradient_min_cost_ +
            static_cast<int>(std::round((occupied_value_ - gradient_min_cost_) * falloff));
          map.data[idx] = static_cast<int8_t>(
            std::max(static_cast<int>(map.data[idx]), std::clamp(cost, gradient_min_cost_, occupied_value_)));
        }
      }
    }
  }

  void publishEmptyMapIfNeeded()
  {
    if (!publish_empty_map_until_first_cloud_ || received_cloud_) {
      return;
    }

    auto [map_origin_x, map_origin_y] = getMapOrigin();
    nav_msgs::msg::OccupancyGrid map;
    stampMap(map);
    map.info.resolution = static_cast<float>(resolution_);
    map.info.width = static_cast<uint32_t>(width_cells_);
    map.info.height = static_cast<uint32_t>(height_cells_);
    map.info.origin.position.x = map_origin_x;
    map.info.origin.position.y = map_origin_y;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(
      static_cast<size_t>(width_cells_ * height_cells_),
      static_cast<int8_t>(free_value_));
    updateRobotTrail();
    clearRobotTrail(map, map_origin_x, map_origin_y);
    clearRobotFootprint(map, map_origin_x, map_origin_y);
    map_pub_->publish(map);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Publishing centered empty /map while waiting for %s", input_cloud_topic_.c_str());
  }

  std::pair<double, double> getMapOrigin()
  {
    if (!center_map_on_robot_) {
      return {origin_x_, origin_y_};
    }

    try {
      const auto robot_transform =
        tf_buffer_.lookupTransform(target_frame_, robot_frame_, tf2::TimePointZero);
      return {
        robot_transform.transform.translation.x - 0.5 * width_cells_ * resolution_,
        robot_transform.transform.translation.y - 0.5 * height_cells_ * resolution_};
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Could not center map on robot; using configured origin. Waiting for %s -> %s: %s",
        target_frame_.c_str(), robot_frame_.c_str(), ex.what());
      return {origin_x_, origin_y_};
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

  void clearRobotFootprint(
    nav_msgs::msg::OccupancyGrid & map,
    const double map_origin_x,
    const double map_origin_y)
  {
    if (clear_robot_radius_m_ <= 0.0) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(target_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Could not clear robot footprint; waiting for %s -> %s: %s",
        target_frame_.c_str(), robot_frame_.c_str(), ex.what());
      return;
    }

    const int center_x =
      static_cast<int>(std::floor((transform.transform.translation.x - map_origin_x) / resolution_));
    const int center_y =
      static_cast<int>(std::floor((transform.transform.translation.y - map_origin_y) / resolution_));
    const int radius_cells = static_cast<int>(std::ceil(clear_robot_radius_m_ / resolution_));

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

  void updateRobotTrail()
  {
    if (!enable_robot_trail_clearing_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(target_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Could not update robot trail; waiting for %s -> %s: %s",
        target_frame_.c_str(), robot_frame_.c_str(), ex.what());
      return;
    }

    const double x = transform.transform.translation.x;
    const double y = transform.transform.translation.y;
    if (!robot_trail_.empty()) {
      const auto & last = robot_trail_.back();
      if (std::hypot(x - last.first, y - last.second) < robot_trail_min_distance_m_) {
        return;
      }
    }

    robot_trail_.emplace_back(x, y);
    while (static_cast<int>(robot_trail_.size()) > robot_trail_max_poses_) {
      robot_trail_.erase(robot_trail_.begin());
    }
  }

  void clearRobotTrail(
    nav_msgs::msg::OccupancyGrid & map,
    const double map_origin_x,
    const double map_origin_y)
  {
    if (!enable_robot_trail_clearing_ || robot_trail_clear_radius_m_ <= 0.0) {
      return;
    }

    const int radius_cells = static_cast<int>(std::ceil(robot_trail_clear_radius_m_ / resolution_));
    for (const auto & pose : robot_trail_) {
      const int center_x =
        static_cast<int>(std::floor((pose.first - map_origin_x) / resolution_));
      const int center_y =
        static_cast<int>(std::floor((pose.second - map_origin_y) / resolution_));
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
  }

  std::string input_cloud_topic_;
  std::string output_map_topic_;
  std::string target_frame_;
  double resolution_ {};
  double width_m_ {};
  double height_m_ {};
  double origin_x_ {};
  double origin_y_ {};
  double min_obstacle_height_ {};
  double max_obstacle_height_ {};
  int min_points_per_cell_ {};
  int occupied_value_ {};
  int free_value_ {};
  int unknown_value_ {};
  bool initialize_as_free_ {};
  int obstacle_dilation_cells_ {};
  bool enable_gradient_costs_ {};
  double gradient_radius_m_ {};
  int gradient_min_cost_ {};
  double gradient_power_ {};
  bool enable_terrain_hazards_ {};
  double terrain_min_height_ {};
  double terrain_max_height_ {};
  double terrain_slope_hazard_deg_ {};
  double terrain_step_hazard_m_ {};
  int terrain_min_points_per_cell_ {};
  int terrain_neighbor_radius_cells_ {};
  int terrain_hazard_dilation_cells_ {};
  double max_input_range_m_ {};
  double clear_robot_radius_m_ {};
  bool enable_robot_trail_clearing_ {};
  double robot_trail_clear_radius_m_ {};
  int robot_trail_max_poses_ {};
  double robot_trail_min_distance_m_ {};
  std::vector<std::pair<double, double>> robot_trail_;
  std::string robot_frame_;
  bool center_map_on_robot_ {true};
  bool publish_empty_map_until_first_cloud_ {true};
  bool received_cloud_ {false};
  int width_cells_ {};
  int height_cells_ {};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr empty_map_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamToOccupancyGrid>());
  rclcpp::shutdown();
  return 0;
}
