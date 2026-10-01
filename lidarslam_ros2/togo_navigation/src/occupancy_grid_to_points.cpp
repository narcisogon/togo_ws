#include <cmath>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

class OccupancyGridToPoints : public rclcpp::Node
{
public:
  OccupancyGridToPoints()
  : Node("occupancy_grid_to_points")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/map");
    output_topic_ = declare_parameter<std::string>("output_topic", "/map_debug_points");
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
    include_unknown_ = declare_parameter<bool>("include_unknown", false);
    point_z_ = declare_parameter<double>("point_z", 0.08);
    unknown_point_z_ = declare_parameter<double>("unknown_point_z", 0.02);
    publish_every_n_ = std::max(1, static_cast<int>(declare_parameter<int>("publish_every_n", 1)));

    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topic_, qos,
      std::bind(&OccupancyGridToPoints::receiveMap, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "OccupancyGrid visualizer: %s -> %s threshold=%d",
      input_topic_.c_str(), output_topic_.c_str(), occupied_threshold_);
  }

private:
  struct Point
  {
    float x;
    float y;
    float z;
    float intensity;
  };

  void receiveMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    ++message_count_;
    if (message_count_ % publish_every_n_ != 0) {
      return;
    }

    const auto width = static_cast<int>(map->info.width);
    const auto height = static_cast<int>(map->info.height);
    if (width <= 0 || height <= 0 || map->data.empty()) {
      return;
    }

    const double resolution = map->info.resolution;
    const double origin_x = map->info.origin.position.x;
    const double origin_y = map->info.origin.position.y;
    std::vector<Point> points;
    points.reserve(map->data.size() / 10);

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int8_t value = map->data[static_cast<size_t>(y * width + x)];
        if (value >= occupied_threshold_) {
          points.push_back(
            Point{
              static_cast<float>(origin_x + (x + 0.5) * resolution),
              static_cast<float>(origin_y + (y + 0.5) * resolution),
              static_cast<float>(point_z_),
              100.0F});
        } else if (include_unknown_ && value < 0) {
          points.push_back(
            Point{
              static_cast<float>(origin_x + (x + 0.5) * resolution),
              static_cast<float>(origin_y + (y + 0.5) * resolution),
              static_cast<float>(unknown_point_z_),
              20.0F});
        }
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = map->header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 16;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(4);
    setField(cloud.fields[0], "x", 0);
    setField(cloud.fields[1], "y", 4);
    setField(cloud.fields[2], "z", 8);
    setField(cloud.fields[3], "intensity", 12);
    cloud.data.resize(static_cast<size_t>(cloud.row_step));

    auto * data = cloud.data.data();
    for (const auto & point : points) {
      writeFloat(data, point.x);
      writeFloat(data + 4, point.y);
      writeFloat(data + 8, point.z);
      writeFloat(data + 12, point.intensity);
      data += cloud.point_step;
    }

    pub_->publish(cloud);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Published %zu occupancy debug points from %s", points.size(), input_topic_.c_str());
  }

  static void setField(sensor_msgs::msg::PointField & field, const std::string & name, const uint32_t offset)
  {
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
  }

  static void writeFloat(uint8_t * destination, const float value)
  {
    std::memcpy(destination, &value, sizeof(float));
  }

  std::string input_topic_;
  std::string output_topic_;
  int occupied_threshold_ {};
  bool include_unknown_ {};
  double point_z_ {};
  double unknown_point_z_ {};
  int publish_every_n_ {1};
  uint64_t message_count_ {};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridToPoints>());
  rclcpp::shutdown();
  return 0;
}
