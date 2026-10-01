#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{

constexpr uint32_t FLOAT32 = sensor_msgs::msg::PointField::FLOAT32;
constexpr uint32_t UINT16 = sensor_msgs::msg::PointField::UINT16;
constexpr uint32_t UINT32 = sensor_msgs::msg::PointField::UINT32;

sensor_msgs::msg::PointField makeField(
  const std::string & name, const uint32_t offset, const uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

template<typename T>
T readValue(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const sensor_msgs::msg::PointField * field,
  const size_t point_index,
  const T fallback)
{
  if (field == nullptr) {
    return fallback;
  }
  const size_t offset = point_index * cloud.point_step + field->offset;
  if (offset + sizeof(T) > cloud.data.size()) {
    return fallback;
  }
  T value {};
  std::memcpy(&value, cloud.data.data() + offset, sizeof(T));
  return value;
}

template<typename T>
void writeValue(sensor_msgs::msg::PointCloud2 & cloud, const size_t offset, const T value)
{
  std::memcpy(cloud.data.data() + offset, &value, sizeof(T));
}

uint16_t intensityToReflectivity(const float intensity)
{
  if (!std::isfinite(intensity) || intensity <= 0.0F) {
    return 0U;
  }
  return static_cast<uint16_t>(
    std::clamp(std::lround(intensity), 0L, static_cast<long>(std::numeric_limits<uint16_t>::max())));
}

}  // namespace

class SeyondCloudTimeAdapter final : public rclcpp::Node
{
public:
  SeyondCloudTimeAdapter()
  : Node("seyond_cloud_time_adapter")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_cloud_topic", "/a300_0000/sensors/seyond_robin_w/scan/points");
    output_topic_ = declare_parameter<std::string>(
      "output_cloud_topic", "/a300_0000/sensors/seyond_robin_w/scan/points_timed");
    scan_period_sec_ = declare_parameter<double>("scan_period_sec", 1.0 / 15.0);
    reverse_column_time_ = declare_parameter<bool>("reverse_column_time", false);
    stamp_at_scan_start_ = declare_parameter<bool>("stamp_at_scan_start", true);
    drop_non_increasing_stamps_ = declare_parameter<bool>("drop_non_increasing_stamps", true);

    if (scan_period_sec_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "scan_period_sec must be positive; using 1/15 s");
      scan_period_sec_ = 1.0 / 15.0;
    }

    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();
    qos.durability_volatile();

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    diagnostics_publisher_ = create_publisher<std_msgs::msg::String>(
      "~/diagnostics", rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&SeyondCloudTimeAdapter::receiveCloud, this, std::placeholders::_1));
    last_receive_time_ = std::chrono::steady_clock::now();
    last_publish_time_ = last_receive_time_;
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&SeyondCloudTimeAdapter::publishDiagnostics, this));

    RCLCPP_INFO(
      get_logger(),
      "Adding synthetic per-column t field: %s -> %s, scan_period=%.6f sec",
      input_topic_.c_str(), output_topic_.c_str(), scan_period_sec_);
  }

private:
  void receiveCloud(const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    ++received_clouds_;
    last_receive_time_ = std::chrono::steady_clock::now();
    if (input->width == 0 || input->height == 0 || input->point_step == 0) {
      ++malformed_clouds_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Skipping malformed cloud");
      return;
    }

    const double input_stamp = rclcpp::Time(input->header.stamp).seconds();
    if (drop_non_increasing_stamps_ && last_output_stamp_ > 0.0 && input_stamp <= last_output_stamp_) {
      ++dropped_non_increasing_stamps_;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Dropping non-increasing cloud stamp before timing adapter: %.9f <= %.9f (dropped=%zu)",
        input_stamp,
        last_output_stamp_,
        dropped_non_increasing_stamps_);
      return;
    }
    last_output_stamp_ = input_stamp;

    const auto * x_field = findField(*input, "x");
    const auto * y_field = findField(*input, "y");
    const auto * z_field = findField(*input, "z");
    const auto * intensity_field = findField(*input, "intensity");
    const auto * ring_field = findField(*input, "ring");

    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
      ++malformed_clouds_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Input cloud has no XYZ fields");
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = input->header;
    if (!stamp_at_scan_start_) {
      const auto stamp = rclcpp::Time(input->header.stamp) -
        rclcpp::Duration::from_seconds(scan_period_sec_);
      output.header.stamp = stamp;
    }
    output.height = input->height;
    output.width = input->width;
    output.is_bigendian = false;
    output.is_dense = input->is_dense;
    output.fields = {
      makeField("x", 0, FLOAT32),
      makeField("y", 4, FLOAT32),
      makeField("z", 8, FLOAT32),
      makeField("intensity", 16, FLOAT32),
      makeField("t", 20, UINT32),
      makeField("reflectivity", 24, UINT16),
      makeField("ring", 26, UINT16),
      makeField("ambient", 28, UINT16),
      makeField("range", 32, UINT32),
    };
    output.point_step = 36;
    output.row_step = output.point_step * output.width;
    output.data.resize(static_cast<size_t>(output.row_step) * output.height, 0U);

    const size_t point_count = static_cast<size_t>(input->width) * input->height;
    const double denom = input->width > 1 ? static_cast<double>(input->width - 1) : 1.0;

    for (size_t i = 0; i < point_count; ++i) {
      const float x = readValue<float>(*input, x_field, i, std::numeric_limits<float>::quiet_NaN());
      const float y = readValue<float>(*input, y_field, i, std::numeric_limits<float>::quiet_NaN());
      const float z = readValue<float>(*input, z_field, i, std::numeric_limits<float>::quiet_NaN());
      const float intensity = readValue<float>(*input, intensity_field, i, 0.0F);

      const size_t row = i / input->width;
      const size_t col = i % input->width;
      const size_t time_col = reverse_column_time_ ? (input->width - 1U - col) : col;
      const double relative_sec = (static_cast<double>(time_col) / denom) * scan_period_sec_;
      const auto t_nsec = static_cast<uint32_t>(
        std::clamp(
          std::llround(relative_sec * 1.0e9),
          0LL,
          static_cast<long long>(std::numeric_limits<uint32_t>::max())));

      const auto ring = readValue<uint16_t>(*input, ring_field, i, static_cast<uint16_t>(row));
      const auto reflectivity = intensityToReflectivity(intensity);
      const double range_m = std::sqrt(
        static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z);
      const auto range_mm = std::isfinite(range_m) && range_m > 0.0 ?
        static_cast<uint32_t>(
          std::min(std::llround(range_m * 1000.0),
          static_cast<long long>(std::numeric_limits<uint32_t>::max()))) :
        0U;

      const size_t base = i * output.point_step;
      writeValue<float>(output, base + 0, x);
      writeValue<float>(output, base + 4, y);
      writeValue<float>(output, base + 8, z);
      writeValue<float>(output, base + 16, intensity);
      writeValue<uint32_t>(output, base + 20, t_nsec);
      writeValue<uint16_t>(output, base + 24, reflectivity);
      writeValue<uint16_t>(output, base + 26, ring);
      writeValue<uint16_t>(output, base + 28, 0U);
      writeValue<uint32_t>(output, base + 32, range_mm);
    }

    publisher_->publish(output);
    ++published_clouds_;
    last_publish_time_ = std::chrono::steady_clock::now();
  }

  void publishDiagnostics()
  {
    const auto now = std::chrono::steady_clock::now();
    const double receive_age = std::chrono::duration<double>(now - last_receive_time_).count();
    const double publish_age = std::chrono::duration<double>(now - last_publish_time_).count();
    const char * state = receive_age > 1.0 ? "RAW_INPUT_MISSING" :
      (publish_age > 1.0 ? "INPUT_REJECTED_OR_PROCESSING" : "HEALTHY");

    std_msgs::msg::String msg;
    msg.data = std::string("state=") + state +
      " received=" + std::to_string(received_clouds_) +
      " published=" + std::to_string(published_clouds_) +
      " non_increasing=" + std::to_string(dropped_non_increasing_stamps_) +
      " malformed=" + std::to_string(malformed_clouds_) +
      " receive_age_s=" + std::to_string(receive_age) +
      " publish_age_s=" + std::to_string(publish_age) +
      " last_stamp=" + std::to_string(last_output_stamp_);
    diagnostics_publisher_->publish(msg);

    if (receive_age > 1.0 || publish_age > 1.0) {
      RCLCPP_WARN(
        get_logger(), "Time adapter heartbeat: %s", msg.data.c_str());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  double scan_period_sec_ {};
  bool reverse_column_time_ {};
  bool stamp_at_scan_start_ {};
  bool drop_non_increasing_stamps_ {};
  double last_output_stamp_ {};
  size_t dropped_non_increasing_stamps_ {};
  size_t received_clouds_ {};
  size_t published_clouds_ {};
  size_t malformed_clouds_ {};
  std::chrono::steady_clock::time_point last_receive_time_;
  std::chrono::steady_clock::time_point last_publish_time_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SeyondCloudTimeAdapter>());
  rclcpp::shutdown();
  return 0;
}
