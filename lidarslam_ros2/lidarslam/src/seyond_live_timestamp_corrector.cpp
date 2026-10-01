#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{

class SeyondLiveTimestampCorrector : public rclcpp::Node
{
public:
  SeyondLiveTimestampCorrector()
  : Node("seyond_live_timestamp_corrector")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/iv_points");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/iv_points/time_corrected");
    imu_topic_ = declare_parameter<std::string>(
      "imu_topic", "/husky/sensors/imu_0/data");
    point_time_field_ = declare_parameter<std::string>("point_time_field", "timestamp");
    offset_sec_ = declare_parameter<double>("offset_sec", -6.575);
    if (!std::isfinite(offset_sec_)) {
      throw std::invalid_argument("offset_sec must be finite");
    }
    offset_nanoseconds_ = static_cast<int64_t>(std::llround(offset_sec_ * 1.0e9));
    adjust_point_times_ = declare_parameter<bool>("adjust_point_times", true);
    require_imu_sync_ = declare_parameter<bool>("require_imu_sync", true);
    diagnostics_period_sec_ = std::max(
      0.5, declare_parameter<double>("diagnostics_period_sec", 5.0));
    sync_warning_sec_ = std::max(
      0.0, declare_parameter<double>("sync_warning_sec", 0.15));
    point_header_sanity_sec_ = std::max(
      0.0, declare_parameter<double>("point_header_sanity_sec", 0.5));

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    lidar_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&SeyondLiveTimestampCorrector::cloudCallback, this, std::placeholders::_1));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        latest_imu_stamp_.store(rclcpp::Time(message->header.stamp).seconds());
      });

    RCLCPP_INFO(
      get_logger(),
      "Live Seyond timestamp correction: %s -> %s offset=%+.9fs point_field=%s "
      "adjust_points=%s require_imu_sync=%s IMU=%s",
      input_topic_.c_str(), output_topic_.c_str(), offset_sec_, point_time_field_.c_str(),
      adjust_point_times_ ? "true" : "false", require_imu_sync_ ? "true" : "false",
      imu_topic_.c_str());
  }

private:
  static double readFloat64(const uint8_t * source, bool big_endian)
  {
    uint8_t bytes[sizeof(double)];
    std::memcpy(bytes, source, sizeof(double));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    if (big_endian) {
      std::reverse(bytes, bytes + sizeof(double));
    }
#else
    if (!big_endian) {
      std::reverse(bytes, bytes + sizeof(double));
    }
#endif
    double value;
    std::memcpy(&value, bytes, sizeof(double));
    return value;
  }

  static void writeFloat64(uint8_t * destination, double value, bool big_endian)
  {
    uint8_t bytes[sizeof(double)];
    std::memcpy(bytes, &value, sizeof(double));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    if (big_endian) {
      std::reverse(bytes, bytes + sizeof(double));
    }
#else
    if (!big_endian) {
      std::reverse(bytes, bytes + sizeof(double));
    }
#endif
    std::memcpy(destination, bytes, sizeof(double));
  }

  bool shiftPointTimes(sensor_msgs::msg::PointCloud2 & cloud, double & minimum, double & maximum)
  {
    const auto field_it = std::find_if(
      cloud.fields.begin(), cloud.fields.end(),
      [this](const sensor_msgs::msg::PointField & field) {
        return field.name == point_time_field_;
      });
    if (field_it == cloud.fields.end()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping cloud: required point-time field '%s' is missing",
        point_time_field_.c_str());
      return false;
    }
    if (
      field_it->datatype != sensor_msgs::msg::PointField::FLOAT64 ||
      field_it->count != 1 || field_it->offset + sizeof(double) > cloud.point_step)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping cloud: field '%s' must be FLOAT64 count=1 within point_step; "
        "datatype=%u count=%u offset=%u point_step=%u",
        point_time_field_.c_str(), field_it->datatype, field_it->count,
        field_it->offset, cloud.point_step);
      return false;
    }
    const std::size_t expected_size =
      static_cast<std::size_t>(cloud.row_step) * static_cast<std::size_t>(cloud.height);
    if (cloud.data.size() < expected_size) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping truncated cloud: data=%zu expected_at_least=%zu",
        cloud.data.size(), expected_size);
      return false;
    }

    minimum = std::numeric_limits<double>::infinity();
    maximum = -std::numeric_limits<double>::infinity();
    for (uint32_t row = 0; row < cloud.height; ++row) {
      const std::size_t row_start = static_cast<std::size_t>(row) * cloud.row_step;
      for (uint32_t column = 0; column < cloud.width; ++column) {
        const std::size_t byte_offset = row_start +
          static_cast<std::size_t>(column) * cloud.point_step + field_it->offset;
        double timestamp = readFloat64(cloud.data.data() + byte_offset, cloud.is_bigendian);
        if (!std::isfinite(timestamp)) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Dropping cloud: point-time field '%s' contains a non-finite value",
            point_time_field_.c_str());
          return false;
        }
        timestamp += offset_sec_;
        writeFloat64(cloud.data.data() + byte_offset, timestamp, cloud.is_bigendian);
        minimum = std::min(minimum, timestamp);
        maximum = std::max(maximum, timestamp);
      }
    }
    return true;
  }

  void cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr cloud)
  {
    const rclcpp::Time raw_time(cloud->header.stamp, RCL_SYSTEM_TIME);
    const int64_t corrected_nanoseconds = raw_time.nanoseconds() + offset_nanoseconds_;
    if (corrected_nanoseconds < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping cloud: offset produced a negative header timestamp");
      return;
    }
    const double raw_stamp = raw_time.seconds();
    const double corrected_stamp = static_cast<double>(corrected_nanoseconds) * 1.0e-9;
    const double imu_stamp = latest_imu_stamp_.load();
    const double corrected_gap =
      imu_stamp > 0.0 ? corrected_stamp - imu_stamp :
      std::numeric_limits<double>::quiet_NaN();
    if (require_imu_sync_ && imu_stamp <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for IMU timestamps before releasing corrected LiDAR clouds");
      ++unsynchronized_drop_count_;
      return;
    }
    if (
      require_imu_sync_ && std::isfinite(corrected_gap) &&
      std::abs(corrected_gap) > sync_warning_sec_)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Blocking corrected LiDAR: corrected_lidar_minus_latest_imu=%+.3fms exceeds "
        "%.3fms. Verify lidar_time_offset_sec (currently %+.9fs)",
        corrected_gap * 1000.0, sync_warning_sec_ * 1000.0, offset_sec_);
      ++unsynchronized_drop_count_;
      return;
    }

    double point_min = corrected_stamp;
    double point_max = corrected_stamp;
    if (adjust_point_times_ && !shiftPointTimes(*cloud, point_min, point_max)) {
      return;
    }
    if (
      adjust_point_times_ &&
      (std::abs(point_min - corrected_stamp) > point_header_sanity_sec_ ||
      std::abs(point_max - corrected_stamp) > point_header_sanity_sec_))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping cloud: corrected point times [%.9f, %.9f] are not within %.3fs "
        "of corrected header %.9f; field '%s' may not contain absolute seconds",
        point_min, point_max, point_header_sanity_sec_, corrected_stamp,
        point_time_field_.c_str());
      return;
    }
    cloud->header.stamp = rclcpp::Time(corrected_nanoseconds, RCL_SYSTEM_TIME);
    publisher_->publish(std::move(cloud));
    ++cloud_count_;

    const auto now = std::chrono::steady_clock::now();
    if (
      last_diagnostic_time_.time_since_epoch().count() == 0 ||
      std::chrono::duration<double>(now - last_diagnostic_time_).count() >=
      diagnostics_period_sec_)
    {
      last_diagnostic_time_ = now;
      RCLCPP_INFO(
        get_logger(),
        "Live timing: clouds=%llu unsync_drops=%llu raw=%.9f corrected=%.9f point_span=%.6fs "
        "latest_imu=%.9f corrected_lidar_minus_imu=%+.3fms",
        static_cast<unsigned long long>(cloud_count_.load()),
        static_cast<unsigned long long>(unsynchronized_drop_count_.load()), raw_stamp, corrected_stamp,
        point_max - point_min, imu_stamp, corrected_gap * 1000.0);
      if (imu_stamp > 0.0 && std::abs(corrected_gap) > sync_warning_sec_) {
        RCLCPP_WARN(
          get_logger(),
          "Corrected LiDAR/IMU gap is %.3fms (warning threshold %.3fms); verify "
          "offset_sec before trusting live odometry",
          corrected_gap * 1000.0, sync_warning_sec_ * 1000.0);
      }
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string imu_topic_;
  std::string point_time_field_;
  double offset_sec_ {0.0};
  bool adjust_point_times_ {true};
  bool require_imu_sync_ {true};
  int64_t offset_nanoseconds_ {0};
  double diagnostics_period_sec_ {5.0};
  double sync_warning_sec_ {0.15};
  double point_header_sanity_sec_ {0.5};
  std::atomic<double> latest_imu_stamp_ {0.0};
  std::atomic<uint64_t> cloud_count_ {0};
  std::atomic<uint64_t> unsynchronized_drop_count_ {0};
  std::chrono::steady_clock::time_point last_diagnostic_time_ {};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SeyondLiveTimestampCorrector>());
  rclcpp::shutdown();
  return 0;
}
