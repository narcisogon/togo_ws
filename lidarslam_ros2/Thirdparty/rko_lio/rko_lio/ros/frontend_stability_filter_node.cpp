#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace
{
constexpr std::size_t DIAG_VALID = 0;
constexpr std::size_t DIAG_KEYPOINTS = 1;
constexpr std::size_t DIAG_CORRESPONDENCES = 2;
constexpr std::size_t DIAG_INLIER_RATIO = 3;
constexpr std::size_t DIAG_MEAN_ERROR = 4;
constexpr std::size_t DIAG_HESSIAN_MIN_EIGEN = 5;
constexpr std::size_t DIAG_HESSIAN_CONDITION = 7;

double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::pair<double, double> roll_pitch_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  const double pitch = std::abs(sinp) >= 1.0 ?
    std::copysign(M_PI / 2.0, sinp) :
    std::asin(sinp);
  return {roll, pitch};
}

geometry_msgs::msg::Quaternion quat_from_rpy(double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

double wrap_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

struct RegistrationDiagnostics
{
  rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
  bool valid {false};
  double keypoints {0.0};
  double correspondences {0.0};
  double inlier_ratio {0.0};
  double mean_error {std::numeric_limits<double>::infinity()};
  double hessian_min_eigen {0.0};
  double hessian_condition {std::numeric_limits<double>::infinity()};

  explicit RegistrationDiagnostics(
    const std_msgs::msg::Float32MultiArray & msg,
    const rclcpp::Time & now)
  : stamp(now)
  {
    const auto & data = msg.data;
    valid = data.size() > DIAG_VALID && data[DIAG_VALID] > 0.5f;
    keypoints = data.size() > DIAG_KEYPOINTS ? data[DIAG_KEYPOINTS] : 0.0;
    correspondences = data.size() > DIAG_CORRESPONDENCES ? data[DIAG_CORRESPONDENCES] : 0.0;
    inlier_ratio = data.size() > DIAG_INLIER_RATIO ? data[DIAG_INLIER_RATIO] : 0.0;
    mean_error = data.size() > DIAG_MEAN_ERROR ? data[DIAG_MEAN_ERROR] :
      std::numeric_limits<double>::infinity();
    hessian_min_eigen = data.size() > DIAG_HESSIAN_MIN_EIGEN ?
      data[DIAG_HESSIAN_MIN_EIGEN] : 0.0;
    hessian_condition = data.size() > DIAG_HESSIAN_CONDITION ?
      data[DIAG_HESSIAN_CONDITION] : std::numeric_limits<double>::infinity();
  }
};
}  // namespace

class FrontendStabilityFilterNode : public rclcpp::Node
{
public:
  explicit FrontendStabilityFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("frontend_stability_filter", options)
  {
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/rko_lio/odometry");
    const auto diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", "/rko_lio/registration_diagnostics");
    stable_odom_topic_ = declare_parameter<std::string>(
      "stable_odom_topic", "/rko_lio/odometry_stable");
    stable_path_topic_ = declare_parameter<std::string>(
      "stable_path_topic", "/rko_lio/path_stable");
    status_topic_ = declare_parameter<std::string>("status_topic", "/rko_lio/stability_status");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    max_poses_ = declare_parameter<int>("max_poses", 12000);

    use_registration_diagnostics_ = declare_parameter<bool>("use_registration_diagnostics", true);
    diagnostics_timeout_sec_ = declare_parameter<double>("diagnostics_timeout_sec", 0.5);
    min_overlap_ratio_ = declare_parameter<double>("min_overlap_ratio", 0.12);
    max_mean_error_m_ = declare_parameter<double>("max_mean_error_m", 1.5);
    min_hessian_eigenvalue_ = declare_parameter<double>("min_hessian_eigenvalue", 1e-5);
    max_hessian_condition_ = declare_parameter<double>("max_hessian_condition", 1e5);

    max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 1.5);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate_deg", 70.0) * M_PI / 180.0;
    hard_gate_multiplier_ = declare_parameter<double>("hard_gate_multiplier", 2.5);
    position_alpha_ = declare_parameter<double>("position_alpha", 0.55);
    yaw_alpha_ = declare_parameter<double>("yaw_alpha", 0.55);
    weak_position_alpha_ = declare_parameter<double>("weak_position_alpha", 0.20);
    weak_yaw_alpha_ = declare_parameter<double>("weak_yaw_alpha", 0.20);
    prediction_decay_ = declare_parameter<double>("prediction_decay", 0.35);
    max_prediction_frames_ = declare_parameter<int>("max_prediction_frames", 3);
    stationary_linear_velocity_ = declare_parameter<double>("stationary_linear_velocity", 0.05);
    stationary_yaw_rate_ = declare_parameter<double>("stationary_yaw_rate_deg", 3.0) * M_PI / 180.0;
    min_dt_sec_ = declare_parameter<double>("min_dt_sec", 0.02);
    max_dt_sec_ = declare_parameter<double>("max_dt_sec", 0.5);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos, std::bind(&FrontendStabilityFilterNode::on_odom, this, std::placeholders::_1));
    diagnostics_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      diagnostics_topic, qos,
      std::bind(&FrontendStabilityFilterNode::on_diagnostics, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(stable_odom_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(stable_path_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(status_topic_, 10);

    RCLCPP_INFO(
      get_logger(), "Stabilizing %s -> %s; diagnostics=%s",
      odom_topic.c_str(), stable_odom_topic_.c_str(), diagnostics_topic.c_str());
  }

private:
  void on_diagnostics(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    latest_diagnostics_ = std::make_unique<RegistrationDiagnostics>(*msg, now());
  }

  std::pair<bool, double> diagnostics_are_weak() const
  {
    if (!use_registration_diagnostics_) {
      return {false, 1.0};
    }
    if (!latest_diagnostics_) {
      return {true, 0.35};
    }
    const double age = (now() - latest_diagnostics_->stamp).seconds();
    if (age > diagnostics_timeout_sec_) {
      return {true, 0.35};
    }

    const bool weak =
      !latest_diagnostics_->valid ||
      latest_diagnostics_->inlier_ratio < min_overlap_ratio_ ||
      latest_diagnostics_->mean_error > max_mean_error_m_ ||
      latest_diagnostics_->hessian_min_eigen < min_hessian_eigenvalue_ ||
      latest_diagnostics_->hessian_condition > max_hessian_condition_;

    const double overlap_score = clamp(
      latest_diagnostics_->inlier_ratio / std::max(min_overlap_ratio_, 1e-6), 0.0, 1.0);
    const double error_score = clamp(
      max_mean_error_m_ / std::max(latest_diagnostics_->mean_error, 1e-6), 0.0, 1.0);
    const double hessian_score =
      latest_diagnostics_->hessian_condition > max_hessian_condition_ ? 0.0 : 1.0;
    return {weak, std::min({overlap_score, error_score, hessian_score})};
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time raw_time(msg->header.stamp);
    if (!last_raw_ || !last_stable_) {
      nav_msgs::msg::Odometry stable = *msg;
      stable.header.frame_id = fixed_frame_.empty() ? msg->header.frame_id : fixed_frame_;
      last_raw_ = *msg;
      last_stable_ = stable;
      last_stable_yaw_ = yaw_from_quat(msg->pose.pose.orientation);
      publish(stable, 0.0, 1.0, 0.0, 0.0);
      return;
    }

    double dt = (raw_time - rclcpp::Time(last_raw_->header.stamp)).seconds();
    if (!std::isfinite(dt) || dt <= 0.0) {
      dt = min_dt_sec_;
    }
    dt = clamp(dt, min_dt_sec_, max_dt_sec_);

    const auto & raw_pos = msg->pose.pose.position;
    const auto & last_raw_pos = last_raw_->pose.pose.position;
    const auto & stable_pos = last_stable_->pose.pose.position;
    const double raw_yaw = yaw_from_quat(msg->pose.pose.orientation);
    const double last_raw_yaw = yaw_from_quat(last_raw_->pose.pose.orientation);

    const double raw_frame_dx = raw_pos.x - last_raw_pos.x;
    const double raw_frame_dy = raw_pos.y - last_raw_pos.y;
    const double raw_frame_dz = raw_pos.z - last_raw_pos.z;
    const double raw_frame_dist = std::sqrt(
      raw_frame_dx * raw_frame_dx + raw_frame_dy * raw_frame_dy + raw_frame_dz * raw_frame_dz);
    const double raw_frame_yaw_delta = std::abs(wrap_angle(raw_yaw - last_raw_yaw));
    const bool raw_stationary =
      raw_frame_dist <= stationary_linear_velocity_ * dt &&
      raw_frame_yaw_delta <= stationary_yaw_rate_ * dt;

    if (raw_stationary) {
      last_delta_x_ = 0.0;
      last_delta_y_ = 0.0;
      last_delta_z_ = 0.0;
      last_delta_yaw_ = 0.0;
      consecutive_prediction_frames_ = 0;
    }

    const double pred_x = stable_pos.x + last_delta_x_;
    const double pred_y = stable_pos.y + last_delta_y_;
    const double pred_z = stable_pos.z + last_delta_z_;
    const double pred_yaw = wrap_angle(last_stable_yaw_ + last_delta_yaw_);

    const double raw_dx = raw_pos.x - stable_pos.x;
    const double raw_dy = raw_pos.y - stable_pos.y;
    const double raw_dz = raw_pos.z - stable_pos.z;
    const double raw_dist = std::sqrt(raw_dx * raw_dx + raw_dy * raw_dy + raw_dz * raw_dz);
    const double raw_yaw_delta = std::abs(wrap_angle(raw_yaw - last_stable_yaw_));

    const double trans_limit = max_linear_velocity_ * dt;
    const double yaw_limit = max_yaw_rate_ * dt;
    const auto [weak, confidence] = diagnostics_are_weak();
    const bool hard_reject =
      raw_dist > trans_limit * hard_gate_multiplier_ ||
      raw_yaw_delta > yaw_limit * hard_gate_multiplier_;
    const bool soft_gate = weak || raw_dist > trans_limit || raw_yaw_delta > yaw_limit;

    double alpha_pos = position_alpha_;
    double alpha_yaw = yaw_alpha_;
    double state = 0.0;
    if (hard_reject) {
      alpha_pos = 0.0;
      alpha_yaw = 0.0;
      state = 2.0;
      ++consecutive_prediction_frames_;
    } else if (soft_gate) {
      alpha_pos = weak_position_alpha_;
      alpha_yaw = weak_yaw_alpha_;
      state = 1.0;
      consecutive_prediction_frames_ = 0;
    } else {
      consecutive_prediction_frames_ = 0;
    }

    const bool prediction_exhausted =
      max_prediction_frames_ >= 0 && consecutive_prediction_frames_ > max_prediction_frames_;
    const double prediction_scale = hard_reject ?
      (prediction_exhausted ? 0.0 : prediction_decay_) :
      1.0;
    const double limited_pred_x = stable_pos.x + prediction_scale * last_delta_x_;
    const double limited_pred_y = stable_pos.y + prediction_scale * last_delta_y_;
    const double limited_pred_z = stable_pos.z + prediction_scale * last_delta_z_;
    const double limited_pred_yaw =
      wrap_angle(last_stable_yaw_ + prediction_scale * last_delta_yaw_);

    const double out_x = limited_pred_x + alpha_pos * (raw_pos.x - limited_pred_x);
    const double out_y = limited_pred_y + alpha_pos * (raw_pos.y - limited_pred_y);
    const double out_z = limited_pred_z + alpha_pos * (raw_pos.z - limited_pred_z);
    const double out_yaw =
      wrap_angle(limited_pred_yaw + alpha_yaw * wrap_angle(raw_yaw - limited_pred_yaw));

    const auto [roll, pitch] = roll_pitch_from_quat(msg->pose.pose.orientation);
    nav_msgs::msg::Odometry stable = *msg;
    stable.header.frame_id = fixed_frame_.empty() ? msg->header.frame_id : fixed_frame_;
    stable.pose.pose.position.x = out_x;
    stable.pose.pose.position.y = out_y;
    stable.pose.pose.position.z = out_z;
    stable.pose.pose.orientation = quat_from_rpy(roll, pitch, out_yaw);

    last_delta_x_ = out_x - stable_pos.x;
    last_delta_y_ = out_y - stable_pos.y;
    last_delta_z_ = out_z - stable_pos.z;
    last_delta_yaw_ = wrap_angle(out_yaw - last_stable_yaw_);
    const double stable_jump = std::sqrt(
      last_delta_x_ * last_delta_x_ + last_delta_y_ * last_delta_y_ + last_delta_z_ * last_delta_z_);

    last_raw_ = *msg;
    last_stable_ = stable;
    last_stable_yaw_ = out_yaw;
    publish(stable, state, confidence, raw_dist, stable_jump);
  }

  void publish(
    const nav_msgs::msg::Odometry & odom,
    double state,
    double confidence,
    double raw_jump,
    double stable_jump)
  {
    odom_pub_->publish(odom);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    poses_.push_back(pose);
    while (static_cast<int>(poses_.size()) > std::max(1, max_poses_)) {
      poses_.pop_front();
    }
    nav_msgs::msg::Path path;
    path.header = odom.header;
    path.poses.assign(poses_.begin(), poses_.end());
    path_pub_->publish(path);

    std_msgs::msg::Float32MultiArray status;
    status.data = {
      static_cast<float>(state),
      static_cast<float>(confidence),
      static_cast<float>(raw_jump),
      static_cast<float>(stable_jump),
      static_cast<float>(consecutive_prediction_frames_)};
    status_pub_->publish(status);
  }

  std::string stable_odom_topic_;
  std::string stable_path_topic_;
  std::string status_topic_;
  std::string fixed_frame_;
  int max_poses_ {12000};

  bool use_registration_diagnostics_ {true};
  double diagnostics_timeout_sec_ {0.5};
  double min_overlap_ratio_ {0.12};
  double max_mean_error_m_ {1.5};
  double min_hessian_eigenvalue_ {1e-5};
  double max_hessian_condition_ {1e5};
  double max_linear_velocity_ {1.5};
  double max_yaw_rate_ {70.0 * M_PI / 180.0};
  double hard_gate_multiplier_ {2.5};
  double position_alpha_ {0.55};
  double yaw_alpha_ {0.55};
  double weak_position_alpha_ {0.20};
  double weak_yaw_alpha_ {0.20};
  double prediction_decay_ {0.35};
  int max_prediction_frames_ {3};
  double stationary_linear_velocity_ {0.05};
  double stationary_yaw_rate_ {3.0 * M_PI / 180.0};
  double min_dt_sec_ {0.02};
  double max_dt_sec_ {0.5};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr diagnostics_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;

  std::unique_ptr<RegistrationDiagnostics> latest_diagnostics_;
  std::optional<nav_msgs::msg::Odometry> last_raw_;
  std::optional<nav_msgs::msg::Odometry> last_stable_;
  std::deque<geometry_msgs::msg::PoseStamped> poses_;
  double last_stable_yaw_ {0.0};
  double last_delta_x_ {0.0};
  double last_delta_y_ {0.0};
  double last_delta_z_ {0.0};
  double last_delta_yaw_ {0.0};
  int consecutive_prediction_frames_ {0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontendStabilityFilterNode>());
  rclcpp::shutdown();
  return 0;
}
