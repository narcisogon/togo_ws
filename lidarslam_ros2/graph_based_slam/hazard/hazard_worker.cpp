#include "hazard_worker.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace graphslam::hazard
{
namespace
{
Eigen::Isometry3d poseFromMessage(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (!q.coeffs().allFinite() || q.norm() < 0.5) {
    throw std::invalid_argument("invalid odometry quaternion");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = q.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  if (!result.matrix().allFinite()) {throw std::invalid_argument("invalid odometry position");}
  return result;
}

std::vector<Eigen::Vector3d> pointsFromCloud(const sensor_msgs::msg::PointCloud2 & message)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(message, cloud);
  std::vector<Eigen::Vector3d> points;
  points.reserve(cloud.size());
  for (const auto & p : cloud) {points.emplace_back(p.x, p.y, p.z);}
  return points;
}

std::vector<Eigen::Vector3d> pointsFromSource(const Source & source)
{
  if (source.cloud) {return pointsFromCloud(*source.cloud);}
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (source.pcd_path.empty() || pcl::io::loadPCDFile(source.pcd_path, cloud) != 0) {
    throw std::runtime_error("cannot load hazard submap " + std::to_string(source.id));
  }
  std::vector<Eigen::Vector3d> points;
  points.reserve(cloud.size());
  for (const auto & p : cloud) {points.emplace_back(p.x, p.y, p.z);}
  return points;
}

double footprintRadius(const std::vector<double> & footprint)
{
  if (footprint.size() < 6 || footprint.size() % 2 != 0) {
    throw std::invalid_argument("hazard/footprint must contain at least three x,y pairs");
  }
  double radius = 0.0;
  for (std::size_t i = 0; i < footprint.size(); i += 2) {
    if (!std::isfinite(footprint[i]) || !std::isfinite(footprint[i + 1])) {
      throw std::invalid_argument("hazard footprint must be finite");
    }
    radius = std::max(radius, std::hypot(footprint[i], footprint[i + 1]));
  }
  return radius;
}
}  // namespace

Worker::Worker(rclcpp::Node & node, std::string map_frame, std::string odom_frame)
: node_(node), map_frame_(std::move(map_frame)), odom_frame_(std::move(odom_frame))
{
  enabled_ = node_.declare_parameter<bool>("hazard/enabled", false);
  auto & c = pending_.config;
  c.resolution = node_.declare_parameter<double>("hazard/resolution_m", c.resolution);
  c.min_height = node_.declare_parameter<double>("hazard/input_min_height_m", c.min_height);
  c.max_height = node_.declare_parameter<double>("hazard/input_max_height_m", c.max_height);
  c.max_range = node_.declare_parameter<double>("hazard/max_range_m", c.max_range);
  c.slope_radius = node_.declare_parameter<double>("hazard/slope_radius_m", c.slope_radius);
  c.max_slope_deg = node_.declare_parameter<double>("hazard/max_slope_deg", c.max_slope_deg);
  c.lethal_radius = node_.declare_parameter<double>("hazard/lethal_inflation_m", c.lethal_radius);
  c.soft_radius = node_.declare_parameter<double>("hazard/soft_inflation_m", c.soft_radius);
  c.unknown_cost = node_.declare_parameter<int>("hazard/unknown_cost", c.unknown_cost);
  footprint_ = node_.declare_parameter<std::vector<double>>("hazard/footprint",
    {0.55, 0.38, 0.55, -0.38, -0.55, -0.38, -0.55, 0.38});
  c.footprint_radius = footprintRadius(footprint_);
  c.min_points = node_.declare_parameter<int>("hazard/min_points_per_cell", c.min_points);
  c.min_neighbors = node_.declare_parameter<int>("hazard/min_plane_neighbors", c.min_neighbors);
  c.terrain_percentile = node_.declare_parameter<double>("hazard/terrain_percentile", c.terrain_percentile);
  c.height_noise = node_.declare_parameter<double>("hazard/height_noise_m", c.height_noise);
  c.min_plane_coverage = node_.declare_parameter<double>("hazard/min_plane_coverage", c.min_plane_coverage);
  c.max_plane_error = node_.declare_parameter<double>("hazard/max_plane_error_m", c.max_plane_error);
  c.clear_confirmations = node_.declare_parameter<int>("hazard/clear_confirmations", c.clear_confirmations);
  c.clear_slope_margin = node_.declare_parameter<double>("hazard/clear_slope_margin_deg", c.clear_slope_margin);
  c.obstacle_min_height = node_.declare_parameter<double>("hazard/obstacle_min_height_m", c.obstacle_min_height);
  c.obstacle_max_height = node_.declare_parameter<double>("hazard/obstacle_max_height_m", c.obstacle_max_height);
  c.obstacle_voxel_height = node_.declare_parameter<double>("hazard/obstacle_voxel_height_m", c.obstacle_voxel_height);
  c.obstacle_min_points = node_.declare_parameter<int>("hazard/obstacle_min_points", c.obstacle_min_points);
  c.obstacle_hit_probability = node_.declare_parameter<double>("hazard/obstacle_hit_probability", c.obstacle_hit_probability);
  c.obstacle_miss_probability = node_.declare_parameter<double>("hazard/obstacle_miss_probability", c.obstacle_miss_probability);
  c.obstacle_mark_probability = node_.declare_parameter<double>("hazard/obstacle_mark_probability", c.obstacle_mark_probability);
  c.obstacle_clear_probability = node_.declare_parameter<double>("hazard/obstacle_clear_probability", c.obstacle_clear_probability);
  c.obstacle_max_probability = node_.declare_parameter<double>("hazard/obstacle_max_probability", c.obstacle_max_probability);
  c.max_rays = node_.declare_parameter<int>("hazard/max_rays", c.max_rays);
  c.max_obstacle_voxels = node_.declare_parameter<int>("hazard/max_obstacle_voxels", c.max_obstacle_voxels);
  lidar_frame_ = node_.declare_parameter<std::string>("hazard/lidar_frame", "lidar3d_0_laser");
  if (lidar_frame_.empty()) {throw std::invalid_argument("hazard/lidar_frame must not be empty");}
  const int max_cells = node_.declare_parameter<int>("hazard/max_cells", static_cast<int>(c.max_cells));
  if (max_cells < 100) {throw std::invalid_argument("hazard/max_cells must be >= 100");}
  c.max_cells = static_cast<std::size_t>(max_cells);
  pending_.max_age = node_.declare_parameter<double>("hazard/max_input_age_sec", 1.0);
  pending_.update_period = node_.declare_parameter<double>("hazard/update_period_sec", 0.2);
  pending_.batch_clouds = node_.declare_parameter<int>("hazard/batch_clouds", 3);
  c.validate();
  if (!std::isfinite(pending_.max_age) || pending_.max_age <= 0.0 ||
    !std::isfinite(pending_.update_period) || pending_.update_period < 0.05 ||
    pending_.batch_clouds < 1 || pending_.batch_clouds > 16)
  {throw std::invalid_argument("invalid hazard cadence or freshness limit");}
  pending_.config_revision = 1;
  pending_.sources = std::make_shared<const std::vector<Source>>();
  parameter_callback_ = node_.add_on_set_parameters_callback(
    [this](const auto & values) {return parameters(values);});
  if (!enabled_) {return;}
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
  // Reuse the backend executor; no listener node or additional spin thread.
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, &node_, false);
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  map_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
  raw_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>("hazard/raw", qos);
  slope_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("hazard/slope", qos);
  cloud_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("hazard/normalized_cloud", qos);
  obstacle_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("hazard/obstacles", qos);
  diagnostics_pub_ = node_.create_publisher<std_msgs::msg::String>("hazard/diagnostics", rclcpp::QoS(10));
  // Volatile health: a navigation process must hear a NEW heartbeat before motion.
  health_pub_ = node_.create_publisher<std_msgs::msg::Bool>("hazard/ready", rclcpp::QoS(1));
  status_pub_ = node_.create_publisher<std_msgs::msg::String>("hazard/status", qos);
  map_pub_->publish(message(rasterize({}, c), false));
  publishHealthLocked();
  // Heartbeats must not depend on the duration of a terrain rasterization job.
  health_timer_ = node_.create_wall_timer(std::chrono::milliseconds(200), [this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    publishHealthLocked();
  });
  thread_ = std::thread(&Worker::run, this);
}

Worker::~Worker()
{
  if (health_timer_) {health_timer_->cancel();}
  stop();
  if (thread_.joinable()) {thread_.join();}
}

void Worker::stop()
{
  stopping_ = true;
  wake_.notify_all();
}

void Worker::receiveOdometry(const nav_msgs::msg::Odometry & odom)
{
  if (!enabled_ || odom.header.frame_id != odom_frame_) {return;}
  try {
    const OdomSample sample{rclcpp::Time(odom.header.stamp).nanoseconds(), poseFromMessage(odom.pose.pose)};
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_.odometry.empty() && sample.stamp <= pending_.odometry.back().stamp) {return;}
    if (pending_.body_frame != odom.child_frame_id) {
      pending_.odometry.clear(); pending_.body_frame = odom.child_frame_id;
    }
    pending_.odometry.push_back(sample);
    while (pending_.odometry.size() > 500) {pending_.odometry.pop_front();}
    wake_.notify_one();
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000, "%s", e.what());
  }
}

void Worker::receiveCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (!enabled_) {return;}
  if (cloud->header.frame_id != odom_frame_) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
      "Hazard input must be in '%s', received '%s'", odom_frame_.c_str(), cloud->header.frame_id.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  // Repeated/older stamps are not fresh sensor observations.
  if (pending_.cloud && rclcpp::Time(cloud->header.stamp) <=
    rclcpp::Time(pending_.cloud->header.stamp)) {return;}
  pending_.cloud = std::move(cloud);
  pending_.clouds.push_back({pending_.cloud, std::chrono::steady_clock::now()});
  while (pending_.clouds.size() > static_cast<std::size_t>(pending_.batch_clouds)) {
    pending_.clouds.pop_front(); ++pending_.dropped_clouds;
  }
  ++pending_.cloud_revision;
  wake_.notify_one();
}

void Worker::updateGraph(std::vector<Source> sources, const Eigen::Isometry3d & correction)
{
  if (!enabled_) {return;}
  auto immutable = std::make_shared<const std::vector<Source>>(std::move(sources));
  std::lock_guard<std::mutex> lock(mutex_);
  // An appended submap does not invalidate the previous map. Changed/removed
  // observations or a changed map-to-odom correction do invalidate its geometry.
  bool changed = !pending_.correction.matrix().isApprox(correction.matrix(), 1e-6) ||
    immutable->size() < pending_.sources->size();
  for (std::size_t i = 0; !changed && i < pending_.sources->size(); ++i) {
    const auto & before = (*pending_.sources)[i];
    const auto & after = (*immutable)[i];
    changed = before.id != after.id ||
      !before.pose.matrix().isApprox(after.pose.matrix(), 1e-6) ||
      !before.up_in_body.isApprox(after.up_in_body, 1e-6);
  }
  if (changed) {++pending_.geometry_revision;}
  pending_.sources = std::move(immutable);
  pending_.correction = correction;
  ++pending_.graph_revision;
  publishHealthLocked();
  wake_.notify_one();
}

std::optional<Eigen::Isometry3d> Worker::poseAt(const Snapshot & s, int64_t stamp) const
{
  if (s.odometry.empty()) {return std::nullopt;}
  const auto after = std::lower_bound(s.odometry.begin(), s.odometry.end(), stamp,
    [](const OdomSample & sample, int64_t t) {return sample.stamp < t;});
  // Permit at most 25ms endpoint skew, otherwise wait for bracketing odometry.
  if (after == s.odometry.begin()) {
    if (std::abs(after->stamp - stamp) <= 25000000) {return after->pose;}
    return std::nullopt;
  }
  if (after == s.odometry.end()) {
    if (std::abs(s.odometry.back().stamp - stamp) <= 25000000) {return s.odometry.back().pose;}
    return std::nullopt;
  }
  const auto before = std::prev(after);
  if (after->stamp - before->stamp > 200000000) {return std::nullopt;}
  const double alpha = static_cast<double>(stamp - before->stamp) / (after->stamp - before->stamp);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = (1.0 - alpha) * before->pose.translation() + alpha * after->pose.translation();
  result.linear() = Eigen::Quaterniond(before->pose.linear()).slerp(alpha,
    Eigen::Quaterniond(after->pose.linear())).toRotationMatrix();
  return result;
}

std::optional<Eigen::Isometry3d> Worker::acquisitionPose(int64_t stamp)
{
  Snapshot snapshot;
  {std::lock_guard<std::mutex> lock(mutex_); snapshot.odometry = pending_.odometry;}
  return poseAt(snapshot, stamp);
}

bool Worker::rayOrigin(const Snapshot & s, int64_t stamp,
  const Eigen::Isometry3d & pose, Eigen::Vector3d & origin) const
{
  constexpr int64_t scan_duration = 100000000;
  constexpr double max_translation = 0.01;
  constexpr double max_rotation = 0.5 * M_PI / 180.0;
  if (s.body_frame.empty()) {return false;}
  try {
    const auto transform = tf_buffer_->lookupTransform(s.body_frame, lidar_frame_, tf2::TimePointZero);
    origin = {transform.transform.translation.x, transform.transform.translation.y,
      transform.transform.translation.z};
    if (!origin.allFinite()) {return false;}
  } catch (const tf2::TransformException &) {return false;}
  // Clouds are deskewed but have no retained per-return acquisition origins.
  // Only a fully bracketed, nearly stationary scan may use one common origin.
  // DLIO retains the sensor header; synthetic timing places returns after that
  // header, while sensor streams can use an end stamp. Cover both conventions.
  if (s.odometry.empty() ||
    s.odometry.front().stamp > stamp - scan_duration ||
    s.odometry.back().stamp < stamp + scan_duration)
  {return false;}
  const auto before = poseAt(s, stamp - scan_duration);
  const auto after = poseAt(s, stamp + scan_duration);
  if (!before || !after) {return false;}
  const auto stationary = [&](const Eigen::Isometry3d & sample) {
      return (sample.translation() - pose.translation()).norm() <= max_translation &&
             Eigen::AngleAxisd(pose.linear().transpose() * sample.linear()).angle() <= max_rotation;
    };
  if (!stationary(*before) || !stationary(*after)) {return false;}
  for (const auto & sample : s.odometry) {
    if (sample.stamp < stamp - scan_duration) {continue;}
    if (sample.stamp > stamp + scan_duration) {break;}
    if (!stationary(sample.pose)) {return false;}
  }
  return true;
}

nav_msgs::msg::OccupancyGrid Worker::message(const Grid & grid, bool raw) const
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = node_.now();
  msg.info.resolution = static_cast<float>(grid.resolution);
  msg.info.width = grid.width;
  msg.info.height = grid.height;
  msg.info.origin.position.x = grid.origin_x * grid.resolution;
  msg.info.origin.position.y = grid.origin_y * grid.resolution;
  msg.info.origin.orientation.w = 1.0;
  msg.data = raw ? grid.raw : grid.costs;
  return msg;
}

void Worker::publishHealthLocked()
{
  if (!health_pub_) {return;}
  const double age = have_map_ ? (node_.now().nanoseconds() - last_source_stamp_) * 1e-9 : -1.0;
  std::string reason = "ready";
  if (stopping_) {reason = "stopping";}
  else if (job_failed_) {reason = "worker_error";}
  else if (!have_map_) {reason = "waiting_for_first_map";}
  else if (completed_config_ != pending_.config_revision) {reason = "configuration_update_pending";}
  else if (completed_geometry_ != pending_.geometry_revision) {reason = "graph_correction_pending";}
  else if (age < -0.1) {reason = "map_timestamp_in_future";}
  else if (age > pending_.max_age) {reason = "published_map_stale";}
  const bool ready = reason == "ready";
  std_msgs::msg::String status;
  status.data = reason;
  status_pub_->publish(status);
  std_msgs::msg::Bool msg; msg.data = ready; health_pub_->publish(msg);
  if (reason != last_health_reason_) {
    RCLCPP_INFO(node_.get_logger(), "Hazard readiness: %s (map source age %.3fs, limit %.3fs)",
      reason.c_str(), age, pending_.max_age);
    last_health_reason_ = reason;
  }
}

void Worker::mergeObservation(const Contribution & cells, int64_t stamp,
  int anchor, const Eigen::Isometry3d & anchor_pose, bool fresh)
{
  Contribution accepted;
  accepted.reserve(cells.size());
  const auto inverse = anchor_pose.inverse();
  for (const auto & [key, cell] : cells) {
    const auto previous = observations_.find(key);
    if (previous != observations_.end() && previous->second.stamp > stamp) {continue;}
    accepted.emplace(key, cell);
  }
  terrain_.update(accepted, stamp, fresh);
  for (const auto & [key, cell] : accepted) {
    const auto & state = *terrain_.state(key);
    const Eigen::Vector3d world((key.x + 0.5) * terrain_resolution_,
      (key.y + 0.5) * terrain_resolution_, state.estimate.height);
    Eigen::Vector3d candidate = world; candidate.z() = state.candidate.height;
    observations_[key] = AnchoredCell{anchor, stamp, inverse * world,
      inverse.linear() * state.estimate.up, inverse * candidate, state};
  }
}

bool Worker::process(const Snapshot & s)
{
  if (!s.cloud) {return false;}
  const auto start = std::chrono::steady_clock::now();
  std::vector<CloudObservation> clouds;
  std::size_t enabled_origins = 0, disabled_origins = 0;
  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_rays;
  int64_t stamp = 0;
  auto queued = s.clouds;
  // Geometry/configuration changes can rebuild using the last genuinely fresh
  // scan even when there is no new batch. Ordinary ticks never replay old scans.
  if (queued.empty()) {queued.push_back({s.cloud, start});}
  for (const auto & item : queued) {
    const auto & cloud = *item.cloud;
    const int64_t acquisition_stamp = rclcpp::Time(cloud.header.stamp).nanoseconds();
    const double age = (node_.now().nanoseconds() - acquisition_stamp) * 1e-9;
    if (cloud.width * cloud.height == 0 || age < -0.1 || age > s.max_age) {return false;}
    const auto pose = poseAt(s, acquisition_stamp);
    if (!pose) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
        "Hazard worker waiting for odometry at cloud acquisition time");
      return false;
    }
    CloudObservation observation;
    const auto inverse = pose->inverse();
    auto points = pointsFromCloud(cloud);
    observation.body_points.reserve(points.size());
    for (const auto & point : points) {observation.body_points.push_back(inverse * point);}
    observation.pose = s.correction * *pose;
    observation.up_in_body = pose->linear().transpose() * Eigen::Vector3d::UnitZ();
    observation.clear_rays = rayOrigin(s, acquisition_stamp, *pose, observation.sensor_origin);
    if (observation.clear_rays) {++enabled_origins;} else {++disabled_origins;}
    if (!observation.clear_rays && !s.clouds.empty()) {pending_rays.push_back(item.cloud);}
    clouds.push_back(std::move(observation));
    stamp = std::max(stamp, acquisition_stamp);
  }

  const bool geometry_changed = !terrain_initialized_ || terrain_geometry_ != s.geometry_revision;
  const bool config_changed = !terrain_initialized_ || terrain_config_ != s.config_revision;
  const bool refilter = !terrain_initialized_ || !s.config.sameTerrain(last_config_);
  if (geometry_changed || refilter) {
    dropped_rays_ += deferred_rays_.size(); deferred_rays_.clear();
  }
  std::size_t rebuilt = 0;
  if (geometry_changed || config_changed) {
    auto previous = std::move(observations_);
    auto previous_obstacles = std::move(obstacles_);
    observations_.clear();
    obstacles_.clear();
    terrain_.reset(s.config);
    terrain_resolution_ = s.config.resolution;
    // Historical clouds are read only at initialization, graph correction, or a
    // filtering/resolution change. New graph submaps do not trigger normal refits.
    if (geometry_changed || refilter) {
      for (const auto & source : *s.sources) {
        auto points = pointsFromSource(source);
        // Historical clouds have no retained visibility evidence. Match the
        // live bootstrap guard: unsupported surfaces above the acquisition
        // body cannot establish traversable ground from an obstacle's top.
        const Eigen::Vector3d up = source.up_in_body.normalized();
        points.erase(std::remove_if(points.begin(), points.end(),
          [&](const auto & point) {return point.dot(up) > 0.0;}), points.end());
        const auto cells = project(points, source.pose, source.up_in_body, s.config);
        mergeObservation(cells, source.stamp, source.id, source.pose, false);
        ++rebuilt;
      }
    }
    if (!refilter) {
      std::unordered_map<int, const Source *> anchors;
      for (const auto & source : *s.sources) {anchors.emplace(source.id, &source);}
      for (const auto & [old_key, cell] : previous) {
        Eigen::Isometry3d anchor_pose = s.correction;
        if (cell.anchor >= 0) {
          const auto it = anchors.find(cell.anchor);
          if (it == anchors.end()) {continue;}  // Removed graph terrain cannot linger.
          anchor_pose = it->second->pose;
        }
        const Eigen::Vector3d world = anchor_pose * cell.point;
        const Key key{static_cast<int>(std::floor(world.x() / s.config.resolution)),
          static_cast<int>(std::floor(world.y() / s.config.resolution))};
        const auto before = observations_.find(key);
        if (before != observations_.end() && before->second.stamp > cell.stamp) {continue;}
        observations_[key] = cell;
        auto state = cell.state;
        state.estimate.height = world.z();
        state.estimate.up = (anchor_pose.linear() * cell.up).normalized();
        state.candidate.height = (anchor_pose * cell.candidate_point).z();
        state.candidate.up = state.estimate.up;
        terrain_.restore(key, state);
      }
      for (const auto & [old_key, obstacle] : previous_obstacles) {
        Eigen::Isometry3d anchor_pose = s.correction;
        if (obstacle.anchor >= 0) {
          const auto it = anchors.find(obstacle.anchor);
          if (it == anchors.end()) {continue;}
          anchor_pose = it->second->pose;
        }
        const Eigen::Vector3d world = anchor_pose * obstacle.point;
        const VoxelKey key{static_cast<int>(std::floor(world.x() / s.config.resolution)),
          static_cast<int>(std::floor(world.y() / s.config.resolution)),
          static_cast<int>(std::floor(world.z() / s.config.obstacle_voxel_height))};
        const auto before = obstacles_.find(key);
        // Rotation can collapse several old voxel centers into one new voxel.
        // Core restore conservatively merges latches, score and latest stamp.
        // Keep the strongest occupied sample as its geometric anchor.
        if (before == obstacles_.end() ||
          (obstacle.state.lethal && !before->second.state.lethal) ||
          (obstacle.state.lethal == before->second.state.lethal &&
          obstacle.state.log_odds > before->second.state.log_odds))
        {obstacles_[key] = obstacle;}
        terrain_.restoreObstacle(key, obstacle.state);
        obstacles_[key].state = terrain_.obstacles().at(key);
      }
    }
  }
  const int anchor = s.sources->empty() ? -1 : s.sources->back().id;
  const Eigen::Isometry3d anchor_pose = s.sources->empty() ? s.correction : s.sources->back().pose;
  const auto inverse_anchor = anchor_pose.inverse();
  terrain_.observe(clouds, stamp, !s.clouds.empty());
  // A start-stamped sweep often arrives before odometry covers its end. Keep
  // only the raw cloud reference, then trace it after that coverage arrives.
  // Latest marking runs first, so older clearing cannot erase a newer hit.
  std::vector<CloudObservation> deferred;
  int64_t deferred_stamp = 0;
  for (auto it = deferred_rays_.begin(); it != deferred_rays_.end();) {
    const int64_t t = rclcpp::Time((*it)->header.stamp).nanoseconds();
    if ((node_.now().nanoseconds() - t) * 1e-9 > s.max_age ||
      (!s.odometry.empty() && s.odometry.front().stamp > t - 100000000))
    {it = deferred_rays_.erase(it); ++dropped_rays_; continue;}
    if (deferred.size() >= static_cast<std::size_t>(s.batch_clouds)) {break;}
    const auto pose = poseAt(s, t);
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    if (!pose || !rayOrigin(s, t, *pose, origin)) {++it; continue;}
    CloudObservation observation;
    const auto inverse = pose->inverse();
    const auto points = pointsFromCloud(**it);
    observation.body_points.reserve(points.size());
    for (const auto & point : points) {observation.body_points.push_back(inverse * point);}
    observation.pose = s.correction * *pose;
    observation.up_in_body = pose->linear().transpose() * Eigen::Vector3d::UnitZ();
    observation.sensor_origin = origin; observation.clear_rays = true;
    deferred.push_back(std::move(observation)); deferred_stamp = std::max(deferred_stamp, t);
    it = deferred_rays_.erase(it);
  }
  if (!deferred.empty()) {terrain_.observe(deferred, deferred_stamp, true, false);}
  for (auto & cloud : pending_rays) {
    if (!deferred_rays_.empty() && rclcpp::Time(cloud->header.stamp) <=
      rclcpp::Time(deferred_rays_.back()->header.stamp)) {continue;}
    deferred_rays_.push_back(std::move(cloud));
    if (deferred_rays_.size() > 16) {deferred_rays_.pop_front(); ++dropped_rays_;}
  }
  const auto & grid = terrain_.finish();
  // Refits can change a neighbor's classification without observing that cell.
  // Save those latches/votes as well as fused heights for graph reprojection.
  for (const auto & key : terrain_.revisedCells()) {
    const auto it = observations_.find(key);
    const auto * state = terrain_.state(key);
    if (!state) {continue;}
    if (state->stamp == stamp || it == observations_.end()) {
      const Eigen::Vector3d world((key.x + 0.5) * terrain_resolution_,
        (key.y + 0.5) * terrain_resolution_, state->estimate.height);
      Eigen::Vector3d candidate = world; candidate.z() = state->candidate.height;
      observations_[key] = AnchoredCell{anchor, state->stamp, inverse_anchor * world,
        inverse_anchor.linear() * state->estimate.up, inverse_anchor * candidate, *state};
    } else {it->second.state = *state;}
  }
  for (const auto & key : terrain_.revisedObstacles()) {
    const auto state = terrain_.obstacles().find(key);
    if (state == terrain_.obstacles().end()) {obstacles_.erase(key); continue;}
    const auto previous = obstacles_.find(key);
    if (state->second.stamp == stamp || previous == obstacles_.end()) {
      const Eigen::Vector3d world((key.x + 0.5) * s.config.resolution,
        (key.y + 0.5) * s.config.resolution, (key.z + 0.5) * s.config.obstacle_voxel_height);
      obstacles_[key] = AnchoredObstacle{anchor, inverse_anchor * world, state->second};
    } else {previous->second.state = state->second;}
  }
  terrain_initialized_ = true;
  terrain_config_ = s.config_revision; terrain_geometry_ = s.geometry_revision;
  last_config_ = s.config;

  // Publish only a complete map for the current graph/configuration.
  std::unique_lock<std::mutex> publication_lock(mutex_);
  if (pending_.geometry_revision != s.geometry_revision ||
    pending_.config_revision != s.config_revision) {return true;}
  last_map_ = message(grid, false);
  map_pub_->publish(last_map_);
  completed_cloud_ = s.cloud_revision; completed_graph_ = s.graph_revision;
  completed_geometry_ = s.geometry_revision; completed_config_ = s.config_revision;
  last_source_stamp_ = stamp;
  have_map_ = std::any_of(grid.raw.begin(), grid.raw.end(), [](auto cost) {return cost >= 0;});
  job_failed_ = false;
  publishHealthLocked();
  publication_lock.unlock();
  if (raw_pub_->get_subscription_count() > 0) {raw_pub_->publish(message(grid, true));}
  if (slope_pub_->get_subscription_count() > 0) {
    pcl::PointCloud<pcl::PointXYZI> slopes;
    for (std::size_t i = 0; i < grid.slope_deg.size(); ++i) {
      if (!std::isfinite(grid.slope_deg[i])) {continue;}
      pcl::PointXYZI p;
      p.x = (static_cast<double>(grid.origin_x) + i % grid.width + 0.5) * grid.resolution;
      p.y = (static_cast<double>(grid.origin_y) + i / grid.width + 0.5) * grid.resolution;
      p.z = 0.05f; p.intensity = grid.slope_deg[i]; slopes.push_back(p);
    }
    sensor_msgs::msg::PointCloud2 msg; pcl::toROSMsg(slopes, msg);
    msg.header = last_map_.header; slope_pub_->publish(msg);
  }
  if (cloud_pub_->get_subscription_count() > 0) {
    pcl::PointCloud<pcl::PointXYZ> filtered;
    const auto & live = clouds.back();
    for (const auto & point : live.body_points) {
      const double z = point.dot(live.up_in_body);
      if (!point.allFinite() || z < s.config.min_height || z > s.config.max_height ||
        point.squaredNorm() - z * z > s.config.max_range * s.config.max_range) {continue;}
      const Eigen::Vector3d world = live.pose * point;
      filtered.push_back(pcl::PointXYZ(world.x(), world.y(), world.z()));
    }
    sensor_msgs::msg::PointCloud2 msg; pcl::toROSMsg(filtered, msg);
    msg.header = last_map_.header; cloud_pub_->publish(msg);
  }
  if (obstacle_pub_->get_subscription_count() > 0) {
    pcl::PointCloud<pcl::PointXYZI> obstacles;
    obstacles.reserve(terrain_.obstacles().size());
    for (const auto & [key, state] : terrain_.obstacles()) {
      pcl::PointXYZI p;
      p.x = (key.x + 0.5) * s.config.resolution;
      p.y = (key.y + 0.5) * s.config.resolution;
      p.z = (key.z + 0.5) * s.config.obstacle_voxel_height;
      p.intensity = 100.0 / (1.0 + std::exp(-state.log_odds));
      obstacles.push_back(p);
    }
    sensor_msgs::msg::PointCloud2 msg; pcl::toROSMsg(obstacles, msg);
    msg.header = last_map_.header; obstacle_pub_->publish(msg);
  }
  const double elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  const double completed_age = (node_.now().nanoseconds() - stamp) * 1e-9;
  std::ostringstream detail;
  detail << std::fixed << std::setprecision(3) << "{\"graph_revision\":" << completed_graph_
         << ",\"config_revision\":" << completed_config_ << ",\"submaps\":" << s.sources->size()
         << ",\"rebuilt_submaps\":" << rebuilt << ",\"batch_clouds\":" << clouds.size()
         << ",\"dropped_clouds\":" << s.dropped_clouds
         << ",\"changed_cells\":" << terrain_.changedCells()
         << ",\"refitted_cells\":" << terrain_.fittedCells()
         << ",\"inflated_cells\":" << terrain_.inflatedCells()
         << ",\"observed_cells\":" << terrain_.observedCells()
         << ",\"held_hazards\":" << terrain_.heldHazards()
         << ",\"cleared_hazards\":" << terrain_.clearedHazards()
         << ",\"rejected_heights\":" << terrain_.rejectedHeights()
         << ",\"obstacle_voxels\":" << terrain_.obstacles().size()
         << ",\"marked_obstacles\":" << terrain_.markedObstacles()
         << ",\"cleared_obstacles\":" << terrain_.clearedObstacles()
         << ",\"traced_rays\":" << terrain_.tracedRays()
         << ",\"skipped_rays\":" << terrain_.skippedRays()
         << ",\"ray_origins_enabled\":" << enabled_origins
         << ",\"ray_origins_disabled\":" << disabled_origins
         << ",\"deferred_ray_clouds\":" << deferred_rays_.size()
         << ",\"clearing_clouds\":" << deferred.size()
         << ",\"dropped_ray_clouds\":" << dropped_rays_
         << ",\"cells\":" << grid.costs.size()
         << ",\"known_cells\":" << std::count_if(grid.raw.begin(), grid.raw.end(), [](auto v) {return v >= 0;})
         << ",\"raw_hazards\":" << std::count(grid.raw.begin(), grid.raw.end(), 100)
         << ",\"source_age_sec\":" << completed_age << ",\"total_ms\":" << elapsed << "}";
  std_msgs::msg::String diagnostic; diagnostic.data = detail.str(); diagnostics_pub_->publish(diagnostic);
  return true;
}

void Worker::run()
{
  while (!stopping_) {
    Snapshot snapshot;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait_for(lock, std::chrono::milliseconds(20));
      if (stopping_) {break;}
      const auto now = node_.now().nanoseconds();
      while (!pending_.clouds.empty() &&
        (now - rclcpp::Time(pending_.clouds.front().cloud->header.stamp).nanoseconds()) * 1e-9 >
        pending_.max_age)
      {pending_.clouds.pop_front(); ++pending_.dropped_clouds;}
      const bool batch_due = !pending_.clouds.empty() &&
        (pending_.clouds.size() >= static_cast<std::size_t>(pending_.batch_clouds) ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
        pending_.clouds.front().arrived).count() >= pending_.update_period);
      const bool rebuild_due = pending_.cloud &&
        (pending_.config_revision != completed_config_ ||
        pending_.geometry_revision != completed_geometry_);
      if (!batch_due && !rebuild_due) {continue;}
      snapshot = pending_;
    }
    try {
      if (process(snapshot)) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto consumed = rclcpp::Time(snapshot.cloud->header.stamp).nanoseconds();
        while (!pending_.clouds.empty() &&
          rclcpp::Time(pending_.clouds.front().cloud->header.stamp).nanoseconds() <= consumed)
        {pending_.clouds.pop_front();}
      }
    } catch (const std::exception & e) {
      terrain_initialized_ = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        job_failed_ = true; publishHealthLocked();
      }
      RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
        "Hazard worker failed: %s", e.what());
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  publishHealthLocked();
}

rcl_interfaces::msg::SetParametersResult Worker::parameters(const std::vector<rclcpp::Parameter> & values)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::lock_guard<std::mutex> lock(mutex_);
  auto candidate = pending_;
  auto footprint = footprint_;
  bool changed = false;
  try {
    for (const auto & value : values) {
      const auto & name = value.get_name();
      if (name.rfind("hazard/", 0) != 0) {continue;}
      changed = true;
      auto & c = candidate.config;
      if (name == "hazard/enabled" || name == "hazard/lidar_frame") {
        throw std::invalid_argument(name + " requires restart");
      }
      else if (name == "hazard/resolution_m") {c.resolution = value.as_double();}
      else if (name == "hazard/input_min_height_m") {c.min_height = value.as_double();}
      else if (name == "hazard/input_max_height_m") {c.max_height = value.as_double();}
      else if (name == "hazard/max_range_m") {c.max_range = value.as_double();}
      else if (name == "hazard/slope_radius_m") {c.slope_radius = value.as_double();}
      else if (name == "hazard/max_slope_deg") {c.max_slope_deg = value.as_double();}
      else if (name == "hazard/lethal_inflation_m") {c.lethal_radius = value.as_double();}
      else if (name == "hazard/soft_inflation_m") {c.soft_radius = value.as_double();}
      else if (name == "hazard/unknown_cost") {c.unknown_cost = value.as_int();}
      else if (name == "hazard/footprint") {footprint = value.as_double_array(); c.footprint_radius = footprintRadius(footprint);}
      else if (name == "hazard/min_points_per_cell") {c.min_points = value.as_int();}
      else if (name == "hazard/min_plane_neighbors") {c.min_neighbors = value.as_int();}
      else if (name == "hazard/terrain_percentile") {c.terrain_percentile = value.as_double();}
      else if (name == "hazard/height_noise_m") {c.height_noise = value.as_double();}
      else if (name == "hazard/min_plane_coverage") {c.min_plane_coverage = value.as_double();}
      else if (name == "hazard/max_plane_error_m") {c.max_plane_error = value.as_double();}
      else if (name == "hazard/clear_confirmations") {c.clear_confirmations = value.as_int();}
      else if (name == "hazard/clear_slope_margin_deg") {c.clear_slope_margin = value.as_double();}
      else if (name == "hazard/obstacle_min_height_m") {c.obstacle_min_height = value.as_double();}
      else if (name == "hazard/obstacle_max_height_m") {c.obstacle_max_height = value.as_double();}
      else if (name == "hazard/obstacle_voxel_height_m") {c.obstacle_voxel_height = value.as_double();}
      else if (name == "hazard/obstacle_min_points") {c.obstacle_min_points = value.as_int();}
      else if (name == "hazard/obstacle_hit_probability") {c.obstacle_hit_probability = value.as_double();}
      else if (name == "hazard/obstacle_miss_probability") {c.obstacle_miss_probability = value.as_double();}
      else if (name == "hazard/obstacle_mark_probability") {c.obstacle_mark_probability = value.as_double();}
      else if (name == "hazard/obstacle_clear_probability") {c.obstacle_clear_probability = value.as_double();}
      else if (name == "hazard/obstacle_max_probability") {c.obstacle_max_probability = value.as_double();}
      else if (name == "hazard/max_rays") {c.max_rays = value.as_int();}
      else if (name == "hazard/max_obstacle_voxels") {c.max_obstacle_voxels = value.as_int();}
      else if (name == "hazard/max_cells") {
        if (value.as_int() < 100) {throw std::invalid_argument("hazard/max_cells must be >= 100");}
        c.max_cells = value.as_int();
      }
      else if (name == "hazard/max_input_age_sec") {candidate.max_age = value.as_double();}
      else if (name == "hazard/update_period_sec") {candidate.update_period = value.as_double();}
      else if (name == "hazard/batch_clouds") {candidate.batch_clouds = value.as_int();}
      else {throw std::invalid_argument("unknown hazard parameter: " + name);}
    }
    candidate.config.validate();
    if (!std::isfinite(candidate.max_age) || candidate.max_age <= 0.0 ||
      !std::isfinite(candidate.update_period) || candidate.update_period < 0.05 ||
      candidate.batch_clouds < 1 || candidate.batch_clouds > 16)
    {throw std::invalid_argument("invalid hazard cadence or freshness limit");}
    if (changed) {
      ++candidate.config_revision;
      pending_ = std::move(candidate);
      while (pending_.clouds.size() > static_cast<std::size_t>(pending_.batch_clouds)) {
        pending_.clouds.pop_front(); ++pending_.dropped_clouds;
      }
      footprint_ = std::move(footprint);
      publishHealthLocked();
      wake_.notify_one();
    }
  } catch (const std::exception & e) {result.successful = false; result.reason = e.what();}
  return result;
}
}  // namespace graphslam::hazard
