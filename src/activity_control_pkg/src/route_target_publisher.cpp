#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>

#include <algorithm>
#include <chrono>
#include <clocale>
#include <cmath>
#include <functional>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

namespace
{
constexpr double kDefaultTimerPeriodSec = 0.05;
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_idx_(std::numeric_limits<std::size_t>::max()),
  has_height_(false),
  current_height_cm_(0.0),
  visual_align_pixel_threshold_(0.0),
  visual_align_required_frames_(0),
  reach_hold_frames_(0),
  visual_takeover_timeout_sec_(0.0),
  fine_data_stale_timeout_sec_(0.0),
  visual_takeover_active_(false),
  has_fine_data_(false),
  fine_error_x_px_(0),
  fine_error_y_px_(0),
  has_apriltag_code_(false),
  latest_apriltag_code_(-1),
  mission_complete_sent_(false),
  reach_hold_count_(0),
  aligned_frame_count_(0)
{
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 9.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 12.0);
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");
  visual_align_pixel_threshold_ = declare_parameter("visual_align_pixel_threshold", 100.0);
  visual_align_required_frames_ = declare_parameter("visual_align_required_frames", 3);
  reach_hold_frames_ = declare_parameter("reach_hold_frames", 3);
  visual_takeover_timeout_sec_ = declare_parameter("visual_takeover_timeout_sec", 5.0);
  fine_data_stale_timeout_sec_ = declare_parameter("fine_data_stale_timeout_sec", 0.5);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  visual_takeover_active_pub_ =
    create_publisher<std_msgs::msg::Bool>("/visual_takeover_active", durable_qos);
  visual_aligned_apriltag_code_pub_ =
    create_publisher<std_msgs::msg::UInt8>("/visual_aligned_apriltag_code", rclcpp::QoS(10).reliable());
  mission_complete_pub_ =
    create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));
  fine_data_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/fine_data",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::fineDataCallback, this, std::placeholders::_1));
  apriltag_code_sub_ = create_subscription<std_msgs::msg::Int32>(
    "/apriltag_code",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::aprilTagCodeCallback, this, std::placeholders::_1));

  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kDefaultTimerPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  publishVisualTakeoverState(false);

  RCLCPP_INFO(
    get_logger(),
    "RouteTargetPublisher initialized: map=%s laser_link=%s topic=%s",
    map_frame_.c_str(),
    laser_link_frame_.c_str(),
    output_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm hold_frames=%d",
    pos_tol_cm_,
    yaw_tol_deg_,
    height_tol_cm_,
    std::max(reach_hold_frames_, 1));
  RCLCPP_INFO(
    get_logger(),
    "Visual takeover: threshold=%.1fpx frames=%d timeout=%.1fs stale=%.1fs",
    visual_align_pixel_threshold_,
    visual_align_required_frames_,
    visual_takeover_timeout_sec_,
    fine_data_stale_timeout_sec_);
}

void RouteTargetPublisherNode::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  const bool was_completed =
    current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size();
  targets_.push_back(target);
  if (was_empty || was_completed) {
    mission_complete_sent_ = false;
    current_idx_ = was_completed ? targets_.size() - 1 : 0;
    publishCurrent();
  }
}

std::size_t RouteTargetPublisherNode::currentIndex() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_idx_;
}

std::size_t RouteTargetPublisherNode::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

void RouteTargetPublisherNode::publishCurrent()
{
  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ < targets_.size()) {
    publishTarget(targets_[current_idx_], current_idx_ == 0);
  }
}

void RouteTargetPublisherNode::publishTarget(const Target & target, bool init_flag)
{
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(4);
  message.data[0] = static_cast<float>(target.x_cm);
  message.data[1] = static_cast<float>(target.y_cm);
  message.data[2] = static_cast<float>(target.z_cm);
  message.data[3] = 0.0f;
  target_pub_->publish(message);

  std_msgs::msg::UInt8 active_msg;
  active_msg.data = 2;
  active_controller_pub_->publish(active_msg);

  RCLCPP_INFO(
    get_logger(),
    "Published target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=0.0deg takeover=%s%s",
    target.x_cm,
    target.y_cm,
    target.z_cm,
    target.is_takeover ? "true" : "false",
    init_flag ? " (first)" : "");
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

void RouteTargetPublisherNode::fineDataCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "/fine_data requires 2 values [x_px, y_px]");
    return;
  }

  fine_error_x_px_ = msg->data[0];
  fine_error_y_px_ = msg->data[1];
  has_fine_data_ = true;
  last_fine_data_time_ = now();
}

void RouteTargetPublisherNode::aprilTagCodeCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  latest_apriltag_code_ = msg->data;
  has_apriltag_code_ = true;
  last_apriltag_code_time_ = now();
}

bool RouteTargetPublisherNode::getCurrentPose(
  double & x_cm,
  double & y_cm,
  double & z_cm,
  double & yaw_deg)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    x_cm = meterToCm(transform.transform.translation.x);
    y_cm = meterToCm(transform.transform.translation.y);
    z_cm = has_height_ ? current_height_cm_ : 0.0;

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "TF lookup failed (%s -> %s): %s",
      map_frame_.c_str(),
      laser_link_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool RouteTargetPublisherNode::isReached(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg) const
{
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dxy = std::hypot(dx, dy);
  const double dz = target.z_cm - z_cm;
  const double dyaw = normalizeAngleDeg(-yaw_deg);

  const bool z_ok = std::fabs(dz) <= height_tol_cm_;
  const bool xy_ok = dxy <= pos_tol_cm_;
  const bool yaw_ok = std::fabs(dyaw) <= yaw_tol_deg_;

  return z_ok && xy_ok && yaw_ok;
}

bool RouteTargetPublisherNode::hasFreshFineData(const rclcpp::Time & now_time) const
{
  if (!has_fine_data_ || last_fine_data_time_.nanoseconds() == 0) {
    return false;
  }
  return (now_time - last_fine_data_time_).seconds() <= fine_data_stale_timeout_sec_;
}

bool RouteTargetPublisherNode::hasFreshAprilTagCode(const rclcpp::Time & now_time) const
{
  if (!has_apriltag_code_ || last_apriltag_code_time_.nanoseconds() == 0) {
    return false;
  }
  return (now_time - last_apriltag_code_time_).seconds() <= fine_data_stale_timeout_sec_;
}

void RouteTargetPublisherNode::enterVisualTakeover()
{
  visual_takeover_active_ = true;
  reach_hold_count_ = 0;
  aligned_frame_count_ = 0;
  visual_takeover_start_time_ = now();
  publishVisualTakeoverState(true);
  RCLCPP_INFO(get_logger(), "Entered visual takeover for target %zu.", current_idx_);
}

void RouteTargetPublisherNode::exitVisualTakeover()
{
  visual_takeover_active_ = false;
  reach_hold_count_ = 0;
  aligned_frame_count_ = 0;
  publishVisualTakeoverState(false);
}

void RouteTargetPublisherNode::advanceToNextTarget()
{
  reach_hold_count_ = 0;
  ++current_idx_;
  if (current_idx_ < targets_.size()) {
    publishCurrent();
  } else {
    current_idx_ = targets_.size();
    if (!mission_complete_sent_ && mission_complete_pub_) {
      std_msgs::msg::Empty mission_complete_msg;
      mission_complete_pub_->publish(mission_complete_msg);
      mission_complete_sent_ = true;
    }
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    RCLCPP_INFO(get_logger(), "All targets completed.");
  }
}

void RouteTargetPublisherNode::publishVisualTakeoverState(bool active)
{
  std_msgs::msg::Bool msg;
  msg.data = active;
  visual_takeover_active_pub_->publish(msg);
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size()) {
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    if (visual_takeover_active_) {
      exitVisualTakeover();
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "All targets completed. Keeping stop signal active.");
    return;
  }

  if (current_idx_ == std::numeric_limits<std::size_t>::max()) {
    return;
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  const Target & target = targets_[current_idx_];
  const rclcpp::Time now_time = now();

  if (visual_takeover_active_) {
    const double elapsed = (now_time - visual_takeover_start_time_).seconds();
    if (elapsed > visual_takeover_timeout_sec_) {
      RCLCPP_WARN(
        get_logger(),
        "Visual takeover timed out for target %zu after %.1fs. Skipping.",
        current_idx_,
        elapsed);
      exitVisualTakeover();
      advanceToNextTarget();
      return;
    }

    if (!hasFreshFineData(now_time)) {
      aligned_frame_count_ = 0;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Waiting for fresh /fine_data while visual takeover is active.");
      return;
    }

    const double pixel_radius = std::hypot(
      static_cast<double>(fine_error_x_px_),
      static_cast<double>(fine_error_y_px_));

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Visual takeover target %zu: x_px=%d y_px=%d radius=%.1f threshold=%.1f frames=%d/%d",
      current_idx_,
      fine_error_x_px_,
      fine_error_y_px_,
      pixel_radius,
      visual_align_pixel_threshold_,
      aligned_frame_count_,
      visual_align_required_frames_);

    if (pixel_radius < visual_align_pixel_threshold_) {
      ++aligned_frame_count_;
      if (aligned_frame_count_ >= visual_align_required_frames_) {
        if (hasFreshAprilTagCode(now_time) && latest_apriltag_code_ >= 0 && latest_apriltag_code_ <= 255) {
          std_msgs::msg::UInt8 apriltag_msg;
          apriltag_msg.data = static_cast<uint8_t>(latest_apriltag_code_);
          visual_aligned_apriltag_code_pub_->publish(apriltag_msg);
          RCLCPP_INFO(
            get_logger(),
            "Visual takeover succeeded for target %zu. Published aligned AprilTag code %u.",
            current_idx_,
            static_cast<unsigned>(apriltag_msg.data));
        } else {
          RCLCPP_WARN(
            get_logger(),
            "Visual takeover succeeded for target %zu, but no fresh valid AprilTag code is available.",
            current_idx_);
        }

        exitVisualTakeover();
        advanceToNextTarget();
      }
    } else {
      aligned_frame_count_ = 0;
    }
    return;
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    5000,
    "Current target %zu: x=%.1f y=%.1f z=%.1f yaw=0.0 takeover=%s",
    current_idx_,
    target.x_cm,
    target.y_cm,
    target.z_cm,
    target.is_takeover ? "true" : "false");

  if (isReached(target, x_cm, y_cm, z_cm, yaw_deg)) {
    ++reach_hold_count_;
    const int required_hold = std::max(reach_hold_frames_, 1);
    if (reach_hold_count_ < required_hold) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "Target %zu within tolerance: hold=%d/%d pos=(%.1f, %.1f, %.1f)cm yaw=%.1fdeg",
        current_idx_,
        reach_hold_count_,
        required_hold,
        x_cm,
        y_cm,
        z_cm,
        yaw_deg);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Target %zu reached: pos=(%.1f, %.1f, %.1f)cm yaw=%.1fdeg hold=%d",
      current_idx_,
      x_cm,
      y_cm,
      z_cm,
      yaw_deg,
      reach_hold_count_);

    if (target.is_takeover) {
      enterVisualTakeover();
      return;
    }

    advanceToNextTarget();
  } else if (reach_hold_count_ != 0) {
    reach_hold_count_ = 0;
  }
}

double RouteTargetPublisherNode::meterToCm(double value_m)
{
  return value_m * 100.0;
}

double RouteTargetPublisherNode::radToDeg(double value_rad)
{
  return value_rad * 180.0 / M_PI;
}

double RouteTargetPublisherNode::normalizeAngleDeg(double angle_deg) const
{
  const double normalized = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(normalized);
}

RouteTestNode::RouteTestNode(
  const std::shared_ptr<RouteTargetPublisherNode> & route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("route_test_node", options),
  route_node_(route_node),
  started_(false),
  next_target_index_(1)
{
  std::setlocale(LC_ALL, "");

  add_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&RouteTestNode::addTimerCallback, this));
  add_timer_->cancel();

  RCLCPP_INFO(get_logger(), "Route test node started. Adding the first target.");

  const Target first{0.0, 0.0, 130.0, 0.0, false};
  route_node_->addTarget(first);

  const auto current = route_node_->currentIndex();
  RCLCPP_INFO(
    get_logger(),
    "Added first target: x=%.1f y=%.1f z=%.1f yaw=%.1f | current=%zu",
    first.x_cm,
    first.y_cm,
    first.z_cm,
    first.yaw_deg,
    (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));

  add_timer_->reset();
  started_ = true;
}

void RouteTestNode::addTimerCallback()
{
  if (!started_) {
    return;
  }

  Target target{};
  switch (next_target_index_) {
    case 1:
      target = Target{100.0, 0.0, 130.0, 0.0, false};
      break;
    case 2:
      target = Target{100.0, 50.0, 130.0, 0.0, false};
      break;
    case 3:
      target = Target{100.0, 50.0, 0.0, 0.0, false};
      break;
 
    default:
      add_timer_->cancel();
      RCLCPP_INFO(get_logger(), "All preset targets have been added.");
      return;
  }

  route_node_->addTarget(target);
  const auto current = route_node_->currentIndex();
  RCLCPP_INFO(
    get_logger(),
    "Queued target idx=%d: x=%.1f y=%.1f z=%.1f yaw=%.1f takeover=%s | current=%zu",
    next_target_index_,
    target.x_cm,
    target.y_cm,
    target.z_cm,
    target.yaw_deg,
    target.is_takeover ? "true" : "false",
    (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));

  ++next_target_index_;
}

}  // namespace activity_control_pkg
