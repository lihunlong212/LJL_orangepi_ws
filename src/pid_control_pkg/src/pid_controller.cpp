#include "pid_control_pkg/pid_controller.hpp"

#include <angles/angles.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pid_control_pkg
{

PIDController::PIDController(
  double kp,
  double ki,
  double kd,
  double max_output,
  double min_output,
  double integral_limit,
  double deadzone)
: kp_(kp),
  ki_(ki),
  kd_(kd),
  max_output_(max_output),
  min_output_(min_output),
  integral_limit_(integral_limit),
  deadzone_(deadzone),
  prev_error_(0.0),
  current_error_(0.0),
  integral_(0.0),
  prev_derivative_(0.0),
  first_call_(true),
  derivative_filter_alpha_(0.8)
{
}

double PIDController::calculate(double setpoint, double measured_value, double dt)
{
  current_error_ = setpoint - measured_value;

  if (std::fabs(current_error_) < deadzone_) {
    current_error_ = 0.0;
  }

  if (first_call_) {
    prev_error_ = current_error_;
    first_call_ = false;
  }

  const double proportional = kp_ * current_error_;

  integral_ += current_error_ * dt;
  if (integral_ > integral_limit_) {
    integral_ = integral_limit_;
  } else if (integral_ < -integral_limit_) {
    integral_ = -integral_limit_;
  }
  const double integral_term = ki_ * integral_;

  const double derivative_raw = (dt > 0.0) ? (current_error_ - prev_error_) / dt : 0.0;
  const double derivative_filtered = derivative_filter_alpha_ * prev_derivative_ +
    (1.0 - derivative_filter_alpha_) * derivative_raw;
  const double derivative_term = kd_ * derivative_filtered;

  double output = proportional + integral_term + derivative_term;

  if (output > max_output_) {
    output = max_output_;
  } else if (output < min_output_) {
    output = min_output_;
  }

  if ((output >= max_output_ && current_error_ > 0.0) ||
    (output <= min_output_ && current_error_ < 0.0))
  {
    integral_ -= current_error_ * dt;
  }

  prev_error_ = current_error_;
  prev_derivative_ = derivative_filtered;

  return output;
}

void PIDController::reset()
{
  prev_error_ = 0.0;
  current_error_ = 0.0;
  integral_ = 0.0;
  prev_derivative_ = 0.0;
  first_call_ = true;
}

void PIDController::setPID(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setOutputLimits(double max_output, double min_output)
{
  max_output_ = max_output;
  min_output_ = min_output;
}

void PIDController::setIntegralLimit(double integral_limit)
{
  integral_limit_ = integral_limit;
}

void PIDController::setDeadzone(double deadzone)
{
  deadzone_ = deadzone;
}
/*
  订阅/发布主题：
    订阅 /target_position (std_msgs::msg::Float32MultiArray)：接收目标位置和朝向，包含四个浮点数 [x_cm, y_cm, z_cm, yaw_deg]。
    订阅 /height 
    发布 /target_velocity (std_msgs::msg::Float32MultiArray)
*/
PositionPIDController::PositionPIDController()
: rclcpp::Node("position_pid_controller"),
  enable_visual_fine_tune_(true),
  pid_x_(0.8, 0.0, 0.2, 36.0, -33.0, 5.0, 0.6),
  pid_y_(0.8, 0.0, 0.2, 36.0, -33.0, 5.0, 0.6),
  pid_yaw_(1.0, 0.0, 0.2, 30.0, -30.0, 2.0, 0.5),
  pid_z_(1.0, 0.0, 0.2, 25.0, -60.0, 3.0, 0.6),
  pid_xy_speed_(0.8, 0.0, 0.2, 36.0, -36.0, 5.0, 0.6),
  target_x_cm_(0.0),
  target_y_cm_(0.0),
  target_z_cm_(0.0),
  target_yaw_deg_(0.0),
  has_target_position_(false),
  has_target_height_(false),
  current_x_cm_(0.0),
  current_y_cm_(0.0),
  current_yaw_deg_(0.0),
  current_z_cm_(0.0),
  has_current_pose_(false),
  control_frequency_(50.0),
  map_frame_("map"),
  laser_link_frame_("laser_link"),
  control_mode_(ControlMode::NORMAL),
  position_tolerance_(6.0),
  yaw_tolerance_(5.0),
  height_tolerance_(6.0),
  max_linear_vel_(36.0),
  max_angular_vel_(30.0),
  max_vertical_vel_(30.0),
  max_slow_vel_(20.0),
  distance_xy_cm_(0.0),
  error_x_cm_(0.0),
  error_y_cm_(0.0),
  error_yaw_deg_(0.0),
  error_z_cm_(0.0),
  // is_active_controller_(false),
  // is_emergency_landing_(false),
  // should_stop_(false),
  last_update_time_(now())
{
  loadParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  target_position_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/target_position", rclcpp::QoS(10),
    std::bind(&PositionPIDController::targetPositionCallback, this, std::placeholders::_1));


  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&PositionPIDController::heightCallback, this, std::placeholders::_1));

  target_velocity_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/target_velocity", rclcpp::QoS(10));

  const double period_sec = 1.0 / std::max(control_frequency_, 1.0);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_sec)),
    std::bind(&PositionPIDController::controlTimerCallback, this));

  RCLCPP_INFO(get_logger(), "Position PID Controller initialized (%.1f Hz)", control_frequency_);
  RCLCPP_INFO(get_logger(), "Frames: map=%s, laser_link=%s", map_frame_.c_str(), laser_link_frame_.c_str());
}


void PositionPIDController::targetPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN(get_logger(), "Target position message requires 4 floats [x_cm, y_cm, z_cm, yaw_deg]");
    return;
  }

  target_x_cm_ = static_cast<double>(msg->data[0]);
  target_y_cm_ = static_cast<double>(msg->data[1]);
  target_z_cm_ = static_cast<double>(msg->data[2]);
  target_yaw_deg_ = static_cast<double>(msg->data[3]);
  has_target_position_ = true;

  RCLCPP_INFO(get_logger(),
    "Received target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",
    target_x_cm_, target_y_cm_, target_z_cm_, target_yaw_deg_);
}

void PositionPIDController::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_z_cm_ = static_cast<double>(msg->data);
  has_target_height_ = true;
}
/*
    获取自动获取tf数据
*/
bool PositionPIDController::getCurrentPose()
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);

    current_x_cm_ = meterToCm(transform.transform.translation.x);
    current_y_cm_ = meterToCm(transform.transform.translation.y);

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_deg_ = radToDeg(yaw);

    has_current_pose_ = true;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to lookup transform %s -> %s: %s",
      map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

double PositionPIDController::normalizeAngleDeg(double angle_deg) const
{
  double result = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(result);
}

void PositionPIDController::calculateErrors()
{
  error_x_cm_ = target_x_cm_ - current_x_cm_;
  error_y_cm_ = target_y_cm_ - current_y_cm_;
  distance_xy_cm_ = std::hypot(error_x_cm_, error_y_cm_);
  error_yaw_deg_ = normalizeAngleDeg(target_yaw_deg_ - current_yaw_deg_);
  if (has_target_height_) {
    error_z_cm_ = target_z_cm_ - current_z_cm_;
  } else {
    error_z_cm_ = 0.0;
  }
}

bool PositionPIDController::isTargetReached() const
{
  return std::fabs(error_x_cm_) <= position_tolerance_ &&
         std::fabs(error_y_cm_) <= position_tolerance_ &&
         std::fabs(error_yaw_deg_) <= yaw_tolerance_ &&
         (!has_target_height_ || std::fabs(error_z_cm_) <= height_tolerance_);
}

void PositionPIDController::setControlMode(ControlMode mode)
{
  control_mode_ = mode;
  double limit = max_linear_vel_;

  switch (mode) {
    case ControlMode::NORMAL:
      limit = max_linear_vel_;
      break;
    case ControlMode::SLOW:
    case ControlMode::LOCK_Y:
    case ControlMode::LOCK_X:
      limit = max_slow_vel_;
      break;
    case ControlMode::HOVER:
      limit = 10.0;
      break;
  }

  pid_x_.setOutputLimits(limit, -limit);
  pid_y_.setOutputLimits(limit, -limit);
  pid_xy_speed_.setOutputLimits(limit, -limit);
}

std_msgs::msg::Float32MultiArray PositionPIDController::processPID(double dt)
{
  std_msgs::msg::Float32MultiArray cmd;
  cmd.data.resize(4);

  calculateErrors();

  double vel_x_cm = 0.0;
  double vel_y_cm = 0.0;

  switch (control_mode_) {
    case ControlMode::NORMAL:
    case ControlMode::SLOW:
    {
      if (distance_xy_cm_ > 0.1) {
        double speed_cmd = -pid_xy_speed_.calculate(0.0, distance_xy_cm_, dt);
        if (speed_cmd < 0.0) {
          speed_cmd = 0.0;
        }
        const double cos_theta = error_x_cm_ / distance_xy_cm_;
        const double sin_theta = error_y_cm_ / distance_xy_cm_;
        vel_x_cm = speed_cmd * cos_theta;
        vel_y_cm = speed_cmd * sin_theta;
      } else {
        vel_x_cm = 0.0;
        vel_y_cm = 0.0;
      }
      break;
    }
    case ControlMode::LOCK_Y:
      vel_y_cm = pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
      vel_x_cm = 0.4 * pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
      break;
    case ControlMode::LOCK_X:
      vel_x_cm = pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
      vel_y_cm = 0.4 * pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
      break;
    case ControlMode::HOVER:
      vel_x_cm = pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
      vel_y_cm = pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
      break;
  }

  const double vel_yaw_deg = pid_yaw_.calculate(0.0, -error_yaw_deg_, dt);
  
  double vel_z_cm = 0.0;
  if (has_target_height_) {
    vel_z_cm = pid_z_.calculate(target_z_cm_, current_z_cm_, dt);
  } else {
    // 如果有目标高度但没有高度反馈，打印警告
    if (std::fabs(target_z_cm_) > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for height data... Z velocity suppressed (Target Z=%.1f)", target_z_cm_);
    }
    vel_z_cm = 0.0;
  }

  cmd.data[0] = static_cast<float>(vel_x_cm);
  cmd.data[1] = static_cast<float>(vel_y_cm);
  cmd.data[2] = static_cast<float>(vel_z_cm);
  cmd.data[3] = static_cast<float>(vel_yaw_deg);
  return cmd;
}

void PositionPIDController::controlTimerCallback()
{
  if (!has_target_position_) {
    return;
  }

  if (!getCurrentPose()) {
    return;
  }

  const rclcpp::Time now_time = now();
  double dt = (now_time - last_update_time_).seconds();
  if (dt <= 0.0) {
    dt = 1.0 / std::max(control_frequency_, 1.0);
  }
  last_update_time_ = now_time;

  auto cmd_vel = processPID(dt);
  
  
  target_velocity_pub_->publish(cmd_vel);
  

  if (isTargetReached()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Target reached: distance=%.1fcm yaw_error=%.1fdeg",
      distance_xy_cm_, error_yaw_deg_);
  }

  RCLCPP_DEBUG(get_logger(),
    "Current[%.1f, %.1f, %.1fdeg] Target[%.1f, %.1f, %.1fdeg] Error[%.1f, %.1f, %.1fdeg]",
    current_x_cm_, current_y_cm_, current_yaw_deg_,
    target_x_cm_, target_y_cm_, target_yaw_deg_,
    error_x_cm_, error_y_cm_, error_yaw_deg_);
}

void PositionPIDController::loadParameters()
{
  control_frequency_ = declare_parameter<double>("control_frequency", 50.0);
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  laser_link_frame_ = declare_parameter<std::string>("laser_link_frame", "laser_link");
  position_tolerance_ = declare_parameter<double>("position_tolerance", 6.0);
  yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 5.0);
  height_tolerance_ = declare_parameter<double>("height_tolerance", 6.0);

  const double kp_xy = declare_parameter<double>("kp_xy", 0.8);
  const double ki_xy = declare_parameter<double>("ki_xy", 0.0);
  const double kd_xy = declare_parameter<double>("kd_xy", 0.2);

  const double kp_yaw = declare_parameter<double>("kp_yaw", 1.0);
  const double ki_yaw = declare_parameter<double>("ki_yaw", 0.0);
  const double kd_yaw = declare_parameter<double>("kd_yaw", 0.2);

  const double kp_z = declare_parameter<double>("kp_z", 1.0);
  const double ki_z = declare_parameter<double>("ki_z", 0.0);
  const double kd_z = declare_parameter<double>("kd_z", 0.2);

  max_linear_vel_ = declare_parameter<double>("max_linear_velocity", 33.0);
  max_angular_vel_ = declare_parameter<double>("max_angular_velocity", 30.0);
  max_vertical_vel_ = declare_parameter<double>("max_vertical_velocity", 30.0);
  max_slow_vel_ = declare_parameter<double>("max_slow_velocity", 20.0);

  pid_x_.setPID(kp_xy, ki_xy, kd_xy);
  pid_y_.setPID(kp_xy, ki_xy, kd_xy);
  pid_yaw_.setPID(kp_yaw, ki_yaw, kd_yaw);
  pid_z_.setPID(kp_z, ki_z, kd_z);
  pid_xy_speed_.setPID(kp_xy, ki_xy, kd_xy);

  pid_x_.setOutputLimits(max_linear_vel_, -max_linear_vel_);
  pid_y_.setOutputLimits(max_linear_vel_, -max_linear_vel_);
  pid_yaw_.setOutputLimits(max_angular_vel_, -max_angular_vel_);
  pid_z_.setOutputLimits(max_vertical_vel_, -60.0);
  pid_xy_speed_.setOutputLimits(max_linear_vel_, -max_linear_vel_);

  RCLCPP_INFO(get_logger(),
    "PID params: XY(kp=%.2f, ki=%.2f, kd=%.2f) Yaw(kp=%.2f, ki=%.2f, kd=%.2f) Z(kp=%.2f, ki=%.2f, kd=%.2f)",
    kp_xy, ki_xy, kd_xy, kp_yaw, ki_yaw, kd_yaw, kp_z, ki_z, kd_z);
  RCLCPP_INFO(get_logger(),
    "Tolerances: pos=%.1fcm yaw=%.1fdeg height=%.1fcm",
    position_tolerance_, yaw_tolerance_, height_tolerance_);
  RCLCPP_INFO(get_logger(),
    "Velocity limits: linear=%.1fcm/s angular=%.1fdeg/s vertical=%.1fcm/s slow=%.1fcm/s",
    max_linear_vel_, max_angular_vel_, max_vertical_vel_, max_slow_vel_);
}

}  // namespace pid_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pid_control_pkg::PositionPIDController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
