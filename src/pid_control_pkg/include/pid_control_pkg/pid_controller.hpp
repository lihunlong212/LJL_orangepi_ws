#ifndef PID_CONTROL_PKG__PID_CONTROLLER_HPP_
#define PID_CONTROL_PKG__PID_CONTROLLER_HPP_

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pid_control_pkg
{

class PIDController
{
public:
  PIDController(
    double kp,
    double ki,
    double kd,
    double max_output = 1.0,
    double min_output = -1.0,
    double integral_limit = 10.0,
    double deadzone = 0.0);

  double calculate(double setpoint, double measured_value, double dt);
  void reset();
  void setPID(double kp, double ki, double kd);
  void setOutputLimits(double max_output, double min_output);
  void setIntegralLimit(double integral_limit);
  void setDeadzone(double deadzone);

  double getError() const { return current_error_; }
  double getIntegral() const { return integral_; }

private:
  double kp_;
  double ki_;
  double kd_;
  double max_output_;
  double min_output_;
  double integral_limit_;
  double deadzone_;
  double prev_error_;
  double current_error_;
  double integral_;
  double prev_derivative_;
  bool first_call_;
  double derivative_filter_alpha_;
};

class PositionPIDController : public rclcpp::Node
{
public:
  PositionPIDController();
  ~PositionPIDController() override = default;

private:
  void targetPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void visualTakeoverCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void fineDataCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void controlTimerCallback();

  bool getCurrentPose();
  bool hasFreshVisualData(const rclcpp::Time & now_time) const;
  void loadParameters();
  void calculateErrors();
  double normalizeAngleDeg(double angle_deg) const;
  std_msgs::msg::Float32MultiArray processPID(double dt);

  inline double meterToCm(double meter) const { return meter * 100.0; }
  inline double radToDeg(double rad) const { return rad * 180.0 / M_PI; }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visual_takeover_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  PIDController pid_yaw_;
  PIDController pid_z_;
  PIDController pid_xy_speed_;
  PIDController pid_visual_x_;
  PIDController pid_visual_y_;

  double target_x_cm_;
  double target_y_cm_;
  double target_z_cm_;
  double target_yaw_deg_;
  bool has_target_position_;
  bool has_target_height_;

  double current_x_cm_;
  double current_y_cm_;
  double current_yaw_deg_;
  double current_z_cm_;

  double control_frequency_;
  std::string map_frame_;
  std::string laser_link_frame_;

  double max_linear_vel_;
  double max_angular_vel_;
  double max_vertical_vel_;

  double visual_kp_x_;
  double visual_ki_x_;
  double visual_kd_x_;
  double visual_kp_y_;
  double visual_ki_y_;
  double visual_kd_y_;
  double visual_pixel_deadzone_;
  double visual_max_xy_velocity_;
  double visual_data_timeout_sec_;

  double distance_xy_cm_;
  double error_x_cm_;
  double error_y_cm_;
  double error_yaw_deg_;
  double error_z_cm_;

  bool visual_takeover_active_;
  bool has_visual_fine_data_;
  double visual_error_x_px_;
  double visual_error_y_px_;
  rclcpp::Time last_visual_data_time_;

  rclcpp::Time last_update_time_;
};

}  // 命名空间 pid_control_pkg

#endif  // 头文件保护宏 PID_CONTROL_PKG__PID_CONTROLLER_HPP_
