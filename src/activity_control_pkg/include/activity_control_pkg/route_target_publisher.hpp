#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

struct Target
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
  bool require_visual_align = false;  // 需要二维码对准后才允许推进航点
  uint8_t camera_side = 0;            // 0=none, 1=right, 2=left
};

class RouteTargetPublisherNode : public rclcpp::Node
{
public:
  explicit RouteTargetPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void addTarget(const Target & target);

  std::size_t currentIndex() const;

  std::size_t size() const;

private:
  void publishCurrent();
  void publishTarget(const Target & target, bool init_flag);

  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg) const;
  bool isNearXY(const Target & target, double x_cm, double y_cm) const;

  void monitorTimerCallback();
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void qrAlignedCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void qrFineOffsetCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void qrAlignedRightCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void qrAlignedLeftCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void qrFineOffsetRightCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void qrFineOffsetLeftCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  // void bluetooth_data_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  
  static double meterToCm(double value_m);
  static double radToDeg(double value_rad);
  double normalizeAngleDeg(double angle_deg) const;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr camera_config_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr qr_aligned_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr qr_fine_offset_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr qr_aligned_right_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr qr_aligned_left_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr qr_fine_offset_right_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr qr_fine_offset_left_sub_;
  // rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr bluetooth_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


  mutable std::mutex mutex_;
  std::vector<Target> targets_;
  std::size_t current_idx_;

  bool has_height_;
  double current_height_cm_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;


  std::string map_frame_;
  std::string laser_link_frame_;
  std::string output_topic_;

  // 视觉相关状态（由独立节点提供）
  bool qr_aligned_{false};
  bool has_qr_aligned_{false};
  geometry_msgs::msg::Point qr_fine_offset_body_cm_{};  // x=forward_cm, y=right_cm
  bool has_qr_fine_offset_{false};

  bool qr_right_aligned_{false};
  bool has_qr_right_aligned_{false};
  geometry_msgs::msg::Point qr_right_fine_offset_body_cm_{};  // x=body_dx, y=body_dy, z=body_dz
  bool has_qr_right_fine_offset_{false};

  bool qr_left_aligned_{false};
  bool has_qr_left_aligned_{false};
  geometry_msgs::msg::Point qr_left_fine_offset_body_cm_{};
  bool has_qr_left_fine_offset_{false};

  // 视觉接管参数
  double visual_takeover_distance_cm_{10.0};
  double fine_offset_limit_cm_{15.0};
  double laser_hold_sec_{0.5};
  double fine_target_publish_hz_{5.0};
  bool enable_visual_takeover_{false};  // 总开关：默认不影响现有逻辑
  bool enable_visual_align_for_low_z_targets_{false};
  double visual_align_z_threshold_cm_{20.0};

  // 激光等待窗口（占位：只做时间窗口与航点推进门控，激光控制可后续接入）
  bool laser_hold_active_{false};
  rclcpp::Time laser_hold_start_;

  // 微调目标点：使用 anchor 防止 “current_pose + offset” 变成移动靶
  bool fine_anchor_valid_{false};
  std::size_t fine_anchor_target_idx_{0};
  double fine_anchor_x_cm_{0.0};
  double fine_anchor_y_cm_{0.0};
  double fine_anchor_z_cm_{0.0};
  rclcpp::Time fine_last_publish_time_;
};

class RouteTestNode : public rclcpp::Node
{
public:
  explicit RouteTestNode(const std::shared_ptr<RouteTargetPublisherNode> & route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // void bluetoothCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void addTimerCallback();

  std::shared_ptr<RouteTargetPublisherNode> route_node_;
  // rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr bluetooth_sub_;
  rclcpp::TimerBase::SharedPtr add_timer_;

  bool started_;
  int next_target_index_;
};

}  // namespace activity_control_pkg



















// #pragma once

// #include <mutex>
// #include <string>
// #include <vector>

// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/float32_multi_array.hpp>
// #include <std_msgs/msg/int16.hpp>
// #include <std_msgs/msg/u_int8.hpp>
// #include <std_msgs/msg/u_int8_multi_array.hpp>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>

// namespace activity_control_pkg
// {

// struct Target
// {
//   double x_cm;
//   double y_cm;
//   double z_cm;
//   double yaw_deg;
// };

// class RouteTargetPublisherNode : public rclcpp::Node
// {
// public:
//   explicit RouteTargetPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

//   void addTarget(const Target & target);

//   std::size_t currentIndex() const;

//   std::size_t size() const;

// private:
//   void publishCurrent();
//   void publishTarget(const Target & target, bool init_flag);

//   bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
//   bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg) const;

//   void monitorTimerCallback();
//   void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
//   // void bluetooth_data_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  
//   static double meterToCm(double value_m);
//   static double radToDeg(double value_rad);
//   double normalizeAngleDeg(double angle_deg) const;

//   rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
//   rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;
//   rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
//   rclcpp::TimerBase::SharedPtr monitor_timer_;
//   // rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr bluetooth_sub_;

//   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


//   mutable std::mutex mutex_;
//   std::vector<Target> targets_;
//   std::size_t current_idx_;

//   bool has_height_;
//   double current_height_cm_;

//   double pos_tol_cm_;
//   double yaw_tol_deg_;
//   double height_tol_cm_;


//   std::string map_frame_;
//   std::string laser_link_frame_;
//   std::string output_topic_;

//   // uint8_t control_mode_ = 0;
// };

// class RouteTestNode : public rclcpp::Node
// {
// public:
//   explicit RouteTestNode(const std::shared_ptr<RouteTargetPublisherNode> & route_node,
//     const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

// private:
//   // void bluetoothCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
//   void addTimerCallback();

//   std::shared_ptr<RouteTargetPublisherNode> route_node_;
//   // rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr bluetooth_sub_;
//   rclcpp::TimerBase::SharedPtr add_timer_;

//   bool started_;
//   int next_target_index_;
// };

// }  // namespace activity_control_pkg
