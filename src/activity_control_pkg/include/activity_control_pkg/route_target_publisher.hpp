#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
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
  bool is_takeover = false;
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
  Target getPublishedTarget(const Target & target) const;

  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg) const;
  bool hasFreshFineData(const rclcpp::Time & now_time) const;
  bool hasFreshAprilTagCode(const rclcpp::Time & now_time) const;

  void monitorTimerCallback();
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void fineDataCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void aprilTagCodeCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void enterVisualTakeover();
  void exitVisualTakeover();
  void advanceToNextTarget();
  void publishVisualTakeoverState(bool active);

  static double meterToCm(double value_m);
  static double radToDeg(double value_rad);
  double normalizeAngleDeg(double angle_deg) const;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_takeover_active_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr visual_aligned_apriltag_code_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr apriltag_code_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

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

  double visual_align_pixel_threshold_;
  int visual_align_required_frames_;
  double visual_takeover_target_height_cm_;
  double visual_takeover_timeout_sec_;
  double fine_data_stale_timeout_sec_;

  bool visual_takeover_active_;
  bool has_fine_data_;
  int fine_error_x_px_;
  int fine_error_y_px_;
  rclcpp::Time last_fine_data_time_;

  bool has_apriltag_code_;
  int latest_apriltag_code_;
  rclcpp::Time last_apriltag_code_time_;
  bool mission_complete_sent_;

  int aligned_frame_count_;
  rclcpp::Time visual_takeover_start_time_;
};

class RouteTestNode : public rclcpp::Node
{
public:
  explicit RouteTestNode(
    const std::shared_ptr<RouteTargetPublisherNode> & route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using RouteId = std::uint8_t;

  void routeChoiceCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  std::unordered_map<RouteId, std::vector<Target>> buildRoutes() const;
  void loadRoute(RouteId route_id, const std::vector<Target> & route);

  std::shared_ptr<RouteTargetPublisherNode> route_node_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr route_choice_sub_;
  std::unordered_map<RouteId, std::vector<Target>> routes_;
  bool route_locked_;
};

}  // namespace activity_control_pkg
