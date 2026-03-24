#include "activity_control_pkg/route_target_publisher.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <angles/angles.h>
#include <clocale>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define fly_start 1


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
  current_height_cm_(0.0)
{
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 9.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 12.0);
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");

  enable_visual_takeover_ = declare_parameter("enable_visual_takeover", false);
  visual_takeover_distance_cm_ = declare_parameter("visual_takeover_distance_cm", 10.0);
  fine_offset_limit_cm_ = declare_parameter("fine_offset_limit_cm", 12.0);
  laser_hold_sec_ = declare_parameter("laser_hold_sec", 0.3);
  fine_target_publish_hz_ = declare_parameter("fine_target_publish_hz", 5.0);
  enable_visual_align_for_low_z_targets_ = declare_parameter("enable_visual_align_for_low_z_targets", false);
  visual_align_z_threshold_cm_ = declare_parameter("visual_align_z_threshold_cm", 20.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", qos);
  camera_config_pub_ = create_publisher<std_msgs::msg::UInt8>("/current_target_camera", qos);
  
  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));

  // 订阅左右相机二维码话题（避免冲突，按航点选择使用哪一个）
  qr_aligned_right_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/qr_right/aligned", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::qrAlignedRightCallback, this, std::placeholders::_1));
  qr_aligned_left_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/qr_left/aligned", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::qrAlignedLeftCallback, this, std::placeholders::_1));

  // 订阅左右微调输出（body(cm) 三轴）
  qr_fine_offset_right_sub_ = create_subscription<geometry_msgs::msg::Point>(
    "/qr_right/fine_offset_body_cm", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::qrFineOffsetRightCallback, this, std::placeholders::_1));
  qr_fine_offset_left_sub_ = create_subscription<geometry_msgs::msg::Point>(
    "/qr_left/fine_offset_body_cm", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::qrFineOffsetLeftCallback, this, std::placeholders::_1));

  // bluetooth_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
  //     "/bluetooth_data", 10,
  //     std::bind(&RouteTargetPublisherNode::bluetooth_data_callback, this, std::placeholders::_1));

  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kDefaultTimerPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  RCLCPP_INFO(get_logger(),
    "RouteTargetPublisher initialized: map=%s laser_link=%s topic=%s", map_frame_.c_str(),
    laser_link_frame_.c_str(), output_topic_.c_str());
  RCLCPP_INFO(get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm",
    pos_tol_cm_, yaw_tol_deg_, height_tol_cm_);
  RCLCPP_INFO(get_logger(),
    "Visual takeover: enable=%s near=%.1fcm fine_limit=%.1fcm laser_hold=%.2fs low_z_enable=%s z_th=%.1fcm",
    enable_visual_takeover_ ? "true" : "false",
    visual_takeover_distance_cm_,
    fine_offset_limit_cm_,
    laser_hold_sec_,
    enable_visual_align_for_low_z_targets_ ? "true" : "false",
    visual_align_z_threshold_cm_);
  RCLCPP_INFO(get_logger(), "Fine target publish hz: %.1f", fine_target_publish_hz_);
}

void RouteTargetPublisherNode::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  targets_.push_back(target);
  if (was_empty) {
    current_idx_ = 0;
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
  message.data[3] = static_cast<float>(target.yaw_deg);
  target_pub_->publish(message);

  std_msgs::msg::UInt8 active_msg;
  // 纯飞机控制：始终使用飞机控制器
  active_msg.data = 2; // Drone
  // RCLCPP_INFO(get_logger(), "激活控制器: 飞机 (Drone)");
  active_controller_pub_->publish(active_msg);

  RCLCPP_INFO(get_logger(),
    "发布目标: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg%s",
    target.x_cm, target.y_cm, target.z_cm, target.yaw_deg,
    init_flag ? " (首个)" : "");
  std_msgs::msg::UInt8 camera_config_msg;
  camera_config_msg.data = target.camera_side;  // 0=none, 1=right, 2=left
  camera_config_pub_->publish(camera_config_msg);
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

void RouteTargetPublisherNode::qrAlignedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  qr_aligned_ = msg->data;
  has_qr_aligned_ = true;
}

void RouteTargetPublisherNode::qrFineOffsetCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  qr_fine_offset_body_cm_ = *msg;
  has_qr_fine_offset_ = true;
}

void RouteTargetPublisherNode::qrAlignedRightCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  qr_right_aligned_ = msg->data;
  has_qr_right_aligned_ = true;
}

void RouteTargetPublisherNode::qrAlignedLeftCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  qr_left_aligned_ = msg->data;
  has_qr_left_aligned_ = true;
}

void RouteTargetPublisherNode::qrFineOffsetRightCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  qr_right_fine_offset_body_cm_ = *msg;
  has_qr_right_fine_offset_ = true;
}

void RouteTargetPublisherNode::qrFineOffsetLeftCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  qr_left_fine_offset_body_cm_ = *msg;
  has_qr_left_fine_offset_ = true;
}

// void RouteTargetPublisherNode::bluetooth_data_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
//     control_mode_ = msg->data[0];
//     if (control_mode_ == 1) {
//       RCLCPP_INFO(this->get_logger(), "Received control mode 1.");
//     } else {
//       RCLCPP_INFO(this->get_logger(), "Received control mode %d.", control_mode_);
//     }
//   }


bool RouteTargetPublisherNode::getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    x_cm = meterToCm(transform.transform.translation.x);
    y_cm = meterToCm(transform.transform.translation.y);
    z_cm = has_height_ ? current_height_cm_ : 0.0;
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "TF 查询失败 (%s->%s): %s", map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
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
  const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);
  
  // 纯飞机控制：始终使用飞机的容忍度
  const double z_tol = height_tol_cm_;
  const double xy_tol = pos_tol_cm_;
  
  const bool z_ok = (std::fabs(dz) <= z_tol);
  const bool xy_ok = (dxy <= xy_tol);
  const bool yaw_ok = (std::fabs(dyaw) <= yaw_tol_deg_);

  // 如果目标Z值大于一个阈值（比如20cm），说明这是一个起飞或空中航点
  // 这种情况下，我们放宽对XY和Yaw的要求，只要高度差不多就认为到达
  if (target.z_cm > 20.0) {
    // 对于起飞阶段，主要关心高度是否到达
    if (current_idx_ == 0) {
        return z_ok;
    }
    // 对于空中的航点，只要高度和水平位置都差不多就行，暂时忽略yaw
    return z_ok && xy_ok;
  }

  // 对于Z值很低（比如降落）或为0的航点，要求所有条件都满足
  return z_ok && xy_ok && yaw_ok;
}

bool RouteTargetPublisherNode::isNearXY(const Target & target, double x_cm, double y_cm) const
{
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dxy = std::hypot(dx, dy);
  return dxy <= visual_takeover_distance_cm_;
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  // {
  //   // ==================== 调试专用：强制悬停测试 ====================
  //   // 1. 定义测试目标点 (0, 0, 100cm, 0 deg)
  //   Target debug_target{0.0, 0.0, 100.0, 0.0};
    
  //   // 2. 持续发布该目标点，确保飞控能收到
  //   publishTarget(debug_target, false);
    
  //   // 3. 打印当前高度，方便你观察 5-8cm 的波动情况
  //   RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
  //     "调试模式：持续发送目标 (0,0,100). 当前高度: %.1f cm", current_height_cm_);

  //   // 4. 直接返回，不执行下面原有的航点切换、视觉接管等逻辑
  //   return;
  // }
  
  // 1. 检查是否已经完成所有目标
  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size()) {
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3; // Drone Stop
    active_controller_pub_->publish(active_msg);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "所有目标已完成，持续发送停止信号(3)");
    return;
  }

  if (current_idx_ == std::numeric_limits<std::size_t>::max()) {
    return;
  }

  // 2. 获取当前位姿
  double x_cm = 0.0, y_cm = 0.0, z_cm = 0.0, yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  const Target & target = targets_[current_idx_];
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "当前目标 %zu: x=%.1f,y=%.1f,z=%.1f,yaw=%.1f",
    current_idx_, target.x_cm, target.y_cm, target.z_cm, target.yaw_deg
  );

  // 3. 判断是否需要视觉对准
  const bool require_visual_align = target.require_visual_align ||
    (enable_visual_align_for_low_z_targets_ && target.z_cm <= visual_align_z_threshold_cm_);

  // 4. 视觉接管逻辑
  if (enable_visual_takeover_ && require_visual_align && isNearXY(target, x_cm, y_cm)) {
    // A. 根据航点配置选择相机
    const uint8_t camera_side = (target.camera_side == 0) ? 1 : target.camera_side;
    const bool use_right = (camera_side == 1);
    const bool use_left = (camera_side == 2);

    bool aligned = false;
    bool has_aligned = false;
    geometry_msgs::msg::Point body_offset_cm{};
    bool has_body_offset = false;

    const char * aligned_topic = use_right ? "/qr_right/aligned" : "/qr_left/aligned";
    const char * offset_topic = use_right ? "/qr_right/fine_offset_body_cm" : "/qr_left/fine_offset_body_cm";

    if (use_right) {
      aligned = qr_right_aligned_;
      has_aligned = has_qr_right_aligned_;
      body_offset_cm = qr_right_fine_offset_body_cm_;
      has_body_offset = has_qr_right_fine_offset_;
    } else if (use_left) {
      aligned = qr_left_aligned_;
      has_aligned = has_qr_left_aligned_;
      body_offset_cm = qr_left_fine_offset_body_cm_;
      has_body_offset = has_qr_left_fine_offset_;
    }

    if (!has_aligned) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "等待对准信号 %s...", aligned_topic);
      return;
    }

    // =============================================================
    // 【修改点】1) 检测到对准：立即推进航点，取消 0.5s 等待
    // =============================================================
    if (aligned) {
      fine_anchor_valid_ = false;   // 立即失效微调基准点
      laser_hold_active_ = false;   // 清除计时标志
      
      RCLCPP_INFO(get_logger(), "QR 对准成功 (%s). 立即切换至下一目标。", aligned_topic);

      current_idx_++;
      if (current_idx_ < targets_.size()) {
        publishCurrent(); // 发布新目标，覆盖任何潜在的微调指令
      } else {
        current_idx_ = targets_.size();
        RCLCPP_INFO(get_logger(), "所有目标已完成");
        std_msgs::msg::UInt8 active_msg;
        active_msg.data = 3; // Drone Stop
        active_controller_pub_->publish(active_msg);
      }
      return; // 关键：推进后立即跳出，防止执行下方的微调发布逻辑
    }

    // =============================================================
    // 【修改点】2) 未对准：发布微调目标点（仅在 aligned 为 false 时执行）
    // =============================================================
    laser_hold_active_ = false; // 确保计时器处于重置状态

    if (!has_body_offset) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "等待偏移量 %s...", offset_topic);
      return;
    }

    // 限制发布频率
    const double hz = std::max(1.0, fine_target_publish_hz_);
    const double min_period = 1.0 / hz;
    const auto now_t = now();
    if (fine_last_publish_time_.nanoseconds() != 0) {
      if ((now_t - fine_last_publish_time_).seconds() < min_period) return;
    }

    // 获取并限幅偏移量
    double body_dx_cm = std::clamp(body_offset_cm.x, -fine_offset_limit_cm_, fine_offset_limit_cm_);
    double body_dy_cm = std::clamp(body_offset_cm.y, -fine_offset_limit_cm_, fine_offset_limit_cm_);
    double body_dz_cm = std::clamp(body_offset_cm.z, -fine_offset_limit_cm_, fine_offset_limit_cm_);

    // 坐标旋转（此处 yaw_rad 默认 0，如需配合航向请取消注释获取真实 yaw）
    const double yaw_rad = 0.0; 
    const double dx_map = std::cos(yaw_rad) * body_dx_cm - std::sin(yaw_rad) * body_dy_cm;
    const double dy_map = std::sin(yaw_rad) * body_dx_cm + std::cos(yaw_rad) * body_dy_cm;

    // 更新基准点 Anchor
    if (!fine_anchor_valid_ || fine_anchor_target_idx_ != current_idx_) {
      fine_anchor_valid_ = true;
      fine_anchor_target_idx_ = current_idx_;
      fine_anchor_x_cm_ = x_cm;
      fine_anchor_y_cm_ = y_cm;
      fine_anchor_z_cm_ = z_cm;
      RCLCPP_INFO(get_logger(), "设置微调基准点: (%.1f, %.1f, %.1f)", x_cm, y_cm, z_cm);
    }

    Target fine_target = target;
    fine_target.x_cm = fine_anchor_x_cm_ + dx_map;
    fine_target.y_cm = fine_anchor_y_cm_ + dy_map;
    fine_target.z_cm = fine_anchor_z_cm_ + body_dz_cm;

    publishTarget(fine_target, false);
    fine_last_publish_time_ = now_t;
    return; // 视觉接管期间不执行常规距离判断
  }

  // 5. 常规航点判定（非视觉接管区）
  if (isReached(target, x_cm, y_cm, z_cm, yaw_deg)) {
    RCLCPP_INFO(get_logger(), "常规目标 %zu 已完成", current_idx_);
    current_idx_++;
    if (current_idx_ < targets_.size()) {
      publishCurrent();
    } else {
      current_idx_ = targets_.size();
      std_msgs::msg::UInt8 active_msg;
      active_msg.data = 3; 
      active_controller_pub_->publish(active_msg);
    }
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

// RouteTestNode::RouteTestNode(
//   const std::shared_ptr<RouteTargetPublisherNode> & route_node,
//   const rclcpp::NodeOptions & options)
// : rclcpp::Node("route_test_node", options),
//   route_node_(route_node),
//   started_(false),
//   next_target_index_(1)
// {
//   std::setlocale(LC_ALL, "");

//   ready_sub_ = create_subscription<std_msgs::msg::UInt8>(
//     "/is_st_ready", rclcpp::QoS(10),
//     std::bind(&RouteTestNode::readyCallback, this, std::placeholders::_1));

//   add_timer_ = create_wall_timer(
//     std::chrono::seconds(1),
//     std::bind(&RouteTestNode::addTimerCallback, this));
//   add_timer_->cancel();

//   RCLCPP_INFO(get_logger(), "Route test node ready. 等待 /is_st_ready == 1");
// }

// void RouteTestNode::readyCallback(const std_msgs::msg::UInt8::SharedPtr msg)
// {
//   if (msg->data == 1 && !started_) {
//     Target first{0.0, 0.0, 140, 0.0};
//     route_node_->addTarget(first);
//     const auto current = route_node_->currentIndex();
//     RCLCPP_INFO(get_logger(),
//       "收到 /is_st_ready=1，添加首个目标: x=%.1f y=%.1f z=%.1f yaw=%.1f | 当前第 %zu 个目标",
//       first.x_cm, first.y_cm, first.z_cm, first.yaw_deg,
//       (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));
//     add_timer_->reset();
//     started_ = true;
//   } else if (!started_) {
//     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
//       "/is_st_ready=%u，等待为1", static_cast<unsigned>(msg->data));
//   }
// }

RouteTestNode::RouteTestNode(
  const std::shared_ptr<RouteTargetPublisherNode> & route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("route_test_node", options),
  route_node_(route_node),
  started_(false),
  next_target_index_(1)
{
  std::setlocale(LC_ALL, "");

  // bluetooth_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
  //     "/bluetooth_data", 10,
  //     std::bind(&RouteTestNode::bluetoothCallback, this, std::placeholders::_1));

  // 创建定时器，但先不启动
  add_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&RouteTestNode::addTimerCallback, this));
  add_timer_->cancel();

  RCLCPP_INFO(get_logger(), "Route test node 启动，自动添加第一个航点...");
  
  Target first{0.0, 0.0, 130.0, 0.0};
  route_node_->addTarget(first);
  
  const auto current = route_node_->currentIndex();
  RCLCPP_INFO(get_logger(),
    "添加首个目标: x=%.1f y=%.1f z=%.1f yaw=%.1f | 当前第 %zu 个目标",
    first.x_cm, first.y_cm, first.z_cm, first.yaw_deg,
    (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));

  add_timer_->reset();
  started_ = true;
}

// void RouteTestNode::bluetoothCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
// {
//   if (started_) {
//     return;
//   }

//   if (!msg->data.empty() && msg->data[0] == 1) {
//     RCLCPP_INFO(get_logger(), "收到蓝牙指令 1，开始执行航点任务！");
    
//     Target first{200.0, 0.0, 4.0, 0.0};
//     route_node_->addTarget(first);
    
//     const auto current = route_node_->currentIndex();
//     RCLCPP_INFO(get_logger(),
//       "添加首个目标: x=%.1f y=%.1f z=%.1f yaw=%.1f | 当前第 %zu 个目标",
//       first.x_cm, first.y_cm, first.z_cm, first.yaw_deg,
//       (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));

//     add_timer_->reset();
//     started_ = true;
//   }
// }




void RouteTestNode::addTimerCallback()
{
  if (!started_) {
    return;
  }

  Target target{};
  switch (next_target_index_) {
    case 1:
      target = Target{100.0, 0.0, 130.0, 0.0};
      break;
    case 2:
      target = Target{100.0, 50.0, 130.0, 0.0};
      break;
    case 3:
      target = Target{100.0, 50.0, 40.0, 0.0};
      break;   
    case 4:
      target = Target{100.0, 50.0, 130.0, 0.0};
      break;  
    case 5:
      target = Target{100.0, 0.0, 130.0, 0.0};
      break;  
    case 6:
      target = Target{0.0, 0.0, 130.0, 0.0};
      break;  
    case 7:
      target = Target{0.0, 0.0, 0.0, 0.0};
      break;
    // case 2:
    //   target = Target{120.0, 0.0, 130.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 1;
    //   break;
    // case 3:
    //   target = Target{170.0, 0.0, 130.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 1;
    //   break;
    // case 4:
    //   target  = Target{230.0, 0.0, 130.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 1;
    //   break;
    // case 5:
    //   target = Target{240.0, 0.0, 75.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 1;
    //   break;
    // case 6:
    //   target = Target{170.0, 0.0, 83.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 1;
    //   break;
    // case 7:
    //   target = Target{120.0, 0.0, 83.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 1;
    //   break;
    // case 8:
    //   target = Target{0.0, 0.0, 83.0, 0.0};
    //   target.require_visual_align = false;
    //   break;
    // case 9:
    //   target = Target{0.0, -96.0, 87.0, 0.0};
    //   target.require_visual_align = false;
    //   break;
    // case 10:
    //   target = Target{170.0, -96.0, 87.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 2;
    //   break;
    // case 11:
    //   target = Target{235.0, -96.0, 87.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 2;
    //   break;
    // case 12:
    //   target = Target{240.0, -96.0, 130.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 2;
    //   break;
    // case 13:
    //   target = Target{170.0, -96.0, 130.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 2;
    //   break;
    // case 14:
    //   target = Target{120.0, -96.0, 130.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 2;
    //   break;
    // case 15:
    //   target = Target{0.0, -96.0, 130.0, 0.0};
    //   target.require_visual_align = false;
    //   break;
    // case 16:
    //   target = Target{0.0, -96.0, 4.0, 0.0};
    //   // target = Target{0.0, -190.0, 130.0, 0.0};
    //   target.require_visual_align = false;
    //   break; 
    // case 18:
    //   target = Target{120.0, -190.0, 130.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 1;
    //   break;
    // case 19:
    //   target = Target{170.0, -190.0, 130.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 1;
    //   break;
    // case 20:
    //   target = Target{235.0, -190.0, 130.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 1;
    //   break;
    // case 21:
    //   target = Target{240.0, -190.0, 75.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 1;
    //   break;
    // case 22:
    //   target = Target{170.0, -190.0, 75.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 1;
    //   break;
    // case 23:
    //   target = Target{120.0, -190.0, 75.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 1;
    //   break;
    // case 24:
    //   target = Target{0.0, -190.0, 75.0, 0.0};
    //   target.require_visual_align = false;
    //   break;
    // case 25:
    //   target = Target{0.0, -270.0, 75.0, 0.0};
    //   target.require_visual_align = false;
    //   break;
    // case 26:
    //   target = Target{120.0, -270.0, 75.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 2;
    //   break;
    // case 27:
    //   target = Target{170.0, -270.0, 75.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 2;
    //   break;
    // case 28:
    //   target = Target{235.0, -270.0, 75.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 2;
    //   break;
    // case 29:
    //   target = Target{240.0, -270.0, 130.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 2;
    //   break;
    // case 30:
    //   target = Target{170.0, -270.0, 130.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 2;
    //   break;
    // case 31:
    //   target = Target{120.0, -270.0, 130.0, 0.0};
    //   target.require_visual_align = true;
    //   target.camera_side = 2;
    //   break;
    // case 32:
    //   target = Target{120.0, -270.0, 4.0, 0.0};
    //   target.require_visual_align = false;
    //   break;
    default:
      add_timer_->cancel();
      RCLCPP_INFO(get_logger(), "预设目标全部添加完毕");
      return;
  }

  route_node_->addTarget(target);
  const auto current = route_node_->currentIndex();
  RCLCPP_INFO(get_logger(),
    "追加目标 idx=%d: x=%.1f y=%.1f z=%.1f yaw=%.1f | 当前第 %zu 个目标",
    next_target_index_, target.x_cm, target.y_cm, target.z_cm, target.yaw_deg,
    (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));

  ++next_target_index_;
}

}  // namespace activity_control_pkg














