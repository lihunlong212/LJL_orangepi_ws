#include "pillar_detector_pkg/pillar_detector_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pillar_detector_pkg
{

PillarDetectorNode::PillarDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pillar_detector", options),
  frame_count_(0),
  done_(false),
  ranges_precomputed_(false)
{
  // ── 地图/柱子有效区域 ──────────────────────────────────────
  map_x_min_m_          = declare_parameter("map_x_min_m",           0.5);
  map_x_max_m_          = declare_parameter("map_x_max_m",           2.5);
  map_y_min_m_          = declare_parameter("map_y_min_m",          -2.5);
  map_y_max_m_          = declare_parameter("map_y_max_m",          -0.5);

  // ── 单帧分组参数 ───────────────────────────────────────────
  // 同一组内相邻点最大距离：柱子宽15cm，点间距最大~15mm(2.5m处)，25cm足够分开不同柱子
  group_dist_m_          = declare_parameter("group_dist_m",          0.25);
  // 一个组至少多少个点才认为是柱子
  min_pts_per_group_     = declare_parameter("min_pts_per_group",     4);
  // 同帧内两个柱子中心最小距离（柱子间距≥80cm，用40cm作安全门槛）
  min_pillar_separation_m_ = declare_parameter("min_pillar_separation_m", 0.40);

  // ── 多帧累积参数 ───────────────────────────────────────────
  accumulation_frames_   = declare_parameter("accumulation_frames",   20);
  cluster_merge_dist_m_  = declare_parameter("cluster_merge_dist_m",  0.20);
  min_votes_             = declare_parameter("min_votes",              8);
  max_pillars_           = declare_parameter("max_pillars",            4);

  const std::string scan_topic = declare_parameter("scan_topic", std::string("/scan"));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&PillarDetectorNode::scanCallback, this, std::placeholders::_1));

  pillars_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/detected_pillars",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  RCLCPP_INFO(get_logger(),
    "柱子检测节点启动，监听 '%s'，累积 %d 帧后输出结果", scan_topic.c_str(), accumulation_frames_);
  RCLCPP_INFO(get_logger(),
    "柱子有效区域: x=[%.1f, %.1f]  y=[%.1f, %.1f]  最多 %d 个柱子",
    map_x_min_m_, map_x_max_m_, map_y_min_m_, map_y_max_m_, max_pillars_);
}

// ─────────────────────────────────────────────────────────────────────────────
// 预计算每个角度到地图边界的最大允许距离
// r_max(θ) = min( x_max/cos(θ),  |y_min|/|sin(θ)| )
// -90°→-45°: r_max 从 2.5m 增大到 3.54m（受右侧边界限制）
// -45°→  0°: r_max 从 3.54m 减小到 2.5m（受前方边界限制）
// ─────────────────────────────────────────────────────────────────────────────
void PillarDetectorNode::precomputeMaxRanges(const sensor_msgs::msg::LaserScan & scan)
{
  const int n = static_cast<int>(scan.ranges.size());
  max_range_per_idx_.resize(n, std::numeric_limits<double>::infinity());

  for (int i = 0; i < n; ++i) {
    const double theta = static_cast<double>(scan.angle_min) +
                         static_cast<double>(i) * static_cast<double>(scan.angle_increment);
    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);
    double r_max = std::numeric_limits<double>::infinity();

    if (cos_t > 1e-9)  { r_max = std::min(r_max, map_x_max_m_ / cos_t); }
    if (sin_t < -1e-9) { r_max = std::min(r_max, (-map_y_min_m_) / (-sin_t)); }

    max_range_per_idx_[i] = r_max;
  }

  ranges_precomputed_ = true;
  RCLCPP_INFO(get_logger(), "地图边界距离表预计算完成（共 %d 个角度）", n);
}

// ─────────────────────────────────────────────────────────────────────────────
// 单帧检测：逻辑大幅简化
// 核心思路：地图范围内只有柱子，相邻的一组点就是一个柱子面
// ─────────────────────────────────────────────────────────────────────────────
std::vector<Detection> PillarDetectorNode::detectInFrame(
  const sensor_msgs::msg::LaserScan & scan)
{
  std::vector<Detection> detections;
  const int n = static_cast<int>(scan.ranges.size());
  if (n < 8) { return detections; }

  // 右前方象限：索引 25%~50%，角度 -π/2 ~ 0
  const int seg_start = n / 4;
  const int seg_end   = n / 2;

  const double ang_res = static_cast<double>(scan.angle_increment);
  auto angle_at = [&](int i) -> double {
    return static_cast<double>(scan.angle_min) + static_cast<double>(i) * ang_res;
  };
  auto is_valid = [&](int i) -> bool {
    const float r = scan.ranges[i];
    return std::isfinite(r) && r >= scan.range_min && r <= scan.range_max;
  };

  // ── Step 1：收集所有落在柱子有效区域内的点 ──────────────────
  struct Pt { double x, y; };
  std::vector<Pt> in_bounds;

  for (int i = seg_start; i < seg_end; ++i) {
    if (!is_valid(i)) { continue; }
    // 第一层：角度方向超出地图外边界，跳过
    if (static_cast<double>(scan.ranges[i]) > max_range_per_idx_[i]) { continue; }

    const double theta = angle_at(i);
    const double r = static_cast<double>(scan.ranges[i]);
    const double x = r * std::cos(theta);
    const double y = r * std::sin(theta);

    // 第二层：(x,y) 必须在柱子有效区域内（地图内只有柱子）
    if (x >= map_x_min_m_ && x <= map_x_max_m_ &&
        y >= map_y_min_m_ && y <= map_y_max_m_)
    {
      in_bounds.push_back({x, y});
    }
  }

  if (in_bounds.empty()) { return detections; }

  // 第一帧打印诊断信息
  if (frame_count_ == 0) {
    RCLCPP_INFO(get_logger(),
      "诊断: 区段有效点=%zu 个落入地图范围内", in_bounds.size());
  }

  // ── Step 2：按距离分组（相邻点 < group_dist 归为同组） ────────
  // 同一柱子上相邻点间距 ≤ 15mm（2.5m处0.3°），不同柱子间距 ≥ 80cm
  // group_dist=25cm 足以区分
  std::vector<std::vector<Pt>> groups;
  std::vector<Pt> cur_group = {in_bounds[0]};

  for (std::size_t k = 1; k < in_bounds.size(); ++k) {
    const Pt & prev = cur_group.back();
    const Pt & curr = in_bounds[k];
    const double dx = curr.x - prev.x;
    const double dy = curr.y - prev.y;
    if (std::sqrt(dx * dx + dy * dy) <= group_dist_m_) {
      cur_group.push_back(curr);
    } else {
      if (static_cast<int>(cur_group.size()) >= min_pts_per_group_) {
        groups.push_back(cur_group);
      }
      cur_group = {curr};
    }
  }
  if (static_cast<int>(cur_group.size()) >= min_pts_per_group_) {
    groups.push_back(cur_group);
  }

  // ── Step 3：计算每组中心，确保两柱子中心不能太近 ─────────────
  for (const auto & group : groups) {
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto & p : group) { sum_x += p.x; sum_y += p.y; }
    const double cx = sum_x / static_cast<double>(group.size());
    const double cy = sum_y / static_cast<double>(group.size());

    // 与已有候选距离检查：防止同一柱子被重复计入
    bool too_close = false;
    for (const auto & d : detections) {
      const double dx = cx - d.x_m;
      const double dy = cy - d.y_m;
      if (std::sqrt(dx * dx + dy * dy) < min_pillar_separation_m_) {
        too_close = true;
        break;
      }
    }
    if (!too_close) {
      detections.push_back({cx, cy});
    }
  }

  return detections;
}

// ─────────────────────────────────────────────────────────────────────────────
// 多帧聚类
// ─────────────────────────────────────────────────────────────────────────────
std::vector<Cluster> PillarDetectorNode::clusterDetections(
  const std::vector<Detection> & dets) const
{
  const int nd = static_cast<int>(dets.size());
  std::vector<int> labels(nd, -1);
  int next_label = 0;

  for (int i = 0; i < nd; ++i) {
    if (labels[i] >= 0) { continue; }
    labels[i] = next_label;
    bool changed = true;
    while (changed) {
      changed = false;
      for (int j = 0; j < nd; ++j) {
        if (labels[j] >= 0) { continue; }
        for (int k = 0; k < nd; ++k) {
          if (labels[k] != next_label) { continue; }
          const double dx = dets[j].x_m - dets[k].x_m;
          const double dy = dets[j].y_m - dets[k].y_m;
          if (std::sqrt(dx * dx + dy * dy) < cluster_merge_dist_m_) {
            labels[j] = next_label;
            changed = true;
            break;
          }
        }
      }
    }
    ++next_label;
  }

  std::vector<Cluster> clusters;
  for (int lbl = 0; lbl < next_label; ++lbl) {
    double sum_x = 0.0, sum_y = 0.0;
    int count = 0;
    for (int i = 0; i < nd; ++i) {
      if (labels[i] == lbl) { sum_x += dets[i].x_m; sum_y += dets[i].y_m; ++count; }
    }
    if (count >= min_votes_) {
      clusters.push_back({sum_x / count, sum_y / count, count});
    }
  }

  std::sort(clusters.begin(), clusters.end(),
    [](const Cluster & a, const Cluster & b) { return a.votes > b.votes; });
  if (static_cast<int>(clusters.size()) > max_pillars_) {
    clusters.resize(max_pillars_);
  }
  return clusters;
}

// ─────────────────────────────────────────────────────────────────────────────
// 发布结果
// ─────────────────────────────────────────────────────────────────────────────
void PillarDetectorNode::publishPillars(const std::vector<Cluster> & pillars)
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(pillars.size() * 2);
  for (std::size_t k = 0; k < pillars.size(); ++k) {
    msg.data[k * 2]     = static_cast<float>(pillars[k].x_m);
    msg.data[k * 2 + 1] = static_cast<float>(pillars[k].y_m);
  }
  pillars_pub_->publish(msg);

  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "╔══════════════════════════════════════════╗");
  RCLCPP_INFO(get_logger(), "║         柱子检测结果（共 %zu 个）          ║", pillars.size());
  RCLCPP_INFO(get_logger(), "╠══════════════════════════════════════════╣");
  for (std::size_t k = 0; k < pillars.size(); ++k) {
    RCLCPP_INFO(get_logger(), "║  第 %zu 个柱子:                            ║", k + 1);
    RCLCPP_INFO(get_logger(), "║    x = %+.3f m  ( 前方 %.1f cm )       ║",
      pillars[k].x_m, pillars[k].x_m * 100.0);
    RCLCPP_INFO(get_logger(), "║    y = %+.3f m  ( 右方 %.1f cm )       ║",
      pillars[k].y_m, -pillars[k].y_m * 100.0);
    RCLCPP_INFO(get_logger(), "║    检测次数: %d / %d 帧                 ║",
      pillars[k].votes, accumulation_frames_);
    if (k + 1 < pillars.size()) {
      RCLCPP_INFO(get_logger(), "╠══════════════════════════════════════════╣");
    }
  }
  RCLCPP_INFO(get_logger(), "╠══════════════════════════════════════════╣");
  RCLCPP_INFO(get_logger(), "║  已发布到 /detected_pillars              ║");
  RCLCPP_INFO(get_logger(), "║  格式: [x1,y1, x2,y2, ...]  单位: 米    ║");
  RCLCPP_INFO(get_logger(), "╚══════════════════════════════════════════╝");
  RCLCPP_INFO(get_logger(), " ");
}

// ─────────────────────────────────────────────────────────────────────────────
// 扫描回调
// ─────────────────────────────────────────────────────────────────────────────
void PillarDetectorNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (done_) { return; }

  if (!ranges_precomputed_) { precomputeMaxRanges(*msg); }

  const auto frame_dets = detectInFrame(*msg);
  ++frame_count_;

  for (const auto & d : frame_dets) { accumulated_.push_back(d); }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "帧 %d/%d  本帧候选=%zu  累积=%zu",
    frame_count_, accumulation_frames_, frame_dets.size(), accumulated_.size());

  if (frame_count_ >= accumulation_frames_) {
    done_ = true;
    RCLCPP_INFO(get_logger(),
      "累积完成（%d 帧，%zu 个候选点），开始聚类...",
      frame_count_, accumulated_.size());
    publishPillars(clusterDetections(accumulated_));
  }
}

}  // namespace pillar_detector_pkg
