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
  // 参数在 launch 文件中配置，这里只声明默认值
  pillar_width_m_       = declare_parameter("pillar_width_m",       0.15);   // 柱子宽度 15 cm
  edge_threshold_m_     = declare_parameter("edge_threshold_m",     0.08);   // 边沿突变阈值 8 cm
  max_cluster_pts_      = declare_parameter("max_cluster_pts",       120);   // 簇最多点数
  stability_range_m_    = declare_parameter("stability_range_m",     0.20);  // 柱面平整度容差 20 cm
  map_x_min_m_          = declare_parameter("map_x_min_m",           0.5);   // 柱子距前方起点最近 50 cm
  map_x_max_m_          = declare_parameter("map_x_max_m",           2.5);   // 柱子距前方边界最近 50 cm
  map_y_min_m_          = declare_parameter("map_y_min_m",          -2.5);   // 柱子距右侧边界最近 50 cm
  map_y_max_m_          = declare_parameter("map_y_max_m",          -0.5);   // 柱子距右侧起点最近 50 cm
  accumulation_frames_  = declare_parameter("accumulation_frames",   20);    // 累积帧数（约 1.3 秒）
  cluster_merge_dist_m_ = declare_parameter("cluster_merge_dist_m",  0.20);  // 聚类合并距离 20 cm
  min_votes_            = declare_parameter("min_votes",              12);    // 最少投票帧数（20帧中至少12帧）
  max_pillars_          = declare_parameter("max_pillars",            4);     // 最多柱子数量

  const std::string scan_topic = declare_parameter("scan_topic", std::string("/scan"));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&PillarDetectorNode::scanCallback, this, std::placeholders::_1));

  pillars_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/detected_pillars",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  RCLCPP_INFO(get_logger(),
    "柱子检测节点启动，监听 '%s'，累积 %d 帧后输出结果",
    scan_topic.c_str(), accumulation_frames_);
  RCLCPP_INFO(get_logger(),
    "柱子有效范围: x=[%.1f, %.1f]  y=[%.1f, %.1f]  最多检测 %d 个柱子",
    map_x_min_m_, map_x_max_m_, map_y_min_m_, map_y_max_m_, max_pillars_);
}

// ─────────────────────────────────────────────────────────────────────────────
// 预计算：对扫描段内每个索引，算出该角度方向上地图正方形的最大允许距离
// 公式：r_max(θ) = min( x_max/cos(θ),  |y_min|/|sin(θ)| )
// 几何意义：
//   -90°→-45°  射线先碰右侧边(y=-3)，r_max = 3/|sinθ|，从 3m 增大到 4.24m
//   -45°       射线打到右上角(3,-3)，r_max = 3√2 ≈ 4.24m（最大值）
//   -45°→ 0°  射线先碰前方边(x=3)，r_max = 3/cosθ，从 4.24m 减小到 3m
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

    // 前方外边界 x ≤ map_x_max：r·cos(θ) ≤ x_max → r ≤ x_max/cos(θ)
    if (cos_t > 1e-9) {
      r_max = std::min(r_max, map_x_max_m_ / cos_t);
    }

    // 右侧外边界 y ≥ map_y_min（负数）：r·sin(θ) ≥ y_min → r ≤ |y_min|/|sin(θ)|
    if (sin_t < -1e-9) {
      r_max = std::min(r_max, (-map_y_min_m_) / (-sin_t));
    }

    max_range_per_idx_[i] = r_max;
  }

  ranges_precomputed_ = true;
  RCLCPP_INFO(get_logger(),
    "地图边界距离表预计算完成（共 %d 个角度）", n);
}

// ─────────────────────────────────────────────────────────────────────────────
// 单帧检测
// ─────────────────────────────────────────────────────────────────────────────
std::vector<Detection> PillarDetectorNode::detectInFrame(
  const sensor_msgs::msg::LaserScan & scan)
{
  std::vector<Detection> detections;
  const int n = static_cast<int>(scan.ranges.size());
  if (n < 8) { return detections; }

  // 右前方象限：索引 25%~50%，对应角度 -π/2 ~ 0
  const int seg_start = n / 4;
  const int seg_end   = n / 2;

  // 角分辨率（弧度），用于自适应计算每个距离上柱子预期点数
  // 0.3° = 0.005236 rad，点数 ≈ 柱宽 / (距离 × 角分辨率)
  const double ang_res = static_cast<double>(scan.angle_increment);

  auto angle_at = [&](int i) -> double {
    return static_cast<double>(scan.angle_min) +
           static_cast<double>(i) * ang_res;
  };

  auto is_valid = [&](int i) -> bool {
    const float r = scan.ranges[i];
    return std::isfinite(r) && r >= scan.range_min && r <= scan.range_max;
  };

  int i = seg_start;
  while (i < seg_end - 1) {
    if (!is_valid(i))     { ++i; continue; }
    if (!is_valid(i + 1)) { i += 2; continue; }

    // ── 第一层过滤：按角度判断该点是否在地图范围内 ──
    // 若测量距离 > 该角度方向到地图边界的最大距离，此点在地图外，直接跳过
    if (static_cast<double>(scan.ranges[i]) > max_range_per_idx_[i]) {
      ++i;
      continue;
    }

    // 进入边沿：ranges[i] 比 ranges[i+1] 突然大（距离变近 = 遇到柱子前面）
    if (static_cast<double>(scan.ranges[i]) - scan.ranges[i + 1] < edge_threshold_m_) {
      ++i;
      continue;
    }

    // ── 收集簇（柱子可见面） ──
    const int cluster_start = i + 1;

    // 自适应最少点数：根据进入边沿的距离计算该距离上柱子预期点数
    // 预期点数 = 柱宽 / (距离 × 角分辨率)，取 2/3 作为最低要求（留余量）
    const double r_entry = static_cast<double>(scan.ranges[cluster_start]);
    const int expected_pts = static_cast<int>(pillar_width_m_ / (r_entry * ang_res));
    const int adaptive_min_pts = std::max(3, expected_pts * 2 / 3);

    int j = cluster_start;
    float r_min = scan.ranges[j];
    float r_max_val = scan.ranges[j];

    while (j < seg_end - 1 && (j - cluster_start) < max_cluster_pts_) {
      if (!is_valid(j + 1)) { break; }
      // 退出边沿：ranges[j+1] 比 ranges[j] 突然大（距离变远 = 离开柱子）
      if (static_cast<double>(scan.ranges[j + 1]) - scan.ranges[j] >= edge_threshold_m_) {
        break;
      }
      ++j;
      r_min = std::min(r_min, scan.ranges[j]);
      r_max_val = std::max(r_max_val, scan.ranges[j]);
    }

    const int cluster_end  = j;
    const int cluster_size = cluster_end - cluster_start + 1;

    const bool has_trailing =
      (j < seg_end - 1) && is_valid(j + 1) &&
      (static_cast<double>(scan.ranges[j + 1]) - scan.ranges[j] >= edge_threshold_m_);

    if (has_trailing &&
        cluster_size >= adaptive_min_pts &&
        cluster_size <= max_cluster_pts_ &&
        static_cast<double>(r_max_val - r_min) <= stability_range_m_)
    {
      // 计算柱面中心坐标
      double sum_r = 0.0;
      for (int k = cluster_start; k <= cluster_end; ++k) {
        sum_r += scan.ranges[k];
      }
      const double r_mean    = sum_r / cluster_size;
      const double theta_mid = (angle_at(cluster_start) + angle_at(cluster_end)) * 0.5;
      const double x = r_mean * std::cos(theta_mid);
      const double y = r_mean * std::sin(theta_mid);

      // ── 第二层确认：(x,y) 落点是否在柱子有效区域内 ──
      // 题目规定柱子中心距边界 ≥50cm，所以有效范围是 [0.5,2.5] × [-2.5,-0.5]
      if (x >= map_x_min_m_ && x <= map_x_max_m_ &&
          y >= map_y_min_m_ && y <= map_y_max_m_)
      {
        detections.push_back({x, y});
        RCLCPP_DEBUG(get_logger(),
          "  候选: x=%.3f y=%.3f  r=%.3f  θ=%.1f°  pts=%d(预期%d)  Δr=%.3f",
          x, y, r_mean, theta_mid * 180.0 / M_PI,
          cluster_size, expected_pts,
          static_cast<double>(r_max_val - r_min));
      }

      i = cluster_end + 2;
    } else {
      i = has_trailing ? (cluster_end + 2) : (cluster_end + 1);
    }
  }

  return detections;
}

// ─────────────────────────────────────────────────────────────────────────────
// 聚类：将多帧累积候选点合并，按票数排序，最多保留 max_pillars_ 个
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

    // BFS：把距当前簇内任意点 < merge_dist 的点归入同簇
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

  // 计算每个簇的质心和票数
  std::vector<Cluster> clusters;
  for (int lbl = 0; lbl < next_label; ++lbl) {
    double sum_x = 0.0, sum_y = 0.0;
    int count = 0;
    for (int i = 0; i < nd; ++i) {
      if (labels[i] == lbl) {
        sum_x += dets[i].x_m;
        sum_y += dets[i].y_m;
        ++count;
      }
    }
    if (count >= min_votes_) {
      clusters.push_back({sum_x / count, sum_y / count, count});
    }
  }

  // 按票数降序排列，最多保留 max_pillars_ 个
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
    RCLCPP_INFO(get_logger(), "║    置信度: %d/%d 帧检测到               ║",
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

  // 第一帧时预计算地图边界距离表
  if (!ranges_precomputed_) {
    precomputeMaxRanges(*msg);
  }

  const auto frame_dets = detectInFrame(*msg);
  ++frame_count_;

  for (const auto & d : frame_dets) {
    accumulated_.push_back(d);
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "帧 %d/%d  本帧候选=%zu  累积=%zu",
    frame_count_, accumulation_frames_,
    frame_dets.size(), accumulated_.size());

  if (frame_count_ >= accumulation_frames_) {
    done_ = true;
    RCLCPP_INFO(get_logger(),
      "累积完成（%d 帧，%zu 个候选点），开始聚类...",
      frame_count_, accumulated_.size());
    const auto pillars = clusterDetections(accumulated_);
    publishPillars(pillars);
  }
}

}  // namespace pillar_detector_pkg
