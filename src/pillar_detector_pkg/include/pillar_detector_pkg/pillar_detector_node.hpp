#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace pillar_detector_pkg
{

struct Detection
{
  double x_m;
  double y_m;
};

struct Cluster
{
  double x_m;
  double y_m;
  int    votes;
};

class PillarDetectorNode : public rclcpp::Node
{
public:
  explicit PillarDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 每帧检测柱子候选
  std::vector<Detection> detectInFrame(const sensor_msgs::msg::LaserScan & scan);

  // 预计算每个角度对应的地图最大允许距离（只算一次）
  void precomputeMaxRanges(const sensor_msgs::msg::LaserScan & scan);

  // 对累积数据聚类，返回按票数降序、最多 max_pillars_ 个柱子
  std::vector<Cluster> clusterDetections(const std::vector<Detection> & dets) const;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishPillars(const std::vector<Cluster> & pillars);

  // ── 参数 ──────────────────────────────────────────────────
  double pillar_width_m_;        // 柱子宽度（15 cm）
  double edge_threshold_m_;      // 边沿突变阈值
  int    max_cluster_pts_;       // 簇最多点数上限（防误判大物体）
  double stability_range_m_;     // 簇内 max-min 距离容差
  double map_x_min_m_;           // 柱子 x 最小（距前方起点至少 50cm）
  double map_x_max_m_;           // 柱子 x 最大（距前方边界至少 50cm）
  double map_y_min_m_;           // 柱子 y 最小（距右侧边界至少 50cm，负数）
  double map_y_max_m_;           // 柱子 y 最大（距右侧起点至少 50cm，负数）
  int    accumulation_frames_;   // 累积帧数
  double cluster_merge_dist_m_;  // 聚类合并距离
  int    min_votes_;             // 聚类最少帧数投票
  int    max_pillars_;           // 最多柱子数量（≤4）

  // ── 状态 ──────────────────────────────────────────────────
  int  frame_count_;
  bool done_;
  bool ranges_precomputed_;
  std::vector<double>    max_range_per_idx_;  // 每个索引的地图边界最大距离
  std::vector<Detection> accumulated_;
  mutable std::mutex mutex_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pillars_pub_;
};

}  // namespace pillar_detector_pkg
