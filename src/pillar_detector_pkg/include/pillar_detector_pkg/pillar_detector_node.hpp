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
  // 每帧检测：收集地图范围内的点 → 分组 → 输出候选柱子坐标
  std::vector<Detection> detectInFrame(const sensor_msgs::msg::LaserScan & scan);

  // 预计算每个角度对应的地图最大允许距离（只算一次）
  void precomputeMaxRanges(const sensor_msgs::msg::LaserScan & scan);

  // 多帧聚类：按票数降序，最多保留 max_pillars_ 个
  std::vector<Cluster> clusterDetections(const std::vector<Detection> & dets) const;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishPillars(const std::vector<Cluster> & pillars);

  // ── 参数 ──────────────────────────────────────────────────
  // 地图/柱子有效区域
  double map_x_min_m_;
  double map_x_max_m_;
  double map_y_min_m_;
  double map_y_max_m_;

  // 单帧分组
  int    min_pts_per_group_;      // 一个组最少点数才算柱子（≥4）
  double group_dist_m_;           // 同组内相邻点最大距离
  double min_pillar_separation_m_;// 同帧内两个柱子中心最小距离

  // 多帧累积
  int    accumulation_frames_;
  double cluster_merge_dist_m_;
  int    min_votes_;
  int    max_pillars_;

  // ── 状态 ──────────────────────────────────────────────────
  int  frame_count_;
  bool done_;
  bool ranges_precomputed_;
  std::vector<double>    max_range_per_idx_;
  std::vector<Detection> accumulated_;
  mutable std::mutex mutex_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pillars_pub_;
};

}  // namespace pillar_detector_pkg
