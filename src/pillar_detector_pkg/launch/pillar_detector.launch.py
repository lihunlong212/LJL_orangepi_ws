from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pillar_detector_pkg',
            executable='pillar_detector_node',
            name='pillar_detector',
            output='screen',
            parameters=[{
                # ── 话题 ────────────────────────────────────
                'scan_topic': '/scan',

                # ── 柱子有效区域（单位：米） ──────────────────
                # 题目：柱子中心距场地边界 ≥50cm，场地 3×3m
                # 无人机在左下角(0,0)，向前+x，向右-y
                'map_x_min_m': 0.5,   # 前方最近
                'map_x_max_m': 2.5,   # 前方最远
                'map_y_min_m': -2.5,  # 右侧最远（负数）
                'map_y_max_m': -0.5,  # 右侧最近（负数）

                # ── 单帧分组参数 ──────────────────────────────
                # 同组内相邻点最大距离：柱子点间距≤15mm，不同柱子间距≥80cm，25cm足够区分
                'group_dist_m': 0.25,
                # 一组内最少点数才认为是柱子（题目说≥4个）
                'min_pts_per_group': 4,
                # 同帧内两个柱子中心最小距离（柱子间距≥80cm，取一半40cm作安全门槛）
                'min_pillar_separation_m': 0.40,

                # ── 多帧累积参数 ──────────────────────────────
                # 雷达 15Hz，累积 20 帧约 1.3 秒
                'accumulation_frames': 20,
                # 两次检测在 20cm 内算同一个柱子
                'cluster_merge_dist_m': 0.20,
                # 至少出现 8 帧才确认为柱子（20帧中60%）
                'min_votes': 8,
                # 最多柱子数量（地图内固定4个）
                'max_pillars': 4,
            }],
        )
    ])
