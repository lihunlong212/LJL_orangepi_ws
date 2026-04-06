from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # 类型检查时使用，不参与覆盖率统计
    LaunchDescription = Any
    Node = Any


def generate_launch_description():
    launch_module = import_module("launch")
    launch_ros_actions = import_module("launch_ros.actions")
    LaunchDescription = getattr(launch_module, "LaunchDescription")
    Node = getattr(launch_ros_actions, "Node")

    route_params = {
        # 坐标系与目标发布参数
        "map_frame": "map",                         # 全局目标坐标系
        "laser_link_frame": "laser_link",           # 用于到达判定的机体位姿坐标系
        "output_topic": "/target_position",         # 发送给 PID 的目标话题

        # 到达判定阈值
        "position_tolerance_cm": 6.0,               # XY 位置容差，单位厘米
        "yaw_tolerance_deg": 5.0,                   # 偏航角容差，单位度
        "height_tolerance_cm": 6.0,                 # 高度容差，单位厘米

        # 视觉接管判定参数
        "visual_align_pixel_threshold": 100.0,      # 对准半径阈值，单位像素
        "visual_align_required_frames": 3,          # 连续满足对准条件所需帧数
        "visual_takeover_timeout_sec": 5.0,         # 视觉接管最长持续时间，单位秒
        "fine_data_stale_timeout_sec": 0.5,         # fine_data 数据超时时间，单位秒
    }

    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="route_target_publisher_node",
            name="route_target_publisher",
            output="screen",
            parameters=[route_params],
        )
    ])
