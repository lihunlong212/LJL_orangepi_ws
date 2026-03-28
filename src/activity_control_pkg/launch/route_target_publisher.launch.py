from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # pragma: no cover
    LaunchDescription = Any
    Node = Any


def generate_launch_description():
    launch_module = import_module("launch")
    launch_ros_actions = import_module("launch_ros.actions")
    LaunchDescription = getattr(launch_module, "LaunchDescription")
    Node = getattr(launch_ros_actions, "Node")

    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="route_target_publisher_node",
            name="route_target_publisher",
            output="screen",
            parameters=[
                {
                    # 航点判定读取的全局坐标系名称；必须与 PID 和 TF 的 map 坐标保持一致。
                    "map_frame": "map",
                    # 航点判定读取的机体坐标系名称；通常与定位/激光的主体 frame 一致。
                    "laser_link_frame": "laser_link",
                    # 当前航点发布主题；默认发给 pid_control_pkg 的 /target_position。
                    "output_topic": "/target_position",
                    # route 判定用的 XY 位置容差，单位 cm；调大更容易判到点，但可能提前切点。
                    "position_tolerance_cm": 6.0,
                    # route 判定用的 yaw 容差，单位 deg；系统目标是保持 yaw=0，调大后更容易放过偏航误差。
                    "yaw_tolerance_deg": 5.0,
                    # route 判定用的高度容差，单位 cm；调大后更容易完成航点，但高度精度会下降。
                    "height_tolerance_cm": 6.0,
                    # route 判定用的连续满足帧数；调大更稳，不容易因抖动误切点，但切点会稍慢。
                    "reach_hold_frames": 3,
                    # 视觉接管成功所需的像素半径阈值，单位 px；调大更容易触发成功，但对准会变松。
                    "visual_align_pixel_threshold": 100.0,
                    # 视觉接管连续对准帧数；调大更稳，但视觉任务完成会更慢。
                    "visual_align_required_frames": 3,
                    # 视觉接管最长等待时间，单位 s；超时后会跳过当前视觉航点。
                    "visual_takeover_timeout_sec": 5.0,
                    # fine_data 新鲜度阈值，单位 s；调小更保守，调大则会容忍更旧的视觉数据。
                    "fine_data_stale_timeout_sec": 0.5,
                }
            ],
        )
    ])
