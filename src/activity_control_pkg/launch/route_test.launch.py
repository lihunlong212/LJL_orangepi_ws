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
            executable="route_test_node",
            name="route_test_node",
            output="screen",
            parameters=[
                {
                    "map_frame": "map",
                    "laser_link_frame": "laser_link",
                    "output_topic": "/target_position",
                    "position_tolerance_cm": 6.0,
                    "yaw_tolerance_deg": 5.0,
                    "height_tolerance_cm": 6.0,
                    "visual_align_pixel_threshold": 100.0,
                    "visual_align_required_frames": 3,
                    "visual_takeover_target_height_cm": 40.0,
                    "visual_takeover_timeout_sec": 5.0,
                    "fine_data_stale_timeout_sec": 0.5,
                }
            ],
        )
    ])
