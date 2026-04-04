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

    route_params = {
        # Frames and target publishing
        "map_frame": "map",                         # Global target frame
        "laser_link_frame": "laser_link",           # Robot pose frame used for reach check
        "output_topic": "/target_position",         # Target topic sent to PID

        # Reach condition thresholds
        "position_tolerance_cm": 6.0,               # XY position tolerance in cm
        "yaw_tolerance_deg": 5.0,                   # Yaw tolerance in deg
        "height_tolerance_cm": 6.0,                 # Height tolerance in cm

        # Visual takeover condition
        "visual_align_pixel_threshold": 100.0,      # Alignment radius threshold in pixels
        "visual_align_required_frames": 3,          # Required consecutive aligned frames
        "visual_takeover_timeout_sec": 5.0,         # Max takeover duration in seconds
        "fine_data_stale_timeout_sec": 0.5,         # fine_data timeout in seconds
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
