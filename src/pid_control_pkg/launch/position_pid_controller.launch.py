from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pid_params = {
        # Control loop and TF frames
        "control_frequency": 50.0,         # PID update frequency in Hz
        "map_frame": "map",                # Global target frame
        "laser_link_frame": "laser_link",  # Robot pose frame used by PID

        # Normal position control: XY position error -> XY velocity command
        "kp_xy": 0.8,                      # XY proportional gain
        "ki_xy": 0.0,                      # XY integral gain
        "kd_xy": 0.2,                      # XY derivative gain

        # Yaw control: yaw error -> yaw rate command
        "kp_yaw": 1.0,                     # Yaw proportional gain
        "ki_yaw": 0.0,                     # Yaw integral gain
        "kd_yaw": 0.2,                     # Yaw derivative gain

        # Height control: height error -> Z velocity command
        "kp_z": 1.0,                       # Height proportional gain
        "ki_z": 0.0,                       # Height integral gain
        "kd_z": 0.2,                       # Height derivative gain

        # Output limits
        "max_linear_velocity": 33.0,       # Max XY velocity in cm/s
        "max_angular_velocity": 30.0,      # Max yaw rate in deg/s
        "max_vertical_velocity": 30.0,     # Max Z velocity in cm/s

        # Visual takeover: pixel error -> XY velocity command
        "visual_kp_x": 0.08,               # Visual X proportional gain
        "visual_ki_x": 0.0,                # Visual X integral gain
        "visual_kd_x": 0.01,               # Visual X derivative gain
        "visual_kp_y": 0.08,               # Visual Y proportional gain
        "visual_ki_y": 0.0,                # Visual Y integral gain
        "visual_kd_y": 0.01,               # Visual Y derivative gain
        "visual_pixel_deadzone": 5.0,      # Deadzone in pixels
        "visual_max_xy_velocity": 20.0,    # Max XY velocity during takeover in cm/s
        "visual_data_timeout_sec": 0.5,    # Hold XY at zero if fine_data times out
    }

    return LaunchDescription([
        Node(
            package="pid_control_pkg",
            executable="position_pid_controller",
            name="position_pid_controller",
            output="screen",
            parameters=[pid_params],
        )
    ])
