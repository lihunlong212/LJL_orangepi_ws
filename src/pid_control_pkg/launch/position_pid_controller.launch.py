from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pid_params = {
        # 控制循环与 TF 坐标系
        "control_frequency": 50.0,         # PID 更新频率，单位 Hz
        "map_frame": "map",                # 全局目标坐标系
        "laser_link_frame": "laser_link",  # PID 使用的机体位姿坐标系

        # 常规位置控制：XY 位置误差 -> XY 速度指令
        "kp_xy": 0.8,                      # XY 比例增益
        "ki_xy": 0.0,                      # XY 积分增益
        "kd_xy": 0.2,                      # XY 微分增益

        # 偏航控制：偏航误差 -> 偏航角速度指令
        "kp_yaw": 1.0,                     # 偏航比例增益
        "ki_yaw": 0.0,                     # 偏航积分增益
        "kd_yaw": 0.2,                     # 偏航微分增益

        # 高度控制：高度误差 -> Z 方向速度指令
        "kp_z": 1.0,                       # 高度比例增益
        "ki_z": 0.0,                       # 高度积分增益
        "kd_z": 0.2,                       # 高度微分增益

        # 输出限幅
        "max_linear_velocity": 33.0,       # XY 最大速度，单位 cm/s
        "max_angular_velocity": 30.0,      # 最大偏航角速度，单位 deg/s
        "max_vertical_velocity": 30.0,     # Z 方向最大速度，单位 cm/s

        # 视觉接管控制：像素误差 -> XY 速度指令
        "visual_kp_x": 0.08,               # 视觉 X 比例增益
        "visual_ki_x": 0.0,                # 视觉 X 积分增益
        "visual_kd_x": 0.01,               # 视觉 X 微分增益
        "visual_kp_y": 0.08,               # 视觉 Y 比例增益
        "visual_ki_y": 0.0,                # 视觉 Y 积分增益
        "visual_kd_y": 0.01,               # 视觉 Y 微分增益
        "visual_pixel_deadzone": 5.0,      # 像素死区
        "visual_max_xy_velocity": 20.0,    # 接管期间 XY 最大速度，单位 cm/s
        "visual_data_timeout_sec": 0.5,    # fine_data 超时后将 XY 保持为 0，单位秒
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
