from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="pid_control_pkg",
            executable="position_pid_controller",
            name="position_pid_controller",
            output="screen",
            parameters=[
                {
                    # 控制循环频率，单位 Hz；调高响应更快，但会放大噪声和串口负担。
                    "control_frequency": 50.0,
                    # PID 使用的全局坐标系名称；需要和 TF 中的定位结果一致。
                    "map_frame": "map",
                    # PID 读取当前位置的机体坐标系名称；通常与激光或机体主坐标一致。
                    "laser_link_frame": "laser_link",
                    # XY 位置环比例系数；调大收敛更快，但更容易在目标附近来回摆动。
                    "kp_xy": 0.8,
                    # XY 位置环积分系数；用于消除稳态误差，调大后更容易积累过冲。
                    "ki_xy": 0.0,
                    # XY 位置环微分系数；用于抑制逼近目标时的冲击，过大时会放大测量抖动。
                    "kd_xy": 0.2,
                    # Yaw 角速度环比例系数；调大后朝向回正更快，但可能引入左右摇头。
                    "kp_yaw": 1.0,
                    # Yaw 角速度环积分系数；用于消除持续偏航误差，通常从 0 开始调。
                    "ki_yaw": 0.0,
                    # Yaw 角速度环微分系数；用于减小回正时的过冲和抖动。
                    "kd_yaw": 0.2,
                    # 高度环比例系数；调大上升/下降更积极，但更容易上下振荡。
                    "kp_z": 1.0,
                    # 高度环积分系数；用于抵消长期高度偏差，过大时容易积累。
                    "ki_z": 0.0,
                    # 高度环微分系数；用于抑制高度方向的冲击和过冲。
                    "kd_z": 0.2,
                    # XY 速度限幅，单位 cm/s；调大能更快接近目标，但靠近航点时更容易冲过头。
                    "max_linear_velocity": 33.0,
                    # 接近航点时的限速触发距离，单位 cm；进入该范围后，XY 合速度将被压到指定上限。
                    "approach_slowdown_distance_cm": 20.0,
                    # 接近航点时的 XY 合速度上限，单位 cm/s；用于减小近点过冲和抖动。
                    "approach_max_linear_velocity": 12.0,
                    # 偏航角速度限幅，单位 deg/s；调大回正更快，但机体转向会更猛。
                    "max_angular_velocity": 30.0,
                    # 垂直速度限幅，单位 cm/s；调大升降更快，但高度稳定性会下降。
                    "max_vertical_velocity": 30.0,
                    # 视觉接管 X 方向比例系数；调大画面横向误差收敛更快。
                    "visual_kp_x": 0.08,
                    # 视觉接管 X 方向积分系数；用于消除持续像素偏差，通常从 0 开始。
                    "visual_ki_x": 0.0,
                    # 视觉接管 X 方向微分系数；用于减小视觉误差快速变化时的抖动。
                    "visual_kd_x": 0.01,
                    # 视觉接管 Y 方向比例系数；调大画面纵向误差收敛更快。
                    "visual_kp_y": 0.08,
                    # 视觉接管 Y 方向积分系数；用于消除持续像素偏差，通常从 0 开始。
                    "visual_ki_y": 0.0,
                    # 视觉接管 Y 方向微分系数；用于减小视觉误差快速变化时的抖动。
                    "visual_kd_y": 0.01,
                    # 视觉接管像素死区，单位 px；调大后小范围误差不会驱动机体，但会降低对准精度。
                    "visual_pixel_deadzone": 5.0,
                    # 视觉接管 XY 最大速度，单位 cm/s；调大能更快对准，但更容易在目标附近抖动。
                    "visual_max_xy_velocity": 20.0,
                    # 视觉数据超时阈值，单位 s；调小更保守，调大则会容忍更旧的图像数据。
                    "visual_data_timeout_sec": 0.5,
                }
            ],
        )
    ])
