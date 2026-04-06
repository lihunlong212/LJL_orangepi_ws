from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    uart_params = {
        # 串口桥运行参数
        "update_rate": 100.0,          # TF 查询与速度发送频率，单位 Hz
        "source_frame": "map",         # 速度源坐标系
        "target_frame": "laser_link",  # 发送到 STM32 前使用的目标坐标系
    }

    return LaunchDescription([
        Node(
            package="uart_to_stm32",
            executable="uart_to_stm32_node",
            name="uart_to_stm32",
            parameters=[uart_params],
            output="screen",
        )
    ])
