from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    uart_params = {
        # Serial bridge runtime parameters
        "update_rate": 100.0,          # TF lookup and velocity send rate in Hz
        "source_frame": "map",         # Velocity source frame
        "target_frame": "laser_link",  # Frame used before sending to STM32
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
