from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    camera_index = LaunchConfiguration('camera_index')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    camera_fps = LaunchConfiguration('camera_fps')
    show_display = LaunchConfiguration('show_display')

    return LaunchDescription([
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='15'),
        DeclareLaunchArgument('show_display', default_value='false'),
        Node(
            package='circle_detector_pkg',
            executable='circle_detector_node',
            name='circle_detector',
            output='screen',
            parameters=[{
                'camera_index': camera_index,
                'width': width,
                'height': height,
                'camera_fps': camera_fps,
                'show_display': show_display,
            }],
        ),
    ])
