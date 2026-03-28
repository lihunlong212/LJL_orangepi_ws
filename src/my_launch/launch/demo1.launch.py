import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, LifecycleNode
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition

def generate_launch_description():
    # 1. 获取包路径
    my_carto_pkg_share = FindPackageShare(package='my_carto_pkg').find('my_carto_pkg')
    uart_to_stm32_pkg_share = FindPackageShare(package='uart_to_stm32').find('uart_to_stm32')
    pid_control_pkg_share = FindPackageShare(package='pid_control_pkg').find('pid_control_pkg')
    activity_control_pkg_share = FindPackageShare(package='activity_control_pkg').find('activity_control_pkg')
    
    # 2. 定义 Launch 文件包含
    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_carto_pkg_share, 'launch', 'fly_carto.launch.py')
        )       
    )

    
    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_to_stm32_pkg_share, 'launch', 'uart_to_stm32.launch.py')
        )
    )
    
    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pid_control_pkg_share, 'launch', 'position_pid_controller.launch.py')
        )
    )   

    route_test_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(activity_control_pkg_share, 'launch', 'route_test.launch.py')
        )
    )

    drone_camera_node = Node(
        package='drone_camera_pkg',
        executable='drone_camera_node',
        name='drone_camera_node',
        output='screen',
        parameters=[
            {
                'camera_device': '/dev/video0',
                'frame_width': 640,
                'frame_height': 480,
                'fps': 15.0,
                'window_name': 'drone_camera_preview',
            }
        ]
    )
    

    

    return LaunchDescription([
        fly_carto_launch, # 立即启动
        uart_to_stm32_launch,
        position_pid_controller_launch,
        route_test_launch,
       # drone_camera_node,

    ])
