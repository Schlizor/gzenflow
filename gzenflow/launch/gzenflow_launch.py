from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gzenflow',
            executable='network_manager',
            name='network_manager',
            output='screen'
        ),
        Node(
            package='gzenflow',
            executable='controller',
            name='bridge_controller',
            output='screen'
        ),
        Node(
            package='gzenflow',
            executable='ros2_gstreamer_streamer',
            name='ros2_gstreamer_streamer',
            output='screen',
        )
    ])

