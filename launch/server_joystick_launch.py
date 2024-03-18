from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='mini_tank_ros2',
            executable='video_subscriber',
            name='video_subscriber',
            output='screen'
        )
    ])