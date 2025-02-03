from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux.py',
            name='twist_mux',
            parameters=['/home/pboon09/ros2_ws/src/twist_mux/config/config.yaml']
        )
    ])