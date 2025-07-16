import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_twist_mux')
    config_path = os.path.join(pkg_share, 'config', 'config.yaml')
    
    twist_mux_node = Node(
        package='ros2_twist_mux',
        executable='ros2_twist_mux.py',
        name='ros2_twist_mux',
        parameters=[config_path],
        output='screen',
        emulate_tty=True,
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
    )

    test_twist_mux = Node(
        package='ros2_twist_mux',
        executable='test_twist_mux.py',
        name='test_twist_mux',
        output='screen',
    )
    
    return LaunchDescription([
        twist_mux_node,
        joy,
        test_twist_mux,
    ])