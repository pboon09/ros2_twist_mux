import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_twist_mux')
    config_path = os.path.join(pkg_share, 'config', 'config.yaml')
    
    twist_mux_node = Node(
        package='ros2_twist_mux',
        executable='twist_mux.py',
        name='twist_mux',
        parameters=[config_path],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        twist_mux_node
    ])