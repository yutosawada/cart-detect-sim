import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('cart_detect_sim'),
        'config', 'cart_detect_sim.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config,
                              description='YAML with simulated cart positions.'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='cart_detect_sim',
            executable='cart_detect_sim_node',
            name='cart_detect_sim',
            parameters=[LaunchConfiguration('config')],
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
        ),
    ])
