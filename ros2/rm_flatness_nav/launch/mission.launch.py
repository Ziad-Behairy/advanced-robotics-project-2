"""Tag detector + flatness planner. Start the robomaster driver first, in its own terminal."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('rm_flatness_nav'), 'config', 'robot.yaml')
    return LaunchDescription([
        Node(package='rm_flatness_nav', executable='tag_detector', parameters=[cfg], output='screen'),
        Node(package='rm_flatness_nav', executable='flatness_planner', parameters=[cfg], output='screen',
             emulate_tty=True),
    ])
