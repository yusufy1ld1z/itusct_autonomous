import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sct_img_filter'),
        'config',
        'config.yaml'
    )
    return LaunchDescription([
        Node(
            package='sct_img_filter',
            executable='img_publisher',
            name='img_publisher',
            parameters=[config],
            output='screen'),
        Node(
            package='sct_img_filter',
            executable='img_filterer',
            name='img_filterer',
            parameters=[config],
            output='screen')
    ])