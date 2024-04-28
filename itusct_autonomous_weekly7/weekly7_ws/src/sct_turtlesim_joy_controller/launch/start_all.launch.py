from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='log',
            ros_arguments=['--ros-args', '--disable-stdout-logs']
            ),
            
        Node(
            package='sct_turtlesim_joy_controller',
            executable='joy_controller',
            name='joy_controller',
            output='screen',
            on_exit=Shutdown()),
    ],
    )
