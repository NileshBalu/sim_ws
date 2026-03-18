import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to a config file (optional, but good for joy settings)
    # If you have a custom yaml for joy params, you'd load it here.

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',  # Standard linux joystick path
            'deadzone': 0.05,
            'autorepeat_rate': 500.0,
        }]
    )

    teleop_joy = Node(
        package='teleop',
        executable='teleop_joy',
        output='screen',
        parameters=[{
            'linear_speed_limit': 0.5,   # Meters per second
            'angular_speed_limit': 1.0,  # Radians per second
        }]
    )

    return LaunchDescription([
        joy_node,
        teleop_joy
    ])