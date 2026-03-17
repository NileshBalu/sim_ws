import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get package directories
    description_dir = get_package_share_directory('a2_description')
    mujoco_dir = get_package_share_directory('a2_mujoco')

    # 2. Declare arguments
    scene_arg = DeclareLaunchArgument(
        'scene',
        default_value='scene.xml',
        description='Scene file to load from a2_description/a2/'
    )

    # 3. Use PathJoinSubstitution to safely combine LaunchConfiguration with strings
    # This handles the "scene" argument dynamically
    scene_path = PathJoinSubstitution([
        description_dir,
        'a2',
        LaunchConfiguration('scene')
    ])

    return LaunchDescription([
        scene_arg,
        Node(
            package='a2_mujoco',
            executable='a2_mujoco',
            output='screen',
            arguments=[
                scene_path,  # argv[1]: Path to the model XML (resolved at runtime)
                mujoco_dir   # argv[2]: Path to the directory containing config.yaml
            ],
            # Set CWD to the a2 folder so MuJoCo finds the 'assets/' folder
            cwd=os.path.join(description_dir, 'a2')
        )
    ])