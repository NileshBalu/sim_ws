import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path to your packages
    pkg_description = get_package_share_directory('a2_description')
    
    # Paths to files
    default_urdf_path = os.path.join(pkg_description, 'urdf', 'a2.urdf')
    default_rviz_config_path = os.path.join(pkg_description, 'rviz', 'default.rviz')

    # 2. Launch Arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_model', 
            default_value=default_urdf_path, 
            description='Absolute path to robot urdf file'
        ),
        DeclareLaunchArgument(
            'rviz_config', 
            default_value=default_rviz_config_path, 
            description='Absolute path to rviz config file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (MuJoCo) clock if true'
        ),

        # 3. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['cat ', LaunchConfiguration('urdf_model')]), value_type=str),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Dedicated Clock Publisher
        Node(
            package='a2_description',
            executable='sim_clock_node',
            condition=IfCondition(LaunchConfiguration('use_sim_time'))
        ),

        # 4. The C++ Bridge Node (LowState -> JointState/IMU)
        Node(
            package='a2_description',
            executable='a2_bridge_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 5. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])