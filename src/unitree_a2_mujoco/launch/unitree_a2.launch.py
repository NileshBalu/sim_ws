import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def create_nodes(context: LaunchContext):
    namespace = ""
    pkg_share = get_package_share_directory('unitree_a2_mujoco')
    
    # Set file paths
    mujoco_model_file = os.path.join(pkg_share, 'a2.xml')
    urdf_model_file = os.path.join(pkg_share, 'urdf', 'a2.urdf')
    ros2_control_params_file = os.path.join(pkg_share, "config", "unitree_a2_controllers.yaml")
    rviz_config_file = os.path.join(pkg_share, "config", "default.rviz")

    # Fetch launch configurations
    rviz = LaunchConfiguration("rviz")

    # Robot Description
    with open(urdf_model_file, 'r') as infp:
        robot_description_content = infp.read()
    robot_description = {'robot_description': robot_description_content}

    # Define the robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Define the mujoco node
    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        # name="controller_manager", # Match controller_manager name for spawners
        namespace=namespace,
        parameters=[
            robot_description,
            ros2_control_params_file,
            {"simulation_frequency": 500.0},
            {"use_sim_time": True},
            {"realtime_factor": -1.0},
            {"robot_model_path": mujoco_model_file},
            {"show_gui": True},
        ],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ]
    )

    # Define the load_joint_state_broadcaster node
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            ["/", "controller_manager"],
            '--param-file',
            ros2_control_params_file,
        ],
    )

    # Individual Leg Controllers
    fl_leg_controller = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["fl_leg_controller", "--controller-manager", ["/", "controller_manager"], '--param-file', ros2_control_params_file], 
        namespace="/"
    )
    
    fr_leg_controller = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["fr_leg_controller", "--controller-manager", ["/", "controller_manager"], '--param-file', ros2_control_params_file], 
        namespace="/"
    )
    
    rl_leg_controller = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["rl_leg_controller", "--controller-manager", ["/", "controller_manager"], '--param-file', ros2_control_params_file], 
        namespace="/"
    )
    
    rr_leg_controller = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["rr_leg_controller", "--controller-manager", ["/", "controller_manager"], '--param-file', ros2_control_params_file], 
        namespace="/"
    )

    # RViz Setup
    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}]
    )

    # Register an event handler to start controllers once mujoco is up
    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="MuJoCo started. Spawning A2 controllers..."),
                load_joint_state_broadcaster,
                fl_leg_controller,
                fr_leg_controller,
                rl_leg_controller,
                rr_leg_controller,
                rviz_node
            ],
        )
    )

    # Return the nodes and handlers
    return [
        robot_state_publisher,
        mujoco,
        load_controllers
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start rviz."
        ),
        OpaqueFunction(function=create_nodes)
    ])