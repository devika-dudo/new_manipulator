import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Declare use_sim_time argument at the very beginning
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # Default to true for Gazebo
        description='Use simulation time'
    )
    
    # Get the use_sim_time value
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get package directories
    pkg_share_manipulator = get_package_share_directory('new_manipulator')
    pkg_share_moveit_config = get_package_share_directory('new_manipulator_moveit_config')
    
    # Define all file paths explicitly
    urdf_file = os.path.join(pkg_share_manipulator, 'urdf', 'model.urdf')
    srdf_file = os.path.join(pkg_share_moveit_config, 'config', 'new_manipulator.srdf')
    kinematics_file = os.path.join(pkg_share_moveit_config, 'config', 'kinematics.yaml')
    joint_limits_file = os.path.join(pkg_share_moveit_config, 'config', 'joint_limits.yaml')
    
    moveit_config = (
        MoveItConfigsBuilder("new_manipulator", package_name="new_manipulator_moveit_config")
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .joint_limits(file_path=joint_limits_file)
        .to_moveit_configs()
    )
    
    # Load servo config
    servo_yaml = load_yaml("new_manipulator_servo", "config/new_manipulator_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    
    # RViz node - ADD use_sim_time parameter
    rviz_config_file = os.path.join(
        get_package_share_directory("new_manipulator_servo"),
        "config",
        "demo_rviz_config_my_robot.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {"use_sim_time": use_sim_time},  # ← ADDED
        ],
    )
    
    # Container - ADD use_sim_time parameter
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        parameters=[{"use_sim_time": use_sim_time}],  # ← ADDED
        composable_node_descriptions=[
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{
                    "child_frame_id": "base_link", 
                    "frame_id": "world",
                    "translation.z": 0.0,
                    "translation.x": 0.0,
                    "translation.y": 0.0,
                    "rotation.x": 0.0,
                    "rotation.y": 0.0,
                    "rotation.z": 0.0,
                    "rotation.w": 1.0,
                    "use_sim_time": use_sim_time,  # ← ADDED
                }],
            ),
            ComposableNode(
                package="new_manipulator_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="joy_to_servo",
                parameters=[{"use_sim_time": use_sim_time}],  # ← ADDED
            ),
        ],
        output="screen",
    )
    
    # Servo node - CHANGE use_sim_time to use parameter
    servo_node = Node(
        package="new_manipulator_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},  # ← CHANGED from False
        ],
        output="screen",
    )
    
    # Joy node - CHANGE use_sim_time to use parameter
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"use_sim_time": use_sim_time}],  # ← CHANGED from False
        remappings=[
            ('/joy', '/joy2'),
        ],
        output="screen",
    )
    
    return LaunchDescription([
        declare_use_sim_time,  # ← ADDED - must be first
        rviz_node,
        container,
        servo_node,
        joy_node,
    ])
