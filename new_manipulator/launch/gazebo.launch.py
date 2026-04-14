from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    pkg_path = get_package_share_directory('new_manipulator')
    urdf_file = os.path.join(pkg_path, 'urdf', 'model.urdf')
    custom_world_path = os.path.join(pkg_path, 'worlds', 'custom.world')
    
    # Declare launch argument for world file path
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=custom_world_path,
        description='Full path to world file to load'
    )
    world = LaunchConfiguration('world')
    
    print("Custom world path:", custom_world_path)
    print("Exists:", os.path.isfile(custom_world_path))
    
    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    # 1. Robot State Publisher - WITH use_sim_time
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content}, 
            {'use_sim_time': True}
        ]
    )
    
    # 2. Spawn Entity - WITH use_sim_time parameter

    spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    name='spawn_entity',
    arguments=[
        '-entity', 'new_manipulator',
        '-topic', '/robot_description',
        '-x', '0', '-y', '0', '-z', '0.0'
    ],
    output='screen',
    parameters=[{'use_sim_time': True}]
)

    spawn_entity_node = TimerAction(
    period=2.0,
    actions=[spawn_entity]
)
    
    # 3. Controller Spawners - ALL WITH use_sim_time
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    arm_group_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_group_controller', '--controller-manager', '/controller_manager'],

        output='screen',
    )
    
    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_controller', '--controller-manager', '/controller_manager']     ,
        output='screen',
    )
    
    # 4. Event Handlers for Sequential Loading
    load_joint_state_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
            TimerAction(
                period=2.0,
                actions=[joint_state_broadcaster_spawner]
            )]
        )
    )
    
    load_arm_and_hand_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_group_controller_spawner, hand_controller_spawner]
        )
    )
    
    return LaunchDescription([
        declare_world_arg,
        # Launch Gazebo WITH use_sim_time enabled
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={
                'world': world,
                'use_sim_time': 'true'  # CRITICAL: Enable sim time for Gazebo
            }.items()
        ),
        # Robot Description Publisher
        robot_state_publisher_node,
        # Spawn the Robot
        spawn_entity_node,
        # Load Controllers After Spawning
        load_joint_state_after_spawn,
        load_arm_and_hand_after_jsb
    ])
