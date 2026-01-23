#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Package
    pkg = FindPackageShare('new_manipulator_moveit_config')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    joystick_device_arg = DeclareLaunchArgument(
        'joystick_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # Include move_group launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'test.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joystick_device'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )
    
    # Joystick teleop node - DELAYED to let move_group start
    joystick_teleop_node = TimerAction(
        period=5.0,  # Wait 5 seconds for move_group to start
        actions=[
            Node(
                package='new_manipulator_moveit_config',
                executable='joystick_cartesian_teleop',
                name='joystick_cartesian_teleop',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
                output='screen',
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        joystick_device_arg,
        move_group_launch,
        joy_node,
        joystick_teleop_node,
    ])
