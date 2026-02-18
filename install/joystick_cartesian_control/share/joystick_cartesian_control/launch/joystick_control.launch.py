#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch arguments
    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='joystick',
        description='Controller type: joystick or continuous'
    )
    
    declare_joystick_device = DeclareLaunchArgument(
        'joystick_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Configuration
    controller_type = LaunchConfiguration('controller_type')
    joystick_device = LaunchConfiguration('joystick_device')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Joy node - publishes joystick data
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': joystick_device,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }],
        output='screen'
    )
    
    # Joystick Cartesian Controller (default)
    joystick_controller = Node(
        package='joystick_cartesian_control',
        executable='joystick_cartesian_node',
        name='joystick_cartesian_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_controller_type,
        declare_joystick_device,
        declare_use_sim_time,
        joy_node,
        joystick_controller,
    ])
