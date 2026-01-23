import launch
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkgPath = FindPackageShare(package='new_manipulator').find('new_manipulator')
    urdfModelPath = os.path.join(pkgPath, 'urdf/model.xacro')
    rvizConfigPath = os.path.join(pkgPath, 'config/config.rviz')
    
    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()
    
    params = {'robot_description': robot_desc}
    
    return launch.LaunchDescription([
        # Declare gui launch argument
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'),
        
        # Robot State Publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
        
        # Joint State Publisher (no GUI)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[params],  # Pass parameters instead of arguments
            condition=UnlessCondition(LaunchConfiguration('gui'))
        ),
        
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[params],  # Pass parameters instead of arguments
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rvizConfigPath]
        )
    ])


