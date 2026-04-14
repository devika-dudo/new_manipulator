from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory # this looks up packages via environment variables like AMENT_PREFIX_PATH, so it only works when you source -- sourceing updates a bunch of environment variables like AMENT_PREFIX_PATH
# PYTHONPATH, PATH , LD_LIBRARY_PATH(.so files),COLCON_PREFIX_PATH 

def generate_launch_description():# this is a mandatory fn in ros2 python launch file, becoz when you run a launch file it searches for this function first , if its not there launch file fails 
    # ==================== PATHS ====================
    pkg_new_manipulator = get_package_share_directory('new_manipulator')
    pkg_new_manipulator_hardware = get_package_share_directory('new_manipulator_hardware')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    urdf_file = os.path.join(pkg_new_manipulator, 'urdf', 'robot.urdf.xacro')
    custom_world_path = os.path.join(pkg_new_manipulator, 'worlds', 'custom.world')
    controllers_config = os.path.join(pkg_new_manipulator_hardware, 'config', 'controllers.yaml')
    
    # ==================== LAUNCH ARGUMENTS ====================
    declare_use_sim_arg = DeclareLaunchArgument( # declares an input variable for launch file 
        'use_sim_time',
        default_value='true',
        description='Use simulation (true) or real hardware (false)'
    )
    
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=custom_world_path,
        description='Full path to world file to load (simulation only)'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time') # tells the launch system that such an argument will be there 
    world = LaunchConfiguration('world')# thats is give me the value filled in 'world' when launch file is run
    
    # ==================== PROCESS URDF WITH XACRO ====================
    robot_description_cmd = Command([ # runs a shell command thats converts xacro to urdf 
        'xacro ', urdf_file, 
        ' use_sim:=', use_sim_time
    ])
    
    # ==================== COMMON NODES ====================
    
    # Robot State Publisher
    robot_state_publisher_node = Node( #rsb node is run and the urdf is passed to it as a string
        package='robot_state_publisher',# rsb recieves urdf and /joint_states -- to build the tf tree (kinematic tree- computes fk )
        executable='robot_state_publisher',
        name='robot_state_publisher',#publishes /tf(for moving joints) and /tf_static(for static joints), it publishes /robot_description
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(robot_description_cmd, value_type=str)},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # ==================== SIMULATION NODES ====================
    
    spawn_entity_node = Node( #given gazebo wads already running
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'new_manipulator',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.27'
        ],
        output='screen',
        condition=IfCondition(use_sim_time)
    )
    #controller_manager is a ros2_control node that manages controllers, talks to hardware and exposes services like switch load controllers
    # Joint State Broadcaster for simulation
    #here the controller we are spawning is jsb
    # it reads position from hardware and publishes /joint_states 
    joint_state_broadcaster_spawner_sim = Node(
        package='controller_manager',
        executable='spawner',#spawner is a helper script that loads, configure and activates controller
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Arm Group Controller for simulation
    arm_group_controller_spawner_sim = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_group_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Hand Controller for simulation
    hand_controller_spawner_sim = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    #once robot launched in gazebo jsb node is run
    
    load_joint_state_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster_spawner_sim]
        ),
        condition=IfCondition(use_sim_time)
    )
    #once jsb is run arm and hand controllers and run
    load_arm_and_hand_after_jsb_sim = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_sim,
            on_exit=[arm_group_controller_spawner_sim, hand_controller_spawner_sim]
        ),
        condition=IfCondition(use_sim_time)
    )
    
    # ==================== HARDWARE NODES ====================
    
    # ROS2 Control Node with longer initialization time
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(robot_description_cmd, value_type=str)},
            controllers_config,#here we have send the yaml file to the ros2 control node , so ros2 control registers them
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=UnlessCondition(use_sim_time)
    )
    
    # Add delay before starting controller manager (give hardware time to init)
    delayed_ros2_control_node = TimerAction(period=3.0, actions=[ros2_control_node])
    
    # Controllers for hardware mode with proper delays
    
    joint_state_broadcaster_spawner_hw = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    arm_group_controller_spawner_hw = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_group_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    hand_controller_spawner_hw = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Start joint_state_broadcaster after ros2_control_node starts
    delayed_joint_state_broadcaster_hw = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner_hw])]
        ),
        condition=UnlessCondition(use_sim_time)
    )
    
    # Start arm controller after joint_state_broadcaster exits
    delayed_arm_controller_hw = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_hw,
            on_exit=[TimerAction(period=1.0, actions=[arm_group_controller_spawner_hw])]
        ),
        condition=UnlessCondition(use_sim_time)
    )
    
    # Start hand controller after arm controller exits
    delayed_hand_controller_hw = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_group_controller_spawner_hw,
            on_exit=[TimerAction(period=1.0, actions=[hand_controller_spawner_hw])]
        ),
        condition=UnlessCondition(use_sim_time)
    )
    
    # ==================== GAZEBO INCLUDE ====================
    
    gazebo_launch = IncludeLaunchDescription( #this runs gzclient(GUI) and gzserver(physics engine) 
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world}.items(),
        condition=IfCondition(use_sim_time)
    )
    
    # ==================== RETURN ====================
    
    return LaunchDescription([
        declare_use_sim_arg,
        declare_world_arg,
        
        # SIMULATION PATH
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        load_joint_state_after_spawn,
        load_arm_and_hand_after_jsb_sim,
        
        # HARDWARE PATH (with delays)
        delayed_ros2_control_node,
        delayed_joint_state_broadcaster_hw,
        delayed_arm_controller_hw,
        delayed_hand_controller_hw,
    ])
