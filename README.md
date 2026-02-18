ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
ros2 launch pid_motor_tune pid_tune.launch.py 

ros2 component load /moveit_servo_demo_container new_manipulator_servo moveit_servo::JoyToServoPub


ros2 param set /multi_motor_signal_generator enabled true



ros2 run joy joy_node 
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v6
ros2 launch new_manipulator unified_launch.py use_sim_time:=false
ros2 launch new_manipulator_moveit_config test.launch.py use_sim_time:=false
ros2 launch new_manipulator servo_example.launch.py 

ros2 run joy joy_node --ros-args -p device_id:=0 -r __node:=joy_node -r /joy:=/joy1
ros2 run joy_pack joystick_serial_node
c


# Switch to Servo
ros2 topic pub /servo_mode std_msgs/msg/Bool "data: true" --once
# Switch to PID
ros2 topic pub /control_mode std_msgs/msg/String "data: 'PID_CONTROL'" --once

# Switch to PWM
ros2 topic pub /control_mode std_msgs/msg/String "data: 'PWM_CONTROL'" --once
ros2 run new_manipulator_hardware mode_switcher.py 

cd ~/joystick_package_extreme_3d/

ros2 run joy_pack joystick_serial_node 

1 LEFT
2 RIGHT
3 2ND JOINT DOWN
4 2ND JOINT UP
5 3RD JOINT UP
6 3RD JOINT DOWN
7 4TH JOINT DOWN
8 4TH JOINT UP
6,4 ---BACKWARDS //THUMB ON RIGHT
5,3----FORWARDS //THUMB ON LEFT
6,3 --- -VE Z DOWNWARDS //SECOND CUT 
5,4 --- +VE Z UPWARDS //FIRST CUT
7-YES MOTION DOWN 
8-YES MOTION UP
9-ANTICLKWISE //AUGER OUT HEAD LEFT TILT
10-CLKWISE //AUGER IN HEAD RIGHT TILT


GRIPPER OPEN
GRIPPER CLOSE
 ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=false \
  enable_infra:=false \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  color_fps:=30 \
  color_width:=640 \
  color_height:=480
 ros2 run usb_cam usb_cam_node_exe   --ros-args   -p video_device:=/dev/video11   -p pixel_format:=yuyv2rgb   -p image_width:=640   -p image_height:=480   -p framerate:=30.0
