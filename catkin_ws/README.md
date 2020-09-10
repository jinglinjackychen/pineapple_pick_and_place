1. roslaunch zed_wrapper zedm.launch (camera)
2. rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSBX (gripper)
   USBX
3.  (FCN)
4. roslaunch arm_operation ur5_real.launch tool_length:=0.18 robot_ip:=192.168.50.11 (robot arm)
5. rosrun tf static_transform_publisher x y z qx qy qz qw base_link zed_left_camera_frame 10
6. rosrun arm_operation goto_pose (function)

rosrun tf static_transform_publisher 0.464005 0.009211 0.744561 0.003557 0.704522 0.013398 0.709546 base_link camera_link 10 (0831 d435)

rosrun tf static_transform_publisher 0.503521 0.009987 0.738177 0.513044 0.487423 -0.511339 0.487584 base_link zed_base_link 10
rosrun tf static_transform_publisher 0.473521 0.009987 0.738177 0.513044 0.487423 -0.511339 0.487584 base_link zed_base_link 10 (0714)

rosrun tf static_transform_publisher 0.540847 -0.381663 0.617452 -0.271014 0.274854 0.627296 0.676392 base_link zed_base_link 10 (0831 right_camera)

terminal 1. Connecting to Dynamixel bus

	laptop $ roslaunch dynamixel_tutorials controller_manager.launch
-----------------------------------------------------------------------------------------------
terminal 2. Start the controller

	laptop $ roslaunch dynamixel_tutorials start_dual_motor_controller.launch
-----------------------------------------------------------------------------------------------
terminal 2. Control the gripper

	(open)
	laptop $ rostopic pub -1 /dual_motor_controller/command std_msgs/Float64 -- 0.5
	(close)
	laptop $ rostopic pub -1 /dual_motor_controller/command std_msgs/Float64 -- -0.55
