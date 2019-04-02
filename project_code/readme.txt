启动命令：rostopic pub -1 /Shooting_command std_msgs/UInt8 1



主机端：roslaunch robot_moveit_config demo.launch 



树梅派端：roslaunch serial_node start_serial.launch 	


测试: rostopic pub -1 /degree std_msgs/String 90,63,90,174,90,22,161,127,90,79,90,121,89,32,136,169
