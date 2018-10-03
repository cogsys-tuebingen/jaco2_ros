# Jaco2 Driver & ROS Node
This package contains the Jaco2 driver and the ROS main Loop publishing the Jaco2 data and sending data to the manipulator.
The jaco2_driver is a simple FSM reading and writing data in each cycle. Per cycle only one. The duration of a cycle depends on the runtime of the executed commands.

## ROS Node Parameters
#### Parameters

| name | type  | default | function |
| --------|-----|------| ----------- |
connection_type|string| USB| Select how the arm is connected: "USB" or  "ethernet".|
ethernet/local_machine_IP|string| 192.168.100.100| IP address of the PC. Adapter which is connected to the Jaco|
ethernet/local_robot_IP|string| 192.168.100.100| IP address of the Jaco. Adapter which is connected to the Jaco|
ethernet/subnet_mask|string| 255.255.255.0| Subnet mask|
ethernet/local_cmd_port|int| 25000|Command Port|
ethernet/local_broadcast_port|int| 25025| Broadcast Port|
right_arm|bool| true| Use the jaco2 as a right or left arm.|
move_home|bool| true| If true, on start up, move arm to home and initialize (open) fingers If false, stay in current configuration.|
init_fingers|bool| true| Initalize fingers while initalizing. If true fingers are initalized and opened.|
|jaco_use_accelerometer_calib|bool|false|Use a accelerometer calibration (yaml file has to be provided, see "jaco_accelerometer_calibration_file")|
|jaco_use_torque_calib|bool| false| Use a torque sensor calibration (yaml file has to be provided, see "jaco_torque_calibration_file". UNDER DEVELOPMENT|
|jaco_serial|string| ""| A Jaco2 if the given serial number will be search and control. If a empty string is provided any Jaco2 will be connected to.|
|tf_prefix|string|jaco_| frame_id prefix for tf|
|vel_controller_type|string|VEL | Changes the velocity controller type. Provided are a standard velocity controller "VEL" and a collision repelling velocity controller "VEL_COLL"|
trajectory_controller|string|TRAJ_P2P_VEL|Changes the trajactory traking controller controller type. One can choose beween velocity and torque control and collision repelling or  rigid controllers. Possible controllers: TRAJ_P2P_VEL, TRAJ_P2P_VEL_COLL, TRAJ_P2P_TOR, TRAJ_P2P_TOR_COLL. Attention: Torque control is still under development.|
|jaco_accelerometer_calibration_file|string|""|Povides calibration parameters for the accelerometers. Paramerters can be optained using jaco2_calibration.|
|jaco_torque_calibration_file|string|""|Under development. Provides a calibration for the torque sensors.|
|robot_model_param_sever|string|/robot_description|  Name of robot_model in parameter server.|
|robot_model_base_link|string|jaco_link_base| Base link of the controlled chain. Can differ e.g. jaco_left_link_base jaco_right_link_base
|robot_model_tip_link|string|jaco_link_hand|Tip link of the controlled chain. Can differ e.g. jaco_left_link_hand jaco_right_link_hand
|publish_fingers|bool|true| Decide if you want to publish the finger joint angles in the joint_state message.


##Warning
Using the collision repelling controllers in a debugger might be a problem, since due to very long sampling periods the collision estimation might get numerically instable.  Be extra careful and just turn the Jaco2 of if something unexpected happens.

Besides, torque control is not recommended and still under development.
##Driver 
The ROS node is just an interface communicating with the real driver. This driver acts as a small Finite State Machine reading and writing commands cyclically.
Different controllers are used implementing the commands. In addition, reading information from the arm can be controlled by the controllers. The information is stored in the jaco2_state.![driver_scematic](/home/zwiener/workspace/development/src/jaco2/jaco2_ros/jaco2_driver/jaco2_driver.png  "driver")
Currently, now cartesian controllers are implemented. You can use MoveIt or calculate the corresponding joint velocities externally.