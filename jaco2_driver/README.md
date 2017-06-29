# Jaco2 Driver & ROS Node
This package contains the Jaco2 driver and the ROS main Loop publishing the Jaco2 data and sending data to the manipulator.
The jaco2_driver is a simple FSM reading and writing data in each cycle. Per cycle only one. The duration of a cycle depends on the runtime of the executed commands.

## ROS Node Parameters
#### Parameters

| name | type  | default | function |
| --------|-----|------| ----------- |
right_arm|bool| true| Use the jaco2 as a right or left arm.|
|jaco_use_accelerometer_calib|bool|false|Use a accelerometer calibration (yaml file has to be provided, see "jaco_accelerometer_calibration_file")|
|jaco_use_torque_calib|bool| false| Use a torque sensor calibration (yaml file has to be provided, see "jaco_torque_calibration_file". UNDER DEVELOPMENT|
|jaco_serial|string| ""| A Jaco2 if the given serial number will be search and control. If a empty string is provided any Jaco2 will be connected to.|
|tf_prefix|string|jaco_| frame_id prefix for tf|
|vel_controller_type|string|VEL | Changes the velocity controller type. Provided are a standard velocity controller "VEL" and a collision repelling velocity controller "VEL_COLL"|
trajectory_controller|string|TRAJ_P2P_VEL|Changes the trajactory traking controller controller type. One can choose beween velocity and torque control and collision repelling or  rigid controllers. Possible controllers: TRAJ_P2P_VEL, TRAJ_P2P_VEL_COLL, TRAJ_P2P_TOR, TRAJ_P2P_TOR_COLL. Attention: Torque control is still under development.|
|jaco_accelerometer_calibration_file|string|""|Povides calibration parameters for the accelerometers. Paramerters can be optained using jaco2_calibration.|
|jaco_torque_calibration_file|string|""|Under development. Provides a calibration for the torque sensors.|
|jaco_velocity_calibration_file|string|""| Provides a velocity sensor calibration. For some API versions the sensed velocity is smaller than the actual. Calibration can be optianed using jaco2_calibration_utils.|
|jaco_gravity_calibration_file|string|""|Gravitiy parameter parameter calibration, provided by the Jaco2 API. Get parameters by running: rosrun jaco2_driver gravity_parameter_estimation.