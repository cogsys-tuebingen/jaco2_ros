# CS Jaco 2 ROS Package

## Maintainer 

- [Adrian Zwiener](http://www.cogsys.cs.uni-tuebingen.de/mitarb/zwiener/welcome_e.html) <<adrian.zwiener@uni-tuebingen.de>>, [Cognitive Science](http://ai.uni-bremen.de/), University of Tübingen

## Table of contents
- [Description](#description)
- [Dependencies](#dependencies)
- [Install](#install)

## Description

This package contains a driver and ROS interfaces for the Kinova Jaco 2 manipulator. Besides, a kinematic, a dynamic model, and a moveit configuration is included. Moreover, we supply tools for calibration of the dynamic model and the Jaco's accelerometers.

It contains:
-  [the diver](#jaco2_driver) containg several controller and basic input and output
-  [a library](#jaco2_kin_dyn_lib) for kinematic and dynamic models
-  [calibration tools](#jaco2_calibration) to calibrate the dynamic model and the accelerometers
- [a moveit config](#jaco2_moveit_config) to use the Jaco 2 with moveit
- [simple moveit apps](#jaco2_simple_moveit_apps) such as random sampling
- [the model](#jaco2_description) of the Jaco 2


## Dependencies
-  [MoveIt!](http://moveit.ros.org/)
- [tracIK](http://wiki.ros.org/trac_ik)
- (optional) [imu_tk](https://gitlab.cs.uni-tuebingen.de/utils/imu_tk) for accelerometer calibration
- (optional)  [Ceres](http://ceres-solver.org/)  for accelerometer calibration

## Install

i. Simply install MoveIt!, tracIK, and Ceres using apt-get: 
```
	sudo apt-get install ros-<distro>-moveit*
	sudo apt-get install ros-<distro>-trac-ik
	sudo apt-get install libceres-dev
```
ii. cd to your workspace and clone imu_tk:
```
	cd <catkin_ws>/src
	git clone gitlab@gitlab.cs.uni-tuebingen.de:utils/imu_tk.git
```
iii. Build your workspace:
```
	cd ..
	catkin_make
```
iv. Finally, copy the udev rule from the jaco2_driver folder to your system:
```
	sudo cp src/jaco2_ros/jaco2_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/

```
Now you can use the Jaco 2 with ROS!

## jaco2_driver

### Start

Currently, there are 2 launch files for our 2 Jaco 2s. To bring up one Jaco2 with calibrated accelerometers launch the launch file of your Jaco2:
```
	roslaunch jaco2_driver jaco2-1_calib.launch
	roslaunch jaco2_driver jaco2-2_calib.launch
```
The label indicating which Jaco 2 you use, can be found on the back of the Jaco 2. If you do not want to use the calibration or two arms simultaneously you can also start the arm by:
```
rosrun jaco2_driver jaco2_driver_node
```
### Topics
The Jaco 2 Node publishes information on topics marked with: 
```
<node_name>/out/x
```
You can publish commands to the Jaco 2 on topics with:
```
<node_name>/in/x


```
- joint_velocity Topic
-- to publish joint velocities to the controller publish our velocity with at least a rate of 20 Hz. Otherwise your comands will be ignored. Be carfull while publishing velocities, collisions are currently not avoided!
### Action Server
Currently, we have 3 action servers to control the arm:

- **arm_joint_angles**:  
Move the arm to a given joint postion.
== Attention:==
== Use this command only if you are certain that the Jaco 2 will not collide with its enviroment or with itself.==
== NEVER EVER SET ALL JOINT ANGLES TO 0  ==

- **finger_joint_angles**: 
Control the fingers to the given position.
- **gripper_command**:
Performs a power grip. Closes the gripper untill the finger do not move anymore.

- **follow_joint_trajectory**:
Trajectory tracking controller using PID velocity control.

### Services
There are 4 services available:

- **/jaco_arm_driver/in/home_arm**:
Brings the arm to its home position.== Only use this if you are sure that the Jaco 2 will not collide with its enviroment. Currently, this service is not available if you want to use a "left" Jaco  2 arm.==

- **/jaco_arm_driver/in/set_torque_zero**:
Sets a torque sensor to zero. Only use this command in torque zero position:
 [ *, 180° 180°, 0, 0, 180°]

- **/jaco_arm_driver/in/stop**:
Stops the API any command will be ignored.

- **/jaco_arm_driver/in/start**:
Releases the API if it was stopped previously.


## jaco2_kin_dyn_lib
A wrapper library for the [orocos Kinematics and Dynamics Library (KDL)](http://wiki.ros.org/orocos_kdl). Besides, this library contains some additional methods, e.g. regression matrix for dynamic calibration or a modified recursive netwon algortihm used e.g. in external torque estimation.
TODO ...

## jaco2_moveit_config
Start MoveIt! and rviz by using:
```
roslaunch jaco2_moveit_config moveit_full_rviz.launch
```
Now you can plan and execute trajectories. For a simulation run:
```
roslaunch jaco2_moveit_config demo.launch
```

## jaco2_calibration
TODO
## jaco2_simple_moveit_apps
TODO
## jaco2_description
The robot model which holds  all parameters needed for the kinematic and the dynamic model. Additionally, meshes are included for visualization and collision checking.



