#!/bin/bash
rosrun jaco2_driver angle_action_client.py -s $1 -d "0 180 180 0 0 0"
for i in $(seq 1 6);
	do
		echo "Setting torque sensor of actuator $i to zero."
		rosservice call $1/in/set_torque_zero "actuator: $i"
	done 


