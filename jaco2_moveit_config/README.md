# jaco2_moveit_config
This packages contain all necessary configurations to plan and control trajectories.

## launch moveit

For a demo (kinematic simulation) you can just use:

	roslaunch jaco2_moveit_config demo.launch	

To control the real Jaco2:

	roslaunch jaco2_moveit_config moveit_full.launch joint_state_topic:=<joint_state topic> controller_in:=<jaco controller> 

###joint_state_topic:
If you  do not specifiy it the default will be used:

	/jaco_arm_driver/out/joint_states
Depending on which Jaco you started you can also use

	/jaco_21_driver/out/joint_states
	/jaco_22_driver/out/joint_states
	
###<jaco controller>
	jaco (default)
	jaco_21
	jaco_22
