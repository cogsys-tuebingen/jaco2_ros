#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import actionlib

import jaco2_msgs.msg
import sensor_msgs.msg

import sys

position = None
def cb(state):
	global position 
	position = state.position

def pose_client():
    client = actionlib.SimpleActionClient('/jaco_21_driver/arm_joint_angles', jaco2_msgs.msg.ArmJointAnglesAction)
    sub = rospy.Subscriber("/jaco_22_driver/out/joint_states", sensor_msgs.msg.JointState, cb)
    goal = jaco2_msgs.msg.ArmJointAnglesGoal()

    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
	if position:
		goal.angles.joint1 = position[0]
		goal.angles.joint2 = position[1]
		goal.angles.joint3 = position[2]
		goal.angles.joint4 = position[3]
		goal.angles.joint5 = position[4]
		goal.angles.joint6 = position[5]
		goal.type = jaco2_msgs.msg.ArmJointAnglesGoal.RADIAN
		print goal
		client.cancel_all_goals()
    		client.send_goal(goal)
#		client.wait_for_result()
        rate.sleep()


    return 1

if __name__ == '__main__':
    try:
        rospy.init_node('arm_pose_client', anonymous=True)
        result = pose_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException: 
        rospy.loginfo("Program interrupted before completion")
        
