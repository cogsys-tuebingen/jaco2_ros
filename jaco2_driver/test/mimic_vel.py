#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import actionlib

import jaco2_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import sys
import math
import tf.transformations

vel = [0 for i in range(6)]
pos = None
last_posl = [ 0 for i in range(6)]
kp = 2.0
kd = 0.1

def cb_master(state):
	global pos
	pos = state.position

def cb_slave(state):
	global vel 
	posl = state.position
	for i in range(6):
		diff_p = normalize(pos[i] -posl[i])
		diff_d = normalize(posl[i] -last_posl[i])
		vel[i] = kp * diff_p + kd * diff_d
		last_posl[i] = posl[i]
 

def normalize(val):
	while val < -math.pi:
		val = val + 2.0*math.pi
	while val >= math.pi:
		val = val - 2.0*math.pi
	return val
		
def pose_client():
    pub = rospy.Publisher('/jaco_21_driver/in/joint_velocity',jaco2_msgs.msg.JointVelocity, queue_size=10)
    sub22 = rospy.Subscriber("/jaco_22_driver/out/joint_states", sensor_msgs.msg.JointState, cb_master)
    sub21 = rospy.Subscriber("/jaco_21_driver/out/joint_states", sensor_msgs.msg.JointState, cb_slave)

    goal = jaco2_msgs.msg.JointVelocity()
    rate = rospy.Rate(65) # 10hz
    while not rospy.is_shutdown():
	if vel:
		goal.joint1 = vel[0]
		goal.joint2 = vel[1] 
		goal.joint3 = vel[2] 
		goal.joint4 = vel[3] 
		goal.joint5 = vel[4]
		goal.joint6 = vel[5]
		print goal
		pub.publish(goal)
        rate.sleep()





    return 1

if __name__ == '__main__':
    try:
        rospy.init_node('arm_pose_client', anonymous=True)
        result = pose_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException: 
        rospy.loginfo("Program interrupted before completion")
        
