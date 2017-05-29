#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import actionlib

import jaco2_msgs.msg
import sensor_msgs.msg

import sys


def pose_client():
    client = actionlib.SimpleActionClient('/jaco_arm_driver/arm_joint_angles', jaco2_msgs.msg.ArmJointAnglesAction)

    goal = jaco2_msgs.msg.ArmJointAnglesGoal()

    if len(sys.argv) < 8:
        goal.angles.joint1 = 4.8089
        goal.angles.joint2 = 2.9226
        goal.angles.joint3 = 1.0028
        goal.angles.joint4 = 4.2031
        goal.angles.joint5 = 1.4448
        goal.angles.joint6 = 1.3206
        goal.type = jaco2_msgs.msg.ArmJointAnglesGoal.RADIAN

        rospy.logwarn("Using test goal: \n%s", goal)
    else:
        goal.type   = float(sys.argv[1])
        goal.angles.joint1 = float(sys.argv[2])
        goal.angles.joint2 = float(sys.argv[3])
        goal.angles.joint3 = float(sys.argv[4])
        goal.angles.joint4 = float(sys.argv[5])
        goal.angles.joint5 = float(sys.argv[6])
        goal.angles.joint6 = float(sys.argv[7])


    client.wait_for_server()
    rospy.loginfo("Connected to Pose server")

    client.send_goal(goal)

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted, pre-empting goal")
        client.cancel_all_goals()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('arm_pose_client', anonymous=True)
        result = pose_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException: 
        rospy.loginfo("Program interrupted before completion")
        
