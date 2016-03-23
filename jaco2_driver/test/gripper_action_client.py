#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import actionlib

import control_msgs.msg

import sys


def pose_client():
    client = actionlib.SimpleActionClient('/jaco_arm_driver/gripper_command', control_msgs.msg.GripperCommandAction)

    goal = control_msgs.msg.GripperCommandGoal()

    if len(sys.argv) < 2:
        goal.command.max_effort = 0.5;

        rospy.logwarn("Using test goal: \n%s", goal)
    else:
        goal.command.max_effort = float(sys.argv[1])

    client.wait_for_server()
    rospy.loginfo("Connected to Finger server")

    client.send_goal(goal)

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted, pre-empting goal")
        client.cancel_all_goals()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('finger_pose_client')
        result = pose_client()
        rospy.loginfo("Result: \n%s", result)
    except rospy.ROSInterruptException: 
        rospy.loginfo("Program interrupted before completion")
        
