#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import actionlib

import jaco2_msgs.msg

import sys


def pose_client():
    client = actionlib.SimpleActionClient('/jaco_arm_driver/gripper_command', jaco2_msgs.msg.GripperControlAction)

    goal = jaco2_msgs.msg.GripperControlGoal()

    if len(sys.argv) < 4:
        goal.useFinger1 = True
        goal.useFinger2 = True
        goal.useFinger3 = True

        rospy.logwarn("Using test goal: \n%s", goal)
    else:
        if int(sys.argv[1]) == 1:
            goal.useFinger1 = True
        else:
            goal.useFinger1 = False

        if int(sys.argv[2]) == 1:
            goal.useFinger2 = True
        else:
            goal.useFinger2 = False

        if int(sys.argv[3]) == 1:
            goal.useFinger3 = True
        else:
            goal.useFinger3 = False

        if int(sys.argv[4]) == 1:
            goal.usePos = True
        else:
            goal.usePos = False

        goal.posFinger1 = int(sys.argv[5])
        goal.posFinger2 = int(sys.argv[6])
        goal.posFinger3 = int(sys.argv[7])
        rospy.loginfo("Using goal: \n%s", goal)

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
        
