#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import actionlib

import jaco2_msgs.msg
import sensor_msgs.msg

import sys, getopt


def pose_client(argv):
    print(argv)
    goal = jaco2_msgs.msg.ArmJointAnglesGoal()
    server_prefix = 'jaco_arm_driver'

    try:
        opts, args = getopt.getopt(argv,"hs:r:d:",["server=","degree=","radian="])
        #print(args)
    except getopt.GetoptError:
        print('angle_action_client.py -s <server_prefix>  -d/r "6 joint angles in degree/radian"')
        sys.exit(2)
    for opt, arg in opts:
        #print(opt,  arg)
        if opt == '-h':
            print('test.py -s <client_prefix>  -d/r <6 joint angles in degree/radian>')
            sys.exit()
            return 0
        elif opt in ("-s", "--server"):
            server_prefix = arg
            #print(arg)
        elif opt in ("-d", "--degree"):
            goal.type = jaco2_msgs.msg.ArmJointAnglesGoal.DEGREE
            joint_vals = arg.split()
            print("input: ", joint_vals, "| len(joint_vals)", len(joint_vals), "| arg:", arg)
        elif opt in ("-r", "--radian"):
            goal.type = jaco2_msgs.msg.ArmJointAnglesGoal.RADIAN
            #print(arg)
            joint_vals = arg.split()
            print len(arg)

    #print(joint_vals)

    server = server_prefix + '/arm_joint_angles'
    print('Connected to: ',server)
    client = actionlib.SimpleActionClient(server, jaco2_msgs.msg.ArmJointAnglesAction)

    if (len(joint_vals) < 6):
         goal.angles.joint1 = 4.8089
         goal.angles.joint2 = 2.9226
         goal.angles.joint3 = 1.0028
         goal.angles.joint4 = 4.2031
         goal.angles.joint5 = 1.4448
         goal.angles.joint6 = 1.3206
         goal.type = jaco2_msgs.msg.ArmJointAnglesGoal.RADIAN

         rospy.logwarn("Wrong number of arguments. Needed 6. Using test goal: \n%s", goal)
    else:

         goal.angles.joint1 = float(joint_vals[0])
         goal.angles.joint2 = float(joint_vals[1])
         goal.angles.joint3 = float(joint_vals[2])
         goal.angles.joint4 = float(joint_vals[3])
         goal.angles.joint5 = float(joint_vals[4])
         goal.angles.joint6 = float(joint_vals[5])

    print(goal)


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
        result = pose_client(sys.argv[1:])
        #rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException: 
        rospy.loginfo("Program interrupted before completion")
        
