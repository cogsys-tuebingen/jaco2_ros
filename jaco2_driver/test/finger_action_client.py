#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco2_driver')
import rospy

import actionlib

import jaco2_msgs.msg

import sys, getopt


def pose_client(argv):
    print(argv)
    goal = jaco2_msgs.msg.SetFingersPositionGoal()
    server_prefix = 'jaco_arm_driver'

    try:
        opts, args = getopt.getopt(argv,"hs:v:",["server=","values="])
        #print(args)
    except getopt.GetoptError:
        print('angle_action_client.py -s <server_prefix>  -v "<finger pos 1> <finger pos 2> <finger pos 3>"')
        sys.exit(2)
    for opt, arg in opts:
        print(opt,  arg)
        if opt == '-h':
            print('angle_action_client.py -s <server_prefix>  -v "<finger pos 1> <finger pos 2> <finger pos 3>"')
            sys.exit()
            return 0
        elif opt in ("-s", "--server"):
            server_prefix = arg
            #print(arg)
        elif opt in ("-v", "--values"):
            joint_vals = arg.split()
            print("input: ", joint_vals, "| len(joint_vals)", len(joint_vals), "| arg:", arg)

    server = server_prefix + '/finger_joint_angles'
    print('Connected to: ',server)
    client = actionlib.SimpleActionClient(server, jaco2_msgs.msg.SetFingersPositionAction)

    if (len(joint_vals) < 3):
         goal.fingers.finger1 = 4000
         goal.fingers.finger2 = 4000
         goal.fingers.finger3 = 4000
  
         rospy.logwarn("Wrong number of arguments. Needed 3. Using test goal: \n%s", goal)
    else:

         goal.fingers.finger1 = float(joint_vals[0])
         goal.fingers.finger2 = float(joint_vals[1])
         goal.fingers.finger3 = float(joint_vals[2])

    print(goal)

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
        result = pose_client(sys.argv[1:])
        rospy.loginfo("Result: \n%s", result)
    except rospy.ROSInterruptException: 
        rospy.loginfo("Program interrupted before completion")
        
