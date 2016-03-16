#ifndef JACO2_DRIVER_NODE_H
#define JACO2_DRIVER_NODE_H

//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
// JACO2 DRIVER
#include <jaco2_driver/jaco2_driver.h>
#include <kinova/KinovaTypes.h>
#include <jaco2_msgs/ArmJointAnglesAction.h>
#include <signal.h>

class Jaco2DriverNode
{
public:
    // Maximum number of joints on Jaco-like robots:
    static const int     JACO_JOINTS_COUNT = 9;
public:
    Jaco2DriverNode();
    void stop();
    void tick();

    static void convert(const AngularPosition& in, std::vector<double>& out);
    static void convert(const AngularPosition &in, jaco2_msgs::JointAngles &out);
    static void convert(const jaco2_msgs::JointAngles &in, AngularPosition &out);

private:
    void jointVelocityCb(const jaco2_msgs::JointVelocityConstPtr& msg);
    void publishJointState();
    void actionAngleGoalCb();

private:
    ros::NodeHandle nh_;

    ros::NodeHandle private_nh_;

    Jaco2Driver controller_;

    ros::Subscriber subJointVelocity_;
    ros::Publisher pubJointState_;
    actionlib::SimpleActionServer<jaco2_msgs::ArmJointAnglesAction> actionAngleServer_;

    ros::Time last_command_;

    std::string tf_prefix_;
    sensor_msgs::JointState jointStateMsg_;

//    control_msgs::FollowJointTrajectoryGoalConstPtr angularPosGoal_;

    bool actionAngleServerRunning_;
    bool actionAngleCmdSent_;
    AngularPosition angleCmd_;

    double j6o_;
};
#endif // JACO2_DRIVER_NODE_H

