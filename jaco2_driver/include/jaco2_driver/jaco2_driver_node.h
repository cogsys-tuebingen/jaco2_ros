#ifndef JACO2_DRIVER_NODE_H
#define JACO2_DRIVER_NODE_H
//System
#include <signal.h>
//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <dynamic_reconfigure/server.h>
// JACO2 DRIVER
#include <jaco2_driver/jaco2_driver.h>
#include <kinova/KinovaTypes.h>
#include <jaco2_msgs/ArmJointAnglesAction.h>
#include <jaco2_driver/jaco2_driver_configureConfig.h>
#include <jaco2_driver/manipulator_info.h>


class Jaco2DriverNode
{
public:
    // Maximum number of joints on Jaco-like robots:
    static const int     JACO_JOINTS_COUNT = 9;
public:
    Jaco2DriverNode();
    void stop();
    void tick();

private:
    void jointVelocityCb(const jaco2_msgs::JointVelocityConstPtr& msg);
    void publishJointState();
    void publishJointAngles();
    void actionAngleGoalCb();
    void trajGoalCb();
    void gripperGoalCb();

    void dynamicReconfigureCb(jaco2_driver::jaco2_driver_configureConfig &config, uint32_t level);
private:
    ros::NodeHandle nh_;

    ros::NodeHandle private_nh_;

    Jaco2Driver controller_;

    ros::Subscriber subJointVelocity_;
    ros::Publisher pubJointState_;
    ros::Publisher pubJointAngles_;

    actionlib::SimpleActionServer<jaco2_msgs::ArmJointAnglesAction> actionAngleServer_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> trajServer_;
    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripperServer_;


    ros::Time last_command_;

    std::string tf_prefix_;
    sensor_msgs::JointState jointStateMsg_;
    jaco2_msgs::JointAngles jointAngleMsg_;

    bool actionAngleServerRunning_;
    bool trajServerRunning_;

    dynamic_reconfigure::Server<jaco2_driver::jaco2_driver_configureConfig> paramServer_;
    dynamic_reconfigure::Server<jaco2_driver::jaco2_driver_configureConfig>::CallbackType f_;


    double j6o_;
};
#endif // JACO2_DRIVER_NODE_H

