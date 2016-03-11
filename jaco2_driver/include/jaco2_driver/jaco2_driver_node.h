#ifndef JACO2_DRIVER_NODE_H
#define JACO2_DRIVER_NODE_H

//ROS
#include <ros/ros.h>
// JACO2 DRIVER
#include <jaco2_driver/jaco2_driver.h>
#include <kinova/KinovaTypes.h>
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

private:
    void jointVelocityCb(const jaco2_msgs::JointVelocityConstPtr& msg);
    void publishJointState();

private:
    ros::NodeHandle nh_;

    ros::NodeHandle private_nh_;

    Jaco2Driver controller_;

    ros::Subscriber subJointVelocity_;
    ros::Publisher pubJointState_;
    ros::Time last_command_;

    std::string tf_prefix_;
    sensor_msgs::JointState jointStateMsg_;

     double j6o_;
};
#endif // JACO2_DRIVER_NODE_H

