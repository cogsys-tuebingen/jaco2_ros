#include <string>
#include <ros/ros.h>
#include "../include/jaco2_kin_dyn/jaco2_kinematics_dynamics.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_jaco2_dynamics");
    ros::NodeHandle node("~");
    std::string robot_desc_string;
    node.getParam("/robot_description", robot_desc_string);//, std::string(""));
    Jaco2KinematicsDynamics jaco2KDL(robot_desc_string,"jaco_link_base","jaco_link_hand");

    return 0;
}
