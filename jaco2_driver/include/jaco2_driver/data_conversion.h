#ifndef DATA_CONVERSION_H
#define DATA_CONVERSION_H
//System
#include <vector>
#include <stdexcept>
//ROS
#include <trajectory_msgs/JointTrajectory.h>

//Kinova
#include <kinova/KinovaTypes.h>
#include <jaco2_msgs/JointAngles.h>
#include "joint_trajectory.h"

namespace DataConversion {

void convert(const AngularPosition &in, std::vector<double> &out)
{
    out.resize(Jaco2DriverNode::JACO_JOINTS_COUNT);
    out[0] = in.Actuators.Actuator1;
    out[1] = in.Actuators.Actuator2;
    out[2] = in.Actuators.Actuator3;
    out[3] = in.Actuators.Actuator4;
    out[4] = in.Actuators.Actuator5;
    out[5] = in.Actuators.Actuator6;

    out[6] = in.Fingers.Finger1;
    out[7] = in.Fingers.Finger2;
    out[8] = in.Fingers.Finger3;
}

void convert(const std::vector<double> &in, AngularPosition &out)
{
    out.InitStruct();
    if(in.size() >= 6)
    {
        out.Actuators.Actuator1 = in[0];
        out.Actuators.Actuator2 = in[1];
        out.Actuators.Actuator3 = in[2];
        out.Actuators.Actuator4 = in[3];
        out.Actuators.Actuator5 = in[4];
        out.Actuators.Actuator6 = in[5];

   if(in.size() == 9)
   {
       out.Fingers.Finger1 = in[6];
       out.Fingers.Finger1 = in[7];
       out.Fingers.Finger1 = in[8];
   }
   if(in.size() == 8)
   {
       out.Fingers.Finger1 = in[6];
       out.Fingers.Finger1 = in[7];
   }
    }
    else
    {
        throw std::logic_error("Illegal Vector Size");
    }
}

void convert(const AngularPosition &in, jaco2_msgs::JointAngles &out)
{
    out.joint1 = in.Actuators.Actuator1;
    out.joint2 = in.Actuators.Actuator2;
    out.joint3 = in.Actuators.Actuator3;
    out.joint4 = in.Actuators.Actuator4;
    out.joint5 = in.Actuators.Actuator5;
    out.joint6 = in.Actuators.Actuator6;
}

void convert(const jaco2_msgs::JointAngles &in, AngularPosition &out)
{
    out.InitStruct();
    out.Actuators.Actuator1 = in.joint1;
    out.Actuators.Actuator2 = in.joint2;
    out.Actuators.Actuator3 = in.joint3;
    out.Actuators.Actuator4 = in.joint4;
    out.Actuators.Actuator5 = in.joint5;
    out.Actuators.Actuator6 = in.joint6;
}

void convert(const trajectory_msgs::JointTrajectory &in, JointTrajectory& out)
{
    out.setJointNames(in.joint_names);
    out.resize(in.points.size());

    for(std::size_t i = 0; i < in.points.size(); ++ i)
    {
        trajectory_msgs::JointTrajectoryPoint p = in.points[i];
        out.setTimeFromStart(i, p.time_from_start.toSec());

        for(std::size_t joints = 0; joints < p.positions.size(); ++joints )
        {
            out.setPosition(i, joints,p.positions[joints]);
            out.setVelocity(i, joints,p.velocities[joints]);
            out.setAcceleration(i, joints,p.accelerations[joints]);
        }
    }
}

}
#endif // DATA_CONVERSION_H

