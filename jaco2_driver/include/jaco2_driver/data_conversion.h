#ifndef DATA_CONVERSION_H
#define DATA_CONVERSION_H
//System
#include <vector>
#include <stdexcept>
#include <chrono>
//ROS
#include <trajectory_msgs/JointTrajectory.h>
#include <angles/angles.h>
#include <geometry_msgs/Vector3Stamped.h>
//Kinova
#include <kinova/KinovaTypes.h>
#include <jaco2_msgs/JointAngles.h>
#include "joint_trajectory.h"
//Jaco2
#include "jaco2_driver_constants.h"


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
    //conversion encoder 2 degrees experimentaly determined
    out[6] = Jaco2DriverConstants::fingerAngleConversion*in.Fingers.Finger1;
    out[7] = Jaco2DriverConstants::fingerAngleConversion*in.Fingers.Finger2;
    out[8] = Jaco2DriverConstants::fingerAngleConversion*in.Fingers.Finger3;
}

void convert(const std::vector<double> &in, AngularPosition &out)
{
    out.InitStruct();
    if(in.size() >= Jaco2DriverConstants::n_Jaco2Joints)
    {
        out.Actuators.Actuator1 = in[0];
        out.Actuators.Actuator2 = in[1];
        out.Actuators.Actuator3 = in[2];
        out.Actuators.Actuator4 = in[3];
        out.Actuators.Actuator5 = in[4];
        out.Actuators.Actuator6 = in[5];

   if(in.size() == Jaco2DriverConstants::n_Jaco2JointsAndKG3)
   {
       out.Fingers.Finger1 = in[6];
       out.Fingers.Finger1 = in[7];
       out.Fingers.Finger1 = in[8];
   }
   if(in.size() == Jaco2DriverConstants::n_Jaco2JointsAndKG2)
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

void from_degrees(std::vector<double> &angles)
{
    for(std::size_t i = 0; i < angles.size(); ++i)
    {
        angles[i] = angles::from_degrees(angles[i]);
    }
}

void to_degrees(std::vector<double>& angles)
{
    for(std::size_t i = 0; i < angles.size(); ++i)
    {
        angles[i] = angles::to_degrees(angles[i]);
    }
}

void normalize(std::vector<double>& angles)
{
    auto it = angles.begin();
    for(std::size_t i = 0; i < 6; ++i){
        *it = angles::normalize_angle(*it);
        ++it;
    }
}

void from_degrees(AngularPosition &values)
{
    values.Actuators.Actuator1 = angles::from_degrees(values.Actuators.Actuator1);
    values.Actuators.Actuator2 = angles::from_degrees(values.Actuators.Actuator2);
    values.Actuators.Actuator3 = angles::from_degrees(values.Actuators.Actuator3);
    values.Actuators.Actuator4 = angles::from_degrees(values.Actuators.Actuator4);
    values.Actuators.Actuator5 = angles::from_degrees(values.Actuators.Actuator5);
    values.Actuators.Actuator6 = angles::from_degrees(values.Actuators.Actuator6);
    values.Fingers.Finger1 = angles::from_degrees(values.Fingers.Finger1);
    values.Fingers.Finger2 = angles::from_degrees(values.Fingers.Finger2);
    values.Fingers.Finger3 = angles::from_degrees(values.Fingers.Finger3);
}


void to_degrees(AngularPosition &values)
{
    values.Actuators.Actuator1 = angles::to_degrees(values.Actuators.Actuator1);
    values.Actuators.Actuator2 = angles::to_degrees(values.Actuators.Actuator2);
    values.Actuators.Actuator3 = angles::to_degrees(values.Actuators.Actuator3);
    values.Actuators.Actuator4 = angles::to_degrees(values.Actuators.Actuator4);
    values.Actuators.Actuator5 = angles::to_degrees(values.Actuators.Actuator5);
    values.Actuators.Actuator6 = angles::to_degrees(values.Actuators.Actuator6);
    values.Fingers.Finger1 = angles::to_degrees(values.Fingers.Finger1);
    values.Fingers.Finger2 = angles::to_degrees(values.Fingers.Finger2);
    values.Fingers.Finger3 = angles::to_degrees(values.Fingers.Finger3);
}

void convert(const AngularAcceleration & in, const std::chrono::time_point<std::chrono::high_resolution_clock>& stamp,  std::vector<geometry_msgs::Vector3Stamped>& out )
{
    geometry_msgs::Vector3Stamped accMsg;
    accMsg.header.stamp.fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count());
    accMsg.vector.x = in.Actuator1_X;
    accMsg.vector.y = in.Actuator1_Y;
    accMsg.vector.z = in.Actuator1_Z;
    accMsg.header.frame_id = Jaco2DriverConstants::name_accel_1;
    out[0] = accMsg;
    accMsg.vector.x = in.Actuator2_X;
    accMsg.vector.y = in.Actuator2_Y;
    accMsg.vector.z = in.Actuator2_Z;
    accMsg.header.frame_id = Jaco2DriverConstants::name_accel_2;
    out[1] = accMsg;
    accMsg.vector.x = in.Actuator3_X;
    accMsg.vector.y = in.Actuator3_Y;
    accMsg.vector.z = in.Actuator3_Z;
    accMsg.header.frame_id = Jaco2DriverConstants::name_accel_3;
    out[2] = accMsg;
    accMsg.vector.x = in.Actuator4_X;
    accMsg.vector.y = in.Actuator4_Y;
    accMsg.vector.z = in.Actuator4_Z;
    accMsg.header.frame_id = Jaco2DriverConstants::name_accel_4;
    out[3] = accMsg;
    accMsg.vector.x = in.Actuator5_X;
    accMsg.vector.y = in.Actuator5_Y;
    accMsg.vector.z = in.Actuator5_Z;
    accMsg.header.frame_id = Jaco2DriverConstants::name_accel_5;
    out[4] = accMsg;
    accMsg.vector.x = in.Actuator6_X;
    accMsg.vector.y = in.Actuator6_Y;
    accMsg.vector.z = in.Actuator6_Z;
    accMsg.header.frame_id = Jaco2DriverConstants::name_accel_6;
    out[5] = accMsg;
}

void convert(const AngularInfo& in, std::vector<double>& out)
{
    out.resize(Jaco2DriverConstants::n_Jaco2Joints);
    out[0] = in.Actuator1;
    out[1] = in.Actuator2;
    out[2] = in.Actuator3;
    out[3] = in.Actuator4;
    out[4] = in.Actuator5;
    out[5] = in.Actuator6;
}

void convert(const std::chrono::time_point<std::chrono::high_resolution_clock>& stamp_in, ros::Time& stamp_out)
{
   stamp_out.fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp_in.time_since_epoch()).count());
}

}
#endif // DATA_CONVERSION_H

