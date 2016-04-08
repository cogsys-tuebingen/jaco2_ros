#ifndef DATA_CONVERSION_H
#define DATA_CONVERSION_H
//System
#include <vector>
#include <stdexcept>
//ROS
#include <trajectory_msgs/JointTrajectory.h>
#include <angles/angles.h>

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
    //conversion encoder 2 degrees experimentaly determined
    out[6] = 9.2028e-3*in.Fingers.Finger1;
    out[7] = 9.2028e-3*in.Fingers.Finger2;
    out[8] = 9.2028e-3*in.Fingers.Finger3;
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

///
/// \brief shiftAngleDriverToDegrees converts angular positions into degrees and subtracts jaco angle offset. Driver uses this data format.
/// \param radians angles in radians
///
void shiftAngleDriverToDegrees(std::vector<double> &radians)
{
    radians[0] = angles::to_degrees(M_PI - radians[0]);
    radians[1] = angles::to_degrees(radians[1] + 1.5*M_PI);
    radians[2] = angles::to_degrees(M_PI_2 - radians[2]);
    radians[3] = angles::to_degrees(M_PI - radians[3]);
    radians[4] = angles::to_degrees(M_PI - radians[4]);
    radians[5] = angles::to_degrees(1.5*M_PI - radians[5]);
    if(radians.size() > 6)
    {
        radians[6] = angles::to_degrees(radians[6]);
        radians[7] = angles::to_degrees(radians[7]);
        radians[8] = angles::to_degrees(radians[8]);
    }
}

///
/// \brief shiftAngleToROSRadians converts angular positions into radias and shifts angles. MoveIt representation
/// \param degrees joint angles in degrees
///
void shiftAngleToROSRadians(std::vector<double> &degrees)
{
    degrees[0] = M_PI - angles::from_degrees(degrees[0]);
    degrees[1] = angles::from_degrees(degrees[1]) - 1.5*M_PI;
    degrees[2] = M_PI_2 - angles::from_degrees(degrees[2]);
    degrees[3] = M_PI - angles::from_degrees(degrees[3]);
    degrees[4] = M_PI - angles::from_degrees(degrees[4]);
    degrees[5] = 1.5*M_PI - angles::from_degrees(degrees[5]);
    if(degrees.size() > 6)
    {
        degrees[6] = angles::from_degrees(degrees[6]);
        degrees[7] = angles::from_degrees(degrees[7]);
        degrees[8] = angles::from_degrees(degrees[8]);
    }
}

///
/// \brief shiftAngleToROS shifts angles to MoveIt representation.
/// \param degrees joint angles in degrees
///
void shiftAngleToROS(std::vector<double> &degrees)
{
    degrees[0] = 360.0 - degrees[0];
    degrees[1] = degrees[1] - 270.0;
    degrees[2] = 90.0 - degrees[2];
    degrees[3] = 180.0 - degrees[3];
    degrees[4] = 180.0 - degrees[4];
    degrees[5] = 270.0 - degrees[5];
    if(degrees.size() > 6)
    {
        degrees[6] = (degrees[6]);
        degrees[7] = (degrees[7]);
        degrees[8] = (degrees[8]);
    }
}
void shiftAngleToROS(AngularPosition & degrees)
{
    degrees.Actuators.Actuator1 = 360.0 - degrees.Actuators.Actuator1;
    degrees.Actuators.Actuator2 = degrees.Actuators.Actuator2 - 270.0;
    degrees.Actuators.Actuator3 = 90.0 - degrees.Actuators.Actuator3;
    degrees.Actuators.Actuator4 = 360.0 - degrees.Actuators.Actuator4;
    degrees.Actuators.Actuator5 = 360.0 - degrees.Actuators.Actuator5;
    degrees.Actuators.Actuator6 = 270.0 - degrees.Actuators.Actuator6;

    degrees.Fingers.Finger1 = degrees.Fingers.Finger1;
    degrees.Fingers.Finger2 = degrees.Fingers.Finger2;
    degrees.Fingers.Finger3 = degrees.Fingers.Finger3;

}
///
/// \brief shiftAngleDriverToDegrees converts angular positions into degrees and subtracts jaco angle offset. Driver uses this data format.
/// \param radians angles in radians
///
void shiftAngleDriverToDegrees(AngularPosition &radians)
{
    radians.Actuators.Actuator1 = angles::to_degrees(M_PI - radians.Actuators.Actuator1);
    radians.Actuators.Actuator2 = angles::to_degrees(radians.Actuators.Actuator2 + 1.5*M_PI);
    radians.Actuators.Actuator3 = angles::to_degrees(M_PI_2 - radians.Actuators.Actuator3);
    radians.Actuators.Actuator4 = angles::to_degrees(M_PI - radians.Actuators.Actuator4);
    radians.Actuators.Actuator5 = angles::to_degrees(M_PI - radians.Actuators.Actuator5);
    radians.Actuators.Actuator6 = angles::to_degrees(1.5*M_PI - radians.Actuators.Actuator6);

    radians.Fingers.Finger1 = angles::to_degrees(radians.Fingers.Finger1);
    radians.Fingers.Finger2 = angles::to_degrees(radians.Fingers.Finger2);
    radians.Fingers.Finger3 = angles::to_degrees(radians.Fingers.Finger3);

}

void shiftAngleDriver(AngularPosition &degrees)
{
    degrees.Actuators.Actuator1 = 360.0 - degrees.Actuators.Actuator1;
    degrees.Actuators.Actuator2 = degrees.Actuators.Actuator2 + 270.0;
    degrees.Actuators.Actuator3 = 90.0 - degrees.Actuators.Actuator3;
    degrees.Actuators.Actuator4 = 360.0  - degrees.Actuators.Actuator4;
    degrees.Actuators.Actuator5 = 360.0  - degrees.Actuators.Actuator5;
    degrees.Actuators.Actuator6 = 270.0 - degrees.Actuators.Actuator6;

    degrees.Fingers.Finger1 = degrees.Fingers.Finger1;
    degrees.Fingers.Finger2 = degrees.Fingers.Finger2;
    degrees.Fingers.Finger3 = degrees.Fingers.Finger3;

}
///
/// \brief shiftAngleToROSRadians converts angular positions into radias and shifts angles. MoveIt representation
/// \param degrees joint angles in degrees
///
void shiftAngleToROSRadians(AngularPosition &degrees)
{
    degrees.Actuators.Actuator1 = M_PI - angles::from_degrees(degrees.Actuators.Actuator1);
    degrees.Actuators.Actuator2 = angles::from_degrees(degrees.Actuators.Actuator2) - 1.5*M_PI;
    degrees.Actuators.Actuator3 = M_PI_2 - angles::from_degrees(degrees.Actuators.Actuator3);
    degrees.Actuators.Actuator4 = M_PI - angles::from_degrees(degrees.Actuators.Actuator4);
    degrees.Actuators.Actuator5 = M_PI - angles::from_degrees(degrees.Actuators.Actuator5);
    degrees.Actuators.Actuator6 = 1.5*M_PI - angles::from_degrees(degrees.Actuators.Actuator6);

    degrees.Fingers.Finger1 = angles::from_degrees(degrees.Fingers.Finger1);
    degrees.Fingers.Finger2 = angles::from_degrees(degrees.Fingers.Finger2);
    degrees.Fingers.Finger3 = angles::from_degrees(degrees.Fingers.Finger3);
}

void transformVelAndAcc(std::vector<double> & vector)
{
    vector[0] *= -1.0;
    // vector[1] = vector[1]; !!
    vector[2] *= -1.0;
    vector[3] *= -1.0;
    vector[4] *= -1.0;
    vector[5] *= -1.0;
}

void transformVelAndAccToDegrees(std::vector<double>& vector)
{
    DataConversion::to_degrees(vector);
    transformVelAndAcc(vector);
}

void transformVelAndAccToRadian(std::vector<double>& vector)
{
    from_degrees(vector);
    transformVelAndAcc(vector);
}
void transformVelAndAcc(AngularPosition & vector)
{
    vector.Actuators.Actuator1 *= -1.0;
    // vector.Actuators.Actuator2  = vector.Actuators.Actuator2; !!
//    vector.Actuators.Actuator2 *= -1.0;
    vector.Actuators.Actuator3 *= -1.0;
    vector.Actuators.Actuator4 *= -1.0;
    vector.Actuators.Actuator5 *= -1.0;
    vector.Actuators.Actuator6 *= -1.0;
}

void transformVelAndAccToDegrees(AngularPosition & vector)
{
    to_degrees(vector);
    transformVelAndAcc(vector);
}

void transformVelAndAccToRadian(AngularPosition& vector)
{
    from_degrees(vector);
    transformVelAndAcc(vector);
}



}
#endif // DATA_CONVERSION_H

