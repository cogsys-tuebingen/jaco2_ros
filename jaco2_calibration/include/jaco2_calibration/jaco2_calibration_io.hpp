#ifndef JACO2_CALIBRATION_IO_HPP
#define JACO2_CALIBRATION_IO_HPP
#include <string>
#include <vector>
#include <fstream>
#include <jaco2_calibration/dynamic_calibrated_parameters.hpp>
#include <jaco2_calibration/dynamic_calibration_sample.hpp>

namespace Jaco2CalibIO {

void save(std::string name, std::vector<DynamicCalibratedParameters>& params)
{
    std::ofstream file(name);
    for(DynamicCalibratedParameters param : params)
    {
        file << param.linkName << std::endl;
        file << "mass: \n" << std::to_string(param.mass) << std::endl;
        tf::Vector3 com = param.coM;
        file << "Center Of Mass: \n" << std::to_string(com.getX()) << " " << std::to_string(com.getY()) << " " << std::to_string(com.getZ()) << std::endl;
        tf::Matrix3x3 inertia = param.inertia;
        file << "Moment Inertia: \n";
        file << "Ixx = " << std::to_string(inertia.getRow(0).getX()) << std::endl;
        file << "Ixy = " << std::to_string(inertia.getRow(0).getY()) << std::endl;
        file << "Ixz = " << std::to_string(inertia.getRow(0).getZ()) << std::endl;
        file << "Iyy = " << std::to_string(inertia.getRow(1).getY()) << std::endl;
        file << "Iyz = " << std::to_string(inertia.getRow(1).getZ()) << std::endl;
        file << "Izz = " << std::to_string(inertia.getRow(2).getZ()) << std::endl;

    }
}

void save(std::string name, std::vector<DynamicCalibrationSample> samples, std::string delimiter = std::string(";"))
{
    std::ofstream file(name);
//    std::string delimiter(";");
    file << "time " << delimiter;
    for(int i = 0; i <samples[0].jointPos.size(); ++i)
    {
        file << "joint_pos_" << std::to_string(i) << delimiter;
    }
    for(int i = 0; i <samples[0].jointVel.size(); ++i)
    {
        file << "joint_vel_" << std::to_string(i) << delimiter;
    }
    for(int i = 0; i <samples[0].jointAcc.size(); ++i)
    {
        file << "joint_acc_" << std::to_string(i) << delimiter;
    }
    for(int i = 0; i <samples[0].jointTorque.size(); ++i)
    {
        file << "joint_torque_" << std::to_string(i) << delimiter;
    }
    file << std::endl;
    for(DynamicCalibrationSample sample : samples)
    {
        file << sample.toString(delimiter) << std::endl;
    }
}

}
#endif // JACO2_CALIBRATION_IO_HPP

