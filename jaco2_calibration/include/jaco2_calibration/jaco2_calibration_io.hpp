#ifndef JACO2_CALIBRATION_IO_HPP
#define JACO2_CALIBRATION_IO_HPP
//#include <string>
//#include <stdio.h>
//#include <vector>
//#include <fstream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
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

void importAsciiData(std::string filename, std::vector<DynamicCalibrationSample>& samples, const char delimiter = ';')
{
    samples.clear();

    std::string line;
    std::ifstream infile;
    double ts, d[24];
    double start = 0.0;

    infile.open ( filename );
    if ( infile.is_open() )
    {


        int l = 0;
        char value[256];

        std::getline ( infile,line );
        while ( std::getline ( infile,line ) )
        {

            std::stringstream ss;
            ss<< line ;

            DynamicCalibrationSample sample;
            int i = 0;
            while( ss.getline( value, 256, delimiter ))
            {
                double val = std::atof(value);
                if(i==0){
                    sample.time = val;
                }
                if( i > 0 && i < 7){
                    sample.jointPos[i-1] = val;
                }
                if(i >= 7 && i<13){
                    sample.jointVel[i-7] = val;
                }
                if(i >= 13 && i<19){
                    sample.jointAcc[i-13] = val;
                }
                if(i >= 19 && i<25){
                    sample.jointTorque[i-19] = val;
                }
                ++i;
            }

            samples.push_back(sample);

        }
        l++;
    }
    infile.close();
}

}
#endif // JACO2_CALIBRATION_IO_HPP

