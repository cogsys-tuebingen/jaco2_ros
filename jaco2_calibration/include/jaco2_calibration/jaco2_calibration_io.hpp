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
#include <jaco2_calibration/acceleration_samples.hpp>
#include <yaml-cpp/yaml.h>

namespace Jaco2Calibration{

void save(std::string name, const std::vector<DynamicCalibratedParameters>& params)
{
    std::ofstream file(name);
    YAML::Emitter yamlEmit(file);
    YAML::Node doc;
    for(DynamicCalibratedParameters param : params)
    {
        YAML::Node pNode;
        pNode["link_name"] = param.linkName;
        pNode["mass"] = param.mass;
//        file << param.linkName << std::endl;
//        file << "mass: \n" << std::to_string(param.mass) << std::endl;
        tf::Vector3 com = param.coM;
        pNode["com_x"] = com.getX();
        pNode["com_y"] = com.getY();
        pNode["com_z"] = com.getZ();
//        file << "Center Of Mass: \n" << std::to_string(com.getX()) << " " << std::to_string(com.getY()) << " " << std::to_string(com.getZ()) << std::endl;
        tf::Matrix3x3 inertia = param.inertia;
//        file << "Moment Inertia: \n";
//        file << "Ixx = " << std::to_string(inertia.getRow(0).getX()) << std::endl;
//        file << "Ixy = " << std::to_string(inertia.getRow(0).getY()) << std::endl;
//        file << "Ixz = " << std::to_string(inertia.getRow(0).getZ()) << std::endl;
//        file << "Iyy = " << std::to_string(inertia.getRow(1).getY()) << std::endl;
//        file << "Iyz = " << std::to_string(inertia.getRow(1).getZ()) << std::endl;
//        file << "Izz = " << std::to_string(inertia.getRow(2).getZ()) << std::endl;
        pNode["Ixx"] = inertia.getRow(0).getX();
        pNode["Ixy"] = inertia.getRow(0).getY();
        pNode["Ixz"] = inertia.getRow(0).getZ();
        pNode["Iyy"] = inertia.getRow(1).getY();
        pNode["Iyz"] = inertia.getRow(1).getZ();
        pNode["Izz"] = inertia.getRow(2).getZ();
        doc["parameter"].push_back(pNode);
    }
    yamlEmit << doc;

}


void loadDynParm(std::string filename, std::vector<DynamicCalibratedParameters>& params)
{
    YAML::Node doc = YAML::LoadFile(filename);
    doc = doc["parameter"];
    if(!doc.IsDefined())
    {
        throw std::runtime_error("illegal document!");
    }
    for(auto it = doc.begin(); it != doc.end(); ++it){
        auto pNode = *it;
        DynamicCalibratedParameters param;
        std::string name(pNode["link_name"].as<std::string>());
        param.linkName = name;
        param.mass = pNode["mass"].as<double>();
        double x = pNode["com_x"].as<double>();
        double y = pNode["com_y"].as<double>();
        double z = pNode["com_z"].as<double>();
        param.coM = tf::Vector3(x, y, z);
        double ixx = pNode["Ixx"].as<double>();
        double ixy = pNode["Ixy"].as<double>();
        double ixz = pNode["Ixz"].as<double>();
        double iyy = pNode["Iyy"].as<double>();
        double iyz = pNode["Iyz"].as<double>();
        double izz = pNode["Izz"].as<double>();
        param.inertia = tf::Matrix3x3(ixx, ixy, ixz,
                                      ixy, iyy, iyz,
                                      ixz, iyz, izz);
        params.push_back(param);
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

void importAsciiDataWithGravity(std::string filename, std::vector<DynamicCalibrationSample>& samples, const char delimiter = ';')
{
    samples.clear();

    std::string line;
    std::ifstream infile;

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
                if(i >= 25 && i<28){
                    sample.gravity(i-25) = val;
                }
                ++i;
            }

            samples.push_back(sample);

        }
        l++;
    }
    infile.close();
}

void importAsciiData(std::string filename, AccelerationSamples& samples, const char delimiter = ';')
{
    samples.clear();

    std::string line;
    std::ifstream infile;

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

            AccelerationData sample[samples.nJoints];
            int i = 0;
            while( ss.getline( value, 256, delimiter ))
            {
                double val = std::atof(value);
                if(i==0){
                    for(std::size_t i = 0; i < samples.nJoints; ++i){
                        sample[0].time = val;
                    }
                }
                if( i > 0 && i < 4){
                    sample[0].vector[i-1] = val;
                }
                if(i >= 4 && i<7){
                    sample[1].vector[i-4] = val;
                }
                if(i >= 7 && i<10){
                    sample[2].vector[i-7] = val;
                }
                if(i >= 10 && i<13){
                    sample[3].vector[i-10] = val;
                }
                if(i >= 13 && i<16){
                    sample[4].vector[i-13] = val;
                }
                if(i >= 16 && i<19){
                    sample[5].vector[i-16] = val;
                }
                ++i;
            }

            for(std::size_t i = 0; i < samples.nJoints; ++i){
                samples.push_back(i, sample[i]);
            }
        }
    }
    infile.close();
}

}
#endif // JACO2_CALIBRATION_IO_HPP

