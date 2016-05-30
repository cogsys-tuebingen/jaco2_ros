#ifndef JACO2_CALIBRATION_IO_HPP
#define JACO2_CALIBRATION_IO_HPP
#include <string>
#include <stdio.h>
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

void importAsciiData(std::string filename, std::vector<DynamicCalibrationSample>& samples, std::string delimiter = std::string(";"))
{
    samples.clear();

    std::string line;
    std::ifstream infile;
    double ts, d[24];
    double start = 0.0;

    infile.open ( filename );
    if ( infile.is_open() )
    {
      char format[266];
//      switch ( type )
//      {
//      case DATASET_COMMA_SEPARATED:
////          sprintf_s ( format,"%%lf, %%lf, %%lf, %%lf" );
//          sprintf ( format,"%%lf, %%lf, %%lf, %%lf" );
//        break;
//      case DATASET_SPACE_SEPARATED:
//      default:
////          sprintf_s ( format,"%%lf %%lf %%lf %%lf" );
//          sprintf ( format,"%%lf %%lf %%lf %%lf" );
//        break;
//      }

      int l = 0;
      while ( std::getline ( infile,line ) )
      {
//          int res = sscanf_s ( line.data(), format, &ts, &d[0], &d[1], &d[2] );
          int res = sscanf ( line.data(), format, &ts, &d[0], &d[1], &d[2], &d[3], &d[4], &d[5],        // pos
                                                       &d[6], &d[7], &d[8], &d[9], &d[10], &d[11],      // vel
                                                       &d[12], &d[13], &d[14], &d[15], &d[16], &d[17],  // acc
                                                       &d[18], &d[19], &d[20], &d[21], &d[22], &d[23]);
        if ( res != 4 )
        {
          std::cout<<"importAsciiData(): error importing data in line "<<l<<", exit"<< std::endl;
        }
        else
        {
            if(start == 0.0) {
                start = ts;
            }

            ts -= start;
//          ts /= unit;
          DynamicCalibrationSample sample;
          sample.time = ts;
          sample.jointPos = {d[0], d[1], d[2], d[3], d[4], d[5]};
          sample.jointVel = {d[6], d[7], d[8], d[9], d[10], d[11]};
          sample.jointAcc = {d[12], d[13], d[14], d[15], d[16], d[17]};
          sample.jointTorque = {d[18], d[19], d[20], d[21], d[22], d[23]};
          samples.push_back(sample);
//            std::cout <<  d[0]<<", "<< d[1]<<", " <<d[2]<<std::endl;
        }
        l++;
      }
      infile.close();
    }
}

}
#endif // JACO2_CALIBRATION_IO_HPP

