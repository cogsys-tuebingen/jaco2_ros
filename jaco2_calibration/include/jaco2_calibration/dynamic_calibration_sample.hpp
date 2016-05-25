#ifndef DYNAMIC_CALIBRATION_SAMPLE_HPP
#define DYNAMIC_CALIBRATION_SAMPLE_HPP
#include <vector>
#include <ceres/ceres.h>
struct DynamicCalibrationSample
{
    DynamicCalibrationSample()
    {
        jointPos.resize(6);
        jointVel.resize(6);
        jointAcc.resize(6);
        jointTorque.resize(6);
    }

    DynamicCalibrationSample(const DynamicCalibrationSample& sample):
        jointPos(sample.jointPos),
        jointVel(sample.jointVel),
        jointAcc(sample.jointAcc),
        jointTorque(sample.jointTorque)
    {
    }

    void set(std::size_t index, double pos, double vel, double acc, double torque)
    {
        if(index > -1 && index < 6){
            jointPos[index] = pos;
            jointVel[index] = vel;
            jointAcc[index] = acc;
            jointTorque[index] = torque;
        }
        else{
            std::cerr << "Index has to be greater than -1 and smaller than 6. Index given is" << index <<std::endl;
        }
    }

    void set(const std::vector<double>& pos, const std::vector<double>& vel, const std::vector<double>& acc, const std::vector<double>& torque)
    {
        if(pos.size() >= 6 && vel.size() >= 6 && acc.size() >=6 &&  torque.size() >=6) {
            for(std::size_t i = 0; i < 6; ++i)
            {
                jointPos[i] = pos[i];
                jointVel[i] = vel[i];
                jointAcc[i] = acc[i];
                jointTorque[i] = torque[i];
            }
        }
        else{
            std::cerr << "Dimensions has to be greater than 6 or equal." <<std::endl;
        }
    }

    std::vector<double> getValues(std::size_t index)
    {
        if(index > -1 && index < 6){
            std::vector<double> result = {jointPos[index], jointVel[index], jointAcc[index], jointTorque[index]};
            return result;
        }
        else{
            std::cerr << "Index has to be greater than -1 and smaller than 6. Index given is" << index <<std::endl;
            return std::vector<double>();
        }
    }

    std::vector<double> jointPos;
    std::vector<double> jointVel;
    std::vector<double> jointAcc;
    std::vector<double> jointTorque;
};

#endif // DYNAMIC_CALIBRATION_SAMPLE_HPP
