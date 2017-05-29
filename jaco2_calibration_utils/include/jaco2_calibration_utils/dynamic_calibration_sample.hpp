#ifndef DYNAMIC_CALIBRATION_SAMPLE_HPP
#define DYNAMIC_CALIBRATION_SAMPLE_HPP
#include <vector>
#include <Eigen/Core>
//#include <ceres/ceres.h>
namespace Jaco2Calibration {

struct DynamicCalibrationSample
{
    DynamicCalibrationSample()
    {
        jointPos.resize(length);
        jointVel.resize(length);
        jointAcc.resize(length);
        jointTorque.resize(length);
    }

    void set(std::size_t index, double pos, double vel, double acc, double torque)
    {
        if(index < length){
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
        if(pos.size() >= length && vel.size() >= length && acc.size() >=length &&  torque.size() >=length) {
            for(std::size_t i = 0; i < length; ++i)
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
        if(index < length){
            std::vector<double> result = {jointPos[index], jointVel[index], jointAcc[index], jointTorque[index]};
            return result;
        }
        else{
            std::cerr << "Index has to be greater than -1 and smaller than 6. Index given is" << index <<std::endl;
            return std::vector<double>();
        }
    }

    std::string toString(std::string delimiter = std::string(";")) const
    {
        std::string res;
        res += std::to_string(time) + delimiter;
        for(std::size_t j = 0; j < length; ++ j)
        {
            res += std::to_string(jointPos[j]) + delimiter;
        }
        for(std::size_t j = 0; j < length; ++ j)
        {
            res += std::to_string(jointVel[j]) + delimiter;
        }
        for(std::size_t j = 0; j < length; ++ j)
        {
            res += std::to_string(jointAcc[j]) + delimiter;
        }
        for(std::size_t j = 0; j < length; ++ j)
        {
            res += std::to_string(jointTorque[j]) + delimiter;
        }
        res += std::to_string(gravity(0)) + delimiter +
                std::to_string(gravity(1)) + delimiter +
                std::to_string(gravity(2)) + delimiter;
        return res;
    }

    DynamicCalibrationSample operator+(const DynamicCalibrationSample &other) const
    {
        DynamicCalibrationSample res;
        res.time = time;

        auto it_res = res.jointPos.begin();
        auto it_other = other.jointPos.begin();
        for(auto d : jointPos){
            *it_res = d + (*it_other);
            ++it_res;
            ++it_other;
        }

        it_res = res.jointVel.begin();
        it_other = other.jointVel.begin();
        for(auto d : jointVel){
            *it_res = d + (*it_other);
            ++it_res;
            ++it_other;
        }

        it_res = res.jointAcc.begin();
        it_other = other.jointAcc.begin();
        for(auto d : jointAcc){
            *it_res = d + (*it_other);
            ++it_res;
            ++it_other;
        }

        it_res = res.jointTorque.begin();
        it_other = other.jointTorque.begin();
        for(auto d :jointTorque){
            *it_res = d + (*it_other);
            ++it_res;
            ++it_other;
        }

        res.gravity = gravity + other.gravity;

        return res;
    }

    DynamicCalibrationSample operator*(const double &value) const
    {
        DynamicCalibrationSample res;
        res.time = time;

        auto it_res = res.jointPos.begin();
        for(auto d : jointPos){
            *it_res = d * value;
            ++it_res;
        }

        it_res = res.jointVel.begin();
        for(auto d : jointVel){
            *it_res = d * value;
            ++it_res;
        }

        it_res = res.jointAcc.begin();
        for(auto d : jointAcc){
            *it_res = d * value;
            ++it_res;
        }

        it_res = res.jointTorque.begin();
        for(auto d : jointTorque){
            *it_res = d * value;
            ++it_res;
        }

        res.gravity = gravity * value;

        return res;
    }

    DynamicCalibrationSample& operator+=(const DynamicCalibrationSample &other)
    {

        auto it = this->jointPos.begin();
        for(auto d : other.jointPos){
            *it += d;
            ++it;
        }

        it = this->jointVel.begin();
        for(auto d : other.jointVel){
            *it += d;
            ++it;

        }

        it = this->jointAcc.begin();
        for(auto d : other.jointAcc){
            *it += d;
            ++it;
        }

        it = this->jointTorque.begin();
        for(auto d : other.jointTorque){
            *it += d;
            ++it;
        }

        gravity += other.gravity;
        return *this;
    }

    DynamicCalibrationSample& operator*=(const double value)
    {

        for(auto it = jointPos.begin(); it > this->jointPos.end(); ++it){
            *it *= value;
        }

        for(auto it = jointVel.begin(); it > this->jointVel.end(); ++it){
            *it *= value;
        }

        for(auto it = jointAcc.begin(); it > this->jointAcc.end(); ++it){
            *it *= value;
        }

        for(auto it = jointTorque.begin(); it > this->jointTorque.end(); ++it){
            *it *= value;
        }

        gravity *= value;
        return *this;
    }



    const static std::size_t length = 6;
    double time;
    std::vector<double> jointPos;
    std::vector<double> jointVel;
    std::vector<double> jointAcc;
    std::vector<double> jointTorque;
    Eigen::Vector3d gravity;
};
}
#endif // DYNAMIC_CALIBRATION_SAMPLE_HPP
