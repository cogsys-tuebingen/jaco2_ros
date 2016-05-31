#ifndef ACCELERATION_SAMPLES_HPP
#define ACCELERATION_SAMPLES_HPP
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
struct AccelerationData {

    AccelerationData()
    {
        time = 0;
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
    }

    AccelerationData(double t, double x, double y, double z)
    {
        time = t;
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }

    std::string toString(const std::string delimiter = std::string(";"))
    {
        std::string res;

        res += std::to_string(time) + delimiter;
        for(std::size_t i = 0; i < dataLength; ++i)
        {
            res += std::to_string(data[i]) + delimiter;
        }
        return res;
    }

    const int dataLength = 3;
    double time;
    double data[3];
};

struct AccelerationSamples {

    AccelerationSamples()
    {

    }

    std::vector<AccelerationData> getSamples(const std::size_t& joint_index) const
    {
        if(joint_index < nJoints)
        {
            return samples[joint_index];
        }
        else
        {
            std::cerr << "joint index " << joint_index << "is invalid. Joint index <" << nJoints << "!" << std::endl;
        }
    }

    void push_back(const std::size_t& joint_index, const AccelerationData& value)
    {
        if(joint_index < nJoints)
        {
            samples[joint_index].push_back(value);
        }
        else
        {
            std::cerr << "joint index " << joint_index << "is invalid. Joint index <" << nJoints << "!" << std::endl;
        }
    }

    void save(std::string filename, std::string delimiter =";")
    {
        std::ofstream file(filename);
        std::size_t maxSize = 0;
        std::size_t iMax = 0;
        for(std::size_t i = 0; i < nJoints; ++i){
            if(samples[i].size() > maxSize)
            {
                maxSize = samples[i].size();
                iMax = i;
            }
        }

        for(std::size_t i = 0; i < maxSize; ++i)
        {
            std::string line = std::to_string(samples[iMax][i].time) + delimiter;
            for(std::size_t j = 0; j < nJoints; ++j){
                if(samples[j].size() > i){
                    line += samples[j][i].toString(delimiter);
                }
                else{
                    line += std::string(delimiter);
                }
            }
            file << line << std::endl;
        }
    }

    const std::size_t nJoints = 6;
    std::vector<AccelerationData> samples[6];
};

#endif // ACCELERATION_SAMPLES_HPP

