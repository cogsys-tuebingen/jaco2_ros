#ifndef ACCELERATION_SAMPLES_HPP
#define ACCELERATION_SAMPLES_HPP
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <jaco2_data/vector3stamped.h>
#include <jaco2_data/types.h>
namespace Jaco2Calibration {

struct AccelerationSamples {

    AccelerationSamples()
    {

    }

    jaco2_data::Vector3StampedCollection getSamples(const std::size_t& joint_index) const
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

    void push_back(const std::size_t& joint_index, const jaco2_data::Vector3Stamped& value)
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
            std::string line = std::to_string(samples[iMax][i].stamp.toSec()) + delimiter;
            for(std::size_t j = 0; j < nJoints; ++j){
                if(samples[j].size() > i){
                    line += samples[j][i].to_string(delimiter);
                }
                else{
                    line += std::string(delimiter);
                }
            }
            file << line << std::endl;
        }
    }

    void clear()
    {
        for(std::size_t j = 0; j < nJoints; ++j){
            samples[j].clear();
        }
    }

    const std::size_t nJoints = 6;
    jaco2_data::Vector3StampedCollection samples[6];
};
}
#endif // ACCELERATION_SAMPLES_HPP

