#ifndef MANIPULATOR_INFO_H
#define MANIPULATOR_INFO_H
#include <cstddef>
#include <stdexcept>

class ManipulatorInfo
{
public:
    ManipulatorInfo() {
        for(std::size_t i = 0; i < length_; ++ i)
        {
            values_[i] = 0;
        }
    }

    ~ManipulatorInfo() {}

    const std::size_t length_ = 6;
    double& operator[](std::size_t idx)
    {
        if(idx < length_)
        {
            return values_[idx];
        }
        else
        {
            throw std::logic_error("Illegal Index ");
        }
    }
    const double& operator[](std::size_t idx) const
    {
        if(idx < length_)
        {
            return values_[idx];
        }
        else
        {
            throw std::logic_error("Illegal Index ");
        }
    }


private:
    double values_[6];
};
#endif // MANIPULATOR_INFO_H

