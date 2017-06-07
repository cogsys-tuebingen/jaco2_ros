#ifndef MANIPULATOR_INFO_H
#define MANIPULATOR_INFO_H
#include <cstddef>
#include <stdexcept>
#include <vector>
#include <string>
#include <math.h>

#include <kinova/KinovaTypes.h>
#include <kinova/KinovaArithmetics.hpp>
#include <jaco2_driver/jaco2_driver_constants.h>
using namespace KinovaArithmetics;

class ManipulatorInfo
{
public:
    ManipulatorInfo() {
        values_.InitStruct();
    }

    ~ManipulatorInfo() {}

    const std::size_t length_ = Jaco2DriverConstants::n_Jaco2Joints;

    float& operator[](std::size_t idx)
    {
        if(idx < length_)
        {
            switch (idx) {
            case 0:
                return values_.Actuator1;
                break;
            case 1:
                return values_.Actuator2;
                break;
            case 2:
                return values_.Actuator3;
                break;
            case 3:
                return values_.Actuator4;
                break;
            case 4:
                return values_.Actuator5;
                break;
            case 5:
                return values_.Actuator6;
                break;
            }
        }
        else
        {
            throw std::logic_error("ManipulatorInfo Illegal Index ");
        }
    }

    const float& operator[](std::size_t idx) const
    {
        if(idx < length_)
        {
            switch (idx) {
            case 0:
                return values_.Actuator1;
                break;
            case 1:
                return values_.Actuator2;
                break;
            case 2:
                return values_.Actuator3;
                break;
            case 3:
                return values_.Actuator4;
                break;
            case 4:
                return values_.Actuator5;
                break;
            case 5:
                return values_.Actuator6;
                break;
            }
        }
        else
        {
            throw std::logic_error("ManipulatorInfo Illegal Index ");
        }
    }

    ManipulatorInfo& operator=(const ManipulatorInfo& arg) // copy/move constructor is called to construct arg
    {
        values_ = arg.values_;
        return *this;
    }

    ManipulatorInfo operator/(const double &b)
    {
        ManipulatorInfo res;
        res.values_ = values_ / b;
        return res;
    }

    ManipulatorInfo& operator/=(const double &b)
    {
        values_ /= b;
        return *this;
    }

    ManipulatorInfo operator*(const double &b) const
    {
        ManipulatorInfo res;
        res.values_ = values_ * b;
        return res;
    }

    ManipulatorInfo operator*(const ManipulatorInfo &other) const
    {
        ManipulatorInfo res;
        res.values_ = values_ *  other.values_;
        return res;
    }

    ManipulatorInfo& operator*=(const double &b)
    {
        values_ *= b;

        return *this;
    }

    ManipulatorInfo operator+(const ManipulatorInfo &other) const
    {
        ManipulatorInfo res;
        res.values_ = values_ +  other.values_;

        return res;
    }

    ManipulatorInfo operator-(const ManipulatorInfo &other) const
    {
        ManipulatorInfo res;

        res.values_ = values_ -  other.values_;

        return res;
    }

    ManipulatorInfo operator+(const AngularInfo &other) const
    {
        ManipulatorInfo res;
        res.values_ = values_ +  other;

        return res;
    }

    ManipulatorInfo operator-(const AngularInfo &other) const
    {
        ManipulatorInfo res;

        res.values_ = values_ -  other;

        return res;
    }

    ManipulatorInfo abs() const
    {
        ManipulatorInfo res;
        res.values_ = KinovaArithmetics::abs(values_);
        return res;
    }

    double getSum()
    {
        double res =  KinovaArithmetics::sum(values_);

        return res;
    }

    void normalizeAngleDegrees()
    {
        for(std::size_t i = 0; i < length_; ++i )
        {
            while((*this)[i] >= 180.0)
            {
                (*this)[i] -= 360.0;
            }
            while((*this)[i] < -180.0)
            {
                (*this)[i] += 360.0;
            }
        }
    }

    void normalizeAngleRadian()
    {
        for(std::size_t i = 0; i < length_; ++i )
        {
            while((*this)[i]>= M_PI)
            {
                (*this)[i] -= 2.0*M_PI;
            }
            while((*this)[i] < -M_PI)
            {
                (*this)[i] += 2.0*M_PI;
            }
        }
    }

    double max(std::size_t& id) const
    {
        double res = -INFINITY;
        id = 0;
        for(std::size_t j = 0; j < length_; ++ j)
        {
            if(res < (*this)[j])
            {
                res = (*this)[j];
                id = j;
            }
        }
        return res;
    }

    std::string toString(std::string delimiter = std::string(";")) const
    {
        std::string res;
        for(std::size_t j = 0; j < length_; ++ j)
        {
            res += std::to_string((*this)[j]) + delimiter;
        }
        return res;
    }

    AngularInfo toAngularInfo() const
    {
        return values_;
    }

    AngularInfo& getAngularInfo()
    {
        return values_;
    }

    void setFrom(const AngularInfo& in)
    {
        values_.Actuator1 = in.Actuator1;
        values_.Actuator2 = in.Actuator2;
        values_.Actuator3 = in.Actuator3;
        values_.Actuator4 = in.Actuator4;
        values_.Actuator5 = in.Actuator5;
        values_.Actuator6 = in.Actuator6;
    }

    static ManipulatorInfo mean(const std::vector<ManipulatorInfo>& vec)
    {
        ManipulatorInfo res;
        for(std::size_t i = 0; i <vec.size(); ++i)
        {
            for(std::size_t j = 0; j < vec[i].length_; ++ j)
            {
                res[j] += vec[i][j];
            }
        }
        res = res/vec.size();
        return res;
    }

    static ManipulatorInfo variance(const std::vector<ManipulatorInfo>& vec, const ManipulatorInfo& mean)
    {
        std::vector<ManipulatorInfo> tmp;
        tmp.resize(vec.size());
        for(std::size_t i = 0; i <vec.size(); ++i)
        {
            for(std::size_t j = 0; j < vec[i].length_; ++ j)
            {
                tmp[i][j] = mean[j] - vec[i][j];
                tmp[i][j] *= tmp[i][j];
            }
        }
        return ManipulatorInfo::sum(tmp)/(double(tmp.size()));
    }

    static ManipulatorInfo sum(const std::vector<ManipulatorInfo>& vec)
    {
        ManipulatorInfo res;
        for(std::size_t i = 0; i <vec.size(); ++i)
        {
            for(std::size_t j = 0; j < vec[i].length_; ++ j)
            {
                res[j] += vec[i][j];
            }
        }
        return res;
    }

    static  std::vector<ManipulatorInfo> abs(const std::vector<ManipulatorInfo>& vec)
    {
        std::vector<ManipulatorInfo> res;
        for(auto m : vec){
            res.emplace_back(m.abs());
        }
        return res;

    }

    static void max(const std::vector<ManipulatorInfo>& vec, ManipulatorInfo& max, std::vector<std::size_t>& id )
    {
        id.resize(max.length_);
        for(std::size_t i = 0; i <vec.size(); ++i)
        {
            for(std::size_t j = 0; j < vec[i].length_; ++ j)
            {
                if(max[j] < vec[i][j])
                {
                    max[j] = vec[i][j];
                    id[j] = i;
                }
            }
        }
    }

    static void max(const std::vector<ManipulatorInfo>& vec, ManipulatorInfo& max)
    {
        for(std::size_t i = 0; i <vec.size(); ++i)
        {
            for(std::size_t j = 0; j < vec[i].length_; ++ j)
            {
                if(max[j] < vec[i][j])
                {
                    max[j] = vec[i][j];
                }
            }
        }
    }

    static void min(const std::vector<ManipulatorInfo>& vec, ManipulatorInfo& min)
    {
        for(std::size_t j = 0; j < min.length_; ++ j){
            min[j] = INFINITY;
        }
        for(std::size_t i = 0; i <vec.size(); ++i)
        {
            for(std::size_t j = 0; j < vec[i].length_; ++ j)
            {
                if(min[j] > vec[i][j])
                {
                    min[j] = vec[i][j];
                }
            }
        }
    }

    static ManipulatorInfo elment_sqrt(const ManipulatorInfo& x)
    {
        ManipulatorInfo res;
        for(std::size_t i = 0; i < res.length_; ++ i)
        {
            double val = x[i];
            res[i] = sqrt(val);
        }
        return res;
    }


private:
    AngularInfo values_;
};
#endif // MANIPULATOR_INFO_H

