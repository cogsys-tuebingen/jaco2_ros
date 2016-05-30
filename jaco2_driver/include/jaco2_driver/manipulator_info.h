#ifndef MANIPULATOR_INFO_H
#define MANIPULATOR_INFO_H
#include <cstddef>
#include <stdexcept>
#include <vector>
#include <string>
#include <math.h>

#include <kinova/KinovaTypes.h>

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
            throw std::logic_error("ManipulatorInfo Illegal Index ");
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
            throw std::logic_error("ManipulatorInfo Illegal Index ");
        }
    }

    ManipulatorInfo& operator=(const ManipulatorInfo& arg) // copy/move constructor is called to construct arg
    {
        for(std::size_t i = 0; i < length_; ++ i)
        {
            values_[i] = arg.values_[i];
        }
    }
    ManipulatorInfo operator/(const double &b)
    {
        ManipulatorInfo res;
        for(std::size_t i = 0; i < length_; ++ i)
        {
            res.values_[i] = values_[i] / b;
        }
        return res;
    }

    ManipulatorInfo& operator/=(const double &b)
    {
        for(std::size_t i = 0; i < length_; ++ i)
        {
            values_[i] /= b;
        }
    }

    ManipulatorInfo operator*(const double &b)
    {
        ManipulatorInfo res;
        for(std::size_t i = 0; i < length_; ++ i)
        {
            res.values_[i] = values_[i] *  b;
        }
        return res;
    }

    ManipulatorInfo operator*(const ManipulatorInfo &other)
    {
        ManipulatorInfo res;
        for(std::size_t i = 0; i < length_; ++ i)
        {
            res.values_[i] = values_[i] *  other.values_[i];
        }
        return res;
    }



    ManipulatorInfo& operator*=(const double &b)
    {
        for(std::size_t i = 0; i < length_; ++ i)
        {
            values_[i] *= b;
        }
    }

    ManipulatorInfo operator+(const ManipulatorInfo &other)
    {
        ManipulatorInfo res;
        for(std::size_t i = 0; i < length_; ++ i)
        {
            res.values_[i] = values_[i] +  other[i];
        }
        return res;
    }

    ManipulatorInfo operator-(const ManipulatorInfo &other)
    {
        ManipulatorInfo res;
        for(std::size_t i = 0; i < length_; ++ i)
        {
            res.values_[i] = values_[i] -  other[i];
        }
        return res;
    }


    double getSum()
    {
        double res = 0;
        for(std::size_t i = 0; i < length_; ++ i)
        {
            res += values_[i];
        }
        return res;
    }

    void normalizeAngleDegrees()
    {
        for(std::size_t i = 0; i < length_; ++i )
        {
            while(values_[i] >= 180.0)
            {
                values_[i] -= 360.0;
            }
            while(values_[i] < -180.0)
            {
                values_[i] += 360.0;
            }
        }
    }

    void normalizeAngleRadian()
    {
        for(std::size_t i = 0; i < length_; ++i )
        {
            while(values_[i] >= M_PI)
            {
                values_[i] -= 2.0*M_PI;
            }
            while(values_[i] < -M_PI)
            {
                values_[i] += 2.0*M_PI;
            }
        }
    }

    double max(std::size_t& id) const
    {
        double res = -INFINITY;
        id = 0;
        for(std::size_t j = 0; j < length_; ++ j)
        {
            if(res < values_[j])
            {
                res = values_[j];
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
            res += std::to_string(values_[j]) + delimiter;
        }
        return res;
    }

    AngularInfo toAngularInfo() const
    {
        AngularInfo res;
        res.InitStruct();
        res.Actuator1 = values_[0];
        res.Actuator2 = values_[1];
        res.Actuator3 = values_[2];
        res.Actuator4 = values_[3];
        res.Actuator5 = values_[4];
        res.Actuator6 = values_[5];
        return res;
    }

    void setFrom(const AngularInfo& in)
    {
        values_[0] = in.Actuator1;
        values_[1] = in.Actuator2;
        values_[2] = in.Actuator3;
        values_[3] = in.Actuator4;
        values_[4] = in.Actuator5;
        values_[5] = in.Actuator6;
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
    double values_[6];
};
#endif // MANIPULATOR_INFO_H

