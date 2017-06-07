#ifndef KINOVA_ARITHMETICS_HPP
#define KINOVA_ARITHMETICS_HPP
#include <kinova/KinovaTypes.h>
#include <vector>
namespace KinovaArithmetics {
inline AngularInfo operator+(const AngularInfo& rhs, const AngularInfo& lhs)
{
    AngularInfo result;
    result.InitStruct();
    result.Actuator1 = rhs.Actuator1 + lhs.Actuator1;
    result.Actuator2 = rhs.Actuator2 + lhs.Actuator2;
    result.Actuator3 = rhs.Actuator3 + lhs.Actuator3;
    result.Actuator4 = rhs.Actuator4 + lhs.Actuator4;
    result.Actuator5 = rhs.Actuator5 + lhs.Actuator5;
    result.Actuator6 = rhs.Actuator6 + lhs.Actuator6;
    return result;
}

inline AngularInfo operator+(const AngularInfo& rhs, const std::vector<double>& lhs)
{
    AngularInfo result;
    result.InitStruct();
    result.Actuator1 = rhs.Actuator1 + lhs[0];
    result.Actuator2 = rhs.Actuator2 + lhs[1];
    result.Actuator3 = rhs.Actuator3 + lhs[2];
    result.Actuator4 = rhs.Actuator4 + lhs[3];
    result.Actuator5 = rhs.Actuator5 + lhs[4];
    result.Actuator6 = rhs.Actuator6 + lhs[5];
    return result;
}

inline AngularInfo operator+(const std::vector<double>& rhs, const AngularInfo& lhs )
{
    AngularInfo result;
    result.InitStruct();
    result.Actuator1 = rhs[0] + lhs.Actuator1 ;
    result.Actuator2 = rhs[1] + lhs.Actuator2 ;
    result.Actuator3 = rhs[2] + lhs.Actuator3 ;
    result.Actuator4 = rhs[3] + lhs.Actuator4 ;
    result.Actuator5 = rhs[4] + lhs.Actuator5 ;
    result.Actuator6 = rhs[5] + lhs.Actuator6 ;
    return result;
}

inline AngularInfo operator-(const AngularInfo& rhs, const AngularInfo& lhs)
{
    AngularInfo result;
    result.InitStruct();
    result.Actuator1 = rhs.Actuator1 - lhs.Actuator1;
    result.Actuator2 = rhs.Actuator2 - lhs.Actuator2;
    result.Actuator3 = rhs.Actuator3 - lhs.Actuator3;
    result.Actuator4 = rhs.Actuator4 - lhs.Actuator4;
    result.Actuator5 = rhs.Actuator5 - lhs.Actuator5;
    result.Actuator6 = rhs.Actuator6 - lhs.Actuator6;
    return result;
}

inline AngularInfo operator-(const AngularInfo& rhs, const std::vector<double>& lhs)
{
    AngularInfo result;
    result.InitStruct();
    result.Actuator1 = rhs.Actuator1 - lhs[0];
    result.Actuator2 = rhs.Actuator2 - lhs[1];
    result.Actuator3 = rhs.Actuator3 - lhs[2];
    result.Actuator4 = rhs.Actuator4 - lhs[3];
    result.Actuator5 = rhs.Actuator5 - lhs[4];
    result.Actuator6 = rhs.Actuator6 - lhs[5];
    return result;
}

inline AngularInfo operator-(const std::vector<double>& rhs, const AngularInfo& lhs)
{
    AngularInfo result;
    result.InitStruct();
    result.Actuator1 = rhs[0] - lhs.Actuator1 ;
    result.Actuator2 = rhs[1] - lhs.Actuator2 ;
    result.Actuator3 = rhs[2] - lhs.Actuator3 ;
    result.Actuator4 = rhs[3] - lhs.Actuator4 ;
    result.Actuator5 = rhs[4] - lhs.Actuator5 ;
    result.Actuator6 = rhs[5] - lhs.Actuator6 ;
    return result;
}


inline AngularInfo operator*(const AngularInfo& rhs, const double& lhs)
{
    AngularInfo result;
    result.InitStruct();
    result.Actuator1 = rhs.Actuator1 * lhs;
    result.Actuator2 = rhs.Actuator2 * lhs;
    result.Actuator3 = rhs.Actuator3 * lhs;
    result.Actuator4 = rhs.Actuator4 * lhs;
    result.Actuator5 = rhs.Actuator5 * lhs;
    result.Actuator6 = rhs.Actuator6 * lhs;
    return result;
}

inline AngularInfo operator*(const AngularInfo& rhs, const AngularInfo& lhs)
{
    AngularInfo result;
    result.Actuator1 = rhs.Actuator1 * lhs.Actuator1;
    result.Actuator2 = rhs.Actuator2 * lhs.Actuator2;
    result.Actuator3 = rhs.Actuator3 * lhs.Actuator3;
    result.Actuator4 = rhs.Actuator4 * lhs.Actuator4;
    result.Actuator5 = rhs.Actuator5 * lhs.Actuator5;
    result.Actuator6 = rhs.Actuator6 * lhs.Actuator6;
    return result;
}


inline AngularInfo& operator+=(AngularInfo &rhs, const AngularInfo &lhs)
{
    rhs.Actuator1 = rhs.Actuator1 + lhs.Actuator1;
    rhs.Actuator2 = rhs.Actuator2 + lhs.Actuator2;
    rhs.Actuator3 = rhs.Actuator3 + lhs.Actuator3;
    rhs.Actuator4 = rhs.Actuator4 + lhs.Actuator4;
    rhs.Actuator5 = rhs.Actuator5 + lhs.Actuator5;
    rhs.Actuator6 = rhs.Actuator6 + lhs.Actuator6;
    return rhs;
}

inline AngularInfo operator*(const double& rhs, const AngularInfo& lhs)
{
    AngularInfo result;
    result.InitStruct();
    result.Actuator1 = lhs.Actuator1 * rhs;
    result.Actuator2 = lhs.Actuator2 * rhs;
    result.Actuator3 = lhs.Actuator3 * rhs;
    result.Actuator4 = lhs.Actuator4 * rhs;
    result.Actuator5 = lhs.Actuator5 * rhs;
    result.Actuator6 = lhs.Actuator6 * rhs;

    return result;
}

inline AngularInfo& operator*=(AngularInfo &rhs, const double &lhs)
{
    rhs.Actuator1 = rhs.Actuator1 * lhs;
    rhs.Actuator2 = rhs.Actuator2 * lhs;
    rhs.Actuator3 = rhs.Actuator3 * lhs;
    rhs.Actuator4 = rhs.Actuator4 * lhs;
    rhs.Actuator5 = rhs.Actuator5 * lhs;
    rhs.Actuator6 = rhs.Actuator6 * lhs;
    return rhs;
}


inline AngularInfo operator/(const AngularInfo& rhs, const double& lhs)
{
    AngularInfo result;
    result.InitStruct();
    result.Actuator1 = rhs.Actuator1 / lhs;
    result.Actuator2 = rhs.Actuator2 / lhs;
    result.Actuator3 = rhs.Actuator3 / lhs;
    result.Actuator4 = rhs.Actuator4 / lhs;
    result.Actuator5 = rhs.Actuator5 / lhs;
    result.Actuator6 = rhs.Actuator6 / lhs;
    return result;
}

inline AngularInfo& operator/=(AngularInfo &rhs, const double &lhs)
{
    rhs.Actuator1 = rhs.Actuator1 / lhs;
    rhs.Actuator2 = rhs.Actuator2 / lhs;
    rhs.Actuator3 = rhs.Actuator3 / lhs;
    rhs.Actuator4 = rhs.Actuator4 / lhs;
    rhs.Actuator5 = rhs.Actuator5 / lhs;
    rhs.Actuator6 = rhs.Actuator6 / lhs;
    return rhs;
}

inline double absSum(const AngularInfo& vals)
{
    double res = 0;
    res += std::abs(vals.Actuator1);
    res += std::abs(vals.Actuator2);
    res += std::abs(vals.Actuator3);
    res += std::abs(vals.Actuator4);
    res += std::abs(vals.Actuator5);
    res += std::abs(vals.Actuator6);
    return res;
}

inline double sum(const AngularInfo& vals)
{
    double res = 0;
    res += vals.Actuator1;
    res += vals.Actuator2;
    res += vals.Actuator3;
    res += vals.Actuator4;
    res += vals.Actuator5;
    res += vals.Actuator6;
    return res;
}

inline AngularInfo abs(const AngularInfo& val)
{
    AngularInfo res;
    res.Actuator1 = std::abs(val.Actuator1);
    res.Actuator2 = std::abs(val.Actuator2);
    res.Actuator3 = std::abs(val.Actuator3);
    res.Actuator4 = std::abs(val.Actuator4);
    res.Actuator5 = std::abs(val.Actuator5);
    res.Actuator6 = std::abs(val.Actuator6);
    return res;
}

}
#endif // KINOVA_ARITHMETICS_HPP
