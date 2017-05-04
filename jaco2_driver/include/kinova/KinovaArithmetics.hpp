#ifndef KINOVA_ARITHMETICS_HPP
#define KINOVA_ARITHMETICS_HPP
#include <kinova/KinovaTypes.h>
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


}
#endif // KINOVA_ARITHMETICS_HPP
