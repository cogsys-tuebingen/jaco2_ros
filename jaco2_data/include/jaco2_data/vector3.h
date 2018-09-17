#ifndef VECTOR3STAMPED_H
#define VECTOR3STAMPED_H
#include <vector>
#include <Eigen/Dense>
#include <jaco2_data/time_stamp.h>

namespace jaco2_data {

class EIGEN_ALIGN16 Vector3
{
public:
    Vector3();

    Vector3(double val);

    Vector3(double x, double y, double z);
    Vector3 operator+(const Vector3 &other) const;
    Vector3 operator-(const Vector3 &other) const;

    Vector3& operator+=(const Vector3 &other);
    Vector3& operator-=(const Vector3 &other);
    Vector3 operator*(const double &b) const;

    Vector3 abs() const;

    double& operator ()(std::size_t i);
    const double& operator ()(std::size_t i) const;

    ///
    /// \brief operator * element wise multiplication
    /// \param other the other vector
    /// \return element wise product
    ///
    Vector3 operator*(const Vector3 &other) const;

    Vector3 operator/(const double &b) const;

    Vector3 operator*=(const double &b);
    Vector3 operator/=(const double &b);

    double norm() const;
    std::vector<double> toVector() const;
    std::string to_string(const std::string delimiter = std::string(";")) const;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d vector;

};
}
#endif // VECTOR3STAMPED_H
