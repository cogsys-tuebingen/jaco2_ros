#ifndef VECTOR3STAMPED_H
#define VECTOR3STAMPED_H
#include <vector>
#include <Eigen/Dense>
#include <jaco2_data/time_stamp.h>

namespace jaco2_data {

//class EIGEN_ALIGN16 Vector3
class Vector3
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
    Vector3& operator =(const Eigen::Vector3d &v);

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

    Vector3& operator*=(const double &b);
    Vector3& operator/=(const double &b);
    void zero();

    double norm() const;
    void normalize();
    std::vector<double> toVector() const;
    std::string to_string(const std::string delimiter = std::string(";")) const;
    Eigen::Vector3d toEigen() const;
    void fromEigen(const Eigen::Vector3d& v);

public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//    Eigen::Vector3d vector;
    std::array<double,3> vector;

};

inline Vector3 operator*(double val , const Vector3& v)
{
  return v * val;
}

}
#endif // VECTOR3STAMPED_H
