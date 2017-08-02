#ifndef VECTOR3STAMPED_H
#define VECTOR3STAMPED_H
#include <vector>
#include <Eigen/Dense>
#include <jaco2_data/time_stamp.h>

namespace jaco2_data {

class Vector3Stamped
{
public:
    Vector3Stamped();

    Vector3Stamped(double val);

    Vector3Stamped(double x, double y, double z);
    Vector3Stamped operator+(const Vector3Stamped &other) const;
    Vector3Stamped operator-(const Vector3Stamped &other) const;

    Vector3Stamped& operator+=(const Vector3Stamped &other);
    Vector3Stamped& operator-=(const Vector3Stamped &other);
    Vector3Stamped operator*(const double &b) const;

    Vector3Stamped abs() const;

    ///
    /// \brief operator * element wise multiplication
    /// \param other the other vector
    /// \return element wise product
    ///
    Vector3Stamped operator*(const Vector3Stamped &other) const;

    Vector3Stamped operator/(const double &b) const;

    Vector3Stamped operator*=(const double &b);
    Vector3Stamped operator/=(const double &b);

    double norm() const;
    std::vector<double> toVector() const;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TimeStamp stamp;
    std::string frame_id;
    Eigen::Vector3d vector;

};
}
#endif // VECTOR3STAMPED_H
