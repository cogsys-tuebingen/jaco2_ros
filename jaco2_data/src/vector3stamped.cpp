#include <jaco2_data/vector3stamped.h>

using namespace jaco2_data;

Vector3Stamped::Vector3Stamped()
    : vector(0,0,0)
{}

Vector3Stamped::Vector3Stamped(double val):
    vector(val,val,val){}

Vector3Stamped::Vector3Stamped(double x, double y, double z):
    vector(x,y,z){}

Vector3Stamped Vector3Stamped::operator+(const Vector3Stamped &other) const
{
    Vector3Stamped res;
    res.frame_id = this->frame_id;
    res.stamp = this->stamp;
    res.vector = this->vector + other.vector;
    return res;
}

Vector3Stamped Vector3Stamped::operator-(const Vector3Stamped &other) const
{
    Vector3Stamped res;
    res.frame_id = this->frame_id;
    res.stamp = this->stamp;
    res.vector = this->vector - other.vector;
    return res;
}

Vector3Stamped& Vector3Stamped::operator+=(const Vector3Stamped &other)
{
    this->vector += other.vector;

    return *this;
}

Vector3Stamped& Vector3Stamped::operator-=(const Vector3Stamped &other)
{
    this->vector -= other.vector;
    return *this;
}

Vector3Stamped Vector3Stamped::operator*(const double &b) const
{
    Vector3Stamped res;
    res.frame_id = this->frame_id;
    res.vector = this->vector *  b;

    return res;
}

Vector3Stamped Vector3Stamped::abs() const
{
    Vector3Stamped res;
    res.frame_id = this->frame_id;
    res.stamp = this->stamp;

    res.vector = this->vector.array().abs();

    return res;
}

Vector3Stamped Vector3Stamped::operator*(const Vector3Stamped &other) const
{
    Vector3Stamped res;
    res.frame_id = this->frame_id;

    res.vector = this->vector.array() *  other.vector.array();

    return res;
}

Vector3Stamped Vector3Stamped::operator/(const double &b) const
{
    Vector3Stamped res;
    res.frame_id = this->frame_id;

    res.vector = this->vector /  b;

    return res;
}

Vector3Stamped Vector3Stamped::operator*=(const double &b)
{

    this->vector *=  b;

    return *this;
}

Vector3Stamped Vector3Stamped::operator/=(const double &b)
{

    this->vector /=  b;

    return *this;
}

double Vector3Stamped::norm() const
{
    return vector.norm();
}

std::vector<double> Vector3Stamped::toVector() const
{
    std::vector<double> res = {vector(0), vector(1), vector(2)};
    return res;

}

std::string Vector3Stamped::to_string(const std::string delimiter) const
{
    std::string res;

    for(std::size_t i = 0; i < 3; ++i)
    {
        res += std::to_string(vector[i]) + delimiter;
    }
    return res;
}
