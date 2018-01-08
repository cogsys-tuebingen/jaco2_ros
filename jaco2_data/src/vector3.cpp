#include <jaco2_data/vector3.h>

using namespace jaco2_data;

Vector3::Vector3()
    : vector(0,0,0)
{}

Vector3::Vector3(double val):
    vector(val,val,val){}

Vector3::Vector3(double x, double y, double z):
    vector(x,y,z){}


double& Vector3::operator ()(std::size_t i)
{
    return vector(i);
}

const double& Vector3::operator ()(std::size_t i) const
{
    return vector(i);
}

Vector3 Vector3::operator+(const Vector3 &other) const
{
    Vector3 res;
    res.vector = this->vector + other.vector;
    return res;
}

Vector3 Vector3::operator-(const Vector3 &other) const
{
    Vector3 res;
    res.vector = this->vector - other.vector;
    return res;
}

Vector3& Vector3::operator+=(const Vector3 &other)
{
    this->vector += other.vector;

    return *this;
}

Vector3& Vector3::operator-=(const Vector3 &other)
{
    this->vector -= other.vector;
    return *this;
}

Vector3 Vector3::operator*(const double &b) const
{
    Vector3 res;
    res.vector = this->vector *  b;

    return res;
}

Vector3 Vector3::abs() const
{
    Vector3 res;

    res.vector = this->vector.array().abs();

    return res;
}

Vector3 Vector3::operator*(const Vector3 &other) const
{
    Vector3 res;
    res.vector = this->vector.array() *  other.vector.array();

    return res;
}

Vector3 Vector3::operator/(const double &b) const
{
    Vector3 res;

    res.vector = this->vector /  b;

    return res;
}

Vector3 Vector3::operator*=(const double &b)
{

    this->vector *=  b;

    return *this;
}

Vector3 Vector3::operator/=(const double &b)
{

    this->vector /=  b;

    return *this;
}

double Vector3::norm() const
{
    return vector.norm();
}

std::vector<double> Vector3::toVector() const
{
    std::vector<double> res = {vector(0), vector(1), vector(2)};
    return res;

}

std::string Vector3::to_string(const std::string delimiter) const
{
    std::string res;

    for(std::size_t i = 0; i < 3; ++i)
    {
        res += std::to_string(vector[i]) + delimiter;
    }
    return res;
}
