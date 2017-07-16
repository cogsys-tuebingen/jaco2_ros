#include <jaco2_data/accelerometer_data.h>

using namespace jaco2_data;

AccelerometerData::AccelerometerData()
    : user_defined_label(-1)
{}

AccelerometerData::iterator AccelerometerData::AccelerometerData::begin()
{
    return lin_acc.begin();
}

AccelerometerData::const_iterator AccelerometerData::begin() const
{
    return lin_acc.begin();
}

AccelerometerData::iterator AccelerometerData::end()
{
    return lin_acc.end();
}

AccelerometerData::const_iterator AccelerometerData::end() const
{
    return lin_acc.end();
}


Vector3Stamped& AccelerometerData::at(std::size_t i)
{
    return lin_acc.at(i);
}

const Vector3Stamped& AccelerometerData::at(std::size_t i) const
{
    return lin_acc.at(i);
}

Vector3Stamped& AccelerometerData::operator[](std::size_t i)
{
    return lin_acc[i];
}

const Vector3Stamped& AccelerometerData::operator [](std::size_t i) const
{
    return lin_acc[i];
}

std::size_t AccelerometerData::size() const
{
    return lin_acc.size();
}

void AccelerometerData::resize(std::size_t n, Vector3Stamped val)
{
    lin_acc.resize(n, val);
}

Vector3Stamped& AccelerometerData::front()
{
    return lin_acc.front();
}

const Vector3Stamped& AccelerometerData::front() const
{
    return lin_acc.front();
}

Vector3Stamped& AccelerometerData::back()
{
    return lin_acc.back();
}

const Vector3Stamped& AccelerometerData::back() const
{
    return lin_acc.back();
}

void AccelerometerData::emplace_back(Vector3Stamped&& val)
{
    lin_acc.emplace_back(val);
}

void AccelerometerData::push_back(const Vector3Stamped &val)
{
    lin_acc.push_back(val);
}
