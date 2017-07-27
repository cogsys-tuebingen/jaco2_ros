#include <jaco2_data/joint_data.h>

using namespace jaco2_data;

JointData::iterator JointData::begin()
{
    return data.begin();
}

JointData::const_iterator JointData::begin() const
{
    return data.begin();
}

JointData::iterator JointData::end()
{
    return data.end();
}

JointData::const_iterator JointData::end() const
{
    return data.end();
}

double& JointData::at(std::size_t i)
{
    return data.at(i);
}

const double& JointData::at(std::size_t i) const
{
    return data.at(i);
}

double& JointData::operator[](std::size_t i)
{
    return data[i];
}

const double& JointData::operator [](std::size_t i) const
{
    return data[i];
}

double& JointData::front()
{
    return data.front();
}

const double& JointData::front() const
{
    return data.front();
}

double& JointData::back()
{
    return data.back();
}

const double& JointData::back() const
{
    return data.back();
}

std::size_t JointData::size() const
{
    return data.size();
}
void JointData::resize(std::size_t n, double val)
{
    data.resize(n,val);
}

void JointData::emplace_back(double&& val)
{
    data.emplace_back(val);
}

void JointData::push_back(const double& val)
{
    data.push_back(val);
}
