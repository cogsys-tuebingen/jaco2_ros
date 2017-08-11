#ifndef JOINT_ANGLES_H
#define JOINT_ANGLES_H
#include <vector>
#include "time_stamp.h"
namespace jaco2_data {

class JointAngles
{
public:
    typedef std::vector<double>::iterator iterator;
    typedef std::vector<double>::const_iterator const_iterator;
public:
    JointAngles() {}

    iterator begin();
    const_iterator begin() const;

    iterator end();
    const_iterator end() const;

    double& at(std::size_t i);
    const double& at(std::size_t i) const;

    double& operator[](std::size_t i);
    const double& operator [](std::size_t i) const;

    double& front();
    const double& front() const;

    double& back();
    const double& back() const;

    std::size_t size() const;
    void resize(std::size_t n, double val = 0);

    void emplace_back(double&& val);
    void push_back(const double& val);


public:
    TimeStamp stamp;
    std::vector<double> angles;
};
}
#endif // JOINT_ANGLES_H
