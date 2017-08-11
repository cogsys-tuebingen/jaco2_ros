#include <jaco2_data/joint_angles.h>

using namespace jaco2_data;

iterator JointAngles::begin();
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
