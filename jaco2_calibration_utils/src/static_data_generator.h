#ifndef STATIC_DATA_GENERATOR_H
#define STATIC_DATA_GENERATOR_H
#include <jaco2_data/extended_joint_state_data.h>
#include <jaco2_msgs/JointAngles.h>

class StaticDataGenerator
{
public:

    StaticDataGenerator();
    void generateData(std::size_t depth);

    void setUpperLimit(int id, double val){upper_limits_[id] = val;}
    void setLowerLimit(int id, double val){lower_limits_[id] = val;}


private:
    std::size_t depth_;
    std::size_t run_;
    std::size_t n_joints_;
    std::size_t steps_;
    std::vector<double> upper_limits_;
    std::vector<double> lower_limits_;
    static const std::size_t steps = 5;
    static const std::size_t n_joints = 6;
};

#endif // STATIC_DATA_GENERATOR_H
