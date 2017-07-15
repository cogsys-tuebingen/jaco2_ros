#ifndef JOINT_STATE_OUTLIER_FILTER_H
#define JOINT_STATE_OUTLIER_FILTER_H
#include <deque>
#include <jaco2_data/extended_joint_state_data.h>



class JointStateOutlierFilter
{
public:
    JointStateOutlierFilter(double threshold_torque = 1e6, double threshold_acc = 65);

    bool filter();

private:
    bool doFiltering();
    void removeJsOutlier(std::size_t i, std::size_t j, std::size_t outlier);
    void removeAccOutlier(std::size_t i, std::size_t j, std::size_t outlier);
private:
    std::size_t buffer_size_;
    double threshold_torque_;
    double threshold_acc_;
    std::deque<jaco2_data::ExtendedJointStateData> jstate_buffer_;

};

#endif // JOINT_STATE_OUTLIER_FILTER_H
