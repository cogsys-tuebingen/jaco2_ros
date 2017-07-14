#ifndef JOINT_STATE_OUTLIER_FILTER_H
#define JOINT_STATE_OUTLIER_FILTER_H
#include <deque>
#include <jaco2_kin_dyn_lib/jaco2_kin_dyn_data_structs.h>


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
//    std::deque<JointState> jstate_buffer_;

};

#endif // JOINT_STATE_OUTLIER_FILTER_H
