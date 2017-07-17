#ifndef JOINT_STATE_OUTLIER_FILTER_H
#define JOINT_STATE_OUTLIER_FILTER_H
#include <deque>
#include <vector>
#include <jaco2_data/extended_joint_state_data.h>



class JointStateOutlierFilter
{
public:
    JointStateOutlierFilter(double threshold_torque = 1e6, double threshold_acc = 65);

    bool filter(const jaco2_data::ExtendedJointStateData& data_in, jaco2_data::ExtendedJointStateData& out);

private:
    bool doFiltering();
    bool checkTorques(std::vector<bool>& test);
    bool checkAccs(std::vector<bool>& test);
    void removeOutlier(std::size_t i, std::size_t j, std::size_t outlier);
    void removeJsOutlier(std::size_t i, std::size_t j, std::size_t outlier);
    void removeAccOutlier(std::size_t i, std::size_t j, std::size_t outlier);
public:
    double threshold_torque;
    double threshold_acc;

private:
    std::size_t buffer_size_;
    std::deque<jaco2_data::ExtendedJointStateData> jstate_buffer_;

};

#endif // JOINT_STATE_OUTLIER_FILTER_H
