#ifndef GRAVITY_ESTIMATOR_H
#define GRAVITY_ESTIMATOR_H
#include <jaco2_data/extended_joint_state_data.h>
#include <deque>
class GravityEstimator
{
public:
    GravityEstimator(std::size_t buffer_size = 10);

    void setBufferSize(std::size_t buffer_size);

    Eigen::Vector3d update(const jaco2_data::ExtendedJointStateData& data);
    Eigen::Vector3d update(const jaco2_data::AccelerometerData& data);

private:
    std::size_t buffer_size_;
    std::deque<Eigen::Vector3d> g_buffer_;
};
#endif // GRAVITY_ESTIMATOR_H
