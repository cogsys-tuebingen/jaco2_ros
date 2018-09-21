#include <jaco2_driver/data/gravity_estimator.h>
using namespace jaco2_data;
GravityEstimator::GravityEstimator(std::size_t buffer_size)
    : static_base(false),
      buffer_size_(buffer_size)
{

}

void GravityEstimator::setBufferSize(std::size_t buffer_size)
{
    buffer_size_ = buffer_size;
}

Vector3 GravityEstimator::update(const jaco2_data::ExtendedJointStateData &data)
{
    return update(data.lin_acc);
}

jaco2_data::Vector3 GravityEstimator::update(const jaco2_data::AccelerometerData& data)
{
    const Vector3Stamped& a0 = data.front();
    jaco2_data::Vector3 gnew(-a0.data(1), -a0.data(0), -a0.data(2)); // jaco_base_link is not accelerometer frame !
    if(static_base){
        gnew.normalize();
    }

    g_buffer_.emplace_back(gnew);
    while(g_buffer_.size() > buffer_size_){
        g_buffer_.pop_front();
    }
    jaco2_data::Vector3 result;
    for(auto v : g_buffer_){
        result += v;
    }

    result *= 9.81 / (double) g_buffer_.size();

    return result;

}

