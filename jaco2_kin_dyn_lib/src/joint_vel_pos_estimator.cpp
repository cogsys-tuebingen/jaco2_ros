#include <jaco2_kin_dyn_lib/joint_vel_pos_estimator.h>


std::vector<double> operator+(const std::vector<double>& v1, const std::vector<double>& v2)
{
    std::vector<double> result;
    result.resize(v1.size());
    if(v1.size() == v2.size()){
        auto it_res = result.begin();
        auto it_v2 = v2.begin();
        for(auto val : v1){
            *it_res = val + *it_v2;
            ++it_res;
            ++it_v2;
        }
    }
    return result;
}

std::vector<double>& operator+=(std::vector<double> &rhs, const std::vector<double> &lhs)
{
        auto it_rhs = rhs.begin();
        for(auto val : lhs){
            *it_rhs += val;
            ++it_rhs;
        }

    return rhs;
}

std::vector<double> operator*(const double& val, const std::vector<double>& v)
{
    std::vector<double> result;
    result.resize(v.size());
    auto it_res = result.begin();
    for(auto vi : v){
        *it_res = val * vi;
        ++it_res;
    }
    return result;
}
std::vector<double> operator*(const std::vector<double>& v, const double& val)
{
    std::vector<double> result;
    result.resize(v.size());
    auto it_res = result.begin();
    for(auto vi : v){
        *it_res = val * vi;
        ++it_res;
    }
    return result;
}



using namespace Jaco2KinDynLib;



JointVelPosEstimator::JointVelPosEstimator() :
    loaded_model_(false),
    initial_values(false)
{}

JointVelPosEstimator::JointVelPosEstimator(const std::string &robot_model, const std::string &chain_root, const std::string &chain_tip):
    model_(robot_model, chain_root, chain_tip),
    loaded_model_(true),
    initial_values(false)
{

}

void JointVelPosEstimator::setModel(const std::__cxx11::string &robot_model, const std::__cxx11::string &chain_root, const std::__cxx11::string &chain_tip)
{
    model_ = Jaco2DynamicModel(robot_model, chain_root, chain_tip);
    loaded_model_ = true;

}

void JointVelPosEstimator::setInitalValues(const IntegrationData &data)
{
    current_vel_ = data.vel;
    current_pos_ = data.pos;

    time_ = data.time;

    initial_values = true;
}



void JointVelPosEstimator::estimate(const IntegrationData &data)
{
    buffer_.push_back(data);

    if(buffer_.size() < 2 || !initial_values){
        return;
    }

    double h = buffer_.time(1) - buffer_.time(0);
    time_ += h;
    model_.getJointAcceleration(data.pos, data.vel, data.torques, current_acc_);
    current_vel_ = data.vel + 0.5 * h * current_acc_;
    current_pos_ = data.pos + h * current_vel_;
}

//void JointVelPosIntegrator::integrate(const IntegrationData &data)
//{
//    buffer_.push_back(data);

//    if(buffer_.size() < 2 || !initial_values){
//        return;
//    }

//    double h = buffer_.time(1) - buffer_.time(0);
//    time_ += h;
//    model_.getJointAcceleration(data.pos, data.vel, data.torques, current_acc_);
//    current_vel_ +=  h * current_acc_;
//    current_pos_ +=  h * current_vel_;
//}



std::vector<double> JointVelPosEstimator::getCurrentPosition() const
{
    return current_pos_;
}
std::vector<double> JointVelPosEstimator::getCurrentVelocity() const
{
    return current_vel_;
}

std::vector<double> JointVelPosEstimator::getCurrentAcceleration() const
{
    return current_acc_;
}


