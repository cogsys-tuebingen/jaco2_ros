#include <jaco2_driver/data/jaco2_joint_state.h>
#include <jaco2_driver/data_conversion.h>
#include <jaco2_driver/data/jaco2_kinova_conversion.h>
using namespace jaco2_data;

Jaco2JointState::Jaco2JointState() :
    buffer_size_(10)
{

}

void Jaco2JointState::setAngularData(const AngularDataFields type, const AngularPosition &pos)
{
    switch (type) {
    case POS:{
        current_state_.joint_state.position = ConvertAngularData::kinova2data(pos);
        current_state_.joint_state.normalize();
        break;
    }
    case VEL:{
        current_state_.joint_state.velocity = ConvertAngularData::kinova2data(pos);
        break;
    }
    case ACC:{
        current_state_.joint_state.acceleration = ConvertAngularData::kinova2data(pos);
        break;
    }
    case TOR:{
        current_state_.joint_state.torque = ConvertAngularData::kinova2data(pos);
        break;
    }
    default:
        break;
    }
}

void Jaco2JointState::setLinearData(const AngularAcceleration& accs, const jaco2_data::TimeStamp& stamp)
{
    current_state_.lin_acc = ConvertAccelerometers::kinova2data(accs, stamp);
    estimateG( accs.Actuator1_Y, accs.Actuator1_X, accs.Actuator1_Z); // jaco_base_link is not accelerometer frame !
}


AngularInfo Jaco2JointState::getAngularData(const AngularDataFields type) const
{
    switch (type) {
    case POS:
        return ConvertAngularData::data2AngularInfo(current_state_.joint_state.position);
        break;
    case VEL:
        return ConvertAngularData::data2AngularInfo(current_state_.joint_state.velocity);
        break;
    case ACC:
        return ConvertAngularData::data2AngularInfo(current_state_.joint_state.acceleration);
        break;
    case TOR:
        return ConvertAngularData::data2AngularInfo(current_state_.joint_state.torque);
        break;
    }
}
std::vector<double> Jaco2JointState::getData(const AngularDataFields type, bool degrees) const
{

}

void Jaco2JointState::set(const KinovaJointState& data)
{
    TimeStamp stamp;
    stamp.stamp = data.acc_stamp;
    setLinearData(data.accelerometers, stamp);



    current_state_.joint_state.stamp.stamp = data.stamp;
    current_state_.joint_state.position = ConvertAngularData::kinova2data(data.position);
    current_state_.joint_state.velocity = ConvertAngularData::kinova2data(data.velocity);
    current_state_.joint_state.acceleration = ConvertAngularData::kinova2data(data.acceleration);
    current_state_.joint_state.torque = ConvertAngularData::kinova2data(data.torque);
    current_state_.joint_state.normalize();

}




void Jaco2JointState::estimateG(double x, double y, double z)
{
    Eigen::Vector3d gnew(x,y,z);
    g_buffer_.emplace_back(gnew);
    while(g_buffer_.size() > buffer_size_){
        g_buffer_.pop_front();
    }

    current_state_.joint_state.gravity = Eigen::Vector3d(0,0,0);

    for(auto v : g_buffer_){
        current_state_.joint_state.gravity += v;
    }

    current_state_.joint_state.gravity *= 9.81 / g_buffer_.size();

}
