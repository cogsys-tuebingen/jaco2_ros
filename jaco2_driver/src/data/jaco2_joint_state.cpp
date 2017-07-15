#include <jaco2_driver/data/jaco2_joint_state.h>
#include <jaco2_driver/data_conversion.h>
Jaco2JointState::Jaco2JointState() :
    buffer_size_(10)
{

}

void Jaco2JointState::setAngularData(const AngularDataFields type, const AngularPosition &pos)
{
    switch (type) {
    case POS:{
        DataConversion::convert(pos, current_state_.position);
        DataConversion::from_degrees(current_state_.position);
        DataConversion::normalize(current_state_.position);
        break;
    }
    case VEL:{
        DataConversion::convert(pos, current_state_.velocity);
        DataConversion::from_degrees(current_state_.velocity);
        break;
    }
    case ACC:{
        DataConversion::convert(pos, current_state_.acceleration);
        DataConversion::from_degrees(current_state_.acceleration);
        break;
    }
    case TOR:{
        DataConversion::convert(pos, current_state_.torque);
        break;
    }
    default:
        break;
    }
}

void Jaco2JointState::setLinearData(const AngularAcceleration& accs)
{
    Eigen::Vector3d v0(accs.Actuator1_X, accs.Actuator1_Y, accs.Actuator1_Z);
    Eigen::Vector3d v1(accs.Actuator2_X, accs.Actuator2_Y, accs.Actuator2_Z);
    Eigen::Vector3d v2(accs.Actuator3_X, accs.Actuator3_Y, accs.Actuator3_Z);
    Eigen::Vector3d v3(accs.Actuator4_X, accs.Actuator4_Y, accs.Actuator4_Z);
    Eigen::Vector3d v4(accs.Actuator5_X, accs.Actuator5_Y, accs.Actuator5_Z);
    Eigen::Vector3d v5(accs.Actuator6_X, accs.Actuator6_Y, accs.Actuator6_Z);

    estimateG( accs.Actuator1_Y, accs.Actuator1_X, accs.Actuator1_Z); // jaco_base_link is not accelerometer frame !

}


AngularInfo Jaco2JointState::getAngularData(const AngularDataFields type) const
{

}
std::vector<double> Jaco2JointState::getData(const AngularDataFields type, bool degrees) const
{

}

void Jaco2JointState::set(KinovaJointState& data)
{
    setLinearData(data.accelerometers);

    current_state_.stamp.stamp = data.stamp;
//    DataConversion::convert(pos, current_state_.position);
//    DataConversion::from_degrees(current_state_.position);
//    DataConversion::normalize(current_state_.position);

//    DataConversion::convert(pos, current_state_.velocity);
//    DataConversion::from_degrees(current_state_.velocity);

//    DataConversion::convert(pos, current_state_.acceleration);
//    DataConversion::from_degrees(current_state_.acceleration);

//    DataConversion::convert(pos, current_state_.torque);

}




void Jaco2JointState::estimateG(double x, double y, double z)
{
    Eigen::Vector3d gnew(x,y,z);
    g_buffer_.emplace_back(gnew);
    while(g_buffer_.size() > buffer_size_){
        g_buffer_.pop_front();
    }

    current_state_.gravity = Eigen::Vector3d(0,0,0);

    for(auto v : g_buffer_){
        current_state_.gravity += v;
    }

    current_state_.gravity *= 9.81 / g_buffer_.size();

}
