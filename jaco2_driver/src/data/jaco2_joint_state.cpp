#include <jaco2_driver/data/jaco2_joint_state.h>
#include <jaco2_driver/data_conversion.h>
#include <jaco2_driver/data/jaco2_kinova_conversion.h>
using namespace jaco2_data;

Jaco2JointState::Jaco2JointState(std::string frameid)
    : frame_id(frameid),
      use_outlier_fiter_(true),
      gravity_(40),
      current_state_(ExtendedJointStateData(Jaco2DriverConstants::n_Jaco2Joints, Jaco2DriverConstants::n_Jaco2Joints))
{
    calibrate_acc_.resize(Jaco2DriverConstants::n_Jaco2Joints, false);
    current_state_.header.frame_id = frame_id;
}

void Jaco2JointState::useOutlierFilter(bool arg)
{
    use_outlier_fiter_ = arg;
}

void Jaco2JointState::setOutlierThreshold(double torque, double acc)
{
    filter_.threshold_torque = torque;
    filter_.threshold_acc = acc;
}

void Jaco2JointState::setAngularData(const jaco2_data::JointStateData::DataType type, const AngularPosition &pos)
{
    switch (type) {
    case JointStateData::DataType::JOINT_POS:{
        current_state_.data.joint_state.position = ConvertAngularData::kinova2data(pos);
//        current_state_.data.joint_state.normalize();
        current_state_.data.joint_state.normalize4Pi();
        break;
    }
    case JointStateData::DataType::JOINT_VEL:{
        current_state_.data.joint_state.velocity = ConvertAngularData::kinova2data(pos);
        break;
    }
    case JointStateData::DataType::JOINT_ACC:{
        current_state_.data.joint_state.acceleration = ConvertAngularData::kinova2data(pos);
        break;
    }
    case JointStateData::DataType::JOINT_TORQUE:{
        current_state_.data.joint_state.torque = ConvertAngularData::kinova2data(pos);
        break;
    }
    default:
        break;
    }
}

void Jaco2JointState::setLinearData(const AngularAcceleration& accs, const jaco2_data::TimeStamp& stamp)
{
    current_state_.data.lin_acc = ConvertAccelerometers::kinova2data(accs, stamp);
    applyCalibration();
}

jaco2_data::JointStateDataStamped Jaco2JointState::getJointState() const
{
    JointStateDataStamped res(current_state_.data.joint_state);
    res.header = current_state_.header;
    return res;
}

jaco2_data::AccelerometerData Jaco2JointState::getLinearAccelerations() const
{
    return current_state_.data.lin_acc;
}

jaco2_data::ExtendedJointStateData Jaco2JointState::getExtJointState() const
{
    return current_state_.data;
}

AngularInfo Jaco2JointState::getAngularData(const jaco2_data::JointStateData::DataType type) const
{
    switch (type) {
    case JointStateData::DataType::JOINT_POS:
        return ConvertAngularData::data2AngularInfo(current_state_.data.joint_state.position);
        break;
    case JointStateData::DataType::JOINT_VEL:
        return ConvertAngularData::data2AngularInfo(current_state_.data.joint_state.velocity);
        break;
    case JointStateData::DataType::JOINT_ACC:
        return ConvertAngularData::data2AngularInfo(current_state_.data.joint_state.acceleration);
        break;
    case JointStateData::DataType::JOINT_TORQUE:
        return ConvertAngularData::data2AngularInfo(current_state_.data.joint_state.torque);
        break;
    }
}

void Jaco2JointState::setJointNames(const std::vector<std::string> &names)
{
    current_state_.data.joint_state.names = names;
}

void Jaco2JointState::setAccelerometerCalibration(std::vector<Jaco2Calibration::AccelerometerCalibrationParam> params)
{
    accCalibParam_.resize(Jaco2DriverConstants::n_Jaco2Joints);

    for(std::size_t i = 0; i < Jaco2DriverConstants::accel_names.size(); ++i) {
        Jaco2Calibration::AccelerometerCalibrationParam param;
        bool contains = Jaco2Calibration::getParam(Jaco2DriverConstants::accel_names[i],params,param);
        if(contains) {
            calibrate_acc_[i] = true;
            accCalibParam_[i] = param;
        }
        else {
            calibrate_acc_[i] = false;
        }
    }
}

void Jaco2JointState::update(const KinovaJointState& data)
{
    setLinearData(data.accelerometers,  data.acc_stamp);

    current_state_.stamp() = data.stamp;
    current_state_.data.joint_state.position = ConvertAngularData::kinova2data(data.position);
    current_state_.data.joint_state.velocity = ConvertAngularData::kinova2data(data.velocity);
    current_state_.data.joint_state.acceleration = ConvertAngularData::kinova2data(data.acceleration);
    current_state_.data.joint_state.torque = ConvertAngularData::kinova2data(data.torque, false);
//    current_state_.data.joint_state.normalize();
    current_state_.data.joint_state.normalize4Pi();

    if(use_outlier_fiter_){
        jaco2_data::ExtendedJointStateDataStamped filtered;
        filter_.filter(current_state_, filtered);
        current_state_ = filtered;
    }

    estimateG();

}

void Jaco2JointState::update(const jaco2_data::TimeStamp& t,
                          const AngularPosition& pos,
                          const AngularPosition& vel,
                          const AngularPosition& acc,
                          const AngularPosition& tor,
                          const AngularAcceleration& lacc)
{

    setLinearData(lacc, t);

    current_state_.header.stamp = t;
    current_state_.data.joint_state.position = ConvertAngularData::kinova2data(pos);
    current_state_.data.joint_state.velocity = ConvertAngularData::kinova2data(vel);
    current_state_.data.joint_state.acceleration = ConvertAngularData::kinova2data(acc);
    current_state_.data.joint_state.torque = ConvertAngularData::kinova2data(tor, false);
//    current_state_.data.joint_state.normalize();
    current_state_.data.joint_state.normalize4Pi();

    if(use_outlier_fiter_){
        jaco2_data::ExtendedJointStateDataStamped filtered;
        filter_.filter(current_state_, filtered);
        current_state_ = filtered;
    }

    estimateG();

}

void Jaco2JointState::estimateG()
{
    current_state_.data.joint_state.gravity = gravity_.update(current_state_.data);
}



void Jaco2JointState::applyCalibration()
{
    auto it_calib = calibrate_acc_.begin();
    auto it_data = current_state_.data.lin_acc.begin();
    auto it_param = accCalibParam_.begin();

    for(std::size_t i = 0; i < Jaco2DriverConstants::n_Jaco2Joints; ++i){
        if(*it_calib){
            Eigen::Vector3d acc_raw = it_data->data.toEigen();
            Eigen::Matrix3d mat = it_param->misalignment;
            Eigen::Vector3d acc_calib = mat*(acc_raw - it_param->bias);
            it_data->data.fromEigen(acc_calib);
        }
        ++it_calib;
        ++it_data;
        ++it_param;
    }
}
