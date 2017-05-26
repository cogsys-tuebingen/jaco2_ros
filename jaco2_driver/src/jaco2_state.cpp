#include <jaco2_driver/jaco2_state.h>
#include <jaco2_driver/jaco2_driver_constants.h>

Jaco2State::Jaco2State(Jaco2API &api)
    : api_(api),
      readCmd_(0),
      readCmdLowPri_(0),
      readCmdHighPri_(0),
      priortyThreshold_(2),
      priortyRate_(priortyThreshold_ + 1),
      calibrate_torque_(false),
      calibrate_torque_fkt_(false)
{
    lowPriority_.push_back(READ_QUICK_STATUS);
    lowPriority_.push_back(READ_CURRENT);
    lowPriority_.push_back(READ_ACCELRATION);
    lowPriority_.push_back(READ_TORQUE_GRAVITY_FREE);
    lowPriority_.push_back(READ_SENSOR_INFO);

    highPriority_.push_back(READ_POSITION);
    highPriority_.push_back(READ_VELOCITY);
    highPriority_.push_back(READ_TORQUE);

    current_position_.InitStruct();
    current_velocity_.InitStruct();
    current_torque_.InitStruct();
    current_current_.InitStruct();
    current_acceleration_.InitStruct();
    current_torque_gravity_free_.InitStruct();

    calibrate_acc_ = {false, false, false, false, false, false};
    velocityFactors_ = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    acc_counter_ =0;
}

AngularPosition Jaco2State::getAngularPosition() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return current_position_;
}

AngularPosition Jaco2State::getAngularVelocity() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return current_velocity_;
}

AngularPosition Jaco2State::getAngularAcceleration() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return current_joint_acceleration_;
}

AngularPosition Jaco2State::getAngularForce() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return current_torque_;
}

AngularPosition Jaco2State::getAngularCurrent() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return current_current_;
}

AngularPosition Jaco2State::getTorqueGFree() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return current_torque_gravity_free_;
}

AngularAcceleration Jaco2State::getLinearAcceleration() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return current_acceleration_;
}

QuickStatus Jaco2State::getQuickStatus() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return quick_status_;
}

SensorsInfo Jaco2State::getSensorInfo() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return sensor_info_;
}

void Jaco2State::readPosVelCur()
{
    switch (readCmd_) {
    case 0:{
        readPosition();
        break;
    }
    case 1:{
        readVelocity();
        break;
    }
    case 2:{
        readCurrent();
        break;
    }
    }

    readCmd_ = (readCmd_ + 1) % 3;
}


void Jaco2State::read()
{
    if(readCmd_ < priortyThreshold_) {
        for(int toread : highPriority_){
            read(toread);
        }
//        std::unique_lock<std::recursive_mutex> lock(data_mutex_);
//        readCmdHighPri_ = (readCmdHighPri_ + 1) % highPriority_.size();
    }
    else {
        for(int toread : lowPriority_){
            read(toread);
        }
//        read(lowPriority_[readCmdLowPri_]);
//        std::unique_lock<std::recursive_mutex> lock(data_mutex_);
//        readCmdLowPri_ =  (readCmdLowPri_ + 1) % lowPriority_.size();
    }

    readCmd_ = (readCmd_ + 1) % (priortyRate_);
}

std::vector<int> Jaco2State::getHighPriQue() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return highPriority_;
}

std::vector<int> Jaco2State::getLowPriQue() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return lowPriority_;
}

void Jaco2State::setHighPriQue(std::vector<int> que)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    highPriority_ = que;
}

void Jaco2State::setLowPriQue(std::vector<int> que)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    lowPriority_ = que;
}

void Jaco2State::setPriorityRate(int rate)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    priortyThreshold_ = rate -1;
    priortyRate_ = rate;
}

void Jaco2State::read(int dataID)
{
    switch (dataID) {
    case READ_POSITION:
        readPosition();
        break;
    case READ_VELOCITY:
        readVelocity();
        break;
    case READ_ACCELRATION:
        readAcceleration();
        break;
    case READ_TORQUE:
        readTorque();
        break;
    case READ_TORQUE_GRAVITY_FREE:
        readTorqueGravityFree();
        break;
    case READ_CURRENT:
        readCurrent();
        break;
    case READ_QUICK_STATUS:
        readQuickStatus();
        break;
    case READ_SENSOR_INFO:
        readSensorInfo();
        break;
    default:
        break;
    }
}

std::chrono::time_point<std::chrono::high_resolution_clock> Jaco2State::getLastUpdate(int read_data) const
{
    std::chrono::time_point<std::chrono::high_resolution_clock> res;
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    switch (read_data) {
    case READ_POSITION:
        res = time_position_;
        break;
    case READ_VELOCITY:
        res = time_velocity_;
        break;
    case READ_ACCELRATION:
        res = time_acceleration_;
        break;
    case READ_TORQUE:
        res = time_torque_;
        break;
    case READ_TORQUE_GRAVITY_FREE:
        res = time_torque_gravity_free_;
        break;
    case READ_CURRENT:
        res = time_current_;
        break;
    case READ_QUICK_STATUS:
        res = time_quick_status_;
        break;
    case READ_SENSOR_INFO:
        res = time_sensor_info_;
        break;
    default:
        break;
    }

    return res;

}

void Jaco2State::readPosVel()
{

    switch (readCmd_) {
    case 0: {
        readVelocity();
        break;
    }
    case 1: {
        readPosition();
        break;
    }
    }

    readCmd_ = (readCmd_ + 1) % 2;
}

void Jaco2State::readPosition()
{
    current_position_ = api_.getAngularPosition();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_position_ = std::chrono::high_resolution_clock::now();
}

void Jaco2State::readVelocity()
{
    lastVelocity_.push_back(current_velocity_);
    current_velocity_ = api_.getAngularVelocity();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    current_velocity_.Actuators.Actuator1 *= velocityFactors_[0];
    current_velocity_.Actuators.Actuator2 *= velocityFactors_[1];
    current_velocity_.Actuators.Actuator3 *= velocityFactors_[2];
    current_velocity_.Actuators.Actuator4 *= velocityFactors_[3];
    current_velocity_.Actuators.Actuator5 *= velocityFactors_[4];
    current_velocity_.Actuators.Actuator6 *= velocityFactors_[5];

    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now - time_velocity_ ;
    time_velocity_ = now;
    dt_.push_back(std::chrono::duration_cast<std::chrono::microseconds>(duration).count()*1e-6);

    while (dt_.size() > 2) {
        dt_.pop_front();
    }

    while(lastVelocity_.size() > 2){
        lastVelocity_.pop_front();
    }
//    acc_counter_ = (acc_counter_ + 1) % 2;
//    acc_counter_ = 0;
    calculateJointAcceleration();
}

void Jaco2State::readTorque()
{
    current_torque_ = api_.getAngularForce();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_torque_ = std::chrono::high_resolution_clock::now();
    applyTorqueOffsets();
}

void Jaco2State::readTorqueGravityFree()
{
    current_torque_gravity_free_ = api_.getAngularForceGravityFree();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_torque_gravity_free_ = std::chrono::high_resolution_clock::now();
    applyTorqueOffsets2TorqueGFree();
}

void Jaco2State::readAcceleration()
{
    current_acceleration_ =api_.getActuatorAcceleration();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_acceleration_ = std::chrono::high_resolution_clock::now();
    applyAccelerationCalibration();
}

void Jaco2State::readCurrent()
{
    current_current_ = api_.getAngularCurrent();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_current_ = std::chrono::high_resolution_clock::now();
}

void Jaco2State::readQuickStatus()
{
    quick_status_ = api_.getQuickStatus();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_quick_status_ = std::chrono::high_resolution_clock::now();
}

void Jaco2State::readSensorInfo()
{
    sensor_info_ = api_.getSensorInfo();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_sensor_info_ = std::chrono::high_resolution_clock::now();
}

void Jaco2State::setAccelerometerCalibration(const std::vector<Jaco2Calibration::AccelerometerCalibrationParam>& params)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
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

void Jaco2State::setTorqueCalibration(const Jaco2Calibration::TorqueOffsetLut &lut)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    torque_offset_ = lut;
    calibrate_torque_ = true;
    calibrate_torque_fkt_ = false;
}

void Jaco2State::setTorqueCalibration(const Jaco2Calibration::TorqueOffsetCalibration &fkt)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    torque_offest_fkt_ = fkt;
    calibrate_torque_ = false;
    calibrate_torque_fkt_  = true;
}


void Jaco2State::setVelocitySensorCalibration(const std::vector<double> &factors)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    velocityFactors_ = factors;
}

void Jaco2State::calculateJointAcceleration()
{
    double dtsum ;
    int i;
    if(dt_.size() == 2){
        dtsum = dt_[0] + dt_[1];
        i = 1;
    }
    else if(dt_.size() == 1)
    {
        i = 0;
        dtsum = dt_[0];
    }
    else{
        return;
    }
//    double dtsum = dt_[0];
//    int i = 0;
    current_joint_acceleration_.Actuators.Actuator1 = (current_velocity_.Actuators.Actuator1 - lastVelocity_[i].Actuators.Actuator1) / dtsum;
    current_joint_acceleration_.Actuators.Actuator2 = (current_velocity_.Actuators.Actuator2 - lastVelocity_[i].Actuators.Actuator2) / dtsum;
    current_joint_acceleration_.Actuators.Actuator3 = (current_velocity_.Actuators.Actuator3 - lastVelocity_[i].Actuators.Actuator3) / dtsum;
    current_joint_acceleration_.Actuators.Actuator4 = (current_velocity_.Actuators.Actuator4 - lastVelocity_[i].Actuators.Actuator4) / dtsum;
    current_joint_acceleration_.Actuators.Actuator5 = (current_velocity_.Actuators.Actuator5 - lastVelocity_[i].Actuators.Actuator5) / dtsum;
    current_joint_acceleration_.Actuators.Actuator6 = (current_velocity_.Actuators.Actuator6 - lastVelocity_[i].Actuators.Actuator6) / dtsum;
    current_joint_acceleration_.Fingers.Finger1 = (current_velocity_.Fingers.Finger1 - lastVelocity_[i].Fingers.Finger1) / dtsum;
    current_joint_acceleration_.Fingers.Finger2 = (current_velocity_.Fingers.Finger2 - lastVelocity_[i].Fingers.Finger2) / dtsum;
    current_joint_acceleration_.Fingers.Finger3 = (current_velocity_.Fingers.Finger3 - lastVelocity_[i].Fingers.Finger3) / dtsum;
}

void Jaco2State::applyAccelerationCalibration()
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    for(std::size_t i = 0; i < Jaco2DriverConstants::n_Jaco2Joints; ++i){
        if(calibrate_acc_[i]){
            Eigen::Vector3d acc_raw;
            getAcceleration(i,current_acceleration_, acc_raw);
            Eigen::Matrix3d mat = accCalibParam_[i].misalignment;
            Eigen::Vector3d acc_calib = mat*(acc_raw - accCalibParam_[i].bias);
            setAcceleration(i,acc_calib,current_acceleration_);
        }
    }
}

void Jaco2State::applyTorqueOffsets()
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    if(calibrate_torque_ && ! calibrate_torque_fkt_){
        current_torque_.Actuators.Actuator1 -= torque_offset_.at(1, current_position_.Actuators.Actuator1);
        current_torque_.Actuators.Actuator2 -= torque_offset_.at(2, current_position_.Actuators.Actuator2);
        current_torque_.Actuators.Actuator3 -= torque_offset_.at(3, current_position_.Actuators.Actuator3);
        current_torque_.Actuators.Actuator4 -= torque_offset_.at(4, current_position_.Actuators.Actuator4);
        current_torque_.Actuators.Actuator5 -= torque_offset_.at(5, current_position_.Actuators.Actuator5);
        current_torque_.Actuators.Actuator6 -= torque_offset_.at(6, current_position_.Actuators.Actuator6);
    }
    if(calibrate_torque_fkt_ && !calibrate_torque_){
        current_torque_.Actuators.Actuator1 -= torque_offest_fkt_(0, current_position_.Actuators.Actuator1);
        current_torque_.Actuators.Actuator2 -= torque_offest_fkt_(1, current_position_.Actuators.Actuator2);
        current_torque_.Actuators.Actuator3 -= torque_offest_fkt_(2, current_position_.Actuators.Actuator3);
        current_torque_.Actuators.Actuator4 -= torque_offest_fkt_(3, current_position_.Actuators.Actuator4);
        current_torque_.Actuators.Actuator5 -= torque_offest_fkt_(4, current_position_.Actuators.Actuator5);
        current_torque_.Actuators.Actuator6 -= torque_offest_fkt_(5, current_position_.Actuators.Actuator6);
    }

}

void Jaco2State::applyTorqueOffsets2TorqueGFree()
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    if(calibrate_torque_ && ! calibrate_torque_fkt_){
        current_torque_gravity_free_.Actuators.Actuator1 -= torque_offset_.at(1, current_position_.Actuators.Actuator1);
        current_torque_gravity_free_.Actuators.Actuator2 -= torque_offset_.at(2, current_position_.Actuators.Actuator2);
        current_torque_gravity_free_.Actuators.Actuator3 -= torque_offset_.at(3, current_position_.Actuators.Actuator3);
        current_torque_gravity_free_.Actuators.Actuator4 -= torque_offset_.at(4, current_position_.Actuators.Actuator4);
        current_torque_gravity_free_.Actuators.Actuator5 -= torque_offset_.at(5, current_position_.Actuators.Actuator5);
        current_torque_gravity_free_.Actuators.Actuator6 -= torque_offset_.at(6, current_position_.Actuators.Actuator6);
    }
    if(calibrate_torque_fkt_ && !calibrate_torque_){
        current_torque_gravity_free_.Actuators.Actuator1 -= torque_offest_fkt_(0, current_position_.Actuators.Actuator1);
        current_torque_gravity_free_.Actuators.Actuator2 -= torque_offest_fkt_(1, current_position_.Actuators.Actuator2);
        current_torque_gravity_free_.Actuators.Actuator3 -= torque_offest_fkt_(2, current_position_.Actuators.Actuator3);
        current_torque_gravity_free_.Actuators.Actuator4 -= torque_offest_fkt_(3, current_position_.Actuators.Actuator4);
        current_torque_gravity_free_.Actuators.Actuator5 -= torque_offest_fkt_(4, current_position_.Actuators.Actuator5);
        current_torque_gravity_free_.Actuators.Actuator6 -= torque_offest_fkt_(5, current_position_.Actuators.Actuator6);
    }
}

void Jaco2State::getAcceleration(const std::size_t &index, const AngularAcceleration &acc, Eigen::Vector3d &vec)
{
    switch (index) {
    case 0: {
        vec(0) = acc.Actuator1_X;
        vec(1) = acc.Actuator1_Y;
        vec(2) = acc.Actuator1_Z;
        break;
    }
    case 1: {
        vec(0) = acc.Actuator2_X;
        vec(1) = acc.Actuator2_Y;
        vec(2) = acc.Actuator2_Z;
        break;
    }
    case 2: {
        vec(0) = acc.Actuator3_X;
        vec(1) = acc.Actuator3_Y;
        vec(2) = acc.Actuator3_Z;
        break;
    }
    case 3: {
        vec(0) = acc.Actuator4_X;
        vec(1) = acc.Actuator4_Y;
        vec(2) = acc.Actuator4_Z;
        break;
    }
    case 4: {
        vec(0) = acc.Actuator5_X;
        vec(1) = acc.Actuator5_Y;
        vec(2) = acc.Actuator5_Z;
        break;
    }
    case 5: {
        vec(0) = acc.Actuator6_X;
        vec(1) = acc.Actuator6_Y;
        vec(2) = acc.Actuator6_Z;
        break;
    }
    default: {
        throw std::runtime_error("illegal index!");
        break;
    }
    }
}

void Jaco2State::setAcceleration(const std::size_t &index, const Eigen::Vector3d vec, AngularAcceleration &acc)
{
    switch (index) {
    case 0:{
        acc.Actuator1_X = vec(0);
        acc.Actuator1_Y = vec(1);
        acc.Actuator1_Z = vec(2);
        break;
    }
    case 1: {
        acc.Actuator2_X = vec(0);
        acc.Actuator2_Y = vec(1);
        acc.Actuator2_Z = vec(2);
        break;
    }
    case 2: {
        acc.Actuator3_X = vec(0);
        acc.Actuator3_Y = vec(1);
        acc.Actuator3_Z = vec(2);
        break;
    }
    case 3: {
        acc.Actuator4_X = vec(0);
        acc.Actuator4_Y = vec(1);
        acc.Actuator4_Z = vec(2);
        break;
    }
    case 4: {
        acc.Actuator5_X = vec(0);
        acc.Actuator5_Y = vec(1);
        acc.Actuator5_Z = vec(2);
        break;
    }
    case 5: {
        acc.Actuator6_X = vec(0);
        acc.Actuator6_Y = vec(1);
        acc.Actuator6_Z = vec(2);
        break;
    }
    default: {
        throw std::runtime_error("illegal index!");
        break;
    }
    }
}

std::vector<Jaco2Calibration::AccelerometerCalibrationParam> Jaco2State::getAccelerometerCalibration() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return accCalibParam_;
}

Jaco2Calibration::TorqueOffsetLut Jaco2State::getTorqueCalibration() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return torque_offset_;
}
