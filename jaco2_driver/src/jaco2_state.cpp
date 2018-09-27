#include <jaco2_driver/jaco2_state.h>
#include <jaco2_driver/jaco2_driver_constants.h>

Jaco2State::Jaco2State(Jaco2API &api, std::string frame_id)
    : api_(api),
      readCmd_(0),
      readCmdLowPri_(0),
      readCmdHighPri_(0),
      priortyThreshold_(2),
      priortyRate_(priortyThreshold_ + 1),
      calibrate_torque_(false),
      calibrate_torque_fkt_(false),
      joint_state_(frame_id)
{
    lowPriority_.push_back(READ_QUICK_STATUS);
    lowPriority_.push_back(READ_CURRENT);
    lowPriority_.push_back(READ_ACCELERATION);
    lowPriority_.push_back(READ_TORQUE_GRAVITY_FREE);
    lowPriority_.push_back(READ_SENSOR_INFO);

    highPriority_.push_back(READ_POSITION);
    highPriority_.push_back(READ_VELOCITY);
    highPriority_.push_back(READ_TORQUE);

    kinova_state_.position.InitStruct();
    kinova_state_.velocity.InitStruct();
    kinova_state_.torque.InitStruct();
    kinova_state_.accelerometers.InitStruct();
    current_current_.InitStruct();
    current_torque_gravity_free_.InitStruct();

//    calibrate_acc_ = {false, false, false, false, false, false};
    velocityFactors_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    acc_counter_ =0;

}

AngularPosition Jaco2State::getAngularPosition() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return kinova_state_.position;
}

AngularPosition Jaco2State::getAngularVelocity() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return kinova_state_.velocity;
}

AngularPosition Jaco2State::getAngularAcceleration() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return kinova_state_.acceleration;
}

AngularPosition Jaco2State::getAngularForce() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return kinova_state_.torque;
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
    updateJointState();
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
    case READ_ACCELERATION:
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

jaco2_data::TimeStamp Jaco2State::getLastUpdate(int read_data) const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    switch (read_data) {
    case READ_POSITION:
        return time_position_;
        break;
    case READ_VELOCITY:
        return time_velocity_;
        break;
    case READ_ACCELERATION:
        return time_acceleration_;
        break;
    case READ_TORQUE:
        return time_torque_;
        break;
    case READ_TORQUE_GRAVITY_FREE:
        return time_torque_gravity_free_;
        break;
    case READ_CURRENT:
        return time_current_;
        break;
    case READ_QUICK_STATUS:
        return time_quick_status_;
        break;
    case READ_SENSOR_INFO:
        return time_sensor_info_;
        break;
    default:
        return jaco2_data::TimeStamp();
        break;
    }


}

jaco2_data::JointStateDataStamped Jaco2State::getJointState() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return joint_state_.getJointState();
}

jaco2_data::AccelerometerData Jaco2State::getAccelerometerData() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return joint_state_.getLinearAccelerations();
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
    kinova_state_.position = api_.getAngularPosition();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_position_.now();
}

void Jaco2State::readVelocity()
{
    lastVelocity_.push_back(kinova_state_.velocity);
    kinova_state_.velocity = api_.getAngularVelocity();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    kinova_state_.velocity.Actuators.Actuator1 *= velocityFactors_[0];
    kinova_state_.velocity.Actuators.Actuator2 *= velocityFactors_[1];
    kinova_state_.velocity.Actuators.Actuator3 *= velocityFactors_[2];
    kinova_state_.velocity.Actuators.Actuator4 *= velocityFactors_[3];
    kinova_state_.velocity.Actuators.Actuator5 *= velocityFactors_[4];
    kinova_state_.velocity.Actuators.Actuator6 *= velocityFactors_[5];

    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now - time_velocity_.stamp ;
    time_velocity_.stamp = now;
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
    kinova_state_.torque = api_.getAngularForce();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_torque_.now();
    kinova_state_.stamp.now();
    applyTorqueOffsets();
}

void Jaco2State::readTorqueGravityFree()
{
    current_torque_gravity_free_ = api_.getAngularForceGravityFree();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_torque_gravity_free_.now();
    applyTorqueOffsets2TorqueGFree();
}

void Jaco2State::readAcceleration()
{
    kinova_state_.accelerometers = api_.getActuatorAcceleration();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_acceleration_.now();
    kinova_state_.acc_stamp.now();
}

void Jaco2State::readCurrent()
{
    current_current_ = api_.getAngularCurrent();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_current_.now();
}

void Jaco2State::readQuickStatus()
{
    quick_status_ = api_.getQuickStatus();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_quick_status_.now();
}

void Jaco2State::readSensorInfo()
{
    sensor_info_ = api_.getSensorInfo();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_sensor_info_.now();
}

void Jaco2State::setAccelerometerCalibration(const std::vector<Jaco2Calibration::AccelerometerCalibrationParam>& params)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    joint_state_.setAccelerometerCalibration(params);
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

void Jaco2State::setJointNames(const std::vector<std::string> &names)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    joint_state_.setJointNames(names);
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
    AngularPosition& current_vel = kinova_state_.velocity;
    kinova_state_.acceleration.Actuators.Actuator1 = (current_vel.Actuators.Actuator1 - lastVelocity_[i].Actuators.Actuator1) / dtsum;
    kinova_state_.acceleration.Actuators.Actuator2 = (current_vel.Actuators.Actuator2 - lastVelocity_[i].Actuators.Actuator2) / dtsum;
    kinova_state_.acceleration.Actuators.Actuator3 = (current_vel.Actuators.Actuator3 - lastVelocity_[i].Actuators.Actuator3) / dtsum;
    kinova_state_.acceleration.Actuators.Actuator4 = (current_vel.Actuators.Actuator4 - lastVelocity_[i].Actuators.Actuator4) / dtsum;
    kinova_state_.acceleration.Actuators.Actuator5 = (current_vel.Actuators.Actuator5 - lastVelocity_[i].Actuators.Actuator5) / dtsum;
    kinova_state_.acceleration.Actuators.Actuator6 = (current_vel.Actuators.Actuator6 - lastVelocity_[i].Actuators.Actuator6) / dtsum;
    kinova_state_.acceleration.Fingers.Finger1 = (current_vel.Fingers.Finger1 - lastVelocity_[i].Fingers.Finger1) / dtsum;
    kinova_state_.acceleration.Fingers.Finger2 = (current_vel.Fingers.Finger2 - lastVelocity_[i].Fingers.Finger2) / dtsum;
    kinova_state_.acceleration.Fingers.Finger3 = (current_vel.Fingers.Finger3 - lastVelocity_[i].Fingers.Finger3) / dtsum;
}


void Jaco2State::applyTorqueOffsets()
{
//    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    AngularPosition& cp = kinova_state_.position;
    AngularPosition& ct = kinova_state_.torque;
    if(calibrate_torque_ && ! calibrate_torque_fkt_){
        ct.Actuators.Actuator1 -= torque_offset_.at(1, cp.Actuators.Actuator1);
        ct.Actuators.Actuator2 -= torque_offset_.at(2, cp.Actuators.Actuator2);
        ct.Actuators.Actuator3 -= torque_offset_.at(3, cp.Actuators.Actuator3);
        ct.Actuators.Actuator4 -= torque_offset_.at(4, cp.Actuators.Actuator4);
        ct.Actuators.Actuator5 -= torque_offset_.at(5, cp.Actuators.Actuator5);
        ct.Actuators.Actuator6 -= torque_offset_.at(6, cp.Actuators.Actuator6);
    }
    if(calibrate_torque_fkt_ && !calibrate_torque_){
        ct.Actuators.Actuator1 -= torque_offest_fkt_(0, cp.Actuators.Actuator1);
        ct.Actuators.Actuator2 -= torque_offest_fkt_(1, cp.Actuators.Actuator2);
        ct.Actuators.Actuator3 -= torque_offest_fkt_(2, cp.Actuators.Actuator3);
        ct.Actuators.Actuator4 -= torque_offest_fkt_(3, cp.Actuators.Actuator4);
        ct.Actuators.Actuator5 -= torque_offest_fkt_(4, cp.Actuators.Actuator5);
        ct.Actuators.Actuator6 -= torque_offest_fkt_(5, cp.Actuators.Actuator6);
    }

}

void Jaco2State::applyTorqueOffsets2TorqueGFree()
{
//    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    AngularPosition& cp = kinova_state_.position;
    if(calibrate_torque_ && ! calibrate_torque_fkt_){
        current_torque_gravity_free_.Actuators.Actuator1 -= torque_offset_.at(1, cp.Actuators.Actuator1);
        current_torque_gravity_free_.Actuators.Actuator2 -= torque_offset_.at(2, cp.Actuators.Actuator2);
        current_torque_gravity_free_.Actuators.Actuator3 -= torque_offset_.at(3, cp.Actuators.Actuator3);
        current_torque_gravity_free_.Actuators.Actuator4 -= torque_offset_.at(4, cp.Actuators.Actuator4);
        current_torque_gravity_free_.Actuators.Actuator5 -= torque_offset_.at(5, cp.Actuators.Actuator5);
        current_torque_gravity_free_.Actuators.Actuator6 -= torque_offset_.at(6, cp.Actuators.Actuator6);
    }
    if(calibrate_torque_fkt_ && !calibrate_torque_){
        current_torque_gravity_free_.Actuators.Actuator1 -= torque_offest_fkt_(0, cp.Actuators.Actuator1);
        current_torque_gravity_free_.Actuators.Actuator2 -= torque_offest_fkt_(1, cp.Actuators.Actuator2);
        current_torque_gravity_free_.Actuators.Actuator3 -= torque_offest_fkt_(2, cp.Actuators.Actuator3);
        current_torque_gravity_free_.Actuators.Actuator4 -= torque_offest_fkt_(3, cp.Actuators.Actuator4);
        current_torque_gravity_free_.Actuators.Actuator5 -= torque_offest_fkt_(4, cp.Actuators.Actuator5);
        current_torque_gravity_free_.Actuators.Actuator6 -= torque_offest_fkt_(5, cp.Actuators.Actuator6);
    }
}


Jaco2Calibration::TorqueOffsetLut Jaco2State::getTorqueCalibration() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return torque_offset_;
}


void Jaco2State::updateJointState()
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    joint_state_.update(kinova_state_);
}
