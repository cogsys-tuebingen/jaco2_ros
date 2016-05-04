#include <jaco2_driver/jaco2_state.h>

Jaco2State::Jaco2State(Jaco2API &api)
    : api_(api), readCmd_(0),readCmdLowPri_(0), readCmdHighPri_(0), priortyThreshold_(2), priortyRate_(priortyThreshold_ + 1)
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

AngularAcceleration Jaco2State::getAngularAcceleration() const
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
    case 0:
    {

//        current_position_ = api_.getAngularPosition();
        readPosition();
        break;
    }
    case 1:
    {
//        current_velocity_ = api_.getAngularVelocity();
        readVelocity();
        break;
    }
    case 2:
    {
//        current_current_ = api_.getAngularCurrent();
        readCurrent();
        break;
    }
    }

    readCmd_ = (readCmd_ + 1) % 3;
}


void Jaco2State::read()
{
    if(readCmd_ < priortyThreshold_)
    {
        read(highPriority_[readCmdHighPri_]);
        std::unique_lock<std::recursive_mutex> lock(data_mutex_);
        readCmdHighPri_ = (readCmdHighPri_ + 1) % highPriority_.size();
    }
    else
    {
        read(lowPriority_[readCmdLowPri_]);
        std::unique_lock<std::recursive_mutex> lock(data_mutex_);
        readCmdLowPri_ =  (readCmdLowPri_ + 1) % lowPriority_.size();
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
    case 0:
    {
//        current_velocity_ = api_.getAngularVelocity();
        readVelocity();
        break;
    }
    case 1:
    {
//        current_position_ = api_.getAngularPosition();
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
    current_velocity_ = api_.getAngularVelocity();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_velocity_ = std::chrono::high_resolution_clock::now();
}

void Jaco2State::readTorque()
{
    current_torque_ = api_.getAngularForce();
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    time_torque_ = std::chrono::high_resolution_clock::now();
}

void Jaco2State::readTorqueGravityFree()
{
  current_torque_gravity_free_ = api_.getAngularForceGravityFree();
  std::unique_lock<std::recursive_mutex> lock(data_mutex_);
  time_torque_gravity_free_ = std::chrono::high_resolution_clock::now();
}

void Jaco2State::readAcceleration()
{
  current_acceleration_ =api_.getActuatorAcceleration();
  std::unique_lock<std::recursive_mutex> lock(data_mutex_);
  time_acceleration_ = std::chrono::high_resolution_clock::now();
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
