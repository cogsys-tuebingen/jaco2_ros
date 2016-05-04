#include <jaco2_driver/jaco2_driver.h>

Jaco2Driver::Jaco2Driver():
    state_(jaco_api_),

    active_controller_(nullptr),

    position_controller_(state_, jaco_api_),
    velocity_controller_(state_, jaco_api_),
    p2p_velocity_controller_(state_,jaco_api_),
    empty_controller_(state_,jaco_api_),
    gripper_controller_(state_,jaco_api_),
    paused_(false)
{
    ROS_INFO_STREAM("create jaco 2 driver");
    int result = jaco_api_.init();
    ROS_INFO_STREAM("Jaco API result: "<< result);

    state_.readQuickStatus();
    usleep(U_SlEEP_TIME);
    usleep(U_SlEEP_TIME);
    quickStatus_ =  state_.getQuickStatus();


    running_ = true;
    spinner_ = std::thread([this](){
        ROS_INFO_STREAM("start thread");
        while(true) {
            {
                std::unique_lock<std::recursive_mutex> lock(running_mutex_);
                if(!running_) {
                    return;
                }
            }

            tick();
        }
        ROS_WARN_STREAM("stop thread");
    });
}

Jaco2Driver::~Jaco2Driver()
{
    spinner_.join();
}

AngularPosition Jaco2Driver::getAngularPosition() const
{
    return state_.getAngularPosition();
}

AngularPosition Jaco2Driver::getAngularVelocity() const
{
    return state_.getAngularVelocity();
}

AngularPosition Jaco2Driver::getAngularForce() const
{
    return state_.getAngularForce();
}
void Jaco2Driver::setAngularVelocity(const AngularPosition &velocity)
{
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Actuators = velocity.Actuators;
    tp.Position.Fingers = velocity.Fingers;
    tp.Position.Type = ANGULAR_VELOCITY;

    velocity_controller_.setVelocity(tp);
    active_controller_ = &velocity_controller_;
}

void Jaco2Driver::setAngularPosition(const AngularPosition &position)
{
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Actuators = position.Actuators;
    tp.Position.Fingers = position.Fingers;
    tp.Position.Type = ANGULAR_POSITION;

    position_controller_.setPosition(tp);
    active_controller_ = &position_controller_;
}

void Jaco2Driver::setTrajectory(const JointTrajectory &trajectory)
{
    p2p_velocity_controller_.setTrajectory(trajectory);
    active_controller_ = &p2p_velocity_controller_;
}

void Jaco2Driver::stop()
{
    //    target_velocity_.Position.Type = ANGULAR_VELOCITY;
    running_ = false;
}

void Jaco2Driver::finish()
{
    active_controller_ = nullptr;
}

void Jaco2Driver::stopMovement()
{
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Type = ANGULAR_VELOCITY;

    velocity_controller_.setVelocity(tp);
    active_controller_ = &velocity_controller_;
}
void Jaco2Driver::stopArm()
{
    executeLater([this](){
        jaco_api_.stopAPI();
        paused_ = true;
        usleep(5000);
    });
}

void Jaco2Driver::startArm()
{
    executeLater([this](){
        jaco_api_.startAPI();
        paused_ = false;
        usleep(5000);
    });
}

void Jaco2Driver::homeArm()
{
    executeLater([this](){
        paused_ = true;
        jaco_api_.moveHome();
        usleep(5000);
        jaco_api_.initFingers();
        usleep(10000);
        paused_ = false;
    });
}

void Jaco2Driver::executeLater(std::function<void ()> fn)
{
    std::unique_lock<std::recursive_mutex> lock(commands_mutex_);
    commands_.emplace_back(fn);
}

void Jaco2Driver::tick()
{
    if(active_controller_ && !paused_) {
        active_controller_->read();
        usleep(4000);
        if(active_controller_)
        {
            active_controller_->execute();
            usleep(5000);
        }
    }
    else
    {
        empty_controller_.read();
        usleep(5000);
    }

    std::unique_lock<std::recursive_mutex> lock(commands_mutex_);
    auto commands = commands_;
    commands_.clear();
    lock.unlock();
    for(auto fn : commands) {
        fn();
    }
}

bool Jaco2Driver::reachedGoal() const
{
    if(active_controller_) {
        return active_controller_->isDone();
    } else {
        return false;
    }
}

AngularPosition Jaco2Driver::getCurrentTrajError() const
{
    AngularPosition res;
    res.Actuators = p2p_velocity_controller_.getJointError();
    return res;
}

void Jaco2Driver::setTrajectoryPGains(const ManipulatorInfo &gains)
{
    p2p_velocity_controller_.setGainP(gains);
}

void Jaco2Driver::setTrajectoryIGains(const ManipulatorInfo &gains)
{
    p2p_velocity_controller_.setGainI(gains);
}

void Jaco2Driver::setTrajectoryDGains(const ManipulatorInfo &gains)
{
    p2p_velocity_controller_.setGainD(gains);
}

void Jaco2Driver::setGripperPGain(const double finger1, const double finger2, const double finger3)
{
    gripper_controller_.setGainP(finger1,finger2,finger3);
}

void Jaco2Driver::setGripperIGain(const double finger1, const double finger2, const double finger3)
{
    gripper_controller_.setGainI(finger1,finger2,finger3);
}

void Jaco2Driver::setGripperDGain(const double finger1, const double finger2, const double finger3)
{
    gripper_controller_.setGainD(finger1,finger2,finger3);
}

void Jaco2Driver::setFingerPosition(const AngularPosition &position)
{
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Fingers = position.Fingers;

    position_controller_.setFingerPosition(tp);
    active_controller_ = &position_controller_;
}

void Jaco2Driver::setFingerVelocity(const AngularPosition &finger_velocity)
{
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Fingers = finger_velocity.Fingers;
    velocity_controller_.setFingerPosition(tp);
    active_controller_ = &velocity_controller_;
}

void Jaco2Driver::setGripperFingerVelocity(const int finger1, const int finger2, const int finger3)
{
    gripper_controller_.setFingerVelocity(finger1, finger2, finger3);
}

void Jaco2Driver::grabObj(const bool &useFinger1, const bool &useFinger2, const bool &useFinger3)
{
    gripper_controller_.grabObj(useFinger1, useFinger2, useFinger3);
    active_controller_ = &gripper_controller_;
}

void Jaco2Driver::grabObjSetUnusedFingerPos(const bool &useFinger1, const bool &useFinger2, const bool &useFinger3, const int posFinger1, const int posFinger2, const int posFinger3)
{
    gripper_controller_.grabObjSetUnusedFingerPos(useFinger1, useFinger2, useFinger3, posFinger1, posFinger2, posFinger3);
    active_controller_ = &gripper_controller_;
}

void Jaco2Driver::setStatePriorityRatio(const int r)
{
    state_.setPriorityRate(r);
}

std::chrono::time_point<std::chrono::high_resolution_clock> Jaco2Driver::getLastReadUpdate(int read_data) const
{
    return state_.getLastUpdate(read_data);
}

AngularPosition Jaco2Driver::getCurrent() const
{
    return state_.getAngularCurrent();
}

AngularPosition Jaco2Driver::getAngularForceGravityFree() const
{
    return state_.getTorqueGFree();
}

AngularAcceleration Jaco2Driver::getActuatorAcceleration() const
{
    return state_.getAngularAcceleration();
}

QuickStatus Jaco2Driver::getQuickStatus() const
{
    return state_.getQuickStatus();
}

SensorsInfo Jaco2Driver::getSensorInfo() const
{
    return state_.getSensorInfo();
}
