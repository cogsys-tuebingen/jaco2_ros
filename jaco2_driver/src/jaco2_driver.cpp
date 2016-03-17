#include <jaco2_driver/jaco2_driver.h>

Jaco2Driver::Jaco2Driver():
    state_(jaco_api_),

    active_controller_(nullptr),

    position_controller_(state_, jaco_api_),
    velocity_controller_(state_, jaco_api_),
    p2p_velocity_controller_(state_,jaco_api_)
{
    ROS_INFO_STREAM("create jaco 2 driver");
    int result = jaco_api_.init();
    ROS_INFO_STREAM("Jaco API result: "<< result);


    quickStatus_ = jaco_api_.getQuickStatus();
    usleep(U_SlEEP_TIME);
    usleep(U_SlEEP_TIME);


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

void Jaco2Driver::tick()
{
    state_.read();
    usleep(3000);

    if(active_controller_) {
        active_controller_->execute();
        usleep(5000);
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
