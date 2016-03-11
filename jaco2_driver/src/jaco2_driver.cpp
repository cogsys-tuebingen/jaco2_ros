#include <jaco2_driver/jaco2_driver.h>

Jaco2Driver::Jaco2Driver()
{
    ROS_INFO_STREAM("create jaco 2 driver");
    int result = jaco_api_.init();
    ROS_INFO_STREAM("Jaco API result: "<< result);

    target_velocity_.InitStruct();
    target_velocity_.Position.Type = ANGULAR_VELOCITY;

    current_velocity_.InitStruct();
    last_command_ = std::time(nullptr);

    quickStatus_ = jaco_api_.getQuickStatus();
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
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return current_position_;
}

AngularPosition Jaco2Driver::getAngularVelocity() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return current_velocity_;
}

AngularPosition Jaco2Driver::getAngularForce() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return current_torque_;
}
void Jaco2Driver::setAngularVelocity(const TrajectoryPoint &velocity)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    target_velocity_ = velocity;
    target_velocity_.Position.Type = ANGULAR_VELOCITY;
    last_command_ = std::time(nullptr);
}

void Jaco2Driver::stop()
{
    target_velocity_.Position.Type = ANGULAR_VELOCITY;
    running_ = false;
}

void Jaco2Driver::tick()
{
    if(std::time(nullptr) - last_command_ > 0.1)
    {
        target_velocity_.InitStruct();
        target_velocity_.Position.Type = ANGULAR_VELOCITY;
    }
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    TrajectoryPoint target_velocity = target_velocity_;
    lock.unlock();

    jaco_api_.setAngularVelocity(target_velocity);
    usleep(5000);
    AngularPosition current_velocity = jaco_api_.getAngularVelocity();
//    usleep(5000);
    AngularPosition current_position = jaco_api_.getAngularPosition();
    AngularPosition current_torque = jaco_api_.getAngularForce();
    usleep(5000);

    lock.lock();
    current_velocity_ = current_velocity;
    current_position_ = current_position;
    current_torque_ = current_torque;
}
