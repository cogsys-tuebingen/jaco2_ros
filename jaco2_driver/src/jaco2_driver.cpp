#include <jaco2_driver/jaco2_driver.h>

Jaco2Driver::Jaco2Driver()
{
    ROS_WARN_STREAM("create jaco 2 driver");
    int result = jaco_api_.init();
    ROS_WARN_STREAM("Jaco API result: "<< result);

    target_velocity_.InitStruct();
    target_velocity_.Position.Type = ANGULAR_VELOCITY;

    current_velocity_.InitStruct();

    running_ = true;
    spinner_ = std::thread([this](){
        ROS_WARN_STREAM("start thread");
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

AngularPosition Jaco2Driver::getAngularVelocity() const
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    return current_velocity_;
}

void Jaco2Driver::setAngularVelocity(const TrajectoryPoint &velocity)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    target_velocity_ = velocity;
    target_velocity_.Position.Type = ANGULAR_VELOCITY;
}

void Jaco2Driver::stop()
{
    target_velocity_.Position.Type = ANGULAR_VELOCITY;
    running_ = false;
}

void Jaco2Driver::tick()
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    TrajectoryPoint target_velocity = target_velocity_;
    lock.unlock();

    jaco_api_.setAngularVelocity(target_velocity);
    usleep(5000);
    AngularPosition current_velocity = jaco_api_.getAngularVelocity();
    usleep(5000);

    lock.lock();
    current_velocity_ = current_velocity;
}
