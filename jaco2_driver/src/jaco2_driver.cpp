#include <jaco2_driver/jaco2_driver.h>

Jaco2Driver::Jaco2Driver():
    readCmd_(0),
    moveToAngularPos_(false)
{
    ROS_INFO_STREAM("create jaco 2 driver");
    int result = jaco_api_.init();
    ROS_INFO_STREAM("Jaco API result: "<< result);

    current_velocity_.InitStruct();
    last_command_ = std::time(nullptr);

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
void Jaco2Driver::setAngularVelocity(const AngularPosition &velocity)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Actuators = velocity.Actuators;
    tp.Position.Fingers = velocity.Fingers;
    tp.Position.Type = ANGULAR_VELOCITY;

    write_ = [tp, this]() {
        jaco_api_.setAngularVelocity(tp);
    };
    last_command_ = std::time(nullptr);
}

void Jaco2Driver::setAngularPosition(const AngularPosition &position)
{
    std::unique_lock<std::recursive_mutex> lock(data_mutex_);
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Actuators = position.Actuators;
    tp.Position.Fingers = position.Fingers;
    tp.Position.Type = ANGULAR_POSITION;
    reachedAngularPos_ = false;

    write_ = [tp, this]() {
        if(!moveToAngularPos_ && !reachedAngularPos_)
        {
            moveToAngularPos_ = true;
            jaco_api_.setAngularPosition(tp);
        }
        reachedAngularPos_ = reachedAngularGoal(tp);
        if(reachedAngularPos_)
        {
            moveToAngularPos_ = false;
        }

    };
    last_command_ = std::time(nullptr);
}

void Jaco2Driver::stop()
{
//    target_velocity_.Position.Type = ANGULAR_VELOCITY;
    running_ = false;
}

void Jaco2Driver::stopMovement()
{
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Type = ANGULAR_VELOCITY;

    write_ = [tp, this]() {
        jaco_api_.setAngularVelocity(tp);
    };
}

void Jaco2Driver::tick()
{
    if(std::time(nullptr) - last_command_ > 0.1 && !moveToAngularPos_ )
    {
        TrajectoryPoint tp;
        tp.InitStruct();
        tp.Position.Type = ANGULAR_VELOCITY;

        write_ = [tp, this]() {
            jaco_api_.setAngularVelocity(tp);
        };
    }

    std::unique_lock<std::recursive_mutex> lock(data_mutex_);

    if(write_)
    {
        write_();
    }
    lock.unlock();
    usleep(3000);

    switch (readCmd_) {
    case 0:
    {
        read_ = [this]() {
                readCmd_ = 1;
//                std::cout << "velocity" << std::endl;
                current_velocity_ = jaco_api_.getAngularVelocity();
        };
        break;
    }
    case 1:
    {
        read_ = [this]() {
                readCmd_ = 2;
//                std::cout << "position" << std::endl;
                current_position_ = jaco_api_.getAngularPosition();
        };
        break;
    }
    case 2:
    {
        read_ = [this](){
                readCmd_ = 0;
//                std::cout << "torque" << std::endl;
                current_torque_ = jaco_api_.getAngularForce();
        };
        break;
    }
    }
    if(read_)
    {
        lock.lock();
        read_();
    }
    lock.unlock();
    usleep(3000);
}

bool Jaco2Driver::reachedAngularGoal(const TrajectoryPoint &goal)
{
    double thres = 1;
    bool result = true;
    double diff1 = fabs(current_position_.Actuators.Actuator1 - goal.Position.Actuators.Actuator1);
    double diff2 = fabs(current_position_.Actuators.Actuator2 - goal.Position.Actuators.Actuator2);
    double diff3 = fabs(current_position_.Actuators.Actuator3 - goal.Position.Actuators.Actuator3);
    double diff4 = fabs(current_position_.Actuators.Actuator4 - goal.Position.Actuators.Actuator4);
    double diff5 = fabs(current_position_.Actuators.Actuator5 - goal.Position.Actuators.Actuator5);
    double diff6 = fabs(current_position_.Actuators.Actuator6 - goal.Position.Actuators.Actuator6);
    double diffF1 = fabs(current_position_.Fingers.Finger1 -goal.Position.Fingers.Finger1);
    double diffF2 = fabs(current_position_.Fingers.Finger2 -goal.Position.Fingers.Finger2);
    double diffF3 = fabs(current_position_.Fingers.Finger3 -goal.Position.Fingers.Finger3);

    result = (diff1 < thres) && (diff2 < thres) && (diff3 < thres) && (diff4 < thres) &&
             (diff5 < thres) && (diff6 < thres) && (diffF1 < thres) && (diffF2 < thres) &&
             (diffF3 < thres);

    return result;

}

bool Jaco2Driver::reachedAngularGoal() const
{
    return reachedAngularPos_;
}
