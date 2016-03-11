#include <jaco2_driver/angle_velocity_command.h>

AngleVelocityCommand::AngleVelocityCommand(std::shared_ptr<Jaco2API>& jaco2_api):
    Jaco2DriverCommand(jaco2_api)
{
//    jaco2_api_ = jaco2_api;
    velocity_.InitStruct();
    velocity_.Position.Type = ANGULAR_POSITION;
}

AngleVelocityCommand::~AngleVelocityCommand()
{

}


void AngleVelocityCommand::execute()
{
    jaco2_api_->setAngularVelocity(velocity_);
}

void AngleVelocityCommand::setTargetVelocity(const TrajectoryPoint velocity)
{
     std::unique_lock<std::recursive_mutex> lock(data_mutex_);
     velocity_ = velocity;
     velocity_.Position.Type = ANGULAR_POSITION;
}
