#include <jaco2_driver/jaco2_driver_command.h>
#include <memory>

Jaco2DriverCommand::Jaco2DriverCommand()
{
    std::vector<std::function<void(Jaco2API&)>> callbacks_;
    int next_callback_ = 0;

    double v = 0.2;
//    callbacks_.push_back([v](Jaco2API& api) {
//        api.setAngularVelocity(v);
//    });
//    callbacks_.push_back([v](Jaco2API& api) {
//        api.setAngularVelocity(v);
//    });

    Jaco2API api;
    while(true) {
        std::function<void(Jaco2API&)> fn = callbacks_[next_callback_];
        fn(api);
        ++next_callback_;
        if(next_callback_ >= callbacks_.size()) {
            next_callback_ = 0;
        }
        usleep(5000);
    }

}


Jaco2DriverCommand::~Jaco2DriverCommand()
{
}



AngleVelocityCommand::AngleVelocityCommand()
{
    cmdTrajPoint_.Position.Type = ANGULAR_VELOCITY;
}

AngleVelocityCommand::~AngleVelocityCommand()
{
}

TrajectoryPoint AngleVelocityCommand::getCommand() const
{
    return cmdTrajPoint_;
}
void AngleVelocityCommand::execute(const Jaco2API &jaco2_api)
{
}

void AngleVelocityCommand::setCommand(const AngularPosition &command)
{
    cmdTrajPoint_.Position.Actuators = command.Actuators;
}


AnglePositionCommand::AnglePositionCommand():
    AngleVelocityCommand()
{
    cmdTrajPoint_.Position.Type = ANGULAR_POSITION;
}

AnglePositionCommand::~AnglePositionCommand()
{
}


