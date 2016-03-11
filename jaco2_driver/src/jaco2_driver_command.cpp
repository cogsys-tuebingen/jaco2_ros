#include <jaco2_driver/jaco2_driver_command.h>

Jaco2DriverCommand::Jaco2DriverCommand(std::shared_ptr<Jaco2API> &jaco2_api)
{
    jaco2_api_ = jaco2_api;
}

Jaco2DriverCommand::~Jaco2DriverCommand()
{

}
