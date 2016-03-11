#ifndef ANGLEVELOCITYCOMMAND_H
#define ANGLEVELOCITYCOMMAND_H
//c++
#include <memory>
//ROS
#include <jaco2_driver/jaco2_driver_command.h>
#include <jaco2_driver/jaco2_api.h>
#include <kinova/KinovaTypes.h>

class AngleVelocityCommand : public Jaco2DriverCommand
{
public:
    AngleVelocityCommand(std::shared_ptr<Jaco2API> &jaco2_api);
    ~AngleVelocityCommand();

    void setTargetVelocity(const TrajectoryPoint velocity);

    void execute();

private:
//    std::shared_ptr<Jaco2API> jaco2_api_;
//    mutable std::recursive_mutex data_mutex_;
    TrajectoryPoint velocity_;
};

#endif // ANGLEVELOCITYCOMMAND_H
