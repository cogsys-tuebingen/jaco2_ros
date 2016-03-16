#ifndef JACO2DRIVERCOMMAND_H
#define JACO2DRIVERCOMMAND_H
#include <memory>
#include <jaco2_driver/jaco2_api.h>

class Jaco2DriverCommand
{
public:
    Jaco2DriverCommand();
    virtual ~Jaco2DriverCommand();

    virtual void execute(const Jaco2API& jaco2_api) = 0;


protected:

};

class AngleVelocityCommand : public  Jaco2DriverCommand
{
public:
    AngleVelocityCommand();
    ~AngleVelocityCommand();

    void execute(const Jaco2API& jaco2_api) override;

    TrajectoryPoint getCommand() const;
    void setCommand(const AngularPosition &command);

protected:
    TrajectoryPoint cmdTrajPoint_;
};

class AnglePositionCommand : public AngleVelocityCommand
{
public:
    AnglePositionCommand();
    ~AnglePositionCommand();

};
#endif // JACO2DRIVERCOMMAND_H
