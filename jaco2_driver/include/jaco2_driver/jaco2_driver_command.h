#ifndef JACO2DRIVERCOMMAND_H
#define JACO2DRIVERCOMMAND_H
#include<memory>
#include <jaco2_driver/jaco2_api.h>

class Jaco2DriverCommand
{
public:
    Jaco2DriverCommand(std::shared_ptr<Jaco2API>& jaco2_api);
    ~Jaco2DriverCommand();

    virtual void execute() = 0;

protected:
    std::shared_ptr<Jaco2API> jaco2_api_;
    mutable std::recursive_mutex data_mutex_;
};

#endif // JACO2DRIVERCOMMAND_H
