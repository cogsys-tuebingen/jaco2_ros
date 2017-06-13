#ifndef JACO2_CONTROLLER_H
#define JACO2_CONTROLLER_H

#include <kinova/KinovaTypes.h>
#include <jaco2_driver/jaco2_api.h>
#include <jaco2_driver/jaco2_state.h>
#include <jaco2_driver/jaco2_driver_configureConfig.h>

class Jaco2Controller
{
public:
    virtual ~Jaco2Controller() = default;

    virtual bool isDone() const = 0;

    virtual void execute()
    {
        // write commands
        write();
    }

    virtual void read()
    {
        state_.read();
    }

    virtual void start() = 0;

    virtual void stop()
    {

    }

    virtual void setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg) = 0;

protected:
    virtual void write() = 0;

protected:
    Jaco2Controller(Jaco2State &state, Jaco2API &api)
        : state_(state), api_(api), done_(false)
    {

    }

protected:
    Jaco2State &state_;
    Jaco2API &api_;
    bool done_;
};
#endif // JACO2_CONTROLLER_H

