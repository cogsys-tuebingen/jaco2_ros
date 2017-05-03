#ifndef JACO2_CONTROLLER_H
#define JACO2_CONTROLLER_H

#include <kinova/KinovaTypes.h>
#include <jaco2_driver/jaco2_api.h>
#include <jaco2_driver/jaco2_state.h>

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

    virtual void start()
    {

    }

    virtual void stop()
    {

    }

protected:
    virtual void write() = 0;

protected:
    Jaco2Controller(Jaco2State &state, Jaco2API &api)
        : state_(state), api_(api)
    {

    }

protected:
    Jaco2State &state_;
    Jaco2API &api_;
};
#endif // JACO2_CONTROLLER_H

