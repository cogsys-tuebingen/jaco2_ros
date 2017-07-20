#ifndef JACO2_CONTROLLER_H
#define JACO2_CONTROLLER_H

#include <kinova/KinovaTypes.h>
#include <jaco2_driver/jaco2_api.h>
#include <jaco2_driver/jaco2_state.h>
#include <jaco2_driver/jaco2_driver_configureConfig.h>
#include <jaco2_driver/utility/delegate.hpp>



class Jaco2Controller
{
public:
    enum Result{
        WORKING = 1,
        SUCCESS = 0,
        UNKNOWN_FAILURE = -1,
        COLLISION = -2
    };

    typedef delegate<void(const Jaco2Controller::Result)> TerminationCallback;

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

    virtual Result getResult() const
    {
        return result_;
    }


    virtual void setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg) = 0;

protected:
    virtual void write() = 0;

protected:
    Jaco2Controller(Jaco2State &state, Jaco2API &api, TerminationCallback& t )
        : state_(state), api_(api), done_(false), result_(SUCCESS), t_(t)
    {

    }

protected:
    Jaco2State &state_;
    Jaco2API &api_;
    bool done_;
    Result result_;
    TerminationCallback t_;
};
#endif // JACO2_CONTROLLER_H

