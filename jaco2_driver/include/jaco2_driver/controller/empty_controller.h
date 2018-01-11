#ifndef EMPTY_CONTROLLER_H
#define EMPTY_CONTROLLER_H

#include <kinova/KinovaTypes.h>
#include <jaco2_driver/jaco2_api.h>
#include <jaco2_driver/jaco2_state.h>
#include <jaco2_driver/controller/jaco2_controller.h>

class EmptyController : public Jaco2Controller
{
public:
    EmptyController(Jaco2State &state, Jaco2API &api, TerminationCallback& t )
        : Jaco2Controller(state, api, t),
          done_(true)
    {
        result_ = Result::SUCCESS;
    }

    virtual void write() override
    {

    }

    virtual void start() override
    {
    }
    virtual void setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg) override
    {

    }



    virtual bool isDone() const override
    {
        return done_;
    }

private:
    bool done_;
};
#endif // EMPTY_CONTROLLER_H

