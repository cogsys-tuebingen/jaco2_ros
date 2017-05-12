#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H
#include "jaco2_controller.h"
#include <kinova/KinovaTypes.h>
#include <kinova/KinovaArithmetics.hpp>

class TorqueController : public Jaco2Controller
{
public:
    TorqueController(Jaco2State &state, Jaco2API &api);

    virtual void start() override;

    void setTorque(const AngularPosition& torques);

    void setGains(double p, double i, double d);


    virtual void write() override;

    virtual bool isDone() const override;

private:
    AngularInfo pidControl();


    AngularInfo meanOfTorqueBuffer() const;

    void calculateEsum(const AngularInfo& diff);

private:
    AngularPosition desired_;
    AngularPosition cmd_;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_command_;
    bool done_;
    double kp_;
    double ki_;
    double kd_;
    double samplingPeriod_;
    AngularInfo last_diff_;
    AngularInfo esum_;
    std::deque<AngularInfo> torque_buffer_;
    std::deque<AngularInfo> e_buffer_;
};

#endif // TORQUE_CONTROLLER_H
