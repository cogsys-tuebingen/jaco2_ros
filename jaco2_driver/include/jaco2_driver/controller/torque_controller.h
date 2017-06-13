#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H
#include "jaco2_controller.h"
#include <kinova/KinovaTypes.h>
#include <kinova/KinovaArithmetics.hpp>
#include <jaco2_kin_dyn_lib/joint_vel_pos_estimator.h>

class TorqueController : public Jaco2Controller
{
public:
    TorqueController(Jaco2State &state, Jaco2API &api);

    virtual void start() override;

    void setTorque(const AngularPosition& torques);
    void setTorque(const AngularInfo& torques);

    void setGains(double p, double i, double d);
    void setQGains(double p, double i, double d);


    virtual void write() override;

    virtual bool isDone() const override;
    virtual void stop() override;

protected:
    AngularInfo pidControl();


    AngularInfo meanOfTorqueBuffer() const;

    void calculateEsum(const AngularInfo& diff);

protected:
    Jaco2KinDynLib::JointVelPosEstimator estimator_;
    AngularPosition desired_;
    AngularPosition cmd_;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_command_;
    double kp_;
    double ki_;
    double kd_;
    double kqp_;
    double kqi_;
    double kqd_;
    double samplingPeriod_;
    AngularInfo last_diff_;
    AngularInfo esum_;
    AngularInfo esumQ_;
    std::deque<AngularInfo> torque_buffer_;
    std::deque<AngularInfo> e_buffer_;
    AngularInfo max_torques_;
    int counter_;
};

#endif // TORQUE_CONTROLLER_H