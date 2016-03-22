#include <jaco2_driver/gripper_pid_controller.h>
#include <math.h>
#include <kinova/KinovaTypes.h>
const double GripperPIDController::max_current_ = 1.0;

GripperPIDController::GripperPIDController(Jaco2State &state, Jaco2API& api)
    : Jaco2Controller(state, api),
      done_(false)
{
    tp_.InitStruct();
    tp_.Position.Type = ANGULAR_VELOCITY;
    tp_.Position.HandMode = VELOCITY_MODE;
}

bool GripperPIDController::isDone() const
{
    return done_;
}

void GripperPIDController::setEffort(const double finger1, const double finger2,const double finger3)
{
    currents_[0] = finger1;
    currents_[1] = finger2;
    currents_[2] = finger3;
    last_command_ = std::chrono::high_resolution_clock::now();
}

void GripperPIDController::read()
{
    state_.readCurrent();
}

void GripperPIDController::write()
{
    AngularPosition current = state_.getAngularCurrent();
    double diff[3];
    double d_diff[3];
    diff[0] = currents_[0] - current.Fingers.Finger1;
    diff[1] = currents_[1] - current.Fingers.Finger2;
    diff[2] = currents_[2] - current.Fingers.Finger3;

    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now - last_command_ ;
    double dt = std::chrono::duration_cast<std::chrono::microseconds>(duration).count()*1e-6;
    last_command_ = now;

    if(fabs(diff[0]) < 5e-2 && fabs(diff[1]) < 5e-2 && fabs(diff[2]) < 5e-2)
    {
          done_ = true;
          return;
    }
    else
    {
        for(std::size_t i = 0; i < 3; ++i)
        {
            eSum_[i] += dt*diff[i];
            d_diff[i] = (diff[i] - eLast_[i])/dt;
            eLast_[i] = diff[i];
        }
        tp_.Position.Fingers.Finger1 = gainP_[0] * diff[0] + gainI_[0]*eSum_[0] + gainD_[0]*d_diff[0];
        tp_.Position.Fingers.Finger2 = gainP_[1] * diff[1] + gainI_[1]*eSum_[1] + gainD_[1]*d_diff[1];
        tp_.Position.Fingers.Finger3 = gainP_[2] * diff[2] + gainI_[2]*eSum_[2] + gainD_[2]*d_diff[2];
        api_.setAngularVelocity(tp_);
    }
}




