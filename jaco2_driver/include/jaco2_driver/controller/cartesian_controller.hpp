#ifndef CARTESIAN_CONTROLLER_HPP
#define CARTESIAN_CONTROLLER_HPP
#include <jaco2_driver/controller/jaco2_controller.h>
#include <ctime>
class CartesianVelocityController : public Jaco2Controller
{
public:
    CartesianVelocityController(Jaco2State& state, Jaco2API& api, TerminationCallback& t)
        : Jaco2Controller(state, api, t),
          last_command_(std::chrono::high_resolution_clock::now())
    {

    }

    inline virtual void setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg) override
    {
        if(cfg.cartesian_control_ref_frame_fixed){
            api_.setReferenceFrameFixed();
        } else {
            api_.setReferenceFrameRotating();
        }
    }

    inline virtual void setVelocity(const TrajectoryPoint& tp)
    {
        if(done_ || result_ != Result::WORKING){
            api_.cartesianControl();
        }
        cmd_ = tp;
        cmd_.Position.HandMode = HAND_NOMOVEMENT;
        cmd_.Position.Type = CARTESIAN_VELOCITY;

        last_command_ = std::chrono::high_resolution_clock::now();
        done_ = false;
        result_ = Result::WORKING;
    }

    inline virtual void write() override
    {
        auto now = std::chrono::high_resolution_clock::now();

        auto durationLast = now - last_command_;

        samplingPeriod_ = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;


        if(samplingPeriod_ > 0.05){
            stopMotion();
            done_ = true;
            result_ = Result::SUCCESS;
            t_(result_);
            ROS_INFO_STREAM("STOPPED");
            return;
        }

        api_.setCartesianVelocity(cmd_);

    }

    inline virtual void start() override
    {
        api_.cartesianControl();
    }

    inline void stopMotion()
    {
        cmd_.InitStruct();
        cmd_.Position.HandMode = HAND_NOMOVEMENT;
        cmd_.Position.Type = CARTESIAN_VELOCITY;
        for(int i = 0; i < 2; ++i){
            api_.setCartesianVelocity(cmd_);
            usleep(5000);
        }
    }

    inline virtual bool isDone() const
    {
         return done_;
    }

private:
    TrajectoryPoint cmd_;
    double samplingPeriod_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_command_;


};
#endif // CARTESIAN_CONTROLLER_HPP
