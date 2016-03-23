#ifndef GRIPPERPIDCONTROLLER_H
#define GRIPPERPIDCONTROLLER_H

#include <jaco2_driver/jaco2_controller.h>
#include <chrono>
#include <kinova/KinovaTypes.h>

class GripperPIDController : public Jaco2Controller
{
public:
    GripperPIDController(Jaco2State &state, Jaco2API& api);

    virtual void write() override;

    virtual bool isDone() const;

    virtual void read() override;

    void setEffort(const double finger1,const double finger2,const double finger3);
    void setGainP(const double finger1,const double finger2,const double finger3);
    void setGainI(const double finger1,const double finger2,const double finger3);
    void setGainD(const double finger1,const double finger2,const double finger3);

private:
    static const double max_current_;
    bool done_;
    double currents_[3];
    TrajectoryPoint tp_;
    double gainP_[3];
    double gainI_[3];
    double gainD_[3];
    double eSum_[3];
    double eLast_[3];

    std::chrono::time_point<std::chrono::high_resolution_clock> last_command_;


};

#endif // GRIPPERPIDCONTROLLER_H
