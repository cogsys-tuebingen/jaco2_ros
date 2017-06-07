#ifndef GRIPPERPIDCONTROLLER_H
#define GRIPPERPIDCONTROLLER_H

#include <jaco2_driver/jaco2_controller.h>
#include <chrono>
#include <kinova/KinovaTypes.h>

class GripperController : public Jaco2Controller
{
public:
    GripperController(Jaco2State &state, Jaco2API& api);
//    ~GripperController();

    virtual void write() override;
    virtual void read() override;

    virtual bool isDone() const;

    virtual void start() override;

    void grabObj(const bool &useFinger1, const bool &useFinger2, const bool &useFinger3);
    void grabObjSetUnusedFingerPos(const bool &useFinger1, const bool &useFinger2, const bool &useFinger3, const int posFinger1, const int posFinger2, const int posFinger3);

    void setFingerVelocity(const int finger1, const int finger2, const int finger3);
    void setGainP(const double finger1, const double finger2, const double finger3);
    void setGainI(const double finger1, const double finger2, const double finger3);
    void setGainD(const double finger1, const double finger2, const double finger3);



private:
    bool notMoving();
    bool reachedGoal();



private:
    bool useFingers_[3];
    bool usePos_;
    AngularPosition lastPosition_;
    AngularPosition currentPosition_;
    TrajectoryPoint tp_;
    double gainP_[3];
    double gainI_[3];
    double gainD_[3];
    double eSum_[3];
    double eLast_[3];
    int fingerVelocity_[3];
    int fingerGoal_[3];
    int counter_;
    double threshold_;
    double diff_[3];


    std::chrono::time_point<std::chrono::high_resolution_clock> last_command_;


};

#endif // GRIPPERPIDCONTROLLER_H
