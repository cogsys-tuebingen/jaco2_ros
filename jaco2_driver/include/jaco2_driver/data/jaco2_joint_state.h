#ifndef JACO2_JOINT_STATE_H
#define JACO2_JOINT_STATE_H
#include <vector>
#include <kinova/KinovaTypes.h>
#include <chrono>
#include <Eigen/Core>
#include <sensor_msgs/JointState.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <deque>

struct KinovaJointState{
    std::chrono::time_point<std::chrono::high_resolution_clock> stamp;
    AngularPosition position;
    AngularPosition velocity;
    AngularPosition acceleration;
    AngularPosition torque;
    AngularAcceleration accelerometers;
};

struct Jaco2JointStateData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d gravity;

    std::chrono::time_point<std::chrono::high_resolution_clock> stamp;
    std::vector<std::string> names;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> torque;

    std::vector<Eigen::Vector3d> linear_accelerations;

    inline jaco2_msgs::Jaco2JointState toMSG()
    {
        jaco2_msgs::Jaco2JointState j;
        j.header.stamp.fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count());
        j.position = position;
        j.velocity = velocity;
        j.acceleration = acceleration;
        j.effort = torque;

        return j;
    }

    inline sensor_msgs::JointState toROS()
    {
        sensor_msgs::JointState j;
        j.header.stamp.fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count());
        j.position = position;
        j.velocity = velocity;
        j.effort = torque;
        return j;
    }

};

enum AngularDataFields{
    POS = 0,
    VEL = 1,
    ACC = 2,
    TOR = 3
};
/**
 * @brief The Jaco2JointState class normalized joint data in radian
 */
class Jaco2JointState
{
public:
    Jaco2JointState();

    void setAngularData(const AngularDataFields type, const AngularPosition& pos);
    void setLinearData(const AngularAcceleration& accs);

    AngularInfo getAngularData(const AngularDataFields type) const;
    std::vector<double> getData(const AngularDataFields type, bool degrees) const;

    void set(KinovaJointState& data);


    sensor_msgs::JointState getROS() const;
    jaco2_msgs::Jaco2JointState getMessage() const;

private:
    void estimateG(double x, double y, double z);


private:
    std::size_t buffer_size_;
    std::deque<Eigen::Vector3d> g_buffer_;
    Jaco2JointStateData current_state_;




};

#endif // JACO2_JOINT_STATE_H
