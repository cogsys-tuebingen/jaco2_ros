#ifndef JACO2_JOINT_STATE_H
#define JACO2_JOINT_STATE_H
/// System
#include <vector>
#include <chrono>
#include <deque>
#include <Eigen/Core>
/// Kinova
#include <kinova/KinovaTypes.h>
/// ROS
#include <sensor_msgs/JointState.h>
/// Jaco2
#include <jaco2_data/joint_state_data.h>

struct KinovaJointState{
    std::chrono::time_point<std::chrono::high_resolution_clock> stamp;
    AngularPosition position;
    AngularPosition velocity;
    AngularPosition acceleration;
    AngularPosition torque;
    AngularAcceleration accelerometers;
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


private:
    void estimateG(double x, double y, double z);


private:
    std::size_t buffer_size_;
    std::deque<Eigen::Vector3d> g_buffer_;
    jaco2_data::JointStateData current_state_;




};

#endif // JACO2_JOINT_STATE_H
