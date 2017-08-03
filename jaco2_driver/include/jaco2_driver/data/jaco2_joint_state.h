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
#include <jaco2_data/extended_joint_state_data.h>
#include <jaco2_data/accelerometer_calibration.hpp>
#include <jaco2_driver/data/joint_state_outlier_filter.h>
#include <jaco2_driver/data/gravity_estimator.h>

struct KinovaJointState{
    jaco2_data::TimeStamp stamp;
    jaco2_data::TimeStamp acc_stamp;
    AngularPosition position;
    AngularPosition velocity;
    AngularPosition acceleration;
    AngularPosition torque;
    AngularAcceleration accelerometers;
};


/**
 * @brief The Jaco2JointState class normalized joint data in radian.
 *        Converts API data to jaco2_data
 *        Estimation of gravity
 *        Outlier filtering
 *        Use update methods!
 */
class Jaco2JointState
{
public:
    Jaco2JointState();

    void useOutlierFilter(bool arg);
    void setOutlierThreshold(double torque, double acc);
    /**
     * @brief setAngularData set Angular data. No gravity estimation, no filtering.
     * @param type type of data
     * @param pos the data
     */
    void setAngularData(const jaco2_data::JointStateData::DataType type, const AngularPosition& pos);

    /**
     * @brief setLinearData set linear acceleration (accelerometer) data.  No gravity estimation, no filtering.
     * @param accs linear acceleration (accelerometer) data
     * @param stamp time stamp od data
     */
    void setLinearData(const AngularAcceleration& accs, const jaco2_data::TimeStamp &stamp);

    AngularInfo getAngularData(const jaco2_data::JointStateData::DataType type) const;

    jaco2_data::JointStateData getJointState() const;
    jaco2_data::AccelerometerData getLinearAccelerations() const;
    jaco2_data::ExtendedJointStateData getExtJointState() const;

    void setJointNames(const std::vector<std::string> &names);
    void setAccelerometerCalibration(std::vector<Jaco2Calibration::AccelerometerCalibrationParam> params);


    void update(const KinovaJointState& data);
    void update(const jaco2_data::TimeStamp& t,
                const AngularPosition& pos,
                const AngularPosition& vel,
                const AngularPosition& acc,
                const AngularPosition& tor,
                const AngularAcceleration& lacc);
    /**
     * @brief estimateG estimates Gravity. Has to be called if setter are used
     */

    void estimateG();

private:
    void applyCalibration();

private:
    bool use_outlier_fiter_;
    GravityEstimator gravity_;
    jaco2_data::ExtendedJointStateData current_state_;
    std::vector<bool> calibrate_acc_;
    std::vector<Jaco2Calibration::AccelerometerCalibrationParam> accCalibParam_;
    JointStateOutlierFilter filter_;




};

#endif // JACO2_JOINT_STATE_H
