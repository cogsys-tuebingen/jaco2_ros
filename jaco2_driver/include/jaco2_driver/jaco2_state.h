#ifndef JACO2_STATE_H
#define JACO2_STATE_H

#include <jaco2_driver/jaco2_api.h>
#include <kinova/KinovaTypes.h>
#include <jaco2_data/accelerometer_calibration.hpp>
#include <jaco2_data/torque_offset_lut.hpp>
#include <jaco2_data/torque_offset_calibration.hpp>
#include <jaco2_driver/data/jaco2_joint_state.h>
#include <jaco2_data/types.h>

#include <deque>

enum ReadData
{
    READ_POSITION = 0,
    READ_VELOCITY = 1,
    READ_ACCELERATION = 2,
    READ_TORQUE = 3,
    READ_TORQUE_GRAVITY_FREE = 4,
    READ_CURRENT = 5,
    READ_QUICK_STATUS = 6,
    READ_SENSOR_INFO = 7

};

class Jaco2State
{
public:
    ///
    /// \brief Jaco2State
    /// \param api jaco2 api
    ///
    /// Default read: reads position, velocity and torque alternately at high priority
    ///               read acceleration, quick_status, sensor_info, current and torque_gravity_free alternately at low prioriy
    ///
    Jaco2State(Jaco2API &api);

    AngularPosition getAngularPosition() const;
    AngularPosition getAngularVelocity() const;
    AngularPosition getAngularAcceleration() const;
    AngularPosition getAngularForce() const;
    AngularPosition getAngularCurrent() const;
    AngularPosition getTorqueGFree() const;
    QuickStatus getQuickStatus() const;
    SensorsInfo getSensorInfo() const;

    jaco2_data::TimeStamp getLastUpdate(int read_data) const;
    jaco2_data::JointStateDataStamped getJointState() const;
    jaco2_data::AccelerometerData getAccelerometerData() const;

    std::vector<int> getHighPriQue() const;
    std::vector<int> getLowPriQue() const;

//    std::vector<Jaco2Calibration::AccelerometerCalibrationParam> getAccelerometerCalibration() const;
    Jaco2Calibration::TorqueOffsetLut getTorqueCalibration() const;

    void setHighPriQue(std::vector<int> que);
    void setLowPriQue(std::vector<int> que);
    void setPriorityRate(int rate);
    void setAccelerometerCalibration(const std::vector<Jaco2Calibration::AccelerometerCalibrationParam>& params);
    void setTorqueCalibration(const Jaco2Calibration::TorqueOffsetLut& lut);
    void setTorqueCalibration(const Jaco2Calibration::TorqueOffsetCalibration& calib);
    void setVelocitySensorCalibration(const std::vector<double>& factors);
    void setJointNames(const std::vector<std::string> &names);

    ///
    /// \brief readQuickStatus reads the arms status over command layer
    ///                        Use this command carefully, otherwise blocking is possible.
    ///
    void readQuickStatus();

    ///
    /// \brief readPosVelCur reads position, velocity and current alternately.
    ///                      Use this command carefully, otherwise blocking is possible.
    ///
    void readPosVelCur();

    ///
    /// \brief read  default read command reads alternatly high and low priority read que
    ///              Use this command carefully, otherwise blocking is possible.
    ///

    void read();

    ///
    /// \brief readPosVel reads position, velocity alternately
    ///              Use this command carefully, otherwise blocking is possible.
    ///
    void readPosVel();


private:
    void readPosition();
    void readVelocity();
    void readTorque();
    void readCurrent();
    void readTorqueGravityFree();
    void readAcceleration();
    void readSensorInfo();
    void read(int dataID);

    void calculateJointAcceleration();
    void applyAccelerationCalibration();
    void applyTorqueOffsets();
    void applyTorqueOffsets2TorqueGFree();

    void updateJointState();


private:
    mutable std::recursive_mutex data_mutex_;

    Jaco2API &api_;

    int readCmd_;
    int readCmdLowPri_;
    int readCmdHighPri_;
    int priortyThreshold_;
    int priortyRate_;

    std::vector<int> highPriority_;
    std::vector<int> lowPriority_;

    KinovaJointState kinova_state_;
    AngularPosition current_current_;
    AngularPosition current_torque_gravity_free_;
    QuickStatus quick_status_;
    SensorsInfo sensor_info_;
    jaco2_data::TimeStamp time_position_;
    jaco2_data::TimeStamp time_velocity_;
    jaco2_data::TimeStamp time_torque_;
    jaco2_data::TimeStamp time_current_;
    jaco2_data::TimeStamp time_torque_gravity_free_;
    jaco2_data::TimeStamp time_acceleration_;
    jaco2_data::TimeStamp time_quick_status_;
    jaco2_data::TimeStamp time_sensor_info_;
    Jaco2Calibration::TorqueOffsetLut torque_offset_;
    Jaco2Calibration::TorqueOffsetCalibration torque_offest_fkt_;
    bool calibrate_torque_;
    bool calibrate_torque_fkt_;
    std::deque<AngularPosition> lastVelocity_;
    std::deque<double> dt_;
    int acc_counter_;
    std::vector<double> velocityFactors_;
    Jaco2JointState joint_state_;



};

#endif // JACO2_STATE_H

