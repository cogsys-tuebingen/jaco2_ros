#ifndef JACO2_STATE_H
#define JACO2_STATE_H

#include <jaco2_driver/jaco2_api.h>
#include <kinova/KinovaTypes.h>
#include <jaco2_driver/accelerometer_calibration.hpp>

#include <deque>

enum ReadData
{
    READ_POSITION = 0,
    READ_VELOCITY = 1,
    READ_ACCELRATION = 2,
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


    AngularAcceleration getLinearAcceleration() const;

    QuickStatus getQuickStatus() const;

    SensorsInfo getSensorInfo() const;

    std::chrono::time_point<std::chrono::high_resolution_clock> getLastUpdate(int read_data) const;

    std::vector<int> getHighPriQue() const;
    std::vector<int> getLowPriQue() const;

    std::vector<Jaco2Calibration::AccelerometerCalibrationParam> getAccelerometerCalibration() const;

    void setHighPriQue(std::vector<int> que);
    void setLowPriQue(std::vector<int> que);
    void setPriorityRate(int rate);
    void setAccelerometerCalibration(const std::vector<Jaco2Calibration::AccelerometerCalibrationParam>& params);

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

    static void getAcceleration(const std::size_t& index, const AngularAcceleration& acc, Eigen::Vector3d& vec);
    static void setAcceleration(const std::size_t& index, const Eigen::Vector3d vec, AngularAcceleration& acc);



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

    AngularPosition current_position_;
    AngularPosition current_velocity_;
    AngularPosition current_joint_acceleration_;
    AngularPosition current_torque_;
    AngularPosition current_current_;
    AngularPosition current_torque_gravity_free_;
    AngularAcceleration current_acceleration_;
    QuickStatus quick_status_;
    SensorsInfo sensor_info_;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_position_;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_velocity_;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_torque_;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_current_;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_torque_gravity_free_;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_acceleration_;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_quick_status_;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_sensor_info_;
    std::vector<Jaco2Calibration::AccelerometerCalibrationParam> accCalibParam_;
    std::vector<bool> calibrate_acc_;
    std::deque<AngularPosition> lastVelocity_;
    std::deque<double> dt_;
    int acc_counter_;

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

};

#endif // JACO2_STATE_H

