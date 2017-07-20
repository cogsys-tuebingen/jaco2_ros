#include <jaco2_driver/jaco2_driver.h>
#include <jaco2_driver/controller/controller_factory.hpp>

Jaco2Driver::Jaco2Driver():
    initialized_(false),
    state_(jaco_api_),

    active_controller_(nullptr),

    position_controller_(state_, jaco_api_),
    empty_controller_(state_,jaco_api_),
    gripper_controller_(state_,jaco_api_),
    gravity_comp_controller_(state_, jaco_api_),
    torque_controller_(state_, jaco_api_),
    paused_(false),
    serviceDone_(true)
{

}

bool Jaco2Driver::initialize(std::string serial, bool right, bool move_home)
{
    serial_ = serial;
    right_arm_ = right;
    ROS_INFO_STREAM("initialize jaco 2 driver for device: " << serial_);
    int result = jaco_api_.init(serial_, right_arm_, move_home);
    ROS_INFO_STREAM("Jaco API result: "<< result);
    int api_major, api_minor, api_version;
    jaco_api_.getApiVersion(api_major, api_minor, api_version);
    ROS_INFO_STREAM("Jaco API VERSION: " << api_major << "." << api_minor << "." << api_version );

    state_.readQuickStatus();
    usleep(U_SlEEP_TIME);
    usleep(U_SlEEP_TIME);
    quickStatus_ =  state_.getQuickStatus();


    running_ = true;
    spinner_ = std::thread([this](){
        ROS_INFO_STREAM("start thread");
        while(true) {
            {
                std::unique_lock<std::recursive_mutex> lock(running_mutex_);
                if(!running_) {
                    return;
                }
            }

            tick();
        }
        ROS_WARN_STREAM("stop thread");
    });

    initialized_ = true;
    return (result == 1);
}

Jaco2Driver::~Jaco2Driver()
{
    spinner_.join();
}

AngularPosition Jaco2Driver::getAngularPosition() const
{
    return state_.getAngularPosition();
}

AngularPosition Jaco2Driver::getAngularVelocity() const
{
    return state_.getAngularVelocity();
}

AngularPosition Jaco2Driver::getAngularAcceleration() const
{
    return state_.getAngularAcceleration();
}

AngularPosition Jaco2Driver::getAngularForce() const
{
    return state_.getAngularForce();
}

void Jaco2Driver::setActiveController(Jaco2Controller* controller)
{
    if(active_controller_ == controller)
    {
        return;
    }
    if(active_controller_){
        active_controller_->stop();
    }

    active_controller_ = controller;
    if(active_controller_){
        active_controller_->start();
    }

}

void Jaco2Driver::setAngularVelocity(const AngularPosition &velocity)
{
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Actuators = velocity.Actuators;
    tp.Position.Fingers = velocity.Fingers;
    tp.Position.Type = ANGULAR_VELOCITY;

    velocity_controller_->setVelocity(tp);
    setActiveController(velocity_controller_.get());
}

void Jaco2Driver::setAngularPosition(const AngularPosition &position)
{
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Actuators = position.Actuators;
    tp.Position.Fingers = position.Fingers;
    tp.Position.Type = ANGULAR_POSITION;

    position_controller_.setPosition(tp);
    setActiveController(&position_controller_);
}

void Jaco2Driver::setTrajectory(const JointTrajectory &trajectory)
{
    trajectory_controller_->setTrajectory(trajectory);
    setActiveController(trajectory_controller_.get());
}

void Jaco2Driver::stop()
{
    //    target_velocity_.Position.Type = ANGULAR_VELOCITY;
    running_ = false;
    stopMovement();
    jaco_api_.exitAPI();

}

void Jaco2Driver::enableGravityCompensation()
{
    setActiveController(&gravity_comp_controller_);
}

void Jaco2Driver::disableGravityCompensation()
{
    setActiveController(nullptr);
}

void Jaco2Driver::finish()
{
    setActiveController(nullptr);
}

void Jaco2Driver::stopMovement()
{
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Type = ANGULAR_VELOCITY;

    velocity_controller_->setVelocity(tp);
    setActiveController(velocity_controller_.get());
}
void Jaco2Driver::stopArm()
{
    executeLater([this](){
        serviceDone_ = false;
        jaco_api_.stopAPI();
        paused_ = true;
        usleep(U_SlEEP_TIME);
        serviceDone_= true;
    });
}

void Jaco2Driver::startArm()
{
    executeLater([this](){
        serviceDone_ = false;
        jaco_api_.startAPI();
        paused_ = false;
        usleep(U_SlEEP_TIME);
        serviceDone_= true;
    });
}

void Jaco2Driver::homeArm()
{
    executeLater([this](){
        serviceDone_ = false;
        paused_ = true;
        jaco_api_.moveHome();
        usleep(U_SlEEP_TIME);
        jaco_api_.initFingers();
        usleep(2*U_SlEEP_TIME);
        paused_ = false;
        serviceDone_= true;
    });
}

void Jaco2Driver::setTorqueZero(int actuator)
{
    executeLater([this, actuator](){
        serviceDone_ = false;
        switch (actuator) {
        case Actuator1:
            setTorqueZeroResult_ = jaco_api_.setTorqueZero(Actuator1);
            break;
        case Actuator2:
            setTorqueZeroResult_ = jaco_api_.setTorqueZero(Actuator2);
            break;
        case Actuator3:
            setTorqueZeroResult_ = jaco_api_.setTorqueZero(Actuator3);
            break;
        case Actuator4:
            setTorqueZeroResult_ = jaco_api_.setTorqueZero(Actuator4);
            break;
        case Actuator5:
            setTorqueZeroResult_ = jaco_api_.setTorqueZero(Actuator5);
            break;
        case Actuator6:
            setTorqueZeroResult_ = jaco_api_.setTorqueZero(Actuator6);
            break;
        default:
            break;
        }
        usleep(2*U_SlEEP_TIME);
        serviceDone_= true;
    });
}

bool Jaco2Driver::serviceDone() const
{
    std::unique_lock<std::recursive_mutex> lock(commands_mutex_);
    return serviceDone_;
}

void Jaco2Driver::executeLater(std::function<void ()> fn)
{
    std::unique_lock<std::recursive_mutex> lock(commands_mutex_);
    commands_.emplace_back(fn);
}


void Jaco2Driver::tick()
{
    if(active_controller_ && !paused_) {
        active_controller_->read();
        if(active_controller_)
        {
            controller_done_ = active_controller_->isDone();
            if(controller_done_){
                result_ = active_controller_->getResult();
                active_controller_->stop();
                setActiveController(&empty_controller_);
            }
            else{
                active_controller_->execute();
                usleep(U_SlEEP_TIME);
            }
        }
    }
    else
    {
        empty_controller_.read();
    }

    std::unique_lock<std::recursive_mutex> lock(commands_mutex_);
    auto commands = commands_;
    commands_.clear();
    lock.unlock();
    for(auto fn : commands) {
        fn();
    }

}

bool Jaco2Driver::reachedGoal(ControllerResult& result_type) const
{
    if(active_controller_) {
        result_type = result_;
        return active_controller_->isDone();
    } else {
        return false;
    }
}

AngularPosition Jaco2Driver::getCurrentTrajError() const
{
    AngularPosition res;
    res.Actuators = trajectory_controller_->getJointError();
    return res;
}

Jaco2Calibration::TorqueOffsetLut Jaco2Driver::getTorqueCalibration() const
{
    return state_.getTorqueCalibration();
}
void Jaco2Driver::updateControllerConfig(jaco2_driver::jaco2_driver_configureConfig& cfg)
{
    std::unique_lock<std::recursive_mutex> lock(commands_mutex_);
    velocity_controller_->setConfig(cfg);
    position_controller_.setConfig(cfg);
    trajectory_controller_->setConfig(cfg);
    empty_controller_.setConfig(cfg);
    gripper_controller_.setConfig(cfg);
    gravity_comp_controller_.setConfig(cfg);
    torque_controller_.setConfig(cfg);
}


void Jaco2Driver::setFingerPosition(const AngularPosition &position)
{
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Fingers = position.Fingers;

    position_controller_.setFingerPosition(tp);
    setActiveController(&position_controller_);
}

void Jaco2Driver::setFingerVelocity(const AngularPosition &finger_velocity)
{
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Fingers = finger_velocity.Fingers;
    velocity_controller_->setFingerPosition(tp);
    setActiveController(velocity_controller_.get());
}

void Jaco2Driver::setTorque(const AngularPosition &torque)
{
    torque_controller_.setTorque(torque);
    setActiveController(&torque_controller_);
}

void Jaco2Driver::setVelocitySensorCalibration(const std::vector<double> &factors)
{
    state_.setVelocitySensorCalibration(factors);
}

void Jaco2Driver::grabObj(const bool &useFinger1, const bool &useFinger2, const bool &useFinger3)
{
    gripper_controller_.grabObj(useFinger1, useFinger2, useFinger3);
    setActiveController(&gripper_controller_);
}

void Jaco2Driver::grabObjSetUnusedFingerPos(const bool &useFinger1, const bool &useFinger2, const bool &useFinger3, const int posFinger1, const int posFinger2, const int posFinger3)
{
    gripper_controller_.grabObjSetUnusedFingerPos(useFinger1, useFinger2, useFinger3, posFinger1, posFinger2, posFinger3);
    setActiveController(&gripper_controller_);
}

void Jaco2Driver::setStatePriorityRatio(const int r)
{
    state_.setPriorityRate(r);
}

void Jaco2Driver::setStateHighPriorityQue(const std::vector<int> &que)
{
    state_.setHighPriQue(que);
}

void Jaco2Driver::setStateLowPriorityQue(const std::vector<int> &que)
{
    state_.setLowPriQue(que);
}

jaco2_data::TimeStamp Jaco2Driver::getLastReadUpdate(int read_data) const
{
    return state_.getLastUpdate(read_data);
}

jaco2_data::JointStateData Jaco2Driver::getJointState() const
{
    return state_,getJointState();
}

jaco2_data::AccelerometerData Jaco2Driver::getAccelerometerData() const
{
    return state_.getAccelerometerData();
}

const jaco2_data::JointStateData& Jaco2Driver::getJointStateRef() const
{
    return state_.getJointStateRef();
}

const jaco2_data::AccelerometerData& Jaco2Driver::getAccelerometerDataRef() const
{
    return state_.getAccelerometerDataRef();
}

AngularPosition Jaco2Driver::getCurrent() const
{
    return state_.getAngularCurrent();
}

AngularPosition Jaco2Driver::getAngularForceGravityFree() const
{
    return state_.getTorqueGFree();
}

QuickStatus Jaco2Driver::getQuickStatus() const
{
    return state_.getQuickStatus();
}

SensorsInfo Jaco2Driver::getSensorInfo() const
{
    return state_.getSensorInfo();
}

void Jaco2Driver::setAccelerometerCalibration(const std::vector<Jaco2Calibration::AccelerometerCalibrationParam> &params)
{
    state_.setAccelerometerCalibration(params);
}

void Jaco2Driver::setTorqueCalibration(const Jaco2Calibration::TorqueOffsetLut &lut)
{
    state_.setTorqueCalibration(lut);
}

void Jaco2Driver::setTorqueCalibration(const Jaco2Calibration::TorqueOffsetCalibration &calib)
{
    state_.setTorqueCalibration(calib);
}

void Jaco2Driver::setJointNames(const std::vector<std::string> &names)
{
    state_.setJointNames(names);
}

bool Jaco2Driver::setGravityParams(const Jaco2Calibration::ApiGravitationalParams& params)
{
    return jaco_api_.setGravityOptimalZParam(params.parameter);
}

int Jaco2Driver::getSetTorqueZeroResult() const
{
    std::unique_lock<std::recursive_mutex> lock(commands_mutex_);
    return setTorqueZeroResult_;
}

void Jaco2Driver::disableForceControl()
{
    executeLater([this](){
        serviceDone_ = false;
        jaco_api_.stopForceControl();
        usleep(5000);
        serviceDone_= true;
    });
}
void Jaco2Driver::enableForceControl()
{
    executeLater([this](){
        serviceDone_ = false;
        jaco_api_.startForceControl();
        usleep(5000);
        serviceDone_= true;
    });
}

void Jaco2Driver::setVelocityController(const std::string &type)
{
    std::unique_lock<std::recursive_mutex> lock(commands_mutex_);
    velocity_controller_ = ControllerFactory::makeVelocityController(state_, jaco_api_, type);
    if(!velocity_controller_){
        std::string msg = "Can not create velocity controller. Wrong Input? Valid types are: "
                + Jaco2DriverConstants::velocity_controller + " & "
                + Jaco2DriverConstants::velocity_collision_controller + ".";
        throw std::runtime_error(msg);
    }
}

void Jaco2Driver::setTrajectoryController(const std::string &type)
{
    std::unique_lock<std::recursive_mutex> lock(commands_mutex_);
    trajectory_controller_ = ControllerFactory::makeTrajectoryTrackingController(state_, jaco_api_, type);
    if(!trajectory_controller_){
        std::string msg = "Can not create trajactory controller. Wrong Input? Valid types are: "
                + Jaco2DriverConstants::trajectory_p2p_velocity_controller + ", "
                + Jaco2DriverConstants::trajectory_p2p_velocity_collision_controller + ", "
                + Jaco2DriverConstants::trajectory_p2p_torque_controller + ", "
                + Jaco2DriverConstants::trajectory_p2p_torque_collision_controller + ".";

        throw std::runtime_error(msg);
    }
}
