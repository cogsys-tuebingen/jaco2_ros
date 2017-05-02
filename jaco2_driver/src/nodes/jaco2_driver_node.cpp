#include <jaco2_driver/jaco2_driver_node.h>
#include <angles/angles.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <jaco2_driver/data_conversion.h>
#include <jaco2_msgs/FingerPosition.h>
#include <jaco2_msgs/Jaco2Sensor.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <jaco2_msgs/Jaco2Accelerometers.h>
#include <jaco2_driver/jaco2_driver_constants.h>
#include <jaco2_driver/torque_offset_lut.hpp>


namespace {
bool g_running_ = true;
/// \brief Proper shutdown using control c.
void siginthandler(int)
{
    g_running_ = false;
    std::exit(0);
}
}

Jaco2DriverNode::Jaco2DriverNode()
    : private_nh_("~"),
      actionAngleServer_(private_nh_, "arm_joint_angles",false),
      trajServer_(private_nh_,"follow_joint_trajectory/manipulator",false),
      graspServer_(private_nh_, "gripper_command", false),
      fingerServer_(private_nh_,"finger_joint_angles",false),
      blockingAngleServer_(private_nh_, "arm_joint_angles_blocking", false),
      actionAngleServerRunning_(false),
      trajServerRunning_(false),
      gripperServerRunning_(false),
      fingerServerRunning_(false),
      rightArm_(true),
      ok_(true)
{
    pubJointState_ = private_nh_.advertise<sensor_msgs::JointState>("out/joint_states", 2);
    pubJointAngles_ = private_nh_.advertise<jaco2_msgs::JointAngles>("out/joint_angles",2);
    pubFingerPositions_ = private_nh_.advertise<jaco2_msgs::FingerPosition>("out/finger_positions",2);
    pubSensorInfo_ = private_nh_.advertise<jaco2_msgs::Jaco2Sensor>("out/sensor_info",2);
    pubJaco2JointState_ = private_nh_.advertise<jaco2_msgs::Jaco2JointState>("out/joint_state_acc", 2);
    pubJaco2LinAcc_ = private_nh_.advertise<jaco2_msgs::Jaco2Accelerometers>("out/accelerometers",2);

    private_nh_.param<bool>("right_arm", rightArm_, true);
    private_nh_.param<std::string>("jaco_serial", serial_,std::string(""));
    private_nh_.param<std::string>("tf_prefix", tf_prefix_, "jaco_");

    boost::function<void(const jaco2_msgs::JointVelocityConstPtr&)> cb = boost::bind(&Jaco2DriverNode::jointVelocityCb, this, _1);
    boost::function<void(const jaco2_msgs::FingerPositionConstPtr&)> cb_finger = boost::bind(&Jaco2DriverNode::fingerVelocityCb, this, _1);
    subJointVelocity_ = private_nh_.subscribe("in/joint_velocity", 10, cb);
    subFingerVelocity_ = private_nh_.subscribe("in/finger_velocity", 10, cb_finger);

    stopService_ = private_nh_.advertiseService("in/stop", &Jaco2DriverNode::stopServiceCallback, this);
    startService_ = private_nh_.advertiseService("in/start", &Jaco2DriverNode::startServiceCallback, this);
    homingService_ = private_nh_.advertiseService("in/home_arm", &Jaco2DriverNode::homeArmServiceCallback, this);
    zeroTorqueService_ = private_nh_.advertiseService("in/set_torque_zero", &Jaco2DriverNode::setTorqueZeroCallback, this);

    actionAngleServer_.registerGoalCallback(boost::bind(&Jaco2DriverNode::actionAngleGoalCb, this));
    trajServer_.registerGoalCallback(boost::bind(&Jaco2DriverNode::trajGoalCb, this));
    graspServer_.registerGoalCallback(boost::bind(&Jaco2DriverNode::gripperGoalCb, this));
    fingerServer_.registerGoalCallback(boost::bind(&Jaco2DriverNode::fingerGoalCb,this));
    blockingAngleServer_.registerGoalCallback(boost::bind(&Jaco2DriverNode::blockingAngleGoalCb, this));

    f_ = boost::bind(&Jaco2DriverNode::dynamicReconfigureCb, this, _1, _2);
    paramServer_.setCallback(f_);


    jointStateMsg_.name.resize(JACO_JOINTS_COUNT);
    jointStateMsg_.position.resize(JACO_JOINTS_COUNT);
    jointStateMsg_.velocity.resize(JACO_JOINTS_COUNT);
    jointStateMsg_.effort.resize(JACO_JOINTS_COUNT);
    jointStateMsg_.name[0] = tf_prefix_ + "joint_1";
    jointStateMsg_.name[1] = tf_prefix_ + "joint_2";
    jointStateMsg_.name[2] = tf_prefix_ + "joint_3";
    jointStateMsg_.name[3] = tf_prefix_ + "joint_4";
    jointStateMsg_.name[4] = tf_prefix_ + "joint_5";
    jointStateMsg_.name[5] = tf_prefix_ + "joint_6";
    jointStateMsg_.name[6] = tf_prefix_ + "joint_finger_1";
    jointStateMsg_.name[7] = tf_prefix_ + "joint_finger_2";
    jointStateMsg_.name[8] = tf_prefix_ + "joint_finger_3";

    sensorMsg_.name.resize(Jaco2DriverConstants::n_Jaco2Joints);
    sensorMsg_.acceleration.resize(Jaco2DriverConstants::n_Jaco2Joints);
    sensorMsg_.temperature.resize(Jaco2DriverConstants::n_Jaco2Joints);
    sensorMsg_.torque.resize(Jaco2DriverConstants::n_Jaco2Joints);
    sensorMsg_.current.resize(Jaco2DriverConstants::n_Jaco2Joints);
    sensorMsg_.name[0] = tf_prefix_ + "joint_1";
    sensorMsg_.name[1] = tf_prefix_ + "joint_2";
    sensorMsg_.name[2] = tf_prefix_ + "joint_3";
    sensorMsg_.name[3] = tf_prefix_ + "joint_4";
    sensorMsg_.name[4] = tf_prefix_ + "joint_5";
    sensorMsg_.name[5] = tf_prefix_ + "joint_6";


    bool init = controller_.initialize(serial_, rightArm_);
    if(!init){
        ROS_ERROR_STREAM("Jaco 2 cloud not be initialized for device: " << serial_);
    }
    ok_ = init;

    bool use_accel_calib, use_torque_calib;
    private_nh_.param<bool>("jaco_use_accelerometer_calib", use_accel_calib, false);
    if(use_accel_calib) {
        std::string acc_calib_file;
        private_nh_.param<std::string>("jaco_accelerometer_calibration_file", acc_calib_file, "");
        std::vector<Jaco2Calibration::AccelerometerCalibrationParam> acc_params;
        Jaco2Calibration::loadAccCalib(acc_calib_file, acc_params);
        controller_.setAccelerometerCalibration(acc_params);
    }
    private_nh_.param<bool>("jaco_use_torque_calib", use_torque_calib, false);
    if(use_torque_calib){
        std::string torque_calib_file;
        private_nh_.param<std::string>("jaco_torque_calibration_file", torque_calib_file, "/home/zwiener/workspace/jaco_ws/src/jaco2_ros/jaco2_driver/config/torque_offset_lut_jaco2-2.yaml");
        Jaco2Calibration::TorqueOffsetLut lut;
        lut.load(torque_calib_file);
        controller_.setTorqueCalibration(lut);
    }

    actionAngleServer_.start();
    trajServer_.start();
    graspServer_.start();
    fingerServer_.start();
    blockingAngleServer_.start();

    lastTimeAccPublished_ = std::chrono::high_resolution_clock ::now();
}


void Jaco2DriverNode::actionAngleGoalCb()
{
    if(actionAngleServer_.isActive()){
        actionAngleServer_.setPreempted();
        controller_.finish();
    }
//    if(!controller_.reachedGoal()){
//        controller_.finish();
//    }

    jaco2_msgs::ArmJointAnglesGoalConstPtr goal = actionAngleServer_.acceptNewGoal();

    AngularPosition position;
    DataConversion::convert(goal->angles,position);
    AngularPosition currentPos = controller_.getAngularPosition();

    position.Fingers = currentPos.Fingers;
    if(goal->type == jaco2_msgs::ArmJointAnglesGoal::RADIAN){
        DataConversion::to_degrees(position);
    }

    actionAngleServerRunning_ = true;
    controller_.setAngularPosition(position);

}

void Jaco2DriverNode::trajGoalCb()
{
    control_msgs::FollowJointTrajectoryGoalConstPtr goal = trajServer_.acceptNewGoal();
    trajectory_msgs::JointTrajectory traj_msgs = goal->trajectory;

    JointTrajectory driver_trajectory;
    bool input_ok = true;
    auto it_traj_points = traj_msgs.points.begin();
    for(; it_traj_points  < traj_msgs.points.end(); ++it_traj_points )
    {
        input_ok &= ((*it_traj_points).positions.size() == Jaco2DriverConstants::n_Jaco2Joints);
        input_ok &= ((*it_traj_points).velocities.size() == Jaco2DriverConstants::n_Jaco2Joints);
        input_ok &= ((*it_traj_points).accelerations.size() == Jaco2DriverConstants::n_Jaco2Joints);

        if(!input_ok){
            break;
        }

        DataConversion::to_degrees((*it_traj_points).positions);
        DataConversion::to_degrees((*it_traj_points).velocities);
        DataConversion::to_degrees((*it_traj_points).accelerations);

    }

    if(input_ok){
        DataConversion::convert(traj_msgs,driver_trajectory);

        trajServerRunning_ = true;
        controller_.setTrajectory(driver_trajectory);
    }
    else{
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        trajServer_.setAborted(result);
    }
}

void Jaco2DriverNode::gripperGoalCb()
{
    jaco2_msgs::GripperControlGoalConstPtr goal = graspServer_.acceptNewGoal();
    gripperServerRunning_ = true;
    if(goal->useFinger1 && goal->useFinger2 && goal->useFinger3 || !goal->usePos)
    {
        controller_.grabObj(goal->useFinger1, goal->useFinger2, goal->useFinger3);
    }
    else
    {
        controller_.grabObjSetUnusedFingerPos(goal->useFinger1, goal->useFinger2, goal->useFinger3,
                                              goal->posFinger1, goal->posFinger2, goal->posFinger3);
    }
}

void Jaco2DriverNode::fingerGoalCb()
{
    jaco2_msgs::SetFingersPositionGoalConstPtr goal = fingerServer_.acceptNewGoal();
    AngularPosition pos;
    pos.InitStruct();
    pos.Fingers.Finger1 = goal->fingers.finger1;
    pos.Fingers.Finger2 = goal->fingers.finger2;
    pos.Fingers.Finger3 = goal->fingers.finger3;
    fingerServerRunning_ = true;
    controller_.setFingerPosition(pos);
}

void Jaco2DriverNode::blockingAngleGoalCb()
{
    if(blockingAngleServer_.isActive()){
        ROS_DEBUG_STREAM("Controller was busy");
        return;
    }

    jaco2_msgs::ArmJointAnglesGoalConstPtr goal = blockingAngleServer_.acceptNewGoal();
    blockingAngleServer_.isPreemptRequested();

    AngularPosition position;
    DataConversion::convert(goal->angles,position);
    AngularPosition currentPos = controller_.getAngularPosition();

    position.Fingers = currentPos.Fingers;
    if(goal->type == jaco2_msgs::ArmJointAnglesGoal::RADIAN){
        DataConversion::to_degrees(position);
    }

    controller_.setAngularPosition(position);
}

bool Jaco2DriverNode::tick()
{
    if(!g_running_) {
        stop();
    }
    publishJointState();
    publishJointAngles();
    publishSensorInfo();
    if(actionAngleServerRunning_)
    {
        jaco2_msgs::ArmJointAnglesFeedback feedback;
        jaco2_msgs::ArmJointAnglesResult result;
        AngularPosition angles = controller_.getAngularPosition();
        DataConversion::convert(angles,feedback.angles);
        actionAngleServer_.publishFeedback(feedback);
        if(controller_.reachedGoal())
        {
            actionAngleServerRunning_ = false;
            result.val = jaco2_msgs::ArmJointAnglesResult::SUCCESSFUL;
            actionAngleServer_.setSucceeded(result);
            controller_.finish();

        } else if(actionAngleServer_.isPreemptRequested() )
        {
            result.val = jaco2_msgs::ArmJointAnglesResult::PREEMPT;
            actionAngleServerRunning_ = false;
            actionAngleServer_.setPreempted();

            controller_.finish();
        }
    }
    if(blockingAngleServer_.isActive())
    {
        jaco2_msgs::ArmJointAnglesFeedback feedback;
        jaco2_msgs::ArmJointAnglesResult result;
        AngularPosition angles = controller_.getAngularPosition();
        DataConversion::convert(angles,feedback.angles);
        blockingAngleServer_.publishFeedback(feedback);
        if(controller_.reachedGoal())
        {
            result.val = jaco2_msgs::ArmJointAnglesResult::SUCCESSFUL;
            blockingAngleServer_.setSucceeded(result);
            controller_.finish();

        } else if(blockingAngleServer_.isPreemptRequested() )
        {
            result.val = jaco2_msgs::ArmJointAnglesResult::PREEMPT;
            blockingAngleServer_.setPreempted();

            controller_.finish();
        }
    }
    if(trajServerRunning_)
    {
        control_msgs::FollowJointTrajectoryFeedback feedback;
        control_msgs::FollowJointTrajectoryResult result;
        AngularPosition angles = controller_.getAngularPosition();
        DataConversion::convert(angles,feedback.actual.positions);
        angles = controller_.getCurrentTrajError();
        DataConversion::convert(angles,feedback.error.positions);
        trajServer_.publishFeedback(feedback);
        if(controller_.reachedGoal())
        {
            result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
            trajServerRunning_ = false;
            trajServer_.setSucceeded(result);

            controller_.finish();

        } else if(actionAngleServer_.isPreemptRequested() )
        {
            result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
            trajServerRunning_ = false;
            trajServer_.setPreempted();

            controller_.finish();
        }
    }
    if(gripperServerRunning_)
    {
        //        jaco2_msgs::GripperControlFeedback feedback; // TODO
        jaco2_msgs::GripperControlResult result;
        if(controller_.reachedGoal())
        {
            result.val = jaco2_msgs::GripperControlResult::SUCCESSFUL;
            gripperServerRunning_ = false;
            graspServer_.setSucceeded(result);
            controller_.finish();
        }
        else if(graspServer_.isPreemptRequested())
        {
            result.val = jaco2_msgs::GripperControlResult::PREEMPT;
            gripperServerRunning_ = false;
            graspServer_.setPreempted(result);
            controller_.finish();
        }
    }
    if(fingerServerRunning_)
    {
        jaco2_msgs::SetFingersPositionFeedback feedback;
        jaco2_msgs::SetFingersPositionResult result;
        AngularPosition pos = controller_.getAngularPosition();
        feedback.fingers.finger1 = pos.Fingers.Finger1;
        feedback.fingers.finger2 = pos.Fingers.Finger2;
        feedback.fingers.finger3 = pos.Fingers.Finger3;
        fingerServer_.publishFeedback(feedback);
        if(controller_.reachedGoal())
        {
            result.error_code = jaco2_msgs::SetFingersPositionResult::SUCCESS;
            fingerServerRunning_ = false;
            fingerServer_.setSucceeded(result);
            controller_.finish();
        }
        else if(fingerServer_.isPreemptRequested())
        {
            result.error_code = jaco2_msgs::SetFingersPositionResult::PREMPTED;
            fingerServerRunning_ = false;
            fingerServer_.setSucceeded(result);
            controller_.finish();
        }
    }
    return ok_;
}

void Jaco2DriverNode::dynamicReconfigureCb(jaco2_driver::jaco2_driver_configureConfig &config, uint32_t level)
{
    ManipulatorInfo trajectoryGainsP;
    trajectoryGainsP[0] = config.trajectory_p_gain_joint_0;
    trajectoryGainsP[1] = config.trajectory_p_gain_joint_1;
    trajectoryGainsP[2] = config.trajectory_p_gain_joint_2;
    trajectoryGainsP[3] = config.trajectory_p_gain_joint_3;
    trajectoryGainsP[4] = config.trajectory_p_gain_joint_4;
    trajectoryGainsP[5] = config.trajectory_p_gain_joint_5;
    ManipulatorInfo trajectoryGainsI;
    trajectoryGainsI[0] = config.trajectory_i_gain_joint_0;
    trajectoryGainsI[1] = config.trajectory_i_gain_joint_1;
    trajectoryGainsI[2] = config.trajectory_i_gain_joint_2;
    trajectoryGainsI[3] = config.trajectory_i_gain_joint_3;
    trajectoryGainsI[4] = config.trajectory_i_gain_joint_4;
    trajectoryGainsI[5] = config.trajectory_i_gain_joint_5;
    ManipulatorInfo trajectoryGainsD;
    trajectoryGainsD[0] = config.trajectory_i_gain_joint_0;
    trajectoryGainsD[1] = config.trajectory_i_gain_joint_1;
    trajectoryGainsD[2] = config.trajectory_i_gain_joint_2;
    trajectoryGainsD[3] = config.trajectory_i_gain_joint_3;
    trajectoryGainsD[4] = config.trajectory_i_gain_joint_4;
    trajectoryGainsD[5] = config.trajectory_i_gain_joint_5;

    controller_.setTrajectoryPGains(trajectoryGainsP);
    controller_.setTrajectoryIGains(trajectoryGainsI);
    controller_.setTrajectoryDGains(trajectoryGainsD);
    controller_.setGripperPGain(config.gripper_p_gain_finger_1,
                                config.gripper_p_gain_finger_2,
                                config.gripper_p_gain_finger_3);
    controller_.setGripperFingerVelocity(config.gipper_controller_finger_vel_1,
                                         config.gipper_controller_finger_vel_2,
                                         config.gipper_controller_finger_vel_3);
    std::vector<int> highPriQue;
    std::vector<int> lowPriQue;
    if(config.state_high_pri_pos){
        highPriQue.push_back(READ_POSITION);
    }
    if(config.state_high_pri_vel){
        highPriQue.push_back(READ_VELOCITY);
    }
    if(config.state_high_pri_acc){
        highPriQue.push_back(READ_ACCELRATION);
    }
    if(config.state_high_pri_torque){
        highPriQue.push_back(READ_TORQUE);
    }
    if(config.state_high_pri_torque_g_free){
        highPriQue.push_back(READ_TORQUE_GRAVITY_FREE);
    }
    if(config.state_high_pri_info){
        highPriQue.push_back(READ_SENSOR_INFO);
    }
    if(config.state_high_pri_current){
        highPriQue.push_back(READ_CURRENT);
    }
    if(config.state_high_pri_status){
        highPriQue.push_back(READ_QUICK_STATUS);
    }

    if(config.state_low_pri_pos){
        lowPriQue.push_back(READ_POSITION);
    }
    if(config.state_low_pri_vel){
        lowPriQue.push_back(READ_VELOCITY);
    }
    if(config.state_low_pri_acc){
        lowPriQue.push_back(READ_ACCELRATION);
    }
    if(config.state_low_pri_torque){
        lowPriQue.push_back(READ_TORQUE);
    }
    if(config.state_low_pri_torque_g_free){
        lowPriQue.push_back(READ_TORQUE_GRAVITY_FREE);
    }
    if(config.state_low_pri_info){
        lowPriQue.push_back(READ_SENSOR_INFO);
    }
    if(config.state_low_pri_current){
        lowPriQue.push_back(READ_CURRENT);
    }
    if(config.state_low_pri_status){
        lowPriQue.push_back(READ_QUICK_STATUS);
    }
    controller_.setStateHighPriorityQue(highPriQue);
    controller_.setStateLowPriorityQue(lowPriQue);

}

void Jaco2DriverNode::jointVelocityCb(const jaco2_msgs::JointVelocityConstPtr& msg)
{
    AngularPosition velocity;
    velocity.InitStruct();

    velocity.Actuators.Actuator1 = msg->joint1;
    velocity.Actuators.Actuator2 = msg->joint2;
    velocity.Actuators.Actuator3 = msg->joint3;
    velocity.Actuators.Actuator4 = msg->joint4;
    velocity.Actuators.Actuator5 = msg->joint5;
    velocity.Actuators.Actuator6 = msg->joint6;

    velocity.Fingers.Finger1 = 0;
    velocity.Fingers.Finger2 = 0;
    velocity.Fingers.Finger3 = 0;

    DataConversion::to_degrees(velocity);

    controller_.setAngularVelocity(velocity);

    last_command_ = ros::Time::now();
}

void Jaco2DriverNode::fingerVelocityCb(const jaco2_msgs::FingerPositionConstPtr &msg)
{
    AngularPosition velocity;
    velocity.InitStruct();

    velocity.Actuators.Actuator1 = 0;
    velocity.Actuators.Actuator2 = 0;
    velocity.Actuators.Actuator3 = 0;
    velocity.Actuators.Actuator4 = 0;
    velocity.Actuators.Actuator5 = 0;
    velocity.Actuators.Actuator6 = 0;

    velocity.Fingers.Finger1 = msg->finger1;
    velocity.Fingers.Finger2 = msg->finger2;
    velocity.Fingers.Finger3 = msg->finger3;

    controller_.setFingerVelocity(velocity);

    last_command_ = ros::Time::now();
}

void Jaco2DriverNode::stop()
{
    controller_.stop();
    ros::shutdown();
}

void Jaco2DriverNode::publishJointState()
{
    DataConversion::convert(controller_.getAngularVelocity(),jointStateMsg_.velocity);
    DataConversion::convert(controller_.getAngularPosition(),jointStateMsg_.position);
    DataConversion::convert(controller_.getAngularForce(), jointStateMsg_.effort);

    DataConversion::from_degrees(jointStateMsg_.position);
    DataConversion::normalize(jointStateMsg_.position);
    DataConversion::from_degrees(jointStateMsg_.velocity);

    jointStateMsg_.header.stamp = ros::Time::now();
    pubJointState_.publish(jointStateMsg_);


    jaco2_msgs::Jaco2JointState jaco2JointStateMsg;
    jaco2JointStateMsg.header = jointStateMsg_.header;
    jaco2JointStateMsg.name = jointStateMsg_.name;
    jaco2JointStateMsg.position = jointStateMsg_.position;
    jaco2JointStateMsg.velocity = jointStateMsg_.velocity;
    jaco2JointStateMsg.effort = jointStateMsg_.effort;

    DataConversion::convert(controller_.getAngularAcceleration(), jaco2JointStateMsg.acceleration);
    DataConversion::from_degrees(jaco2JointStateMsg.acceleration);

    pubJaco2JointState_.publish(jaco2JointStateMsg);
}

void Jaco2DriverNode::publishJointAngles()
{

    AngularPosition pos = controller_.getAngularPosition();

//        DataConversion::from_degrees(pos);

    jointAngleMsg_.joint1 = pos.Actuators.Actuator1;
    jointAngleMsg_.joint2 = pos.Actuators.Actuator2;
    jointAngleMsg_.joint3 = pos.Actuators.Actuator3;
    jointAngleMsg_.joint4 = pos.Actuators.Actuator4;
    jointAngleMsg_.joint5 = pos.Actuators.Actuator5;
    jointAngleMsg_.joint6 = pos.Actuators.Actuator6;

    pubJointAngles_.publish(jointAngleMsg_);

    AngularPosition posFing = controller_.getAngularPosition();
    jaco2_msgs::FingerPosition finger_msg;
    finger_msg.finger1 = posFing.Fingers.Finger1;
    finger_msg.finger2 = posFing.Fingers.Finger2;
    finger_msg.finger3 = posFing.Fingers.Finger3;
    pubFingerPositions_.publish(finger_msg);

}

void Jaco2DriverNode::publishSensorInfo()
{
    AngularAcceleration acc = controller_.getActuatorAcceleration(); // TODO remove acc from sensor
    std::chrono::time_point<std::chrono::high_resolution_clock>  stamp = controller_.getLastReadUpdate(READ_ACCELRATION);
    DataConversion::convert(acc,stamp,sensorMsg_.acceleration);
    if(stamp != lastTimeAccPublished_) {
        std::vector<geometry_msgs::Vector3Stamped> acc_msg(Jaco2DriverConstants::n_Jaco2Joints);
        DataConversion::convert(acc,stamp, acc_msg);
        jaco2_msgs::Jaco2Accelerometers jaco2_acc;
        jaco2_acc.lin_acc = acc_msg;
        pubJaco2LinAcc_.publish(jaco2_acc);
        lastTimeAccPublished_ = stamp;

    }
    stamp = controller_.getLastReadUpdate(READ_TORQUE_GRAVITY_FREE);
    AngularPosition torque = controller_.getAngularForceGravityFree();
    DataConversion::convert(torque.Actuators,sensorMsg_.torque);
    DataConversion::convert(stamp,sensorMsg_.torque_time);
    SensorsInfo info = controller_.getSensorInfo();
    stamp = controller_.getLastReadUpdate(READ_SENSOR_INFO);
    DataConversion::convert(stamp,sensorMsg_.temperature_time);
    sensorMsg_.temperature[0] = info.ActuatorTemp1;
    sensorMsg_.temperature[1] = info.ActuatorTemp2;
    sensorMsg_.temperature[2] = info.ActuatorTemp3;
    sensorMsg_.temperature[3] = info.ActuatorTemp4;
    sensorMsg_.temperature[4] = info.ActuatorTemp5;
    sensorMsg_.temperature[5] = info.ActuatorTemp6;
    AngularPosition current = controller_.getCurrent();
    stamp = controller_.getLastReadUpdate(READ_CURRENT);
    DataConversion::convert(stamp,sensorMsg_.current_time);
    DataConversion::convert(current.Actuators,sensorMsg_.current);

    pubSensorInfo_.publish(sensorMsg_);
}

bool Jaco2DriverNode::startServiceCallback(jaco2_msgs::Start::Request &req, jaco2_msgs::Start::Response &res)
{
    controller_.startArm();
    res.start_result = "Arm started";
    ROS_DEBUG("Arm stop requested");
    return true;
}

bool Jaco2DriverNode::stopServiceCallback(jaco2_msgs::Stop::Request &req, jaco2_msgs::Stop::Response &res)
{
    controller_.stopArm();
    res.stop_result = "Arm stopped";
    ROS_DEBUG("Arm stop requested");
    return true;
}

bool Jaco2DriverNode::homeArmServiceCallback(jaco2_msgs::HomeArm::Request &req, jaco2_msgs::HomeArm::Response &res)
{

    controller_.homeArm();
    res.homearm_result = "JACO ARM HAS BEEN RETURNED HOME";
    return true;
}

bool Jaco2DriverNode::setTorqueZeroCallback(jaco2_msgs::SetTorqueZero::Request &req, jaco2_msgs::SetTorqueZero::Response &res)
{
    controller_.setTorqueZero(req.actuator);
    sleep(2.0);
    while(!controller_.serviceDone())
    {
        usleep(10000);
        ROS_INFO("Waiting");
    }
    res.result = controller_.getSetTorqueZeroResult();
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco_arm_driver", ros::init_options::NoSigintHandler);

    signal(SIGINT, siginthandler);

    Jaco2DriverNode node;
    ros::Rate r(65);
    bool driver_ok = true;
    while(ros::ok() && driver_ok)
    {
        driver_ok = node.tick();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

