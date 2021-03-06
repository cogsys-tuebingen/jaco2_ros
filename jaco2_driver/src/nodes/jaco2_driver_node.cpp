#include <jaco2_driver/jaco2_driver_node.h>
#include <angles/angles.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <jaco2_driver/data_conversion.h>
#include <jaco2_msgs/FingerPosition.h>
#include <jaco2_msgs/Jaco2Sensor.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <jaco2_msgs/Jaco2Accelerometers.h>
#include <jaco2_driver/jaco2_driver_constants.h>
#include <jaco2_data/gravity_params.hpp>
#include <jaco2_data/velocity_calibration.hpp>
#include <jaco2_msgs/Jaco2GfreeTorques.h>
#include <jaco2_msgs_conversion/jaco2_ros_msg_conversion.h>
#include <kinova/KinovaTypes.h>

Jaco2DriverNode::Jaco2DriverNode()
    : private_nh_("~"),
      action_angle_server_(private_nh_, "arm_joint_angles",false),
      traj_server_(private_nh_,"follow_joint_trajectory/manipulator",false),
      grasp_server_(private_nh_, "gripper_command", false),
      finger_server_(private_nh_,"finger_joint_angles",false),
      blocking_angle_server_(private_nh_, "arm_joint_angles_blocking", false),
      action_angle_server_running_(false),
      traj_server_running_(false),
      gripper_server_running_(false),
      finger_server_running_(false),
      right_arm_(true),
      torque_control_active_(false),
      ok_(true),
      dyn_model_calib_file_path_(""),
      robot_description_("/robot_description"),
      base_link_("jaco_link_base"),
      tip_link_("jaco_link_tip")
{
    node_name_ = ros::this_node::getName();
    if(node_name_.front() == '/'){
        node_name_.erase(node_name_.begin());
    }
    ROS_INFO_STREAM("Node Name: " <<  node_name_);
    driver_ = std::shared_ptr<Jaco2Driver>(new Jaco2Driver(node_name_));

    pub_joint_state_ = private_nh_.advertise<sensor_msgs::JointState>("out/joint_states", 2);
    pub_joint_angles_ = private_nh_.advertise<jaco2_msgs::JointAngles>("out/joint_angles",2);
    pub_finger_positions_ = private_nh_.advertise<jaco2_msgs::FingerPosition>("out/finger_positions",2);
    pub_sensor_info_ = private_nh_.advertise<jaco2_msgs::Jaco2Sensor>("out/sensor_info",2);
    pub_jaco_joint_state_ = private_nh_.advertise<jaco2_msgs::Jaco2JointState>("out/joint_state_acc", 2);
    pub_jaco_lin_acc_ = private_nh_.advertise<jaco2_msgs::Jaco2Accelerometers>("out/accelerometers",2);
    pub_g_free_toruqes_ = private_nh_.advertise<jaco2_msgs::Jaco2GfreeTorques>("out/torques_g_free",2);

    right_arm_ = private_nh_.param<bool>("right_arm", true);
    bool move_home = private_nh_.param<bool>("move_home",true);
    publish_fingers_ = private_nh_.param<bool>("publish_fingers",true);

    if(right_arm_){
        ROS_INFO_STREAM("Right arm");
    }
    else{
        ROS_INFO_STREAM("Left arm");
    }
    std::string serial_ = private_nh_.param<std::string>("jaco_serial", std::string(""));
    std::string tf_prefix_ = private_nh_.param<std::string>("tf_prefix", "jaco_");
    std::string vel_controller_type = private_nh_.param<std::string>("velocity_controller", Jaco2DriverConstants::velocity_controller);
    std::string traj_controller_type = private_nh_.param<std::string>("trajectory_controller", Jaco2DriverConstants::trajectory_p2p_velocity_controller);
    std::string conntection_type = private_nh_.param<std::string>("connection_type", "USB");

    driver_->setVelocityController(vel_controller_type);
    driver_->setTrajectoryController(traj_controller_type);

    sub_joint_velocity_ = private_nh_.subscribe("in/joint_velocity", 10, &Jaco2DriverNode::jointVelocityCb, this);
    sub_finger_velocity_ = private_nh_.subscribe("in/finger_velocity", 10, &Jaco2DriverNode::fingerVelocityCb, this);
    sub_cat_velocity_ = private_nh_.subscribe("in/cartesian_velocity", 10, &Jaco2DriverNode::cartesianVelocityCb, this);
    sub_joint_torque_ = private_nh_.subscribe("in/joint_torques", 1, &Jaco2DriverNode::jointTorqueCb, this);

    stop_service_ = private_nh_.advertiseService("in/stop", &Jaco2DriverNode::stopServiceCallback, this);
    start_service_ = private_nh_.advertiseService("in/start", &Jaco2DriverNode::startServiceCallback, this);
    homing_service_ = private_nh_.advertiseService("in/home_arm", &Jaco2DriverNode::homeArmServiceCallback, this);
    zero_torque_service_ = private_nh_.advertiseService("in/set_torque_zero", &Jaco2DriverNode::setTorqueZeroCallback, this);
    set_payload_service_ = private_nh_.advertiseService("in/set_payload", &Jaco2DriverNode::setPayloadCallback, this);
    gravity_compensation_service_ = private_nh_.advertiseService("in/enable_gravity_compensation_mode", &Jaco2DriverNode::gravityCompCallback, this);
    admittance_control_service_ = private_nh_.advertiseService("in/enable_admittance_mode", &Jaco2DriverNode::admittanceControlCallback, this);
    shutdown_service_ = private_nh_.advertiseService("in/shutdown", &Jaco2DriverNode::shutdownServiceCb, this);

#ifdef TORQUE_EXPERT
    set_torque_expert_mode_ = private_nh_.advertiseService("in/set_torque_expert_mode", &Jaco2DriverNode::setTorqueExportMode, this);
#endif

    activate_torque_control_ = private_nh_.advertiseService("in/toggle_torque_control", &Jaco2DriverNode::activateTorqueControlCb, this);

    action_angle_server_.registerGoalCallback(boost::bind(&Jaco2DriverNode::actionAngleGoalCb, this));
    traj_server_.registerGoalCallback(boost::bind(&Jaco2DriverNode::trajGoalCb, this));
    grasp_server_.registerGoalCallback(boost::bind(&Jaco2DriverNode::gripperGoalCb, this));
    finger_server_.registerGoalCallback(boost::bind(&Jaco2DriverNode::fingerGoalCb,this));
    blocking_angle_server_.registerGoalCallback(boost::bind(&Jaco2DriverNode::blockingAngleGoalCb, this));

    dyn_model_calib_file_path_ = private_nh_.param<std::string>("jaco_dynamic_model_calibration_file", "");
    if(dyn_model_calib_file_path_ != ""){
        ROS_INFO_STREAM("Using dynamic model calibration.");
    }
    std::string rdes = private_nh_.param<std::string>("robot_model_param_sever", "");
    if(rdes != ""){
        robot_description_ = rdes;
    }
    ROS_INFO_STREAM("Robot model: " << robot_description_);

    std::string link_base = private_nh_.param<std::string>("robot_model_base_link", "");
    if(link_base != ""){
        base_link_ = link_base;
    }
    std::string tip_link  = private_nh_.param<std::string>("robot_model_tip_link", "jaco_link_hand");
    if(tip_link != ""){
        tip_link_ = tip_link;
    }


    f_ = boost::bind(&Jaco2DriverNode::dynamicReconfigureCb, this, _1, _2);
    param_server_.setCallback(f_);

    joint_state_msg_.position.resize(JACO_JOINTS_COUNT);
    joint_state_msg_.velocity.resize(JACO_JOINTS_COUNT);
    joint_state_msg_.effort.resize(JACO_JOINTS_COUNT);
    joint_names_.resize(JACO_JOINTS_COUNT);
    joint_names_[0] = tf_prefix_ + "joint_1";
    joint_names_[1] = tf_prefix_ + "joint_2";
    joint_names_[2] = tf_prefix_ + "joint_3";
    joint_names_[3] = tf_prefix_ + "joint_4";
    joint_names_[4] = tf_prefix_ + "joint_5";
    joint_names_[5] = tf_prefix_ + "joint_6";
    joint_names_[6] = tf_prefix_ + "joint_finger_1";
    joint_names_[7] = tf_prefix_ + "joint_finger_2";
    joint_names_[8] = tf_prefix_ + "joint_finger_3";
    joint_state_msg_.name = joint_names_;
    driver_->setJointNames(joint_names_);

    sensor_msg_.name.resize(Jaco2DriverConstants::n_Jaco2Joints);
    sensor_msg_.temperature.resize(Jaco2DriverConstants::n_Jaco2Joints);
    sensor_msg_.current.resize(Jaco2DriverConstants::n_Jaco2Joints);
    sensor_msg_.name[0] = tf_prefix_ + "joint_1";
    sensor_msg_.name[1] = tf_prefix_ + "joint_2";
    sensor_msg_.name[2] = tf_prefix_ + "joint_3";
    sensor_msg_.name[3] = tf_prefix_ + "joint_4";
    sensor_msg_.name[4] = tf_prefix_ + "joint_5";
    sensor_msg_.name[5] = tf_prefix_ + "joint_6";


    bool use_usb = !(conntection_type == "ethernet");
    if(use_usb){
      ROS_INFO("Using USB connection.");
    } else{
      ROS_INFO("Using Ethernet connection.");
    }

    EthernetConfig econf;
    econf.local_ip_address = private_nh_.param<std::string>("ethernet/local_machine_IP", "192.168.100.100");
    econf.robot_ip_address = private_nh_.param<std::string>("ethernet/local_robot_IP", "192.168.100.101");
    econf.subnet_mask = private_nh_.param<std::string>("ethernet/subnet_mask", "255.255.255.0");
    econf.local_cmd_port = private_nh_.param<int>("ethernet/local_cmd_port", 25015);
    econf.local_bcast_port = private_nh_.param<int>("ethernet/local_broadcast_port", 25025);

    bool init_fingers = private_nh_.param<bool>("init_fingers",true);

    bool init = driver_->initialize(econf, serial_, right_arm_, move_home, init_fingers, use_usb);
    if(!init){
        ROS_ERROR_STREAM("Jaco 2 could not be initialized for device: " << serial_);
    }
    ok_ = init;


    bool use_accel_calib = private_nh_.param<bool>("jaco_use_accelerometer_calib", false);
    if(use_accel_calib) {
        ROS_INFO_STREAM("Using accelerometer calibration.");
        std::string acc_calib_file;
        private_nh_.param<std::string>("jaco_accelerometer_calibration_file", acc_calib_file, "");
        std::vector<Jaco2Calibration::AccelerometerCalibrationParam> acc_params;
        Jaco2Calibration::loadAccCalib(acc_calib_file, acc_params);
        driver_->setAccelerometerCalibration(acc_params);
    }

//    bool use_torque_calib = private_nh_.param<bool>("jaco_use_torque_calib", false);
//    if(use_torque_calib){

//        std::string torque_calib_file = private_nh_.param<std::string>("jaco_torque_calibration_file", "");
//        try{
//            Jaco2Calibration::TorqueOffsetLut lut;
//            lut.load(torque_calib_file);
//            driver_->setTorqueCalibration(lut);
//            ROS_INFO_STREAM("Using torque LUT calibration.");
//        }
//        catch(const std::runtime_error& e){
//            Jaco2Calibration::TorqueOffsetCalibration sine_calib;
//            sine_calib.load(torque_calib_file);
//            driver_->setTorqueCalibration(sine_calib);
//            ROS_INFO_STREAM("Using torque SINE calibration.");
//        }
//        catch(...){
//            ROS_ERROR_STREAM("Unkown failure!!!");
//            throw;
//        }
//    }

    std::string velocity_calib_file = private_nh_.param<std::string>("jaco_velocity_calibration_file", "");
    if(velocity_calib_file != ""){
        ROS_INFO_STREAM("Using velocity calibration");
        Jaco2Calibration::VelocityCalibrationParams v_params;
        Jaco2Calibration::load(velocity_calib_file,v_params);
        driver_->setVelocitySensorCalibration(v_params.parameter);
    }

    std::string gravity_calib_file = private_nh_.param<std::string>("jaco_gravity_calibration_file", "");
    if(gravity_calib_file != ""){
        ROS_INFO_STREAM("Using optimal gravity parameters.");
        Jaco2Calibration::ApiGravitationalParams g_params;
        Jaco2Calibration::load(gravity_calib_file, g_params);
        bool success = driver_->setGravityParams(g_params);
        if(!success){
            ROS_ERROR_STREAM("Wrong number of gravity parameters in file " << gravity_calib_file
                             <<". Or setting the parameters failed. Using default parameters instead.");

        }
    }


    action_angle_server_.start();
    traj_server_.start();
    grasp_server_.start();
    finger_server_.start();
    blocking_angle_server_.start();

    last_time_acc_published_.now();
    if(ok_){
      ROS_INFO_STREAM("Initialization successfully completed. You can use the Jaco2 now.");
    } else{
      ROS_INFO_STREAM("Initialization failed. Check connection, parameters and retry.");
    }
}


void Jaco2DriverNode::actionAngleGoalCb()
{
    if(action_angle_server_.isActive()){
        //        actionAngleServer_.setPreempted();
        driver_->finish();
    }
    //    if(!controller_.reachedGoal()){
    //        controller_.finish();
    //    }

    jaco2_msgs::ArmJointAnglesGoalConstPtr goal = action_angle_server_.acceptNewGoal();
    AngularPosition position;
    DataConversion::convert(goal->angles,position);
    AngularPosition currentPos = driver_->getAngularPosition();

    position.Fingers = currentPos.Fingers;
    if(goal->type == jaco2_msgs::ArmJointAnglesGoal::RADIAN){
        DataConversion::to_degrees(position);
    }

    action_angle_server_running_ = true;
    driver_->setAngularPosition(position);

}

void Jaco2DriverNode::trajGoalCb()
{
    control_msgs::FollowJointTrajectoryGoalConstPtr goal = traj_server_.acceptNewGoal();
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

        traj_server_running_ = true;
        driver_->setTrajectory(driver_trajectory);
    }
    else{
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        traj_server_.setAborted(result);
    }
}

void Jaco2DriverNode::gripperGoalCb()
{
    jaco2_msgs::GripperControlGoalConstPtr goal = grasp_server_.acceptNewGoal();
    gripper_server_running_ = true;
    if(goal->useFinger1 && goal->useFinger2 && goal->useFinger3 || !goal->usePos)
    {
//        ROS_INFO_STREAM("Power Grasp");
        driver_->grabObj(goal->useFinger1, goal->useFinger2, goal->useFinger3);
    }
    else
    {
//        ROS_INFO_STREAM("Position Mode");
        driver_->grabObjSetUnusedFingerPos(goal->useFinger1, goal->useFinger2, goal->useFinger3,
                                          goal->posFinger1, goal->posFinger2, goal->posFinger3);
    }
}

void Jaco2DriverNode::fingerGoalCb()
{
    jaco2_msgs::SetFingersPositionGoalConstPtr goal = finger_server_.acceptNewGoal();
    AngularPosition pos;
    pos.InitStruct();
    pos.Fingers.Finger1 = goal->fingers.finger1;
    pos.Fingers.Finger2 = goal->fingers.finger2;
    pos.Fingers.Finger3 = goal->fingers.finger3;
    finger_server_running_ = true;
    driver_->setFingerPosition(pos);
}

void Jaco2DriverNode::blockingAngleGoalCb()
{
    if(blocking_angle_server_.isActive()){
        ROS_DEBUG_STREAM("Controller was busy");
        return;
    }

    jaco2_msgs::ArmJointAnglesGoalConstPtr goal = blocking_angle_server_.acceptNewGoal();
    blocking_angle_server_.isPreemptRequested();

    AngularPosition position;
    DataConversion::convert(goal->angles,position);
    AngularPosition currentPos = driver_->getAngularPosition();

    position.Fingers = currentPos.Fingers;
    if(goal->type == jaco2_msgs::ArmJointAnglesGoal::RADIAN){
        DataConversion::to_degrees(position);
    }

    driver_->setAngularPosition(position);
}

bool Jaco2DriverNode::gravityCompCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if(req.data){
        driver_->enableGravityCompensation();
        res.message = "Switched to torque control.";
    }
    else{
        driver_->disableGravityCompensation();
        res.message = "Switched to position control.";
    }

    res.success = true;
    return true;
}

bool Jaco2DriverNode::admittanceControlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if(req.data){
        driver_->enableForceControl();
        res.message = "Admittance control active.";
    }
    else{
        driver_->disableForceControl();
        res.message = "Admittance control deactivated.";
    }

    res.success = true;
    return true;
}

bool Jaco2DriverNode::shutdownServiceCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.success = true;
    res.message = "Driver shuts down";
    ok_ = false;
    return true;
}

bool Jaco2DriverNode::tick()
{
    publishJointState();
    publishJointAngles();
    publishSensorInfo();
    Jaco2Controller::Result c_res;
    if(action_angle_server_running_)
    {
        torque_control_active_ = false;
        jaco2_msgs::ArmJointAnglesFeedback feedback;
        jaco2_msgs::ArmJointAnglesResult result;
        AngularPosition angles = driver_->getAngularPosition();
        DataConversion::convert(angles,feedback.angles);
        action_angle_server_.publishFeedback(feedback);
        if(driver_->controllerFinished(c_res))
        {
            action_angle_server_running_ = false;
            result.val = jaco2_msgs::ArmJointAnglesResult::SUCCESSFUL;
            action_angle_server_.setSucceeded(result);
            driver_->finish();

        } else if(action_angle_server_.isPreemptRequested() )
        {
            result.val = jaco2_msgs::ArmJointAnglesResult::PREEMPT;
            action_angle_server_running_ = false;
            action_angle_server_.setPreempted();
            ROS_INFO_STREAM("Preempt!!!");
//            driver_->finish();
        }
    }
    if(blocking_angle_server_.isActive())
    {
        torque_control_active_ = false;
        jaco2_msgs::ArmJointAnglesFeedback feedback;
        jaco2_msgs::ArmJointAnglesResult result;
        AngularPosition angles = driver_->getAngularPosition();
        DataConversion::convert(angles,feedback.angles);
        blocking_angle_server_.publishFeedback(feedback);
        if(driver_->controllerFinished(c_res))
        {
            result.val = jaco2_msgs::ArmJointAnglesResult::SUCCESSFUL;
            blocking_angle_server_.setSucceeded(result);
//            driver_->finish();

        } else if(blocking_angle_server_.isPreemptRequested() )
        {
            result.val = jaco2_msgs::ArmJointAnglesResult::PREEMPT;
            blocking_angle_server_.setPreempted();
            ROS_INFO_STREAM("Preempt!!!");
            driver_->finish();
        }
    }
    if(traj_server_running_)
    {
        torque_control_active_ = false;
        control_msgs::FollowJointTrajectoryFeedback feedback;
        control_msgs::FollowJointTrajectoryResult result;
        AngularPosition angles = driver_->getAngularPosition();
        DataConversion::convert(angles,feedback.actual.positions);
        angles = driver_->getCurrentTrajError();
        DataConversion::convert(angles,feedback.error.positions);
        traj_server_.publishFeedback(feedback);
        if(driver_->controllerFinished(c_res))
        {
            ROS_INFO_STREAM("trajServer result " << c_res);
            switch (c_res) {
            case Jaco2Controller::Result::SUCCESS:{
                result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
                result.error_string = "Success, goal reached.";
                break;
            }
            case Jaco2Controller::Result::COLLISION:{
                result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
                result.error_string = "Trajectory lead to collision";
                break;
            }
            default:
                result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
                break;
            }
            traj_server_running_ = false;
            traj_server_.setSucceeded(result);


        } else if(action_angle_server_.isPreemptRequested() )
        {
            result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
            traj_server_running_ = false;
            traj_server_.setPreempted();
            ROS_INFO_STREAM("Preempt!!!");
            driver_->finish();
        }
    }
    if(gripper_server_running_)
    {
        jaco2_msgs::GripperControlFeedback feedback;
        AngularPosition angles = driver_->getAngularPosition();
        AngularPosition vel = driver_->getAngularVelocity();
        feedback.posFinger1 = angles.Fingers.Finger1;
        feedback.posFinger2 = angles.Fingers.Finger2;
        feedback.posFinger3 = angles.Fingers.Finger3;
        feedback.velFinger1 = vel.Fingers.Finger1;
        feedback.velFinger2 = vel.Fingers.Finger2;
        feedback.velFinger3 = vel.Fingers.Finger3;
        grasp_server_.publishFeedback(feedback);
        jaco2_msgs::GripperControlResult result;
        if(driver_->controllerFinished(c_res))
        {
            result.val = jaco2_msgs::GripperControlResult::SUCCESSFUL;
            gripper_server_running_ = false;
            grasp_server_.setSucceeded(result);
        }
        else if(grasp_server_.isPreemptRequested())
        {
            result.val = jaco2_msgs::GripperControlResult::PREEMPT;
            gripper_server_running_ = false;
            grasp_server_.setPreempted(result);
            driver_->finish();
        }
    }
    if(finger_server_running_)
    {
        jaco2_msgs::SetFingersPositionFeedback feedback;
        jaco2_msgs::SetFingersPositionResult result;
        AngularPosition pos = driver_->getAngularPosition();
        feedback.fingers.finger1 = pos.Fingers.Finger1;
        feedback.fingers.finger2 = pos.Fingers.Finger2;
        feedback.fingers.finger3 = pos.Fingers.Finger3;
        finger_server_.publishFeedback(feedback);
        if(driver_->controllerFinished(c_res))
        {
            result.error_code = jaco2_msgs::SetFingersPositionResult::SUCCESS;
            finger_server_running_ = false;
            finger_server_.setSucceeded(result);
        }
        else if(finger_server_.isPreemptRequested())
        {
            result.error_code = jaco2_msgs::SetFingersPositionResult::PREMPTED;
            finger_server_running_ = false;
            finger_server_.setSucceeded(result);
            ROS_INFO_STREAM("Preempt!!!");
            driver_->finish();
        }
    }
    return ok_;
}

void Jaco2DriverNode::dynamicReconfigureCb(jaco2_driver::jaco2_driver_configureConfig &config, uint32_t level)
{

    if(config.dynamic_model_calibration_file == "" && dyn_model_calib_file_path_ != ""){
        config.dynamic_model_calibration_file = dyn_model_calib_file_path_;
    }

    if(config.robot_model_param_sever == "" && robot_description_!= ""){
        config.robot_model_param_sever = robot_description_;
    }
    if(config.robot_model_base_link == "" && base_link_!= ""){
        config.robot_model_base_link = base_link_;
    }
    if(config.robot_model_tip_link == "" && tip_link_!= ""){
        config.robot_model_tip_link = tip_link_;
    }

    driver_->updateControllerConfig(config);


    std::vector<int> highPriQue;
    std::vector<int> lowPriQue;
    if(config.state_high_pri_pos){
        highPriQue.push_back(READ_POSITION);
    }
    if(config.state_high_pri_vel){
        highPriQue.push_back(READ_VELOCITY);
    }
    if(config.state_high_pri_acc){
        highPriQue.push_back(READ_ACCELERATION);
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
        lowPriQue.push_back(READ_ACCELERATION);
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
    driver_->setStateHighPriorityQue(highPriQue);
    driver_->setStateLowPriorityQue(lowPriQue);

}

void Jaco2DriverNode::jointVelocityCb(const jaco2_msgs::JointVelocityConstPtr& msg)
{
    torque_control_active_ = false;
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

    driver_->setAngularVelocity(velocity);

    last_command_ = ros::Time::now();
}

void Jaco2DriverNode::jointTorqueCb(const jaco2_msgs::JointAnglesConstPtr& msg)
{
    if(!torque_control_active_){
        ROS_WARN_STREAM("Commands will have no effect enable torque control first! Commands: " << *msg);
        return;
    }
    AngularPosition torque;
    torque.InitStruct();

    torque.Actuators.Actuator1 = msg->joint1;
    torque.Actuators.Actuator2 = msg->joint2;
    torque.Actuators.Actuator3 = msg->joint3;
    torque.Actuators.Actuator4 = msg->joint4;
    torque.Actuators.Actuator5 = msg->joint5;
    torque.Actuators.Actuator6 = msg->joint6;

    torque.Fingers.Finger1 = 0;
    torque.Fingers.Finger2 = 0;
    torque.Fingers.Finger3 = 0;


    driver_->setTorque(torque);

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

    driver_->setFingerVelocity(velocity);

    last_command_ = ros::Time::now();
}

void Jaco2DriverNode::cartesianVelocityCb(const geometry_msgs::TwistConstPtr &msg)
{
    CartesianPosition vel;
    vel.Coordinates.X = msg->linear.x;
    vel.Coordinates.Y = msg->linear.y;
    vel.Coordinates.Z = msg->linear.z;
    vel.Coordinates.ThetaX = msg->angular.x;
    vel.Coordinates.ThetaY = msg->angular.y;
    vel.Coordinates.ThetaZ = msg->angular.z;

    vel.Fingers.InitStruct();

    driver_->setCartesianVelocity(vel);

    last_command_ = ros::Time::now();
}

void Jaco2DriverNode::stop()
{
    driver_->stop();
    ros::shutdown();
}

void Jaco2DriverNode::publishJointState()
{
    jaco2_data::JointStateDataStamped jdata = driver_->getJointState();


    if(jdata.stamp() != last_time_js_published_){
      if(!publish_fingers_){
        jdata.popToSize(Jaco2DriverConstants::n_Jaco2Joints);
      }
        joint_state_msg_ = jaco2_msgs::JointStateConversion::data2SensorMsgs(jdata);
//        joint_state_msg_.header.frame_id = node_name_;
        pub_joint_state_.publish(joint_state_msg_);
        jaco_joint_state_msg_ = jaco2_msgs::JointStateConversion::data2Jaco2Msgs(jdata);
//        jaco_joint_state_msg_.header.frame_id = node_name_;
        pub_jaco_joint_state_.publish(jaco_joint_state_msg_);
        last_time_js_published_ = jdata.stamp();
    }

    jaco2_msgs::Jaco2GfreeTorques g_msg;
    g_msg.header = joint_state_msg_.header;
    jaco2_data::TimeStamp stamp = driver_->getLastReadUpdate(READ_TORQUE_GRAVITY_FREE);
    g_msg.header.stamp = jaco2_msgs::TimeConversion::data2ros(stamp);
    DataConversion::convert(driver_->getAngularForceGravityFree(), g_msg.effort_g_free);
    pub_g_free_toruqes_.publish(g_msg);
}

void Jaco2DriverNode::publishJointAngles()
{

    AngularPosition pos = driver_->getAngularPosition();

    joint_angle_msg_.joint1 = pos.Actuators.Actuator1;
    joint_angle_msg_.joint2 = pos.Actuators.Actuator2;
    joint_angle_msg_.joint3 = pos.Actuators.Actuator3;
    joint_angle_msg_.joint4 = pos.Actuators.Actuator4;
    joint_angle_msg_.joint5 = pos.Actuators.Actuator5;
    joint_angle_msg_.joint6 = pos.Actuators.Actuator6;

    pub_joint_angles_.publish(joint_angle_msg_);

    AngularPosition posFing = driver_->getAngularPosition();
    jaco2_msgs::FingerPosition finger_msg;
    finger_msg.finger1 = posFing.Fingers.Finger1;
    finger_msg.finger2 = posFing.Fingers.Finger2;
    finger_msg.finger3 = posFing.Fingers.Finger3;
    pub_finger_positions_.publish(finger_msg);

}

void Jaco2DriverNode::publishSensorInfo()
{
    auto stamp = driver_->getLastReadUpdate(READ_ACCELERATION);

    if(stamp != last_time_acc_published_) {

        const jaco2_data::AccelerometerData& acc_data = driver_->getAccelerometerData();
        jaco2_msgs::Jaco2Accelerometers jaco2_acc = jaco2_msgs::AccelerometerConversion::data2ros(acc_data);
        pub_jaco_lin_acc_.publish(jaco2_acc);
        last_time_acc_published_ = stamp;
    }

    stamp = driver_->getLastReadUpdate(READ_SENSOR_INFO);

    SensorsInfo info = driver_->getSensorInfo();
    DataConversion::convert(stamp.stamp, sensor_msg_.temperature_time);
    sensor_msg_.temperature[0] = info.ActuatorTemp1;
    sensor_msg_.temperature[1] = info.ActuatorTemp2;
    sensor_msg_.temperature[2] = info.ActuatorTemp3;
    sensor_msg_.temperature[3] = info.ActuatorTemp4;
    sensor_msg_.temperature[4] = info.ActuatorTemp5;
    sensor_msg_.temperature[5] = info.ActuatorTemp6;
    AngularPosition current = driver_->getCurrent();
    stamp = driver_->getLastReadUpdate(READ_CURRENT);
    DataConversion::convert(stamp.stamp, sensor_msg_.current_time);
    DataConversion::convert(current.Actuators,sensor_msg_.current);

    pub_sensor_info_.publish(sensor_msg_);
}

bool Jaco2DriverNode::startServiceCallback(jaco2_msgs::Start::Request &req, jaco2_msgs::Start::Response &res)
{
    torque_control_active_ = false;
    driver_->startArm();
    res.start_result = "Arm started";
    ROS_DEBUG("Arm stop requested");
    return true;
}

bool Jaco2DriverNode::stopServiceCallback(jaco2_msgs::Stop::Request &req, jaco2_msgs::Stop::Response &res)
{
    torque_control_active_ = false;
    driver_->stopArm();
    res.stop_result = "Arm stopped";
    ROS_DEBUG("Arm stop requested");
    return true;
}

bool Jaco2DriverNode::homeArmServiceCallback(jaco2_msgs::HomeArm::Request &req, jaco2_msgs::HomeArm::Response &res)
{
    torque_control_active_ = false;
    driver_->homeArm();
    res.homearm_result = "JACO ARM HAS BEEN RETURNED HOME";
    return true;
}

bool Jaco2DriverNode::setTorqueZeroCallback(jaco2_msgs::SetTorqueZero::Request &req, jaco2_msgs::SetTorqueZero::Response &res)
{
    driver_->setTorqueZero(req.actuator);
    sleep(2.0);
    while(!driver_->serviceDone())
    {
        usleep(10000);
        ROS_INFO("Waiting");
    }
    res.result = driver_->getSetTorqueZeroResult();
    return true;
}

bool Jaco2DriverNode::setPayloadCallback(jaco2_msgs::SetPayloadParams::Request &req, jaco2_msgs::SetPayloadParams::Response &res)
{
    jaco2_data::PayloadGravityParams p = jaco2_msgs::PayloadConversion::ros2data(req.payload);
    driver_->setPayload(p);
    while(!driver_->serviceDone()){
        usleep(10000);
        ROS_INFO("Waiting");
    }
    return true;
}

bool Jaco2DriverNode::setTorqueExportMode(jaco2_msgs::SetTorqueExpertMode::Request &req, jaco2_msgs::SetTorqueExpertMode::Response &res)
{
    if(req.password == "ExpertMode"){
#ifdef TORQUE_EXPERT
        driver_->setTorqueExpert();
#endif
        while(!driver_->serviceDone()){
            usleep(10000);
            ROS_INFO("Waiting");
        }
        res.message = "success";
        torque_control_active_ = true;
        return true;
    } else{
        res.message = "Wrong password";
        std::string gravity_calib_file = private_nh_.param<std::string>("jaco_gravity_calibration_file", "");
        if(gravity_calib_file != ""){
            ROS_INFO_STREAM("Using optimal gravity parameters.");
            Jaco2Calibration::ApiGravitationalParams g_params;
            Jaco2Calibration::load(gravity_calib_file, g_params);
            bool success = driver_->setGravityParams(g_params);
        }
        return false;
    }
}

bool Jaco2DriverNode::activateTorqueControlCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    torque_control_active_ = !torque_control_active_;
    res.message = activate_torque_control_ ? "torque control activated" : "torque control deactivated";
    res.success = true;
    return true;
}

namespace {
Jaco2DriverNode* g_driver = nullptr;
void siginthandler(int sig){
    std::cout << "shutdown due to signal: "<< sig << std::endl;
    std::terminate();
}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco_arm_driver", ros::init_options::NoSigintHandler);

    Jaco2DriverNode node;
    g_driver = &node;

    for(int i = 1; i <= 17; ++i){
        signal(i, siginthandler);
    }

    std::set_terminate([](){
        g_driver->stop();
        std::cout << "DONE" << std::endl;
        ROS_INFO_STREAM("All your base belong to us now!");
        std::exit(0);
    });

    ros::Rate r(65);

    bool driver_ok = true;
    while(ros::ok() && driver_ok)
    {
        driver_ok = node.tick();
        ros::spinOnce();
        r.sleep();
    }

    node.stop();

    return 0;
}

