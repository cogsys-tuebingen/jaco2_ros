#ifndef JACO2_DRIVER_NODE_H
#define JACO2_DRIVER_NODE_H
//System
#include <signal.h>
#include <atomic>
#include <memory>
//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
// JACO2 DRIVER
#include <kinova/KinovaTypes.h>
#include <jaco2_driver/jaco2_driver.h>
#include <jaco2_driver/jaco2_driver_configureConfig.h>
#include <jaco2_driver/manipulator_info.h>
//JACO2 MSGS
#include <jaco2_msgs/ArmJointAnglesAction.h>
#include <jaco2_msgs/SetFingersPositionAction.h>
#include <jaco2_msgs/GripperControlAction.h>
#include <jaco2_msgs/Start.h>
#include <jaco2_msgs/Stop.h>
#include <jaco2_msgs/HomeArm.h>
#include <jaco2_msgs/SetTorqueZero.h>
#include <jaco2_msgs/Jaco2Sensor.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <jaco2_msgs/SetPayloadParams.h>
#include <jaco2_msgs/SetTorqueExpertMode.h>

class Jaco2DriverNode
{
public:
    // Maximum number of joints on Jaco-like robots:
    static const int     JACO_JOINTS_COUNT = 9;
public:
    Jaco2DriverNode();
    void stop();
    bool tick();

private:
    void jointVelocityCb(const jaco2_msgs::JointVelocityConstPtr& msg);
    void fingerVelocityCb(const jaco2_msgs::FingerPositionConstPtr &msg);
    void cartesianVelocityCb(const geometry_msgs::TwistConstPtr& msg);
    void jointTorqueCb(const jaco2_msgs::JointAnglesConstPtr& msg);

    void publishJointState();
    void publishJointAngles();
    void publishSensorInfo();
    void actionAngleGoalCb();
    void trajGoalCb();
    void gripperGoalCb();
    void fingerGoalCb();
    void blockingAngleGoalCb();

    bool stopServiceCallback(jaco2_msgs::Stop::Request &req, jaco2_msgs::Stop::Response &res);
    bool startServiceCallback(jaco2_msgs::Start::Request &req, jaco2_msgs::Start::Response &res);
    bool homeArmServiceCallback(jaco2_msgs::HomeArm::Request &req, jaco2_msgs::HomeArm::Response &res);
    bool setTorqueZeroCallback(jaco2_msgs::SetTorqueZero::Request &req, jaco2_msgs::SetTorqueZero::Response & res);
    bool setPayloadCallback(jaco2_msgs::SetPayloadParams::Request & req, jaco2_msgs::SetPayloadParams::Response& res);
    bool gravityCompCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool admittanceControlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool shutdownServiceCb(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response& res);
    bool setTorqueExportMode(jaco2_msgs::SetTorqueExpertMode::Request& req, jaco2_msgs::SetTorqueExpertMode::Response& res);
    bool activateTorqueControlCb(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response& res);

    void dynamicReconfigureCb(jaco2_driver::jaco2_driver_configureConfig &config, uint32_t level);

private:
    ros::NodeHandle nh_;

    ros::NodeHandle private_nh_;

    std::shared_ptr<Jaco2Driver> driver_;

    ros::Subscriber sub_joint_velocity_;
    ros::Subscriber sub_finger_velocity_;
    ros::Subscriber sub_cat_velocity_;
    ros::Subscriber sub_joint_torque_;

    ros::Publisher pub_joint_state_;
    ros::Publisher pub_joint_angles_;
    ros::Publisher pub_finger_positions_;
    ros::Publisher pub_sensor_info_;
    ros::Publisher pub_jaco_joint_state_;
    ros::Publisher pub_jaco_lin_acc_;
    ros::Publisher pub_g_free_toruqes_;

    ros::ServiceServer stop_service_;
    ros::ServiceServer start_service_;
    ros::ServiceServer homing_service_;
    ros::ServiceServer zero_torque_service_;
    ros::ServiceServer gravity_compensation_service_;
    ros::ServiceServer admittance_control_service_;
    ros::ServiceServer shutdown_service_;
    ros::ServiceServer activate_torque_control_;
    ros::ServiceServer set_payload_service_;
    ros::ServiceServer set_torque_expert_mode_;

    actionlib::SimpleActionServer<jaco2_msgs::ArmJointAnglesAction> action_angle_server_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> traj_server_;
    actionlib::SimpleActionServer<jaco2_msgs::GripperControlAction> grasp_server_;
    actionlib::SimpleActionServer<jaco2_msgs::SetFingersPositionAction> finger_server_;
    actionlib::SimpleActionServer<jaco2_msgs::ArmJointAnglesAction> blocking_angle_server_;


    ros::Time last_command_;

    std::string tf_prefix_;
    std::vector<std::string> joint_names_;
    sensor_msgs::JointState joint_state_msg_;
    jaco2_msgs::Jaco2JointState jaco_joint_state_msg_;
    jaco2_msgs::JointAngles joint_angle_msg_;
    jaco2_msgs::Jaco2Sensor sensor_msg_;

    std::atomic_bool action_angle_server_running_;
    bool traj_server_running_;
    bool gripper_server_running_;
    bool finger_server_running_;
    bool right_arm_;
    bool torque_control_active_;
    bool ok_;
    bool publish_fingers_;
    std::string serial_;
    dynamic_reconfigure::Server<jaco2_driver::jaco2_driver_configureConfig> param_server_;
    dynamic_reconfigure::Server<jaco2_driver::jaco2_driver_configureConfig>::CallbackType f_;

    jaco2_data::TimeStamp  last_time_acc_published_;
    jaco2_data::TimeStamp  last_time_js_published_;

    double j6o_;
    std::string dyn_model_calib_file_path_;
    std::string robot_description_;
    std::string base_link_;
    std::string tip_link_;
    std::string node_name_;
};
#endif // JACO2_DRIVER_NODE_H

