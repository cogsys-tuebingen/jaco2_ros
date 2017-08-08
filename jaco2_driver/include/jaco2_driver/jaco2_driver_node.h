#ifndef JACO2_DRIVER_NODE_H
#define JACO2_DRIVER_NODE_H
//System
#include <signal.h>
#include <atomic>
//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
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
    bool gravityCompCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool admittanceControlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool shutdownServiceCb(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response& res);


    void dynamicReconfigureCb(jaco2_driver::jaco2_driver_configureConfig &config, uint32_t level);

private:
    ros::NodeHandle nh_;

    ros::NodeHandle private_nh_;

    Jaco2Driver driver_;

    ros::Subscriber subJointVelocity_;
    ros::Subscriber subFingerVelocity_;
    ros::Subscriber subJointTorque_;

    ros::Publisher pubJointState_;
    ros::Publisher pubJointAngles_;
    ros::Publisher pubFingerPositions_;
    ros::Publisher pubSensorInfo_;
    ros::Publisher pubJaco2JointState_;
    ros::Publisher pubJaco2LinAcc_;
    ros::Publisher pubGfreeToruqes_;

    ros::ServiceServer stopService_;
    ros::ServiceServer startService_;
    ros::ServiceServer homingService_;
    ros::ServiceServer zeroTorqueService_;
    ros::ServiceServer gravityCompensationService_;
    ros::ServiceServer admittanceControlService_;
    ros::ServiceServer shutdownService_;

    actionlib::SimpleActionServer<jaco2_msgs::ArmJointAnglesAction> actionAngleServer_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> trajServer_;
    actionlib::SimpleActionServer<jaco2_msgs::GripperControlAction> graspServer_;
    actionlib::SimpleActionServer<jaco2_msgs::SetFingersPositionAction> fingerServer_;
    actionlib::SimpleActionServer<jaco2_msgs::ArmJointAnglesAction> blockingAngleServer_;


    ros::Time last_command_;

    std::string tf_prefix_;
    std::vector<std::string> joint_names_;
    sensor_msgs::JointState jointStateMsg_;
    jaco2_msgs::Jaco2JointState jaco2JointStateMsg_;
    jaco2_msgs::JointAngles jointAngleMsg_;
    jaco2_msgs::Jaco2Sensor sensorMsg_;

    std::atomic_bool actionAngleServerRunning_;
    bool trajServerRunning_;
    bool gripperServerRunning_;
    bool fingerServerRunning_;
    bool rightArm_;
    bool ok_;
    std::string serial_;
    dynamic_reconfigure::Server<jaco2_driver::jaco2_driver_configureConfig> paramServer_;
    dynamic_reconfigure::Server<jaco2_driver::jaco2_driver_configureConfig>::CallbackType f_;

    jaco2_data::TimeStamp  lastTimeAccPublished_;
    jaco2_data::TimeStamp  lastTimeJsPublished_;

    double j6o_;
    std::string dyn_model_calib_file_path_;
};
#endif // JACO2_DRIVER_NODE_H

