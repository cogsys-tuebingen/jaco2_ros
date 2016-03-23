#include <jaco2_driver/jaco2_driver_node.h>
#include <angles/angles.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <jaco2_driver/data_conversion.h>
#include <jaco2_msgs/FingerPosition.h>

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
      gripperEffortServer_(private_nh_, "gripper_command", false),
      fingerServer_(private_nh_,"finger_joint_angles",false),
      actionAngleServerRunning_(false),
      trajServerRunning_(false),
      gripperServerRunning_(false),
      fingerServerRunning_(false)
{
    pubJointState_ = private_nh_.advertise<sensor_msgs::JointState>("out/joint_states", 2);
    pubJointAngles_ = private_nh_.advertise<jaco2_msgs::JointAngles>("out/joint_angles",2);
    pubFingerPositions_ = private_nh_.advertise<jaco2_msgs::FingerPosition>("out/finger_positions",2);
    boost::function<void(const jaco2_msgs::JointVelocityConstPtr&)> cb = boost::bind(&Jaco2DriverNode::jointVelocityCb, this, _1);
    subJointVelocity_ = private_nh_.subscribe("in/joint_velocity", 10, cb);

    actionAngleServer_.registerGoalCallback(boost::bind(&Jaco2DriverNode::actionAngleGoalCb, this));
    trajServer_.registerGoalCallback(boost::bind(&Jaco2DriverNode::trajGoalCb, this));
    gripperEffortServer_.registerGoalCallback(boost::bind(&Jaco2DriverNode::gripperGoalCb, this));
    fingerServer_.registerGoalCallback(boost::bind(&Jaco2DriverNode::fingerGoalCb,this));

    f_ = boost::bind(&Jaco2DriverNode::dynamicReconfigureCb, this, _1, _2);
    paramServer_.setCallback(f_);

    private_nh_.param<std::string>("tf_prefix", tf_prefix_, "jaco_");

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

    actionAngleServer_.start();
    trajServer_.start();
    gripperEffortServer_.start();
    fingerServer_.start();

    //     j6o_ = controller_.getRobotType() == 2 ? 270.0 : 260.0;
}


void Jaco2DriverNode::actionAngleGoalCb()
{
    jaco2_msgs::ArmJointAnglesGoalConstPtr goal = actionAngleServer_.acceptNewGoal();
    AngularPosition position;
    DataConversion::convert(goal->angles,position);
    AngularPosition currentPos = controller_.getAngularPosition();

    position.Fingers = currentPos.Fingers;
    DataConversion::shiftAngleDriver(position);
    actionAngleServerRunning_ = true;
    controller_.setAngularPosition(position);
}

void Jaco2DriverNode::trajGoalCb()
{
    control_msgs::FollowJointTrajectoryGoalConstPtr goal = trajServer_.acceptNewGoal();
    trajectory_msgs::JointTrajectory traj_msgs = goal->trajectory;

    for(std::size_t i = 0; i < traj_msgs.points.size(); ++ i)
    {
        DataConversion::shiftAngleDriverToDegrees(traj_msgs.points[i].positions);
        DataConversion::transformVelAndAccToDegrees(traj_msgs.points[i].velocities);
        DataConversion::transformVelAndAccToDegrees(traj_msgs.points[i].accelerations);
    }

    JointTrajectory driver_trajectory;
    DataConversion::convert(traj_msgs,driver_trajectory);

    trajServerRunning_ = true;
    controller_.setTrajectory(driver_trajectory);
}

void Jaco2DriverNode::gripperGoalCb()
{
    control_msgs::GripperCommandGoalConstPtr goal = gripperEffortServer_.acceptNewGoal();
    gripperServerRunning_ = true;
    controller_.setGripperEffort(goal->command.max_effort);
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

void Jaco2DriverNode::tick()
{
    if(!g_running_) {
        stop();
    }
    publishJointState();
    publishJointAngles();
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
//        control_msgs::GripperCommandFeedback feedback; // TODO
        control_msgs::GripperCommandResult result;
        if(controller_.reachedGoal())
        {
            result.reached_goal = true;
            gripperEffortServer_.setSucceeded(result);
            controller_.finish();
        }
        else if(gripperEffortServer_.isPreemptRequested())
        {
            result.reached_goal = false;
            gripperEffortServer_.setPreempted(result);
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
            fingerServer_.setSucceeded(result);
            controller_.finish();
        }
        else if(fingerServer_.isPreemptRequested())
        {
            result.error_code = jaco2_msgs::SetFingersPositionResult::PREMPTED;
            fingerServer_.setSucceeded(result);
            controller_.finish();
        }
    }
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
    controller_.setGripperPGain(config.gripper_p_gain_finger_1,config.gripper_p_gain_finger_2,config.gripper_p_gain_finger_3);
    controller_.setGripperIGain(config.gripper_i_gain_finger_1,config.gripper_i_gain_finger_2,config.gripper_i_gain_finger_3);
    controller_.setGripperDGain(config.gripper_d_gain_finger_1,config.gripper_d_gain_finger_2,config.gripper_d_gain_finger_3);
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

    DataConversion::transformVelAndAcc(velocity);

    controller_.setAngularVelocity(velocity);

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

    DataConversion::shiftAngleToROSRadians(jointStateMsg_.position);
    DataConversion::transformVelAndAccToRadian(jointStateMsg_.velocity);

    jointStateMsg_.header.stamp = ros::Time::now();
    pubJointState_.publish(jointStateMsg_);


    AngularPosition pos = controller_.getAngularPosition();
    jaco2_msgs::FingerPosition finger_msg;
    finger_msg.finger1 = pos.Fingers.Finger1;
    finger_msg.finger2 = pos.Fingers.Finger2;
    finger_msg.finger3 = pos.Fingers.Finger3;
    pubFingerPositions_.publish(finger_msg);
}

void Jaco2DriverNode::publishJointAngles()
{

    AngularPosition pos = controller_.getAngularPosition();

    DataConversion::shiftAngleToROS(pos);

    jointAngleMsg_.joint1 = pos.Actuators.Actuator1;
    jointAngleMsg_.joint2 = pos.Actuators.Actuator2;
    jointAngleMsg_.joint3 = pos.Actuators.Actuator3;
    jointAngleMsg_.joint4 = pos.Actuators.Actuator4;
    jointAngleMsg_.joint5 = pos.Actuators.Actuator5;
    jointAngleMsg_.joint6 = pos.Actuators.Actuator6;

    pubJointAngles_.publish(jointAngleMsg_);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco_arm_driver", ros::init_options::NoSigintHandler);

    signal(SIGINT, siginthandler);

    Jaco2DriverNode node;
    ros::Rate r(80);

    while(ros::ok())
    {
        node.tick();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

