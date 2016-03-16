#include <jaco2_driver/jaco2_driver_node.h>
#include <angles/angles.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

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
      actionAngleServerRunning_(false)
{
    pubJointState_ = private_nh_.advertise<sensor_msgs::JointState>("out/joint_states", 2);
    boost::function<void(const jaco2_msgs::JointVelocityConstPtr&)> cb = boost::bind(&Jaco2DriverNode::jointVelocityCb, this, _1);
    subJointVelocity_ = private_nh_.subscribe("in/joint_velocity", 10, cb);

    actionAngleServer_.registerGoalCallback(boost::bind(&Jaco2DriverNode::actionAngleGoalCb, this));



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

//     j6o_ = controller_.getRobotType() == 2 ? 270.0 : 260.0;
}


void Jaco2DriverNode::actionAngleGoalCb()
{
    jaco2_msgs::ArmJointAnglesGoalConstPtr goal = actionAngleServer_.acceptNewGoal();
    AngularPosition position;
    convert(goal->angles,position);
    AngularPosition currentPos = controller_.getAngularPosition();

    position.Fingers = currentPos.Fingers;
    actionAngleServerRunning_ = true;
    actionAngleCmdSent_ = false;
    angleCmd_ = position;
    controller_.setAngularPosition(position);

}

void Jaco2DriverNode::tick()
{
    if(!g_running_) {
        stop();
    }
    if(actionAngleServerRunning_)
    {
//        if(actionAngleCmdSent_)
//        {
//            controller_.setAngularPosition(angleCmd_);
//            actionAngleCmdSent_ = true;
//        }
        jaco2_msgs::ArmJointAnglesFeedback feedback;
        AngularPosition angles = controller_.getAngularPosition();
        convert(angles,feedback.angles);
        actionAngleServer_.publishFeedback(feedback);
        if(controller_.reachedAngularGoal())
        {
            actionAngleServerRunning_ = false;
            jaco2_msgs::ArmJointAnglesResult result;
            result.val = jaco2_msgs::ArmJointAnglesResult::SUCCESSFUL;
            actionAngleServer_.setSucceeded(result);
        }
        if(actionAngleServer_.isPreemptRequested() )
        {
            jaco2_msgs::ArmJointAnglesResult result;
            result.val = jaco2_msgs::ArmJointAnglesResult::PREEMPT;
            actionAngleServerRunning_ = false;
            actionAngleServer_.setPreempted();
        }
    }
    publishJointState();
}

void Jaco2DriverNode::jointVelocityCb(const jaco2_msgs::JointVelocityConstPtr& msg)
{
    AngularPosition pointToSend;
    pointToSend.InitStruct();

    pointToSend.Actuators.Actuator1 = msg->joint1;
    pointToSend.Actuators.Actuator2 = msg->joint2;
    pointToSend.Actuators.Actuator3 = msg->joint3;
    pointToSend.Actuators.Actuator4 = msg->joint4;
    pointToSend.Actuators.Actuator5 = msg->joint5;
    pointToSend.Actuators.Actuator6 = msg->joint6;

    pointToSend.Fingers.Finger1 = 0;
    pointToSend.Fingers.Finger2 = 0;
    pointToSend.Fingers.Finger3 = 0;

    controller_.setAngularVelocity(pointToSend);

    last_command_ = ros::Time::now();
}

void Jaco2DriverNode::stop()
{
    controller_.stop();
    ros::shutdown();
}

void Jaco2DriverNode::publishJointState()
{
    convert(controller_.getAngularVelocity(),jointStateMsg_.velocity);
    convert(controller_.getAngularPosition(),jointStateMsg_.position);
    convert(controller_.getAngularForce(), jointStateMsg_.effort);
//    for(std::size_t i = 0; i < jointStateMsg_.position.size(); ++i)
//    {
//        jointStateMsg_.position[i] = angles::to_degrees(angles::normalize_angle(angles::from_degrees(jointStateMsg_.position[i])));
//        jointStateMsg_.velocity[i] = jointStateMsg_.velocity[i];

//    }

    jointStateMsg_.header.stamp = ros::Time::now();
    pubJointState_.publish(jointStateMsg_);
}

void Jaco2DriverNode::convert(const AngularPosition &in, std::vector<double> &out)
{
    out.resize(Jaco2DriverNode::JACO_JOINTS_COUNT);
    out[0] = in.Actuators.Actuator1;
    out[1] = in.Actuators.Actuator2;
    out[2] = in.Actuators.Actuator3;
    out[3] = in.Actuators.Actuator4;
    out[4] = in.Actuators.Actuator5;
    out[5] = in.Actuators.Actuator6;

    out[6] = in.Fingers.Finger1;
    out[7] = in.Fingers.Finger2;
    out[8] = in.Fingers.Finger3;
}

void Jaco2DriverNode::convert(const AngularPosition &in, jaco2_msgs::JointAngles &out)
{
    out.joint1 = in.Actuators.Actuator1;
    out.joint2 = in.Actuators.Actuator2;
    out.joint3 = in.Actuators.Actuator3;
    out.joint4 = in.Actuators.Actuator4;
    out.joint5 = in.Actuators.Actuator5;
    out.joint6 = in.Actuators.Actuator6;
}

void Jaco2DriverNode::convert(const jaco2_msgs::JointAngles &in, AngularPosition &out)
{
    out.InitStruct();
    out.Actuators.Actuator1 = in.joint1;
    out.Actuators.Actuator2 = in.joint2;
    out.Actuators.Actuator3 = in.joint3;
    out.Actuators.Actuator4 = in.joint4;
    out.Actuators.Actuator5 = in.joint5;
    out.Actuators.Actuator6 = in.joint6;
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

