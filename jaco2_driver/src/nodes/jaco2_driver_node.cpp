#include <jaco2_driver/jaco2_driver_node.h>
#include <angles/angles.h>

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
    : private_nh_("~")
{
    pubJointState_ = private_nh_.advertise<sensor_msgs::JointState>("out/joint_states", 2);
    boost::function<void(const jaco2_msgs::JointVelocityConstPtr&)> cb = boost::bind(&Jaco2DriverNode::jointVelocityCb, this, _1);
    subJointVelocity_ = private_nh_.subscribe("in/joint_velocity", 10, cb);

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

     j6o_ = controller_.getRobotType() == 2 ? 270.0 : 260.0;
}

void Jaco2DriverNode::tick()
{
    if(!g_running_) {
        stop();
    }
    publishJointState();
}

void Jaco2DriverNode::jointVelocityCb(const jaco2_msgs::JointVelocityConstPtr& msg)
{
    TrajectoryPoint pointToSend;
    pointToSend.InitStruct();
    pointToSend.Position.Type = ANGULAR_VELOCITY;

    //We get the actual angular command of the robot.
    AngularPosition currentCommand;// = controller.getAngularVelocity();
    currentCommand.InitStruct();
    //    usleep(2000);

    pointToSend.Position.Actuators.Actuator1 = msg->joint1;
    pointToSend.Position.Actuators.Actuator2 = msg->joint2;
    pointToSend.Position.Actuators.Actuator3 = msg->joint3;
    pointToSend.Position.Actuators.Actuator4 = msg->joint4;
    pointToSend.Position.Actuators.Actuator5 = msg->joint5;
    pointToSend.Position.Actuators.Actuator6 = msg->joint6;

    pointToSend.Position.Fingers.Finger1 = currentCommand.Fingers.Finger1;
    pointToSend.Position.Fingers.Finger2 = currentCommand.Fingers.Finger2;
    pointToSend.Position.Fingers.Finger3 = currentCommand.Fingers.Finger3;

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
    for(std::size_t i = 0; i < jointStateMsg_.position.size(); ++i)
    {
        jointStateMsg_.position[i] = angles::to_degrees(angles::normalize_angle(angles::from_degrees(jointStateMsg_.position[i])));
        jointStateMsg_.velocity[i] = jointStateMsg_.velocity[i];

    }

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


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco_arm_driver", ros::init_options::NoSigintHandler);

    signal(SIGINT, siginthandler);

    Jaco2DriverNode node;
    ros::Rate r(20);

    while(ros::ok())
    {
        node.tick();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

