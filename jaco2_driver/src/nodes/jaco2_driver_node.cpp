//ROS
#include <ros/ros.h>
// JACO2 DRIVER
#include <jaco2_driver/jaco2_driver.h>
#include <signal.h>

namespace {
bool g_running_ = true;
void siginthandler(int)
{
    g_running_ = false;
    std::exit(0);
}

}

class Jaco2DriverNode
{
public:
    Jaco2DriverNode()
        : private_nh("~")
    {
        //    pubJointState_ = nodeHandle_.advertise<sensor_msgs::JointState>("out/joint_states", 2);
        boost::function<void(const jaco2_msgs::JointVelocityConstPtr&)> cb = boost::bind(&Jaco2DriverNode::jointVelocityCb, this, _1);
        subJointVelocity_ = private_nh.subscribe("in/joint_velocity", 10, cb);
    }

    void tick()
    {
        if(!g_running_) {
            stop();
        }

        ros::Time now = ros::Time::now();
        ros::Duration time_since_last_msg = now - last_command_;

        if(time_since_last_msg > ros::Duration(0.1)) {
            TrajectoryPoint zero;
            zero.InitStruct();
            controller.setAngularVelocity(zero);
        }
    }

    void jointVelocityCb(const jaco2_msgs::JointVelocityConstPtr& msg)
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

        controller.setAngularVelocity(pointToSend);

        last_command_ = ros::Time::now();
    }

    void stop()
    {
        controller.stop();
        ros::shutdown();
    }

private:
    ros::NodeHandle nh;

    ros::NodeHandle private_nh;


    Jaco2Driver controller;

    ros::Subscriber subJointVelocity_;
    ros::Publisher pubJointState_;
    ros::Time last_command_;
};

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

