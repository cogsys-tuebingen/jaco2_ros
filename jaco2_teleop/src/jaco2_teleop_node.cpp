#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <jaco2_msgs/JointVelocity.h>
#include <jaco2_msgs/Start.h>
#include <jaco2_msgs/Stop.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
class Jaco2Teleop
{
public:
    Jaco2Teleop(std::string state_topic = "/jaco_arm_driver/out/joint_states",
                std::string joy_topic = "/joy",
                std::string vel_topic = "/jaco_arm_driver/in/joint_velocity",
                std::string start_srv = "/jaco2_arm_driver/in/start",
                std::string stop_srv = "/jaco2_arm_driver/in/stop",
                std::string gcomp_srv = "/jaco2_arm_driver/in/enable_gravity_compensation_mode",
                std::string ad_srv = "/jaco2_arm_driver/in/enable_admittance_mode") :
        gcomp_(false),
        admitance_(false),
        cartesian_(false),
        nh_("~"),
        rate_(50)
    {
        sub_joint_states_ = nh_.subscribe(state_topic, 2, &Jaco2Teleop::stateCb, this);
        sub_joy_ = nh_.subscribe(joy_topic,2, &Jaco2Teleop::joyCb ,this);
        pub_joint_vel = nh_.advertise<jaco2_msgs::JointVelocity>(vel_topic, 2);
        srv_start_ = nh_.serviceClient<jaco2_msgs::Start>(start_srv);
        srv_stop_ = nh_.serviceClient<jaco2_msgs::Start>(stop_srv);
        srv_gcomp_= nh_.serviceClient<std_srvs::SetBool>(gcomp_srv);
        srv_admitance_ = nh_.serviceClient<std_srvs::SetBool>(ad_srv);
    }

    void stateCb(const sensor_msgs::JointStateConstPtr& msg)
    {

    }

    void joyCb(const sensor_msgs::JoyConstPtr& msg)
    {
        if(msg->buttons[7]){// R2: STOP
            jaco2_msgs::Stop stop;
            srv_stop_.call(stop.request, stop.response);
            ROS_INFO_STREAM("API has been stoped");
        }
        if(msg->buttons[6]){// L2: START
            jaco2_msgs::Start start;
            srv_start_.call(start.request, start.response);
            ROS_INFO_STREAM("API has been released");
        }
        if(msg->buttons[0]){ // SQUARE
            gcomp_ = !gcomp_;
            std_srvs::SetBool srv;
            srv.request.data = gcomp_;
            srv_gcomp_.call(srv.request, srv.response);
            ROS_INFO_STREAM(srv.response.message);
        }
        if(msg->buttons[1]){ // X
            admitance_ = !admitance_;
            std_srvs::SetBool srv;
            srv.request.data = admitance_;
            srv_admitance_.call(srv.request, srv.response);
            ROS_INFO_STREAM(srv.response.message);
        }
        if(msg->buttons[2]){ // circle
            //            cartesian_  = !cartesian_;
            ROS_INFO_STREAM("Cartesian mode not yet implemented!");
        }
        bool move = false;
        for(std::size_t i = 0; i < 4; ++i){
            move |= std::abs(msg->axes[i]) > 0.02;
        }
        if(move){
            if(!cartesian_){
                jaco2_msgs::JointVelocity vel;
                vel.joint1 = 0;
                vel.joint2 = 0;
                vel.joint3 = msg->axes[2];
                vel.joint4 = msg->axes[3];
                vel.joint5 = 0;
                vel.joint6 = 0;
                if(!msg->buttons[4]){ // L1
                    vel.joint1 = msg->axes[0];
                    vel.joint2 = msg->axes[1];
                }
                else{
                    vel.joint5 = msg->axes[0];
                    vel.joint6 = msg->axes[1];
                }
                pub_joint_vel.publish(vel);
            }
        }

    }

    void tick()
    {
        rate_.sleep();
        ros::spinOnce();
    }

private:
    bool gcomp_;
    bool admitance_;
    bool cartesian_;
    ros::NodeHandle nh_;
    ros::Rate rate_;
    ros::Subscriber sub_joint_states_;
    ros::Subscriber sub_joy_;
    ros::Publisher  pub_joint_vel;
    ros::ServiceClient srv_start_;
    ros::ServiceClient srv_stop_;
    ros::ServiceClient srv_gcomp_;
    ros::ServiceClient srv_admitance_;
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_teleop_node");
    ros::NodeHandle nh("~");
    std::string state_topic = nh.param<std::string>("state_topic", "/jaco_arm_driver/out/joint_states");
    std::string joy_topic   = nh.param<std::string>("joy_topic", "/joy");
    std::string vel_topic   = nh.param<std::string>("vel_topic", "/jaco_arm_driver/in/joint_velocity");
    std::string start_srv   = nh.param<std::string>("start_service", "/jaco2_arm_driver/in/start");
    std::string stop_srv    = nh.param<std::string>("stop_service", "/jaco2_arm_driver/in/stop");
    std::string gcomp_srv   = nh.param<std::string>("gcomp_service", "/jaco2_arm_driver/in/enable_gravity_compensation_mode");
    std::string ad_srv      = nh.param<std::string>("admitance_service", "/jaco2_arm_driver/in/enable_admittance_mode");

    Jaco2Teleop telenode(state_topic, joy_topic, vel_topic, start_srv, stop_srv, gcomp_srv, ad_srv);

    while(ros::ok()){
          telenode.tick();
    }


    return 0;
}


