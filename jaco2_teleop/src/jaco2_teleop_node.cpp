#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <jaco2_msgs/JointVelocity.h>
#include <jaco2_msgs/Start.h>
#include <jaco2_msgs/Stop.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <jaco2_kin_dyn_lib/jaco2_kinematic_model.h>
#include <jaco2_data/joint_state_data.h>
#include <jaco2_msgs_conversion/jaco2_ros_msg_conversion.h>
class Jaco2Teleop
{
public:
    enum class Function{
        STOP = 1,
        GCOMP = 2,
        ADMIT = 3,
        CART = 4
    };
    const double joystick_threshold_ = 0.1;
    const std::vector<int> default_axes = {0,1,3,4};
    const std::vector<int> default_buttons = {0,1,2,3,4,5,6};


    Jaco2Teleop(std::string state_topic = "/jaco_arm_driver/out/joint_states",
                std::string joy_topic = "/joy",
                std::string vel_topic = "/jaco_arm_driver/in/joint_velocity",
                std::string start_srv = "/jaco_arm_driver/in/start",
                std::string stop_srv = "/jaco_arm_driver/in/stop",
                std::string gcomp_srv = "/jaco_arm_driver/in/enable_gravity_compensation_mode",
                std::string ad_srv = "/jaco2_arm_driver/in/enable_admittance_mode",
                std::string robot_description = "robot_description",
                std::string base_link = "jaco_link_base",
                std::string end_eff = "link_6") :
        gcomp_(false),
        admitance_(false),
        cartesian_(false),
        send_zero_(false),
        has_joy_(false),
        nh_("~"),
        rate_(50),
        model_(robot_description, base_link, end_eff)
    {

        axes_ = nh_.param("joy_mapping/axes", default_axes);
        if(axes_.size() < default_axes.size()){
            ROS_ERROR_STREAM("At least " << default_axes.size() << " axes expected! Given " << axes_.size());
            ros::shutdown();
        }
        buttons_ = nh_.param("joy_mapping/buttons", default_buttons);
        if(buttons_.size() < default_buttons.size()){
            ROS_ERROR_STREAM("At least " << default_buttons.size() << " buttons expected! Given " << buttons_.size());
            ros::shutdown();
        }
        sub_joint_states_ = nh_.subscribe(state_topic, 2, &Jaco2Teleop::stateCb, this);
        sub_joy_ = nh_.subscribe(joy_topic,2, &Jaco2Teleop::joyCb ,this);
        pub_joint_vel = nh_.advertise<jaco2_msgs::JointVelocity>(vel_topic, 2);
        srv_start_ = nh_.serviceClient<jaco2_msgs::Start>(start_srv);
        srv_stop_ = nh_.serviceClient<jaco2_msgs::Stop>(stop_srv);
        srv_gcomp_= nh_.serviceClient<std_srvs::SetBool>(gcomp_srv);
        srv_admitance_ = nh_.serviceClient<std_srvs::SetBool>(ad_srv);
        toggle_states_[Function::STOP] = false;
        toggle_states_[Function::GCOMP] = false;
        toggle_states_[Function::ADMIT] = false;
        toggle_states_[Function::CART] = false;
        ROS_WARN_STREAM("DOES YET NOT WORK PROBABLY !!!");
    }

    void stateCb(const sensor_msgs::JointStateConstPtr& msg)
    {
       state_ = jaco2_msgs::JointStateConversion::sensorMsgs2Data(*msg);
    }

    void joyCb(const sensor_msgs::JoyConstPtr& msg)
    {
        if(!msg || msg->buttons.size() < buttons_.size()){
            return;
        }
        last_joy_ = *msg;
        has_joy_ = true;

    }

    void tick()
    {
        rate_.sleep();
        ros::spinOnce();

        if(!has_joy_){
            return;
        }

        if(last_joy_.buttons[buttons_[5]] && !toggle_states_[Function::STOP]){// R2: STOP
            jaco2_msgs::Stop stop;
            srv_stop_.call(stop.request, stop.response);
            ROS_INFO_STREAM(stop.response);
            toggle_states_[Function::STOP] = true;
        }
        if(last_joy_.buttons[buttons_[6]] && toggle_states_[Function::STOP]){// L2: START
            jaco2_msgs::Start start;
            srv_start_.call(start.request, start.response);
            ROS_INFO_STREAM(start.response);
            toggle_states_[Function::STOP] = false;
        }
        if(last_joy_.buttons[buttons_[0]] && !toggle_states_[Function::GCOMP]){ // SQUARE
            gcomp_ = !gcomp_;
            std_srvs::SetBool srv;
            srv.request.data = gcomp_;
            srv_gcomp_.call(srv.request, srv.response);
            ROS_INFO_STREAM(srv.response.message);
            toggle_states_[Function::GCOMP] = true;
            ros::Duration(0.5).sleep();
        }else{
            toggle_states_[Function::GCOMP] = false;
        }
        if(last_joy_.buttons[buttons_[1]] && !toggle_states_[Function::ADMIT]){ // X
            admitance_ = !admitance_;
            std_srvs::SetBool srv;
            srv.request.data = admitance_;
            srv_admitance_.call(srv.request, srv.response);
            ROS_INFO_STREAM(srv.response.message);
            toggle_states_[Function::ADMIT] = true;
            ros::Duration(0.5).sleep();
        }else{
            toggle_states_[Function::ADMIT] = false;
        }
        if(last_joy_.buttons[buttons_[2]]){ // circle
            toggle_states_[Function::CART] = !toggle_states_[Function::CART];
            if(toggle_states_[Function::CART]){
                ROS_INFO_STREAM("SWITCHED to CARTESIAN CONTROL.");
            }else{
                ROS_INFO_STREAM("SWITCHED to JOINT CONTROL.");
            }
            ros::Duration(0.5).sleep();

        }
        //        bool move = doMove(msg);

        if(last_joy_.buttons[buttons_[4]]){//dead man switch L1

            if(!toggle_states_[Function::CART]){
                jaco2_msgs::JointVelocity vel;
                vel.joint1 = 0;
                vel.joint2 = 0;
                vel.joint3 = 0.8*getCmd(last_joy_.axes[axes_[2]], joystick_threshold_);
                vel.joint4 = 0.8*getCmd(last_joy_.axes[axes_[3]], joystick_threshold_);
                vel.joint5 = 0;
                vel.joint6 = 0;
                if(!last_joy_.buttons[buttons_[3]]){ // L1
                    vel.joint1 = 0.8*getCmd(last_joy_.axes[axes_[0]], joystick_threshold_);
                    vel.joint2 = 0.8*getCmd(last_joy_.axes[axes_[1]], joystick_threshold_);
                }else{
                    vel.joint5 = getCmd(last_joy_.axes[axes_[0]], joystick_threshold_);
                    vel.joint6 = getCmd(last_joy_.axes[axes_[1]], joystick_threshold_);
                }
                pub_joint_vel.publish(vel);
            }else{
                KDL::Twist v;
                v.rot.Zero();
                v.vel.Zero();
//                KDL::Frame trans;
//                model_.getFKPose(state_.position, trans, model_.getTipLink());
                if(!last_joy_.buttons[buttons_[3]]){ // R1
                    KDL::Vector input(getCmd(last_joy_.axes[axes_[0]], joystick_threshold_),
                                      getCmd(last_joy_.axes[axes_[1]], joystick_threshold_),
                                      getCmd(last_joy_.axes[axes_[2]], joystick_threshold_));
                    v.vel = input;
                    ROS_INFO_STREAM("lin vel: " << v.vel.x() << ", " << v.vel.y() << ", " << v.vel.z());
//                    v = v;
                }else{
                    KDL::Vector input(getCmd(last_joy_.axes[axes_[0]], joystick_threshold_),
                                      getCmd(last_joy_.axes[axes_[1]], joystick_threshold_),
                                      getCmd(last_joy_.axes[axes_[2]], joystick_threshold_));
                    v.rot = input;
                    ROS_INFO_STREAM("rot vel: " << v.rot.x() << ", " << v.rot.y() << ", " << v.rot.z());
                }


                jaco2_data::JointData jd;
                int ec = model_.getJointVelocities(state_.position, v, jd.data);
                 if(!last_joy_.buttons[buttons_[3]]){ // R1
                     jd.data[5] += getCmd(last_joy_.axes[axes_[0]], joystick_threshold_);
                 }
                jaco2_msgs::JointVelocity vel = jaco2_msgs::JointDataConversion::data2Velocity(jd);
                pub_joint_vel.publish(vel);

                ROS_INFO_STREAM("inverse vel error: " << ec <<" : joint velocities: \n" << vel);

            }
            send_zero_ =false;
        }else if(!send_zero_){
            jaco2_msgs::JointVelocity vel;
            vel.joint1 = 0;
            vel.joint2 = 0;
            vel.joint3 = 0;
            vel.joint4 = 0;
            vel.joint5 = 0;
            vel.joint6 = 0;
            pub_joint_vel.publish(vel);
            ros::spinOnce();
            rate_.sleep();

            send_zero_ = true;
            ROS_INFO_STREAM("STOP");
        }
    }


    double getCmd(double cmd, double thresh) const
    {
        if(std::abs(cmd) > thresh ){
            return cmd;
        }
        else{
            return 0;
        }
    }





private:
    bool gcomp_;
    bool admitance_;
    bool cartesian_;
    bool send_zero_;
    bool has_joy_;
    std::map<Function,bool> toggle_states_;
    ros::NodeHandle nh_;
    ros::Rate rate_;
    ros::Subscriber sub_joint_states_;
    ros::Subscriber sub_joy_;
    ros::Publisher  pub_joint_vel;
    ros::ServiceClient srv_start_;
    ros::ServiceClient srv_stop_;
    ros::ServiceClient srv_gcomp_;
    ros::ServiceClient srv_admitance_;
    Jaco2KinDynLib::Jaco2KinematicModel model_;
    jaco2_data::JointStateData state_;
    std::vector<int> axes_;
    std::vector<int> buttons_;
    sensor_msgs::Joy last_joy_;

};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_teleop_node");
    ros::NodeHandle nh("~");
    std::string state_topic = nh.param<std::string>("state_topic", "/jaco_arm_driver/out/joint_states");
    std::string joy_topic   = nh.param<std::string>("joy_topic", "/ds4joy");
    std::string vel_topic   = nh.param<std::string>("vel_topic", "/jaco_arm_driver/in/joint_velocity");
    std::string start_srv   = nh.param<std::string>("start_service", "/jaco_arm_driver/in/start");
    std::string stop_srv    = nh.param<std::string>("stop_service", "/jaco_arm_driver/in/stop");
    std::string gcomp_srv   = nh.param<std::string>("gcomp_service", "/jaco_arm_driver/in/enable_gravity_compensation_mode");
    std::string ad_srv      = nh.param<std::string>("admitance_service", "/jaco_arm_driver/in/enable_admittance_mode");

    ROS_INFO_STREAM( start_srv << " | " << stop_srv);
    Jaco2Teleop telenode(state_topic,
                         joy_topic,
                         vel_topic,
                         start_srv,
                         stop_srv,
                         gcomp_srv,
                         ad_srv);

    ros::Rate r(80);
    while(ros::ok()){
        telenode.tick();
        r.sleep();
    }


    return 0;
}


