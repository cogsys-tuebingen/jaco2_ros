#include <ros/ros.h>
#include <jaco2_msgs/JointVelocity.h>
#include <sensor_msgs/JointState.h>
#include <jaco2_data/joint_state_data.h>
#include <jaco2_msgs_conversion/jaco2_ros_msg_conversion.h>
struct SimDriver{

    SimDriver(ros::NodeHandle& _nh,
              std::string state_topic ="/jaco_arm_driver/out/joint_states",
              std::string vel_topic="/jaco_arm_driver/in/joint_velocity") :
        nh(_nh)
    {
        sub_vel = nh.subscribe(vel_topic, 2, &SimDriver::cb, this);
        pub_joint_state = nh.advertise<sensor_msgs::JointState>(state_topic, 2);

        state.names = {"jaco_joint_1", "jaco_joint_2", "jaco_joint_3", "jaco_joint_4", "jaco_joint_5", "jaco_joint_6",
                       "jaco_joint_finger_1", "jaco_joint_finger_2", "jaco_joint_finger_3",
                       "jaco_joint_finger_tip_1", "jaco_joint_finger_tip_2", "jaco_joint_finger_tip_3"};

        state.position = {4.8089, 2.9226, 1.0028, 4.2031, 1.4448, 1.3206, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        state.velocity.resize(state.position.size(),0);
        last_stamp  = ros::Time::now();
    }

    void cb(const jaco2_msgs::JointVelocityConstPtr &msg)
    {
        vel = jaco2_msgs::JointDataConversion::velocity2Data(*msg);

        for(std::size_t i = 0; i < state.position.size() - vel.size(); ++i){
            vel.push_back(0);
        }

        ros::Time now = ros::Time::now();
        double dt = ( now - last_stamp).toSec();
        jaco2_data::JointStateData next = state;
        next.velocity = vel.data;
        auto it_v = state.velocity.begin();
        for(double& val : next.position){
            val += dt * (*it_v);
            ++it_v;
        }

        state = next;
        state.stamp.now();
        sensor_msgs::JointState jstate = jaco2_msgs::JointStateConversion::data2SensorMsgs(state);
        pub_joint_state.publish(jstate);
        last_stamp = now;
    }

    void tick()
    {
        state.stamp.now();
        sensor_msgs::JointState jstate = jaco2_msgs::JointStateConversion::data2SensorMsgs(state);
        pub_joint_state.publish(jstate);
    }


    ros::NodeHandle nh;
    ros::Subscriber sub_vel;
    ros::Publisher pub_joint_state;
    jaco2_data::JointData vel;
    jaco2_data::JointStateData state;
    ros::Time last_stamp;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_arm_driver");
    ros::NodeHandle nh("~");
    SimDriver driver(nh);

    ros::Rate r(80);
    while(ros::ok()){
        ros::spinOnce();
        driver.tick();
        r.sleep();
    }
    return 0;
}
