#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

sensor_msgs::JointState aggregated_msg;
void jaco_21_callback(const sensor_msgs::JointStateConstPtr& msg) {
    for(int i = 0; i < msg->name.size(); i++) {
        aggregated_msg.name.push_back(msg->name[i]);
        aggregated_msg.position.push_back(msg->position[i]);
        aggregated_msg.velocity.push_back(msg->velocity[i]);
        aggregated_msg.effort.push_back(msg->effort[i]);
    }
}

void jaco_22_callback(const sensor_msgs::JointStateConstPtr& msg) {
    for(int i = 0; i < msg->name.size(); i++) {
        aggregated_msg.name.push_back(msg->name[i]);
        aggregated_msg.position.push_back(msg->position[i]);
        aggregated_msg.velocity.push_back(msg->velocity[i]);
        aggregated_msg.effort.push_back(msg->effort[i]);
    }
}


int main(int argc, char** argv )
{
    //TODO: not hardcoded
    ros::init(argc, argv, "joint_state_aggregator");
    ros::NodeHandle n;
    ros::Rate r(30);
    ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("aggregated_joint_states", 1);
    ros::Subscriber jaco_21_sub = n.subscribe<sensor_msgs::JointState>("jaco_21_driver/out/joint_states", 10, jaco_21_callback);
    ros::Subscriber jaco_22_sub = n.subscribe<sensor_msgs::JointState>("jaco_22_driver/out/joint_states", 10, jaco_22_callback);
    //ros::Subscriber jaco_21_sub = n.subscribe<sensor_msgs::JointState>("jaco_21_driver/out/joint_states", 10, jaco_21_callback);    //scitos head

    while (ros::ok()) {
        ros::spinOnce();
        js_pub.publish(aggregated_msg);
        aggregated_msg.name.clear();
        aggregated_msg.position.clear();
        aggregated_msg.velocity.clear();
        aggregated_msg.effort.clear();

    }

}

