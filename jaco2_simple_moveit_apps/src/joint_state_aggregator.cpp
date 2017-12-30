#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

sensor_msgs::JointState aggregated_msg;
bool received_jaco_21 = false;
bool received_jaco_22 = false;

void jaco_21_callback(const sensor_msgs::JointStateConstPtr& msg) {
    if (!received_jaco_21) {
        aggregated_msg.name.insert(aggregated_msg.name.end(), msg->name.begin(), msg->name.end());
        aggregated_msg.position.insert(aggregated_msg.position.end(), msg->position.begin(), msg->position.end());
        aggregated_msg.velocity.insert(aggregated_msg.velocity.end(), msg->velocity.begin(), msg->velocity.end());
        aggregated_msg.effort.insert(aggregated_msg.effort.end(), msg->effort.begin(), msg->effort.end());

        aggregated_msg.header.stamp = msg->header.stamp;
        received_jaco_21 = true;
    }
}

void jaco_22_callback(const sensor_msgs::JointStateConstPtr& msg) {
    if (!received_jaco_22){
        aggregated_msg.name.insert(aggregated_msg.name.end(), msg->name.begin(), msg->name.end());
        aggregated_msg.position.insert(aggregated_msg.position.end(), msg->position.begin(), msg->position.end());
        aggregated_msg.velocity.insert(aggregated_msg.velocity.end(), msg->velocity.begin(), msg->velocity.end());
        aggregated_msg.effort.insert(aggregated_msg.effort.end(), msg->effort.begin(), msg->effort.end());
        aggregated_msg.header.stamp = msg->header.stamp;
        received_jaco_22 = true;
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
    //ros::Subscriber jaco_21_sub = n.subscribe<sensor_msgs::JointState>("jaco_21_driver/out/joint_states", 10, jaco_21_callback);    //todo: scitos head

    while (ros::ok()) {
        ros::spinOnce();
        if (received_jaco_21 && received_jaco_22) {
            js_pub.publish(aggregated_msg);
            aggregated_msg.name.clear();
            aggregated_msg.position.clear();
            aggregated_msg.velocity.clear();
            aggregated_msg.effort.clear();
            received_jaco_21 = false;
            received_jaco_22 = false;
        }

    }

}

