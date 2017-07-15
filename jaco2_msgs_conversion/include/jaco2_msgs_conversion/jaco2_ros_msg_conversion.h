#ifndef JACO2_ROS_MSG_CONVERSION_H
#define JACO2_ROS_MSG_CONVERSION_H
/// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
/// JACO2 ROS MSGS
#include <jaco2_msgs/Jaco2JointState.h>
#include <jaco2_msgs/Jaco2Accelerometers.h>
/// JACO2 DATA
#include <jaco2_data/time_stamp.h>
#include <jaco2_data/vector3stamped.h>
#include <jaco2_data/joint_state_data.h>
#include <jaco2_data/accelerometer_data.h>
#include <jaco2_data/extended_joint_state_data.h>

namespace jaco2_msgs {

struct TimeConversion{
    static ros::Time data2ROS(const jaco2_data::TimeStamp& data);
    static jaco2_data::TimeStamp ros2data(const ros::Time& time);
};

struct Vector3StampedConverion{
    static geometry_msgs::Vector3Stamped data2ROS(const jaco2_data::Vector3Stamped &data);
    static jaco2_data::Vector3Stamped ros2data(const geometry_msgs::Vector3Stamped& msg);
};

struct JointStateConversion {
    static sensor_msgs::JointState data2SensorMsgs(const jaco2_data::JointStateData& data);
    static jaco2_msgs::Jaco2JointState datata2Jaco2Msgs(const jaco2_data::JointStateData &data);
    static jaco2_data::JointStateData sensorMsgs2Data(const sensor_msgs::JointState& msg);
    static jaco2_data::JointStateData jaco2Msg2Data(const jaco2_msgs::Jaco2JointState& msg);
};

struct AccelerometerConversion{
    static jaco2_msgs::Jaco2Accelerometers data2ros(const jaco2_data::AccelerometerData& data);
    static jaco2_data::AccelerometerData ros2data(const jaco2_msgs::Jaco2Accelerometers& msg);
};
}
#endif // JACO2_ROS_MSG_CONVERSION_H
