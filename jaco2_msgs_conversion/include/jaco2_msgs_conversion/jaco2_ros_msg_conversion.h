#ifndef JACO2_ROS_MSG_CONVERSION_H
#define JACO2_ROS_MSG_CONVERSION_H
/// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
/// JACO2 ROS MSGS
#include <jaco2_msgs/JointData.h>
#include <jaco2_msgs/JointAngles.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <jaco2_msgs/Jaco2Accelerometers.h>
#include <jaco2_msgs/JointVelocity.h>
/// JACO2 DATA
#include <jaco2_data/types.h>
#include <jaco2_data/time_stamp.h>
#include <jaco2_data/joint_data.h>
#include <jaco2_data/joint_angles.h>
#include <jaco2_data/vector3.h>
#include <jaco2_data/wrench.h>
#include <jaco2_data/joint_state_data.h>
#include <jaco2_data/accelerometer_data.h>
#include <jaco2_data/extended_joint_state_data.h>

namespace jaco2_msgs {

struct TimeConversion{
    static ros::Time data2ros(const jaco2_data::TimeStamp& data);
    static jaco2_data::TimeStamp ros2data(const ros::Time& time);
};

struct HeaderConversion{
    static std_msgs::Header data2ros(const jaco2_data::Header& data);
    static jaco2_data::Header ros2data(const std_msgs::Header& data);
};

struct Vector3Conversion{
    static geometry_msgs::Vector3 data2ros(const jaco2_data::Vector3& data);
    static geometry_msgs::Vector3 data2ros(const jaco2_data::Vector3Stamped& data);
    static jaco2_data::Vector3 ros2data(const geometry_msgs::Vector3& msg);
    static jaco2_data::Vector3 ros2data(const geometry_msgs::Vector3Stamped& msg);
};


struct Vector3StampedConversion{
    static geometry_msgs::Vector3Stamped data2ros(const jaco2_data::Vector3& data);
    static geometry_msgs::Vector3Stamped data2ros(const jaco2_data::Vector3Stamped& data);
    static jaco2_data::Vector3Stamped ros2data(const geometry_msgs::Vector3& msg);
    static jaco2_data::Vector3Stamped ros2data(const geometry_msgs::Vector3Stamped& msg);
};

struct WrenchConversion{
    static geometry_msgs::Wrench data2ros(const jaco2_data::Wrench& data);
    static geometry_msgs::Wrench data2ros(const jaco2_data::WrenchStamped& data);
    static jaco2_data::Wrench ros2data(const geometry_msgs::Wrench& msg);
    static jaco2_data::Wrench ros2data(const geometry_msgs::WrenchStamped& msg);
};

struct WrenchStampedConversion{
    static geometry_msgs::WrenchStamped data2ros(const jaco2_data::Wrench& data);
    static geometry_msgs::WrenchStamped data2ros(const jaco2_data::WrenchStamped& data);
    static jaco2_data::WrenchStamped ros2data(const geometry_msgs::Wrench& msg);
    static jaco2_data::WrenchStamped ros2data(const geometry_msgs::WrenchStamped& msg);
};

struct JointStateConversion {
    static sensor_msgs::JointState data2SensorMsgs(const jaco2_data::JointStateData& data);
    static sensor_msgs::JointState data2SensorMsgs(const jaco2_data::JointStateDataStamped& data);
    static jaco2_msgs::Jaco2JointState data2Jaco2Msgs(const jaco2_data::JointStateData &data);
    static jaco2_msgs::Jaco2JointState data2Jaco2Msgs(const jaco2_data::JointStateDataStamped &data);
    static jaco2_data::JointStateData sensorMsgs2Data(const sensor_msgs::JointState& msg);
    static jaco2_data::JointStateData jaco2Msg2Data(const jaco2_msgs::Jaco2JointState& msg);
};

struct JointStateStampedConversion {
    static jaco2_data::JointStateDataStamped sensorMsgs2Data(const sensor_msgs::JointState& msg);
    static jaco2_data::JointStateDataStamped jaco2Msg2Data(const jaco2_msgs::Jaco2JointState& msg);
};


struct AccelerometerConversion{
    static jaco2_msgs::Jaco2Accelerometers data2ros(const jaco2_data::AccelerometerData& data);
    static jaco2_data::AccelerometerData ros2data(const jaco2_msgs::Jaco2Accelerometers& msg);
};

struct JointAngleConversion{
    static jaco2_msgs::JointAngles data2ros(const jaco2_data::JointAngles& data);
    static jaco2_msgs::JointAngles data2ros(const jaco2_data::JointAnglesStamped& data);
    static jaco2_data::JointAngles ros2data(const jaco2_msgs::JointAngles& data);
};

struct JointAngleStampedConversion{
    static jaco2_data::JointAnglesStamped ros2data(const jaco2_msgs::JointAngles& data);
};

struct JointDataConversion{
    static jaco2_msgs::JointData data2ros(const jaco2_data::JointData& data);
    static jaco2_msgs::JointData data2ros(const jaco2_data::JointDataStamped& data);
    static jaco2_data::JointData ros2data(const jaco2_msgs::JointData& data);
    static jaco2_msgs::JointAngles data2rosAngles(const jaco2_data::JointData &data);
    static jaco2_msgs::JointVelocity data2Velocity(const jaco2_data::JointData &data);
    static jaco2_msgs::JointAngles data2rosAngles(const jaco2_data::JointDataStamped &data);
    static jaco2_msgs::JointVelocity data2Velocity(const jaco2_data::JointDataStamped &data);
    static jaco2_data::JointData velocity2Data(const jaco2_msgs::JointVelocity &vel);
};
}
#endif // JACO2_ROS_MSG_CONVERSION_H
