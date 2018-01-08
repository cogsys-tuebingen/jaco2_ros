#include <jaco2_msgs_conversion/jaco2_ros_msg_conversion.h>

using namespace jaco2_msgs;

// Time Conversion -------------------------------------------------------------------------
ros::Time TimeConversion::data2ros(const jaco2_data::TimeStamp &data)
{
    ros::Time res;
    res.fromNSec(data.toNSec());
    return res;
}

jaco2_data::TimeStamp TimeConversion::ros2data(const ros::Time &time)
{
    jaco2_data::TimeStamp res;
    res.fromNSec(time.toNSec());
    return res;
}

// Header Conversion -------------------------------------------------------------------------

std_msgs::Header HeaderConversion::data2ros(const jaco2_data::Header &data)
{
    std_msgs::Header res;
    res.frame_id = data.frame_id;
    res.stamp = TimeConversion::data2ros(data.stamp);
    return res;
}

jaco2_data::Header HeaderConversion::ros2data(const std_msgs::Header &data)
{
    jaco2_data::Header res;
    res.frame_id = data.frame_id;
    res.stamp = TimeConversion::ros2data(data.stamp);
    return res;
}

// Vector3 Conversion -------------------------------------------------------------------------

geometry_msgs::Vector3 Vector3Conversion::data2ros(const jaco2_data::Vector3& data)
{
    geometry_msgs::Vector3 res;
    res.x = data.vector(0);
    res.y = data.vector(1);
    res.z = data.vector(2);
    return res;
}

geometry_msgs::Vector3 Vector3Conversion::data2ros(const jaco2_data::Vector3Stamped& data)
{
    geometry_msgs::Vector3 res = data2ros(data.data);
    return res;
}

jaco2_data::Vector3 Vector3Conversion::ros2data(const geometry_msgs::Vector3 &msg)
{
    jaco2_data::Vector3 res;
    res.vector = Eigen::Vector3d(msg.x, msg.y, msg.z);
    return res;
}

jaco2_data::Vector3 Vector3Conversion::ros2data(const geometry_msgs::Vector3Stamped& msg)
{
    jaco2_data::Vector3 res;
    res.vector = Eigen::Vector3d(msg.vector.x, msg.vector.y, msg.vector.z);
    return res;
}

// Vector3Stamped Conversion -------------------------------------------------------------------------

geometry_msgs::Vector3Stamped Vector3StampedConversion::data2ros(const jaco2_data::Vector3& data)
{
    geometry_msgs::Vector3Stamped res;
    res.header.stamp = ros::Time::now();
    res.header.frame_id = "";
    res.vector = Vector3Conversion::data2ros(data);
    return res;
}

geometry_msgs::Vector3Stamped Vector3StampedConversion::data2ros(const jaco2_data::Vector3Stamped& data)
{
    geometry_msgs::Vector3Stamped res;
    res.header =  HeaderConversion::data2ros(data.header);
    res.vector = Vector3Conversion::data2ros(data.data);
    return res;
}

jaco2_data::Vector3Stamped Vector3StampedConversion::ros2data(const geometry_msgs::Vector3 &msg)
{
    jaco2_data::Vector3Stamped res;
    res.header.stamp.now();
    res.data = Vector3Conversion::ros2data(msg);
    return res;
}

jaco2_data::Vector3Stamped Vector3StampedConversion::ros2data(const geometry_msgs::Vector3Stamped& msg)
{
    jaco2_data::Vector3Stamped res;
    res.header = HeaderConversion::ros2data(msg.header);
    res.data   = Vector3Conversion::ros2data(msg.vector);
    return res;
}
// Wrench Conversion -------------------------------------------------------------------------

geometry_msgs::Wrench WrenchConversion::data2ros(const jaco2_data::Wrench& data)
{
    geometry_msgs::Wrench res;
    res.torque = Vector3Conversion::data2ros(data.torque);
    res.force = Vector3Conversion::data2ros(data.force);
    return res;
}
geometry_msgs::Wrench WrenchConversion::data2ros(const jaco2_data::WrenchStamped& data)
{
    geometry_msgs::Wrench res;
    res.torque = Vector3Conversion::data2ros(data.data.torque);
    res.force = Vector3Conversion::data2ros(data.data.force);
    return res;
}

jaco2_data::Wrench WrenchConversion::ros2data(const geometry_msgs::Wrench& msg)
{
    jaco2_data::Wrench res;
    res.torque = Vector3Conversion::ros2data(msg.torque);
    res.force = Vector3Conversion::ros2data(msg.force);
    return res;
}

jaco2_data::Wrench WrenchConversion::ros2data(const geometry_msgs::WrenchStamped& msg)
{
    jaco2_data::Wrench res;
    res.torque = Vector3Conversion::ros2data(msg.wrench.torque);
    res.force = Vector3Conversion::ros2data(msg.wrench.force);
    return res;
}

// WrenchStamped Conversion -------------------------------------------------------------------------

 geometry_msgs::WrenchStamped WrenchStampedConversion::data2ros(const jaco2_data::Wrench& data)
 {
     geometry_msgs::WrenchStamped res;
     res.header.frame_id ="";
     res.header.stamp = ros::Time::now();
     res.wrench = WrenchConversion::data2ros(data);
     return res;
 }

 geometry_msgs::WrenchStamped WrenchStampedConversion::data2ros(const jaco2_data::WrenchStamped& data)
 {
     geometry_msgs::WrenchStamped res;
     res.header = HeaderConversion::data2ros(data.header);
     res.wrench = WrenchConversion::data2ros(data.data);
     return res;
 }

 jaco2_data::WrenchStamped WrenchStampedConversion::ros2data(const geometry_msgs::Wrench& msg)
 {
     jaco2_data::WrenchStamped res;
     res.header.stamp.now();
     res.data = WrenchConversion::ros2data(msg);
     return res;
 }

 jaco2_data::WrenchStamped WrenchStampedConversion::ros2data(const geometry_msgs::WrenchStamped& msg)
 {
     jaco2_data::WrenchStamped res;
     res.header = HeaderConversion::ros2data(msg.header);
     res.data = WrenchConversion::ros2data(msg.wrench);
     return res;
 }

// JointState Conversion -------------------------------------------------------------------------

sensor_msgs::JointState JointStateConversion::data2SensorMsgs(const jaco2_data::JointStateData& data)
{
    sensor_msgs::JointState res;
    res.header.stamp = ros::Time::now();
    res.header.frame_id = "";
    res.name = data.names;
    res.position = data.position;
    res.velocity = data.velocity;
    res.effort = data.torque;
    return res;
}

sensor_msgs::JointState JointStateConversion::data2SensorMsgs(const jaco2_data::JointStateDataStamped& data)
{
    sensor_msgs::JointState res = JointStateConversion::data2SensorMsgs(data.data);
    res.header = HeaderConversion::data2ros(data.header);
    return res;
}

jaco2_msgs::Jaco2JointState JointStateConversion::data2Jaco2Msgs(const jaco2_data::JointStateData& data)
{
    jaco2_msgs::Jaco2JointState res;
    res.header.stamp = ros::Time::now();
    res.header.frame_id = "";
    res.gx = data.gravity(0);
    res.gy = data.gravity(1);
    res.gz = data.gravity(2);
    res.name = data.names;
    res.position = data.position;
    res.velocity = data.velocity;
    res.acceleration = data.acceleration;
    res.effort = data.torque;
    return res;
}

jaco2_msgs::Jaco2JointState JointStateConversion::data2Jaco2Msgs(const jaco2_data::JointStateDataStamped& data)
{
    jaco2_msgs::Jaco2JointState res = JointStateConversion::data2Jaco2Msgs(data.data);
    res.header = HeaderConversion::data2ros(data.header);
    return res;
}

jaco2_data::JointStateData JointStateConversion::sensorMsgs2Data(const sensor_msgs::JointState& msg)
{
    jaco2_data::JointStateData res;
    res.names = msg.name;
    res.position = msg.position;
    res.velocity = msg.velocity;
    res.torque = msg.effort;
    return res;
}


jaco2_data::JointStateData JointStateConversion::jaco2Msg2Data(const jaco2_msgs::Jaco2JointState& msg)
{
    jaco2_data::JointStateData res;
    res.gravity = Eigen::Vector3d(msg.gx, msg.gy, msg.gz);
    res.names = msg.name;
    res.position = msg.position;
    res.velocity = msg.velocity;
    res.acceleration = msg.acceleration;
    res.torque = msg.effort;
    return res;
}

// JointStateStamped Conversion -------------------------------------------------------------------------

jaco2_data::JointStateDataStamped JointStateStampedConversion::sensorMsgs2Data(const sensor_msgs::JointState& msg)
{
    jaco2_data::JointStateDataStamped res;
    res.data = JointStateConversion::sensorMsgs2Data(msg);
    res.header = HeaderConversion::ros2data(msg.header);
    return res;
}

jaco2_data::JointStateDataStamped JointStateStampedConversion::jaco2Msg2Data(const jaco2_msgs::Jaco2JointState& msg)
{
    jaco2_data::JointStateDataStamped res;
    res.data = JointStateConversion::jaco2Msg2Data(msg);
    res.header = HeaderConversion::ros2data(msg.header);
    return res;
}

// Accelerometer Conversion -------------------------------------------------------------------------

jaco2_msgs::Jaco2Accelerometers AccelerometerConversion::data2ros(const jaco2_data::AccelerometerData& data)
{
    jaco2_msgs::Jaco2Accelerometers res;
    for(auto d : data){
        res.lin_acc.emplace_back(Vector3StampedConversion::data2ros(d));
    }
    return res;
}


jaco2_data::AccelerometerData AccelerometerConversion::ros2data(const Jaco2Accelerometers &msg)
{
    jaco2_data::AccelerometerData res;
    for(auto d : msg.lin_acc){
        res.push_back(Vector3StampedConversion::ros2data(d));
    }
    return res;
}

// JointAngle Conversion -------------------------------------------------------------------------

jaco2_msgs::JointAngles JointAngleConversion::data2ros(const jaco2_data::JointAngles &data)
{
    jaco2_msgs::JointAngles res;
    if(data.size() >= 6){
        res.joint1 = data[0];
        res.joint2 = data[1];
        res.joint3 = data[2];
        res.joint4 = data[3];
        res.joint5 = data[4];
        res.joint6 = data[5];
    }
    return res;
}

jaco2_msgs::JointAngles JointAngleConversion::data2ros(const jaco2_data::JointAnglesStamped& data)
{
    jaco2_msgs::JointAngles res = JointAngleConversion::data2ros(data.data);
    return res;
}

jaco2_data::JointAngles JointAngleConversion::ros2data(const jaco2_msgs::JointAngles &data)
{
    jaco2_data::JointAngles res;
    res.resize(6,0);
    res[0] = data.joint1;
    res[1] = data.joint2;
    res[2] = data.joint3;
    res[3] = data.joint4;
    res[4] = data.joint5;
    res[5] = data.joint6;
    return res;
}

jaco2_data::JointAnglesStamped JointAngleStampedConversion::ros2data(const jaco2_msgs::JointAngles& data)
{
    jaco2_data::JointAnglesStamped res;
    res.header.frame_id = "";
    res.header.stamp.now();
    res.data = JointAngleConversion::ros2data(data);
    return res;
}

jaco2_data::JointData JointDataConversion::ros2data(const jaco2_msgs::JointData &data)
{
    jaco2_data::JointData res;
    res.data = data.data;
    return res;
}


jaco2_msgs::JointData JointDataConversion::data2ros(const jaco2_data::JointData &data)
{
    jaco2_msgs::JointData res;
    res.data = data.data;
    return res;
}

jaco2_msgs::JointData JointDataConversion::data2ros(const jaco2_data::JointDataStamped& data)
{
    jaco2_msgs::JointData res = JointDataConversion::data2ros(data.data);
    return res;
}

jaco2_msgs::JointAngles JointDataConversion::data2rosAngles(const jaco2_data::JointData &data)
{
    jaco2_data::JointAngles d = data;
    jaco2_msgs::JointAngles res = JointAngleConversion::data2ros(d);
    return res;
}


jaco2_msgs::JointVelocity JointDataConversion::data2Velocity(const jaco2_data::JointData &data)
{
    jaco2_msgs::JointVelocity vel;
    if(data.size() >= 6){
        vel.joint1 = data[0];
        vel.joint2 = data[1];
        vel.joint3 = data[2];
        vel.joint4 = data[3];
        vel.joint5 = data[4];
        vel.joint6 = data[5];
    }
    return vel;
}

jaco2_data::JointData JointDataConversion::velocity2Data(const jaco2_msgs::JointVelocity &vel)
{
    jaco2_data::JointData data;
    data.resize(6);
    data[0] = vel.joint1;
    data[1] = vel.joint2;
    data[2] = vel.joint3;
    data[3] = vel.joint4;
    data[4] = vel.joint5;
    data[5] = vel.joint6;
    return data;

}

