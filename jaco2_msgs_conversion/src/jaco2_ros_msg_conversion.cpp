#include <jaco2_msgs_conversion/jaco2_ros_msg_conversion.h>

using namespace jaco2_msgs;


ros::Time TimeConversion::data2ROS(const jaco2_data::TimeStamp &data)
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



geometry_msgs::Vector3Stamped Vector3StampedConverion::data2ROS(const jaco2_data::Vector3Stamped& data)
{
    geometry_msgs::Vector3Stamped res;
    res.header.stamp = TimeConversion::data2ROS(data.stamp);
    res.header.frame_id = data.frame_id;
    res.vector.x = data.vector(0);
    res.vector.y = data.vector(1);
    res.vector.z = data.vector(2);
    return res;
}

jaco2_data::Vector3Stamped Vector3StampedConverion::ros2data(const geometry_msgs::Vector3Stamped &msg)
{
    jaco2_data::Vector3Stamped res;
    res.stamp = TimeConversion::ros2data(msg.header.stamp);
    res.frame_id = msg.header.frame_id;
    res.vector = Eigen::Vector3d(msg.vector.x, msg.vector.y, msg.vector.z);
    return res;
}



sensor_msgs::JointState JointStateConversion::data2SensorMsgs(const jaco2_data::JointStateData& data)
{
    sensor_msgs::JointState res;
    res.header.stamp = TimeConversion::data2ROS(data.stamp);
    res.name = data.names;
    res.position = data.position;
    res.velocity = data.velocity;
    res.effort = data.torque;
    return res;
}

jaco2_msgs::Jaco2JointState JointStateConversion::datata2Jaco2Msgs(const jaco2_data::JointStateData& data)
{
    jaco2_msgs::Jaco2JointState res;
    res.header.stamp = TimeConversion::data2ROS(data.stamp);
    res.name = data.names;
    res.position = data.position;
    res.velocity = data.velocity;
    res.acceleration = data.acceleration;
    res.effort = data.torque;
    return res;
}

jaco2_data::JointStateData JointStateConversion::sensorMsgs2Data(const sensor_msgs::JointState& msg)
{
    jaco2_data::JointStateData res;
    res.stamp = TimeConversion::ros2data(msg.header.stamp);
    res.names = msg.name;
    res.position = msg.position;
    res.velocity = msg.velocity;
    res.torque = msg.effort;
}

jaco2_data::JointStateData JointStateConversion::jaco2Msg2Data(const jaco2_msgs::Jaco2JointState& msg)
{
    jaco2_data::JointStateData res;
    res.stamp = TimeConversion::ros2data(msg.header.stamp);
    res.names = msg.name;
    res.position = msg.position;
    res.velocity = msg.velocity;
    res.acceleration = msg.acceleration;
    res.torque = msg.effort;
    return res;
}



jaco2_msgs::Jaco2Accelerometers AccelerometerConversion::data2ros(const jaco2_data::AccelerometerData& data)
{
    jaco2_msgs::Jaco2Accelerometers res;
    for(auto d : data.lin_acc){
        res.lin_acc.emplace_back(Vector3StampedConverion::data2ROS(d));
    }
    return res;
}


jaco2_data::AccelerometerData AccelerometerConversion::ros2data(const Jaco2Accelerometers &msg)
{
    jaco2_data::AccelerometerData res;
    for(auto d : msg.lin_acc){
        res.lin_acc.emplace_back(Vector3StampedConverion::ros2data(d));
    }
    return res;
}
