#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <jaco2_msgs/Jaco2Sensor.h>
//#include <jaco2_msgs/CalibAcc.h>
#include <jaco2_msgs/JointVelocity.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <jaco2_msgs/ArmJointAnglesAction.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <jaco2_calibration_utils/dynamic_calibration_sample.hpp>
#include <jaco2_calibration_utils/acceleration_samples.hpp>
#include <jaco2_calibration_utils/jaco2_calibration_io.h>

std::vector<Jaco2Calibration::DynamicCalibrationSample> samples_;
bool recieved_data_ = false;

void cb(const jaco2_msgs::Jaco2JointStateConstPtr& msg)
{
    Jaco2Calibration::DynamicCalibrationSample sample;
    for(std::size_t i = 0; i < 6; ++i){
        sample.jointPos[i] = msg->position[i];
        sample.jointVel[i] = msg->velocity[i];
        sample.jointTorque[i] = -msg->effort[i];
        sample.jointAcc[i] = msg->acceleration[i];
    }
    recieved_data_ = true;
    samples_.push_back(sample);

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_creat_calib_data_node");
    ros::NodeHandle private_nh_("~");
//    ros::Rate r(50);
    //    boost::function<void(const jaco2_msgs::Jaco2JointStateConstPtr&)> cb = boost::bind(&jointStateCb, this _1);
    ros::Subscriber subJointState_ = private_nh_.subscribe("/jaco_arm_driver/out/joint_state_acc", 10, cb);
    ros::Publisher pubJointVel = private_nh_.advertise<jaco2_msgs::JointVelocity>("/jaco_arm_driver/in/joint_velocity",2);
    actionlib::SimpleActionClient<jaco2_msgs::ArmJointAnglesAction> ac("/jaco_arm_driver/arm_joint_angles", true);

    ac.waitForServer();
    ros::AsyncSpinner r(65);
    r.start();

//    while (ros::ok()) {


        ros::spinOnce();
        if(recieved_data_){
            double pos0 = samples_.back().jointPos[0];

            jaco2_msgs::ArmJointAnglesGoal goal;

            goal.angles.joint1 = pos0 + 2*M_PI;
            goal.angles.joint2 = samples_.back().jointPos[1];
            goal.angles.joint3 = samples_.back().jointPos[2];
            goal.angles.joint4 = samples_.back().jointPos[3];
            goal.angles.joint5 = samples_.back().jointPos[4];
            goal.angles.joint6 = samples_.back().jointPos[5];

            ac.sendGoal(goal);

            bool finished = ac.waitForResult(ros::Duration(20));

            jaco2_msgs::ArmJointAnglesGoal goal1;

            goal1.angles.joint1 = pos0;
            goal1.angles.joint2 = goal.angles.joint2;
            goal1.angles.joint3 = goal.angles.joint3;
            goal1.angles.joint4 = goal.angles.joint4;
            goal1.angles.joint5 = goal.angles.joint5;
            goal1.angles.joint6 = goal.angles.joint6;

            ac.sendGoal(goal1);

            finished = ac.waitForResult(ros::Duration(20));


//        }

//        r.sleep();

    }
    return 0;
}
