#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <jaco2_msgs/Jaco2Sensor.h>
//#include <jaco2_msgs/CalibAcc.h>
#include <jaco2_msgs/JointVelocity.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <jaco2_calibration/jaco2_calibration.h>
#include <jaco2_calibration/dynamic_calibration_sample.hpp>
#include <jaco2_calibration/acceleration_samples.hpp>
#include <jaco2_calibration/jaco2_calibration_io.hpp>

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
    ros::Rate r(50);
    //    boost::function<void(const jaco2_msgs::Jaco2JointStateConstPtr&)> cb = boost::bind(&jointStateCb, this _1);
    ros::Subscriber subJointState_ = private_nh_.subscribe("/jaco_arm_driver/out/joint_state_acc", 10, cb);
    ros::Publisher pubJointVel = private_nh_.advertise<jaco2_msgs::JointVelocity>("/jaco_arm_driver/in/joint_velocity",2);

    ros::Rate r2(65);

    while (ros::ok()) {


        ros::spinOnce();
        if(recieved_data_){
            double pos0 = samples_.back().jointPos[0];
            double pos5 = samples_.back().jointPos[5];
            bool reachedGoal = false;
            double p1 = 1.5;
            while(!reachedGoal){
                jaco2_msgs::JointVelocity v_msg;
                v_msg.joint1 = p1*(2*M_PI+pos0 -samples_.back().jointPos[0]);
                v_msg.joint2 = 0;
                v_msg.joint3 = 0;
                v_msg.joint4 = 0;
                v_msg.joint5 = 0;
                v_msg.joint6 = 10;

                pubJointVel.publish(v_msg);
                ros::spinOnce();
                reachedGoal = fabs(samples_.back().jointPos[0] - pos0 - 2*M_PI) < 0.01;
                r2.sleep();
            }
            ros::spinOnce();
            double p5 = 1.5;
            reachedGoal = false;
            while(!reachedGoal){
                jaco2_msgs::JointVelocity v_msg;
                v_msg.joint1 = p1*(pos0 -samples_.back().jointPos[0]);
                v_msg.joint2 = 0;
                v_msg.joint3 = 0;
                v_msg.joint4 = 0;
                v_msg.joint5 = 0;
                v_msg.joint6 = p5*(pos5 - samples_.back().jointPos[5]);

                pubJointVel.publish(v_msg);
                ros::spinOnce();
                reachedGoal = fabs(samples_.back().jointPos[0] - pos0) < 0.01;
                r2.sleep();
            }
            if(reachedGoal)
            {
                return 0;
            }
        }

        r.sleep();

    }
    return 0;
}
