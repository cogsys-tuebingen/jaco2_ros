#include<memory>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <jaco2_kin_dyn_lib/jaco2_kinematics_dynamics.h>
#include <jaco2_msgs/JointAngles.h>
#include <jaco2_calibration/jaco2_calibration_io.hpp> 

class Jaco2TorquePublisher
{
public:
    Jaco2TorquePublisher(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip)
        : inital_(true),
          private_nh_("~"),
          solver_(robot_model,chain_root,chain_tip)
    {
        boost::function<void(const sensor_msgs::JointStateConstPtr&)> cb = boost::bind(&Jaco2TorquePublisher::jointStateCb, this, _1);
        subJointState_ = private_nh_.subscribe("/jaco_arm_driver/out/joint_states", 1, cb);
        publisher_ = private_nh_.advertise<jaco2_msgs::JointAngles>("model_torques",2);
        diffPublisher_ = private_nh_.advertise<jaco2_msgs::JointAngles>("torque_diffs",2);
        lastTime_ = ros::Time::now();

        std::vector<Jaco2Calibration::DynamicCalibratedParameters> calibParam;
        Jaco2Calibration::loadDynParm("/tmp/param.txt", calibParam);
        for(Jaco2Calibration::DynamicCalibratedParameters param : calibParam){
            solver_.changeDynamicParams(param.linkName, param.mass, param.coM, param.inertia);
        }

    }

    void jointStateCb(const sensor_msgs::JointStateConstPtr& msg)
    {
        if(inital_){
            jointPos_.resize(6);
            jointVel_.resize(6);
            jointVelLast_.resize(6);
            jointAcc_.resize(6);
            jointTorques_.resize(6);
            modelTorques_.resize(6);
            for(std::size_t i = 0; i < 6; ++i){

                jointVelLast_[i] = msg->velocity[i];
            }
            lastTime_ = ros::Time::now();
            inital_ = false;
        }
        ros::Time now = ros::Time::now();
        ros::Duration dt = now-lastTime_;
        lastTime_ = now;
        dt_ = dt.toSec();
        for(std::size_t i = 0; i < 6; ++i){
            jointPos_[i] = msg->position[i];
            jointVel_[i] = msg->velocity[i];
            jointTorques_[i] = msg->effort[i];

        }
        for(std::size_t i = 0; i < 6; ++i){
            if(dt_ !=0){
                jointAcc_[i] = (jointVel_[i] -jointVelLast_[i])/dt_;
                jointVelLast_[i] = jointVel_[i];
            }
            else{
                jointAcc_[i] = 0;
            }
        }
    }

    void tick()
    {
        ros::spinOnce();
        if(!inital_){
            solver_.getTorques(jointPos_,jointVel_,jointAcc_,modelTorques_);
            jaco2_msgs::JointAngles pubMsg;
            pubMsg.joint1 = modelTorques_[0];
            pubMsg.joint2 = modelTorques_[1];
            pubMsg.joint3 = modelTorques_[2];
            pubMsg.joint4 = modelTorques_[3];
            pubMsg.joint5 = modelTorques_[4];
            pubMsg.joint6 = modelTorques_[5];
            publisher_.publish(pubMsg);

            jaco2_msgs::JointAngles diffMsg;

            diffMsg.joint1 = modelTorques_[0] / jointTorques_[0];
            diffMsg.joint2 = modelTorques_[1] / jointTorques_[1];
            diffMsg.joint3 = modelTorques_[2] / jointTorques_[2];
            diffMsg.joint4 = modelTorques_[3] / jointTorques_[3];
            diffMsg.joint5 = modelTorques_[4] / jointTorques_[4];
            diffMsg.joint6 = modelTorques_[5] / jointTorques_[5];
            diffPublisher_.publish(diffMsg);
        }
    }


private:
    bool inital_;
    ros::NodeHandle private_nh_;
    Jaco2KinematicsDynamicsModel solver_;
    ros::Subscriber subJointState_;
    ros::Publisher publisher_;
    ros::Publisher diffPublisher_;
    std::vector<double> jointPos_;
    std::vector<double> jointVel_;
    std::vector<double> jointVelLast_;
    std::vector<double> jointAcc_;
    std::vector<double> jointTorques_;
    std::vector<double> modelTorques_;
    ros::Time lastTime_;
    double dt_;

};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pub_jaco2_model_torques");
//    ros::NodeHandle node("~");
    Jaco2TorquePublisher jacomodel("robot_description","jaco_link_base","jaco_link_hand");
    ros::Rate r(80);
    while(ros::ok()){
        ros::spinOnce();
        jacomodel.tick();
        r.sleep();
    }
    return 0;
}

