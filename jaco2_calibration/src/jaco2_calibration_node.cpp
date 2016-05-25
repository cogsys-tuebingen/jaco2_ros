#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <jaco2_calibration/jaco2_calibration.h>
#include <jaco2_calibration/dynamic_calibration_sample.hpp>

class CalibNode
{
public:
    CalibNode(bool genData, std::string urdf_param, std::string root, std::string tip, int numberOfSamples)
        : private_nh_("~"),
          genData_(genData),
          initial_(true),
          calibration_(urdf_param,root,tip),
          numberOfSamples_(numberOfSamples),
          currentSamples_(0)
    {
        boost::function<void(const sensor_msgs::JointStateConstPtr&)> cb = boost::bind(&CalibNode::jointStateCb, this, _1);
        subJointState_ = private_nh_.subscribe("/jaco_arm_driver/out/joint_states", 1, cb);
    }

    void jointStateCb(const sensor_msgs::JointStateConstPtr& msg)
    {
        if(initial_){
            lastTime_ = ros::Time::now();
            initial_ = false;
        }
        if(currentSamples_ < numberOfSamples_)
        {
            ros::Time now = ros::Time::now();
            ros::Duration dt = now-lastTime_;
            lastTime_ = now;
            dt_ = dt.toSec();
            DynamicCalibrationSample sample;
            for(std::size_t i = 0; i < 6; ++i){
                sample.jointPos[i] = msg->position[i];
                sample.jointVel[i] = msg->velocity[i];
                sample.jointTorque[i] = msg->effort[i];
                if(currentSamples_ > 0 && dt_ !=0)
                {
                    sample.jointAcc[i] = (sample.jointVel[i] - samples_.back().jointVel[i])/dt_;
                }
                else{
                    sample.jointAcc[i] = 0;
                }
            }
            samples_.push_back(sample);
        }
        ++currentSamples_;

    }
    void tick()
    {
        ros::spinOnce();
        if(genData_)
        {
            //TODO IMPLEMENT MOVING
        }
        if(currentSamples_ == numberOfSamples_)
        {
            int ec = calibration_.calibrateCoMandInertia(samples_);
            if(ec){
                std::vector<DynamicCalibratedParameters> dynparams = calibration_.getDynamicCalibration();
            }
        }

    }


private:
    ros::NodeHandle private_nh_;
    bool genData_;
    bool initial_;
    Jaco2Calibration calibration_;
    int numberOfSamples_;
    int currentSamples_;
    ros::Subscriber subJointState_;
    std::vector<DynamicCalibrationSample> samples_;
    ros::Time lastTime_;
    double dt_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_calibration node");
    ros::Rate r(40);
    CalibNode node(false,"robot_description","jaco_link_base","jaco_link_hand",10000);
    while (ros::ok()) {
        node.tick();
        r.sleep();
    }
    return 0;
}

