#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <jaco2_msgs/Jaco2Sensor.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <jaco2_calibration/jaco2_calibration.h>
#include <jaco2_calibration/dynamic_calibration_sample.hpp>
#include <jaco2_calibration/acceleration_samples.hpp>
#include <jaco2_calibration/jaco2_calibration_io.hpp>


class CalibNode
{
public:
    CalibNode(bool genData, std::string urdf_param, std::string root, std::string tip, int numberOfSamples)
        : private_nh_("~"),
          genData_(genData),
          initial_(true),
          initialSensor_(true),
          notCalib_(true),
          calibration_(urdf_param,root,tip),
          numberOfSamples_(numberOfSamples),
          currentSamples_(0)
    {
        boost::function<void(const sensor_msgs::JointStateConstPtr&)> cb = boost::bind(&CalibNode::jointStateCb, this, _1);
        subJointState_ = private_nh_.subscribe("/jaco_arm_driver/out/joint_states", 10, cb);
        boost::function<void(const jaco2_msgs::Jaco2SensorConstPtr&)> cbS = boost::bind(&CalibNode::sensorCb, this, _1);
        subSensors_ = private_nh_.subscribe("/jaco_arm_driver/out/sensor_info", 10, cbS);
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
        double thres = 0.008;
        if(!initialSensor_ && currentSamples_ < numberOfSamples_){
            bool test = true;
            for(std::size_t i = 0; i <6;++i)
            {
                //                if(fabs(samples_[currentSamples_].jointVel[i]) < thres && fabs(samples_[currentSamples_].jointAcc[i]) < thres)
                //                {
                //                    geometry_msgs::Vector3Stamped acc = jacoSensorMsg_.acceleration[i];

                //                    AccelerationData data(acc.header.stamp.toSec(), acc.vector.x, acc.vector.y, acc.vector.z);
                //                    accSamples_.push_back(i,data);
                test &= fabs(samples_[currentSamples_].jointVel[i]) < thres && fabs(samples_[currentSamples_].jointAcc[i]) < thres;
                //                }
            }
            if(test){
                for(std::size_t i = 0; i <6;++i)
                {
                    geometry_msgs::Vector3Stamped acc = jacoSensorMsg_.acceleration[i];

                    AccelerationData data(acc.header.stamp.toSec(), acc.vector.x, acc.vector.y, acc.vector.z);
                    accSamples_.push_back(i,data);
                }
            }
        }
        ++currentSamples_;
    }

    void sensorCb(const jaco2_msgs::Jaco2SensorConstPtr& msg)
    {
        jacoSensorMsg_.acceleration = msg->acceleration;
        jacoSensorMsg_.temperature = msg->temperature;
        jacoSensorMsg_.torque = msg->torque;
        jacoSensorMsg_.temperature_time = msg->temperature_time;
        jacoSensorMsg_.torque_time = msg->torque_time;
        if(initialSensor_)
        {
            initialSensor_ = false;
        }
    }

    void tick()
    {
        ros::spinOnce();
        if(genData_)
        {
            //TODO IMPLEMENT MOVING
        }
        if(currentSamples_ > numberOfSamples_ && notCalib_)
        {
            std::cout << "calibrating ... " << std::endl;
            notCalib_ = false;
            //            int ec = calibration_.calibrateCoMandInertia(samples_);
            int ec = 0;
            if(ec > -1){
                //                std::vector<DynamicCalibratedParameters> dynparams = calibration_.getDynamicCalibration();
                //                Jaco2CalibIO::save("/tmp/test_params.txt", dynparams);
                Jaco2CalibIO::save("/tmp/data.txt",samples_);
                accSamples_.save("/tmp/acc_samples.txt");

            }
        }
        std::cout << "recording data ... samples: " << currentSamples_  << std::endl;

    }


private:
    ros::NodeHandle private_nh_;
    bool genData_;
    bool initial_;
    bool initialSensor_;
    bool notCalib_;
    Jaco2Calibration calibration_;
    int numberOfSamples_;
    int currentSamples_;
    ros::Subscriber subJointState_;
    ros::Subscriber subSensors_;
    std::vector<DynamicCalibrationSample> samples_;
    AccelerationSamples accSamples_;
    ros::Time lastTime_;
    double dt_;
    jaco2_msgs::Jaco2Sensor jacoSensorMsg_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_calibration_node");
    CalibNode node(false,"/robot_description","jaco_link_base","jaco_link_hand",7000);
    ros::Rate r(25);
    while (ros::ok()) {
        node.tick();
        //        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

