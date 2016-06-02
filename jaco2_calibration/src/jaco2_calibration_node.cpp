#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <jaco2_msgs/Jaco2Sensor.h>
#include <jaco2_msgs/CalibAcc.h>
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
          calibAcc_(true),
          calibration_(urdf_param,root,tip),
          numberOfSamples_(numberOfSamples),
          currentSamples_(0)
    {
        boost::function<void(const sensor_msgs::JointStateConstPtr&)> cb = boost::bind(&CalibNode::jointStateCb, this, _1);
        subJointState_ = private_nh_.subscribe("/jaco_arm_driver/out/joint_states", 10, cb);
        boost::function<void(const jaco2_msgs::Jaco2SensorConstPtr&)> cbS = boost::bind(&CalibNode::sensorCb, this, _1);
        subSensors_ = private_nh_.subscribe("/jaco_arm_driver/out/sensor_info", 10, cbS);
        calibration_.setGravityMagnitude(1.0);
        calibration_.setInitAccSamples(500);
        calibServiceServer_ = private_nh_.advertiseService("calibrate_acc", &CalibNode::changeCalibCallback, this);
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
            Jaco2Calibration::DynamicCalibrationSample sample;
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

                    Jaco2Calibration::AccelerationData data(acc.header.stamp.toSec(), acc.vector.x, acc.vector.y, acc.vector.z);
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

    bool changeCalibCallback(jaco2_msgs::CalibAcc::Request & req, jaco2_msgs::CalibAcc::Response& res)
    {
        calibAcc_ = req.calib_acc;
        if(calibAcc_){
            res.calib_acc_result = "Starting Accelerometer Calibration.";
        }
        else{
            res.calib_acc_result = "Starting Dynamic Parameter Calibration.";
        }
        notCalib_ = true;
        return true;
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
            accSamples_.save("/tmp/acc_samples.txt");
            Jaco2Calibration::save("/tmp/data.txt",samples_);
            if(calibAcc_){
                bool succ = calibration_.calibrateAcc(accSamples_);
                if(succ){
                    Jaco2Calibration::save("/tmp/acc_calib.txt",calibration_.getAccCalibration());
                }

            }
            else{
                int ec = calibration_.calibrateCoMandInertia(samples_);
                if(ec > -1){
                    std::vector<Jaco2Calibration::DynamicCalibratedParameters> dynparams = calibration_.getDynamicCalibration();
                    Jaco2Calibration::save("/tmp/test_params.txt", dynparams);
                }

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
    bool calibAcc_;
    Jaco2Calibration::Jaco2Calibration calibration_;
    int numberOfSamples_;
    int currentSamples_;
    ros::Subscriber subJointState_;
    ros::Subscriber subSensors_;
    std::vector<Jaco2Calibration::DynamicCalibrationSample> samples_;
    Jaco2Calibration::AccelerationSamples accSamples_;
    ros::Time lastTime_;
    double dt_;
    jaco2_msgs::Jaco2Sensor jacoSensorMsg_;
    ros::ServiceServer calibServiceServer_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_calibration_node");
    CalibNode node(false,"/robot_description","jaco_link_base","jaco_link_hand",20000);
    ros::Rate r(25);
    while (ros::ok()) {
        node.tick();
        //        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

