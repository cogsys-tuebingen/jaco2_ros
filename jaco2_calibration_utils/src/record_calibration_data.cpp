#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <jaco2_msgs/Jaco2Sensor.h>
#include <jaco2_msgs/CalibAcc.h>
#include <jaco2_msgs/JointAngles.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <geometry_msgs/Vector3Stamped.h>
//#include <jaco2_calibration/jaco2_calibration.h>
#include <jaco2_calibration/dynamic_calibration_sample.hpp>
#include <jaco2_calibration/acceleration_samples.hpp>
#include <jaco2_calibration/jaco2_calibration_io.hpp>


class CalibRecordNode
{
public:
    CalibRecordNode(bool genData, std::string urdf_param, std::string root, std::string tip, int numberOfSamples)
        : private_nh_("~"),
          genData_(genData),
          initial_(true),
          initialSensor_(true),
          calibAcc_(false),
          numberOfSamples_(numberOfSamples),
          currentSamples_(0),
          g_counter_(0)
    {
        boost::function<void(const jaco2_msgs::Jaco2JointStateConstPtr&)> cb = boost::bind(&CalibRecordNode::jointStateCb, this, _1);
        subJointState_ = private_nh_.subscribe("/jaco_arm_driver/out/joint_state_acc", 10, cb);
        boost::function<void(const jaco2_msgs::Jaco2SensorConstPtr&)> cbS = boost::bind(&CalibRecordNode::sensorCb, this, _1);
        subSensors_ = private_nh_.subscribe("/jaco_arm_driver/out/sensor_info", 10, cbS);



    }

    void jointStateCb(const jaco2_msgs::Jaco2JointStateConstPtr& msg)
    {
        if(initial_){
            lastTime_ = ros::Time::now();
            initial_ = false;
        }

        ros::Time now = ros::Time::now();
        ros::Duration dt = now-lastTime_;
        lastTime_ = now;
        dt_ = dt.toSec();
        Jaco2Calibration::DynamicCalibrationSample sample;
        for(std::size_t i = 0; i < 6; ++i){
            sample.jointPos[i] = msg->position[i];
            sample.jointVel[i] = msg->velocity[i];
            sample.jointTorque[i] = -msg->effort[i];
            sample.jointAcc[i] = msg->acceleration[i];
        }


        if(!initialSensor_ /*&& currentSamples_ < numberOfSamples_*/){

            for(std::size_t i = 0; i <6;++i)
            {
                geometry_msgs::Vector3Stamped acc = jacoSensorMsg_.acceleration[i];

                Jaco2Calibration::AccelerationData data(acc.header.stamp.toSec(), acc.vector.x, acc.vector.y, acc.vector.z);
                accSamples_.push_back(i,data);
            }

            //estimate jaco's base acceleration
            Eigen::Vector3d g(jacoSensorMsg_.acceleration[0].vector.y, jacoSensorMsg_.acceleration[0].vector.x, jacoSensorMsg_.acceleration[0].vector.z );
            g *= 9.81;
            gsum_.push_back(g);
            Eigen::Vector3d mean(0,0,0);
            int counter = 0;
            for(auto i = gsum_.rbegin(); i != gsum_.rend(); ++i)
            {
                if( counter < 5)
                {
                    mean += *i;
                    ++counter;
                }
                if(counter == 5)
                {
                    break;
                }
            }
            mean *= 1.0/((double)counter);
            sample.gravity = mean;
        }
        if(currentSamples_ > 10) {
            samples_.push_back(sample);

        }
        ++currentSamples_;
//        ROS_INFO_STREAM("Recoding_Data");
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
        ROS_INFO_STREAM("Recorded Samples: " << currentSamples_);

    }

    void save_data(std::string acc_file, std::string dyn_file) {
        accSamples_.save(acc_file);
        Jaco2Calibration::save(dyn_file,samples_);
    }


private:
    ros::NodeHandle private_nh_;
    bool genData_;
    bool initial_;
    bool initialSensor_;
    bool done_;
    bool calibAcc_;
    int numberOfSamples_;
    int currentSamples_;
    int g_counter_;
    ros::Subscriber subJointState_;
    ros::Subscriber subSensors_;
    ros::Subscriber subJointAcc_;
    std::vector<Jaco2Calibration::DynamicCalibrationSample> samples_;
    std::vector<Eigen::Vector3d> gravity_;
    Jaco2Calibration::AccelerationSamples accSamples_;
    ros::Time lastTime_;
    double dt_;
    jaco2_msgs::Jaco2Sensor jacoSensorMsg_;
    ros::ServiceServer calibServiceServer_;
    std::vector<Eigen::Vector3d> gsum_;
    std::vector<std::string> jointGroupNames_;

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_calibration_data_record_node");
    CalibRecordNode node(true,"/robot_description","jaco_link_base","jaco_link_hand",20000);
    ros::Rate r(25);

    std::string acc_file = "/tmp/acc_data.txt";
    std::string dyn_file = "/tmp/dyn_data.txt";

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string stop;
    bool done = false;
    while (ros::ok() && !done) {
        node.tick();
        //        ros::spinOnce();
        std::cin >> stop;
        if(stop == "stop")
        {
            done = true;
        }
        r.sleep();
    }

    spinner.stop();
    node.save_data(acc_file,dyn_file);


    return 0;
}


