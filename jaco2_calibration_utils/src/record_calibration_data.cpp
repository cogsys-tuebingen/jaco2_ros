#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <jaco2_msgs/Jaco2Accelerometers.h>
#include <jaco2_msgs/CalibAcc.h>
#include <jaco2_msgs/JointAngles.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <jaco2_data/joint_state_data.h>
#include <jaco2_data/accelerometer_data.h>
#include <jaco2_msgs_conversion/jaco2_ros_msg_conversion.h>
#include <jaco2_calibration_utils/acceleration_samples.hpp>
#include <jaco2_calibration_utils/jaco2_calibration_io.h>
#include <jaco2_driver/data/gravity_estimator.h>
#include <jaco2_msgs_conversion/jaco2_ros_msg_conversion.h>

class CalibRecordNode
{
public:
    CalibRecordNode()
        : private_nh_("~"),
          initial_(true),
          initialSensor_(true),
          calibAcc_(false),
          currentSamples_(0),
          g_counter_(0)
    {
        boost::function<void(const jaco2_msgs::Jaco2JointStateConstPtr&)> cb = boost::bind(&CalibRecordNode::jointStateCb, this, _1);
        subJointState_ = private_nh_.subscribe("/jaco_arm_driver/out/joint_state_acc", 10, cb);
        boost::function<void(const jaco2_msgs::Jaco2AccelerometersConstPtr&)> cbA = boost::bind(&CalibRecordNode::accsCb, this, _1);
        subAccs_ = private_nh_.subscribe("/jaco_arm_driver/out/accelerometers", 10, cbA);

    }

    void jointStateCb(const jaco2_msgs::Jaco2JointStateConstPtr& msg)
    {
        if(initial_){
            lastTime_ = ros::Time::now();
            initial_ = false;
        }

        jaco2_data::JointStateDataStamped sample = jaco2_msgs::JointStateStampedConversion::jaco2Msg2Data(*msg);

        if(!initialSensor_ /*&& currentSamples_ < numberOfSamples_*/){


            for(std::size_t i = 0; i <6;++i)
            {
                accSamples_.push_back(i,jacoAccMsg_[i]);
            }

        }
        if(currentSamples_ > 10) {
            samples_.push_back(sample);

        }
        ++currentSamples_;
        //        ROS_INFO_STREAM("Recoding_Data");
    }

    void accsCb(const jaco2_msgs::Jaco2AccelerometersConstPtr& msg)
    {
        jacoAccMsg_ = jaco2_msgs::AccelerometerConversion::ros2data(*msg);

        if(initialSensor_)
        {
            initialSensor_ = false;
        }
    }


    void tick()
    {
        ros::spinOnce();
        if(currentSamples_ % 100 == 0){
            ROS_INFO_STREAM("Recorded Samples: " << currentSamples_);
        }

    }

    void save_data(std::string acc_file, std::string dyn_file) {
        accSamples_.save(acc_file);
        Jaco2Calibration::Jaco2CalibrationIO::save(dyn_file, samples_);
    }



private:
    ros::NodeHandle private_nh_;
    bool initial_;
    bool initialSensor_;
    bool done_;
    bool calibAcc_;
    int numberOfSamples_;
    int currentSamples_;
    int g_counter_;
    ros::Subscriber subJointState_;
    ros::Subscriber subAccs_;
    ros::Subscriber subacceleration_;
    jaco2_data::JointStateDataStampedCollection samples_;
    Jaco2Calibration::AccelerationSamples accSamples_;
    ros::Time lastTime_;
    double dt_;
    jaco2_data::AccelerometerData jacoAccMsg_;
    ros::ServiceServer calibServiceServer_;
    std::vector<Eigen::Vector3d> gsum_;
    std::vector<std::string> jointGroupNames_;
    GravityEstimator estimate_g_;
};

bool done = false;
void doneCb(const std_msgs::BoolConstPtr& msg)
{
    done = msg->data;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_calibration_data_record_node");

    ros::NodeHandle nh("~");

    std::string acc_data_path, dyn_data_path;
    nh.param<std::string>("acc_data_path", acc_data_path, "/tmp");
    nh.param<std::string>("dyn_data_path", dyn_data_path, "/tmp");


    CalibRecordNode node;
    ros::Rate r(25);

    ros::Time now = ros::Time::now();
    std::string acc_file = acc_data_path + "/acc_data_" + std::to_string(now.toSec()) + ".txt";
    std::string dyn_file = dyn_data_path + "/dyn_data_" + std::to_string(now.toSec()) + ".txt";

    ROS_INFO_STREAM("data files: " << acc_file << " and " << dyn_file);

    ros::Subscriber subDone = nh.subscribe("/dyn_calib_create_data/create_data_done", 1, doneCb);

    ros::AsyncSpinner spinner(1);
    spinner.start();


    done = false;
    while (ros::ok() && !done) {
        node.tick();
        ros::spinOnce();
        r.sleep();
    }

    spinner.stop();
    node.save_data(acc_file,dyn_file);


    return 0;
}


