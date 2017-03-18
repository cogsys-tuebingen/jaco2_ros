#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <jaco2_msgs/Jaco2Accelerometers.h>
#include <jaco2_msgs/CalibAcc.h>
#include <jaco2_msgs/JointAngles.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <jaco2_calibration_utils/dynamic_calibration_sample.hpp>
#include <jaco2_calibration_utils/acceleration_samples.hpp>
#include <jaco2_calibration_utils/jaco2_calibration_io.h>


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


//        ros::Time now = ros::Time::now();
//        ros::Duration dt = now-lastTime_;
//        lastTime_ = now;
//        dt_ = dt.toSec();
        Jaco2Calibration::DynamicCalibrationSample sample;
        sample.time = msg->header.stamp.toSec();
        for(std::size_t i = 0; i < 6; ++i){
            sample.jointPos[i] = msg->position[i];
            sample.jointVel[i] = msg->velocity[i];
            sample.jointTorque[i] = -msg->effort[i];
            sample.jointAcc[i] = msg->acceleration[i];
        }


        if(!initialSensor_ /*&& currentSamples_ < numberOfSamples_*/){

            for(std::size_t i = 0; i <6;++i)
            {
                geometry_msgs::Vector3Stamped acc = jacoAccMsg_.lin_acc[i];

                Jaco2Calibration::AccelerationData data(acc.header.stamp.toSec(), acc.vector.x, acc.vector.y, acc.vector.z);
                accSamples_.push_back(i,data);
            }

            //estimate jaco's base acceleration
            Eigen::Vector3d g(-jacoAccMsg_.lin_acc[0].vector.y, -jacoAccMsg_.lin_acc[0].vector.x, -jacoAccMsg_.lin_acc[0].vector.z );
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

    void accsCb(const jaco2_msgs::Jaco2AccelerometersConstPtr& msg)
    {
        jacoAccMsg_ = *msg;

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
    ros::Subscriber subJointAcc_;
    std::vector<Jaco2Calibration::DynamicCalibrationSample> samples_;
    std::vector<Eigen::Vector3d> gravity_;
    Jaco2Calibration::AccelerationSamples accSamples_;
    ros::Time lastTime_;
    double dt_;
    jaco2_msgs::Jaco2Accelerometers jacoAccMsg_;
    ros::ServiceServer calibServiceServer_;
    std::vector<Eigen::Vector3d> gsum_;
    std::vector<std::string> jointGroupNames_;

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


