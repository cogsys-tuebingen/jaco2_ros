#include<memory>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <jaco2_kin_dyn_lib/jaco2_kinematics_dynamics.h>
#include <jaco2_kin_dyn_lib/yaml_to_kdl_tranform.h>
#include <jaco2_msgs/JointAngles.h>
#include <jaco2_msgs/Jaco2Sensor.h>
#include <jaco2_calibration/jaco2_calibration_io.hpp> 
#include <tf/transform_listener.h>
//#include <jaco2_msgs/Jaco2Acc.h>
#include <jaco2_msgs/Jaco2Accelerometers.h>
#include <tf_conversions/tf_kdl.h>
class Jaco2TorquePublisher
{
public:
    Jaco2TorquePublisher(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip)
        : inital_(true),
          initalSensor_(true),
          counter_(0),
          private_nh_("~"),
          solver_(robot_model,chain_root,chain_tip)
    {
        boost::function<void(const jaco2_msgs::Jaco2JointStateConstPtr&)> cb = boost::bind(&Jaco2TorquePublisher::jointStateCb, this, _1);
        subJointState_ = private_nh_.subscribe("/jaco_arm_driver/out/joint_state_acc", 1, cb);
        boost::function<void(const jaco2_msgs::Jaco2SensorConstPtr&)> cbS = boost::bind(&Jaco2TorquePublisher::sensorInfoCb, this, _1);
        subSensorInfo_ = private_nh_.subscribe("/jaco_arm_driver/out/sensor_info", 1,  cbS);
        publisher_ = private_nh_.advertise<jaco2_msgs::JointAngles>("model_torques",2);
        diffPublisher_ = private_nh_.advertise<jaco2_msgs::JointAngles>("torque_diffs",2);
        accPublisher_ = private_nh_.advertise<jaco2_msgs::Jaco2Accelerometers>("model_acc",2);        lastTime_ = ros::Time::now();

        std::vector<Jaco2Calibration::DynamicCalibratedParameters> calibParam;
        Jaco2Calibration::loadDynParm("/tmp/regression_rb_param.txt", calibParam);
        for(Jaco2Calibration::DynamicCalibratedParameters param : calibParam){
            solver_.changeDynamicParams(param.linkName, param.mass, param.coM, param.inertia);
        }



        Jaco2Yaml2KDLTransform::load("/tmp/acc_transforms.yaml",staticAccTrans_);
        //        listener_.waitForTransform("jaco_link_base","jaco_link_hand",ros::Time(0),ros::Duration(3));  //or use tf and store transformations
        links_ = solver_.getLinkNames();

    }

    void jointStateCb(const jaco2_msgs::Jaco2JointStateConstPtr& msg)
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
            jointAcc_[i] = msg->acceleration[i];

        }
        //        for(std::size_t i = 0; i < 6; ++i){
        //            if(dt_ !=0){
        //                jointAcc_[i] = (jointVel_[i] -jointVelLast_[i])/dt_;
        //                jointVelLast_[i] = jointVel_[i];
        //            }
        //            else{
        //                jointAcc_[i] = 0;
        //            }
        //        }
    }

    void sensorInfoCb(const jaco2_msgs::Jaco2SensorConstPtr& msg)
    {
        std::size_t id = gravityCounter_ % 6;
        gravity_[id][0] = msg->acceleration[0].vector.x;
        gravity_[id][1] = msg->acceleration[0].vector.y;
        gravity_[id][2] = msg->acceleration[0].vector.z;
        if(gravityCounter_ > 5){
            double sum[3];
            for( std::size_t i = 0; i < 6; ++i) {
                for(std::size_t j = 0; j < 3; ++j) {
                    sum[j] += gravity_[i][j]/6.0;
                }
            }
            gravityMean_[0] = sum[0];
            gravityMean_[1] = sum[1];
            gravityMean_[2] = sum[2];
        }
        ++gravityCounter_;
        if(initalSensor_) {

            frameNames_.resize(msg->name.size());
            acceleration_.resize(msg->name.size());
            initalSensor_ = false;
        }

        for(std::size_t i = 0; i < msg->name.size(); ++i){
            frameNames_[i] = msg->acceleration[i].header.frame_id;
            acceleration_[i] = Eigen::Vector3d(msg->acceleration[i].vector.x,
                                               msg->acceleration[i].vector.y,
                                               msg->acceleration[i].vector.z);
        }
    }

    void tick()
    {
        ros::spinOnce();
        if(!inital_ && !initalSensor_){
            //            if(counter_ == 0){
            double x =  gravityMean_[1] * (9.81);
            double y =  gravityMean_[0] * (9.81);
            double z =  gravityMean_[2] * (9.81);
//            double x =  0;
//            double y =  0;
//            double z =  -9.81;
            solver_.setGravity(x,y,z);
            //            }

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

            diffMsg.joint1 = modelTorques_[0] - jointTorques_[0];
            diffMsg.joint2 = modelTorques_[1] - jointTorques_[1];
            diffMsg.joint3 = modelTorques_[2] - jointTorques_[2];
            diffMsg.joint4 = modelTorques_[3] - jointTorques_[3];
            diffMsg.joint5 = modelTorques_[4] - jointTorques_[4];
            diffMsg.joint6 = modelTorques_[5] - jointTorques_[5];
            diffPublisher_.publish(diffMsg);
            counter_ = (counter_ + 1) % 10;

            std::vector<KDL::Twist> spatial_accs;
            std::vector<std::string> frame_names;
            solver_.getAcceleration(x,y,z,jointPos_,jointVel_,jointAcc_,frame_names,spatial_accs);

            jaco2_msgs::Jaco2Accelerometers modelAccs;

            for(std::size_t i =0; i < links_.size() -1; ++i) { // accelerometer 1 to 5 is published

                KDL::Twist a = staticAccTrans_[i+1].frame.Inverse() * spatial_accs[i]/9.81; //staticAccTrans_ contains accelerometers 0 to 5;
                geometry_msgs::Vector3Stamped vs;
                vs.header.frame_id = frameNames_[i+1]; //frameNames_ contains accelerometers 0 to 5;
                vs.header.stamp = ros::Time::now();
                vs.vector.x = a.vel(0);
                vs.vector.y = a.vel(1);
                vs.vector.z = a.vel(2);
                modelAccs.lin_acc.push_back(vs);
            }

            accPublisher_.publish(modelAccs);

        }
    }


private:
    bool inital_;
    bool initalSensor_;
    int counter_;
    ros::NodeHandle private_nh_;
    Jaco2KinematicsDynamicsModel solver_;
    ros::Subscriber subJointState_;
    ros::Subscriber subSensorInfo_;
    ros::Publisher publisher_;
    ros::Publisher diffPublisher_;
    ros::Publisher accPublisher_;
    std::vector<double> jointPos_;
    std::vector<double> jointVel_;
    std::vector<double> jointVelLast_;
    std::vector<double> jointAcc_;
    std::vector<double> jointTorques_;
    std::vector<double> modelTorques_;
    ros::Time lastTime_;
    double dt_;
    double gravity_[6][3];
    double gravityMean_[3];
    int gravityCounter_;
    //    tf::TransformListener listener_;
    std::vector<Jaco2Yaml2KDLTransform::KDLTransformation> staticAccTrans_;
    std::vector<std::string> links_;
    std::vector<std::string> frameNames_;
    std::vector<Eigen::Vector3d> acceleration_;


};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pub_jaco2_model_torques");
    //    ros::NodeHandle node("~");
    Jaco2TorquePublisher jacomodel("robot_description","jaco_link_base","jaco_link_hand");
    ros::Rate r(40);
    while(ros::ok()){
        ros::spinOnce();
        jacomodel.tick();
        r.sleep();
    }
    return 0;
}

