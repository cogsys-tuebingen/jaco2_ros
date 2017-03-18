/// System
#include <vector>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <signal.h>
/// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
/// Jaco 2
#include <jaco2_msgs/Jaco2Accelerometers.h>
#include <jaco2_msgs/CalibAcc.h>
#include <jaco2_msgs/JointAngles.h>
#include <jaco2_msgs/Jaco2Sensor.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <jaco2_msgs/ArmJointAnglesAction.h>
#include <jaco2_msgs/SetTorqueZero.h>
#include <jaco2_calibration_utils/dynamic_calibration_sample.hpp>
#include <jaco2_calibration_utils/acceleration_samples.hpp>
#include <jaco2_calibration_utils/jaco2_calibration_io.h>
#include <jaco2_kin_dyn_lib/jaco2_kinematic_model.h>
#include <jaco2_driver/torque_offset_lut.hpp>

struct OffsetData{
    OffsetData()
    {
        angles  = Eigen::VectorXd::Zero(6);
        torques = Eigen::VectorXd::Zero(6);
    }

    Eigen::VectorXd angles;
    Eigen::VectorXd torques;
};

class RecordStaticDataNode
{
public:
    RecordStaticDataNode()
        : private_nh_("~"),
          initial_(true),
          initialAngles_(true),
          init_step2_(false),
          start_(false),
          numberOfSamples_(10),
          link_counter_(0),
          steps_(6),
          ac_("/jaco_arm_driver/arm_joint_angles_blocking")
    {
        subJointAngles_ = private_nh_.subscribe("/jaco_arm_driver/out/joint_angles",10, &RecordStaticDataNode::jointAngleCb, this);
        subSensorInfo_ = private_nh_.subscribe("/jaco_arm_driver/out/sensor_info",10, &RecordStaticDataNode::sensorInfoCb, this);
        zero_client_ = private_nh_.serviceClient<jaco2_msgs::SetTorqueZero>("/jaco_arm_driver/in/set_torque_zero");
        ac_.waitForServer(ros::Duration(10));
    }

    void jointAngleCb(const jaco2_msgs::JointAnglesConstPtr& msg)
    {
        last_angles_ = *msg;
        initialAngles_ = false;
    }

    void sensorInfoCb(const jaco2_msgs::Jaco2SensorConstPtr& msg)
    {
        if(initialAngles_){
            return;
        }
        start_ = true;
        OffsetData current;
        current.angles(0) = last_angles_.joint1;
        current.angles(1) = last_angles_.joint2;
        current.angles(2) = last_angles_.joint3;
        current.angles(3) = last_angles_.joint4;
        current.angles(4) = last_angles_.joint5;
        current.angles(5) = last_angles_.joint6;

        std::size_t i = 0;
        for(auto val : msg->torque){
            current.torques(i) = val;
            ++i;
        }

        buffer_.push_back(current);

        while(buffer_.size() > numberOfSamples_){
            buffer_.pop_front();
        }
    }


    void tick(bool& done)
    {
        ros::spinOnce();
        ros::spinOnce();
        if(initial_){
            jaco2_msgs::ArmJointAnglesGoal goal;
            goal.type = jaco2_msgs::ArmJointAnglesGoal::DEGREE;

            goal.angles.joint1 = 0;
            goal.angles.joint2 = 180;
            goal.angles.joint3 = 180;
            goal.angles.joint4 = 0;
            goal.angles.joint5 = 0;
            std::cout << "Go to zero position" << std::endl;
            //            ac_.sendGoalAndWait(goal,ros::Duration(180),ros::Duration(2));

            ac_.sendGoal(goal);
            ac_.waitForResult(ros::Duration(180));
            std::cout << "Go to zero position done." << std::endl;
            jaco2_msgs::SetTorqueZero srv;
            for(std::size_t i = 1; i < 7; ++i){
                srv.request.actuator = i;
                zero_client_.call(srv.request,srv.response);
            }
            ros::Duration(4).sleep();
            initial_ = false;

        }
        if(start_ && !initial_){
            //            double pos0 = samples_.back().jointPos[0];

            if(link_counter_ == 1  || link_counter_ == 2 )
            {
                jaco2_msgs::ArmJointAnglesGoal goal;
                goal.angles.joint1 = sweep_angles_[0];
                if(link_counter_ == 2)
                    goal.angles.joint1 = sweep_angles_[1];
                goal.angles.joint2 = 180;
                goal.angles.joint3 = 180;
                goal.angles.joint4 = 0;
                goal.angles.joint5 = 0;
                goal.angles.joint6 = 0;
                //                ac_.sendGoalAndWait(goal,ros::Duration(180),ros::Duration(2));

                ac_.sendGoal(goal);
                ac_.waitForResult(ros::Duration(180));
            }
            if(link_counter_ < lut_.n_links){
                for(std::size_t step = 0; step < lut_.steps(link_counter_); ++step){

                    if(link_counter_ == 1 && step > lut_.steps(link_counter_)/2 && !init_step2_){
                        ROS_INFO_STREAM("Sweep other side of link 2.");
                        jaco2_msgs::ArmJointAnglesGoal goal;
                        double angle = step * lut_.resolution(link_counter_);
                        double goal_angle = lut_.lower_limits(link_counter_) + angle;
                        goal.angles.joint1 = sweep_angles_[1];
                        goal.angles.joint2 = goal_angle;
                        goal.angles.joint3 = 180;
                        goal.angles.joint4 = 0;
                        goal.angles.joint5 = 0;
                        goal.angles.joint6 = 0;
                        ac_.sendGoal(goal);
                        ac_.waitForResult(ros::Duration(360));
                        //                    ac_.sendGoalAndWait(goal,ros::Duration(180),ros::Duration(2));
                        init_step2_ = true;
                    }

                    jaco2_msgs::ArmJointAnglesGoal goal;
                    getGoal(step, goal);
                    ac_.sendGoal(goal);
//                    auto result = ac_.sendGoalAndWait(goal,ros::Duration(30),ros::Duration(2));

                    if(step == 0){
                        bool finished = ac_.waitForResult(ros::Duration(120));
                    }
                    else{
                        bool finished = ac_.waitForResult(ros::Duration(20));
                    }


                    ros::Rate r(40);
                    for(std::size_t i = 0; i < 2* numberOfSamples_; ++i){
                        ros::spinOnce();
                        r.sleep();

                    }
                    setLutEntry();
                    ROS_INFO_STREAM("current link: " << link_counter_ + 1
                                    << " step " << step << " of " << lut_.steps(link_counter_) << " steps." );
                    std::cout << step << " of " << lut_.steps(link_counter_) -1 << " steps." << std::endl;
                }
            }

            ++link_counter_;
        }

        done = link_counter_ > steps_;

    }



    void getGoal(const std::size_t step, jaco2_msgs::ArmJointAnglesGoal& goal)
    {
        double angle = step * lut_.resolution(link_counter_);
        double goal_angle = lut_.lower_limits(link_counter_) + angle;
        goal.type = jaco2_msgs::ArmJointAnglesGoal::DEGREE;
        goal.angles.joint1 = 0;
        goal.angles.joint2 = 180;
        goal.angles.joint3 = 180;
        goal.angles.joint4 = 0;
        goal.angles.joint5 = 0;
        goal.angles.joint6 = 0;
        switch(link_counter_){
        case 0:
            goal.angles.joint1 = goal_angle;
            break;
        case 1:{
            int id = (int)(step > lut_.steps(link_counter_) / 2);
            goal.angles.joint1 = sweep_angles_[id];
            goal.angles.joint2 = goal_angle;
            break;
        }
        case 2:
            goal.angles.joint3 = goal_angle;
            break;
        case 3:
            goal.angles.joint4 = goal_angle;
            break;
        case 4:
            goal.angles.joint5 = goal_angle;
            break;
        case 5:
            goal.angles.joint6 = goal_angle;
            break;
        }
    }

    void saveLUT(std::string path)
    {
        lut_.save(path);
    }

    void setLUT(const Jaco2Calibration::TorqueOffsetLut& lut)
    {
        lut_ = lut;
    }

    Jaco2Calibration::TorqueOffsetLut getLUT() const
    {
        return lut_;
    }

    void setSweepAngles(std::vector<double>& angles)
    {
        sweep_angles_ = angles;
    }

    void setLutEntry()
    {
        OffsetData mean;
        for(auto d : buffer_){
            mean.angles += d.angles;
            mean.torques += d.torques;
        }
        int elem = buffer_.size();
        mean.angles /= elem;
        mean.torques /= elem;

        std::cout << " angle: " << mean.angles(link_counter_) << " offset: " << mean.torques(link_counter_) << " size: " << elem <<std::endl;

        lut_.set(link_counter_ + 1,
                 mean.angles(link_counter_),
                 mean.torques(link_counter_));
    }


private:
    ros::NodeHandle private_nh_;
    bool initial_;
    bool initialAngles_;
    bool done_;
    bool init_step2_;
    bool start_;
    int numberOfSamples_;
    //    int currentSamples_;
    //    int g_counter_;
    int link_counter_;
    int steps_;
    actionlib::SimpleActionClient<jaco2_msgs::ArmJointAnglesAction> ac_;
    //    ros::Subscriber subJointState_;
    ros::Subscriber subJointAngles_;
    ros::Subscriber subSensorInfo_;
    ros::ServiceClient zero_client_;

    jaco2_msgs::JointAngles last_angles_;
    std::deque<OffsetData> buffer_;
    Jaco2Calibration::TorqueOffsetLut lut_;
    std::vector<double> sweep_angles_;




};

void spinThread()
{
    ros::spin();
}

std::shared_ptr<RecordStaticDataNode> node;
std::string save_file = "/tmp/lut.yaml";

void mySigintHandler(int sig)
{
    node->saveLUT(save_file);
    ros::shutdown();
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_static_data_record_node");

    ros::NodeHandle nh("~");

    nh.param<std::string>("file", save_file, "/tmp/torque_offset_lut_jaco2-2.yaml");

    Eigen::VectorXd upper_limits(6);
    Eigen::VectorXd lower_limits(6);

    std::vector<double> zero_angle(2);
    Eigen::VectorXd resolution(6);

    nh.param<double>("joint_1_upper_limit",upper_limits(0),722);
    nh.param<double>("joint_1_lower_limit",lower_limits(0),-722);

    nh.param<double>("joint_2_upper_limit",upper_limits(1),313);
    nh.param<double>("joint_2_lower_limit",lower_limits(1),47);

    nh.param<double>("joint_2_lower_limit_joint_1_angle",zero_angle[0],87);
    nh.param<double>("joint_2_upper_limit_joint_1_angle",zero_angle[1],277);

    nh.param<double>("joint_3_upper_limit",upper_limits(2),341);
    nh.param<double>("joint_3_lower_limit",lower_limits(2),19);

    nh.param<double>("joint_4_upper_limit",upper_limits(3),722);
    nh.param<double>("joint_4_lower_limit",lower_limits(3),-722);

    nh.param<double>("joint_5_upper_limit",upper_limits(4),722);
    nh.param<double>("joint_5_lower_limit",lower_limits(4),-722);

    nh.param<double>("joint_6_upper_limit",upper_limits(5),722);
    nh.param<double>("joint_6_lower_limit",lower_limits(5),-722);

    nh.param<double>("joint_1_resolution", resolution(0), 1.5);
    nh.param<double>("joint_2_resolution", resolution(1), 1.5);
    nh.param<double>("joint_3_resolution", resolution(2), 1.5);
    nh.param<double>("joint_4_resolution", resolution(3), 1.5);
    nh.param<double>("joint_5_resolution", resolution(4), 1.5);
    nh.param<double>("joint_6_resolution", resolution(5), 1.5);


    Eigen::VectorXi steps(6);
    for(std::size_t i = 0; i < 6; ++i){
        std::size_t step = std::floor((upper_limits(i) - lower_limits(i))/ resolution(i));
        steps(i) = step;
    }

    std::size_t sz = steps.maxCoeff();

    std::cout <<" Max. steps " << sz << std::endl;

    Jaco2Calibration::TorqueOffsetLut lut;
    lut.lut = Eigen::MatrixXd::Zero(6, sz);
    lut.steps = steps;
    lut.resolution = resolution;
    lut.lower_limits = lower_limits;

    //    LutIndex li = lut.index(2,46.803495407104492);

    //    std::cout << li.first << "  " << li.second << std::endl;


    //    lut.save("/tmp/torque_lut.yaml");

    //    TorqueOffsetLut lutl;

    //    lutl.load("/tmp/torque_lut.yaml");

    //    std::cout << "n_links: " << lutl.n_links << std::endl;
    //    std::cout << "steps: \n" << lutl.steps << std::endl;
    //    std::cout << "resolution: \n" << lutl.resolution << std::endl;
    //    std::cout << "lower_limits: \n" << lutl.lower_limits << std::endl;
    //    std::cout << "data: \n" << lutl.lut << std::endl;


    RecordStaticDataNode node;

    node.setLUT(lut);
    node.setSweepAngles(zero_angle);
    ros::Rate r(25);

    //    ros::MultiThreadedSpinner mspin(2);
    //    ros::AsyncSpinner spinner(1);
    //    spinner.start();
    ROS_INFO_STREAM("Start");

    boost::thread spin_thread(&spinThread);

    bool done = false;
    while (ros::ok() && !done) {
        node.tick(done);
        ros::spinOnce();
        r.sleep();
    }
    ros::shutdown();
    spin_thread.join();
    //    spinner.stop();
    node.saveLUT(save_file);


    return 0;
}


