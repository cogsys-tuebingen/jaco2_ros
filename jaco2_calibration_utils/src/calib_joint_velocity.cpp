#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <jaco2_msgs/JointVelocity.h>
#include <jaco2_msgs/ArmJointAnglesAction.h>
#include <jaco2_data/velocity_calibration.hpp>
#include <Eigen/Core>

double getMean(const std::vector<double>& vec)
{
    double mean = 0;
    if(!vec.empty()){
        for(auto val : vec){
            mean += val;
        }

        mean /= (double) vec.size();
    }
    return mean;
}

struct VelocityCalibSample{
    std::size_t joint_id;
    double cmd_vel;
    double measured_vel;
    double derived_vel;
    double factor;

    VelocityCalibSample(const std::size_t joint,
                        const double command_vel,
                        const std::vector<double>& derived_samples,
                        const std::vector<double>& measured_samples):
        joint_id(joint),
        cmd_vel(command_vel),
        measured_vel(0),
        derived_vel(0),
        factor(0)
    {

        derived_vel = getMean(derived_samples);
        measured_vel = getMean(measured_samples);
        if(measured_vel != 0){
            factor = derived_vel / measured_vel;
        }
        else {
            factor = 0;
            std::cerr << "Measured velocities == 0: sample size: " << measured_samples.size() << std::endl;
        }
    }


};

double getMeanFactor(const::std::vector<VelocityCalibSample>& joint_calib_samples)
{
    double result = 0;
    for(auto val : joint_calib_samples){
        result += val.factor;
    }
    result /= (double) joint_calib_samples.size();

    return result;
}

class CalibrateJointVelocity
{
public:
    CalibrateJointVelocity(ros::NodeHandle& handle, std::string driver_name) :
        nh(handle),
        ac(driver_name + "/arm_joint_angles_blocking")

    {

        subJointState = nh.subscribe(driver_name + "/out/joint_states", 20, &CalibrateJointVelocity::cb, this);
        pubJointVel = nh.advertise<jaco2_msgs::JointVelocity>(driver_name + "/in/joint_velocity",1);

        ac.waitForServer(ros::Duration(10));
    }

    void cb(const sensor_msgs::JointStateConstPtr& msg)
    {
        jstates.push_back(*msg);

    }

    void setCmdVel(jaco2_msgs::JointVelocity& msg, const std::size_t joint, const double val)
    {
        msg.joint1 = 0;
        msg.joint2 = 0;
        msg.joint3 = 0;
        msg.joint4 = 0;
        msg.joint5 = 0;
        msg.joint6 = 0;

        switch (joint) {
        case 0:
            msg.joint1 = val;
            break;
        case 1:
            msg.joint2 = val;
            break;
        case 2:
            msg.joint3 = val;
            break;
        case 3:
            msg.joint4 = val;
            break;
        case 4:
            msg.joint5 = val;
            break;
        case 5:
            msg.joint6 = val;
            break;
        default:
            break;
        }
    }

    void calculateVelosities(const std::size_t joint_id, std::vector<double>& derived_vel, std::vector<double>& measured_vel)
    {
        std::size_t n_ele = data.size();
        std::vector<double> debug;

        if(n_ele > 1){

            for(std::size_t i = 1; i < n_ele ; ++ i){

                double dt = data[i].header.stamp.toSec() - data[i-1].header.stamp.toSec() ;
                double deriv = (data[i].position[joint_id] - data[i-1].position[joint_id]) / dt;

                if(std::abs(deriv) > 0.05 && std::abs(deriv) < 2.0){
                    derived_vel.emplace_back(deriv);
                    measured_vel.emplace_back(data[i].velocity[joint_id]);
                }
                else{
                    debug.push_back(deriv);
                }

            }

            if(derived_vel.empty()){
                double mean = getMean(debug);
                auto it_min = std::min_element(debug.begin(), debug.end());
                auto it_max = std::max_element(debug.begin(), debug.end());

                std::cout << "mean: " << mean << " | min: " << *it_min  << " | max: " << *it_max << std::endl;
            }
        }
        else{
            std::cerr << "just on elemet in data??" << std::endl;
        }

    }

    void calibrate()
    {


        std::vector<double> limits = {2*M_PI, M_PI_2, M_PI_2, 2*M_PI, 2*M_PI, 2*M_PI};
        std::vector<double> cmd_vel = { 0.2, -0.2, 0.3, -0.3, 0.5, -0.5, 0.8, -0.8};
        std::vector <double> start_pos = {M_PI_2, M_PI, M_PI, 0 , 0, 0};

        jaco2_msgs::ArmJointAnglesGoal goal;
        goal.type = jaco2_msgs::ArmJointAnglesGoal::RADIAN;
        goal.angles.joint1 = start_pos[0];
        goal.angles.joint2 = start_pos[1];
        goal.angles.joint3 = start_pos[2];
        goal.angles.joint4 = start_pos[3];
        goal.angles.joint5 = start_pos[4];
        goal.angles.joint6 = start_pos[5];
        ac.sendGoal(goal);
        ac.waitForResult(ros::Duration(60));

        std::vector<std::vector<VelocityCalibSample>> parameterPerJoint;
        parameterPerJoint.resize(6);

        ros::spinOnce();
        ros::Rate r(40);
        ros::Duration pause(0.5);
        ros::Duration d(4);

        while(jstates.empty()){
            ros::spinOnce();
            r.sleep();
        }


        for(std::size_t joint_id = 0; joint_id < 6; ++joint_id){
            double limit = limits[joint_id];
            for(double vel : cmd_vel){
                double pos = jstates.back().position[joint_id];
                ros::Time start = ros::Time::now();
                data.clear();
                ros::Duration since_start = ros::Time::now() -start;
                bool pos_valid = -limit < pos && pos < limit;
                if(joint_id == 1 || joint_id == 2){
                    pos_valid = M_PI - limit -0.2 <= pos && pos <= M_PI + limit + 0.2;
                }
                while(ros::ok() && since_start < d && pos_valid){
                    jaco2_msgs::JointVelocity cmd;
                    setCmdVel(cmd, joint_id, vel);
                    pubJointVel.publish(cmd);
                    ros::spinOnce();
                    r.sleep();
                    pos = jstates.back().position[joint_id];
                    data.emplace_back(jstates.back());
                    since_start = ros::Time::now() -start;
                    if(joint_id == 1 || joint_id == 2){
                        pos_valid = M_PI - limit <= pos && pos <= M_PI + limit;
                    }
                    else{
                        pos_valid = -limit < pos && pos < limit;
                    }
                    if(!pos_valid){
                        std::cout << "reached limit" << std::endl;
                    }
                    //                    std::cout << "moving for " << since_start.toSec() << " seconds" << std::endl;
                }
                std::vector<double> derived_samples, measured_samples;
                calculateVelosities(joint_id,derived_samples, measured_samples);
                VelocityCalibSample sample(joint_id,vel,derived_samples, measured_samples);
                std::cout << "joint: " << joint_id << ", velocity: " << vel << ". samples: " << derived_samples.size() << ", factor: " << sample.factor << std::endl;
                if(std::abs(sample.factor) > 0 ){
                    parameterPerJoint[joint_id].push_back(sample);
                }

                ac.sendGoal(goal);
                ac.waitForResult(ros::Duration(60));
                pause.sleep();
                ros::spinOnce();
            }
        }

        for(auto d : parameterPerJoint){
            calibration.parameter.push_back(getMeanFactor(d));
        }
    }
public:

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<jaco2_msgs::ArmJointAnglesAction> ac;
    std::vector<sensor_msgs::JointState> jstates;
    std::vector<sensor_msgs::JointState> data;
    ros::Subscriber subJointState;
    ros::Publisher pubJointVel;
    Jaco2Calibration::VelocityCalibrationParams calibration;
};









int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_calib_velocity");

    ros::NodeHandle nh("~");
    std::string save_path;
    std::string driver_name;
    nh.param<std::string>("driver_name",driver_name,"/jaco_22_driver");
    nh.param<std::string>("save_path",save_path,"/tmp/velocity_calibration.yaml");



    CalibrateJointVelocity tool(nh, driver_name);
    tool.calibrate();


    Jaco2Calibration::save(save_path,tool.calibration);



    return 0;
}
