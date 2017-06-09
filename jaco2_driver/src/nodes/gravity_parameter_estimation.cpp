#include <ros/ros.h>
#include <jaco2_driver/jaco2_api.h>
#include <jaco2_driver/gravity_params.hpp>
int main(int argc, char *argv[])
{
    ros::init (argc, argv, "jaco2_gravity_parameter_estimation");
    ros::NodeHandle nh("~");
    std::string file;
    nh.param<std::string>("/jaco2_gravity_parameter_path", file, std::string("/tmp/jaco_g_params.yaml"));

    Jaco2API api;
    int result = api.init();
    ros::Duration r(1);
    r.sleep();

    auto data = api.getAngularPosition();
    r.sleep();

    ROS_INFO_STREAM("Angles: " << data.Actuators.Actuator1 << " "
                    << data.Actuators.Actuator2 << " "
                    << data.Actuators.Actuator3 << " "
                    << data.Actuators.Actuator4 << " "
                    << data.Actuators.Actuator5 << " "
                    << data.Actuators.Actuator6);

    if(result == 1){
        TrajectoryPoint tp;
        tp.InitStruct();
        tp.Position.Actuators.Actuator2 = 180;
        tp.Position.Actuators.Actuator3 = 180;
        tp.Position.Type = ANGULAR_POSITION;
        api.setAngularPosition(tp);
        ros::Duration(40).sleep();
        for(std::size_t i = 1; i < 7; ++i){
            ActuatorID id = static_cast<ActuatorID>(i);
            ROS_INFO_STREAM("Set torque zero for Actuator " << i);
            api.setTorqueZero(id);
        }
        Jaco2Calibration::ApiGravitationalParams params;
        //        api.disableTorque();
        ros::Duration(0.2).sleep();
        ROS_INFO_STREAM("ATTENTION RUNNING PARAMETER ESTIMATION!");
        std::cout << "ATTENTION RUNNING PARAMETER ESTIMATION!" << std::endl;
        api.runGravityEstimationSequnce(params.parameter, JACOV2_6DOF_SERVICE);
        ros::Duration(0.2).sleep();
        ROS_INFO_STREAM("Found Parameter");
        std::cout << "Found parameter." << std::endl;

        api.setGravityOptimalZParam(params.parameter);
        api.enableDirectTorqueMode(1.0);

        Jaco2Calibration::save(file, params);
        std::cout << "saved params" << std::endl;
        ros::Duration d(5);
        ROS_INFO_STREAM("Gravity Compensation for the next " << d.toSec() << "s.");
        ros::Time start = ros::Time::now();
        ros::Duration diff = ros::Time::now() - start;
        ros::Rate r(50);

        while( diff <= d){
            AngularPosition tau;
            tau.InitStruct();
            api.setAngularTorque(tau);
            ROS_INFO_STREAM("still " << (d-diff).toSec() << "s left ...");
            diff = ros::Time::now() - start;
            r.sleep();
        }

        ROS_INFO_STREAM("DONE!");

        api.setGravityType(MANUAL_INPUT);
        api.exitAPI();

        return 0;
    }
    else{
        std::cerr << "NO ROBOT FOUND!" << std::endl;
        return 23;
    }
}
