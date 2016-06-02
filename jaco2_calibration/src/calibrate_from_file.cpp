#include <ros/ros.h>
#include <jaco2_calibration/jaco2_calibration.h>
#include <jaco2_calibration/jaco2_calibration_io.hpp>
#include <jaco2_calibration/acceleration_samples.hpp>

int main(int argc, char *argv[])
{
    std::string calib_mode = argv[1];
    std::string file = argv[2];

    ros::init(argc, argv, "jaco2_file_calibration_node");

    std::string urdf_param("/robot_description");
    std::string base("jaco_link_base");
    std::string tip("jaco_link_hand");
    Jaco2Calibration::Jaco2Calibration calib(urdf_param, base, tip);
    if(calib_mode == "dynamic"){
        std::vector<Jaco2Calibration::DynamicCalibrationSample> samples;
        Jaco2Calibration::importAsciiData(file,samples);
        calib.calibrateCoMandInertia(samples);
        std::vector<Jaco2Calibration::DynamicCalibratedParameters> param = calib.getDynamicCalibration();
        Jaco2Calibration::save("/tmp/param.txt",param);
    }
    if(calib_mode == "acc"){
        calib.setGravityMagnitude(1.0);
        calib.setInitAccSamples(800);
        Jaco2Calibration::AccelerationSamples samples;
        Jaco2Calibration::importAsciiData(file,samples);
        calib.calibrateAcc(samples);
        Jaco2Calibration::save("/tmp/acc_calib_param.txt",calib.getAccCalibration());
    }

    return 0;
}

