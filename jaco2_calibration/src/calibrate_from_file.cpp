#include <ros/ros.h>
#include <jaco2_calibration/jaco2_calibration.h>
#include <jaco2_calibration/jaco2_calibration_io.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_file_calibration_node");
    std::string urdf_param("/robot_description");
    std::string base("jaco_link_base");
    std::string tip("jaco_link_hand");
    Jaco2Calibration calib(urdf_param, base, tip);
    std::vector<DynamicCalibrationSample> samples;
    Jaco2CalibIO::importAsciiData("/tmp/data.txt",samples);
    calib.calibrateCoMandInertia(samples);
    std::vector<DynamicCalibratedParameters> param = calib.getDynamicCalibration();
    Jaco2CalibIO::save("/tmp/param.txt",param);
    return 0;
}

