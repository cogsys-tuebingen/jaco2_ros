#include <random>
#include <ros/ros.h>
#include <jaco2_calibration/jaco2_calibration.h>
#include <jaco2_calibration_utils/jaco2_calibration_io.h>
#include <jaco2_calibration_utils/acceleration_samples.hpp>

int main(int argc, char *argv[])
{
    std::string calib_mode = argv[1];
    std::string file = argv[2];
    bool addNoise = std::stoi(argv[3]);

    ros::init(argc, argv, "jaco2_file_calibration_node");

    std::string urdf_param("/robot_description");
    std::string base("jaco_link_base");
    std::string tip("jaco_link_hand");
    Jaco2Calibration::Jaco2Calibration calib(urdf_param, base, tip);



    if(calib_mode == "dynamic"){
        jaco2_data::JointStateDataCollection samples;
        Jaco2Calibration::Jaco2CalibrationIO::importAsciiDataWithGravity(file,samples);
        if(addNoise)
        {
            std::default_random_engine generator;
            std::normal_distribution<double> distribution(0,0.4);
            //TODO TEST
            for(auto sample = samples.begin(); sample != samples.end(); ++sample) {
                for(auto tau = sample->torque.begin(); tau != sample->torque.end(); ++tau)
                {
                    double white_noise = distribution(generator);
                    (*tau) += white_noise;
                }
            }
        }
        Jaco2Calibration::Jaco2CalibrationIO::save("/tmp/dyn_calib_with_noise.txt",samples);
        calib.calibrateCoMandInertia(samples);

        Jaco2Calibration::DynamicCalibratedParametersCollection param = calib.getDynamicCalibration();
        Jaco2Calibration::Jaco2CalibrationIO::save("/tmp/param_sim.txt",param);
        Jaco2Calibration::DynamicCalibratedParametersCollection org_param = calib.getDynamicUrdfParam();
        Jaco2Calibration::DynamicCalibratedParametersCollection diff;
        for(std::size_t i = 0; i < param.size(); ++i)
        {
            Jaco2Calibration::DynamicCalibratedParameters tmp;

            tmp.linkName = param[i].linkName;
            tmp.coM = (param[i].coM - org_param[i].coM).array() / param[i].coM.array() ;
            tmp.inertia = (param[i].inertia - org_param[i].inertia).array() / param[i].inertia.array();
            diff.push_back(tmp);
        }
        Jaco2Calibration::Jaco2CalibrationIO::save("/tmp/diff_param.txt",diff);
    }
    if(calib_mode == "acc"){
        calib.setGravityMagnitude(1.0);
        calib.setInitAccSamples(800);
        Jaco2Calibration::AccelerationSamples samples;
        Jaco2Calibration::Jaco2CalibrationIO::importAsciiData(file,samples);
        calib.calibrateAcc(samples);
        Jaco2Calibration::save("/tmp/acc_calib_param.txt",calib.getAccCalibration());
    }

    return 0;
}

