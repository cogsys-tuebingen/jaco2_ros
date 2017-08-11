#ifndef JACO2CALIBRATION_H
#define JACO2CALIBRATION_H
//System
#include <vector>
#include <ceres/ceres.h>
//ROS
#include <imu_tk/base.h>
#include <imu_tk/calibration.h>
//Jaco2 ROS
#include <jaco2_data/accelerometer_calibration.hpp>
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>
#include <jaco2_calibration_utils/dynamic_calibration_sample.hpp>
#include <jaco2_calibration_utils/dynamic_calibration_sample.hpp>
#include <jaco2_calibration_utils/acceleration_samples.hpp>
#include <jaco2_calibration_utils/dynamic_calibrated_parameters.hpp>
namespace Jaco2Calibration {

class Jaco2Calibration
{
public:
    Jaco2Calibration(std::string& urdf_param, std::string& root, std::string& tip);
    ~Jaco2Calibration();

    int calibrateCoMandInertia(const jaco2_data::JointStateDataCollection &samples);
    int calibrateArmDynamic(const std::vector<DynamicCalibrationSample> &samples);

    bool calibrateAcc(const AccelerationSamples & samples);

    void setInitAccSamples(int n){initAccSamples_ = n;}
    void setGravityMagnitude(double g){gravityMag_ = g;}

    DynamicCalibratedParametersCollection getDynamicCalibration() const { return dynParams_;}
    std::vector<AccelerometerCalibrationParam> getAccCalibration() const { return accParams_;}

    std::string getTipFrame() const {return model_.getTipLink();}
    std::string getRootFrame() const {return model_.getRootLink();}
    std::vector<std::string> getLinkNames() const {return model_.getLinkNames();}

    DynamicCalibratedParametersCollection getDynamicUrdfParam() const;



private:
    Jaco2KinDynLib::Jaco2DynamicModel model_;
    DynamicCalibratedParametersCollection dynParams_;
    int initAccSamples_;
    double gravityMag_;
    std::vector<AccelerometerCalibrationParam> accParams_;

    void convert(const std::size_t& idx, const AccelerationSamples& samples, std::vector<imu_tk::TriadData>& data );
    void convert(const std::size_t &idx, const std::vector<imu_tk::TriadData> &data, AccelerationSamples &samples);
};
}
#endif // JACO2CALIBRATION_H
