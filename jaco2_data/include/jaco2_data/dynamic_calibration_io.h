#ifndef DYNAMIC_CALIBRATION_IO_H
#define DYNAMIC_CALIBRATION_IO_H
#include <jaco2_data/dynamic_calibrated_parameters.hpp>

namespace Jaco2Calibration {
class DynCalibrationIO{
public:
    DynCalibrationIO() {}

    static void save(std::string name, const DynamicCalibratedParametersCollection &params);
    static void loadDynParm(std::string filename, DynamicCalibratedParametersCollection& params);
};
}
#endif // DYNAMIC_CALIBRATION_IO_H
