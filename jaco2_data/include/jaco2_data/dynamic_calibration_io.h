#ifndef DYNAMIC_CALIBRATION_IO_H
#define DYNAMIC_CALIBRATION_IO_H
#include <jaco2_data/dynamic_calibrated_parameters.hpp>

namespace Jaco2Calibration {

    void save(std::string name, const DynamicCalibratedParametersCollection &params);
    void loadDynParm(std::string filename, DynamicCalibratedParametersCollection& params);
}
#endif // DYNAMIC_CALIBRATION_IO_H
