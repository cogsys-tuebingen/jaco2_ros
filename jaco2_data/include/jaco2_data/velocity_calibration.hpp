#ifndef VELOCITY_CALIBRATION_HPP
#define VELOCITY_CALIBRATION_HPP
#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>
namespace Jaco2Calibration {
struct VelocityCalibrationParams{
    std::vector<double> parameter;
};

inline void save(const std::string& filename, const VelocityCalibrationParams& params)
{
    std::ofstream file(filename);
    YAML::Emitter yamlEmit(file);
    YAML::Node doc;
    for(auto p : params.parameter){
        doc["velocity_scale_parameter"].push_back(p);
    }

    yamlEmit << doc;
}

inline void load(const std::string& filename, VelocityCalibrationParams& params)
{
    YAML::Node doc = YAML::LoadFile(filename);
    doc = doc["velocity_scale_parameter"];
    if(!doc.IsDefined() || !doc.IsSequence())
    {
        throw std::runtime_error("illegal document!");
    }
    for(auto node : doc){
        params.parameter.push_back(node.as<double>());
    }
}
}
#endif // VELOCITY_CALIBRATION_HPP
