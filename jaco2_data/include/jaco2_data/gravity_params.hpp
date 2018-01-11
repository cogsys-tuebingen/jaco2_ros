#ifndef GRAVITY_PARAMS_HPP
#define GRAVITY_PARAMS_HPP
#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>
namespace Jaco2Calibration {
struct ApiGravitationalParams{
    std::vector<double> parameter;
};

inline void save(const std::string& filename, const ApiGravitationalParams& params)
{
    std::ofstream file(filename);
    YAML::Emitter yamlEmit(file);
    YAML::Node doc;
    for(auto p : params.parameter){
        doc["gravity_parameter"].push_back(p);
    }

    yamlEmit << doc;
}

inline void load(const std::string& filename, ApiGravitationalParams& params)
{
    YAML::Node doc = YAML::LoadFile(filename);
    doc = doc["gravity_parameter"];
    if(!doc.IsDefined() || !doc.IsSequence())
    {
        throw std::runtime_error("illegal document!");
    }
    for(auto node : doc){
        params.parameter.push_back(node.as<double>());
    }
}
}
#endif // GRAVITY_PARAMS_HPP
