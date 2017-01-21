#ifndef YAML_TO_TRANSFORM_CONVERSION_HPP
#define YAML_TO_TRANSFORM_CONVERSION_HPP
#include <vector>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <yaml-cpp/yaml.h>

namespace Jaco2KinDynLib{

struct KDLTransformation {
    KDLTransformation(){}

    std::string name;
    std::string parent;
    KDL::Frame frame;


};

void save(std::string name, const std::vector<KDLTransformation>& transforms);

void load(std::string filename, std::vector<KDLTransformation>& transforms);

void convert(YAML::Node& node, std::vector<KDLTransformation>& transforms);


}
#endif // YAML_TO_TRANSFORM_CONVERSION_HPP

