#ifndef YAML_TO_TRANSFORM_CONVERSION_HPP
#define YAML_TO_TRANSFORM_CONVERSION_HPP
#include <vector>
#include <jaco2_data/suppress_warnings_start.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <jaco2_data/suppress_warnings_end.h>
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

void frames(const std::vector<KDLTransformation>& in, std::vector<KDL::Frame>& out);


}
#endif // YAML_TO_TRANSFORM_CONVERSION_HPP

