#ifndef CONFIGURATION_LIST_H
#define CONFIGURATION_LIST_H
#include <vector>
#include <string>

namespace jaco2_utils {

struct Configuration{

    bool equal(const Configuration& other, const std::vector<double> &threshold) const;
    std::string to_string() const;

    std::vector<double> angles;
};

struct ConfigurationList{

    bool contains(const Configuration& conf) const;
    bool getIndex(const Configuration& c, std::size_t& id) const;

    void add(const Configuration& c);

    void save(std::string filename) const;
    bool load(std::string filename);

    std::size_t n_joints_;
    std::vector<double> offsets;
    std::vector<std::string> jointNames;
    std::vector<Configuration> configurations;
};

}
#endif // CONFIGURATION_LIST_H
