#ifndef TORQUE_OFFSET_LUT_HPP
#define TORQUE_OFFSET_LUT_HPP
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <ostream>
#include <fstream>
#include <jaco2_data/eigen_yaml.hpp>
namespace Jaco2Calibration {

typedef std::pair<std::size_t, std::size_t> LutIndex;
struct TorqueOffsetLut
{
    double at(std::size_t link, double angle) const
    {
        std::pair<std::size_t, std::size_t> id = index(link, angle);
        if(id.first < n_links){

            if(id.second > (std::size_t) steps(id.first) || angle < lower_limits(id.first)){
                while(angle > 720){
                    angle -= 360;
                }
                while(angle < -720){
                    angle += 360;
                }
                id = index(link, angle);
            }
            if(id.second >= (std::size_t) steps(id.first)){
                return lut(id.first, steps(id.first) -1);
            }
            else{
               return lut(id.first, id.second);
            }
        }
        else{
            throw std::logic_error("link index should be between 1 and 6.");
        }
    }

    void set(std::size_t link, double angle, double offset)
    {
        std::pair<std::size_t, std::size_t> id = index(link, angle);
        if(id.first < n_links){

            if(id.second > (std::size_t) steps(id.first) || angle < lower_limits(id.first)){
                while(angle > 720){
                    angle -= 360;
                }
                while(angle < -720){
                    angle += 360;
                }
                id = index(link, angle);
            }
            lut(id.first, id.second) = offset;
        }
        else{
            throw std::logic_error("link index should be between 1 and 6.");
        }

    }

    void initialize(const Eigen::VectorXd& lowerLimit, const Eigen::VectorXd& upper_limit, const Eigen::VectorXd& res)
    {
        lower_limits = lowerLimit;
        resolution = res;

        n_links = lowerLimit.size();
        steps.setZero(n_links);

        for(std::size_t i = 0; i < 6; ++i){
            std::size_t step = std::floor((upper_limit(i) - lower_limits(i))/ res(i));
            steps(i) = step;
        }

        lut.setZero(n_links, steps.maxCoeff());

    }

    double getAngle(std::size_t link, std::size_t n) const
    {
        if(link > 0 && link <= n_links){
            return lower_limits(link -1) + n * resolution(link -1);
        }
        else{
            throw std::logic_error("link index should be between 1 and 6.");
        }
    }

    /**
     * @brief index returns the index for given link and angle
     * @param link link index from 1 to 6
     * @param angle the joint angle
     * @return pair of link index and joint angle index
     */
    std::pair<std::size_t, std::size_t> index(std::size_t link, double angle) const
    {
        std::pair<std::size_t, std::size_t> res;

        res.first = link - 1;

        res.second = std::ceil((angle - lower_limits(res.first)) / resolution(res.first));
        return res;
    }

    void save(std::string name)
    {
        std::ofstream file(name);
        YAML::Emitter yamlEmit(file);
        YAML::Node doc;

        YAML::Node node;
        node["n_links"] = n_links;
        node["steps"] = steps;
        node["resolution"] = resolution;
        node["lower_limits"] = lower_limits;
        doc["meta"].push_back(node);
        YAML::Node data;
        data["data"] = lut;
        doc["lut"].push_back(data);

        yamlEmit << doc;

    }

    void load(std::string name)
    {
        YAML::Node doc = YAML::LoadFile(name);
        auto nmeta = doc["meta"];
        if(!nmeta.IsDefined()){
            throw std::runtime_error("illegal document!");
        }
        n_links      = nmeta[0]["n_links"].as<std::size_t>();
        steps        = nmeta[0]["steps"].as<Eigen::VectorXi>();
        resolution   = nmeta[0]["resolution"].as<Eigen::VectorXd>();
        lower_limits = nmeta[0]["lower_limits"].as<Eigen::VectorXd>();

        auto nlut = doc["lut"];
        if(!nlut.IsDefined()){
            throw std::runtime_error("illegal document!");
        }

        lut = nlut[0]["data"].as<Eigen::MatrixXd>();


    }

    std::size_t n_links = 6;

    Eigen::VectorXi steps;
    Eigen::VectorXd resolution;
    Eigen::VectorXd lower_limits;
    Eigen::MatrixXd lut;
};
}

#endif // TORQUE_OFFSET_LUT_HPP
