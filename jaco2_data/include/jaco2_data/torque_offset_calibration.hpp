#ifndef TORQUE_OFFSET_CALIBRATION_HPP
#define TORQUE_OFFSET_CALIBRATION_HPP
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <ostream>
#include <fstream>

namespace Jaco2Calibration {
enum SineFuncParamType{
    AMPLITUDE = 0,
    FREQUENCY = 1,
    PHASE = 2,
};

struct SineFunc{

    SineFunc()
    {
        params.resize(n_params);
        for(auto val : params){
            val = 0;
        }
    }

    double& amplitude()
    {
        return params[AMPLITUDE];
    }

    const double& amplitude() const
    {
        return params[AMPLITUDE];
    }

    double& frequency()
    {
        return params[FREQUENCY];
    }

    const double& frequency() const
    {
        return params[FREQUENCY];
    }

    double& phase()
    {
        return params[PHASE];
    }

    const double& phase() const
    {
        return params[PHASE];
    }

    double eval(double angle) const
    {
        return params[AMPLITUDE] * std::sin(params[FREQUENCY] * angle + params[PHASE]);
    }

    double operator() (double angle) const
    {
        return eval(angle);
    }

    Eigen::Vector3d gradParam(double angle) const
    {
        Eigen::Vector3d res;
        res(0) =  std::sin(params[FREQUENCY] * angle + params[PHASE]);
        res(1) = params[AMPLITUDE] * angle * std::cos(params[FREQUENCY] * angle + params[PHASE]);
        res(2) = params[AMPLITUDE] * std::cos(params[FREQUENCY] * angle + params[PHASE]);

        return res;
    }

    void gradParam(double angle, std::vector<double>& res) const
    {
        res.resize(n_params);
        res[AMPLITUDE] =  std::sin(params[FREQUENCY] * angle + params[PHASE]);
        res[FREQUENCY] = params[AMPLITUDE] * angle * std::cos(params[FREQUENCY] * angle + params[PHASE]);
        res[PHASE] = params[AMPLITUDE] * std::cos(params[FREQUENCY] * angle + params[PHASE]);
    }

    static const std::size_t n_params = 3;

    std::vector<double> params;
};

struct ActuatorTorqueOffset{

    ActuatorTorqueOffset(std::size_t n_sine = 0)
    {
        function.resize(n_sine);
    }

    double& amplitude(std::size_t comp)
    {
        return function[comp].params[AMPLITUDE];
    }

    const double& amplitude(std::size_t comp) const
    {
        return function[comp].params[AMPLITUDE];
    }

    double& frequency(std::size_t comp)
    {
        return function[comp].params[FREQUENCY];
    }

    const double& frequency(std::size_t comp) const
    {
        return function[comp].params[FREQUENCY];
    }

    double& phase(std::size_t comp)
    {
        return function[comp].params[PHASE];
    }

    const double& phase(std::size_t comp) const
    {
        return function[comp].params[PHASE];
    }

    void setParams(const std::vector<double>& param)
    {
        function.resize(param.size()/SineFunc::n_params);
        std::size_t counter = 0;
        for(auto val : param){
            std::size_t id = counter/ SineFunc::n_params;
            switch (counter %  SineFunc::n_params) {
            case 0:
                amplitude(id) = val;
                break;
            case 1:
                frequency(id) = val;
                break;
            case 2:
                phase(id) = val;
                break;
            default:
                break;
            }
            ++counter;
        }
    }

    std::vector<double> getParams() const
    {
        std::vector<double> res;
        for(auto f : function){
            res.emplace_back(f.amplitude());
            res.emplace_back(f.frequency());
            res.emplace_back(f.phase());
        }
        return res;
    }

    double eval(double angle) const
    {
        double res = 0;
        for(auto func : function){
            res += func(angle);
        }
        return res;
    }

    double operator() (double angle) const
    {
        return eval(angle);
    }

    void gradParm(double angle, Eigen::VectorXd& res) const
    {
        res.setZero(function.size() *SineFunc::n_params);

        std::size_t i = 0;
        for(auto f : function){
            res.block(i,0,3,1) = f.gradParam(angle);
            i += SineFunc::n_params;
        }
    }

    void gradParm(double angle, std::vector<double>& res) const
    {
        res.clear();
        for(auto f : function){
            std::vector<double> tmp;
            f.gradParam(angle, tmp);
            res.insert(res.end(), tmp.begin(), tmp.end());
        }
    }

    std::size_t id;
    std::vector<SineFunc> function;
};

struct TorqueOffsetCalibration{

    double eval(std::size_t id, double angle) const
    {
        double res = 0;
        const ActuatorTorqueOffset& ac = calibration[id];
        for(auto func = ac.function.begin(); func < ac.function.end(); ++func){
            res += func->eval(angle);
        }
        return res;
    }

    double operator() (std::size_t id, double angle) const
    {
        return eval(id, angle);
    }

    std::vector<double> eval(const std::vector<double>& angles) const
    {
        std::vector<double> res;
        res.resize(calibration.size());
        if(calibration.size() == angles.size()){
            auto it_res = res.begin();
            auto it_angle = res.begin();
            for(auto actuator : calibration){
                *it_res = actuator(*it_angle);
                ++it_res;
                ++it_angle;
            }
        }
        return res;
    }

    std::vector<double> operator ()(const std::vector<double>& angles)
    {
        return eval(angles);
    }

    void save(std::string name)
    {
        std::ofstream file(name);
        YAML::Emitter yamlEmit(file);
        YAML::Node doc;

        doc["torque_calibration"] = calibration;

        yamlEmit << doc;
    }

    void load(std::string file)
    {
        YAML::Node doc = YAML::LoadFile(file);
        auto ndata = doc["torque_calibration"];
        if(!ndata.IsDefined() || !ndata.IsSequence()){
            throw std::runtime_error("illegal document!");
        }

        calibration = ndata.as<std::vector<ActuatorTorqueOffset>>();

    }

    std::vector<ActuatorTorqueOffset> calibration;
};
}

namespace YAML {
template<>
struct convert<Jaco2Calibration::SineFunc> {
    static Node encode(const Jaco2Calibration::SineFunc& rhs) {
        Node node;
        node["amplitude"] = rhs.amplitude();
        node["frequency"] = rhs.frequency();
        node["phase"] = rhs.phase();
        return node;
    }

    static bool decode(const Node& node, Jaco2Calibration::SineFunc& rhs) {

        auto namp = node["amplitude"];
        if(!namp.IsDefined()){
            return false;
        }
        rhs.amplitude() = namp.as<double>();

        auto nfreq = node["frequency"];
        if(!nfreq.IsDefined()){
            return false;
        }
        rhs.frequency() = nfreq.as<double>();

        auto npha = node["phase"];
        if(!npha.IsDefined()){
            return false;
        }
        rhs.phase() = npha.as<double>();

        return true;
    }
};

template<>
struct convert<Jaco2Calibration::ActuatorTorqueOffset> {
    static Node encode(const Jaco2Calibration::ActuatorTorqueOffset& rhs) {
        Node node;

        node["parameters"] = rhs.function;

        return node;
    }

    static bool decode(const Node& node, Jaco2Calibration::ActuatorTorqueOffset& rhs) {

        auto data = node["parameters"];
        if(!data.IsDefined()){
            return false;
        }
        if(!data.IsSequence()) {
            return false;
        }

        rhs.function = data.as<std::vector<Jaco2Calibration::SineFunc>>();

        return true;
    }
};
}
#endif // TORQUE_OFFSET_CALIBRATION_HPP
