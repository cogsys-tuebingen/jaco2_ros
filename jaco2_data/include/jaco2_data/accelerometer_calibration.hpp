#ifndef ACCELEROMETERCALIBRATION_HPP
#define ACCELEROMETERCALIBRATION_HPP
#include <string>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace Jaco2Calibration {
struct EIGEN_ALIGN16 AccelerometerCalibrationParam {

    AccelerometerCalibrationParam(){}

    AccelerometerCalibrationParam(const std::string& name, double biasX, double biasY, double biasZ,
                                   double mXX, double mXY, double mXZ,
                                   double mYX, double mYY, double mYZ,
                                   double mZX, double mZY, double mZZ)
        : acc_name(name),
          bias(biasX, biasY, biasZ)

    {
        misalignment << mXX, mXY, mXZ,
                mYX, mYY, mYZ,
                mZX, mZY, mZZ;
    }

    AccelerometerCalibrationParam(const std::string& name, const Eigen::Matrix<double, 3,1>& vec,
                                   const Eigen::Matrix<double, 3,3>& ms,
                                   const Eigen::Matrix<double, 3,3>& scale)
        : acc_name(name),
          bias(vec),
          misalignment(ms*scale)

    {}

    void setMisalignmant(const Eigen::Matrix<double, 3, 3>& ms, const Eigen::Matrix<double, 3,3> scale)
    {
        misalignment = ms * scale;
    }

    std::string acc_name;
    Eigen::Matrix<double, 3,1> bias;
    Eigen::Matrix<double, 3,3> misalignment;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

inline void save(const std::string& filename, const std::vector<Jaco2Calibration::AccelerometerCalibrationParam>& params)
{
    std::ofstream file(filename);
    YAML::Emitter yamlEmit(file);
    YAML::Node doc;
    for(AccelerometerCalibrationParam param : params){
        YAML::Node pNode;
        pNode["acc_name"] = param.acc_name;
        pNode["bias_x"] = param.bias(0);
        pNode["bias_y"] = param.bias(1);
        pNode["bias_z"] = param.bias(2);
        Eigen::Matrix< double, 3 , 3> ms_mat = param.misalignment;
        pNode["ms_xx"] = ms_mat(0,0);
        pNode["ms_xy"] = ms_mat(0,1);
        pNode["ms_xz"] = ms_mat(0,2);
        pNode["ms_yx"] = ms_mat(1,0);
        pNode["ms_yy"] = ms_mat(1,1);
        pNode["ms_yz"] = ms_mat(1,2);
        pNode["ms_zx"] = ms_mat(2,0);
        pNode["ms_zy"] = ms_mat(2,1);
        pNode["ms_zz"] = ms_mat(2,2);
        doc["parameter"].push_back(pNode);
    }
    yamlEmit << doc;
}

inline void loadAccCalib(const std::string& filename, std::vector<Jaco2Calibration::AccelerometerCalibrationParam>& params)
{
    YAML::Node doc = YAML::LoadFile(filename);
    doc = doc["parameter"];
    if(!doc.IsDefined())
    {
        throw std::runtime_error("illegal document!");
    }
    for(auto it = doc.begin(); it != doc.end(); ++it){
        auto pNode = *it;
        std::string name(pNode["acc_name"].as<std::string>());
        double x = pNode["bias_x"].as<double>();
        double y = pNode["bias_y"].as<double>();
        double z = pNode["bias_z"].as<double>();
        double mXX = pNode["ms_xx"].as<double>();
        double mXY = pNode["ms_xy"].as<double>();
        double mXZ = pNode["ms_xz"].as<double>();
        double mYX = pNode["ms_yx"].as<double>();
        double mYY = pNode["ms_yy"].as<double>();
        double mYZ = pNode["ms_yz"].as<double>();
        double mZX = pNode["ms_zx"].as<double>();
        double mZY = pNode["ms_zy"].as<double>();
        double mZZ = pNode["ms_zz"].as<double>();
        AccelerometerCalibrationParam param(name, x, y,z,
                                             mXX, mXY, mXZ,
                                             mYX, mYY, mYZ,
                                             mZX, mZY, mZZ);

        params.push_back(param);
    }
}

inline bool getParam(const std::string& acc_name, const std::vector<Jaco2Calibration::AccelerometerCalibrationParam>& params,
                     Jaco2Calibration::AccelerometerCalibrationParam& result)
{
    for(auto param : params){
        if(param.acc_name == acc_name){
            result.acc_name = acc_name;
            result.bias = param.bias;
            result.misalignment = param.misalignment;
            return true;
        }
    }
    return false;
}

}
#endif // ACCELEROMETERCALIBRATION_HPP

