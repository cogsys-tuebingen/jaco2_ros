#include <jaco2_calibration_utils/jaco2_calibration_io.h>
#include <yaml-cpp/yaml.h>

//namespace Jaco2Calibration{
using namespace Jaco2Calibration;

void Jaco2CalibrationIO::save(std::string name, const std::vector<DynamicCalibratedParameters>& params)
{
    std::ofstream file(name);
    YAML::Emitter yamlEmit(file);
    YAML::Node doc;
    for(DynamicCalibratedParameters param : params)
    {
        YAML::Node pNode;
        pNode["link_name"] = param.linkName;
        pNode["mass"] = param.mass;
        Eigen::Vector3d com = param.coM;
        pNode["com_x"] = com(0);
        pNode["com_y"] = com(1);
        pNode["com_z"] = com(2);
        Eigen::Matrix3d inertia = param.inertia;
        pNode["Ixx"] = inertia(0,0);
        pNode["Ixy"] = inertia(0,1);
        pNode["Ixz"] = inertia(0,2);
        pNode["Iyy"] = inertia(1,1);
        pNode["Iyz"] = inertia(1,1);
        pNode["Izz"] = inertia(2,2);
        doc["parameter"].push_back(pNode);
    }
    yamlEmit << doc;

}


void Jaco2CalibrationIO::loadDynParm(std::string filename, std::vector<DynamicCalibratedParameters>& params)
{
    YAML::Node doc = YAML::LoadFile(filename);
    doc = doc["parameter"];
    if(!doc.IsDefined())
    {
        throw std::runtime_error("illegal document!");
    }
    for(auto it = doc.begin(); it != doc.end(); ++it){
        auto pNode = *it;
        DynamicCalibratedParameters param;
        std::string name(pNode["link_name"].as<std::string>());
        param.linkName = name;
        param.mass = pNode["mass"].as<double>();
        double x = pNode["com_x"].as<double>();
        double y = pNode["com_y"].as<double>();
        double z = pNode["com_z"].as<double>();
        param.coM = Eigen::Vector3d(x, y, z);
        double ixx = pNode["Ixx"].as<double>();
        double ixy = pNode["Ixy"].as<double>();
        double ixz = pNode["Ixz"].as<double>();
        double iyy = pNode["Iyy"].as<double>();
        double iyz = pNode["Iyz"].as<double>();
        double izz = pNode["Izz"].as<double>();
        param.inertia << ixx, ixy, ixz,
                         ixy, iyy, iyz,
                         ixz, iyz, izz;
        params.push_back(param);
    }
}

void Jaco2CalibrationIO::save(std::string name, std::vector<DynamicCalibrationSample> samples, std::string delimiter)
{
    std::ofstream file(name);
    //    std::string delimiter(";");
    file << "time " << delimiter;
    for(std::size_t i = 0; i <samples[0].jointPos.size(); ++i)
    {
        file << "joint_pos_" << std::to_string(i) << delimiter;
    }
    for(std::size_t i = 0; i <samples[0].jointVel.size(); ++i)
    {
        file << "joint_vel_" << std::to_string(i) << delimiter;
    }
    for(std::size_t i = 0; i <samples[0].jointAcc.size(); ++i)
    {
        file << "joint_acc_" << std::to_string(i) << delimiter;
    }
    for(std::size_t i = 0; i <samples[0].jointTorque.size(); ++i)
    {
        file << "joint_torque_" << std::to_string(i) << delimiter;
    }
    file << std::endl;
    for(DynamicCalibrationSample sample : samples)
    {
        file << sample.toString(delimiter) << std::endl;
    }
}

void Jaco2CalibrationIO::importAsciiData(std::string filename, std::vector<DynamicCalibrationSample>& samples, const char delimiter)
{
    samples.clear();

    std::string line;
    std::ifstream infile;

    infile.open ( filename );
    if ( infile.is_open() )
    {


        int l = 0;
        char value[256];

        std::getline ( infile,line );
        while ( std::getline ( infile,line ) )
        {

            std::stringstream ss;
            ss<< line ;

            DynamicCalibrationSample sample;
            int i = 0;
            while( ss.getline( value, 256, delimiter ))
            {
                double val = std::atof(value);
                if(i==0){
                    sample.time = val;
                }
                if( i > 0 && i < 7){
                    sample.jointPos[i-1] = val;
                }
                if(i >= 7 && i<13){
                    sample.jointVel[i-7] = val;
                }
                if(i >= 13 && i<19){
                    sample.jointAcc[i-13] = val;
                }
                if(i >= 19 && i<25){
                    sample.jointTorque[i-19] = val;
                }
                ++i;
            }

            samples.push_back(sample);

        }
        l++;
    }
    infile.close();
}

void Jaco2CalibrationIO::importAsciiDataWithGravity(std::string filename, std::vector<DynamicCalibrationSample>& samples, const char delimiter)
{
    samples.clear();

    std::string line;
    std::ifstream infile;

    infile.open ( filename );
    if ( infile.is_open() )
    {


        int l = 0;
        char value[256];

        std::getline ( infile,line );
        while ( std::getline ( infile,line ) )
        {

            std::stringstream ss;
            ss<< line ;

            DynamicCalibrationSample sample;
            int i = 0;
            while( ss.getline( value, 256, delimiter ))
            {
                double val = std::atof(value);
                if(i==0){
                    sample.time = val;
                }
                if( i > 0 && i < 7){
                    sample.jointPos[i-1] = val;
                }
                if(i >= 7 && i<13){
                    sample.jointVel[i-7] = val;
                }
                if(i >= 13 && i<19){
                    sample.jointAcc[i-13] = val;
                }
                if(i >= 19 && i<25){
                    sample.jointTorque[i-19] = val;
                }
                if(i >= 25 && i<28){
                    sample.gravity(i-25) = val;
                }
                ++i;
            }

            samples.push_back(sample);

        }
        l++;
    }
    infile.close();
}

void Jaco2CalibrationIO::importAsciiData(std::string filename, AccelerationSamples& samples, const char delimiter)
{
    samples.clear();

    std::string line;
    std::ifstream infile;

    infile.open ( filename );
    if ( infile.is_open() )
    {


//        int l = 0;
        char value[256];

        std::getline ( infile,line );
        while ( std::getline ( infile,line ) )
        {

            std::stringstream ss;
            ss<< line ;

            AccelerationData sample[samples.nJoints];
            int i = 0;
            while( ss.getline( value, 256, delimiter ))
            {
                double val = std::atof(value);
                if(i==0){
                    for(std::size_t i = 0; i < samples.nJoints; ++i){
                        sample[0].time = val;
                    }
                }
                if( i > 0 && i < 4){
                    sample[0].vector[i-1] = val;
                }
                if(i >= 4 && i<7){
                    sample[1].vector[i-4] = val;
                }
                if(i >= 7 && i<10){
                    sample[2].vector[i-7] = val;
                }
                if(i >= 10 && i<13){
                    sample[3].vector[i-10] = val;
                }
                if(i >= 13 && i<16){
                    sample[4].vector[i-13] = val;
                }
                if(i >= 16 && i<19){
                    sample[5].vector[i-16] = val;
                }
                ++i;
            }

            for(std::size_t i = 0; i < samples.nJoints; ++i){
                samples.push_back(i, sample[i]);
            }
        }
    }
    infile.close();
}

//}


