#include <jaco2_utils/configuration_list.h>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace jaco2_utils;


bool Configuration::equal(const Configuration &other,const std::vector<double>& threshold) const
{
    if(other.angles.size() == angles.size()){
        bool eq = true;
        for(std::size_t i = 0; i < angles.size(); ++i){
            double diff = std::abs(angles[i] - other.angles[i]);
            eq &= diff < threshold[i];
        }
        return eq;
    }
    else{
        return false;
    }
}

std::string Configuration::to_string() const
{
    std::string res;
    for(auto val : angles){
        res += std::to_string(val) + ";";
    }
    return res;
}



bool  ConfigurationList::contains(const Configuration& conf) const
{
    if(conf.angles.size() == jointNames.size()){
        bool test = false;
        for(auto elemets : configurations){
            test |= elemets.equal(conf, offsets);
            if(test){
                return test;
            }
        }
        return test;

    }
    else{
        return false;
    }
}

bool ConfigurationList::getIndex(const Configuration& c, std::size_t& id) const{
    if(c.angles.size() == jointNames.size()){
        bool test = false;
        id = 0;
        for(auto elemets : configurations){
            test |= elemets.equal(c, offsets);
            ++id;
            if(test){
                return test;
            }
        }
        return test;

    }
    else{
        return false;
    }
}


void ConfigurationList::add(const Configuration &c)
{
    if(!contains(c)){
        configurations.push_back(c);
    }
}

void  ConfigurationList::save(std::string filename) const
{
    std::ofstream file(filename);

    for(auto name : jointNames){
        file << name << ";";
    }

    file << std::endl;

    for(auto conf : configurations){
        file  << conf.to_string() << std::endl;
    }
}

bool  ConfigurationList::load(std::string filename)
{


    std::string line;
    std::ifstream infile;
    char delimiter = ';';

    jointNames.clear();
    //        configurations.clear();

    infile.open ( filename );
    if ( infile.is_open() )
    {

        char value[256];

        std::getline ( infile,line );
        std::stringstream ss;
        ss << line ;

        while( ss.getline( value, 256, delimiter )){
            jointNames.push_back(value);
        }


        while ( std::getline ( infile,line ) ){

            std::stringstream ss;
            ss << line ;

            Configuration cfg;
            while( ss.getline( value, 256, delimiter ))
            {
                double val = std::atof(value);
                cfg.angles.push_back(val);
            }

            configurations.push_back(cfg);

        }
        infile.close();
        n_joints_ = jointNames.size();
        return true;
    }
    else{
        return false;
    }


}
