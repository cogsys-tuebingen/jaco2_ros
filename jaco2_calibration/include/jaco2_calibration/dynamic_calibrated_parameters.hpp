#ifndef DYNAMIC_CALIBRATED_PARAMETERS_HPP
#define DYNAMIC_CALIBRATED_PARAMETERS_HPP
#include <string>
//#include <tf/tf.h>
#include <Eigen/Core>
namespace Jaco2Calibration {

struct DynamicCalibratedParameters
{
    std::string linkName;
    double mass;
    Eigen::Vector3d coM;
    Eigen::Matrix3d inertia;

};

typedef std::vector<Jaco2Calibration::DynamicCalibratedParameters> Jaco2ManipulatorDynParams;


void to_vector(const Jaco2Calibration::Jaco2ManipulatorDynParams& in, std::vector<double>& out, std::vector<std::string>& links,
               bool move_mass = true, bool move_coM = true, bool move_inertia = true)
{
    out.clear();
    links.clear();
    for(auto param : in)
    {
        if(move_mass) {
            links.push_back(param.linkName);
        }
        if(move_coM) {
            out.push_back(param.coM(0));
            out.push_back(param.coM(1));
            out.push_back(param.coM(2));
        }
        if(move_inertia) {
            out.push_back(param.inertia(0,0));
            out.push_back(param.inertia(0,1));
            out.push_back(param.inertia(0,2));
            out.push_back(param.inertia(1,1));
            out.push_back(param.inertia(1,2));
            out.push_back(param.inertia(2,2));
        }
    }

}

void vector2Jaco2ManipulatorDynParams(const std::vector<double>& in, const std::vector<std::string> links,
                                      Jaco2Calibration::Jaco2ManipulatorDynParams& out,
                                      bool move_mass = true, bool move_coM = true, bool move_inertia = true)
{
   int elements_per_link = 0;
   if(move_mass) {
       elements_per_link += 1;
   }
   if(move_coM) {
       elements_per_link += 3;
   }
   if(move_inertia) {
       elements_per_link += 6;
   }
   if(links.size() != in.size() / elements_per_link)
   {
       throw std::logic_error("dimension mismatch between double vector and link name vector");
   }
   out.clear();
   int link_counter = 0;
   double values[elements_per_link];
   for(std::size_t i = 0; i < in.size(); ++i) {
       int local_id = i % elements_per_link;
       values[local_id] = in[i];
       if( i > 0 &&  local_id == elements_per_link - 1) {
           DynamicCalibratedParameters param;
           param.linkName = links[link_counter];
           ++link_counter;
           switch (elements_per_link) {
           case 1:
               param.mass = values[0];
               break;
           case 3:
               param.coM = Eigen::Vector3d(values[0], values[1], values[2]);
               break;
           case 4:
               param.mass = values[0];
               param.coM = Eigen::Vector3d(values[1], values[2], values[3]);
               break;
           case 6:
               param.inertia << values[0], values[1], values[2],
                                values[1], values[3], values[4],
                                values[2], values[4], values[5];
               break;
           case 7:
               param.mass = values[0];
               param.inertia << values[1], values[2], values[3],
                                values[2], values[4], values[5],
                                values[2], values[5], values[6];
               break;
           case 9:
               param.coM = Eigen::Vector3d(values[0], values[1], values[2]);
               param.inertia << values[3], values[4], values[5],
                                values[4], values[6], values[7],
                                values[5], values[7], values[8];
               break;
           case 10:
               param.mass = values[0];
               param.coM = Eigen::Vector3d(values[1], values[2], values[3]);
               param.inertia << values[4], values[5], values[6],
                                values[5], values[7], values[8],
                                values[6], values[8], values[9];
               break;
           default:
               throw std::logic_error("something terribly wrong");
               break;
           }
           out.push_back(param);
       }
   }
}

std::vector<std::string> getLinkNames(const Jaco2Calibration::Jaco2ManipulatorDynParams& params)
{
    std::vector<std::string> result;
    for(auto param : params) {
        result.push_back(param.linkName);
    }
    return result;
}


}
#endif // DYNAMIC_CALIBRATED_PARAMETERS_HPP

