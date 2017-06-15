#ifndef JOINT_VEL_POS_INTEGRATOR_H
#define JOINT_VEL_POS_INTEGRATOR_H
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>
#include <deque>

namespace Jaco2KinDynLib {



struct IntegrationData{

    double  dt;
    std::vector<double> torques;  // control command
    std::vector<double> vel;      // current velocity
    std::vector<double> pos;      // current position
};

struct IntegrationDataBuffer{
    IntegrationDataBuffer(std::size_t size=3):
        max_size(size){}

    void emplace_back(IntegrationData& d)
    {
        data.emplace_back(d);
        check_size();
    }

    void push_back(const IntegrationData& d)
    {
        data.push_back(d);
        check_size();
    }

    void check_size()
    {
        while(data.size() > max_size){
            data.pop_front();
        }
    }

    std::size_t size()
    {
        return data.size();
    }

    double time(std::size_t i)
    {
        return data[i].dt;
    }

    std::size_t max_size;
    std::deque<IntegrationData> data;
};

class JointVelPosEstimator
{
public:
    JointVelPosEstimator();
    JointVelPosEstimator(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);

    void setModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);
    void setInitalValues(const IntegrationData &data);

    void setGravity(double gx, double gy, double gz) const;
    void estimate(const IntegrationData &data);
    void estimateGfree(const IntegrationData &data);
//    void integrate(const IntegrationData &data);

    std::vector<double> getCurrentPosition() const;
    std::vector<double> getCurrentVelocity() const;
    std::vector<double> getCurrentAcceleration() const;
    std::vector<double> getModelTorques(std::vector<double> currentAcc = std::vector<double>{0,0,0,0,0,0}) const;

    inline double getTime() const { return time_;}

private:
    mutable Jaco2KinDynLib::Jaco2DynamicModel model_;
    bool loaded_model_;
    bool initial_values;
    IntegrationDataBuffer buffer_;
    std::vector<double> current_pos_;
    std::vector<double> current_vel_;
    std::vector<double> current_acc_;
    double time_;

};

}
#endif // JOINT_VEL_POS_INTEGRATOR_H
