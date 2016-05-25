#ifndef COM_INERTIA_RESIDUALS_HPP
#define COM_INERTIA_RESIDUALS_HPP
#include <ceres/ceres.h>
#include <tf/tf.h>
#include <jaco2_kin_dyn/jaco2_kinematics_dynamics.h>
#include <jaco2_calibration/dynamic_calibration_sample.hpp>

//namespace jaco2_calib {
typedef ceres::Jet<double,9> Vector9d;
class ComInetriaResiduals
{
public:
    ComInetriaResiduals(Jaco2KinematicsDynamicsModel* solver, DynamicCalibrationSample sample, std::string& link)
        : solver_(solver),
          sample_(sample),
          link_(link)
    {}

    bool operator() ( const double* const params, double* residuals ) const
    {
        double mass = solver_->getLinkMass(link_);
        tf::Vector3  vect(params[0], params[1], params[2]);

        tf::Matrix3x3 mat(params[3], params[4], params[5],
                          params[4], params[6], params[7],
                          params[5], params[7], params[8]);

        solver_->changeDynamicParams(link_ ,mass, vect, mat);
        std::vector<double> modelTorques;
        int ec = solver_->getTorques(sample_.jointPos, sample_.jointVel, sample_.jointAcc, modelTorques);
        if(ec >= 0)
        {
            for(std::size_t i = 0; i < 6; ++i){
                residuals[i] = modelTorques[i] - sample_.jointTorque[i];
            }
            return true;

        }
        else
        {
            std::cerr <<"Chaned Parameters results in the solver not converging." << std::endl;
            return false;
        }

    }

     bool operator() ( const Vector9d* const params, Vector9d* residuals ) const
     {
         double mass = solver_->getLinkMass(link_);
         tf::Vector3  vect(params->v(0), params->v(1), params->v(2));

         tf::Matrix3x3 mat(params->v(3), params->v(4), params->v(5),
                           params->v(4), params->v(6), params->v(7),
                           params->v(5), params->v(7), params->v(8));

         solver_->changeDynamicParams(link_ ,mass, vect, mat);
         std::vector<double> modelTorques;
         int ec = solver_->getTorques(sample_.jointPos, sample_.jointVel, sample_.jointAcc, modelTorques);
         if(ec >= 0)
         {
             for(std::size_t i = 0; i < 6; ++i){
                 residuals->v(i) = modelTorques[i] - sample_.jointTorque[i];
             }
             return true;

         }
         else
         {
             std::cerr <<"Chaned Parameters results in the solver not converging." << std::endl;
             return false;
         }
     }

    static ceres::CostFunction* Create (Jaco2KinematicsDynamicsModel* solver, DynamicCalibrationSample sample, std::string& link )
    {
        return ( new ceres::AutoDiffCostFunction< ComInetriaResiduals, 6, 9 > (
                     new ComInetriaResiduals( solver, sample, link ) ) );
    }

private:
    Jaco2KinematicsDynamicsModel* solver_;
    const DynamicCalibrationSample sample_;
    const std::string link_;


};
//}
#endif // COM_INERTIA_RESIDUALS_HPP

