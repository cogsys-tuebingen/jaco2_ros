#ifndef COM_INERTIA_RESIDUALS_HPP
#define COM_INERTIA_RESIDUALS_HPP
#include <ceres/ceres.h>
#include <tf/tf.h>
#include <jaco2_kin_dyn_lib/jaco2_kinematics_dynamics.h>
#include <jaco2_calibration/dynamic_calibration_sample.hpp>

namespace Jaco2Calibration {
typedef ceres::Jet<double,9> Vector9d;
typedef ceres::Jet<double,6> Vector6d;
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
            double sum = 0;
            for(std::size_t i = 0; i < 6; ++i){
                sum += fabs(modelTorques[i] - sample_.jointTorque[i]);
            }
            residuals[0] = sum;
            return true;

        }
        else
        {
            std::cerr <<"Chaned Parameters results in the solver not converging." << std::endl;
            return false;
        }

    }

//     bool operator() ( const Vector9d* const params, Vector9d* residuals ) const
//     {
//         double mass = solver_->getLinkMass(link_);
//         tf::Vector3  vect(params[0].a, params[1].a, params[2].a);

//         tf::Matrix3x3 mat(params[3].a, params[4].a, params[5].a,
//                           params[4].a, params[6].a, params[6].a,
//                           params[5].a, params[7].a, params[8].a);

//         solver_->changeDynamicParams(link_ ,mass, vect, mat);
//         std::vector<double> modelTorques;
//         int ec = solver_->getTorques(sample_.jointPos, sample_.jointVel, sample_.jointAcc, modelTorques);
//         if(ec >= 0)
//         {
//             double sum = 0;
//             for(std::size_t i = 0; i < 6; ++i){
////                 residuals[i].v(0) = modelTorques[i] - sample_.jointTorque[i];
////                 residuals[i] = Vector9d(modelTorques[i] - sample_.jointTorque[i]);
//                 sum += fabs(modelTorques[i] - sample_.jointTorque[i]);
//             }
//             residuals[0] = Vector9d(sum);
//             return true;

//         }
//         else
//         {
//             std::cerr <<"Chaned Parameters results in the solver not converging." << std::endl;
//             return false;
//         }
//     }

    static ceres::CostFunction* Create (Jaco2KinematicsDynamicsModel* solver, DynamicCalibrationSample sample, std::string& link )
    {
        return ( new ceres::NumericDiffCostFunction< ComInetriaResiduals, ceres::CENTRAL, 1, 9 > (
                     new ComInetriaResiduals( solver, sample, link ), ceres::TAKE_OWNERSHIP ) );
    }

private:
    Jaco2KinematicsDynamicsModel* solver_;
    const DynamicCalibrationSample sample_;
    const std::string link_;


};
}
#endif // COM_INERTIA_RESIDUALS_HPP

