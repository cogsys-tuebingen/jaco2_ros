#ifndef COM_INERTIA_RESIDUALS_HPP
#define COM_INERTIA_RESIDUALS_HPP
#include <ceres/ceres.h>
#include <tf/tf.h>
#include <jaco2_kin_dyn_lib/jaco2_kinematics_dynamics.h>
#include <jaco2_calibration_utils/dynamic_calibration_sample.hpp>

namespace Jaco2Calibration {
typedef ceres::Jet<double,9> Vector9d;
typedef ceres::Jet<double,6> Vector6d;
class ComInetriaResiduals
{
public:
    ComInetriaResiduals(Jaco2KinematicsDynamicsModel* solver, DynamicCalibrationSample sample, std::string& link)
        : solver_(solver),
          sample_(sample),
          link_(link)//,
//          mass_(solver_->getLinkMass(link))
    {}

    bool operator() ( const double* const params, double* residuals ) const
    {
        double mass = solver_->getLinkMass(link_);
        Eigen::Vector3d  vect(params[0], params[1], params[2]);

        Eigen::Matrix3d mat;
        mat << params[3], params[4], params[5],
               params[4], params[6], params[7],
               params[5], params[7], params[8];

//        solver_->changeDynamicParams(link_ ,mass, vect, mat);
        solver_->changeDynamicParams(link_, mass, vect, mat, sample_.gravity(0), sample_.gravity(1), sample_.gravity(2));
        std::vector<double> modelTorques;
        int ec = solver_->getTorques(sample_.jointPos, sample_.jointVel, sample_.jointAcc, modelTorques);
        if(ec >= 0)
        {
            double sum = 0;
            std::size_t endJoint = solver_->getKDLSegmentIndex(link_);
            for(std::size_t i = 0; i <= endJoint; ++i){
//                sum += fabs(modelTorques[i] - sample_.jointTorque[i]);

                double diff = modelTorques[i] - sample_.jointTorque[i];
                //sum of Mahalanobis-Distance
                sum += sqrt(diff*(1.4/0.16)*diff);
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

