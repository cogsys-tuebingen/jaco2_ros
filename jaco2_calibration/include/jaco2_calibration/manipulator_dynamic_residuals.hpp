#ifndef MANIPULATOR_DYNAMIC_RESIDUALS_HPP
#define MANIPULATOR_DYNAMIC_RESIDUALS_HPP

#include <ceres/ceres.h>
#include <tf/tf.h>
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>
#include <jaco2_calibration_utils/dynamic_calibration_sample.hpp>

namespace Jaco2Calibration {
typedef ceres::Jet<double,9> Vector9d;
typedef ceres::Jet<double,6> Vector6d;
class ManipulatorDynamicResiduals
{
public:
    ManipulatorDynamicResiduals(Jaco2DynamicModel* solver, DynamicCalibrationSample sample)
        : solver_(solver),
          sample_(sample)
    {
        std::vector<std::string> links_ =solver_->getLinkNames();
        for(auto link: links_) {
            masses_.push_back(solver_->getLinkMass(link));
        }
    }

    bool operator() ( const double* const params1,
                      const double* const params2,
                      const double* const params3,
                      const double* const params4,
                      const double* const params5,
                      const double* const params6,
                      double* residuals ) const
    {

//        std::vector<tf::Vector3> com(6);
//        std::vector<tf::Matrix3x3> inertia(6);

//        tf::Vector3  vect(params1[0], params1[1], params1[2]);

//        tf::Matrix3x3 mat(params1[3], params1[4], params1[5],
//                params1[4], params1[6], params1[7],
//                params1[5], params1[7], params1[8]);

//        com[0] = vect;
//        inertia[0] = mat;

//        com[1] = tf::Vector3(params2[0], params2[1], params2[2]);
//        inertia[1] = tf::Matrix3x3(params2[3], params2[4], params2[5],
//                params2[4], params2[6], params2[7],
//                params2[5], params1[7], params2[8]);

        std::vector<const double*> a;
        a.push_back(params1);
        a.push_back(params2);
        a.push_back(params3);
        a.push_back(params4);
        a.push_back(params5);
        a.push_back(params6);


        for(std::size_t i = 0; i < links_.size(); ++i) {
            std::string link = links_[i];
////            double mass = solver_->getLinkMass(link);
//            std::size_t id = i*9;
//            tf::Vector3  vect(params[id], params[id + 1], params[id + 2]);

//            tf::Matrix3x3 mat(params[id + 3], params[id + 4], params[id + 5],
//                              params[id + 4], params[id + 6], params[id + 7],
//                              params[id + 5], params[id + 7], params[id + 8]);

            Eigen::Vector3d  vect(a[i][0], a[i][1], a[i][2]);

            Eigen::Matrix3d mat;
            mat << a[i][3], a[i][4], a[i][5],
                   a[i][4], a[i][6], a[i][7],
                   a[i][5], a[i][7], a[i][8];

            solver_->changeDynamicParams(link, masses_[i], vect, mat, sample_.gravity(0), sample_.gravity(1), sample_.gravity(2));
        }
        std::vector<double> modelTorques;
        int ec = solver_->getTorques(sample_.jointPos, sample_.jointVel, sample_.jointAcc, modelTorques);
        if(ec >= 0)
        {
            double sum = 0;
//            std::size_t endJoint = solver_->getKDLSegmentIndex(link_);
            for(std::size_t i = 0; i < 6; ++i){
                sum += fabs(modelTorques[i] - sample_.jointTorque[i]);
//                residuals[i] =  fabs(modelTorques[i] - sample_.jointTorque[i]);
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

    static ceres::CostFunction* Create (Jaco2DynamicModel* solver, DynamicCalibrationSample sample)
    {
        return ( new ceres::NumericDiffCostFunction< ManipulatorDynamicResiduals, ceres::CENTRAL, 1, 9, 9, 9, 9, 9, 9 > (
                     new ManipulatorDynamicResiduals( solver, sample), ceres::TAKE_OWNERSHIP ) );
    }

private:
    Jaco2DynamicModel* solver_;
    const DynamicCalibrationSample sample_;
    std::vector<std::string> links_;
    std::vector<double> masses_;


};
}




#endif // MANIPULATOR_DYNAMIC_RESIDUALS_HPP

