#ifndef DYNAMIC_RESIDUAL_H
#define DYNAMIC_RESIDUAL_H

#include <string>
#include <Eigen/Dense>
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>
#include <jaco2_calibration_utils/dynamic_calibration_sample.hpp>
#include <jaco2_calibration_utils/dynamic_calibrated_parameters.hpp>

class DynamicResidual
{
public:
    DynamicResidual(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);

    void loadData(std::string data_file);
    void useInitialGuess(bool use, double factor=0);
    void setScaleMatrix(Eigen::MatrixXd& scale);
    bool calculteMatrix();

    Eigen::MatrixXd getRegressionMatrix();
    Eigen::MatrixXd getTorques();

    std::size_t getProblemSize() const {return n_cols_;}
    Eigen::MatrixXd getInitialParams() const {return initial_params_;}

private:

private:
    Jaco2KinDynLib::Jaco2DynamicModel model_;
    bool calculated_matrix_;
    bool use_initial_;
    bool set_scale_mat_;

    Eigen::MatrixXd reg_mat_;
    Eigen::MatrixXd torques_;
    Eigen::MatrixXd initial_params_;
    Eigen::MatrixXd init_scale_;
    Eigen::MatrixXd uncertainty_scale_;
    std::vector<Jaco2Calibration::DynamicCalibrationSample> samples_;
    std::size_t n_samples_;
    const static std::size_t n_param_;
    std::size_t n_links_;
    std::size_t n_cols_;
    std::size_t n_rows_;

};

#endif // DYNAMIC_RESIDUAL_H
