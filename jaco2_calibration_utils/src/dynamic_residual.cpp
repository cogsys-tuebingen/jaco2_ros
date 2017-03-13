#include <jaco2_calibration_utils/dynamic_residual.h>

#include <jaco2_calibration_utils/jaco2_calibration_io.h>


const std::size_t DynamicResidual::n_param_ = 10;

DynamicResidual::DynamicResidual(const std::string &robot_model, const std::string &chain_root, const std::string &chain_tip):
    model_(robot_model, chain_root, chain_tip),
    calculated_matrix_(false),
    use_initial_(false),
    set_scale_mat_(false)
{
    //initial parameters
    std::vector<Jaco2Calibration::DynamicCalibratedParameters> init_param;
    for(auto link : model_.getLinkNames())
    {
        Jaco2Calibration::DynamicCalibratedParameters param;
        param.linkName = link;
        param.mass = model_.getLinkMass(link);
        param.coM = model_.getLinkCoM(link);
        param.inertia = model_.getLinkInertiaCoM(link);
        init_param.push_back(param);
    }

    Jaco2Calibration::to_eigen(init_param, initial_params_);
    n_links_ = model_.getNrOfSegments();
    n_cols_ = n_param_ * n_links_;

    init_scale_ = Eigen::MatrixXd::Identity(n_cols_, n_cols_);
    uncertainty_scale_ = Eigen::MatrixXd::Identity(n_cols_, n_cols_);
}

void DynamicResidual::loadData(std::string data_file)
{
    Jaco2Calibration::Jaco2CalibrationIO::importAsciiDataWithGravity(data_file, samples_);
    n_samples_ = samples_.size();

}

void DynamicResidual::useInitialGuess(bool use, double factor)
{
    use_initial_ = use;
    init_scale_ *= factor;
}

void DynamicResidual::setScaleMatrix(Eigen::MatrixXd &scale)
{
    std::size_t rows = scale.rows();
    std::size_t cols = scale.cols();
    if(rows == n_links_ && cols == n_links_){
        uncertainty_scale_ = scale;
    }
    else{
        throw std::logic_error("Dimension mismatch of matrix.");
    }
}

Eigen::MatrixXd DynamicResidual::getRegressionMatrix()
{
    if(!calculated_matrix_){
        calculteMatrix();
    }
    return reg_mat_;
}

Eigen::MatrixXd DynamicResidual::getTorques()
{
    if(!calculated_matrix_){
        calculteMatrix();
    }
    return torques_;
}

bool DynamicResidual::calculteMatrix()
{
    std::vector<std::string> links = model_.getLinkNames();
    std::string first_link = links.front();
    std::string last_link = links.back();
    std::size_t n_points = n_links_ * n_samples_;
    n_rows_ = n_points;
    if(use_initial_){
        n_rows_ +=  n_cols_;
    }

    reg_mat_ = Eigen::MatrixXd::Zero(n_rows_, n_cols_);
    torques_ = Eigen::MatrixXd::Zero(n_rows_, 1);

    for(std::size_t n = 0; n <samples_.size(); ++ n) {

        Jaco2Calibration::DynamicCalibrationSample sample = samples_[n];
        if(sample.jointPos.size() == n_links_) {
            Eigen::MatrixXd sample_mat  =
                    model_.getRigidBodyRegressionMatrix(first_link, last_link,
                                                       sample.jointPos,
                                                       sample.jointVel,
                                                       sample.jointAcc,
                                                       sample.gravity(0),
                                                       sample.gravity(1),
                                                       sample.gravity(2));

            Eigen::MatrixXd sample_tau = Eigen::MatrixXd::Zero(n_links_, 1);
            for(std::size_t i = 0; i < n_links_; ++i) {
                sample_tau(i) = sample.jointTorque[i];
            }

            reg_mat_.block(n * n_links_, 0, n_links_, n_cols_) =  uncertainty_scale_ * sample_mat;
            if(use_initial_){
                torques_.block(n * n_links_, 0, n_links_, 1) = uncertainty_scale_ * (sample_tau - sample_mat * initial_params_);
            }
            else{
                torques_.block(n * n_links_, 0, n_links_, 1) = uncertainty_scale_ * sample_tau;
            }

        }
        else {
            std::string error = "Flawed sample detected with index " + std::to_string(n);
            throw std::logic_error(error);
            return false;
        }
    }

    if(use_initial_){
        reg_mat_.block(n_samples_, 0, n_cols_, n_cols_) = init_scale_;

    }
    return true;
}
