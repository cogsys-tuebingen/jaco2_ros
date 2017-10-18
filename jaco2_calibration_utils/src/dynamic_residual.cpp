#include <jaco2_calibration_utils/dynamic_residual.h>

#include <jaco2_calibration_utils/jaco2_calibration_io.h>

using namespace jaco2_data;
using namespace Jaco2Calibration;
const std::size_t DynamicResidual::n_param_ = 10;
const std::size_t DynamicResidual::n_param_static_ = 4;
const std::size_t DynamicResidual::n_param_dynamic_ = 6;

DynamicResidual::DynamicResidual(const std::string &robot_model, const std::string &chain_root, const std::string &chain_tip):
    model_(robot_model, chain_root, chain_tip),
    calculated_matrix_(false),
    use_initial_(false),
    set_scale_mat_(false),
    residual_type_(ALL),
    n_opt_(n_param_)
{
    //initial parameters
    DynamicCalibratedParametersCollection init_param;
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
    uncertainty_scale_ = Eigen::MatrixXd::Identity(n_links_, n_links_);
}

void DynamicResidual::loadData(std::string data_file)
{
    Jaco2Calibration::Jaco2CalibrationIO::importAsciiDataWithGravity(data_file, samples_);
    n_samples_ = samples_.size();
}

void DynamicResidual::setData(JointStateDataCollection& samples)
{
    samples_ = samples;
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

const Eigen::MatrixXd& DynamicResidual::getRegressionMatrix()
{
    if(!calculated_matrix_){
        calculteMatrix();
    }
    return reg_mat_;
}

const Eigen::MatrixXd &DynamicResidual::getTorques()
{
    if(!calculated_matrix_){
        calculteMatrix();
    }
    return torques_;
}

void DynamicResidual::setResidualType(int type)
{
    switch (type) {
    case ALL:
        residual_type_ = type;
        n_opt_ = n_param_;
        break;
    case STATIC:
        residual_type_ = type;
        n_opt_ = n_param_static_;
        break;
    case DYNAMIC:
        residual_type_ = type;
        n_opt_ = n_param_dynamic_;
        break;
    default:
        residual_type_ = ALL;
        n_opt_ = n_param_;
        break;
    }
}

bool DynamicResidual::calculteMatrix()
{
    std::vector<std::string> links = model_.getLinkNames();
    std::string first_link = links.front();
    std::string last_link = links.back();

    JointStateDataCollection* used_samples;
    JointStateDataCollection selected;
    if(residual_type_ != ALL){
        selectData(selected);
        used_samples = &selected;
    }
    else{
        used_samples = &samples_;
    }

    n_samples_ = used_samples->size();
    if(n_samples_ == 0){
        throw std::logic_error("Samples used for optimization are empty! Forgot to set data path or thresholds?");
    }


    std::size_t n_points = n_links_ * n_samples_;
    n_rows_ = n_points;
    if(use_initial_){
        n_rows_ +=  n_cols_;
    }

    reg_mat_ = Eigen::MatrixXd::Zero(n_rows_, n_cols_);
    torques_ = Eigen::MatrixXd::Zero(n_rows_, 1);

    auto it = used_samples->begin();
    for(std::size_t n = 0; n < n_samples_; ++ n) {

        const JointStateData& sample = *it;
        ++it;

        if(sample.position.size() == n_links_) {
            Eigen::MatrixXd sample_mat  =
                    model_.getRigidBodyRegressionMatrix(first_link, last_link,
                                                        sample.position,
                                                        sample.velocity,
                                                        sample.acceleration,
                                                        sample.gravity(0),
                                                        sample.gravity(1),
                                                        sample.gravity(2));

            Eigen::MatrixXd sample_tau = Eigen::MatrixXd::Zero(n_links_, 1);
            for(std::size_t i = 0; i < n_links_; ++i) {
                sample_tau(i) = sample.torque[i];
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
    calculated_matrix_ = true;
    return true;
}

bool DynamicResidual::staticSample(const jaco2_data::JointStateData &sample) const
{
    double vel_norm = 0;
    for(auto vel : sample.velocity){
        vel_norm += vel *vel;
    }
    vel_norm = std::sqrt(vel_norm);

    double acc_norm = 0;
    for(auto acc : sample.acceleration){
        acc_norm += acc *acc;
    }
    acc_norm = sqrt(acc_norm);

    return vel_norm < static_vel_thresold_ && acc_norm < static_acc_thresold_;
}

void DynamicResidual::selectData(jaco2_data::JointStateDataCollection &selected)
{
    for(auto data : samples_){
        bool is_static = staticSample(data);
        if(residual_type_ == STATIC){
            if(is_static){
                selected.push_back(data);
            }
        }
        else if(residual_type_ == DYNAMIC){
            if(!is_static){
                selected.push_back(data);
            }
        }
    }
}

double DynamicResidual::getResidual(const std::vector<double> &x, std::vector<double> &grad)
{
    Eigen::MatrixXd params;
    paramVector2Eigen(x, params);

    return getResidual(params, grad);
}

double DynamicResidual::getResidual(const Eigen::MatrixXd &params, std::vector<double> &grad)
{
    if(!calculated_matrix_){
        calculteMatrix();
    }

    Eigen::MatrixXd diff = torques_ - reg_mat_ * params;
    Eigen::MatrixXd diffT = diff.transpose();
    Eigen::MatrixXd scalar = diffT * diff;

    double cost = scalar.array().sum();

    if (!grad.empty())  {
        Eigen::MatrixXd  e_grad = -2.0 * diffT * reg_mat_;
        paramEigen2Vector(e_grad, grad);
//        std::cout << "grad" << std::endl;
    }

//    std::cout << cost << std::endl;
    return cost;
}


double DynamicResidual::residual(const std::vector<double> &x, std::vector<double> &grad, void *data)
{

    DynamicResidual * object = (DynamicResidual*) data;

    double fitness = object->getResidual(x, grad);
    return fitness;
}

void DynamicResidual::paramVector2Eigen(const std::vector<double> &x, Eigen::MatrixXd &res) const
{
    std::size_t nparam = x.size();
    switch(residual_type_){
    case ALL:
    {
        if(nparam != n_cols_){
            throw std::logic_error("Expected in total (number of links * 10 ) paramters.");
        }
        res = Eigen::MatrixXd::Zero(x.size(),1);
        std::size_t id = 0;
        for(auto val : x){
            res(id) = val;
            ++id;
        }
        break;
    }
    case STATIC:
    {
        if(nparam != n_links_ * n_param_static_){
            throw std::logic_error("Expected in total (number of links * 4) paramters.");
        }
        res = initial_params_;
        std::size_t i = 0;
        for(auto value : x){
            std::size_t id = i / 4 * n_param_ + i % 4;
            res(id) = value;
            ++i;
        }
        break;
    }
    case DYNAMIC:
    {
        if(nparam != n_links_ * n_param_dynamic_){
            throw std::logic_error("Expected in total (number of links * 6) paramters.");
        }
        res = initial_params_;
        std::size_t i = 0;
        for(auto value : x){
            std::size_t id = i / n_param_dynamic_ * n_param_ + i % n_param_dynamic_ + n_param_static_;
            res(id) = value;
            ++i;
        }
        break;
    }
    }
}

void DynamicResidual::paramEigen2Vector(const Eigen::MatrixXd &mat, std::vector<double> &result)
{
    if(result.size() != n_links_ * n_opt_){
        result.resize(n_links_ * n_opt_);
    }
    paramEigen2Vector(mat, result, residual_type_);
}

void DynamicResidual::paramEigen2Vector(const Eigen::MatrixXd &mat, std::vector<double> &result, int type)
{
    auto it = result.begin();
    int lower_limit = 0;
    int limit = 9;

    switch(type){
    case STATIC:
        limit = STATIC_PARAMS;
        break;
    case DYNAMIC:
        lower_limit = DYNAMIC_PARAMS;
        break;
    default:
        break;
    }
    std::size_t max = std::max(mat.cols(), mat.rows());

    for(std::size_t i = 0; i < max; ++ i){
        std::size_t ptype = i % n_param_;
        if(ptype >= lower_limit && ptype <= limit){
            *it = mat(i);
            ++it;
        }
    }
}
