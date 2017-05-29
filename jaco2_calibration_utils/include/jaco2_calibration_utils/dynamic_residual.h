#ifndef DYNAMIC_RESIDUAL_H
#define DYNAMIC_RESIDUAL_H

#include <string>
#include <Eigen/Dense>
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>
#include <jaco2_calibration_utils/dynamic_calibration_sample.hpp>
#include <jaco2_calibration_utils/dynamic_calibrated_parameters.hpp>

enum ResidualType{
    ALL = 0,
    STATIC = 1,
    DYNAMIC = 2
};

enum ParamertType{
    MASS_PARAMS = 0,
    STATIC_PARAMS = 3,
    DYNAMIC_PARAMS = 4
};

class DynamicResidual
{
public:
    DynamicResidual(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);

    void loadData(std::string data_file);
    void useInitialGuess(bool use, double factor=0);
    void setScaleMatrix(Eigen::MatrixXd& scale);
    bool calculteMatrix();

    const Eigen::MatrixXd& getRegressionMatrix();
    const Eigen::MatrixXd& getTorques();

    inline std::size_t getProblemSize() const {return n_cols_;}
    inline Eigen::MatrixXd getInitialParams() const {return initial_params_;}
    inline std::vector<std::string> getLinkNames() const {return model_.getLinkNames();}
    inline std::size_t getNumOfSamples() const {return n_samples_;}
    inline std::size_t getNumOfLinks() const {return n_links_;}
    inline std::size_t getNumOfParams() const {return n_cols_;}
    inline std::size_t getNumOfRows() const {return n_rows_;}
    inline void setStaticVelThreshold(double val){static_vel_thresold_ = val;}
    inline void setStaticAccThreshold(double val){static_acc_thresold_ = val;}
    inline void setInitalParams(Eigen::MatrixXd& params){initial_params_ = params;}
    void setResidualType(int type);


    double getResidual(const std::vector<double> &x, std::vector<double> &grad);
    double getResidual(const Eigen::MatrixXd &params, std::vector<double> &grad);

    /**
     * @brief residual static residual function used for nlopt.
     * @param x     the parameters to optimize
     * @param grad  the gradient of the problem
     * @param data  pointer to a DynamicResidual object
     * @return
     */
    static double residual(const std::vector<double>& x, std::vector<double>& grad, void* data);

    /**
     * @brief paramVector2Eigen converts a std::vector containing some parameters to
     * a Eigen Matrix<double 60,1> containg all parameters.
     * @param x     vector containg only the parameters which are optimized.
     * @param res   vector containing all 60 parameters
     */
    void paramVector2Eigen(const std::vector<double>& x, Eigen::MatrixXd& res) const;
    /**
     * @brief paramEigen2Vector shortens the Eigen vector with all parameters to a std::vector
     * containing only the obtimized ones.
     * @param mat       Eigen representation for all parameters
     * @param result    std::vector with the optimized vectors
     */
    void paramEigen2Vector(const Eigen::MatrixXd& mat, std::vector<double>& result);
    /**
     * @brief paramEigen2Vector shortens the Eigen vector with all parameters to a std::vector
     * containing only the obtimized ones.
     * @param mat       Eigen representation for all parameters
     * @param result    std::vector with the optimized vectors. It is expcected that result has the correct size!
     * @param type      type STATIC or DYNAMIC
     */
    static void paramEigen2Vector(const Eigen::MatrixXd& mat, std::vector<double>& result, int type);

private:
    bool staticSample(const Jaco2Calibration::DynamicCalibrationSample& sample) const;
    void selectData(std::vector<Jaco2Calibration::DynamicCalibrationSample>& selected);

private:
    Jaco2KinDynLib::Jaco2DynamicModel model_;
    bool calculated_matrix_;
    bool use_initial_;
    bool set_scale_mat_;
    int residual_type_;
    double static_vel_thresold_;
    double static_acc_thresold_;

    Eigen::MatrixXd reg_mat_;
    Eigen::MatrixXd torques_;
    Eigen::MatrixXd initial_params_;
    Eigen::MatrixXd init_scale_;
    Eigen::MatrixXd uncertainty_scale_;
    std::vector<Jaco2Calibration::DynamicCalibrationSample> samples_;
    std::size_t n_samples_;
    const static std::size_t n_param_;
    const static std::size_t n_param_static_;
    const static std::size_t n_param_dynamic_;
    std::size_t n_opt_;
    std::size_t n_links_;
    std::size_t n_cols_;
    std::size_t n_rows_;

};

#endif // DYNAMIC_RESIDUAL_H
