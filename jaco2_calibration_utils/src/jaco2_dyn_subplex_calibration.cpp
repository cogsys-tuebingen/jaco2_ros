#include <jaco2_calibration_utils/dynamic_residual.h>
#include <jaco2_calibration_utils/dynamic_calibrated_parameters.hpp>
#include <jaco2_calibration_utils/jaco2_calibration_io.h>
#include <nlopt.hpp>

struct SubplexData{
    Eigen::MatrixXd reg_mat;
    Eigen::MatrixXd torques;
};


Eigen::MatrixXd arrayResidual(const Eigen::MatrixXd& reg_mat, const Eigen::MatrixXd& params, const Eigen::MatrixXd& torques)
{
    Eigen::MatrixXd diff = (torques.array().abs() - (reg_mat * params).array().abs()).abs();
    return diff;
}


double residual(const Eigen::MatrixXd& reg_mat, const Eigen::MatrixXd& params, const Eigen::MatrixXd& torques)
{
    Eigen::MatrixXd opti_vec = torques - reg_mat * params;

    Eigen::MatrixXd diff = opti_vec.transpose() * opti_vec;

    double cost = diff.array().abs().sum();

    return cost;

}

double minfunc(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
    Eigen::MatrixXd params(x.size(),1);
    std::size_t id = 0;
    for(auto val : x){
        params(id) = val;
        ++id;
    }
    SubplexData * opt_data = (SubplexData*) data;

    Eigen::MatrixXd diff = opt_data->torques - opt_data->reg_mat * params;
    Eigen::MatrixXd diffT = diff.transpose();
    Eigen::MatrixXd scalar = diffT * diff;

    double cost = scalar.array().sum();

    //    double cost = residual(opt_data->reg_mat, params, opt_data->torques);
    //    Eigen::MatrixXd diff = opti_vec.transpose() * opti_vec;
    if (!grad.empty())  {
        Eigen::MatrixXd  e_grad = -2.0 * diffT * opt_data->reg_mat;
        std::size_t id_grad = 0;
        for(double& grad_i : grad){
            grad_i = e_grad(id_grad);
            ++id_grad;
        }
    }
    std::cout << cost << std::endl;
    return cost;
}
struct RbCalibrationBounds{

    enum ParamType
    {
        MASS = 0,
        MASS_TIMES_COM_X = 1,
        MASS_TIMES_COM_Y = 2,
        MASS_TIMES_COM_Z = 3,
        INERTIA_XX = 4,
        INERTIA_XY = 5,
        INERTIA_XZ = 6,
        INERTIA_YY = 7,
        INERTIA_YZ = 8,
        INERTIA_ZZ = 9
    };

    RbCalibrationBounds(Eigen::MatrixXd& init_param, double fac)
    {
        for(std::size_t i = 0; i < init_param.rows(); ++i){
            int type = i % 10;
            double value =  init_param(i,0);
            if( type == MASS){
                lower_bounds.push_back(0);
                upper_bounds.push_back(fac * value);
            }
            else{
                if(value == 0){
                    lower_bounds.push_back(-1.0);
                    upper_bounds.push_back(1.0);
                }
                else{
//                    double bound = fac * std::abs(value);
                    if(value < 0)
                    {
                        lower_bounds.push_back(fac*value);
                        upper_bounds.push_back((1-fac)*value);
                    }
                    else{
                        lower_bounds.push_back((1-fac)*value);
                        upper_bounds.push_back(fac*value);
                    }
//                    lower_bounds.push_back(-bound);
//                    upper_bounds.push_back(bound);
                }
            }
        }
    }

    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
};

void eigen2vector(const Eigen::MatrixXd& mat, std::vector<double>& result)
{
    result.clear();
    for(std::size_t i = 0; i < mat.rows(); ++i){
        for(std::size_t j = 0; j < mat.cols(); ++j){
            result.push_back(mat(i,j));
        }
    }
}

void vector2eigen(const std::vector<double>& data, Eigen::MatrixXd& result)
{
    for(std::size_t i = 0; i < result.rows(); ++i){
        for(std::size_t j = 0; j < result.cols(); ++j){
            result(i,j) = data.at(i*result.cols() + j);
        }
    }
}

Eigen::MatrixXd getMeanJointList(Eigen::MatrixXd list, int sample_size, int num_joints)
{
    Eigen::MatrixXd joint_mean = Eigen::MatrixXd::Zero(num_joints, 1);

    for(std::size_t i = 0; i < sample_size; ++i) {
        for(std::size_t j = 0; j < num_joints; ++j) {
            joint_mean(j) += std::abs(list(i*num_joints+j));
        }
    }
    return joint_mean/sample_size;
}



int main(int argc, char *argv[])
{
    if(argc > 2) {
        std::string input = argv[1];
        std::string output = argv[2];


        double lambda = 10;
        double u_scale = 1;

        ros::init(argc, argv, "jaco2_rigid_body_subplex_node");

        ros::NodeHandle nh("~");

        std::string urdf_param("/robot_description");
        std::string base("jaco_link_base");
        std::string tip("jaco_link_hand");

        DynamicResidual model(urdf_param, base, tip);

        model.loadData(input);
        bool suc = model.calculteMatrix();
        if(suc){
            std::size_t problem_size = model.getProblemSize();
            nlopt::opt optimizer(nlopt::LD_SLSQP, problem_size);
            SubplexData sdata;
            sdata.reg_mat = model.getRegressionMatrix();
            sdata.torques = model.getTorques();

            Eigen::MatrixXd init_params = model.getInitialParams();
            RbCalibrationBounds bounds(init_params, 1.2);

            optimizer.set_lower_bounds(bounds.lower_bounds);
            optimizer.set_upper_bounds(bounds.upper_bounds);
            optimizer.set_xtol_rel(0.01);
            optimizer.set_min_objective(minfunc, &sdata);

            double f0 = residual(sdata.reg_mat, init_params, sdata.torques);
            double f = f0;
            std::vector<double> x;
            eigen2vector(init_params, x);
            ROS_INFO_STREAM("Start Optimization");
            ros::Time start = ros::Time::now();

            optimizer.optimize(x, f);

            ros::Time end = ros::Time::now();
            ros::Duration dur = end - start;


            ROS_INFO_STREAM("Optimization took " << dur.toSec() << " s" );
            ROS_INFO_STREAM("Initial residual: " << f0 << " | Final residual: " << f);

            std::size_t n_params = model.getNumOfParams();
            std::size_t n_links = model.getNumOfLinks();
            std::size_t n_samples = model.getNumOfSamples();
            Eigen::MatrixXd final_param = Eigen::MatrixXd::Zero(n_params, 1);
            vector2eigen(x, final_param);

            Eigen::MatrixXd mean_diff_inital = getMeanJointList(arrayResidual(sdata.reg_mat,init_params,sdata.torques),n_samples,n_links);
            Eigen::MatrixXd mean_diff_final = getMeanJointList(arrayResidual(sdata.reg_mat,final_param,sdata.torques),n_samples,n_links);

            ROS_INFO_STREAM("Initial mean resiudal per joint: " << mean_diff_inital);
            ROS_INFO_STREAM("Final  mean resiudal per joint: " << mean_diff_final);


            Jaco2Calibration::Jaco2ManipulatorDynParams optimized_params;
            Jaco2Calibration::to_Jaco2ManipulatorDynParams(x,model.getLinkNames(),optimized_params);
            Jaco2Calibration::Jaco2CalibrationIO::save(output,optimized_params);


        }


        return 0;
    }
    else{
        return 42;
    }

}
