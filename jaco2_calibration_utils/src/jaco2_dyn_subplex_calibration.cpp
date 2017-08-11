#include <jaco2_calibration_utils/dynamic_residual.h>
#include <jaco2_calibration_utils/dynamic_calibrated_parameters.hpp>
#include <jaco2_calibration_utils/jaco2_calibration_io.h>
#include <nlopt.hpp>


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


struct RbCalibrationBounds{


    RbCalibrationBounds(Eigen::MatrixXd& init_param, double fac, std::vector<int> types)
    {
        for(std::size_t i = 0; i < init_param.rows(); ++i){
            int type = i % 10;
            double value =  init_param(i,0);
            bool contains  = std::find(types.begin(),types.end(), type)!= types.end();
            if( type == MASS && contains){
                lower_bounds.push_back(0);
                upper_bounds.push_back(fac * value);
            }
            else if(contains){
                if(value == 0){
                    lower_bounds.push_back(-0.2);
                    upper_bounds.push_back(0.2);
                }
                else{
                    if(value < 0)
                    {
                        lower_bounds.push_back(fac*value);
                        upper_bounds.push_back((1-fac)*value);
                    }
                    else{
                        lower_bounds.push_back((1-fac)*value);
                        upper_bounds.push_back(fac*value);
                    }
                }
            }
        }
    }

    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
};


//void vector2eigen(const std::vector<double>& data, Eigen::MatrixXd& result)
//{
//    for(std::size_t i = 0; i < result.rows(); ++i){
//        for(std::size_t j = 0; j < result.cols(); ++j){
//            result(i,j) = data.at(i*result.cols() + j);
//        }
//    }
//}

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
        model.setResidualType(STATIC);
        model.setStaticAccThreshold(0.01 * sqrt(6));
        model.setStaticVelThreshold(0.01 * sqrt(6));

        Eigen::MatrixXd init_params = model.getInitialParams();
        Eigen::MatrixXd final_param;
        bool suc = model.calculteMatrix();

        if(suc){
            for(std::size_t mode = 0; mode < 1; ++mode){
            std::size_t problem_size;
            std::vector<int> tooptimize;

            if(mode == 0){
                problem_size = 24;
                tooptimize = {MASS, MASS_TIMES_COM_X,  MASS_TIMES_COM_Y, MASS_TIMES_COM_Z};
            }
            else{
                problem_size = 36;
                model.setResidualType(DYNAMIC);
                model.calculteMatrix();
                tooptimize = {INERTIA_XX, INERTIA_XY, INERTIA_XZ, INERTIA_YY, INERTIA_YZ, INERTIA_ZZ};
            }
            nlopt::opt optimizer(nlopt::LD_SLSQP, problem_size);



            RbCalibrationBounds bounds(init_params, 1.5, tooptimize);


            optimizer.set_lower_bounds(bounds.lower_bounds);
            optimizer.set_upper_bounds(bounds.upper_bounds);
            optimizer.set_xtol_rel(0.01);
            optimizer.set_min_objective(DynamicResidual::residual, &model);

            std::vector<double> grad(problem_size);
            double f0 = model.getResidual(init_params, grad);
            double f = f0;
            std::vector<double> x;
            model.paramEigen2Vector(init_params, x);
            ROS_INFO_STREAM("Start Optimization " << mode);
            ros::Time start = ros::Time::now();

            optimizer.optimize(x, f);

            ros::Time end = ros::Time::now();
            ros::Duration dur = end - start;


            ROS_INFO_STREAM("Optimization took " << dur.toSec() << " s" );
            ROS_INFO_STREAM("Initial residual: " << f0 << " | Final residual: " << f);

            std::size_t n_params = model.getNumOfParams();
            std::size_t n_links = model.getNumOfLinks();
            std::size_t n_samples = model.getNumOfSamples();
            final_param = Eigen::MatrixXd::Zero(n_params, 1);
            model.paramVector2Eigen(x, final_param);


            const Eigen::MatrixXd& reg_mat = model.getRegressionMatrix();
            const Eigen::MatrixXd& torques = model.getTorques();
            Eigen::MatrixXd mean_diff_inital = getMeanJointList(arrayResidual(reg_mat, init_params, torques), n_samples, n_links);
            Eigen::MatrixXd mean_diff_final = getMeanJointList(arrayResidual(reg_mat, final_param, torques), n_samples, n_links);

            ROS_INFO_STREAM("Initial mean resiudal per joint: " << mean_diff_inital);
            ROS_INFO_STREAM("Final  mean resiudal per joint: " << mean_diff_final);

            init_params = final_param;
            model.setInitalParams(init_params);
        }


            Jaco2Calibration::DynamicCalibratedParametersCollection optimized_params;
            Jaco2Calibration::to_Jaco2ManipulatorDynParams(final_param, model.getLinkNames(), optimized_params);
            Jaco2Calibration::Jaco2CalibrationIO::save(output,optimized_params);


        }


        return 0;
    }
    else{
        return 42;
    }

}
