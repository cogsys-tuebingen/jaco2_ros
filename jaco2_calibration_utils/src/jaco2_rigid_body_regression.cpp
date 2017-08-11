#include <string>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>
#include <jaco2_calibration_utils/jaco2_calibration_io.h>
#include <jaco2_calibration_utils/acceleration_samples.hpp>
#include <jaco2_calibration_utils/dynamic_residual.h>

Eigen::MatrixXd jointMeanfromList(Eigen::MatrixXd list, int sample_size, int num_joints)
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

        const int num_links = 6;
        const int num_param = 10;

        const int num_cols = 60;
        const int num_params = 60;


        ros::init(argc, argv, "jaco2_rigid_body_regression_node");

        double lambda = 10;
        double u_scale = 1;

        ros::NodeHandle nh("~");
        nh.param<double>("lambda",lambda,lambda);
        nh.param<double>("u_scale",u_scale,u_scale);

        std::string urdf_param("/robot_description");
        std::string base("jaco_link_base");
        std::string tip("jaco_link_hand");

        DynamicResidual model(urdf_param, base, tip);

        model.loadData(input);
        model.useInitialGuess(true, lambda);

        std::size_t n_links = model.getNumOfLinks();
        std::size_t num_samples = model.getNumOfSamples();
        std::size_t num_points = n_links * num_samples;
        Eigen::MatrixXd uncertainty_scale = u_scale * Eigen::MatrixXd::Identity(n_links, n_links);
        model.setScaleMatrix(uncertainty_scale);

        bool suc = model.calculteMatrix();

        if(!suc){
            return 23;
        }

        Eigen::MatrixXd initial_param = model.getInitialParams();
        const Eigen::MatrixXd& reg_mat = model.getRegressionMatrix();
        const Eigen::MatrixXd& torques = model.getTorques();

        ROS_INFO_STREAM("Start Optimization");
        ros::Time start = ros::Time::now();

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(reg_mat, Eigen::ComputeThinU | Eigen::ComputeThinV | Eigen::FullPivHouseholderQRPreconditioner);
        Eigen::Matrix<double, num_cols, 1> param = svd.solve(torques);

        ros::Time end = ros::Time::now();
        ros::Duration dur = end - start;
        ROS_INFO_STREAM("Optimization took " << dur.toSec() << " s" );

        param += initial_param;

        Eigen::MatrixXd sol = (reg_mat * param).block(0,0, num_points, 1);
        Eigen::MatrixXd sample_tau = torques.block(0, 0, num_points, 1)  + reg_mat.block(0,0, num_points, num_cols) * initial_param;
        Eigen::MatrixXd tau_param = sol.block(0, 0,  num_points, 1);
        Eigen::MatrixXd diff = (tau_param.array().abs() - sample_tau.array().abs()).abs().matrix();
        double mean_diff = diff.array().mean();


        Eigen::MatrixXd inital_sol = reg_mat *initial_param;
        Eigen::MatrixXd inital_tau = inital_sol.block(0, 0, num_points, 1);
        Eigen::MatrixXd intitial_diff =  (sample_tau.array().abs() - inital_tau.array().abs()).abs().matrix();
        double mean_init_diff  = intitial_diff.array().mean();

        std::cout <<"Mean difference between calib. model and sensor: " << mean_diff << std::endl;
        std::cout <<"Mean difference between initial model and sensor: " <<  mean_init_diff << std::endl;

        std::cout << "mean diff per joint calib model: \n " << jointMeanfromList(diff, num_samples, num_links) << std::endl;
        std::cout << "mean diff per joint initial model: \n " << jointMeanfromList(intitial_diff, num_samples, num_links) << std::endl;

        Eigen::MatrixXd chi_sq = (sample_tau - sol).transpose() * (sample_tau - sol);
        std::cout << "chi square: " << chi_sq << std::endl;
        std::cout << "standard deviation: " << std::sqrt(chi_sq(0))/(num_samples - num_params) << std::endl;
        std::cout << "mean difference between initial parameters: " << (param - initial_param).array().abs().mean() << std::endl;


        if(mean_diff < 3.5)
        {
            Jaco2Calibration::DynamicCalibratedParametersCollection params;
            Jaco2Calibration::to_Jaco2ManipulatorDynParams(param, model.getLinkNames(),params);
            Jaco2Calibration::Jaco2CalibrationIO::save(output,params);
            Jaco2Calibration::DynamicCalibratedParametersCollection init_param;
            Jaco2Calibration::to_Jaco2ManipulatorDynParams(initial_param, model.getLinkNames(), init_param);
            Jaco2Calibration::Jaco2CalibrationIO::save("/tmp/cad_params.txt",init_param);
        }

        return 0;
    }
    else {
        return 42;
    }
}
