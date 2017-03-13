#include <string>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>
#include <jaco2_calibration_utils/jaco2_calibration_io.h>
#include <jaco2_calibration_utils/acceleration_samples.hpp>

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
        Jaco2KinDynLib::Jaco2DynamicModel model(urdf_param, base, tip);

        //initial parameters
        std::vector<Jaco2Calibration::DynamicCalibratedParameters> init_param;
        for(auto link : model.getLinkNames())
        {
            Jaco2Calibration::DynamicCalibratedParameters param;
            param.linkName = link;
            param.mass = model.getLinkMass(link);
            param.coM = model.getLinkCoM(link);
            param.inertia = model.getLinkInertiaCoM(link);
            init_param.push_back(param);
        }
        Eigen::MatrixXd initial_param;

        Jaco2Calibration::to_eigen(init_param,initial_param);



        // load samples
        std::vector<Jaco2Calibration::DynamicCalibrationSample> samples_org;
        Jaco2Calibration::Jaco2CalibrationIO::importAsciiDataWithGravity(input, samples_org);
        std::vector<Jaco2Calibration::DynamicCalibrationSample> samples;


        for(Jaco2Calibration::DynamicCalibrationSample sample : samples_org) {

            samples.push_back(sample);
        }


        std::size_t num_samples = samples.size();
        if(num_samples == 0){
            std::cerr << "No data loaded. Does the provided file exist?."  << std::endl;
            return 42;
        }
        std::size_t num_points = num_links * num_samples;
        std::size_t num_rows = num_points + num_cols;


        Eigen::MatrixXd full_matrix = Eigen::MatrixXd::Zero(num_rows, num_cols);
        Eigen::MatrixXd tau = Eigen::MatrixXd::Zero(num_rows, 1);

        std::cout << "samples size " << num_samples<< std::endl;


        //        int us_size = num_links*samples.size()+num_cols;

        Eigen::MatrixXd uncertainty_scale =  Eigen::MatrixXd::Identity(num_links, num_links);
        uncertainty_scale(1,1) = 1.0;
        uncertainty_scale(2,2) = 1.0;
        Eigen::MatrixXd inv_unsc = uncertainty_scale.inverse();

        Eigen::MatrixXd scale_init_param = lambda* Eigen::MatrixXd::Identity(num_cols, num_cols);


        int sample_counter = 0;

        for(std::size_t n = 0; n <samples.size(); ++ n) {

            Jaco2Calibration::DynamicCalibrationSample sample = samples[n];
            if(sample.jointPos.size() == 6) {
                Eigen::Matrix<double, num_links, num_cols> sample_mat  =
                        model.getRigidBodyRegressionMatrix("jaco_link_1", "jaco_link_hand",
                                                           sample.jointPos,
                                                           sample.jointVel,
                                                           sample.jointAcc,
                                                           sample.gravity(1),
                                                           sample.gravity(0),
                                                           sample.gravity(2));

                Eigen::Matrix<double, num_links, 1> sample_tau;
                for(std::size_t i = 0; i < num_links; ++i) {
                    sample_tau(i) = sample.jointTorque[i];
                }

                //                full_matrix.block<num_links, num_cols>(sample_counter * num_links,0) = uncertainty_scale * sample_mat;
                full_matrix.block<num_links, num_cols>(sample_counter * num_links,0) =  uncertainty_scale * sample_mat;
                tau.block<num_links,1>(sample_counter * num_links,0) = uncertainty_scale * (sample_tau - sample_mat * initial_param);

                //                    full_matrix.block<num_links, num_cols>(sample_counter * num_links,0) = sample_mat;
                //                    tau.block<num_links,1>(sample_counter * num_links,0) = sample_tau - sample_mat * initial_param;
                ++sample_counter;
            }
            else {
                std::cerr << "flawed sample "/* << "part: "<< part*/
                          << ", sample counter: " << sample_counter << std::endl;
            }
        }

        full_matrix.block<num_cols,num_cols>(num_samples, 0) = scale_init_param;


        Eigen::JacobiSVD<Eigen::MatrixXd> svd(full_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV | Eigen::FullPivHouseholderQRPreconditioner);
        //        std::cout << Eigen::FullPivHouseholderQRPreconditioner << std::endl;
        //            svd.setThreshold()
        Eigen::Matrix<double, num_cols, 1> param = svd.solve(tau);
        param += initial_param;

        Eigen::MatrixXd sol = full_matrix * param;
        Eigen::MatrixXd sample_tau = tau.block(0, 0, num_points, 1);
        Eigen::MatrixXd tau_param = sol.block(0, 0,  num_points, 1);
        Eigen::MatrixXd diff = (tau_param.array().abs() - sample_tau.array().abs()).abs().matrix();
        double mean_diff = diff.array().mean();


        Eigen::MatrixXd inital_sol = full_matrix *initial_param;
        Eigen::MatrixXd inital_tau = inital_sol.block(0, 0, num_points, 1);
        Eigen::MatrixXd intitial_diff =  (sample_tau.array().abs() - inital_tau.array().abs()).abs().matrix();

        std::cout <<"Mean difference between calib. model and sensor: " << mean_diff << std::endl;
        std::cout <<"Mean difference between initial model and sensor: " << intitial_diff.array().mean() << std::endl;

        std::cout << "mean diff per joint calib model: \n " << jointMeanfromList(diff, num_samples, num_links) << std::endl;
        std::cout << "mean diff per joint initial model: \n " << jointMeanfromList(intitial_diff, num_samples, num_links) << std::endl;

        Eigen::MatrixXd chi_sq = (tau - full_matrix* param).transpose() * (tau - full_matrix* param);
        std::cout << "chi square: " << chi_sq << std::endl;
        std::cout << "standard deviation: " << std::sqrt(chi_sq(0))/(samples.size() - num_params) << std::endl;
        std::cout << "mean difference between initial parameters: " << (param - initial_param).array().abs().mean() << std::endl;


        if(mean_diff < 3.5)
        {
            Jaco2Calibration::Jaco2ManipulatorDynParams params;
            Jaco2Calibration::to_Jaco2ManipulatorDynParams(param, model.getLinkNames(),params);
            Jaco2Calibration::Jaco2CalibrationIO::save(output,params);
            Jaco2Calibration::Jaco2CalibrationIO::save("/tmp/cad_params.txt",init_param);
        }

        return 0;
    }
    else {
        return 42;
    }
}
