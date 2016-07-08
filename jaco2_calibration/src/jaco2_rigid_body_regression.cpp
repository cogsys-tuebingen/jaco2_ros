#include <string>
#include <random>
#include <algorithm>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <jaco2_kin_dyn_lib/jaco2_kinematics_dynamics.h>
#include <jaco2_calibration/jaco2_calibration_io.hpp>
#include <jaco2_calibration/acceleration_samples.hpp>

int main(int argc, char *argv[])
{
    if(argc > 2) {
        std::string input = argv[1];
        std::string output = argv[2];

        const int num_links = 6;
        const int num_param = 10;

        const int num_cols = 60;


        ros::init(argc, argv, "jaco2_rigid_body_regression_node");

        std::string urdf_param("/robot_description");
        std::string base("jaco_link_base");
        std::string tip("jaco_link_hand");
        Jaco2KinematicsDynamicsModel model(urdf_param, base, tip);

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
        Jaco2Calibration::importAsciiDataWithGravity(input,samples_org);
        std::vector<Jaco2Calibration::DynamicCalibrationSample> samples;


        for(Jaco2Calibration::DynamicCalibrationSample sample : samples_org) {
            if(sample.jointPos.size() == 6) {
                double vel_sum = 0;
                for( auto vel : sample.jointVel) {
                    vel_sum += vel;
                }
                if(vel_sum > 0.01) {
                    samples.push_back(sample);
                }
            }
        }
        std::cout << samples.size() << std::endl;

//        std::default_random_engine generator;
//        std::uniform_int_distribution<int> uniform_dist(0, samples.size());
//        std::vector<int> used_indeces;

//        std::vector<std::vector<Jaco2Calibration::DynamicCalibrationSample> >  samples_dc;
//        if(samples.size() > 10000)
//        {
//            std::vector<Jaco2Calibration::DynamicCalibrationSample> tmp;
//            while(used_indeces.size() != samples.size()){
//                int id = uniform_dist(generator);
//                if(std::find(used_indeces.begin(), used_indeces.end(), id) == used_indeces.end()) {
//                    // used_indeces does not contain x
//                    used_indeces.push_back(id);
//                    tmp.push_back(samples[id]);
//                }
//                if(tmp.size() == 1000) {
//                    samples_dc.push_back(tmp);
//                    tmp.clear();
//                }
//            }
//        }
//        else {
//            samples_dc.push_back(samples);
//        }

        std::vector<Eigen::Matrix<double, num_cols, 1> > param_dc;
        Eigen::Matrix<double, num_cols, 1> mean_param = Eigen::Matrix<double, num_cols, 1>::Zero();

        Eigen::MatrixXd full_matrix;
        Eigen::MatrixXd tau;
//        double lambda = 0.2;
//        Eigen::Matrix<double, num_cols, num_cols> constraint_scale = lambda * Eigen::Matrix<double, num_cols, num_cols>::Identity();


//        int iterations = samples_dc.size();// =3;

//        for(std::size_t part = 0; part < iterations; ++part) {

//            std::cout << samples_dc[part].size() << std::endl;

//            full_matrix = Eigen::MatrixXd::Zero(num_links*samples_dc[part].size()+num_cols,num_cols);
//            tau = Eigen::MatrixXd::Zero(num_links*samples_dc[part].size()+num_cols,1);
            full_matrix = Eigen::MatrixXd::Zero(num_links*samples.size(),num_cols);
            tau = Eigen::MatrixXd::Zero(num_links*samples.size(),1);
//            Eigen::MatrixXd scale = Eigen::MatrixXd::Identity(num_links * samples_dc[part].size(),num_links * samples_dc[part].size());

            int sample_counter = 0;
//            for(Jaco2Calibration::DynamicCalibrationSample sample : samples_dc[part]) {
            for(std::size_t n = 0; n <samples.size(); ++ n) {

                Jaco2Calibration::DynamicCalibrationSample sample = samples[n];
                if(sample.jointPos.size() == 6) {
                    Eigen::Matrix<double, num_links, num_cols> sample_mat  = model.getRigidBodyRegressionMatrix("jaco_link_1", "jaco_link_hand",
                                                                                                                sample.jointPos,
                                                                                                                sample.jointVel,
                                                                                                                sample.jointAcc,
                                                                                                                sample.gravity(0),
                                                                                                                sample.gravity(1),
                                                                                                                sample.gravity(2));

                    Eigen::Matrix<double, num_links, 1> sample_tau;
                    for(std::size_t i = 0; i < num_links; ++i) {
                        sample_tau(i) = sample.jointTorque[i];
                    }

//                    full_matrix.block<num_links, num_cols>(sample_counter * num_links,0) = scale * sample_mat;
//                    tau.block<num_links,1>(sample_counter * num_links,0) = sample_tau - sample_mat * initial_param;

                    full_matrix.block<num_links, num_cols>(sample_counter * num_links,0) = sample_mat;
                    tau.block<num_links,1>(sample_counter * num_links,0) = sample_tau - sample_mat * initial_param;
                    ++sample_counter;
                }
                else {
                    std::cerr << "flawed sample "/* << "part: "<< part*/
                              << ", sample counter: " << sample_counter << std::endl;
                }
            }
//            full_matrix.block<num_cols,num_cols>(num_links*samples_dc[part].size(),0) = constraint_scale;

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(full_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::Matrix<double, num_cols, 1> param = svd.solve(tau);
            param += initial_param;

            param_dc.push_back(param);

            mean_param = mean_param + param;

            Eigen::MatrixXd tau_param = full_matrix * param;

            Eigen::MatrixXd diff = tau_param - tau;

            double mean_diff = diff.array().abs().mean();

            std::cout << mean_diff << std::endl;
//        }
//        mean_param *= 1.0/((double) iterations);
//        Eigen::MatrixXd tau_param = full_matrix * mean_param;

//        Eigen::MatrixXd diff = tau_param - tau;

//        double mean_diff = diff.array().abs().mean();

        std::cout << "mean difference between measured torques: " <<mean_diff << std::endl;
        std::cout << "mean difference between initial parameters: " << (mean_param - initial_param).array().abs().mean() << std::endl;


        if(mean_diff < 2)
        {
            Jaco2Calibration::Jaco2ManipulatorDynParams params;
            Jaco2Calibration::to_Jaco2ManipulatorDynParams(mean_param, model.getLinkNames(),params);
            Jaco2Calibration::save(output,params);
        }

        return 0;
    }
    else {
        return 42;
    }
}
