#include <string>
#include <random>
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

        std::vector<Jaco2Calibration::DynamicCalibrationSample> samples;
        Jaco2Calibration::importAsciiDataWithGravity(input,samples);

        Eigen::MatrixXd full_matrix(num_links*samples.size(),num_cols);
        Eigen::MatrixXd tau(num_links*samples.size(),1);
        int sample_counter = 0;
        for(Jaco2Calibration::DynamicCalibrationSample sample : samples) {

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

            full_matrix.block<num_links, num_cols>(sample_counter * num_links,0) = sample_mat;
            tau.block<num_links,1>(sample_counter * num_links,0) = sample_tau;

        }

        //
        Eigen::MatrixXd full_matrix_t = full_matrix.transpose();
        Eigen::Matrix<double, num_cols, num_cols> square_mat = full_matrix_t * full_matrix;
        Eigen::Matrix<double, num_cols, num_cols> inverse_sq_mat = square_mat.inverse();
        Eigen::Matrix<double, num_cols, 1> trans_mat_tau = full_matrix_t * tau;
        Eigen::Matrix<double, num_cols, 1> param = inverse_sq_mat * trans_mat_tau;

        Jaco2Calibration::Jaco2ManipulatorDynParams params;
        Jaco2Calibration::to_Jaco2ManipulatorDynParams(param,model.getLinkNames(),params);
        Jaco2Calibration::save(output,params);

        return 0;
    }
    else {
        return 42;
    }
}
