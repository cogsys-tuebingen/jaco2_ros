#include <jaco2_calibration/jaco2_calibration.h>
#include <jaco2_calibration/com_inertia_residuals.hpp>
#include <jaco2_calibration/manipulator_dynamic_residuals.hpp>
#include <ceres/loss_function.h>
#include <imu_tk/calibration.h>
namespace Jaco2Calibration {
Jaco2Calibration::Jaco2Calibration(std::string& urdf_param, std::string& root, std::string& tip)
    : model_(urdf_param, root, tip)
{
}

Jaco2Calibration::~Jaco2Calibration()
{
}



int Jaco2Calibration::calibrateCoMandInertia(const std::vector<DynamicCalibrationSample> &samples)
{
    //    dynParams_.resize(model_.getNrOfSegments());
    std::vector<std::string> links = model_.getLinkNames();

    //TODO ignore long sequences of not changing data

    //    auto link = links.rbegin();

    dynParams_.resize(links.size());
    for(int i = 0; i < 3; ++i) {
        std::size_t link_counter = 0;
        for(auto link = links.begin(); link != links.end(); ++link ) {
            std::vector<double> dyn_calib_params(9);

            std::cout << *link << std::endl;
            tf::Vector3 com = model_.getLinkCoM(*link);
            tf::Matrix3x3 inertia = model_.getLinkInertiaCoM(*link);

            if(i == 0) {

                dyn_calib_params[0] = com.getX();
                dyn_calib_params[1] = com.getY();
                dyn_calib_params[2] = com.getZ();
                dyn_calib_params[3] = inertia.getRow(0).getX();
                dyn_calib_params[4] = inertia.getRow(0).getY();
                dyn_calib_params[5] = inertia.getRow(0).getZ();
                dyn_calib_params[6] = inertia.getRow(1).getY();
                dyn_calib_params[7] = inertia.getRow(1).getZ();
                dyn_calib_params[8] = inertia.getRow(2).getZ();
            }
            else {
                dyn_calib_params[0] = dynParams_[i].coM.getX();
                dyn_calib_params[1] = dynParams_[i].coM.getY();
                dyn_calib_params[2] = dynParams_[i].coM.getZ();
                dyn_calib_params[3] = dynParams_[i].inertia.getRow(0).getX();
                dyn_calib_params[4] = dynParams_[i].inertia.getRow(0).getY();
                dyn_calib_params[5] = dynParams_[i].inertia.getRow(0).getZ();
                dyn_calib_params[6] = dynParams_[i].inertia.getRow(1).getY();
                dyn_calib_params[7] = dynParams_[i].inertia.getRow(1).getZ();
                dyn_calib_params[8] = dynParams_[i].inertia.getRow(2).getZ();
            }

            ceres::Problem problem;
            problem.AddParameterBlock(dyn_calib_params.data(), 9);
            for(auto sample : samples)
            {
                ceres::CostFunction* cost_function = ComInetriaResiduals::Create(&model_, sample, *link);
                problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), dyn_calib_params.data());

            }/*NULL */

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = true;
            //        options.max_num_iterations = 300;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            std::cout << summary.BriefReport() << std::endl;


            DynamicCalibratedParameters linkparams;
            linkparams.linkName = *link;
            linkparams.mass = model_.getLinkMass(*link);
            linkparams.coM = tf::Vector3(dyn_calib_params[0], dyn_calib_params[1], dyn_calib_params[2]);
            linkparams.inertia = tf::Matrix3x3(dyn_calib_params[3], dyn_calib_params[4], dyn_calib_params[5],
                    dyn_calib_params[4], dyn_calib_params[6], dyn_calib_params[7],
                    dyn_calib_params[5], dyn_calib_params[7], dyn_calib_params[8]);

            dynParams_[i] = (linkparams);

            ++link_counter;


        }
    }
    return 0; // due to lack of better idea: TODO error handling
}

int Jaco2Calibration::calibrateArmDynamic(const std::vector<DynamicCalibrationSample> &samples)
{
    //    dynParams_.resize(model_.getNrOfSegments());
    std::vector<std::string> links = model_.getLinkNames();

    //TODO ignore long sequences of not changing data

    //    std::vector<double> dyn_calib_params(54);
    std::vector<std::vector<double> > params(6);
    ceres::Problem problem;

    for(std::size_t i = 0; i < links.size(); ++i ){
        params[i].resize(9);
        std::string link = links[i];
        std::cout << link << std::endl;
        tf::Vector3 com = model_.getLinkCoM(link);
        tf::Matrix3x3 inertia = model_.getLinkInertiaCoM(link);


        params[i][0] = com.getX();
        params[i][1] = com.getY();
        params[i][2] = com.getZ();
        params[i][3] = inertia.getRow(0).getX();
        params[i][4] = inertia.getRow(0).getY();
        params[i][5] = inertia.getRow(0).getZ();
        params[i][6] = inertia.getRow(1).getY();
        params[i][7] = inertia.getRow(1).getZ();
        params[i][8] = inertia.getRow(2).getZ();

        problem.AddParameterBlock(params[i].data(),9);

        //        std::size_t id = i*9;
        //        dyn_calib_params[id] = com.getX();
        //        dyn_calib_params[id + 1] = com.getY();
        //        dyn_calib_params[id + 2] = com.getZ();
        //        dyn_calib_params[id + 3] = inertia.getRow(0).getX();
        //        dyn_calib_params[id + 4] = inertia.getRow(0).getY();
        //        dyn_calib_params[id + 5] = inertia.getRow(0).getZ();
        //        dyn_calib_params[id + 6] = inertia.getRow(1).getY();
        //        dyn_calib_params[id + 7] = inertia.getRow(1).getZ();
        //        dyn_calib_params[id + 8] = inertia.getRow(2).getZ();
    }

    //    problem.AddParameterBlock(dyn_calib_params.data(), 54);

    for(auto sample : samples)
    {
        ceres::CostFunction* cost_function = ManipulatorDynamicResiduals::Create(&model_, sample);
        //        problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), dyn_calib_params.data(),);
        problem.AddResidualBlock(cost_function, NULL, params[0].data(), params[1].data(), params[2].data(),
                params[3].data(), params[4].data(), params[5].data());

    }/*NULL */

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.numeric_derivative_relative_step_size = 1e-3;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;
    options.use_nonmonotonic_steps = true;
    options.initial_trust_region_radius = 0.4;
    //        options.max_num_iterations = 300;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    for(std::size_t i = 0; i < links.size(); ++i) {
        std::size_t id = i*9;
        DynamicCalibratedParameters linkparams;
        linkparams.linkName = links[i];
        linkparams.mass = model_.getLinkMass(links[i]);
        //        linkparams.coM = tf::Vector3(dyn_calib_params[id], dyn_calib_params[id + 1], dyn_calib_params[id + 2]);
        //        linkparams.inertia = tf::Matrix3x3(dyn_calib_params[id + 3], dyn_calib_params[id + 4], dyn_calib_params[id + 5],
        //                                           dyn_calib_params[id + 4], dyn_calib_params[id + 6], dyn_calib_params[id + 7],
        //                                           dyn_calib_params[id + 5], dyn_calib_params[id + 7], dyn_calib_params[id + 8]);

        linkparams.coM = tf::Vector3(params[i][0], params[i][1], params[i][2]);
        linkparams.inertia = tf::Matrix3x3(params[i][3], params[i][4], params[i][5],
                params[i][4], params[i][6], params[i][7],
                params[i][5], params[i][7], params[i][8]);

        dynParams_.push_back(linkparams);
    }

    return 0; // due to lack of better idea: TODO error handling
}



bool Jaco2Calibration::calibrateAcc(const AccelerationSamples &samples)
{
    imu_tk::MultiPosCalibration accCalib;

    std::vector< imu_tk::TriadData > acc_data;
    imu_tk::CalibratedTriad init_acc_calib;
    init_acc_calib.setBias( Eigen::Vector3d(0.01, 0.01, 0.01) );
    accCalib.setGravityMagnitude(1.0);

    accCalib.enableVerboseOutput(true);
    accCalib.enableAccUseMeans(true);
    accCalib.setIntarvalsNumSamples(100);
    accCalib.setInitStaticIntervalDuration(6.5);

    accParams_.resize(samples.nJoints);

    AccelerationSamples calibAccSamples;

    for(std::size_t i = 0; i  < samples.nJoints; ++i)
    {
        std::cout << "jaco_accelerometer_" << i+1 << std::endl;
        convert(i, samples, acc_data);
        accCalib.calibrateAccJumpDistDetect(acc_data);
        imu_tk::CalibratedTriad calibParm = accCalib.getAccCalib();
        std::string name = "jaco_accelerometer_" + std::to_string(i+1);
        AccelerometerCalibrationParam param(name, calibParm.getBiasVector(),
                                            calibParm.getMisalignmentMatrix(),
                                            calibParm.getScaleMatrix());

        accParams_[i] = param;
        std::vector<imu_tk::TriadData> calibSamples = accCalib.getCalibAccSamples();

        convert(i, calibSamples, calibAccSamples);
    }
    calibAccSamples.save("/tmp/calib_acc_samples.txt");
    return true;
}

void Jaco2Calibration::convert(const std::size_t &idx, const AccelerationSamples &samples, std::vector<imu_tk::TriadData> &data)
{
    std::size_t nElem = samples.samples[idx].size();
    data.resize(nElem);
    if(nElem > 0) {
        double t0 = samples.samples[0].at(0).time;
        for(std::size_t i = 0; i < nElem; ++i){
            double t = samples.samples[idx][i].time -t0;
            double x = samples.samples[idx][i].vector[0];
            double y = samples.samples[idx][i].vector[1];
            double z = samples.samples[idx][i].vector[2];

            imu_tk::TriadData tkAcc(t, x, y, z);
            data[i] = tkAcc;
        }
    }
}

void Jaco2Calibration::convert(const std::size_t &idx, const std::vector<imu_tk::TriadData> &data, AccelerationSamples &samples)
{
    std::size_t nElem = data.size();
    //    samples[idx].resize(nElem);

    for(std::size_t i = 0; i < nElem; ++i){

        double t = data[i].timestamp();
        double x = data[i].x();
        double y = data[i].y();
        double z = data[i].z();
        AccelerationData samp(t, x, y,z);
        samples.push_back(idx,samp);
    }
}
}
