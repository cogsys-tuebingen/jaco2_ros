#include <jaco2_calibration/jaco2_calibration.h>
#include <jaco2_calibration/com_inertia_residuals.hpp>
#include <jaco2_calibration/manipulator_dynamic_residuals.hpp>
#include <ceres/loss_function.h>
#include <imu_tk/calibration.h>
#include <random>
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

    std::default_random_engine gen;
    std::normal_distribution<double> dist(0,0.05);

    for(std::size_t nLinks = 0; nLinks < links.size(); ++nLinks)
    {

        dynParams_[nLinks].linkName = links[nLinks];
        dynParams_[nLinks].mass = model_.getLinkMass(links[nLinks]);
        dynParams_[nLinks].coM = model_.getLinkCoM(links[nLinks]) + Eigen::Vector3d(dist(gen), dist(gen), dist(gen));
        double rv1 = dist(gen);
        double rv2 = dist(gen);
        double rv3 = dist(gen);
        double rv4 = dist(gen);
        double rv5 = dist(gen);
        double rv6 = dist(gen);
        Eigen::Matrix3d in = model_.getLinkInertiaCoM(links[nLinks]);
        //        tf::Matrix3x3 rmat(rv1+in.getRow(0).getX(),rv2+in.getRow(0).getY(),rv3+in.getRow(0).getZ(),
        //                           rv2+in.getRow(0).getY(),rv4+in.getRow(1).getX(),rv5+in.getRow(1).getZ(),
        //                           rv3+in.getRow(0).getZ(),rv5+in.getRow(1).getZ(),rv6+in.getRow(2).getZ());
        Eigen::Matrix3d rmat;
        rmat << rv1, rv2, rv3,
                rv2, rv4, rv5,
                rv4, rv5, rv6;
        dynParams_[nLinks].inertia =  in + rmat;
    }

    for(int n = 0; n < 1; ++n) {
        std::cout << "iteration: " << n + 1 << " of 3" << std::endl;
        for(std::size_t nLinks = 0; nLinks < links.size(); ++nLinks ) {

            std::string link = links[nLinks];
            std::cout << link << std::endl;
            //            tf::Vector3 com = model_.getLinkCoM(*link);
            //            tf::Matrix3x3 inertia = model_.getLinkInertiaCoM(*link);

            std::vector<double> dyn_calib_params(9);
            dyn_calib_params[0] = dynParams_[nLinks].coM(0);
            dyn_calib_params[1] = dynParams_[nLinks].coM(1);
            dyn_calib_params[2] = dynParams_[nLinks].coM(2);
            dyn_calib_params[3] = dynParams_[nLinks].inertia(0,0);
            dyn_calib_params[4] = dynParams_[nLinks].inertia(0,1);
            dyn_calib_params[5] = dynParams_[nLinks].inertia(0,2);
            dyn_calib_params[6] = dynParams_[nLinks].inertia(1,1);
            dyn_calib_params[7] = dynParams_[nLinks].inertia(1,2);
            dyn_calib_params[8] = dynParams_[nLinks].inertia(2,2);

            ceres::Problem problem;

            problem.AddParameterBlock(dyn_calib_params.data(), 9);
            //            for(std::size_t i= 0; i <9; ++i) {
            double scale = 1.5;
            for(std::size_t i = 0; i < 9; ++i) {
                if( i < 3) {
                    double val = model_.getURDFLinkCoM(link).maxCoeff();
                    problem.SetParameterLowerBound(dyn_calib_params.data(), i, -scale * val);
                    problem.SetParameterUpperBound(dyn_calib_params.data(), i,  scale * val);
                }
                else {
                    double val = model_.getURDFLinkInertiaCoM(link).maxCoeff();
                    problem.SetParameterLowerBound(dyn_calib_params.data(), i, -scale * val);
                    problem.SetParameterUpperBound(dyn_calib_params.data(), i,  scale * val);
                }
            }


            for(auto sample : samples)
            {
                ceres::CostFunction* cost_function = ComInetriaResiduals::Create(&model_, sample, link);
                problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.1), dyn_calib_params.data());

            }/*NULL */

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = true;

            //        options.max_num_iterations = 300;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            std::cout << summary.BriefReport() << std::endl;


            DynamicCalibratedParameters linkparams;
            linkparams.linkName = link;
            linkparams.mass = model_.getLinkMass(link);
            linkparams.coM = Eigen::Vector3d(dyn_calib_params[0], dyn_calib_params[1], dyn_calib_params[2]);

            linkparams.inertia << dyn_calib_params[3], dyn_calib_params[4], dyn_calib_params[5],
                                  dyn_calib_params[4], dyn_calib_params[6], dyn_calib_params[7],
                                  dyn_calib_params[5], dyn_calib_params[7], dyn_calib_params[8];

            dynParams_[nLinks] = linkparams;


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
        Eigen::Vector3d com = model_.getLinkCoM(link);
        Eigen::Matrix3d inertia = model_.getLinkInertiaCoM(link);


        params[i][0] = com(0);
        params[i][1] = com(1);
        params[i][2] = com(2);
        params[i][3] = inertia(0,0);
        params[i][4] = inertia(0,1);
        params[i][5] = inertia(0,1);
        params[i][6] = inertia(1,1);
        params[i][7] = inertia(1,2);
        params[i][8] = inertia(2,2);

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
    options.gradient_check_numeric_derivative_relative_step_size = 1e-3;
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

        linkparams.coM = Eigen::Vector3d(params[i][0], params[i][1], params[i][2]);
        linkparams.inertia  << params[i][3], params[i][4], params[i][5],
                               params[i][4], params[i][6], params[i][7],
                               params[i][5], params[i][7], params[i][8];

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
        std::string name = "jaco_accelerometer_" + std::to_string(i);
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

std::vector<DynamicCalibratedParameters> Jaco2Calibration::getDynamicUrdfParam() const
{
    std::vector<DynamicCalibratedParameters> result;
    for(auto link : model_.getLinkNames())
    {
        DynamicCalibratedParameters param;
        param.linkName = link;
        param.coM = model_.getURDFLinkCoM(link);
        param.inertia = model_.getURDFLinkInertiaCoM(link);
        param.mass = model_.getURDFLinkMass(link);

        result.push_back(param);
    }
    return result;
}
}
