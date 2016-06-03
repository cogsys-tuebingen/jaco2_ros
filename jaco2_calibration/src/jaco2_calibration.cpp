#include <jaco2_calibration/jaco2_calibration.h>
#include <jaco2_calibration/com_inertia_residuals.hpp>
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
    for(std::vector<std::string>::reverse_iterator link = links.rbegin(); link != links.rend(); ++link ){
        std::cout << *link << std::endl;
        tf::Vector3 com = model_.getLinkCoM(*link);
        tf::Matrix3x3 inertia = model_.getLinkInertiaCoM(*link);

        std::vector<double> dyn_calib_params(9);
        dyn_calib_params[0] = com.getX();
        dyn_calib_params[1] = com.getY();
        dyn_calib_params[2] = com.getZ();
        dyn_calib_params[3] = inertia.getRow(0).getX();
        dyn_calib_params[4] = inertia.getRow(0).getY();
        dyn_calib_params[5] = inertia.getRow(0).getZ();
        dyn_calib_params[6] = inertia.getRow(1).getY();
        dyn_calib_params[7] = inertia.getRow(1).getZ();
        dyn_calib_params[8] = inertia.getRow(2).getZ();

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

    for(std::size_t i = 0; i  < samples.nJoints; ++i)
    {
        std::cout << "jaco_accelerometer_" << i+1 << std::endl;
        convert(i, samples, acc_data);
        accCalib.calibrateAccJumpDistDetect(acc_data);
        imu_tk::CalibratedTriad calibParm = accCalib.getAccCalib();
        std::string name = "jaco_accelerometer_" + std::to_string(i+1);
        AccerlerometerCalibrationParam param(name, calibParm.getBiasVector(),
                                             calibParm.getMisalignmentMatrix(),
                                             calibParm.getScaleMatrix());
        accParams_[i] = param;
    }

}

void Jaco2Calibration::convert(const std::size_t &idx, const AccelerationSamples &samples, std::vector<imu_tk::TriadData> &data)
{
    std::size_t nElem = samples.samples[idx].size();
    data.resize(nElem);
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
