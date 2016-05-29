#include <jaco2_calibration/jaco2_calibration.h>
#include <jaco2_calibration/com_inertia_residuals.hpp>

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

    for(auto link : links)
    {
        tf::Vector3 com = model_.getLinkCoM(link);
        tf::Matrix3x3 inertia = model_.getLinkInertia(link);

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
        for(auto sample : samples)
        {
            ceres::CostFunction* cost_function = ComInetriaResiduals::Create(&model_, sample, link);
            problem.AddResidualBlock(cost_function, NULL /* squared loss */, dyn_calib_params.data());
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
        DynamicCalibratedParameters linkparams;
        linkparams.linkName = link;
        linkparams.mass = model_.getLinkMass(link);
        linkparams.coM = tf::Vector3(dyn_calib_params[0], dyn_calib_params[1], dyn_calib_params[2]);
        linkparams.inertia = tf::Matrix3x3(dyn_calib_params[3], dyn_calib_params[4], dyn_calib_params[5],
                                           dyn_calib_params[4], dyn_calib_params[6], dyn_calib_params[7],
                                           dyn_calib_params[5], dyn_calib_params[7], dyn_calib_params[8]);
        dynParams_.push_back(linkparams);


    }
    return 0; // due to lack of better idea: TODO error handling
}
