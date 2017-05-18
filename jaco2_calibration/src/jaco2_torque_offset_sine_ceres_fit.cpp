
#include <ceres/ceres.h>

#include <ros/ros.h>

#include <jaco2_driver/torque_offset_calibration.hpp>
#include <jaco2_driver/torque_offset_lut.hpp>
#include <jaco2_calibration_utils/fft_analysis.hpp>

class QuadraticCostFunction
        : public ceres::CostFunction
{
public:
    QuadraticCostFunction()

    {
        set_num_residuals(1);
    }

    QuadraticCostFunction(std::size_t active_actuator,
                          Jaco2Calibration::TorqueOffsetLut& lut,
                          Jaco2Calibration::ActuatorTorqueOffset& func):
        active_actuator_(active_actuator),
        lut_(lut),
        func_(func)
    {
        set_num_residuals(1);
    }

    virtual ~QuadraticCostFunction() {}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {

        std::vector<double> params;
        std::size_t i = 0;
        const std::vector<int> p_block_sz = parameter_block_sizes();
        for(auto p : p_block_sz){
            for(std::size_t j = 0; j < p; ++j){
                params.push_back(parameters[i][j]);
            }
            ++i;
        }

        func_.setParams(params);

        residuals[0] = 0;

        Eigen::VectorXd v_grad;
        std::size_t n_params = func_.function.size() * Jaco2Calibration::SineFunc::n_params;

        if (jacobians != NULL && jacobians[0] != NULL) {
            v_grad.setZero(n_params);
        }


        for(std::size_t i = 0; i < lut_.steps(active_actuator_); ++i)
        {
            double angle = lut_.getAngle(active_actuator_ + 1 , i);
            double func = func_(angle) ;
            double lut_v = lut_.at(active_actuator_ + 1, angle);
            double tmp =  lut_v * lut_v - func * func;
            residuals[0] += tmp * tmp;

            if(jacobians != NULL){
                Eigen::VectorXd tmp2;
                func_.gradParm(angle, tmp2);
                v_grad -= 4.0 * tmp * func * tmp2;
            }

        }

        if(jacobians != NULL && jacobians[0] != NULL){
            for(std::size_t i = 0; i < n_params; ++ i){
                jacobians[0][i] = v_grad(i);
            }
        }


        return true;
    }

    /**
     * @brief setParameterBlockSizes of Problem:
     * @param i first block: Alternative 1: i = 1; Alternative 2: i = n;
     * @param j second block: Alternative 1: j = n; Alternative 2: j = 1;
     */
    void setParameterBlockSizes(int i, int j)
    {
        mutable_parameter_block_sizes()->resize(i);
        for(std::size_t n = 0; n < i; ++n){
            mutable_parameter_block_sizes()->at(n) = j;
        }
    }

    std::size_t active_actuator_;
    Jaco2Calibration::TorqueOffsetLut lut_;
    mutable Jaco2Calibration::ActuatorTorqueOffset func_;



};

struct FFTAnalyzation
{
    std::vector<double> getStartingValues(const Jaco2Calibration::TorqueOffsetLut& lut, std::size_t comp, std::size_t n_sine)
    {

        std::size_t len = lut.steps(comp);
        Eigen::VectorXd x,y;
        x.setZero(len);
        y.setZero(len);
        for(std::size_t i = 0; i < len; ++i){
            double angle = lut.getAngle(comp +1, i);
            x(i) = angle;
            y(i) = lut.at(comp +1, angle);
        }

        FFTAnalysis ana;

        return ana.getSineParams(x, y, n_sine);


    }


};


struct MyScalarCostFunctor
{
    bool operator()(const double* const x1,
                    double* residuals) const
    {
        std::vector<double> params;

        std::size_t n_params = func_.function.size() * Jaco2Calibration::SineFunc::n_params;
        for(std::size_t i = 0; i < n_params; ++i){
            params.emplace_back(x1[i]);
        }

        func_.setParams(params);

        residuals[0] = 0;

        for(std::size_t i = 0; i < lut_.steps(active_actuator_); ++i)
        {
            double angle = lut_.getAngle(active_actuator_ + 1 , i);
            double func = func_(angle) ;
            double lut_v = lut_.at(active_actuator_ + 1, angle);
            double tmp = func * func - lut_v * lut_v;
            residuals[0] += tmp * tmp;

        }


        return true;
    }

    std::size_t active_actuator_;
    Jaco2Calibration::TorqueOffsetLut lut_;
    mutable Jaco2Calibration::ActuatorTorqueOffset func_;


};


int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "jaco2_torque_offset_sine_fit");
    ros::NodeHandle nh("~");
    std::string file, out_file;
    nh.param<std::string>("input_file", file, "/home/zwiener/workspace/jaco_ws/src/jaco2_ros/jaco2_driver/config/torque_offset_lut_jaco2-2.yaml");
    nh.param<std::string>("output_path", out_file, "/tmp/torque_offset_sine.yaml");

    QuadraticCostFunction f;
    f.lut_.load(file);

    MyScalarCostFunctor fu;
    fu.lut_ = f.lut_;


    Jaco2Calibration::TorqueOffsetCalibration result;

    FFTAnalyzation fft;
    auto start = fft.getStartingValues(f.lut_, 0, 2);
    std::cout << "start vals: ";
    for(auto s : start){
        std::cout << s << ", ";
    }
    std::cout <<";" << std::endl;


    std::vector<std::size_t> problem_sizes = {6, 9, 9, 6, 6, 6};
    for(std::size_t i = 0; i < 1; ++i){
        double min_cost = 2e10;
        Jaco2Calibration::ActuatorTorqueOffset min_func;
//        for(std::size_t k = 0; k < 2; ++k){
            f.func_ = Jaco2Calibration::ActuatorTorqueOffset(problem_sizes[i]/3);
            f.active_actuator_ = i;

            fu.func_ = Jaco2Calibration::ActuatorTorqueOffset(problem_sizes[i]/3);
            fu.active_actuator_ = i;

            f.setParameterBlockSizes(1, problem_sizes[i]);
            std::vector<double> x = start;
//            for(std::size_t para = 0; para < problem_sizes[i]; ++para){
//                if(para % 3 == 0){
//                    x[para] *= 1.1;
//                }
//            }


//            double sumsine = fu.func_(0.5);

            //        std::vector<double> x = {0.6246, 0.1396, 2.934 , 0.2004, 0.0741, -1.573}  ;
//            x = {0.6246, 0.1396, 3.108, 0.2004, 0.07466, -1.48 };
            fu.func_.setParams(x);
            //        x.resize( problem_sizes[i], 1);

            // Set up the only cost function (also known as residual).
//            ceres::CostFunction* cost_function = &f;

            ceres::CostFunction* cost_function = new ceres::NumericDiffCostFunction<MyScalarCostFunctor, ceres::CENTRAL, 1, 6>(&fu);

            //        auto pb = cost_function->parameter_block_sizes();
            //        std::cout << pb.size() << std::endl;
            //        for(auto p : pb){
            //            std::cout << p << std::endl;
            //        }

            ceres::Problem problem;
            ceres::CauchyLoss loss(0.2);
            problem.AddResidualBlock(cost_function, NULL/*&loss*/, x.data());
//                    problem.SetParameterLowerBound(x.data(), 1, 0);
//                    problem.SetParameterLowerBound(x.data(), 4, 0);
            //        problem.SetParameterUpperBound(x.data(), 0, 2);

            // Run the solver!
            ceres::Solver::Options options;
            options.max_num_line_search_direction_restarts = 10;
            options.max_num_line_search_step_size_iterations = 1000;
            options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 400;
//            options.minimizer_progress_to_stdout = true;
            //        options.
            ceres::Solver::Summary summary;
            ROS_INFO_STREAM("Start Optimization joint " << i);
            ros::Time start = ros::Time::now();

            ceres::Solve(options, &problem, &summary);



            ros::Time end = ros::Time::now();
            ros::Duration dur = end - start;
//            problem.getc
//            problem.getc

            ROS_INFO_STREAM("Optimization took " << dur.toSec() << " s" );

//            if(){
//                min_func = fu.func_;
//            }

            std::cout << summary.BriefReport() << "\n";
            int iter = 0;
            for(auto val : x){
                std::cout << " a_" << iter << "=" << val << "\n";
                ++iter;
            }
//        }



        result.calibration.emplace_back(fu.func_);


    }

    result.save(out_file);


    return 0;
}
