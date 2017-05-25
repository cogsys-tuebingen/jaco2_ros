
#include <ros/ros.h>
#include <ceres/ceres.h>

#include <jaco2_driver/torque_offset_lut.hpp>
#include <jaco2_driver/torque_offset_calibration.hpp>
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

    QuadraticCostFunction(std::size_t active_actuator,
                          Jaco2Calibration::TorqueOffsetLut& lut,
                          Jaco2Calibration::ActuatorTorqueOffset& func,
                          int i,
                          int j):
        active_actuator_(active_actuator),
        lut_(lut),
        func_(func)
    {
        set_num_residuals(1);
        setParameterBlockSizes(i,j);
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

struct MeanOfTorqueLuts{

    MeanOfTorqueLuts():
        calculated(false)
    {}

    void loadLuts(std::vector<std::string> files)
    {
        luts.resize(files.size());
        auto it_lut = luts.begin();
        for(auto p : files){
            it_lut->load(p);
            ++it_lut;
        }
    }

    Jaco2Calibration::TorqueOffsetLut getMean()
    {
        if(!calculated){

            mean = luts.front();

            for(auto l = luts.begin() +1; l < luts.end(); ++l){
                mean.lut += l->lut;
            }

            mean.lut /= luts.size();
            calculated = true;
        }
        return mean;
    }

    bool calculated;
    std::vector<Jaco2Calibration::TorqueOffsetLut> luts;
    Jaco2Calibration::TorqueOffsetLut mean;

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

        return ana.getSineParams2(x, y, n_sine);


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

    std::string file        = nh.param<std::string>("input_file", "/home/zwiener/workspace/jaco_ws/src/jaco2_ros/jaco2_driver/config/torque_offset_lut_jaco2-2.yaml");
    std::string out_file    = nh.param<std::string>("output_path", "/tmp/torque_offset_sine.yaml");
    std::string input_files = nh.param<std::string>("input_files", "/home/zwiener/workspace/jaco_ws/src/data/jaco2-2_torque_static_data.yaml");

    Jaco2Calibration::TorqueOffsetLut data;
    std::vector<std::string> lut_paths;

    if(input_files == ""){
        data.load(file);

    }
    else{
        YAML::Node doc = YAML::LoadFile(input_files);
        auto data_f = doc["files"];
        if(!data_f.IsDefined() || !data_f.IsSequence()){
            return 42;
        }

        lut_paths = data_f.as<std::vector<std::string>>();
        MeanOfTorqueLuts m;
        m.loadLuts(lut_paths);
        data = m.getMean();
        data.save("/tmp/mean_lut.yaml");

    }

    Jaco2Calibration::TorqueOffsetCalibration result;

    FFTAnalyzation fft;


    std::vector<std::size_t> problem_sizes = {3*1, 3*5, 3*1, 3*2, 3*2, 3*2};

    for(std::size_t i = 0; i < 6; ++i){
        double min_cost = 2e10;
        ceres::Problem problem;

        Jaco2Calibration::ActuatorTorqueOffset func(problem_sizes[i]/3);
        func.id = i;

        std::vector<double> x = fft.getStartingValues(data, i, problem_sizes[i]/3 );
        func.setParams(x);

        ceres::CostFunction* cost_function = new QuadraticCostFunction(i, data, func, 1, problem_sizes[i]);


        problem.AddResidualBlock(cost_function, NULL, x.data());


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

        ROS_INFO_STREAM("Start Optimization joint " << i + 1 );
        ros::Time start = ros::Time::now();

        ceres::Solve(options, &problem, &summary);

        ros::Time end = ros::Time::now();
        ros::Duration dur = end - start;

        ROS_INFO_STREAM("Optimization took " << dur.toSec() << " s" );



        std::cout << summary.BriefReport() << "\n";
        int iter = 0;
        for(auto val : x){
            std::cout << " a_" << iter << "=" << val << "\n";
            ++iter;
        }




        result.calibration.emplace_back(func);


    }

    result.save(out_file);


    return 0;
}
