#include <jaco2_driver/torque_offset_lut.hpp>
#include <jaco2_driver/torque_offset_calibration.hpp>
#include <nlopt.hpp>
#include <ros/ros.h>

struct TorqueOffsetResidualData
{
    std::size_t active_actuator;
    Jaco2Calibration::TorqueOffsetLut lut;
    Jaco2Calibration::ActuatorTorqueOffset func;
};

double residual(const std::vector<double> &x, std::vector<double> &grad, void *data)
{

    TorqueOffsetResidualData * ob = (TorqueOffsetResidualData*) data;
    std::size_t active_act = ob->active_actuator;
    std::size_t n_params = ob->func.function.size() * Jaco2Calibration::SineFunc::n_params;

    ob->func.setParams(x);

    double fitness = 0;

    Eigen::VectorXd v_grad;
    if(!grad.empty()){
        v_grad.setZero(n_params);
    }

    for(std::size_t i = 0; i < ob->lut.steps(active_act); ++i)
    {
        double angle = ob->lut.getAngle(active_act + 1 , i);
        double func = ob->func(angle);
        double lut_val = ob->lut.at(active_act + 1, angle);
        double tmp =  func * func - lut_val * lut_val ;
        fitness += tmp * tmp;

        if(!grad.empty()){
            Eigen::VectorXd tmp2;
            ob->func.gradParm(angle, tmp2);
            v_grad += 2.0 * tmp * tmp2;
        }
    }

    if(!grad.empty()){
        for(std::size_t i = 0; i < n_params; ++ i){
            grad[i] = v_grad(i);
        }
    }

    std::cout << fitness << std::endl;

    return fitness;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_torque_offset_sine_fit");
    ros::NodeHandle nh("~");
    std::string file, out_file;
    nh.param<std::string>("input_file", file, "/home/zwiener/workspace/jaco_ws/src/jaco2_ros/jaco2_driver/config/torque_offset_lut_jaco2-2.yaml");
    nh.param<std::string>("output_path", out_file, "/tmp/torque_offset_sine.yaml");
    TorqueOffsetResidualData data;
    data.lut.load(file);

    Jaco2Calibration::TorqueOffsetCalibration result;

    std::vector<std::size_t> problem_sizes = {6, 9, 9, 6, 6, 6};
    for(std::size_t i = 0; i < 1; ++i){
        data.func = Jaco2Calibration::ActuatorTorqueOffset(problem_sizes[i]/3);
        data.active_actuator = i;
        nlopt::opt optimizer(nlopt::GD_MLSL, problem_sizes[i]);
        optimizer.set_population(10);



        std::vector<double> lb, ub;
        lb.resize(problem_sizes[i], -1e6);
        ub.resize(problem_sizes[i],  1e6);
        optimizer.set_lower_bounds(lb);
        optimizer.set_upper_bounds(ub);
        optimizer.set_xtol_rel(0.01);
        optimizer.set_min_objective(residual, &data);
//        std::vector<double> x;
         std::vector<double> x = {0.6246, 58.19, 2.934 , 0.2004, 31.12, -1.573}  ;
//        x.resize( problem_sizes[i], 1);
        std::vector<double> grad;
        grad.resize( problem_sizes[i], 0);
        double f0 = residual(x, grad , &data);
        double f = f0;

        ROS_INFO_STREAM("Start Optimization joint " << i);
        ros::Time start = ros::Time::now();

        optimizer.optimize(x, f);

        ros::Time end = ros::Time::now();
        ros::Duration dur = end - start;


        ROS_INFO_STREAM("Optimization took " << dur.toSec() << " s" );
        ROS_INFO_STREAM("Initial residual: " << f0 << " | Final residual: " << f);
        std::cout << "params:" << std::endl;
        for(auto val : x){
            std::cout << val <<", ";
        }
        std::cout << std::endl;

        result.calibration.emplace_back(data.func);


    }

    result.save(out_file);


    return 0;
}
