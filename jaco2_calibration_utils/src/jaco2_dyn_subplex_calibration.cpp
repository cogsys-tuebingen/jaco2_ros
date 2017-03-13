#include <jaco2_calibration_utils/dynamic_residual.h>
//#include <
#include <nlopt.hpp>

struct SubplexData{
    Eigen::MatrixXd reg_mat;
    Eigen::MatrixXd torques;
};

double minfunc(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
    Eigen::MatrixXd params(x.size(),1);
    std::size_t id = 0;
    for(auto val : x){
        params(id) = val;
        ++id;
    }
    SubplexData * opt_data = (SubplexData*) data;
    Eigen::MatrixXd opti_vec = opt_data->reg_mat * params;
    Eigen::ArrayXd diff = opt_data->torques.array().abs() - opti_vec.array().abs();

    double cost = diff.sum();
    return cost;
}

int main(int argc, char *argv[])
{
    if(argc > 2) {
        std::string input = argv[1];
        std::string output = argv[2];


        double lambda = 10;
        double u_scale = 1;

        ros::NodeHandle nh("~");

        std::string urdf_param("/robot_description");
        std::string base("jaco_link_base");
        std::string tip("jaco_link_hand");

        DynamicResidual model(urdf_param, base, tip);

        model.loadData(input);
        bool suc = model.calculteMatrix();
        if(suc){
            std::size_t problem_size = model.getProblemSize();
            nlopt::opt optimizer(nlopt::LN_SBPLX, problem_size);
            SubplexData sdata;
            sdata.reg_mat = model.getRegressionMatrix();
            sdata.torques = model.getTorques();

            std::vector<double> lower_bound(problem_size);
            std::vector<double> upper_bound(problem_size);
//            std::fill(lower_bound.begin(),lower_bound.end(),0);

            optimizer.set_lower_bounds(lower_bound);
            optimizer.set_upper_bounds(upper_bound);


            optimizer.set_min_objective(minfunc, &sdata);

        }


        return 0;
    }
    else{
        return 42;
    }

}
