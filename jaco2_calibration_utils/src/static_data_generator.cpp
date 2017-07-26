#include "static_data_generator.h"
#include "tree.hpp"

StaticDataGenerator::StaticDataGenerator():
    depth_(1),
    run_(0)
{

}


void StaticDataGenerator::generateData(std::size_t depth)
{
    Node<n_joints, steps> tree;
//    int steps = 5;

    std::vector<std::vector<int>> index = Node<n_joints, steps>::getIndeces(tree);
    std::vector<double> goal(6,0);
    for(auto id : index){
        for(std::size_t i = 0; i < 6; ++i){
            goal[i] = lower_limits_[i] + (double)id[i]*(upper_limits_[i] - lower_limits_[i])/double(steps);

        }
        //TODO everything else
    }

}



