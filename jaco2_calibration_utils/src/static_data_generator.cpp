#include "static_data_generator.h"

StaticDataGenerator::StaticDataGenerator():
    depth_(1),
    run_(0)
{

}


void StaticDataGenerator::generateData(std::size_t depth)
{
    depth_ = depth;
    steps_ = 1;
    while(run_ <= depth_)
    {
        for(int i = 0; i <= steps; ++i)
        {

            for(int j = 0; j <= n_joints_; ++j)
            {
                angles[j] = lower_limits_[j] + (double)i*(upper_limits_[j] - lower_limits_[j])/double(steps);
                for(int j = 0; j <= steps; ++j)
                {
                    angles[2] = lowerLimits_[1] + (double)j*(upperLimits_[1] - lowerLimits_[1])/double(steps);
                    for(int k = 0; k <= steps; ++k)
                    {

                    }
                }

            }
        }
        steps_ *= 2;
    }
}


void StaticDataGenerator::genData(std::size_t nj)
{
    if(nj = n_joints_ -1){
        for(int n = 0; n < steps_; ++ n ){
            angles[nj] = lowerLimits_[1] + (double)j*(upperLimits_[1] - lowerLimits_[1])/double(steps);
        }
    }
}
