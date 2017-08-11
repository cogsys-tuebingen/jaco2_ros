#include <jaco2_data/joint_state_data.h>

using namespace jaco2_data;

JointStateData::JointStateData() :
    label(-1)
{
}

JointStateData::JointStateData(std::size_t n) :
    label(-1)
{
    resize(n);
}

void JointStateData::resize(std::size_t n, double val)
{
    names.resize(n,"");
    position.resize(n, val);
    velocity.resize(n, val);
    acceleration.resize(n, val);
    torque.resize(n, val);
}

void JointStateData::normalize(std::size_t offset )
{

    std::size_t end = position.size() - offset;
    auto it = position.begin();

    for(std::size_t i = 0; i < end; ++i){
        while(*it > 2*M_PI){
            *it -= 2.0*M_PI;
        }
        while(*it < -2*M_PI){
            *it += 2.0 *M_PI;
        }
        ++it;
    }
}

Eigen::VectorXd JointStateData::getEigenVector(AngularDataType type, std::size_t offset) const
{
    const std::vector<double>* data;
    switch (type) {
    case AngularDataPOS:
        data = &position;
        break;
    case AngularDataVEL:
        data = &velocity;
        break;
    case AngularDataACC:
        data = &acceleration;
        break;
    case AngularDataTORQUE:
        data = &torque;
        break;
    }
    return convert2eigen(data, offset);
}

Eigen::VectorXd JointStateData::convert2eigen(const std::vector<double> *data, std::size_t offset)
{
    if(!data){
        return Eigen::VectorXd();
    }
    std::size_t size = data->size() - offset;
    Eigen::VectorXd res(size);
    auto it = data->begin();
    for(std::size_t i = 0; i < size; ++i){
        res(i) = *it;
        ++it;
    }
    return res;

}

JointStateData JointStateData::abs() const
{
    JointStateData res = *this;

    auto it = res.position.begin();
    for(auto p : res.position)
    {
        *it = fabs(p);
        ++it;
    }

    it = res.velocity.begin();
    for(auto p : res.velocity)
    {
        *it = fabs(p);
        ++it;
    }

    it = res.acceleration.begin();
    for(auto p : res.acceleration)
    {
        *it = fabs(p);
        ++it;
    }

    it = res.torque.begin();
    for(auto p : res.torque)
    {
        *it = fabs(p);
        ++it;
    }

    return res;
}


double JointStateData::norm(int type) const
{
    const std::vector<double>* data;
    switch (type) {
    case AngularDataPOS:
        data = &position;
        break;
    case AngularDataVEL:
        data = &velocity;
        break;
    case AngularDataACC:
        data = &acceleration;
        break;
    case AngularDataTORQUE:
        data = &torque;
        break;
    default:
        data = nullptr;
        break;
    }

    double norm = 0;
    if(data){
        for(double val : *data){
            norm += val*val;
        }
        norm = sqrt(norm);
    }
    return norm;
}
