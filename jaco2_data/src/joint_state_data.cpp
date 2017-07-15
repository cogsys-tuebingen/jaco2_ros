#include <jaco2_data/joint_state_data.h>

using namespace jaco2_data;

JointStateData::JointStateData() :
    user_defined_label(-1)
{
}

JointStateData::JointStateData(std::size_t n) :
    user_defined_label(-1)
{
    resize(n);
}

void JointStateData::resize(std::size_t n)
{
    names.resize(n);
    position.resize(n);
    velocity.resize(n);
    acceleration.resize(n);
    torque.resize(n);
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
    case AngularPOS:
        data = &position;
        break;
    case ANGUlarVEL:
        data = &velocity;
        break;
    case AngularACC:
        data = &acceleration;
        break;
    case TORQUE:
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
