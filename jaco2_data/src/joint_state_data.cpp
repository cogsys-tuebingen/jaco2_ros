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

//    std::size_t end = position.end() - offset;
//    auto it = position.begin();

    for(auto it = position.begin(); it < position.end() - offset; ++it){
        while(*it > 2*M_PI){
            *it -= 2.0*M_PI;
        }
        while(*it < -2*M_PI){
            *it += 2.0 *M_PI;
        }

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

JointStateData JointStateData::operator+(const JointStateData &other) const
{
    JointStateData res(this->position.size());
    auto it_res = res.position.begin();
    auto it_other = other.position.begin();

    res.gravity = this->gravity + other.gravity;
    res.stamp.fromNSec(0.5* (this->stamp.toNSec() + other.stamp.toNSec()));
    for(auto d : this->position){
        *it_res = d + *it_other;
        ++it_res;
        ++it_other;
    }

    it_res = res.velocity.begin();
    it_other = other.velocity.begin();

    for(auto d : this->velocity){
        *it_res = d + *it_other;
        ++it_res;
        ++it_other;
    }

    it_res = res.acceleration.begin();
    it_other = other.acceleration.begin();

    for(auto d : this->acceleration){
        *it_res = d + *it_other;
        ++it_res;
        ++it_other;
    }

    it_res = res.torque.begin();
    it_other = other.torque.begin();

    for(auto d : this->torque){
        *it_res = d + *it_other;
        ++it_res;
        ++it_other;
    }
    return res;

}
JointStateData& JointStateData::operator+=(const JointStateData &other)
{
    auto it_other = other.position.begin();

    this->gravity += other.gravity;
    this->stamp.fromNSec(0.5* (this->stamp.toNSec() + other.stamp.toNSec()));
    for(auto it_p = this->position.begin();  it_p < this->position.end(); ++it_p){
        *it_p += *it_other;
        ++it_other;
    }

    it_other = other.velocity.begin();

    for(auto it_v = this->velocity.begin();  it_v < this->velocity.end(); ++it_v){
        *it_v += *it_other;
        ++it_other;
    }

    it_other = other.acceleration.begin();

    for(auto it_v = this->acceleration.begin();  it_v < this->acceleration.end(); ++it_v){
        *it_v += *it_other;
        ++it_other;
    }

    it_other = other.torque.begin();

    for(auto it_v = this->torque.begin();  it_v < this->torque.end(); ++it_v){
        *it_v += *it_other;
        ++it_other;
    }
    return *this;
}

JointStateData& JointStateData::operator*=(const double &b)
{
    this->gravity *= b;

    for(double& d  : this->position){
        d *= b;
    }

    for(double& d : this->velocity){
        d *= b;
    }

    for(double& d : this->acceleration){
        d *= b;
    }


    for(double& d : this->torque){
        d *= b;
    }
    return *this;
}

JointStateData& JointStateData::operator/=(const double &b)
{
    this->gravity *= b;

    for(double& d  : this->position){
        d /= b;
    }

    for(double& d : this->velocity){
        d /= b;
    }

    for(double& d : this->acceleration){
        d /= b;
    }


    for(double& d : this->torque){
        d /= b;
    }
    return *this;
}
