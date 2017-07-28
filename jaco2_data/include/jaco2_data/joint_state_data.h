#ifndef JOINT_STATE_DATA_H
#define JOINT_STATE_DATA_H
#include <Eigen/Core>
#include <jaco2_data/time_stamp.h>
namespace jaco2_data {

enum AngularDataType{
    AngularDataPOS = 1,
    AngularDataVEL = 2,
    AngularDataACC = 3,
    AngularDataTORQUE = 4
};

class JointStateData
{
public:
    JointStateData();
    JointStateData(std::size_t n);

    void resize(std::size_t n, double val = 0);

    void normalize(std::size_t offset = 0);

    Eigen::VectorXd getEigenVector(AngularDataType type, std::size_t offset = 0) const;

    JointStateData abs() const;

    double norm(int type) const;

    JointStateData operator+(const JointStateData &other) const;
    JointStateData& operator+=(const JointStateData &other);
    JointStateData& operator*=(const double &b);
    JointStateData& operator/=(const double &b);

private:
    static Eigen::VectorXd convert2eigen(const std::vector<double> *data, std::size_t offset = 0);

public:
    int label;
    std::string frame_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d gravity;

    TimeStamp stamp;
    std::vector<std::string> names;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> torque;
};
}
#endif // JOINT_STATE_DATA_H
