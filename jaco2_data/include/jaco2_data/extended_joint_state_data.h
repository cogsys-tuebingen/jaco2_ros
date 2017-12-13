#ifndef EXTENDED_JOINT_STATE_DATA_H
#define EXTENDED_JOINT_STATE_DATA_H
#include <jaco2_data/joint_state_data.h>
#include <jaco2_data/accelerometer_data.h>
#include <memory>
namespace jaco2_data {

class ExtendedJointStateData
{
public:

    enum class DataType {
        JOINT_POS = 1,
        JOINT_VEL = 2,
        JOINT_ACC = 3,
        JOINT_TORQUE = 4,
        LIN_ACC = 5
    };

    ExtendedJointStateData() {}

    ExtendedJointStateData(std::size_t nj, std::size_t na);

    std::shared_ptr<std::vector<double>> dataAccess(int type) const;

    ExtendedJointStateData abs() const;

    std::string to_string(const std::string delimiter) const;

    void setLabel(int i);

    std::vector<double>& position();
    const std::vector<double> &position() const;

    std::vector<double>& velocity();
    const std::vector<double>& velocity() const;

    std::vector<double>& acceleration();
    const std::vector<double> &acceleration() const;

    std::vector<double>& torque();
    const std::vector<double>& torque() const;

    ExtendedJointStateData operator+(const ExtendedJointStateData &other) const;
    ExtendedJointStateData operator-(const ExtendedJointStateData &other) const;
    ExtendedJointStateData operator*(const double &other) const;
    ExtendedJointStateData operator/(const double &other) const;
    ExtendedJointStateData& operator+=(const ExtendedJointStateData &other);
    ExtendedJointStateData& operator*=(const double &b);
    ExtendedJointStateData& operator/=(const double &b);


public:
    JointStateData joint_state;
    AccelerometerData lin_acc;
};
}
#endif // EXTENDED_JOINT_STATE_DATA_H
