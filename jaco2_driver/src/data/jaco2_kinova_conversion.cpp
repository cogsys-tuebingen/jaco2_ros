#include <jaco2_driver/data/jaco2_kinova_conversion.h>
#include <jaco2_driver/jaco2_driver_constants.h>
using namespace jaco2_data;

std::string ex1 = "Vector has not enough values, " + std::to_string(Jaco2DriverConstants::n_Jaco2Joints) + " values expected";
std::string ex2 = "Vector has not enough values, " + std::to_string(Jaco2DriverConstants::n_Jaco2JointsAndKG3) + " values expected";

AccelerometerData ConvertAccelerometers::kinova2data(const AngularAcceleration& accs, const jaco2_data::TimeStamp& t)
{
    AccelerometerData res;
    Vector3Stamped a1;
    a1.frame_id = Jaco2DriverConstants::name_accel_1;
    a1.stamp = t;
    a1.vector = Eigen::Vector3d(accs.Actuator1_X, accs.Actuator1_Y, accs.Actuator1_Z);
    res.lin_acc.emplace_back(a1);
    Vector3Stamped a2;
    a2.frame_id = Jaco2DriverConstants::name_accel_2;
    a2.stamp = t;
    a2.vector = Eigen::Vector3d(accs.Actuator2_X, accs.Actuator2_Y, accs.Actuator2_Z);
    res.lin_acc.emplace_back(a2);
    Vector3Stamped a3;
    a3.frame_id = Jaco2DriverConstants::name_accel_3;
    a3.stamp = t;
    a3.vector = Eigen::Vector3d(accs.Actuator3_X, accs.Actuator3_Y, accs.Actuator3_Z);
    res.lin_acc.emplace_back(a3);
    Vector3Stamped a4;
    a4.frame_id = Jaco2DriverConstants::name_accel_4;
    a4.stamp = t;
    a4.vector = Eigen::Vector3d(accs.Actuator4_X, accs.Actuator4_Y, accs.Actuator4_Z);
    res.lin_acc.emplace_back(a4);
    Vector3Stamped a5;
    a5.frame_id = Jaco2DriverConstants::name_accel_5;
    a5.stamp = t;
    a5.vector = Eigen::Vector3d(accs.Actuator5_X, accs.Actuator5_Y, accs.Actuator5_Z);
    res.lin_acc.emplace_back(a5);
    Vector3Stamped a6;
    a6.frame_id = Jaco2DriverConstants::name_accel_6;
    a6.stamp = t;
    a6.vector = Eigen::Vector3d(accs.Actuator6_X, accs.Actuator6_Y, accs.Actuator6_Z);
    res.lin_acc.emplace_back(a6);
    return res;

}

AngularAcceleration ConvertAccelerometers::data2kinova(const jaco2_data::AccelerometerData& data)
{
    AngularAcceleration res;
    if(data.lin_acc.size() < Jaco2DriverConstants::n_Jaco2Joints){

        throw std::logic_error(ex1);
    }

    res.Actuator1_X = data.lin_acc[0].vector(0);
    res.Actuator1_Y = data.lin_acc[0].vector(1);
    res.Actuator1_Z = data.lin_acc[0].vector(2);

    res.Actuator2_X = data.lin_acc[1].vector(0);
    res.Actuator2_Y = data.lin_acc[1].vector(1);
    res.Actuator2_Z = data.lin_acc[1].vector(2);

    res.Actuator3_X = data.lin_acc[2].vector(0);
    res.Actuator3_Y = data.lin_acc[2].vector(1);
    res.Actuator3_Z = data.lin_acc[2].vector(2);

    res.Actuator4_X = data.lin_acc[3].vector(0);
    res.Actuator4_Y = data.lin_acc[3].vector(1);
    res.Actuator4_Z = data.lin_acc[3].vector(2);

    res.Actuator5_X = data.lin_acc[4].vector(0);
    res.Actuator5_Y = data.lin_acc[4].vector(1);
    res.Actuator5_Z = data.lin_acc[4].vector(2);

    res.Actuator6_X = data.lin_acc[5].vector(0);
    res.Actuator6_Y = data.lin_acc[5].vector(1);
    res.Actuator6_Z = data.lin_acc[5].vector(2);

    return res;

}



ConvertAngularData::ConvertAngularData(){}

std::vector<double> ConvertAngularData::kinova2data(const AngularPosition& data)
{
    std::vector<double> res(Jaco2DriverConstants::n_Jaco2JointsAndKG3);
    res[0] = toRadian(data.Actuators.Actuator1);
    res[1] = toRadian(data.Actuators.Actuator2);
    res[2] = toRadian(data.Actuators.Actuator3);
    res[3] = toRadian(data.Actuators.Actuator4);
    res[4] = toRadian(data.Actuators.Actuator5);
    res[5] = toRadian(data.Actuators.Actuator6);
    //conversion encoder 2 degrees experimentaly determined
    res[6] = toRadian(Jaco2DriverConstants::fingerAngleConversion*data.Fingers.Finger1);
    res[7] = toRadian(Jaco2DriverConstants::fingerAngleConversion*data.Fingers.Finger2);
    res[8] = toRadian(Jaco2DriverConstants::fingerAngleConversion*data.Fingers.Finger3);
    return res;
}

std::vector<double> ConvertAngularData::kinova2data(const AngularInfo& data)
{
    std::vector<double> res(Jaco2DriverConstants::n_Jaco2Joints);
    res[0] = toRadian(data.Actuator1);
    res[1] = toRadian(data.Actuator2);
    res[2] = toRadian(data.Actuator3);
    res[3] = toRadian(data.Actuator4);
    res[4] = toRadian(data.Actuator5);
    res[5] = toRadian(data.Actuator6);
    return res;
}

AngularPosition ConvertAngularData::data2AngularPosition(const std::vector<double>& data)
{
    AngularPosition out;
    out.InitStruct();
    std::size_t nj = data.size();
    if(nj >= Jaco2DriverConstants::n_Jaco2Joints)
    {
        out.Actuators.Actuator1 = toDegrees(data[0]);
        out.Actuators.Actuator2 = toDegrees(data[1]);
        out.Actuators.Actuator3 = toDegrees(data[2]);
        out.Actuators.Actuator4 = toDegrees(data[3]);
        out.Actuators.Actuator5 = toDegrees(data[4]);
        out.Actuators.Actuator6 = toDegrees(data[5]);

        if(nj == Jaco2DriverConstants::n_Jaco2JointsAndKG3)
        {
            out.Fingers.Finger1 = toDegrees(data[6])/Jaco2DriverConstants::fingerAngleConversion;
            out.Fingers.Finger1 = toDegrees(data[7])/Jaco2DriverConstants::fingerAngleConversion;
            out.Fingers.Finger1 = toDegrees(data[8])/Jaco2DriverConstants::fingerAngleConversion;
        }
        if(nj == Jaco2DriverConstants::n_Jaco2JointsAndKG2)
        {
            out.Fingers.Finger1 = toDegrees(data[6])/Jaco2DriverConstants::fingerAngleConversion;
            out.Fingers.Finger1 = toDegrees(data[7])/Jaco2DriverConstants::fingerAngleConversion;
        }
    }
    else
    {
        throw std::logic_error("Illegal Vector Size");
    }
    return out;


}

AngularInfo ConvertAngularData::data2AngularInfo(const std::vector<double>& data)
{
    AngularInfo out;
    out.InitStruct();
    if(data.size() >= Jaco2DriverConstants::n_Jaco2Joints)
    {
        out.Actuator1 = toDegrees(data[0]);
        out.Actuator2 = toDegrees(data[1]);
        out.Actuator3 = toDegrees(data[2]);
        out.Actuator4 = toDegrees(data[3]);
        out.Actuator5 = toDegrees(data[4]);
        out.Actuator6 = toDegrees(data[5]);
    }
    else
    {
        throw std::logic_error(ex1);
    }
    return out;
}


double ConvertAngularData::toDegrees(const double& radian)
{
    return radian / M_PI * 180.0;
}

double ConvertAngularData::toRadian(const double& degrees)
{
    return degrees / 180.0 * M_PI;
}

