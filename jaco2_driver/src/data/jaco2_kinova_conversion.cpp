#include <jaco2_driver/data/jaco2_kinova_conversion.h>
#include <jaco2_driver/jaco2_driver_constants.h>
#include <iostream>

using namespace jaco2_data;

std::string ex1 = "Vector has not enough values, " + std::to_string(Jaco2DriverConstants::n_Jaco2Joints) + " values expected";
std::string ex2 = "Vector has not enough values, " + std::to_string(Jaco2DriverConstants::n_Jaco2JointsAndKG3) + " values expected";

AccelerometerData ConvertAccelerometers::kinova2data(const AngularAcceleration& accs, const jaco2_data::TimeStamp& t)
{
    AccelerometerData res;
    res.resize(6);
    Vector3Stamped a1;
    a1.header.frame_id = Jaco2DriverConstants::name_accel_1;
    a1.header.stamp = t;
    a1.data = Vector3(accs.Actuator1_X, accs.Actuator1_Y, accs.Actuator1_Z);
    res[0] = a1;
    Vector3Stamped a2;
    a2.header.frame_id = Jaco2DriverConstants::name_accel_2;
    a2.header.stamp = t;
    a2.data = Vector3(accs.Actuator2_X, accs.Actuator2_Y, accs.Actuator2_Z);
    res[1] = a2;
    Vector3Stamped a3;
    a3.header.frame_id = Jaco2DriverConstants::name_accel_3;
    a3.header.stamp = t;
    a3.data = Vector3(accs.Actuator3_X, accs.Actuator3_Y, accs.Actuator3_Z);
    res[2] = a3;
    Vector3Stamped a4;
    a4.header.frame_id = Jaco2DriverConstants::name_accel_4;
    a4.header.stamp = t;
    a4.data = Vector3(accs.Actuator4_X, accs.Actuator4_Y, accs.Actuator4_Z);
    res[3] = a4;
    Vector3Stamped a5;
    a5.header.frame_id = Jaco2DriverConstants::name_accel_5;
    a5.header.stamp = t;
    a5.data = Vector3(accs.Actuator5_X, accs.Actuator5_Y, accs.Actuator5_Z);
    res[4] = a5;
    Vector3Stamped a6;
    a6.header.frame_id = Jaco2DriverConstants::name_accel_6;
    a6.header.stamp = t;
    a6.data = Vector3(accs.Actuator6_X, accs.Actuator6_Y, accs.Actuator6_Z);
    res[5] = a6;
    return res;
}

AngularAcceleration ConvertAccelerometers::data2kinova(const jaco2_data::AccelerometerData& data)
{
    AngularAcceleration res;
    if(data.size() < Jaco2DriverConstants::n_Jaco2Joints){

        throw std::logic_error(ex1);
    }

    res.Actuator1_X = data[0].data(0);
    res.Actuator1_Y = data[0].data(1);
    res.Actuator1_Z = data[0].data(2);

    res.Actuator2_X = data[1].data(0);
    res.Actuator2_Y = data[1].data(1);
    res.Actuator2_Z = data[1].data(2);

    res.Actuator3_X = data[2].data(0);
    res.Actuator3_Y = data[2].data(1);
    res.Actuator3_Z = data[2].data(2);

    res.Actuator4_X = data[3].data(0);
    res.Actuator4_Y = data[3].data(1);
    res.Actuator4_Z = data[3].data(2);

    res.Actuator5_X = data[4].data(0);
    res.Actuator5_Y = data[4].data(1);
    res.Actuator5_Z = data[4].data(2);

    res.Actuator6_X = data[5].data(0);
    res.Actuator6_Y = data[5].data(1);
    res.Actuator6_Z = data[5].data(2);

    return res;

}



ConvertAngularData::ConvertAngularData(){}

std::vector<double> ConvertAngularData::kinova2data(const AngularPosition& data, bool convertRadian)
{
    std::vector<double> res(Jaco2DriverConstants::n_Jaco2JointsAndKG3);
    res[0] = (data.Actuators.Actuator1);
    res[1] = (data.Actuators.Actuator2);
    res[2] = (data.Actuators.Actuator3);
    res[3] = (data.Actuators.Actuator4);
    res[4] = (data.Actuators.Actuator5);
    res[5] = (data.Actuators.Actuator6);
    //conversion encoder 2 degrees experimentaly determined
    res[6] = (Jaco2DriverConstants::fingerAngleConversion*data.Fingers.Finger1);
    res[7] = (Jaco2DriverConstants::fingerAngleConversion*data.Fingers.Finger2);
    res[8] = (Jaco2DriverConstants::fingerAngleConversion*data.Fingers.Finger3);
    if(convertRadian){
        for(auto it = res.begin(); it < res.begin() + Jaco2DriverConstants::n_Jaco2JointsAndKG3; ++it ){
            toRadian(*it);
        }
    }
    return res;
}

std::vector<double> ConvertAngularData::kinova2data(const AngularInfo& data, bool convertRadian)
{
    std::vector<double> res(Jaco2DriverConstants::n_Jaco2Joints);
    res[0] = (data.Actuator1);
    res[1] = (data.Actuator2);
    res[2] = (data.Actuator3);
    res[3] = (data.Actuator4);
    res[4] = (data.Actuator5);
    res[5] = (data.Actuator6);
    if(convertRadian){
        for(double& val : res){
            toRadian(val);
        }
    }
    return res;
}

AngularPosition ConvertAngularData::data2AngularPosition(const std::vector<double>& data, bool convertDegree)
{
    AngularPosition out;
    out.InitStruct();
    std::size_t nj = data.size();
    if(nj >= Jaco2DriverConstants::n_Jaco2Joints)
    {
        out.Actuators.Actuator1 = (data[0]);
        out.Actuators.Actuator2 = (data[1]);
        out.Actuators.Actuator3 = (data[2]);
        out.Actuators.Actuator4 = (data[3]);
        out.Actuators.Actuator5 = (data[4]);
        out.Actuators.Actuator6 = (data[5]);

        if(nj == Jaco2DriverConstants::n_Jaco2JointsAndKG3)
        {
            out.Fingers.Finger1 = data[6];
            out.Fingers.Finger1 = data[7];
            out.Fingers.Finger1 = data[8];
        }
        if(nj == Jaco2DriverConstants::n_Jaco2JointsAndKG2)
        {
            out.Fingers.Finger1 = data[6];
            out.Fingers.Finger1 = data[7];
        }
    }
    else
    {
        throw std::logic_error("Illegal Vector Size");
    }
    if(convertDegree){
        toDegrees( out.Actuators.Actuator1 );
        toDegrees( out.Actuators.Actuator2 );
        toDegrees( out.Actuators.Actuator3 );
        toDegrees( out.Actuators.Actuator4 );
        toDegrees( out.Actuators.Actuator5 );
        toDegrees( out.Actuators.Actuator6 );
        out.Fingers.Finger1 = toDegrees(out.Fingers.Finger1)/Jaco2DriverConstants::fingerAngleConversion;
        out.Fingers.Finger1 = toDegrees(out.Fingers.Finger1)/Jaco2DriverConstants::fingerAngleConversion;
        out.Fingers.Finger1 = toDegrees(out.Fingers.Finger1)/Jaco2DriverConstants::fingerAngleConversion;
    }
    return out;


}

AngularInfo ConvertAngularData::data2AngularInfo(const std::vector<double>& data, bool convertDegree)
{
    AngularInfo out;
    out.InitStruct();
    if(data.size() >= Jaco2DriverConstants::n_Jaco2Joints)
    {
        out.Actuator1 = (data[0]);
        out.Actuator2 = (data[1]);
        out.Actuator3 = (data[2]);
        out.Actuator4 = (data[3]);
        out.Actuator5 = (data[4]);
        out.Actuator6 = (data[5]);
    }
    else
    {
        throw std::logic_error(ex1);
    }
    if(convertDegree){
        toDegrees( out.Actuator1 );
        toDegrees( out.Actuator2);
        toDegrees( out.Actuator3 );
        toDegrees( out.Actuator4 );
        toDegrees( out.Actuator5 );
        toDegrees( out.Actuator6 );
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

void ConvertAngularData::toDegrees(double& radian)
{
    radian *= 1.0 /  M_PI * 180.0;
}

void ConvertAngularData::toRadian(double& degrees)
{
     degrees *= 1.0 / 180.0 * M_PI;
}

