#ifndef JACO2_KINOVA_CONVERSION_H
#define JACO2_KINOVA_CONVERSION_H
/// Kinova
#include <kinova/KinovaTypes.h>
/// Jaco2
#include <jaco2_data/accelerometer_data.h>
#include <jaco2_data/joint_state_data.h>
#include <jaco2_data/time_stamp.h>
#include <jaco2_data/vector3.h>

struct ConvertAccelerometers
{
   static jaco2_data::AccelerometerData kinova2data(const AngularAcceleration& accs, const jaco2_data::TimeStamp& t);
   static AngularAcceleration data2kinova(const jaco2_data::AccelerometerData& data);

};

class ConvertAngularData
{
public:
    ConvertAngularData();

    static std::vector<double> kinova2data(const AngularPosition& data, bool convertRadian = true);
    static std::vector<double> kinova2data(const AngularInfo& data, bool convertRadian = true);

    static AngularPosition data2AngularPosition(const std::vector<double>& data, bool convertDegree = true);
    static AngularInfo data2AngularInfo(const std::vector<double>& data, bool convertDegree = true);

private:
    static double toDegrees(const double& radian);
    static double toRadian(const double& degrees);
    static void toDegrees(double& radian);
    static void toRadian(double& degrees);
};

#endif // JACO2_KINOVA_CONVERSION_H
