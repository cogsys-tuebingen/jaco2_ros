#ifndef JACO2_DRIVER_CONSTANTS_H
#define JACO2_DRIVER_CONSTANTS_H
#include <string>
#include <vector>
namespace Jaco2DriverConstants {

const double fingerAngleConversion(9.2028e-3);

const std::size_t n_Jaco2Joints(6);
const std::size_t n_Jaco2JointsAndKG3(9);
const std::size_t n_Jaco2JointsAndKG2(8);

const std::string name_accel_1("jaco_accelerometer_0");
const std::string name_accel_2("jaco_accelerometer_1");
const std::string name_accel_3("jaco_accelerometer_2");
const std::string name_accel_4("jaco_accelerometer_3");
const std::string name_accel_5("jaco_accelerometer_4");
const std::string name_accel_6("jaco_accelerometer_5");

const std::vector<std::string> accel_names = { name_accel_1, name_accel_2, name_accel_3,
                                       name_accel_4, name_accel_5, name_accel_6};
}
#endif // JACO2_DRIVER_CONSTANTS_H

