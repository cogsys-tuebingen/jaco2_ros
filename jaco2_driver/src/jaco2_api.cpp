#include <jaco2_driver/jaco2_api.h>
#include <ros/console.h>

Jaco2API::Jaco2API():
    stopedAPI_(true)
{
    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

    InitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
    CloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
    StopControlAPI = (int (*)()) dlsym(commandLayer_handle,"StopControlAPI");
    StartControlAPI = (int (*)()) dlsym(commandLayer_handle,"StartControlAPI");
    MoveHome = (int (*)()) dlsym(commandLayer_handle,"MoveHome");
    InitFingers = (int (*)()) dlsym(commandLayer_handle,"InitFingers");
    GetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
    SetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
    SendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
    GetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");
    GetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularPosition");
    GetAngularVelocity = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularVelocity");
    GetAngularForce = (int (*)(AngularPosition &Response)) dlsym(commandLayer_handle,"GetAngularForce");
    GetAngularForceGravityFree = (int (*)(AngularPosition &Response)) dlsym(commandLayer_handle,"GetAngularForceGravityFree");
    GetQuickStatus = (int (*)(QuickStatus &)) dlsym(commandLayer_handle,"GetQuickStatus");
    GetAngularCurrent = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCurrent");
    EraseAllTrajectories = (int (*)()) dlsym(commandLayer_handle,"EraseAllTrajectories");
    GetActuatorAcceleration = (int (*)(AngularAcceleration &)) dlsym(commandLayer_handle,"GetActuatorAcceleration");
    GetAngularForceGravityFree = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularForceGravityFree");
    SetAngularControl = (int (*)()) dlsym(commandLayer_handle,"SetAngularControl");
    SetCartesianControl = (int (*)()) dlsym(commandLayer_handle,"SetCartesianControl");
    GetSensorsInfo = (int (*)(SensorsInfo &)) dlsym(commandLayer_handle,"GetSensorsInfo");
    SetTorqueZero = (int (*)(int)) dlsym(commandLayer_handle,"SetTorqueZero");

}

Jaco2API::~Jaco2API()
{
    EraseAllTrajectories();
    StopControlAPI();
    CloseAPI();
    dlclose(commandLayer_handle);
}


int Jaco2API::init()
{
    int result;
    if((InitAPI == NULL) || (CloseAPI == NULL) || (SendBasicTrajectory == NULL) ||
            (SendBasicTrajectory == NULL) || (MoveHome == NULL) || (InitFingers == NULL))
    {
        std::cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << std::endl;
        result = ERROR_NOT_INITIALIZED;
    }
    else
    {
        std::cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << std::endl << std::endl;

        result = (*InitAPI)();

        std::cout << "Initialization's result : " << result << std::endl;
        if(result == 1015)
        {
            return ERROR_NOT_INITIALIZED;
        }
        stopedAPI_ = false;

//        result = 1015;
//        int count = 1;
//        while(result != 1 && count < 4 && result == 1015)
//        {
//            result = (*InitAPI)();
//            std::cout << "Initialization's result : " << result << " attempt : " << count << std::endl;
//            if(result != 1){
//                sleep(10);
//            }
//            ++count;
//        }



        KinovaDevice list[MAX_KINOVA_DEVICE];

        int devicesCount = GetDevices(list, result);

        for(int i = 0; i < devicesCount; i++)
        {
            std::cout << "Found a robot on the USB bus (" << std::string(list[i].SerialNumber) << ")" << std::endl;

            //Setting the current device as the active device.
            result = SetActiveDevice(list[i]);

            std::cout << "Send the robot to HOME position" << std::endl;
            result = MoveHome();

            std::cout << "Initializing the fingers" << std::endl;
            result = InitFingers();
            std::cout << std::endl << "D R I V E R   R E A D Y" << std::endl << std::endl;
        }
    }
    return result;
}

void Jaco2API::startAPI()
{
    if(stopedAPI_)
    {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        int result = StartControlAPI();
        if(result ==1)
        {
            stopedAPI_ = false;
        }
        else
        {
            std::cerr << "Could not start API." << std::endl;
        }
    }
}

void Jaco2API::stopAPI()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    stopedAPI_ = true;
    EraseAllTrajectories();
    StopControlAPI();

}

QuickStatus Jaco2API::getQuickStatus() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    QuickStatus status;
    GetQuickStatus(status);
    return status;
}

AngularPosition Jaco2API::getAngularPosition() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    AngularPosition position;
    GetAngularPosition(position);
    return position;
}

AngularPosition Jaco2API::getAngularVelocity() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    AngularPosition velocity;
    GetAngularVelocity(velocity);
    return velocity;
}

AngularPosition Jaco2API::getAngularForce() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    AngularPosition torque;
    GetAngularForce(torque);
    return torque;
}

AngularPosition Jaco2API::getAngularForceGravityFree() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    AngularPosition torque;
    GetAngularForceGravityFree(torque);
    return torque;
}

AngularAcceleration Jaco2API::getActuatorAcceleration() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    AngularAcceleration acc;
    GetActuatorAcceleration(acc);
    return acc;
}

SensorsInfo Jaco2API::getSensorInfo() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    SensorsInfo info;
    GetSensorsInfo(info);
    return info;
}

void Jaco2API::setAngularVelocity(const TrajectoryPoint &target_velocity)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if(!stopedAPI_){
        SendBasicTrajectory(target_velocity);
    }
}


void Jaco2API::setAngularPosition(const TrajectoryPoint &position)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if(!stopedAPI_){
        SendBasicTrajectory(position);
    }
}

AngularPosition Jaco2API::getAngularCurrent() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    AngularPosition current;
    GetAngularCurrent(current);
    return current;
}

bool Jaco2API::isStopped() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return stopedAPI_;
}

void Jaco2API::moveHome()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    MoveHome();
}

void Jaco2API::initFingers()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    InitFingers();
}

int Jaco2API::setTorqueZero(int actuator)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    int result; //may return
    switch (actuator) {
    case 1:
        result = (*SetTorqueZero)(16);
        break;
    case 2:
        result = (*SetTorqueZero)(17);
        break;
    case 3:
        result = (*SetTorqueZero)(18);
        break;
    case 4:
        result = (*SetTorqueZero)(19);
        break;
    case 5:
        result = (*SetTorqueZero)(20);
        break;
    case 6:
        result = (*SetTorqueZero)(21);
        break;
    default:
        break;
    }
    usleep(100000);
    return result;
}
