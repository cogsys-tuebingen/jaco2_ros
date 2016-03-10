#include <jaco2_driver/jaco2_api.h>
#include <ros/console.h>

Jaco2API::Jaco2API(void)
{
    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

    InitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
    CloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
    MoveHome = (int (*)()) dlsym(commandLayer_handle,"MoveHome");
    InitFingers = (int (*)()) dlsym(commandLayer_handle,"InitFingers");
    GetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
    SetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
    SendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
    GetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");
    GetAngularVelocity = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularVelocity");
}

Jaco2API::~Jaco2API()
{
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

        std::cout << "Initialization's result :" << result << std::endl;

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
        }
    }
    return result;
}

AngularPosition Jaco2API::getAngularVelocity() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    AngularPosition velocity;
    GetAngularVelocity(velocity);
    return velocity;
}

void Jaco2API::setAngularVelocity(const TrajectoryPoint &target_velocity)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    SendBasicTrajectory(target_velocity);
}
