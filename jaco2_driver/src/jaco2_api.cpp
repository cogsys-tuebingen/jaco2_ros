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
    SendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendAdvanceTrajectory");
    StopCurrentLimitation = (int (*)()) dlsym(commandLayer_handle,"StopCurrentLimitation");
    GetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianPosition");
    GetAPIVersion = (int (*)(int Response[API_VERSION_COUNT])) dlsym(commandLayer_handle,"GetAPIVersion");
    // untested functions since available  5.2.0
    RunGravityZEstimationSequence = (int(*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) dlsym(commandLayer_handle, "RunGravityZEstimationSequence");
    SwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
    SetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
    SendAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendAngularTorqueCommand");
    SendCartesianForceCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendCartesianForceCommand");
    SetGravityVector = (int(*)(float Command[3])) dlsym(commandLayer_handle, "SetGravityVector");
    SetGravityPayload = (int(*)(float Command[GRAVITY_PAYLOAD_SIZE])) dlsym(commandLayer_handle, "SetGravityPayload");
    SetGravityOptimalZParam =  (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "SetGravityOptimalZParam");
    SetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");
    GetCartesianForce = (int(*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianForce");
    SetTorqueVibrationController = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueVibrationController");
    SetTorqueControlType = (int(*)(TORQUECONTROL_TYPE)) dlsym(commandLayer_handle, "SetTorqueControlType");
    GetTrajectoryTorqueMode = (int(*)(int &)) dlsym(commandLayer_handle, "GetTrajectoryTorqueMode");
    SetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");
    SetGravityOptimalZParam = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "SetGravityOptimalZParam");
    SetActuatorPID = (int (*)(unsigned int, float, float, float )) dlsym(commandLayer_handle,"SetActuatorPID");

}

Jaco2API::~Jaco2API()
{
    EraseAllTrajectories();
    StopControlAPI();
    CloseAPI();
    dlclose(commandLayer_handle);
}


int Jaco2API::init(std::string serial, bool right)
{
    right_arm_ = right;
    int result = -1;
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
        std::size_t length = serial.length();

        for(int i = 0; i < devicesCount; i++)
        {
            std::string serial_i = std::string(list[i].SerialNumber);

            std::cout << "Found a robot on the USB bus (" << serial_i << ")" << std::endl;
            if(serial_i.compare(0,length,serial) == 0 || serial == std::string("")){
                //Setting the current device as the active device.
                result = SetActiveDevice(list[i]);

                std::cout << "Send the robot to HOME position" << std::endl;
                if(right){
                    result = MoveHome();
                }
                else{
                    moveHomeLeft();
                }

                std::cout << "Initializing the fingers" << std::endl;
                result = InitFingers();
                std::cout << std::endl << "D R I V E R   R E A D Y" << std::endl << std::endl;
            }
        }

        SetGravityType(MANUAL_INPUT);

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

void Jaco2API::exitAPI()
{
    SetGravityType(MANUAL_INPUT);
    stopAPI();
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    CloseAPI();
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

void Jaco2API::setAngularTorque(const AngularPosition& torque)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    //    SetTorqueControlType(DIRECTTORQUE);
    float cmd[COMMAND_SIZE];
    memset(cmd,0.0,sizeof(float)*COMMAND_SIZE);
    cmd[0] = torque.Actuators.Actuator1;
    cmd[1] = torque.Actuators.Actuator2;
    cmd[2] = torque.Actuators.Actuator3;
    cmd[3] = torque.Actuators.Actuator4;
    cmd[4] = torque.Actuators.Actuator5;
    cmd[5] = torque.Actuators.Actuator6;

    SendAngularTorqueCommand(cmd);
}

void Jaco2API::setAngularTorque(const AngularInfo& torque)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    //    SetTorqueControlType(DIRECTTORQUE);
    float cmd[COMMAND_SIZE];
    memset(cmd,0.0,sizeof(float)*COMMAND_SIZE);
    cmd[0] = torque.Actuator1;
    cmd[1] = torque.Actuator2;
    cmd[2] = torque.Actuator3;
    cmd[3] = torque.Actuator4;
    cmd[4] = torque.Actuator5;
    cmd[5] = torque.Actuator6;

    SendAngularTorqueCommand(cmd);
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
    if(!stopedAPI_){
        if(right_arm_){
            MoveHome();
        }
    }
}

void Jaco2API::initFingers()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    SwitchTrajectoryTorque(POSITION);
    InitFingers();
}
void Jaco2API::enableDirectTorqueMode(double torque_saftey_factor, double vibration_controller)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    int res1 = SetTorqueControlType(DIRECTTORQUE);
    std::cout << "torque control type: " << res1 << std::endl;
    SetTorqueSafetyFactor(torque_saftey_factor);
    SetTorqueVibrationController(vibration_controller);
    int res2 = SwitchTrajectoryTorque(TORQUE);
    std::cout << "switching result: " << res2 << std::endl;
}

void Jaco2API::disableTorque()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    SwitchTrajectoryTorque(POSITION);
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

void Jaco2API::getApiVersion(int &v_major, int &v_minor, int &version)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    int data[API_VERSION_COUNT];
    GetAPIVersion(data);
    v_major = data[0];
    v_minor = data[1];
    version = data[2];

}

bool Jaco2API::inTrajectoryMode()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    int mode;
    GetTrajectoryTorqueMode(mode);
    bool res  = mode == 0;
    return res;
}

bool Jaco2API::setGravityOptimalZParam(const std::vector<double> &params)
{
    std::unique_lock<std::recursive_mutex>lock(mutex_);
    SwitchTrajectoryTorque(POSITION);
    SetGravityType(MANUAL_INPUT);
    usleep(30000);
    bool ok = params.size() == OPTIMAL_Z_PARAM_SIZE;
    int res = 0;
    if(ok){

        float optimalParams [OPTIMAL_Z_PARAM_SIZE];
//        std::size_t i = 0;
        for(std::size_t i = 0; i < OPTIMAL_Z_PARAM_SIZE; ++i){
            optimalParams[i] = (float) params[i];
            std::cout << i << " | " << optimalParams[i] << std::endl;
        }
        res = SetGravityOptimalZParam(optimalParams);
        usleep(30000);
        std::cout << "setting parameter result:  " << res<< std::endl;
        int res2 = SetGravityType(OPTIMAL);
        res +=res2;
        std::cout << "changing gravity type result:  " << res2<< std::endl;
        usleep(30000);
        if(res != 2){
            setGravityType(MANUAL_INPUT);
        }
    }
    return ok && (res == 2);
}

void Jaco2API::setGravityType(GRAVITY_TYPE type)
{
    std::unique_lock<std::recursive_mutex>lock(mutex_);
    SetGravityType(type);
    usleep(30000);
}

void Jaco2API::runGravityEstimationSequnce(std::vector<double> &res, ROBOT_TYPE type)
{
    SwitchTrajectoryTorque(POSITION);
    double optimalParams[OPTIMAL_Z_PARAM_SIZE];
    RunGravityZEstimationSequence(type, optimalParams);
    res.resize(OPTIMAL_Z_PARAM_SIZE);
    for(std::size_t i = 0; i < OPTIMAL_Z_PARAM_SIZE; ++i){
        res[i] = optimalParams[i];
    }
}

void Jaco2API::moveHomeLeft()
{
    AngularPosition p =getAngularPosition();

    AngularPosition ps;
    ps.InitStruct();
    ps.Actuators.Actuator1 =  89.93475342;
    ps.Actuators.Actuator2 = 209.94900513;
    ps.Actuators.Actuator3 = 333.11074257;
    ps.Actuators.Actuator4 = 92.1072998;
    ps.Actuators.Actuator5 = 354.7420435;
    ps.Actuators.Actuator6 = 260.12473297;

    bool test = std::abs(p.Actuators.Actuator1 - ps.Actuators.Actuator1 ) < 3.0;
    test &= std::abs(p.Actuators.Actuator2 - ps.Actuators.Actuator2 ) < 3.0;
    test = std::abs(p.Actuators.Actuator3 - ps.Actuators.Actuator3 ) < 3.0;
    test = std::abs(p.Actuators.Actuator4 - ps.Actuators.Actuator4 ) < 3.0;
    test = std::abs(p.Actuators.Actuator5 - ps.Actuators.Actuator5 ) < 3.0;
    test = std::abs(p.Actuators.Actuator6 - ps.Actuators.Actuator6 ) < 3.0;

    SwitchTrajectoryTorque(POSITION);

    if(test){
        TrajectoryPoint homeLeft;
        homeLeft.InitStruct();
        homeLeft.Position.HandMode = HAND_NOMOVEMENT;

        homeLeft.Limitations.speedParameter1 = 10.0f;
        homeLeft.Limitations.speedParameter2 = 10.0f;
        homeLeft.Limitations.speedParameter3 = 13.0f;
        homeLeft.LimitationsActive = 1;


        homeLeft.Position.Type = ANGULAR_POSITION;
        homeLeft.Position.Actuators.Actuator1 = 84.58526611f;
        homeLeft.Position.Actuators.Actuator2 = 192.55497742f;
        homeLeft.Position.Actuators.Actuator3 = 302.53701019f;
        homeLeft.Position.Actuators.Actuator4 = 119.17248535f;
        homeLeft.Position.Actuators.Actuator5 = 277.21775818f;
        homeLeft.Position.Actuators.Actuator6 = 284.33439636f;

        //        SendBasicTrajectory(homeLeft);
        SendAdvanceTrajectory(homeLeft);
        StopCurrentLimitation();
    }
    else{
        MoveHome();
    }

}


void Jaco2API::setActuatorPID(ActuatorID actuator, double p, double i, double d)
{
    std::unique_lock<std::recursive_mutex>lock(mutex_);
    if(actuator > 0 && actuator < 7){
        unsigned int address = 0;
        address = 16 + actuator -1;
        SetActuatorPID(address, p, i, d);
    }
    else{
        std::cerr << "Wrong actuator number. Given "<< actuator <<". But supported are only 1 to 6.";
    }

}
