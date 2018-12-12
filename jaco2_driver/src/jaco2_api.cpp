#include <jaco2_driver/jaco2_api.h>
#include <ros/console.h>
#include <arpa/inet.h>

Jaco2API::Jaco2API():
    stopedAPI_(true),
    initialized_(false)
{

}

Jaco2API::~Jaco2API()
{
    EraseAllTrajectories();
    StopControlAPI();
    CloseAPI();
    dlclose(api_command_lib_);
}

void Jaco2API::setupCommandInterface(const kinova::KinovaAPIType& api_type)
{
    api_type_ = api_type;

    if(api_type_ == kinova::KinovaAPIType::USB){
        api_command_lib_ = dlopen(KINOVA_USB_LIBRARY,RTLD_NOW|RTLD_GLOBAL);
        kinova_comm_lib_ = dlopen(KINOVA_USB_LIBRARY, RTLD_NOW | RTLD_GLOBAL);
    } else if (api_type_ == kinova::KinovaAPIType::ETHERNET) {
        api_command_lib_ = dlopen(KINOVA_ETH_LIBRARY,RTLD_NOW|RTLD_GLOBAL);
        kinova_comm_lib_ = dlopen(KINOVA_ETH_LIBRARY,RTLD_NOW|RTLD_GLOBAL);
    }

    if (api_type_ !=  kinova::KinovaAPIType::USB){
        InitEthernetAPI = (int (*)(EthernetCommConfig &))initCommandLayerFunction("InitEthernetAPI");
    }


    InitAPI                         = (int (*)()) initCommandLayerFunction("InitAPI");
    CloseAPI                        = (int (*)()) initCommandLayerFunction("CloseAPI");
    StopControlAPI                  = (int (*)()) initCommandLayerFunction("StopControlAPI");
    StartControlAPI                 = (int (*)()) initCommandLayerFunction("StartControlAPI");
    MoveHome                        = (int (*)()) initCommandLayerFunction("MoveHome");
    InitFingers                     = (int (*)()) initCommandLayerFunction("InitFingers");
    GetDevices                      = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) initCommandLayerFunction("GetDevices");
    SetActiveDevice                 = (int (*)(KinovaDevice devices)) initCommandLayerFunction("SetActiveDevice");
    SendBasicTrajectory             = (int (*)(TrajectoryPoint)) initCommandLayerFunction("SendBasicTrajectory");
    GetAngularCommand               = (int (*)(AngularPosition &)) initCommandLayerFunction("GetAngularCommand");
    GetAngularPosition              = (int (*)(AngularPosition &)) initCommandLayerFunction("GetAngularPosition");
    GetAngularVelocity              = (int (*)(AngularPosition &)) initCommandLayerFunction("GetAngularVelocity");
    GetAngularForce                 = (int (*)(AngularPosition &Response)) initCommandLayerFunction("GetAngularForce");
    GetAngularForceGravityFree      = (int (*)(AngularPosition &Response)) initCommandLayerFunction("GetAngularForceGravityFree");
    GetQuickStatus                  = (int (*)(QuickStatus &)) initCommandLayerFunction("GetQuickStatus");
    GetAngularCurrent               = (int (*)(AngularPosition &)) initCommandLayerFunction("GetAngularCurrent");
    EraseAllTrajectories            = (int (*)()) initCommandLayerFunction("EraseAllTrajectories");
    GetActuatorAcceleration         = (int (*)(AngularAcceleration &)) initCommandLayerFunction("GetActuatorAcceleration");
    GetAngularForceGravityFree      = (int (*)(AngularPosition &)) initCommandLayerFunction("GetAngularForceGravityFree");
    SetAngularControl               = (int (*)()) initCommandLayerFunction("SetAngularControl");
    SetCartesianControl             = (int (*)()) initCommandLayerFunction("SetCartesianControl");
    GetSensorsInfo                  = (int (*)(SensorsInfo &)) initCommandLayerFunction("GetSensorsInfo");
    SetTorqueZero                   = (int (*)(int)) initCommandLayerFunction("SetTorqueZero");
    SendAdvanceTrajectory           = (int (*)(TrajectoryPoint)) initCommandLayerFunction("SendAdvanceTrajectory");
    StopCurrentLimitation           = (int (*)()) initCommandLayerFunction("StopCurrentLimitation");
    GetCartesianPosition            = (int (*)(CartesianPosition &)) initCommandLayerFunction("GetCartesianPosition");
    GetAPIVersion                   = (int (*)(int Response[API_VERSION_COUNT])) initCommandLayerFunction("GetAPIVersion");
    RefresDevicesList               = (int (*)()) initCommandLayerFunction("RefresDevicesList");
    // untested functions since available  5.2.0
    RunGravityZEstimationSequence   = (int(*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) initCommandLayerFunction( "RunGravityZEstimationSequence");
    SwitchTrajectoryTorque          = (int(*)(GENERALCONTROL_TYPE)) initCommandLayerFunction( "SwitchTrajectoryTorque");
    SetTorqueSafetyFactor           = (int(*)(float)) initCommandLayerFunction( "SetTorqueSafetyFactor");
    SendAngularTorqueCommand        = (int(*)(float Command[COMMAND_SIZE])) initCommandLayerFunction( "SendAngularTorqueCommand");
    SendCartesianForceCommand       = (int(*)(float Command[COMMAND_SIZE])) initCommandLayerFunction( "SendCartesianForceCommand");
    SetGravityVector                = (int(*)(float Command[3])) initCommandLayerFunction( "SetGravityVector");
    SetGravityPayload               = (int(*)(float Command[GRAVITY_PAYLOAD_SIZE])) initCommandLayerFunction( "SetGravityPayload");
    SetGravityOptimalZParam         = (int(*)(float Command[GRAVITY_PARAM_SIZE])) initCommandLayerFunction( "SetGravityOptimalZParam");
    SetGravityType                  = (int(*)(GRAVITY_TYPE Type)) initCommandLayerFunction( "SetGravityType");
    GetCartesianForce               = (int(*)(CartesianPosition &)) initCommandLayerFunction( "GetCartesianForce");
    SetTorqueVibrationController    = (int(*)(float)) initCommandLayerFunction( "SetTorqueVibrationController");
    SetTorqueControlType            = (int(*)(TORQUECONTROL_TYPE)) initCommandLayerFunction( "SetTorqueControlType");
    GetTrajectoryTorqueMode         = (int(*)(int &)) initCommandLayerFunction( "GetTrajectoryTorqueMode");
    SetGravityType                  = (int(*)(GRAVITY_TYPE Type)) initCommandLayerFunction( "SetGravityType");
    SetGravityOptimalZParam         = (int(*)(float Command[GRAVITY_PARAM_SIZE])) initCommandLayerFunction( "SetGravityOptimalZParam");
    SetActuatorPID                  = (int (*)(unsigned int, float, float, float )) initCommandLayerFunction("SetActuatorPID");
    StartForceControl               = (int (*)()) initCommandLayerFunction("StartForceControl");
    StopForceControl                = (int (*)()) initCommandLayerFunction("StopForceControl");
    SetFrameType                    = (int (*)(int)) initCommandLayerFunction("SetFrameType");
}

void* Jaco2API::initCommLayerFunction(const char* name)
{
    char functionName[100];
    strcpy(functionName,name);
    if (api_type_ == kinova::KinovaAPIType::ETHERNET)
    {
        strcpy(functionName, "Ethernet_Communication_");
        strcat(functionName, name);
    }
    void * function_pointer = dlsym(kinova_comm_lib_, name);
    assert(function_pointer != NULL);
    return function_pointer;
}

void* Jaco2API::initCommandLayerFunction(const char* name)
{
    char functionName[100];
    strcpy(functionName,name);
    if (api_type_ ==  kinova::KinovaAPIType::ETHERNET)
    {
        strcpy(functionName, "Ethernet_");
        strcat(functionName, name);
    }
    void * function_pointer = dlsym(api_command_lib_,functionName);
    assert(function_pointer != NULL);
    return function_pointer;
}

int Jaco2API::initUSB()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    int result = -1;
    if((InitAPI == NULL) || (CloseAPI == NULL) || (SendBasicTrajectory == NULL) ||
            (SendBasicTrajectory == NULL) || (MoveHome == NULL) || (InitFingers == NULL))
    {
        std::cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << std::endl;
        result = ERROR_NOT_INITIALIZED;
    }
    result = InitAPI();
    if(result != NO_ERROR_KINOVA){
        std::cerr << "Could not initialize Kinova Ethernet API" << std::endl;
    } else {
        initialized_ = true;
    }
    std::cout << "Initialization's result : " << result << std::endl;
    return result;
}

int Jaco2API::initEthernet(const EthernetConfig &conf)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    int result = -1;
    if((InitAPI == NULL) || (CloseAPI == NULL) || (SendBasicTrajectory == NULL) ||
            (SendBasicTrajectory == NULL) || (MoveHome == NULL) || (InitFingers == NULL))
    {
        std::cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << std::endl;
        result = ERROR_NOT_INITIALIZED;
    }
    EthernetCommConfig econf;
    econf.localBcastPort = conf.local_bcast_port;
    econf.localCmdport = conf.local_cmd_port;
    econf.localIpAddress = inet_addr(conf.local_ip_address.c_str());
    econf.subnetMask = inet_addr(conf.subnet_mask.c_str());
    econf.rxTimeOutInMs = 2000;
    econf.robotIpAddress = inet_addr(conf.robot_ip_address.c_str());
    econf.robotPort = 55000;
    result = InitEthernetAPI(econf);
    if(result != NO_ERROR_KINOVA){
        std::cerr << "Could not initialize Kinova Ethernet API" << std::endl;
    } else {
        initialized_ = true;
    }
    return result;
}

int Jaco2API::init(std::string serial, bool right, bool move_home, bool init_fingers)
{
    right_arm_ = right;
    init_fingers_ = init_fingers;
    int result = -1;

    if((InitAPI == NULL) || (CloseAPI == NULL) || (SendBasicTrajectory == NULL) ||
            (SendBasicTrajectory == NULL) || (MoveHome == NULL) || (InitFingers == NULL))
    {
        std::cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << std::endl;
        result = ERROR_NOT_INITIALIZED;
    }
    else
    {
        stopedAPI_ = false;


        result = RefresDevicesList();


        result = NO_ERROR_KINOVA;
        KinovaDevice list[MAX_KINOVA_DEVICE];
        int devicesCount = GetDevices(list, result);
        if(result != NO_ERROR_KINOVA){
            return result;
        }

        //        std::size_t length = serial.length();

        for(int i = 0; i < devicesCount; ++i)
        {
            std::string serial_i = std::string(list[i].SerialNumber);
            std::cout << "Found a robot " << ((api_type_ == kinova::KinovaAPIType::ETHERNET) ? "using ETHERNET (" : "on the USB bus (")
                      << serial_i << ")" << std::endl;
            // If no device is specified, just use the first available device
            if (serial == "" || std::strcmp(serial.c_str(), serial_i.c_str()) == 0){
                std::cout << "Connect to the robot (" << serial_i << ")" << std::endl;
                result = SetActiveDevice(list[i]);


                if(move_home){
                    std::cout << "Send the robot to HOME position " << ((right_arm_) ? "right" : "left") << std::endl;
                    if(right_arm_){
                        result = MoveHome();
                    }
                    else{
                        moveHomeLeft();
                    }
                }
                if(init_fingers_){
                    std::cout << "Initializing the fingers" << std::endl;
                    result = InitFingers();
                }

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
    memset(&position, 0, sizeof(position));  // zero structure
    GetAngularPosition(position);
    return position;
}

AngularPosition Jaco2API::getAngularVelocity() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    AngularPosition velocity;
    memset(&velocity, 0, sizeof(velocity));  // zero structure
    GetAngularVelocity(velocity);
    return velocity;
}

AngularPosition Jaco2API::getAngularForce() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    AngularPosition torque;
    memset(&torque, 0, sizeof(torque));  // zero structure
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

void Jaco2API::setCartesianVelocity(const TrajectoryPoint &velocity)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if(!stopedAPI_){
        SendBasicTrajectory(velocity);
    }
}

void Jaco2API::setAngularPosition(const TrajectoryPoint &position)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if(!stopedAPI_){
        SendBasicTrajectory(position);
    }
}

void Jaco2API::setLimitedAngularCmd(const TrajectoryPoint &point)
{
    std::unique_lock<std::recursive_mutex>lock(mutex_);
    TrajectoryPoint cp = point;
    cp.LimitationsActive = 1;
    if(!stopedAPI_){
        SendAdvanceTrajectory(point);
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
    //    std::cout << "torque control type: " << res1 << std::endl;
    SetTorqueSafetyFactor(torque_saftey_factor);
    SetTorqueVibrationController(vibration_controller);
    int res2 = SwitchTrajectoryTorque(TORQUE);
    //    std::cout << "switching result: " << res2 << std::endl;
}

void Jaco2API::disableTorque()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    SwitchTrajectoryTorque(POSITION);
}

int Jaco2API::setTorqueZero(ActuatorID actuator)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    int result; //may return
    switch (actuator) {
    case Actuator1:
        result = (*SetTorqueZero)(16);
        break;
    case Actuator2:
        result = (*SetTorqueZero)(17);
        break;
    case Actuator3:
        result = (*SetTorqueZero)(18);
        break;
    case Actuator4:
        result = (*SetTorqueZero)(19);
        break;
    case Actuator5:
        result = (*SetTorqueZero)(20);
        break;
    case Actuator6:
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

    /* Email Kinova: Martine Blouin:
     * Subject: return of SetGravityOptimalZParam == 2005:
     * This is not a real error. Parameters should have been sent normally. The
     * reason for this error is because we do not send the same number of bytes in
     * this message than what the DSP expects to receive (we actually send more
     * bytes). So the robot actually receives what it needs even if it sends you
     * back this weird message.
     */
    std::unique_lock<std::recursive_mutex>lock(mutex_);
    SwitchTrajectoryTorque(POSITION);
    SetGravityType(MANUAL_INPUT);
    usleep(30000);
    bool ok = params.size() == OPTIMAL_Z_PARAM_SIZE;
    bool suc_change_param = false;
    bool suc_change_type = false;
    if(ok){

        float optimalParams [OPTIMAL_Z_PARAM_SIZE];
        //        std::size_t i = 0;
        for(std::size_t i = 0; i < OPTIMAL_Z_PARAM_SIZE; ++i){
            optimalParams[i] = (float) params[i];
            std::cout << i << " | " << optimalParams[i] << std::endl;
        }
        int res_change_param = SetGravityOptimalZParam(optimalParams);
        suc_change_param = (res_change_param == 1) || (res_change_param == 2005);
        usleep(30000);
        std::cout << "setting parameter result:  " << res_change_param<< std::endl;
        int res_change_type = SetGravityType(OPTIMAL);
        suc_change_type = res_change_type == 1;
        std::cout << "changing gravity type result:  " << res_change_type<< std::endl;
        usleep(30000);
        if(!suc_change_type || !suc_change_param){
            setGravityType(MANUAL_INPUT);
            std::cout << "something failed ... use manual prams." << std::endl;
        }
    }
    return ok && suc_change_type && suc_change_param;
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

    bool test = std::abs(p.Actuators.Actuator1 - ps.Actuators.Actuator1 ) < 200.0;
    test &= std::abs(p.Actuators.Actuator2 - ps.Actuators.Actuator2 ) < 200.0;
    test = std::abs(p.Actuators.Actuator3 - ps.Actuators.Actuator3 ) < 200.0;
    test = std::abs(p.Actuators.Actuator4 - ps.Actuators.Actuator4 ) < 200.0;
    test = std::abs(p.Actuators.Actuator5 - ps.Actuators.Actuator5 ) < 280.0;
    test = std::abs(p.Actuators.Actuator6 - ps.Actuators.Actuator6 ) < 360.0;

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

void Jaco2API::startForceControl()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    int res = StartForceControl();
    std::cout <<  "stop force control " << res << std::endl;
}

void Jaco2API::stopForceControl()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    int res = StopForceControl();
    std::cout << "stop force control " << res << std::endl;
}

void Jaco2API::setPayload(const jaco2_data::PayloadGravityParams &params)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    float payload[4];
    payload[0] = params.mass;
    payload[1] = params.cmx;
    payload[2] = params.cmy;
    payload[3] = params.cmz;

   /* int res = */SetGravityPayload(payload);

    return;

}

void Jaco2API::setReferenceFrameRotating()
{
    if(!initialized_ || SetFrameType == NULL){
        return;
    }
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    (*SetFrameType)(1);
    std::cout << "using rotating frame" << std::endl;
}

void Jaco2API::setReferenceFrameFixed()
{
    if(!initialized_ || SetFrameType == NULL){
        return;
    }
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    (*SetFrameType)(0);
    std::cout << "using fixed frame" << std::endl;
}
