#include "gimbals/real.hpp"

#include <stdexcept>

#include "linuxDriver/driver_Linux.h"
#include "service/service.h"

void RealGimbal::initializeDriver()
{
    /*  - - - - - - - - SBGC Hardware-Software Init - - - - - - - - - */
    /* Driver Init */
    SBGC_1.Drv = malloc(sizeof(Driver_t));
    DriverInit(SBGC_1.Drv, (char *)SBGC_SERIAL_PORT);

    if (initFailed())
    {
        throw std::runtime_error("Bad file descriptor - no device connected.");
        // TODO: Throw more specific exception like -> throw std::system_error("Bad file descriptor - no device connected.");
    }

    /* High Layer Init */
    SBGC32_DefaultInit(&SBGC_1, PortTransmitData, PortReceiveByte, GetAvailableBytes,
                       PrintDebugData, GetTimeMs, SBGC_PROTOCOL_V2);
}

void RealGimbal::configControl()
{
    /* Control Configurations */
    ControlConfig.AxisCC[ROLL].angleLPF = 6;
    ControlConfig.AxisCC[PITCH].angleLPF = 6;
    ControlConfig.AxisCC[YAW].angleLPF = 7;

    ControlConfig.AxisCC[ROLL].angleLPF = 6;
    ControlConfig.AxisCC[PITCH].speedLPF = 6;
    ControlConfig.AxisCC[YAW].speedLPF = 7;
    ControlConfig.flags = RTCCF_CONTROL_CONFIG_FLAG_NO_CONFIRM;

    Control.controlMode[ROLL] = CtrlM_MODE_ANGLE;
    Control.controlMode[PITCH] = CtrlM_MODE_ANGLE;
    Control.controlMode[YAW] = CtrlM_MODE_ANGLE;

    Control.AxisC[ROLL].angle = 0;
    Control.AxisC[PITCH].angle = 0;
    Control.AxisC[YAW].angle = 0;

    Control.AxisC[ROLL].speed = SPEED_TO_VALUE(20);
    Control.AxisC[PITCH].speed = SPEED_TO_VALUE(20);
    Control.AxisC[YAW].speed = SPEED_TO_VALUE(20);

    SBGC32_ControlConfig(&SBGC_1, &ControlConfig, &Confirm);
}

bool RealGimbal::initFailed()
{
    return ((Driver_t *)SBGC_1.Drv)->devFD == -1;
}

RealGimbal::RealGimbal()
{
    initializeDriver();
    configControl();
}

void RealGimbal::motorsOn()
{
    SBGC32_SetMotorsON(&SBGC_1, &Confirm);
    // std::cout << "Motors turned on." << std::endl;
}

void RealGimbal::motorsOff()
{
    SBGC32_SetMotorsOFF(&SBGC_1, MM_SAFE_STOP, &Confirm);
    // std::cout << "Motors turned off." << std::endl;
}

void RealGimbal::moveToAngles(Angles target, int speed)
{
    Control.AxisC[YAW].angle = DEGREE_TO_ANGLE(target.yaw);
    Control.AxisC[PITCH].angle = DEGREE_TO_ANGLE(target.pitch);
    Control.AxisC[ROLL].angle = DEGREE_TO_ANGLE(target.roll);

    Control.AxisC[YAW].speed = SPEED_TO_VALUE(speed);
    Control.AxisC[PITCH].speed = SPEED_TO_VALUE(speed);
    Control.AxisC[ROLL].speed = SPEED_TO_VALUE(speed);

    SBGC32_Control(&SBGC_1, &Control);
}

Angles RealGimbal::getCurrentAngles()
{
    GetAngles_t rawAngles;
    Angles gimbalDegreeAngles;

    SBGC32_GetAngles(&SBGC_1, &rawAngles);
    gimbalDegreeAngles.pitch = rawAngles.AxisGA[PITCH].IMU_Angle * 0.02197265625;
    gimbalDegreeAngles.yaw = rawAngles.AxisGA[YAW].IMU_Angle * 0.02197265625;
    gimbalDegreeAngles.roll = rawAngles.AxisGA[ROLL].IMU_Angle * 0.02197265625;

    return gimbalDegreeAngles;
}

RealGimbal::~RealGimbal()
{
}