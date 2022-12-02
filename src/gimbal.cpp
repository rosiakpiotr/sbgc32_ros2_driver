#include "gimbal.hpp"

Gimbal::Gimbal()
{
    initializeDriver();
    configControl();
}

void Gimbal::initializeDriver()
{
    /*  - - - - - - - - SBGC Hardware-Software Init - - - - - - - - - */
    /* Driver Init */
    SBGC_1.Drv = malloc(sizeof(Driver_t));
    DriverInit(SBGC_1.Drv, (char *)SBGC_SERIAL_PORT);

    /* High Layer Init */
    SBGC32_DefaultInit(&SBGC_1, PortTransmitData, PortReceiveByte, GetAvailableBytes,
                       PrintDebugData, GetTimeMs, SBGC_PROTOCOL_V2);
}

bool Gimbal::initFailed()
{
    return ((Driver_t *)SBGC_1.Drv)->devFD == -1;
}

void Gimbal::configControl()
{
    /* Control Configurations */
    ControlConfig.AxisCC[ROLL].angleLPF = 6;
    ControlConfig.AxisCC[PITCH].angleLPF = 6;
    ControlConfig.AxisCC[YAW].angleLPF = 7;

    ControlConfig.AxisCC[ROLL].angleLPF = 6;
    ControlConfig.AxisCC[PITCH].speedLPF = 6;
    ControlConfig.AxisCC[YAW].speedLPF = 7;
    ControlConfig.flags = RTCCF_CONTROL_CONFIG_FLAG_NO_CONFIRM;

    Control.controlMode[ROLL] = CtrlM_MODE_ANGLE | CtrlF_CONTROL_FLAG_TARGET_PRECISE;
    Control.controlMode[PITCH] = CtrlM_MODE_ANGLE | CtrlF_CONTROL_FLAG_TARGET_PRECISE;
    Control.controlMode[YAW] = CtrlM_MODE_ANGLE | CtrlF_CONTROL_FLAG_TARGET_PRECISE;

    Control.AxisC[ROLL].angle = 0;
    Control.AxisC[PITCH].angle = 0;
    Control.AxisC[YAW].angle = 0;

    Control.AxisC[ROLL].speed = SPEED_TO_VALUE(20);
    Control.AxisC[PITCH].speed = SPEED_TO_VALUE(20);
    Control.AxisC[YAW].speed = SPEED_TO_VALUE(20);

    SBGC32_ControlConfig(&SBGC_1, &ControlConfig, &Confirm);
}

void Gimbal::moveYawTo(int angle, int speed = 70)
{
    Control.AxisC[YAW].angle = DEGREE_TO_ANGLE_INT(angle);
    Control.AxisC[YAW].speed = SPEED_TO_VALUE(speed);
    SBGC32_Control(&SBGC_1, &Control);
}

void Gimbal::movePitchTo(int angle, int speed = 70)
{
    Control.AxisC[PITCH].angle = DEGREE_TO_ANGLE_INT(angle);
    Control.AxisC[PITCH].speed = SPEED_TO_VALUE(speed);
    SBGC32_Control(&SBGC_1, &Control);
}

void Gimbal::moveRollTo(int angle, int speed = 70)
{
    Control.AxisC[ROLL].angle = DEGREE_TO_ANGLE_INT(angle);
    Control.AxisC[ROLL].speed = SPEED_TO_VALUE(speed);
    SBGC32_Control(&SBGC_1, &Control);
}

void Gimbal::motorsOn()
{
    SBGC32_SetMotorsON(&SBGC_1, &Confirm);
    // std::cout << "Motors turned on." << std::endl;
}

void Gimbal::motorsOff()
{
    SBGC32_SetMotorsOFF(&SBGC_1, MM_SAFE_STOP, &Confirm);
    // std::cout << "Motors turned off." << std::endl;
}