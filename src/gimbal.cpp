#include "gimbal.hpp"

Gimbal::Gimbal()
{
}

void Gimbal::initializeDriver()
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

void Gimbal::initializeRealTimeData(uint16_t pollingInterval)
{
    /* Data Stream Configurations */
    DataStreamInterval.cmdID = CMD_REALTIME_DATA_CUSTOM;
    DataStreamInterval.intervalMs = pollingInterval;
    DataStreamInterval.syncToData = STD_SYNC_OFF;

    ui32 DataStreamIntervalConfig = RTDCF_STATOR_ROTOR_ANGLE; // | RTDCF_GYRO_DATA | RTDCF_ACC_DATA;
    memcpy(DataStreamInterval.config, &DataStreamIntervalConfig, sizeof(DataStreamIntervalConfig));

    SBGC32_RequestDataStream(&SBGC_1, &DataStreamInterval, &Confirm);
}

bool Gimbal::initFailed()
{
    return ((Driver_t *)SBGC_1.Drv)->devFD == -1;
}

void Gimbal::moveToAngles(Angles target, int speed)
{
    Control.AxisC[YAW].angle = DEGREE_TO_ANGLE(target.yaw);
    Control.AxisC[PITCH].angle = DEGREE_TO_ANGLE(target.pitch);
    Control.AxisC[ROLL].angle = DEGREE_TO_ANGLE(target.roll);

    Control.AxisC[YAW].speed = SPEED_TO_VALUE(speed);
    Control.AxisC[PITCH].speed = SPEED_TO_VALUE(speed);
    Control.AxisC[ROLL].speed = SPEED_TO_VALUE(speed);

    SBGC32_Control(&SBGC_1, &Control);
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

Angles Gimbal::getCurrentPosition()
{
    RealTimeDataCustom_t RealTimeDataCustom;
    ui8 DataStreamBuff[20];
    SBGC32_ParseDataStream(&SBGC_1, DataStreamBuff, (SBGC_Commands_t)DataStreamInterval.cmdID);

    /* Preparing */
    ui8 BuffRPx = 2; // ui16 timestampMs offset

    BuffRPx += ConvertWithPM(RealTimeDataCustom.frameCamAngle, &DataStreamBuff[BuffRPx],
                             sizeof(RealTimeDataCustom.targetAngles), PM_DEFAULT_16BIT);
    // BuffRPx += ConvertWithPM(RealTimeDataCustom.gyroData, &DataStreamBuff[BuffRPx],
    //                          sizeof(RealTimeDataCustom.gyroData), PM_DEFAULT_16BIT);
    // BuffRPx += ConvertWithPM(RealTimeDataCustom.ACC_Data, &DataStreamBuff[BuffRPx],
    //                          sizeof(RealTimeDataCustom.ACC_Data), PM_DEFAULT_16BIT);

    return {
        RealTimeDataCustom.frameCamAngle[PITCH] * 0.02197265625,
        RealTimeDataCustom.frameCamAngle[YAW] * 0.02197265625,
        RealTimeDataCustom.frameCamAngle[ROLL] * 0.02197265625,
    };
}