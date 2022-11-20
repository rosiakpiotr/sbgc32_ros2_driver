#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include "linuxDriver/driver_Linux.h"

#include "adjvar/adjvar.h"

#include "gimbalControl/gimbalControl.h"

#include "profiles/profiles.h"

#include "realtime/realtime.h"

#include "service/service.h"

#include "stdio.h"

#include <iostream>

/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾ */
/*   					Global Software Objects  					  */
/* __________________________________________________________________ */

// TODO: Change global variables to class attributes 
static GeneralSBGC_t SBGC_1;

static ConfirmationState_t Confirm;

static Control_t Control;
static ControlConfig_t ControlConfig;

static BoardInfo_t BoardInfo;
static BoardInfo3_t BoardInfo3;
static MainParams3_t MainParams3;
static MainParamsExt_t MainParamsExt;
static MainParamsExt2_t MainParamsExt2;
static MainParamsExt3_t MainParamsExt3;

static RealTimeDataCustom_t RealTimeDataCustom;
static RealTimeData_t RealTimeData;

static AdjVarsGeneral_t AdjVarsGeneral[3];
extern
const
    AdjVarsDebugInfo_t AdjVarsDebugInfoArray[];

static DataStreamInterval_t DataStreamInterval;

static BeeperSettings_t BeeperSettings;

static ui8 DataStreamBuff[20];

class Gimbal {
    public:
        Gimbal() {

        }

    void initializeDriver() {
        /*  - - - - - - - - SBGC Hardware-Software Init - - - - - - - - - */
        /* Driver Init */
        SBGC_1.Drv = malloc(sizeof(Driver_t));
        DriverInit(SBGC_1.Drv, SBGC_SERIAL_PORT);

        /* High Layer Init */
        SBGC32_DefaultInit( & SBGC_1, PortTransmitData, PortReceiveByte, GetAvailableBytes,
            PrintDebugData, GetTimeMs, SBGC_PROTOCOL_V2);
    }

    void configControl()
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

    void moveYawTo(int angle, int speed = 70)
    {
        Control.AxisC[YAW].angle = DEGREE_TO_ANGLE_INT(angle);
        Control.AxisC[YAW].speed = SPEED_TO_VALUE(speed);
        SBGC32_Control(&SBGC_1, &Control);
    }

    void movePitchTo(int angle, int speed = 70)
    {
        Control.AxisC[PITCH].angle = DEGREE_TO_ANGLE_INT(angle);
        Control.AxisC[PITCH].speed = SPEED_TO_VALUE(speed);
        SBGC32_Control(&SBGC_1, &Control);
    }

    void moveRollTo(int angle, int speed = 70)
    {
        Control.AxisC[ROLL].angle = DEGREE_TO_ANGLE_INT(angle);
        Control.AxisC[ROLL].speed = SPEED_TO_VALUE(speed);
        SBGC32_Control(&SBGC_1, &Control);
    }

    void motorsOn()
    {
        SBGC32_SetMotorsON(&SBGC_1, &Confirm);
        std::cout << "Motors turned on." << std::endl;
    }

    void motorsOff()
    {
        SBGC32_SetMotorsOFF(&SBGC_1, MM_SAFE_STOP, &Confirm);
        std::cout << "Motors turned off." << std::endl;
    }
};

#endif