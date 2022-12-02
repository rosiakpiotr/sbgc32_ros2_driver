#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include "linuxDriver/driver_Linux.h"
#include "adjvar/adjvar.h"
#include "gimbalControl/gimbalControl.h"
#include "profiles/profiles.h"
#include "realtime/realtime.h"
#include "service/service.h"
#include "stdio.h"

class Gimbal
{
public:
    // Performs necessary initializations
    Gimbal();

    bool initFailed();

    void moveYawTo(int angle, int withSpeed = 70);

    void movePitchTo(int angle, int withSpeed = 70);

    void moveRollTo(int angle, int withSpeed = 70);

    void motorsOn();

    void motorsOff();

private:
    void initializeDriver();
    void configControl();

    GeneralSBGC_t SBGC_1;
    Control_t Control;

    ControlConfig_t ControlConfig;
    ConfirmationState_t Confirm;
};

#endif