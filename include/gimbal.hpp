#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include <stdio.h>
#include <stdexcept>

#include "linuxDriver/driver_Linux.h"
#include "adjvar/adjvar.h"
#include "gimbalControl/gimbalControl.h"
#include "profiles/profiles.h"
#include "realtime/realtime.h"
#include "service/service.h"
#include "angles.hpp"

class Gimbal
{
public:
    Gimbal();

    ///////////////////////////////////////////////
    //// Call these methods in following order       ////
    //// before attempting to move the gimbal. ////
    ///////////////////////////////////////////////
    void initializeDriver(); // Throws an exception if initialization fails.

    void initializeRealTimeData(uint16_t pollingInterval);

    void configControl();

    void motorsOn();
    ///////////////////////////////////////////////

    bool initFailed();

    void moveYawTo(int angle, int withSpeed = 70);

    void movePitchTo(int angle, int withSpeed = 70);

    void moveRollTo(int angle, int withSpeed = 70);

    void motorsOff();

    Angles getCurrentPosition();

private:
    GeneralSBGC_t SBGC_1;
    Control_t Control;

    ControlConfig_t ControlConfig;
    ConfirmationState_t Confirm;

    // Realtime data from the gimbal driver.
    DataStreamInterval_t DataStreamInterval;
};

#endif