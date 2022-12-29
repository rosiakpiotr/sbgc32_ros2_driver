#ifndef GIMBAL_REAL_HPP
#define GIMBAL_REAL_HPP

#include "gimbal.hpp"

#include "gimbalControl/gimbalControl.h"
#include "realtime/realtime.h"

class RealGimbal : public Gimbal
{
private:
    GeneralSBGC_t SBGC_1;
    Control_t Control;

    ControlConfig_t ControlConfig;
    ConfirmationState_t Confirm;

    // Realtime data from the gimbal driver.
    DataStreamInterval_t DataStreamInterval;

private:
    void initializeDriver(); // Throws an exception if initialization fails.
    void configControl();
    bool initFailed();

public:
    RealGimbal();

    void motorsOn() override;
    void motorsOff() override;
    void moveToAngles(Angles target, int withSpeed = 70) override;
    Angles getCurrentPosition() override;

    virtual ~RealGimbal();
};

#endif