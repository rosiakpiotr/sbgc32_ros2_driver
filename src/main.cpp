#include "gimbal.hpp"
#include <math.h>

int main()
{
    Gimbal gimbal;
    gimbal.initializeDriver();
    gimbal.configControl();
    gimbal.motorsOn();

    const int R = 25;
    const int resolution = 720;
    double step = (360.0 / (double)resolution) * M_PI / 180.0;

    while(true)
    {
        FOR_(i, resolution)
        {
            gimbal.movePitchTo(R * sin(i*step));
            gimbal.moveYawTo(R * cos(i*step));
            usleep(10000);
        }
    }

    gimbal.motorsOff();
    return 0;
}