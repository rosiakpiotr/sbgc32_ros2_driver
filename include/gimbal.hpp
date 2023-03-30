#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include "angles.hpp"

class Gimbal
{
public:
    virtual void motorsOn() = 0;
    virtual void motorsOff() = 0;
    virtual void moveToAngles(Angles target, int withSpeed = 70) = 0;
    virtual Angles getCurrentAngles() = 0;

    virtual ~Gimbal();
};

#endif