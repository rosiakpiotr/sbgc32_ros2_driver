#include "angles.hpp"

Angles Angles::operator+(Angles const &right)
{
    Angles result;
    result.pitch = this->pitch + right.pitch;
    result.yaw = this->yaw + right.yaw;
    result.roll = this->roll + right.roll;
    return result;
}