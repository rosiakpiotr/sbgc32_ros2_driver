#include "angles.hpp"

Angles::Angles()
    : pitch(0), yaw(0), roll(0)
{}

Angles::Angles(double pitch, double yaw, double roll)
    : pitch(pitch), yaw(yaw), roll(roll)
{}

Angles Angles::operator+(Angles const &right)
{
    Angles result;
    result.pitch = this->pitch + right.pitch;
    result.yaw = this->yaw + right.yaw;
    result.roll = this->roll + right.roll;
    return result;
}

std::ostream &operator<<(std::ostream &out, const Angles &angles)
{
    out << "(" << angles.pitch << " ";
    out << angles.yaw << " ";
    out << angles.roll << ")";
    return out;
}
