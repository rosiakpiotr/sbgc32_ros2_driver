#ifndef ANGLES_H
#define ANGLES_H

#include <iostream>

struct Angles
{
    double pitch;
    double yaw;
    double roll;

    Angles();
    Angles(double pitch, double yaw, double roll);

    Angles operator+(Angles const &right);
};

std::ostream &operator<<(std::ostream &out, const Angles &angles);

#endif
