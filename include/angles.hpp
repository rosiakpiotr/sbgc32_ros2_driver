#ifndef ANGLES_H
#define ANGLES_H

struct Angles
{
    double pitch;
    double yaw;
    double roll;

    Angles operator+(Angles const &right);
};

#endif
