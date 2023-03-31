#include "gimbals/fake.hpp"

#include <iostream>
using namespace std;

FakeGimbal::FakeGimbal()
{
}

void FakeGimbal::motorsOn()
{
    cout << "Motors turned on." << endl;
}

void FakeGimbal::motorsOff()
{
    cout << "Motors turned off." << endl;
}

void FakeGimbal::moveToAngles(Angles target, int speed)
{
    cout << "Move to angles " << target << " ";
    cout << "with speed " << speed << "." << endl;
    angles = target;
}

Angles FakeGimbal::getCurrentAngles()
{
    return angles;
}

FakeGimbal::~FakeGimbal()
{
}