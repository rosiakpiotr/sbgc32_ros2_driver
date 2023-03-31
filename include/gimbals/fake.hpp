#ifndef GIMBAL_FAKE_HPP
#define GIMBAL_FAKE_HPP

#include "gimbal.hpp"
#include "angles.hpp"

class FakeGimbal : public Gimbal
{
public:
    FakeGimbal();

    void motorsOn() override;
    void motorsOff() override;
    void moveToAngles(Angles target, int withSpeed = 70) override;
    Angles getCurrentAngles() override;

    virtual ~FakeGimbal();

private:
    Angles angles;
};

#endif