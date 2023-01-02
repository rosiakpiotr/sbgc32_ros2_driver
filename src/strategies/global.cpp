#include "strategies/global.hpp"

#include "constants.hpp"

GlobalStrategy::GlobalStrategy(bool show)
    : Strategy(show)
{}

Angles GlobalStrategy::offset(cv::Mat &frame, cv::Point point)
{
    cv::Point offset = point - cv::Point(CAPTURE_WIDTH / 2, CAPTURE_HEIGHT / 2);
    Angles angles;
    angles.pitch = offset.y * CAMERA_FOCAL_LENGTH / ((double) CAPTURE_HEIGHT);
    angles.yaw = offset.x * CAMERA_FOCAL_LENGTH / ((double) CAPTURE_WIDTH);
    angles.roll = 0;
    return angles;
}

GlobalStrategy::~GlobalStrategy()
{
}


