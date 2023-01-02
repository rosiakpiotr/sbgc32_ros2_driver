#include "strategies/global.hpp"

#include "constants.hpp"

GlobalStrategy::GlobalStrategy(bool show)
    : Strategy(show)
{}

Angles GlobalStrategy::offset(cv::Mat &frame, cv::Point point)
{
    cv::Size size = frame.size();
    cv::Point offset = point - cv::Point(size.width / 2, size.height / 2);
    Angles angles;
    angles.pitch = (offset.y * double(CAMERA_FOCAL_LENGTH)) / ((double) size.height);
    angles.yaw = (offset.x * double(CAMERA_FOCAL_LENGTH)) / ((double) size.width);
    angles.roll = 0;
    return angles;
}

GlobalStrategy::~GlobalStrategy()
{
}


