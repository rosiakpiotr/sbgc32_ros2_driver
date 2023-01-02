#include "strategies/global.hpp"

#include "constants.hpp"

GlobalStrategy::GlobalStrategy(double focalLength, bool show)
    : focalLength(focalLength), Strategy(show)
{}

Angles GlobalStrategy::offset(cv::Mat &frame, cv::Point point)
{
    cv::Size size = frame.size();
    cv::Point center(size.width / 2, size.height / 2);
    cv::Point offset = point - center;
    Angles angles;
    angles.pitch = (offset.y * focalLength) / ((double) size.height);
    angles.yaw = (offset.x * focalLength) / ((double) size.width);
    angles.roll = 0;

    if(show) {
        cv::arrowedLine(frame, center, point, cv::Scalar(0, 255, 255), 2);
    }

    return angles;
}

GlobalStrategy::~GlobalStrategy()
{
}


