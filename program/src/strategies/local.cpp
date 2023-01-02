#include "strategies/local.hpp"

#include "constants.hpp"

LocalStrategy::LocalStrategy(double angle, bool show)
    : angle(angle), Strategy(show)
{}

Angles LocalStrategy::offset(cv::Mat &frame, cv::Point point)
{
    cv::Size size = frame.size();
    cv::Point center(size.width / 2, size.height / 2);
    cv::Point offset = point - center;
    Angles angles;
    if (fabs(offset.x) != 0 || fabs(offset.y) != 0) {
        if (fabs(offset.x) > fabs(offset.y)) {
            angles.pitch = (offset.y / fabs(offset.x)) * angle;
            angles.yaw = (offset.x / fabs(offset.x)) * angle;;
        }
        if (fabs(offset.y) > fabs(offset.x)) {
            angles.pitch = (offset.y / fabs(offset.y)) * angle;
            angles.yaw = (offset.x / fabs(offset.y)) * angle;;
        }
        if (fabs(offset.x) == fabs(offset.y)) {
            angles.pitch = angle;
            angles.yaw = angle;;
        }
    }
    angles.roll = 0;

    if (show) {
        cv::arrowedLine(frame, center, point, cv::Scalar(0, 255, 255), 2);
    }

    return angles;
}

LocalStrategy::~LocalStrategy()
{
}


