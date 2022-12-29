#include "helpers.hpp"

Angles globalOffsetToCameraAngles(cv::Point globalOffset)
{
    Angles angles;
    angles.pitch = globalOffset.y * CAMERA_FOCAL_LENGTH / ((double)CAPTURE_HEIGHT);
    angles.yaw = globalOffset.x * CAMERA_FOCAL_LENGTH / ((double)CAPTURE_WIDTH);
    return angles;
}
