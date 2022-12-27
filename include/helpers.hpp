#ifndef HELPERS_H
#define HELPERS_H

#include <opencv2/opencv.hpp>

#include "constants.hpp"
#include "angles.hpp"

Angles globalOffsetToCameraAngles(cv::Point globalOffset);

#endif