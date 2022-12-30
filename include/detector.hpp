#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <opencv2/opencv.hpp>

class Detector
{
protected:
    bool show;

public:
    Detector(bool show);

    virtual cv::Point detect(cv::Mat &frame) = 0;

    virtual ~Detector();
};

#endif
