#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <boost/optional.hpp>

class Detector
{
protected:
    bool show;

public:
    Detector(bool show);

    virtual boost::optional<cv::Point> detect(cv::Mat &frame) = 0;

    virtual ~Detector();
};

#endif
