#ifndef DERECTORS_CASCADE_HPP
#define DETECTORS_CASCADE_HPP

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>

#include "detector.hpp"

class CascadeDetector: public Detector
{
private:
    cv::CascadeClassifier classifier;
    int minSize;
    int maxSize;
    float detectionConfidence;

public:
    CascadeDetector(std::string filename, int minSize, int maxSize, float detectionConfidence, bool show);

    cv::Point detect(cv::Mat &frame) override;

    virtual ~CascadeDetector();
};

#endif
