#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <memory>

#include <opencv2/opencv.hpp>
#include <boost/optional.hpp>

#include "angles.hpp"
#include "detector.hpp"
#include "gimbal.hpp"

class Strategy
{
protected:
    bool show;

public:
    Strategy(bool show);

    virtual Angles offset(cv::Mat &frame, cv::Point point) = 0;

    virtual ~Strategy();
};

class Controller
{
protected:
    std::shared_ptr<Detector> detector;
    std::shared_ptr<Gimbal> gimbal;
    std::shared_ptr<Strategy> strategy;
    int withSpeed;
    unsigned int step;
    bool show;

public:
    Controller(const std::shared_ptr<Detector> &detector,
               const std::shared_ptr<Gimbal> &gimbal,
               const std::shared_ptr<Strategy> &strategy,
               int withSpeed,
               unsigned int step,
               bool show);

    void control(unsigned long long count, cv::Mat &frame);

    virtual ~Controller();
};

#endif
