#ifndef STRATEGIES_LOCAL_HPP
#define STRATEGIES_LOCAL_HPP

#include <opencv2/opencv.hpp>

#include "controller.hpp"

class LocalStrategy : public Strategy
{
protected:
    double angle;
    bool show;

public:
    LocalStrategy(double angle, bool show);

    Angles offset(cv::Mat &frame, cv::Point point) override;

    virtual ~LocalStrategy();
};

#endif
