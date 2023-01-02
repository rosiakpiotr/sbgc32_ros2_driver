#ifndef STRATEGIES_GLOBAL_HPP
#define STRATEGIES_GLOBAL_HPP

#include <opencv2/opencv.hpp>

#include "controller.hpp"

class GlobalStrategy : public Strategy
{
protected:
    bool show;

public:
    GlobalStrategy(bool show);

    Angles offset(cv::Mat &frame, cv::Point point) override;

    virtual ~GlobalStrategy();
};

#endif
