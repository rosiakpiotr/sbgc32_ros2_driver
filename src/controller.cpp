#include "controller.hpp"

Strategy::Strategy(bool show)
    : show(show)
{

}
Strategy::~Strategy()
{

}

Controller::Controller(const std::shared_ptr<Detector> &detector,
                       const std::shared_ptr<Gimbal> &gimbal,
                       const std::shared_ptr<Strategy> &strategy,
                       unsigned int step,
                       bool show)
    : detector(detector), gimbal(gimbal), strategy(strategy), step(step), show(show)
{}

void Controller::control(unsigned long long  count, cv::Mat &frame)
{
    boost::optional<cv::Point> point = detector->detect(frame);
    if (count % step == 0) {
        Angles current = gimbal->getCurrentAngles();
        Angles offset;
        if (point) {
            offset = strategy->offset(frame, *point);
        }
        Angles target = current + offset;
        gimbal->moveToAngles(target);

    }
}

Controller::~Controller()
{
}