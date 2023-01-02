#include "controller.hpp"

#include <sstream>

#include "constants.hpp"

using namespace std;

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

void Controller::control(unsigned long long count, cv::Mat &frame)
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

        if (show) {
            auto font = cv::FONT_HERSHEY_PLAIN;
            double scale = 2;
            int b = 0;
            cv::Size s = cv::getTextSize("0", font, scale, 2, &b);

            cv::Size size = frame.size();
            double d = 50;
            cv::Scalar color(255, 0, 255);
            int thickness = 2;

            ostringstream pitch;
            pitch.precision(2);
            pitch << current.pitch << " + " << offset.pitch;
            cv::ellipse(frame, cv::Point(d, size.height / 2), cv::Size(int(d / 2), int(d)), 0, 0, 270, color, thickness);
            cv::putText(frame,
                        pitch.str(),
                        cv::Point(d + d / 2 + s.width, size.height / 2 + s.height / 2),
                        font,
                        scale,
                        color,
                        thickness);

            ostringstream yaw;
            yaw.precision(2);
            yaw << current.yaw << " + " << offset.yaw;
            cv::ellipse(frame, cv::Point(size.width / 2, d), cv::Size(int(d), int(d / 2)), 0, 0, 270, color, thickness);
            cv::putText(frame,
                        yaw.str(),
                        cv::Point(size.width / 2+d+s.width,d+s.height/2),
                        font,
                        scale,
                        color,
                        thickness);
        }
    }
}

Controller::~Controller()
{
}