#include <boost/test/unit_test.hpp>

#include "strategies/local.hpp"
#include "strategies/global.hpp"

#include "constants.hpp"

using namespace std;


BOOST_AUTO_TEST_SUITE(StrategySuite)

BOOST_AUTO_TEST_CASE(GlobalStrategyTest)
{
    GlobalStrategy strategy(CAMERA_FOCAL_LENGTH, false);
    cv::Mat frame(CAPTURE_HEIGHT, CAPTURE_WIDTH, CV_8UC1, cv::Scalar(0, 0, 0));
    cv::Point point(CAPTURE_WIDTH/2, CAPTURE_HEIGHT/2);
    Angles offset = strategy.offset(frame, point);
    BOOST_TEST(offset.pitch == 0);
    BOOST_TEST(offset.yaw == 0);
    BOOST_TEST(offset.roll == 0);
}

BOOST_AUTO_TEST_CASE(LocalStrategyTest)
{
    double angle = 2;
    LocalStrategy strategy(angle, false);
    cv::Mat frame(CAPTURE_HEIGHT, CAPTURE_WIDTH, CV_8UC1, cv::Scalar(0, 0, 0));
    cv::Point point(CAPTURE_WIDTH/2, CAPTURE_HEIGHT/2);
    Angles offset = strategy.offset(frame, point);
    BOOST_TEST(offset.pitch == 0);
    BOOST_TEST(offset.yaw == 0);
    BOOST_TEST(offset.roll == 0);
}

BOOST_AUTO_TEST_SUITE_END()
