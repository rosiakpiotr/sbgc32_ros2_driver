#include "detectors/cascade.hpp"

#include <iostream>

using namespace std;

CascadeDetector::CascadeDetector(string filename, int minSize, int maxSize, float detectionConfidence, bool show)
    : minSize(minSize), maxSize(maxSize), detectionConfidence(detectionConfidence), Detector(show)
{
    bool success = classifier.load(filename);
    if (!success) {
        throw runtime_error("Problem while loading cascade model.");
    }
}

boost::optional<cv::Point> CascadeDetector::detect(cv::Mat &frame)
{
    cv::Mat frameGray;
    cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Rect> results;
    std::vector<int> levels;
    std::vector<double> weights;

    classifier.detectMultiScale(frameGray,
                                results,
                                levels,
                                weights,
                                1.1,
                                3,
                                0 | cv::CASCADE_SCALE_IMAGE,
                                cv::Size(minSize, minSize),
                                cv::Size(maxSize, maxSize),
                                true);

    boost::optional<cv::Rect> bestResult = boost::none;
    double bestConfidence = 0;
    for (int i = 0; i < results.size(); i++) {
        if (weights[i] > detectionConfidence) {
            if (weights[i] > bestConfidence) {
                bestResult = results[i];
                bestConfidence = weights[i];
            }
        }
    }

    boost::optional<cv::Point> point = boost::none;
    if (bestResult) {
        cv::Rect result = *bestResult;
        int x = result.x + result.width / 2;
        int y = result.y + result.height / 2;
        point = cv::Point(x, y);

        if (show) {
            cv::rectangle(frame, result, cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, *point, 10, cv::Scalar(0, 255, 0), 2);
        }
    }

    return point;
}

CascadeDetector::~CascadeDetector()
{
}
