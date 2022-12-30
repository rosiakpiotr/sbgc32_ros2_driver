#include "detectors/cascade.hpp"

using namespace std;

CascadeDetector::CascadeDetector(string filename, int minSize, int maxSize, float detectionConfidence, bool show)
    : minSize(minSize), maxSize(maxSize), detectionConfidence(detectionConfidence), Detector(show)
{
    bool success = classifier.load(filename);
    if (!success)
    {
        throw runtime_error("Problem while loading cascade model.");
    }
}

cv::Point CascadeDetector::detect(cv::Mat &frame)
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

    if (show) {
        for (int i = 0; i < results.size(); i++) {
            if (weights[i] > detectionConfidence) {
                cv::rectangle(frame, results[i], cv::Scalar(0, 255, 0), 2);
                int x = results[i].x + results[i].width / 2;
                int y = results[i].y + results[i].height / 2;
                cv::circle(frame, cv::Point(x, y), 10, cv::Scalar(0, 255, 0), 2);
            }
        }
    }

    for (int i = 0; i < results.size(); i++) {
        if (weights[i] > detectionConfidence) {
            int x = results[i].x + results[i].width / 2;
            int y = results[i].y + results[i].height / 2;
            return cv::Point(x, y);
        }
    }

    return cv::Point(0, 0);
}

CascadeDetector::~CascadeDetector()
{
}
