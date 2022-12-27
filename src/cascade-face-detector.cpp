#include "cascade-face-detector.hpp"

CascadeFaceDetector::CascadeFaceDetector(cv::CascadeClassifier *classifier, cv::Mat &frame,
                                         int faceMinSize, int faceMaxSize, float detectionConfidence)
    : minSizeDetected(faceMinSize),
      maxSizeDetected(faceMaxSize),
      detectionConfidence(detectionConfidence)
{
    detect(classifier, frame);
}

bool CascadeFaceDetector::faceDetected()
{
    return results.size() > 0;
}

void CascadeFaceDetector::showResults(cv::Mat &outFrame)
{
    for (int i = 0; i < results.size(); i++)
    {
        if (weights[i] > detectionConfidence)
        {
            cv::rectangle(outFrame, results[i], cv::Scalar(0, 255, 0), 2);
            int x = results[i].x + results[i].width / 2;
            int y = results[i].y + results[i].height / 2;
            cv::circle(outFrame, cv::Point(x, y), 10, cv::Scalar(0, 255, 0), 2);
        }
    }
}

cv::Point CascadeFaceDetector::centerOfFirstHit()
{
    for (int i = 0; i < results.size(); i++)
    {
        if (weights[i] > detectionConfidence)
        {
            int x = results[i].x + results[i].width / 2;
            int y = results[i].y + results[i].height / 2;
            return cv::Point(x, y);
        }
    }

    return cv::Point(0, 0);
}

void CascadeFaceDetector::detect(cv::CascadeClassifier *classifier, cv::Mat &frame)
{
    cv::Mat frameGray;
    cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);

    classifier->detectMultiScale(frameGray, results, levels, weights, 1.1, 3, 0 | cv::CASCADE_SCALE_IMAGE,
                                 cv::Size(minSizeDetected, minSizeDetected), cv::Size(maxSizeDetected, maxSizeDetected), true);
}
