#ifndef CASCADE_FACE_DETECTOR
#define CASCADE_FACE_DETECTOR

#include <vector>
#include <opencv2/opencv.hpp>

class CascadeFaceDetector
{
public:
    CascadeFaceDetector(cv::CascadeClassifier *classifier, cv::Mat &frame,
                        int faceMinSize, int faceMaxSize, float detectionConfidence);

    bool faceDetected();

    // Puts shapes on identified areas by drawing circles and rectangles around detected faces.
    void showResults(cv::Mat &outFrame);

    cv::Point centerOfFirstHit();

private:
    void detect(cv::CascadeClassifier *classifier, cv::Mat &frame);

    int minSizeDetected;
    int maxSizeDetected;
    float detectionConfidence;

    std::vector<cv::Rect> results;
    std::vector<int> levels;
    std::vector<double> weights;

    cv::CascadeClassifier *classifier;
};

#endif
