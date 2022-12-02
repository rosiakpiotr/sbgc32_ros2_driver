#include "opencv2/opencv.hpp"
#include <iostream>


using namespace std;


cv::Point przesuw;

void detect(cv::Mat& frame)
{
    cv::Mat image = frame.clone();

    cv::medianBlur(image, image, 5);
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    //        captured_frame_bgr = cv2.cvtColor(captured_frame, cv2.COLOR_BGRA2BGR)
    //    # First blur to reduce noise prior to color space conversion
    //    captured_frame_bgr = cv2.medianBlur(captured_frame_bgr, 3)
    //    # Convert to Lab color space, we only need to check one channel (a-channel) for red here
    //    captured_frame_lab = cv2.cvtColor(captured_frame_bgr, cv2.COLOR_BGR2Lab)
    // captured_frame_lab_red = cv2.inRange(captured_frame_lab, np.array([20, 150, 150]), np.array([190, 255, 255]))

    int w = hsv_image.cols;
    int h = hsv_image.rows;
    //    cout << w << " " << h << endl;
    auto color = hsv_image.at<cv::Vec3b>(w / 2, h / 2);
    cout << int(color[0]) << " " << int(color[1]) << " " << int(color[2]) << endl;

    int minS = 128;
    int minV = 128;
    cv::Mat lower_red_hue_range;
    cv::Mat upper_red_hue_range;
    cv::inRange(hsv_image, cv::Scalar(0, minS, minV), cv::Scalar(10, 255, 255), lower_red_hue_range);
    cv::inRange(hsv_image, cv::Scalar(170, minS, minV), cv::Scalar(179, 255, 255), upper_red_hue_range);
    cv::Mat red_hue_image;
    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

    cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(red_hue_image, circles, cv::HOUGH_GRADIENT, 1, red_hue_image.rows / 8, 100, 20, 100, 200);

    for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
        cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
        int radius = std::round(circles[current_circle][2]);
        przesuw = center;
        cv::circle(frame, center, radius, cv::Scalar(255, 0, 0), 2);
       
    }

    cv::cvtColor(red_hue_image, red_hue_image, cv::COLOR_GRAY2BGR);
    cv::Mat result;
    cv::addWeighted(red_hue_image, 1.0, frame, 1.0, 0.0, result);
    result.copyTo(frame);

    przesuw -=cv::Point(640/2,480/2);
}
void move(cv::Point przesuw)
{
    if (przesuw.x > -10 && przesuw.x < 10)
    {
        cout << "Osiagnales poziom" << endl;
    }
    else
    {
        if (przesuw.x > 0)
        {
            cout << "Przesun kamere w prawo o " << przesuw.x << endl
                << "czyli o kat " << (przesuw.x * 63) / 640 << "stopni" << endl;
        }
        if (przesuw.x < 0)
        {
            cout << "Przesun kamere w lewo o " << przesuw.x << endl
                << "czyli o kat " << (przesuw.x * 63) / 640 << "stopni" << endl;
        }
    }

    if (przesuw.y > -10 && przesuw.y < 10)
    {
        cout << "Osiagnales pion" << endl;
    }
    else
    {
        if (przesuw.y > 0)
        {
            cout << "Przesun kamere w dol o " << przesuw.y << endl
                << "czyli o kat " << (przesuw.y * 63) / 640 << "stopni" << endl;
        }
        if (przesuw.y < 0)
        {
            cout << "Przesun kamere w gore o " << przesuw.y << endl
                << "czyli o kat " << (przesuw.y * 63) / 640 << "stopni" << endl;
        }
    }

}

int main()
{
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    while (1) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty())
            break;
        detect(frame);

        move(przesuw);

        imshow("Frame", frame);
        char c = (char)cv::waitKey(25);
        if (c == 27)
            break;
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}