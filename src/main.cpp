#include <iostream>
#include <exception>
#include <math.h>

#include "gimbal.hpp"
#include "camera.hpp"

#define CAPTURE_WIDTH 640
#define CAPTURE_HEIGHT 480
#define FRAMERATE 15

using namespace std;
float stara_pozycja=0,nowa_pozycja=0;

std::string gstreamer_pipeline(int capture_width, int capture_height, int framerate, int display_width, int display_height)
{
    return " libcamerasrc ! video/x-raw, "
           " width=(int)" +
           std::to_string(capture_width) + ","
                                           " height=(int)" +
           std::to_string(capture_height) + ","
                                            " framerate=(fraction)" +
           std::to_string(framerate) + "/1 !"
                                       " videoconvert ! videoscale !"
                                       " video/x-raw,"
                                       " width=(int)" +
           std::to_string(display_width) + ","
                                           " height=(int)" +
           std::to_string(display_height) + " ! appsink";
}

cv::Point detect(cv::Mat &frame)
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

    cv::Point przesuw;

    for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle)
    {
        cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
        int radius = std::round(circles[current_circle][2]);

        przesuw = center;

        cv::circle(frame, center, radius, cv::Scalar(255, 0, 0), 2);
    }

    cv::cvtColor(red_hue_image, red_hue_image, cv::COLOR_GRAY2BGR);
    cv::Mat result;
    cv::addWeighted(red_hue_image, 1.0, frame, 1.0, 0.0, result);
    result.copyTo(frame);
    
    przesuw -= cv::Point(640/2,480/2);
    
    
    return przesuw;
}

void move(cv::Point &przesuw, Gimbal &gimbal)
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
                stara pozycja=(przesuw.x*63)/640;
                nowa_pozycja=nowa_pozycja+stara_pozycja;
                gimbal.movePitchTo(nowa_pozycja);


        }
        if (przesuw.x < 0)
        {
            cout << "Przesun kamere w lewo o " << przesuw.x << endl
                << "czyli o kat " << (przesuw.x * 63) / 640 << "stopni" << endl;
                stara pozycja=(przesuw.x*63)/640;
                nowa_pozycja=nowa_pozycja+stara_pozycja;
                gimbal.movePitchTo(nowa_pozycja);
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
                stara pozycja=(przesuw.y*63)/640;
                nowa_pozycja=nowa_pozycja+stara_pozycja;
                gimbal.moveYawTo(-nowa_pozycja);
        }
        if (przesuw.y < 0)
        {
            cout << "Przesun kamere w gore o " << przesuw.y << endl
                << "czyli o kat " << (przesuw.y * 63) / 640 << "stopni" << endl;
                stara pozycja=(przesuw.y*63)/640;
                nowa_pozycja=nowa_pozycja+stara_pozycja;
                gimbal.moveYawTo(-nowa_pozycja);
        }
    }
}

int main()
{
    
    Gimbal gimbal;

    try
    {
        gimbal.initializeDriver();
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    
    gimbal.configControl();
    gimbal.motorsOn();
    // std::cout << cv::getBuildInformation();

    // pipeline parameters
    
    int capture_width = 640;  // 1280 ;
    int capture_height = 480; // 720 ;
    int framerate = 15;
    int display_width = 640;  // 1280 ;
    int display_height = 480; // 720 ;

    // reset frame average
    std::string pipeline = gstreamer_pipeline(capture_width, capture_height, framerate,
                                              display_width, display_height);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n\n\n";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        std::cout << "Failed to open camera." << std::endl;
        return (-1);
    }

    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);

    // cv::VideoCapture cap(0);
    // cv::VideoCapture cap("v4l2src device=/dev/video0 io-mode=2 ! image/jpeg, width=(int)320, height=(int)240 ! nvjpegdec ! video/x-raw, format=I420 ! appsink", cv::CAP_GSTREAMER);
    // cv::VideoCapture cap("v4l2src device=/dev/video0", cv::CAP_GSTREAMER);
    // cv::VideoCapture cap("v4l2src device=/dev/video0 io-mode=2 ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! videoscale ! appsink", cv::CAP_GSTREAMER);

    // if (!cap.isOpened())
    // {
    //     cout << "Error opening video stream or file" << endl;
    //     return -1;
    // }

    while (1)
    {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty())
            break;
        auto przesuw = detect(frame);
        move(przesuw, gimbal);
                        
        imshow("Frame", frame);
        char c = (char)cv::waitKey(25);
        if (c == 27)
            break;
    }

    cap.release();
    cv::destroyAllWindows();
    
    gimbal.motorsOff();
    
    /*
    Gimbal gimbal;

    try
    {
        gimbal.initializeDriver();
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    Camera camera(
        CAPTURE_WIDTH,
        CAPTURE_HEIGHT,
        FRAMERATE);

    if (!camera.isOpened())
    {
        std::cerr << "Couldn't open the camera." << std::endl;
        return -1;
    }

    gimbal.configControl();
    gimbal.motorsOn();

    const int R = 25;
    const int resolution = 720;
    double step = (360.0 / (double)resolution) * M_PI / 180.0;

    while (true)
    {
        FOR_(i, resolution)
        {
            gimbal.movePitchTo(R * sin(i * step));
            gimbal.moveYawTo(R * cos(i * step));
            usleep(10000);
        }
    }

    gimbal.motorsOff();
    */

    return 0;
}
