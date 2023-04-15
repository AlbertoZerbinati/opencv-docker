#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../include/lab4.hpp"
#include "opencv2/ximgproc.hpp"

#define original_img_window "original img"

using namespace lab4;

int main(int argc, char** argv) {
    if (!areArgumentsEnough(argc, 2)) {
        std::cout << "\nusage: task2.exe <img_path>\n";
        return 1;
    }

    cv::Mat img = cv::imread(argv[1]);
    if (!isImageValid(img)) {
        std::cout << "\n" << argv[1] << " is not a vaild path for an image\n";
        return 1;
    }

    cv::Mat smooth, canny;

    // smooth noise away
    cv::blur(img, smooth, cv::Size(3, 3));

    // edge detection
    cv::Canny(smooth, canny, 50, 200, 3);

    // get hough circles
    cv::Mat hough = img.clone();
    std::vector<cv::Vec3f> circles;
    HoughCircles(canny, circles, cv::HOUGH_GRADIENT, 2, smooth.rows / 2, 60,
                 30, 6, 8);

    // draw circles
    for (size_t i = 0; i < circles.size(); ++i) {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        int radius = c[2];
        circle(hough, center, radius, cv::Scalar(0, 255, 0), cv::FILLED,
               cv::LINE_AA);
    }

    showImage(hough, "Hough Circles");
    cv::waitKey(0);

    return 0;
}
