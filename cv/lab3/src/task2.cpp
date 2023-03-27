#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "../include/lab3.hpp"

#define windowName "image"

using namespace lab3;

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

    showImage(img, windowName);
    cv::setMouseCallback(windowName, lab3::onMouseClicked, (void*)&img);

    cv::waitKey(0);
    return 0;
}

void lab3::onMouseClicked(int event, int x, int y, int f, void* userdata) {
    // assert it's a left click
    if (event != cv::EVENT_LBUTTONDOWN) return;

    // extract the img data
    cv::Mat img = *(cv::Mat*)userdata;
    // clone the original image
    cv::Mat imgCopy = img.clone();
    // get the BGR value
    cv::Vec3b pixelValues = imgCopy.at<cv::Vec3b>(y, x);

    // put the text at position
    std::string text = "(" + std::to_string(pixelValues[0]) + ", " +
                       std::to_string(pixelValues[1]) + ", " +
                       std::to_string(pixelValues[2]) + ")";
    cv::Point p{x, y};
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double fontSize = 0.7;
    cv::Scalar fontColor{50, 0, 255};
    int thickness = 2;
    cv::putText(imgCopy, text, p, font, fontSize, fontColor, thickness);

    // update the shown image
    showImage(imgCopy, windowName);
}
