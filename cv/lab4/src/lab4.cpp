#include "../include/lab4.hpp"

#include <iostream>

bool lab4::areArgumentsEnough(int argc, int n) { return argc >= n; }

bool lab4::isImageValid(const cv::Mat& img) { return img.data != NULL; }

void lab4::showImage(const cv::Mat& img, std::string windowName) {
    cv::namedWindow(windowName);
    cv::imshow(windowName, img);
}
