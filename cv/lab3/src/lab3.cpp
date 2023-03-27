#include "../include/lab3.hpp"

#include <iostream>

bool lab3::areArgumentsEnough(int argc, int n) { return argc >= n; }

bool lab3::isImageValid(const cv::Mat& img) { return img.data != NULL; }

void lab3::showImage(const cv::Mat& img, std::string windowName) {
    cv::namedWindow(windowName);
    cv::imshow(windowName, img);
}
