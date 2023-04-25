#include "../include/lab5.hpp"

#include <iostream>

bool lab5::areArgumentsEnough(int argc, int n) { return argc >= n; }

bool lab5::isImageValid(const cv::Mat& img) { return img.data != NULL; }

void lab5::showImage(const cv::Mat& img, std::string windowName) {
    cv::namedWindow(windowName);
    cv::imshow(windowName, img);
}
