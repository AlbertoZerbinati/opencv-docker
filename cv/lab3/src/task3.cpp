#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "../include/lab3.hpp"

#define windowName "image"

using namespace lab3;

int main(int argc, char** argv) {
    if (!areArgumentsEnough(argc, 2)) {
        std::cout << "\nusage: task3.exe <img_path>\n";
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

    // make sure the 9x9 grid is in the image
    if (y - 1 < 0 || y + 1 >= imgCopy.rows || x - 1 < 0 ||
        x + 1 >= imgCopy.cols) {
        std::cerr << "neighborhood out of bounds\n";
        return;
    }

    // get the mean values for each channel, in the 9x9 neighbors around (x, y)
    cv::Rect neighborhood(x - 1, y - 1, 3, 3);
    cv::Scalar mean = cv::mean(imgCopy(neighborhood));

    // put the text at position
    std::string text = "(" + std::to_string((int)mean[0]) + ", " +
                       std::to_string((int)mean[1]) + ", " +
                       std::to_string((int)mean[2]) + ")";
    cv::Point p{x, y};
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double fontSize = 0.7;
    cv::Scalar fontColor{50, 255, 0};
    int thickness = 2;
    cv::putText(imgCopy, text, p, font, fontSize, fontColor, thickness);

    // update the shown image
    showImage(imgCopy, windowName);
}
