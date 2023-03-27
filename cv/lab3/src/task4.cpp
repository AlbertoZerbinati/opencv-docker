#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "../include/lab3.hpp"

#define windowName "image"
#define trackbarName "Color segmentation threshold"

using namespace lab3;

// the threshold for the color segmentation
// not the best to have this as a global variable. should find a way to update
// it from the trackbar callback and pass it to the mouse callback
int thresholdSlider = 10;

int main(int argc, char** argv) {
    if (!areArgumentsEnough(argc, 2)) {
        std::cout << "\nusage: task4.exe <img_path>\n";
        return 1;
    }

    cv::Mat img = cv::imread(argv[1]);
    if (!isImageValid(img)) {
        std::cout << "\n" << argv[1] << " is not a vaild path for an image\n";
        return 1;
    }

    showImage(img, windowName);
    cv::setMouseCallback(windowName, lab3::onMouseClicked, (void*)&img);
    cv::createTrackbar(trackbarName, windowName, &thresholdSlider, 100,
                       lab3::onTrackbarChanged);

    cv::waitKey(0);
    return 0;
}

void lab3::onMouseClicked(int event, int x, int y, int f, void* userdata) {
    // assert it's a left click
    if (event != cv::EVENT_LBUTTONDOWN) return;

    // extract the img data
    cv::Mat img = *(cv::Mat*)userdata;
    // clone the original image
    cv::Mat imgMask(img.rows, img.cols, CV_8UC1);

    // make sure the 9x9 grid is in the image
    if (y - 1 < 0 || y + 1 >= img.rows || x - 1 < 0 || x + 1 >= img.cols) {
        std::cerr << "neighborhood out of bounds\n";
        return;
    }

    // get the mean values for each channel, in the 9x9 neighbors around (x, y)
    cv::Rect neighborhood(x - 1, y - 1, 3, 3);
    cv::Scalar referenceColor = cv::mean(img(neighborhood));

    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            cv::Vec3b originalPixel = img.at<cv::Vec3b>(r, c);

            // compare the original pixel with the reference color, using a
            // threshold
            if (std::abs(originalPixel[0] - referenceColor[0]) <=
                    thresholdSlider &&
                std::abs(originalPixel[1] - referenceColor[1]) <=
                    thresholdSlider &&
                std::abs(originalPixel[2] - referenceColor[2]) <=
                    thresholdSlider) {
                // similar
                imgMask.at<uchar>(r, c) = 255;
            } else {
                // different
                imgMask.at<uchar>(r, c) = 0;
            }
        }
    }

    // update the shown image (new window)
    showImage(imgMask, "mask");
}

void lab3::onTrackbarChanged(int newValue, void* userdata) {
    thresholdSlider = newValue;
}
