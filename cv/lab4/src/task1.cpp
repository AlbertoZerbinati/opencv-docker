#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../include/lab4.hpp"

#define original_img_window "original img"

using namespace lab4;

struct Params {
    cv::Mat* img;
    int* threshold1;
    int* threshold2;
    int* apertureSize;
    bool* L2gradient;
};

int main(int argc, char** argv) {
    if (!areArgumentsEnough(argc, 2)) {
        std::cout << "\nusage: task1.exe <img_path>\n";
        return 1;
    }

    cv::Mat img = cv::imread(argv[1]);
    if (!isImageValid(img)) {
        std::cout << "\n" << argv[1] << " is not a vaild path for an image\n";
        return 1;
    }

    showImage(img, original_img_window);

    // define params of the Canny filter
    int threshold1 = 0;
    int threshold2 = 0;
    int apertureSize = 3;
    bool L2gradient = false;

    Params params;
    params.img = &img;
    params.threshold1 = &threshold1;
    params.threshold2 = &threshold2;
    params.apertureSize = &apertureSize;
    params.L2gradient = &L2gradient;

    // create four trackbars to change the params
    cv::createTrackbar("threshold 1", original_img_window, &threshold1, 1000,
                       lab4::onThresholdChanged, (void*)&params);
    cv::createTrackbar("threshold 2", original_img_window, &threshold2, 1000,
                       lab4::onThresholdChanged, (void*)&params);
    cv::createTrackbar("aperture size", original_img_window, &apertureSize, 10,
                       lab4::onThresholdChanged, (void*)&params);
    cv::createTrackbar("L2 gradient", original_img_window, (int*)&L2gradient, 1,
                       lab4::onThresholdChanged, (void*)&params);

    cv::Mat cannyFiltered;
    cv::Canny(img, cannyFiltered, threshold1, threshold2, apertureSize,
              L2gradient);
    showImage(cannyFiltered, "canny");

    cv::waitKey(0);
    return 0;
}

void lab4::onThresholdChanged(int newValue, void* userdata) {
    // retrieve params
    Params params = *(Params*)userdata;

    // clone the original img
    cv::Mat original = *params.img;
    cv::Mat img = original.clone();

    // check whether params are valid
    bool shouldShowErrorMessage = false;
    if (*params.apertureSize % 2 == 0 || *params.apertureSize < 3 ||
        *params.apertureSize > 7) {
        *params.apertureSize = 3;
        shouldShowErrorMessage = true;
        std::cerr << "Aperture size should be odd between 3 and 7" << std::endl;
    }

    // apply the filter
    cv::Mat cannyFiltered{CV_8UC3};
    cv::Canny(img, cannyFiltered, *params.threshold1, *params.threshold2,
              *params.apertureSize, *params.L2gradient);

    // put error message in the original img window if there are invalid params
    if (shouldShowErrorMessage) {
        std::string text = "Aperture size should be odd between 3 and 7";
        cv::Point p{img.size().width / 2 - 250, img.size().height / 2};
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double fontSize = 0.8;
        cv::Scalar fontColor{50, 0, 255};
        int thickness = 2;
        cv::putText(img, text, p, font, fontSize, fontColor, thickness);
    }
    showImage(cannyFiltered, "canny");
    showImage(img, original_img_window);
}
