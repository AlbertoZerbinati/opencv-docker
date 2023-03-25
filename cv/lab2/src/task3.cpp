#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../include/lab2.hpp"

using namespace lab2;

int main(int argc, char** argv) {
    if (!areArgumentsEnough(argc, 2)) {
        std::cout << "\nusage: task3.exe <img_path>\n";
        return 1;
    }

    cv::Mat img = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
    if (!isImageValid(img)) {
        std::cout << "\n" << argv[1] << " is not a vaild path for an image\n";
        return 1;
    }

    showImage(img, "b&w img");

    int kernelSize = 7;
    int kernelSigma = 5;
    cv::Mat medianFiltered(img.rows, img.cols, img.type());
    cv::medianBlur(img, medianFiltered, kernelSize);

    cv::Mat gaussianFiltered(img.rows, img.cols, img.type());
    cv::GaussianBlur(img, gaussianFiltered, cv::Size(kernelSize, kernelSize),
                     kernelSigma);

    showImage(medianFiltered, "median filter " + std::to_string(kernelSize));
    showImage(gaussianFiltered, "gaussian filter " +
                                    std::to_string(kernelSize) + ", " +
                                    std::to_string(kernelSigma));

    cv::waitKey(0);
    return 0;
}
