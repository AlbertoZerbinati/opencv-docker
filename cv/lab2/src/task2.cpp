#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../include/lab2.hpp"

using namespace lab2;

int main(int argc, char** argv) {
    if (!areArgumentsEnough(argc, 3)) {
        std::cout << "\nusage: task2.exe <img_path> <kernel_size>\n";
        return 1;
    }

    // IMREAD_UNCHANGED flag is needed to read the image correctly with 1
    // channel. Indeed by default imread uses IMREAD_COLOR, which uses 3
    // channels. It uses the single channel value copied to the other two
    // channels. Then if I have a Vec3b pixel and read it as uchar, I get
    // impredictable behavior. Same if I had a uchar and read it as Vec3b.
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
    if (!isImageValid(img)) {
        std::cout << "\n" << argv[1] << " is not a vaild path for an image\n";
        return 1;
    }

    showImage(img, "b&w img");

    int kernelSize = atoi(argv[2]);
    cv::Mat minFiltered = minFilter(img, kernelSize);
    cv::Mat maxFiltered = maxFilter(img, kernelSize);

    showImage(minFiltered, "min filter " + std::to_string(kernelSize));
    showImage(maxFiltered, "max filter " + std::to_string(kernelSize));

    cv::waitKey(0);
    return 0;
}
