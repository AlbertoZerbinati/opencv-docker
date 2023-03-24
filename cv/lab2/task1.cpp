#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

bool areArgumentsEnough(int argc, int n);
bool isImageValid(const cv::Mat& img);
void showImage(const cv::Mat& img, std::string windowName);
cv::Mat convertToGreyScale(const cv::Mat& img);

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

    showImage(img, "original img");

    cv::Mat greyScaledImg = convertToGreyScale(img);

    cv::imwrite("asset/grescale_image.jpg", greyScaledImg);

    return 0;
}

bool areArgumentsEnough(int argc, int n) { return argc >= n; }
bool isImageValid(const cv::Mat& img) { return img.data != NULL; }
void showImage(const cv::Mat& img, std::string windowName) {
    cv::namedWindow(windowName);
    cv::imshow(windowName, img);
    cv::waitKey(0);
}
cv::Mat convertToGreyScale(const cv::Mat& img) {
    cv::Mat ret(img.rows, img.cols, CV_8UC1); // 1 channel 8 bits
    cv::cvtColor(img, ret, cv::COLOR_RGB2GRAY);
    return ret;
}
