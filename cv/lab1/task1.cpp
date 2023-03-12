#include <iostream>
#include <opencv2/highgui.hpp>

bool areArgumentsEnough(int argc, int n);
bool isImageValid(cv::Mat& img);

int main(int argc, char** argv) {
    if (!areArgumentsEnough(argc, 2)) {
        std::cout << "\nusage: intro.exe <img_path>\n";
        return 1;
    }

    cv::Mat img = cv::imread(argv[1]);
    if (!isImageValid(img)) {
        std::cout << "\n" << argv[1] << " is not a vaild path for an image\n";
        return 1;
    }

    cv::namedWindow("Lab 1");
    cv::imshow("Lab 1", img);
    cv::waitKey(0);

    return 0;
}

bool areArgumentsEnough(int argc, int n) { return argc >= n; }
bool isImageValid(cv::Mat& img) { return img.data != NULL; }
