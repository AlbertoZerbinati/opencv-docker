#include <iostream>
#include <opencv2/highgui.hpp>

bool areArgumentsEnough(int argc, int n);
bool areChannelsEnough(int channels, int n);
bool isImageValid(cv::Mat& img);
void setChannelToZero(int channel, cv::Mat& img);

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

    int nChannels = img.channels();
    std::cout << "\nthe image has " << nChannels << " channels\n";
    if (!areChannelsEnough(nChannels, 3)) {
        std::cout << "\n image has too few channels, needs 3 instead they are "
                  << nChannels << "\n";
        return 1;
    }

    // setChannelToZero(0, img); // image becomes yellow
    // setChannelToZero(1, img); // image becomes magenta
    setChannelToZero(2, img);  // image becomes cyan

    cv::namedWindow("Lab 1");
    cv::imshow("Lab 1", img);

    char c = cv::waitKey(0);
    std::cout << "\nyou pressed the " << c << " key\n";

    return 0;
}

bool areArgumentsEnough(int argc, int n) { return argc >= n; }
bool areChannelsEnough(int channels, int n) { return channels >= n; }
bool isImageValid(cv::Mat& img) { return img.data != NULL; }
void setChannelToZero(int channel, cv::Mat& img) {
    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j) {
            // modify the input Mat reference
            img.at<cv::Vec3b>(i, j)[channel] = 0;
        }
    }
}
