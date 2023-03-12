#include <iostream>
#include <opencv2/highgui.hpp>

bool areArgumentsEnough(int argc, int n);
bool areChannelsEnough(int channels, int n);
bool isImageValid(cv::Mat& img);
cv::Mat createSingleChannelImage(int channel, const cv::Mat& img);

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

    cv::Mat firstChannelImage = createSingleChannelImage(0, img);
    cv::Mat secondChannelImage = createSingleChannelImage(1, img);
    cv::Mat thirdChannelImage = createSingleChannelImage(2, img);

    cv::namedWindow("Lab 1");
    cv::namedWindow("Lab 1.1");
    cv::namedWindow("Lab 1.2");
    cv::namedWindow("Lab 1.3");
    cv::imshow("Lab 1", img);
    cv::imshow("Lab 1.1", firstChannelImage);
    cv::imshow("Lab 1.2", secondChannelImage);
    cv::imshow("Lab 1.3", thirdChannelImage);

    char c = cv::waitKey(0);
    std::cout << "\nyou pressed the " << c << " key\n";

    return 0;
}

bool areArgumentsEnough(int argc, int n) { return argc >= n; }
bool areChannelsEnough(int channels, int n) { return channels >= n; }
bool isImageValid(cv::Mat& img) { return img.data != NULL; }
cv::Mat createSingleChannelImage(int channel, const cv::Mat& img) {
    cv::Mat ret = cv::Mat(img.rows, img.cols, img.type());

    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j) {
            ret.at<cv::Vec3b>(i, j)[channel] = img.at<cv::Vec3b>(i, j)[channel];
        }
    }

    return ret;
}
