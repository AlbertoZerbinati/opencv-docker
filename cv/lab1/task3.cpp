#include <opencv2/highgui.hpp>
#include <iostream>

void setChannelToZero(int channel, cv::Mat& img) {
    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j) {
            img.at<cv::Vec3b>(i, j)[channel] = 0; // how to at compile time the <type> ???
        }
    }

    // alternative: return a different Mat, without altering the original
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "\nusage: intro.exe <img_path>\n\n";
        return 1;
    }
    cv::Mat img = cv::imread(argv[1]);

    if (img.data == NULL) {
        std::cout << "\n" << argv[1] << " is not a vaild path for an image\n\n";
        return 1;
    }

    int nChannels = img.channels();
    std::cout << "\nthe image has " << nChannels << " channels\n";

    // TODO

    // setChannelToZero(0, img); // image becomes yellow
    // setChannelToZero(1, img); // image becomes magenta
    setChannelToZero(2, img); // image becomes cyan

    cv::namedWindow("Lab 1");
    cv::imshow("Lab 1", img);
    
    char c = cv::waitKey(0);
    std::cout << "\nyou pressed the " << c << " key\n";

    return 0;
}
