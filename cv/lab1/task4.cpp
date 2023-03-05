#include <opencv2/highgui.hpp>
#include <iostream>

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

    cv::namedWindow("Lab 1");
    cv::imshow("Lab 1", img);
    
    char c = cv::waitKey(0);
    std::cout << "\nyou pressed the " << c << " key\n";

    return 0;
}
