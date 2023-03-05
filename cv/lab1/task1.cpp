#include <opencv2/highgui.hpp>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "\nusage: intro.exe <img_path>\n" << std::endl;
        return 1;
    }
    cv::Mat img = cv::imread(argv[1]);

    if (img.data == NULL) {
        std::cout << "\n" << argv[1] << " is not a vaild pathfor an image\n" << std::endl;
        return 1;
    }
    cv::namedWindow("Lab 1");
    cv::imshow("Lab 1", img);
    cv::waitKey(0);

    return 0;
}
