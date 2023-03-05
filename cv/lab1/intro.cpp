#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1]);
    cv::namedWindow("Lab 1");
    cv::imshow("Lab 1", img);
    cv::waitKey(0);

    return 0;
}
