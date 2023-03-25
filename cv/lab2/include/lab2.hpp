#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace lab2 {

// task 1
bool areArgumentsEnough(int argc, int n);
bool isImageValid(const cv::Mat& img);
void showImage(const cv::Mat& img, std::string windowName);
cv::Mat convertToGreyScale(const cv::Mat& img);

// task 2
cv::Mat minFilter(const cv::Mat& img, int kernelSize);
cv::Mat maxFilter(const cv::Mat& img, int kernelSize);

// task 4
cv::Mat calcHistImg(const cv::Mat& img,
                    int nBins);  // implementation only supports nBins = 256

}  // namespace lab2
