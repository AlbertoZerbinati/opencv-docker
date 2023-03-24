#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

cv::Mat minFilter(const cv::Mat& img, int kernelSize);
cv::Mat maxFilter(const cv::Mat& img, int kernelSize);
