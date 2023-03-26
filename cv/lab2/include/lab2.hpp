#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace lab2 {

// task 1
/** Returns true if the number of arguments is at least n. */
bool areArgumentsEnough(int argc, int n);
/** Checks that img is not null. */
bool isImageValid(const cv::Mat& img);
/** Creates a named window and displays the img in it. */
void showImage(const cv::Mat& img, std::string windowName);
/** Converts img to grayscale. */
cv::Mat convertToGreyScale(const cv::Mat& img);

// task 2
/**
 * Applies a min filter to img, using a square kernel of size kernelSize (odd).
 */
cv::Mat minFilter(const cv::Mat& img, int kernelSize);
/**
 * Applies a max filter to img, using a square kernel of size kernelSize (odd).
 */
cv::Mat maxFilter(const cv::Mat& img, int kernelSize);

// task 4
/** Returns an histogram representation for the given img.*/
cv::Mat calcHistImg(const cv::Mat& img,
                    int nBins);  // implementation only supports nBins = 256

}  // namespace lab2
