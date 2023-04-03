#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace lab4 {

// task 1
/** Returns true if the number of arguments is at least n. */
bool areArgumentsEnough(int argc, int n);
/** Checks that img is not null. */
bool isImageValid(const cv::Mat& img);
/** Creates a named window and displays the img in it. */
void showImage(const cv::Mat& img, std::string windowName);
/** Callback to one threshold being changed. */
void onThresholdChanged(int newValue, void* userdata);

}  // namespace lab4
