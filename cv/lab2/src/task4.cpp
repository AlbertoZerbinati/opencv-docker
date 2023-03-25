#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../include/lab2.hpp"

using namespace lab2;

int main(int argc, char** argv) {
    cv::Mat src = cv::imread(argv[1]);
    cv::Mat img;
    cvtColor(src, img, cv::COLOR_BGR2GRAY);

    // my impl
    // cv::Mat histImg = lab2::calcHistImg(greyScaledImg, 256);

    int nBins = 256;
    float range[] = {0, 256};
    const float* histRange[] = {range};
    bool uniform = true, accumulate = false;
    // calculate histogram using calcHist
    cv::Mat hist;
    calcHist(&img, 1, 0, cv::Mat(), hist, 1, &nBins, histRange, uniform,
             accumulate);

    // create image to display the histogram
    int histWidth = 512, histHeight = 400;
    int binWidth = cvRound((double)histWidth / nBins);
    cv::Mat histImg(histHeight, histWidth, CV_8UC3, cv::Scalar(0, 0, 0));

    // normalize the histogram
    normalize(hist, hist, 0, histImg.rows, cv::NORM_MINMAX, -1, cv::Mat());

    // display the histogram
    for (int i = 1; i < nBins; ++i) {
        cv::line(
            histImg,
            cv::Point(binWidth * (i - 1),
                      histHeight - cvRound(hist.at<float>(i - 1))),
            cv::Point(binWidth * (i), histHeight - cvRound(hist.at<float>(i))),
            cv::Scalar(255, 255, 255), 1, 8, 0);
    }
    showImage(histImg, "hist");

    cv::waitKey(0);
    return 0;
}
