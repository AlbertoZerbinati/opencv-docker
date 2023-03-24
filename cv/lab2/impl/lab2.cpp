#include "../include/lab2.hpp"

#include <iostream>

cv::Mat minFilter(const cv::Mat& img, int kernelSize) {
    if (kernelSize % 2 == 0) {
        std::invalid_argument("kernelSize must be an odd number.");
    }
    int pixelReduction = kernelSize / 2;
    cv::Mat ret(img.rows - (pixelReduction * 2),
                img.cols - (pixelReduction * 2), img.type());

    for (int i = pixelReduction; i < img.rows - pixelReduction; i++) {
        for (int j = pixelReduction; j < img.cols - pixelReduction; j++) {
            std::vector<uchar> neighbours;
            for (int k = -pixelReduction; k <= pixelReduction; k++) {
                for (int l = -pixelReduction; l <= pixelReduction; l++) {
                    neighbours.push_back(img.at<uchar>(i + k, j + l));
                }
            }

            uchar min =
                *(std::min_element(neighbours.begin(), neighbours.end()));

            ret.at<uchar>(i - pixelReduction, j - pixelReduction) = min;
        }
    }

    return ret;
}

cv::Mat maxFilter(const cv::Mat& img, int kernelSize) {
    if (kernelSize % 2 == 0) {
        std::invalid_argument("kernelSize must be an odd number.");
    }
    int pixelReduction = kernelSize / 2;
    cv::Mat ret(img.rows - (pixelReduction * 2),
                img.cols - (pixelReduction * 2), img.type());

    for (int i = pixelReduction; i < img.rows - pixelReduction; i++) {
        for (int j = pixelReduction; j < img.cols - pixelReduction; j++) {
            std::vector<uchar> neighbours;
            for (int k = -pixelReduction; k <= pixelReduction; k++) {
                for (int l = -pixelReduction; l <= pixelReduction; l++) {
                    neighbours.push_back(img.at<uchar>(i + k, j + l));
                }
            }
            uchar max =
                *(std::max_element(neighbours.begin(), neighbours.end()));

            ret.at<uchar>(i - pixelReduction, j - pixelReduction) = max;
        }
    }

    return ret;
}
