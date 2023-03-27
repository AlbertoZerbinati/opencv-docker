#include "../include/lab2.hpp"

#include <iostream>

bool lab2::areArgumentsEnough(int argc, int n) { return argc >= n; }

bool lab2::isImageValid(const cv::Mat& img) { return img.data != NULL; }

void lab2::showImage(const cv::Mat& img, std::string windowName) {
    cv::namedWindow(windowName);
    cv::imshow(windowName, img);
}

cv::Mat lab2::convertToGreyScale(const cv::Mat& img) {
    cv::Mat ret(img.rows, img.cols, CV_8UC1);  // 1 channel 8 bits
    cv::cvtColor(img, ret, cv::COLOR_RGB2GRAY);
    return ret;
}

cv::Mat lab2::minFilter(const cv::Mat& img, int kernelSize) {
    if (kernelSize % 2 == 0) {
        throw std::invalid_argument("kernelSize must be an odd number.");
    }
    // how many pixels are lost due to kernelSize (no 0-padding is applied)
    int pixelReduction = kernelSize / 2;
    cv::Mat ret(img.rows - (pixelReduction * 2),
                img.cols - (pixelReduction * 2), img.type());

    // (i, j) are coordinates of points used as centers of the kernel. The first
    //   and last pixelReduction pixels won't be centers.
    for (int i = pixelReduction; i < img.rows - pixelReduction; i++) {
        for (int j = pixelReduction; j < img.cols - pixelReduction; j++) {
            // find all the neighbours of (i, j), based on kernelSize
            std::vector<uchar> neighbours;
            for (int k = -pixelReduction; k <= pixelReduction; k++) {
                for (int l = -pixelReduction; l <= pixelReduction; l++) {
                    neighbours.push_back(img.at<uchar>(i + k, j + l));
                }
            }
            // find the min
            uchar min =
                *(std::min_element(neighbours.begin(), neighbours.end()));

            // set the value
            ret.at<uchar>(i - pixelReduction, j - pixelReduction) = min;
        }
    }

    return ret;
}

cv::Mat lab2::maxFilter(const cv::Mat& img, int kernelSize) {
    if (kernelSize % 2 == 0) {
        throw std::invalid_argument("kernelSize must be an odd number.");
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
            // find the max
            uchar max =
                *(std::max_element(neighbours.begin(), neighbours.end()));

            ret.at<uchar>(i - pixelReduction, j - pixelReduction) = max;
        }
    }

    return ret;
}

cv::Mat lab2::calcHistImg(const cv::Mat& img, int nBins) {
    if (nBins != 256) {
        throw std::invalid_argument(
            "nBins different from 256 is not supported");
    }
    // calc hist: each pixel contributes to one bin
    std::vector<float> bins(nBins);
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            uchar pix = img.at<uchar>(i, j);
            bins[pix]++;
        }
    }
    // find the max value and normalize in [0, 1] for convenience
    float max = *(std::max_element(bins.begin(), bins.end()));
    for (int i = 0; i < bins.size(); i++) bins[i] = bins[i] / max;

    // put hist in a Mat
    int histHeight = 512;
    cv::Mat ret(cv::Size(nBins, histHeight), CV_8UC1);
    for (int i = 0; i < ret.rows; i++) {
        for (int j = 0; j < ret.cols; j++) {
            if (bins[j] > float(i) / histHeight) {
                // black if the bin column is high enough
                ret.at<uchar>(i, j) = 0;
            } else {
                // white otherwise
                ret.at<uchar>(i, j) = 255;
            }
        }
    }
    // the above Mat is flipped, so deal with that
    cv::flip(ret, ret, 0);

    return ret;
}