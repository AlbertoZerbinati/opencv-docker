#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../include/lab4.hpp"

#define original_img_window "original img"
#define INTENSITY_THRESHOLD 220
#define ROAD_THRESHOLD 5
#define PIXEL_THRESHOLD 17

using namespace lab4;

int main(int argc, char** argv) {
    if (!areArgumentsEnough(argc, 2)) {
        std::cout << "\nusage: task2.exe <img_path>\n";
        return 1;
    }

    cv::Mat img = cv::imread(argv[1]);
    if (!isImageValid(img)) {
        std::cout << "\n" << argv[1] << " is not a vaild path for an image\n";
        return 1;
    }

    cv::Mat gray, smooth, result, grad_x, grad_y;

    // smooth the image
    cv::GaussianBlur(img, smooth, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

    // convert to grayscale
    cv::cvtColor(smooth, gray, cv::COLOR_BGR2GRAY);

    // apply two diagonal sobel filters
    cv::Sobel(gray, grad_y, CV_16S, 0, 1, 1, cv::BORDER_DEFAULT);
    cv::Sobel(gray, grad_x, CV_16S, 1, 0, 1, cv::BORDER_DEFAULT);

    // converting back to CV_8U
    convertScaleAbs(grad_x, grad_x);
    convertScaleAbs(grad_y, grad_y);

    // add the two images
    cv::addWeighted(grad_x, 0.5, grad_y, 0.5, 0, result);

    // apply binary threshold
    cv::threshold(result, result, INTENSITY_THRESHOLD, 255, cv::THRESH_BINARY);

    cv::Scalar roadColor(108, 113, 108);
    for (int r = 0; r < result.rows; r++) {
        for (int c = 0; c < result.cols; c++) {
            // if white, see if it can be removed
            if (result.at<uchar>(r, c) == 255) {
                // select original img neighbours
                std::vector<cv::Vec3b> neighbours;
                for (int k = -1; k <= 1; k++) {
                    for (int l = -1; l <= 1; l++) {
                        if (r + k >= 0 && r + k < result.rows && c + l >= 0 &&
                            c + l < result.cols) {
                            neighbours.push_back(
                                img.at<cv::Vec3b>(r + k, c + l));
                        }
                    }
                }

                // see if there's a neighbour with the road's color or with
                // another grey color (all channels are similar)
                bool shouldRemove = true;
                for (int i = 0; i < neighbours.size(); i++) {
                    if (std::abs(neighbours[i][0] - roadColor[0]) <
                            PIXEL_THRESHOLD &&
                        std::abs(neighbours[i][1] - roadColor[1]) <
                            PIXEL_THRESHOLD &&
                        std::abs(neighbours[i][2] - roadColor[2]) <
                            PIXEL_THRESHOLD) {
                        shouldRemove = false;
                    }
                    if (std::abs(neighbours[i][0] - neighbours[i][1]) <
                            ROAD_THRESHOLD &&
                        std::abs(neighbours[i][1] - neighbours[i][2]) <
                            ROAD_THRESHOLD) {
                        shouldRemove = false;
                    }
                }
                if (shouldRemove) {
                    result.at<uchar>(r, c) = 0;
                }
            }
        }
    }

    showImage(img, original_img_window);
    showImage(result, "road edges");

    cv::waitKey(0);

    return 0;
}