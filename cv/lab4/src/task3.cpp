#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../include/lab4.hpp"
#include "opencv2/ximgproc.hpp"

#define original_img_window "original img"

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

    cv::Mat gray, smooth, canny;

    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // smooth noise away
    cv::blur(gray, smooth, cv::Size(3, 3));

    // edge detection
    cv::Canny(smooth, canny, 50, 200, 3);

    // get hough lines
    cv::Mat hough = img.clone();
    std::vector<cv::Vec3f> lines;
    cv::HoughLines(canny, lines, 1, CV_PI / 180, 140);

    // get image representation in the hough space
    cv::Mat houghSpace;
    cv::ximgproc::FastHoughTransform(canny, houghSpace, 2);

    // filter out lines based on the angle
    std::vector<cv::Vec3f> candidateLeftLines;
    std::vector<cv::Vec3f> candidateRightLines;

    for (size_t i = 0; i < lines.size(); ++i) {
        float theta = lines[i][1];
        if (theta > CV_PI / 180 * 40 && theta < CV_PI / 180 * 60) {
            // around 45 deg
            candidateLeftLines.push_back(lines[i]);
        } else if (theta > CV_PI / 180 * 130 && theta < CV_PI / 180 * 150) {
            // around 135 deg
            candidateRightLines.push_back(lines[i]);
        }
    }

    // take the lines with the most votes (the strongest lines)
    cv::Vec3f leftLine =
        *std::max_element(candidateLeftLines.begin(), candidateLeftLines.end(),
                          [](cv::Vec3f const& lhs, cv::Vec3f const& rhs) {
                              return lhs[2] < rhs[2];
                          });

    cv::Vec3f rightLine = *std::max_element(
        candidateRightLines.begin(), candidateRightLines.end(),
        [](cv::Vec3f const& lhs, cv::Vec3f const& rhs) {
            return lhs[2] < rhs[2];
        });

    // draw the area underneath the two lines as a poligon defined by 3 points
    float rho1 = leftLine[0];
    float rho2 = rightLine[0];
    float theta1 = leftLine[1];
    float theta2 = rightLine[1];

    // p1: first line's intersection with the bottom of the image
    cv::Point p1;
    p1.y = hough.rows - 1;
    p1.x = -(sin(theta1) * p1.y - rho1) / cos(theta1);

    // p2: second line's intersection with the bottom of the image
    cv::Point p2;
    p2.y = hough.rows - 1;
    p2.x = -(sin(theta2) * p2.y - rho2) / cos(theta2);

    // p3: intersection point between two lines (given polar equations)
    cv::Point p3;
    p3.x = -(rho2 / sin(theta2) - rho1 / sin(theta1)) /
           (cos(theta1) / sin(theta1) - cos(theta2) / sin(theta2));
    p3.y = (rho1 + cos(theta1) * -p3.x) / sin(theta1);

    cv::fillPoly(hough, std::vector<cv::Point>{p1, p2, p3},
                 cv::Scalar(0, 0, 255));

    showImage(hough, "Hough Lines");
    showImage(houghSpace, "Hough Space");
    cv::waitKey(0);

    return 0;
}
