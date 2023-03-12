#include <iostream>
#include <opencv2/highgui.hpp>

void fillWithVerticalGradient(cv::Mat& img);
void fillWithHorizontalGradient(cv::Mat& img);
void createChessboardPattern(int squareSize, cv::Mat& img);

int main(int argc, char** argv) {
    cv::Mat imgVerticalGradient = cv::Mat(256, 256, CV_8UC1);
    cv::Mat imgHorizontalGradient = cv::Mat(256, 256, CV_8UC1);

    fillWithVerticalGradient(imgVerticalGradient);
    fillWithHorizontalGradient(imgHorizontalGradient);

    cv::Mat chessboard1 = cv::Mat(300, 300, CV_8UC1);
    cv::Mat chessboard2 = cv::Mat(300, 300, CV_8UC1);

    createChessboardPattern(20, chessboard1);
    createChessboardPattern(50, chessboard2);

    cv::namedWindow("Lab 1.1");
    cv::namedWindow("Lab 1.2");
    cv::namedWindow("Lab 1.3");
    cv::namedWindow("Lab 1.4");
    cv::imshow("Lab 1.1", imgVerticalGradient);
    cv::imshow("Lab 1.2", imgHorizontalGradient);
    cv::imshow("Lab 1.3", chessboard1);
    cv::imshow("Lab 1.4", chessboard2);

    char c = cv::waitKey(0);
    std::cout << "\nyou pressed the " << c << " key\n";

    return 0;
}

void fillWithVerticalGradient(cv::Mat& img) {
    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j) {
            img.at<unsigned char>(i, j) = i;
        }
    }
}

void fillWithHorizontalGradient(cv::Mat& img) {
    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j) {
            img.at<unsigned char>(i, j) = j;
        }
    }
}

void createChessboardPattern(int squareSize, cv::Mat& img) {
    unsigned char color = 255;
    unsigned char initialColor = color;
    for (int i = 0; i < img.rows; ++i) {
        color = initialColor;
        for (int j = 0; j < img.cols; ++j) {
            if (j != 0 && j % squareSize == 0) {
                if (color == 255)
                    color = 0;
                else
                    color = 255;
            }
            img.at<unsigned char>(i, j) = color;
        }
        if (i != 0 && i % squareSize == 0)
            if (initialColor == 255)
                initialColor = 0;
            else
                initialColor = 255;
    }
}
