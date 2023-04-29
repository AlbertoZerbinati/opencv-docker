#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

bool readFileNamesFromFolder(const std::string &input_folder_name,
                             std::vector<std::string> &names);
bool loadCameraParams(const std::string &file_name, cv::Size &image_size,
                      cv::Mat &camera_matrix, cv::Mat &dist_coeffs);
