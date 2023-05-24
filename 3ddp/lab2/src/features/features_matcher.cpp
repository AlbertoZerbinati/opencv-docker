#include "features/features_matcher.h"

#include <iostream>
#include <map>

FeatureMatcher::FeatureMatcher(cv::Mat intrinsics_matrix, cv::Mat dist_coeffs,
                               double focal_scale) {
    intrinsics_matrix_ = intrinsics_matrix.clone();
    dist_coeffs_ = dist_coeffs.clone();
    new_intrinsics_matrix_ = intrinsics_matrix.clone();
    new_intrinsics_matrix_.at<double>(0, 0) *= focal_scale;
    new_intrinsics_matrix_.at<double>(1, 1) *= focal_scale;
}

void FeatureMatcher::extractFeatures() {
    features_.resize(images_names_.size());
    descriptors_.resize(images_names_.size());
    feats_colors_.resize(images_names_.size());

    auto detector = cv::ORB::create(10000, 1.2, 8);

    for (int i = 0; i < images_names_.size(); i++) {
        std::cout << "Computing descriptors for image " << i << std::endl;
        cv::Mat img = readUndistortedImage(images_names_[i]);

        //////////////////////////// Code to be completed (1/6)
        ////////////////////////////////////
        // Extract salient points + descriptors from i-th image, and store them
        // into features_[i] and descriptors_[i] vector, respectively.
        // Extract also the color (i.e., the cv::Vec3b information) of each
        // feature, and store it into feats_colors_[i] vector.
        /////////////////////////////////////////////////////////////////////////////////////////

        //Use of the detector to obtain keypoints and the corresponding descriptors
        //Then save those in the vectors as required
        detector->detectAndCompute(img, cv::Mat(), features_[i],  descriptors_[i]);

        //For each keypoint extract the corresponding color and add it to a vector which is then stored as required
        std::vector<cv::Vec3b> kp_colors;
        for (cv::KeyPoint kp : features_[i]) {
            //Retrieve the coordinates for the keypoint
            cv::Point2f kp_point = kp.pt;
            //obtain the color in the image
            cv::Vec3b kp_color = img.at<cv::Vec3b>(kp_point);
            kp_colors.push_back(kp_color);
        }
        //save the color values
        feats_colors_[i] = kp_colors;

        /////////////////////////////////////////////////////////////////////////////////////////
    }
}

void FeatureMatcher::exhaustiveMatching() {
    for (int i = 0; i < images_names_.size() - 1; i++) {
        for (int j = i + 1; j < images_names_.size(); j++) {
            std::cout << "Matching image " << i << " with image " << j
                      << std::endl;
            std::vector<cv::DMatch> matches, inlier_matches;

            //////////////////////////// Code to be completed (2/6)
            ////////////////////////////////////
            // Match descriptors between image i and image j, and perform
            // geometric validation, possibly discarding the outliers (remember
            // that features have been extracted from undistorted images that
            // now has new_intrinsics_matrix_ as K matrix and no distortions).
            // As geometric models, use both the Essential matrix and the
            // Homography
            // matrix, both by setting new_intrinsics_matrix_ as K matrix. As
            // threshold in the functions to estimate both models, you may
            // use 1.0 or similar. Store inlier matches into the inlier_matches
            // vector.
            // Do not set matches between two images if the amount of inliers
            // matches (i.e., geomatrically verified matches) is small
            // (say <= 5 matches).
            // In case of success, set the matches with the  function:
            // setMatches( i, j, inlier_matches);
            /////////////////////////////////////////////////////////////////////////////////////////

            // Matcher initialization: if AKAZE or SIFT matcher is used then
            // NORM_L2 should be used, 
            // Since ORB was the choice NORM_HAMMING is used as normType, as suggested in OpenCV
            // doc.
            cv::Ptr<cv::BFMatcher> matcher =
                cv::BFMatcher::create(cv::NORM_HAMMING);

            // Matches are computed and their relative points are stored in
            // their relative data structure for both images (i and j).
            matcher->match(descriptors_[i], descriptors_[j], matches);

            std::vector<cv::Point2f> src_points, dst_points;
            for (cv::DMatch match : matches) {
                src_points.push_back(features_[i][match.queryIdx].pt);
                dst_points.push_back(features_[j][match.trainIdx].pt);
            }

            // Definition of the masks used to compute the inliers.
            cv::Mat inlier_mask;
            std::vector<cv::DMatch> inliers_H;
            std::vector<cv::DMatch> inliers_E;

            // Computation of the Homography using RANSAC and corresponding inliers
            cv::findHomography(src_points, dst_points, cv::RANSAC, 1.0, inlier_mask);

            for (int id_match = 0; id_match < src_points.size(); id_match++) {
                if (inlier_mask.at<uchar>(id_match)) {
                    inliers_H.push_back(matches[id_match]);
                }
            }

            // Computation of the Essential Matrix using RANSAC and corresponding inliers
            cv::findEssentialMat(src_points, dst_points, new_intrinsics_matrix_,
                                 cv::RANSAC, 0.999, 1.0, inlier_mask);

            for (int id_match = 0; id_match < src_points.size(); id_match++) {
                if (inlier_mask.at<uchar>(id_match)) {
                    inliers_E.push_back(matches[id_match]);
                }
            }

            int num_inliers_E = inliers_E.size();
            int num_inliers_H = inliers_H.size();
            // Choose the best result, in terms of number of inliers, as final
            // result, using 5 as threshold.
            if (num_inliers_E >= num_inliers_H &&  num_inliers_E > 5) {
                //Save only the inliers as matches
                std::copy(inliers_E.begin(), inliers_E.end(),
                          std::back_inserter(inlier_matches));

            } else if (num_inliers_H > 5) {
                //Save only the inliers as matches
                std::copy(inliers_H.begin(), inliers_H.end(),
                          std::back_inserter(inlier_matches));
            } else
            // we don't have more than 5 inliers neither for E nor H
            {
                inlier_matches = {};
            }

            setMatches(i, j, inlier_matches);

            /////////////////////////////////////////////////////////////////////////////////////////
        }
    }
}

void FeatureMatcher::writeToFile(const std::string &filename,
                                 bool normalize_points) const {
    FILE *fptr = fopen(filename.c_str(), "w");

    if (fptr == NULL) {
        std::cerr << "Error: unable to open file " << filename;
        return;
    };

    fprintf(fptr, "%d %d %d\n", num_poses_, num_points_, num_observations_);

    double *tmp_observations;
    cv::Mat dst_pts;
    if (normalize_points) {
        cv::Mat src_obs(num_observations_, 1,
                        cv::traits::Type<cv::Vec2d>::value,
                        const_cast<double *>(observations_.data()));
        cv::undistortPoints(src_obs, dst_pts, new_intrinsics_matrix_,
                            cv::Mat());
        tmp_observations = reinterpret_cast<double *>(dst_pts.data);
    } else {
        tmp_observations = const_cast<double *>(observations_.data());
    }

    for (int i = 0; i < num_observations_; ++i) {
        fprintf(fptr, "%d %d", pose_index_[i], point_index_[i]);
        for (int j = 0; j < 2; ++j) {
            fprintf(fptr, " %g", tmp_observations[2 * i + j]);
        }
        fprintf(fptr, "\n");
    }

    if (colors_.size() == 3 * num_points_) {
        for (int i = 0; i < num_points_; ++i)
            fprintf(fptr, "%d %d %d\n", colors_[i * 3], colors_[i * 3 + 1],
                    colors_[i * 3 + 2]);
    }

    fclose(fptr);
}

void FeatureMatcher::testMatches(double scale) {
    // For each pose, prepare a map that reports the pairs [point index,
    // observation index].
    std::vector<std::map<int, int>> cam_observation(num_poses_);
    for (int i_obs = 0; i_obs < num_observations_; i_obs++) {
        int i_cam = pose_index_[i_obs], i_pt = point_index_[i_obs];
        cam_observation[i_cam][i_pt] = i_obs;
    }

    for (int r = 0; r < num_poses_; r++) {
        for (int c = r + 1; c < num_poses_; c++) {
            int num_mathces = 0;
            std::vector<cv::DMatch> matches;
            std::vector<cv::KeyPoint> features0, features1;
            for (auto const &co_iter : cam_observation[r]) {
                if (cam_observation[c].find(co_iter.first) !=
                    cam_observation[c].end()) {
                    features0.emplace_back(
                        observations_[2 * co_iter.second],
                        observations_[2 * co_iter.second + 1], 0.0);
                    features1.emplace_back(
                        observations_[2 * cam_observation[c][co_iter.first]],
                        observations_[2 * cam_observation[c][co_iter.first] +
                                      1],
                        0.0);
                    matches.emplace_back(num_mathces, num_mathces, 0);
                    num_mathces++;
                }
            }
            cv::Mat img0 = readUndistortedImage(images_names_[r]),
                    img1 = readUndistortedImage(images_names_[c]), dbg_img;

            cv::drawMatches(img0, features0, img1, features1, matches, dbg_img);
            cv::resize(dbg_img, dbg_img, cv::Size(), scale, scale);
            cv::imshow("", dbg_img);
            if (cv::waitKey() == 27) return;
        }
    }
}

void FeatureMatcher::reset() {
    point_index_.clear();
    pose_index_.clear();
    observations_.clear();
    colors_.clear();

    num_poses_ = num_points_ = num_observations_ = 0;
}

cv::Mat FeatureMatcher::readUndistortedImage(const std::string &filename) {
    cv::Mat img = cv::imread(filename), und_img, dbg_img;
    cv::undistort(img, und_img, intrinsics_matrix_, dist_coeffs_,
                  new_intrinsics_matrix_);

    return und_img;
}

void FeatureMatcher::setMatches(int pos0_id, int pos1_id,
                                const std::vector<cv::DMatch> &matches) {
    const auto &features0 = features_[pos0_id];
    const auto &features1 = features_[pos1_id];

    auto pos_iter0 = pose_id_map_.find(pos0_id),
         pos_iter1 = pose_id_map_.find(pos1_id);

    // Already included position?
    if (pos_iter0 == pose_id_map_.end()) {
        pose_id_map_[pos0_id] = num_poses_;
        pos0_id = num_poses_++;
    } else
        pos0_id = pose_id_map_[pos0_id];

    // Already included position?
    if (pos_iter1 == pose_id_map_.end()) {
        pose_id_map_[pos1_id] = num_poses_;
        pos1_id = num_poses_++;
    } else
        pos1_id = pose_id_map_[pos1_id];

    for (auto &match : matches) {
        // Already included observations?
        uint64_t obs_id0 = poseFeatPairID(pos0_id, match.queryIdx),
                 obs_id1 = poseFeatPairID(pos1_id, match.trainIdx);
        auto pt_iter0 = point_id_map_.find(obs_id0),
             pt_iter1 = point_id_map_.find(obs_id1);
        // New point
        if (pt_iter0 == point_id_map_.end() &&
            pt_iter1 == point_id_map_.end()) {
            int pt_idx = num_points_++;
            point_id_map_[obs_id0] = point_id_map_[obs_id1] = pt_idx;

            point_index_.push_back(pt_idx);
            point_index_.push_back(pt_idx);
            pose_index_.push_back(pos0_id);
            pose_index_.push_back(pos1_id);
            observations_.push_back(features0[match.queryIdx].pt.x);
            observations_.push_back(features0[match.queryIdx].pt.y);
            observations_.push_back(features1[match.trainIdx].pt.x);
            observations_.push_back(features1[match.trainIdx].pt.y);

            // Average color between two corresponding features (suboptimal
            // since we shouls also consider the other observations of the same
            // point in the other images).
            cv::Vec3f color =
                (cv::Vec3f(feats_colors_[pos0_id][match.queryIdx]) +
                 cv::Vec3f(feats_colors_[pos1_id][match.trainIdx])) /
                2;

            colors_.push_back(cvRound(color[2]));
            colors_.push_back(cvRound(color[1]));
            colors_.push_back(cvRound(color[0]));

            num_observations_++;
            num_observations_++;
        }
        // New observation.
        else if (pt_iter0 == point_id_map_.end()) {
            int pt_idx = point_id_map_[obs_id1];
            point_id_map_[obs_id0] = pt_idx;

            point_index_.push_back(pt_idx);
            pose_index_.push_back(pos0_id);
            observations_.push_back(features0[match.queryIdx].pt.x);
            observations_.push_back(features0[match.queryIdx].pt.y);
            num_observations_++;
        } else if (pt_iter1 == point_id_map_.end()) {
            int pt_idx = point_id_map_[obs_id0];
            point_id_map_[obs_id1] = pt_idx;

            point_index_.push_back(pt_idx);
            pose_index_.push_back(pos1_id);
            observations_.push_back(features1[match.trainIdx].pt.x);
            observations_.push_back(features1[match.trainIdx].pt.y);
            num_observations_++;
        }
        //    else if( pt_iter0->second != pt_iter1->second )
        //    {
        //      std::cerr<<"Shared observations does not share 3D
        //      point!"<<std::endl;
        //    }
    }
}
