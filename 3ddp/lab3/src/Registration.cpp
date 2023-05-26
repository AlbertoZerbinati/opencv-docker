#include "Registration.h"

struct PointDistance {
    ///////////////////////////////////////////////////////////////////////////
    // This class should include an auto-differentiable cost function.
    // To rotate a point given an axis-angle rotation, use
    // the Ceres function:
    // AngleAxisRotatePoint(...) (see ceres/rotation.h)
    // Similarly to the Bundle Adjustment case initialize the struct variables
    // with the source and  the target point. You have to optimize only the
    // 6-dimensional array (rx, ry, rz, tx ,ty, tz). WARNING: When dealing with
    // the AutoDiffCostFunction template parameters, pay attention to the order
    // of the template parameters
    ///////////////////////////////////////////////////////////////////////////
};

Registration::Registration(std::string cloud_source_filename,
                           std::string cloud_target_filename) {
    open3d::io::ReadPointCloud(cloud_source_filename, source_);
    open3d::io::ReadPointCloud(cloud_target_filename, target_);
    Eigen::Vector3d gray_color;
    source_for_icp_ = source_;
}

Registration::Registration(open3d::geometry::PointCloud cloud_source,
                           open3d::geometry::PointCloud cloud_target) {
    source_ = cloud_source;
    target_ = cloud_target;
    source_for_icp_ = source_;
}

void Registration::draw_registration_result() {
    // clone input
    open3d::geometry::PointCloud source_clone = source_;
    open3d::geometry::PointCloud target_clone = target_;

    // different color
    Eigen::Vector3d color_s;
    Eigen::Vector3d color_t;
    color_s << 1, 0.706, 0;
    color_t << 0, 0.651, 0.929;

    target_clone.PaintUniformColor(color_t);
    source_clone.PaintUniformColor(color_s);
    source_clone.Transform(transformation_);

    auto src_pointer =
        std::make_shared<open3d::geometry::PointCloud>(source_clone);
    auto target_pointer =
        std::make_shared<open3d::geometry::PointCloud>(target_clone);
    open3d::visualization::DrawGeometries({src_pointer, target_pointer});
    return;
}

void Registration::execute_icp_registration(double threshold, int max_iteration,
                                            double relative_rmse,
                                            std::string mode) {
    ///////////////////////////////////////////////////////////////////////////
    // ICP main loop
    // Check convergence criteria and the current iteration.
    // If mode=="svd" use get_svd_icp_registration if mode=="lm" use
    // get_lm_icp_registration. Remember to update transformation_ class
    // variable, you can use source_for_icp_ to store transformed 3d points.
    ///////////////////////////////////////////////////////////////////////////

    // Store the previous rmse to check convergence.
    double prev_rmse = std::numeric_limits<double>::max();

    for (int i = 0; i < max_iteration; i++) {
        // Find matches and rmse.
        std::tuple<std::vector<size_t>, std::vector<size_t>, double>
            matches_and_error = find_closest_point(threshold);

        // Find transformation matrix based on the mode.
        Eigen::Matrix4d transformation_matrix;
        if (mode == "svd") {
            transformation_matrix = get_svd_icp_registration(
                std::get<0>(matches_and_error), std::get<1>(matches_and_error));
        } else if (mode == "lm") {
            transformation_matrix = get_lm_icp_registration(
                std::get<0>(matches_and_error), std::get<1>(matches_and_error));
        }
        // Update transformation.
        transformation_ = transformation_ * transformation_matrix;

        // Check convergence.
        double rmse = std::get<2>(matches_and_error);
        std::cout << "Iteration: " << i << " RMSE: " << rmse << std::endl;
        if (rmse > prev_rmse || std::abs(prev_rmse - rmse) < 0.00000001 ||
            rmse < relative_rmse) {
            std::cout << "Converged!" << std::endl;
            break;
        }
        prev_rmse = rmse;
    }

    // Update source_ with the transformed points.
    source_ = source_for_icp_;

    return;
}

std::tuple<std::vector<size_t>, std::vector<size_t>, double>
Registration::find_closest_point(double threshold) {
    ///////////////////////////////////////////////////////////////////////////
    // Find source and target indices: for each source point find the closest
    // one in the target and discard if their distance is bigger than threshold
    ///////////////////////////////////////////////////////////////////////////

    std::vector<size_t> source_indices;
    std::vector<size_t> target_indices;
    double mse;

    // KDTree to efficiently find closest target points.
    open3d::geometry::KDTreeFlann target_kd_tree(target_);
    std::vector<int> idx(1);
    std::vector<double> dist2(1);

    // Use the currently available transformation of the source.
    open3d::geometry::PointCloud source_clone = source_;
    source_clone.Transform(transformation_);
    int num_source_points = source_clone.points_.size();

    int num_matches = 0;

    for (size_t i = 0; i < num_source_points; ++i) {
        // Find closest target point.
        Eigen::Vector3d source_point = source_clone.points_[i];
        target_kd_tree.SearchKNN(source_point, 1, idx, dist2);

        // If distance is smaller than threshold, add indices to the vectors and
        // update mse.
        if (sqrt(dist2[0]) <= threshold) {
            source_indices.push_back(i);
            target_indices.push_back(idx[0]);
            mse = mse * i / (i + 1) + dist2[0] / (i + 1);

            num_matches++;
        }
    }

    std::cout << "--------> matched " << num_matches << "/" << num_source_points
              << " source points. \n\n";

    return {source_indices, target_indices, sqrt(mse)};
}

Eigen::Matrix4d Registration::get_svd_icp_registration(
    std::vector<size_t> source_indices, std::vector<size_t> target_indices) {
    ///////////////////////////////////////////////////////////////////////////
    // Find point clouds centroids and subtract them.
    // Use SVD (Eigen::JacobiSVD<Eigen::MatrixXd>) to find best rotation and
    // translation matrix. Use source_indices and target_indices to extract
    // point to compute the 3x3 matrix to be decomposed. Remember to manage the
    // special reflection case.
    ///////////////////////////////////////////////////////////////////////////
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity(4, 4);

    // Use the currently available transformation of the source.
    open3d::geometry::PointCloud source_clone = source_;
    source_clone.Transform(transformation_);
    open3d::geometry::PointCloud target_clone = target_;

    // Compute centroids.
    Eigen::Vector3d source_centroid =
                        std::get<0>(source_clone.ComputeMeanAndCovariance()),
                    target_centroid =
                        std::get<0>(target_clone.ComputeMeanAndCovariance());

    // Find the W matrix for SVD.
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < source_indices.size(); i++) {
        int source_index = source_indices[i];
        int target_index = target_indices[i];
        Eigen::Vector3d source_point = source_clone.points_[source_index];
        Eigen::Vector3d target_point = target_clone.points_[target_index];

        W += (source_point - source_centroid) *
             (target_point - target_centroid).transpose();
    }

    // Compute SVD of W.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // Compute rotation and translation (decoupled).
    Eigen::Matrix3d R = V * U.transpose();
    // Manage special reflection case.
    if (R.determinant() == -1) {
        V.col(2) *= -1;
        R = V * U.transpose();
    }
    Eigen::Vector3d t = target_centroid - R * source_centroid;

    // Store rotation and translation in transformation.
    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = t;

    return transformation;
}

Eigen::Matrix4d Registration::get_lm_icp_registration(
    std::vector<size_t> source_indices, std::vector<size_t> target_indices) {
    ///////////////////////////////////////////////////////////////////////////
    // Use LM (Ceres) to find best rotation and translation matrix.
    // Remember to convert the euler angles in a rotation matrix, store it
    // coupled with the final translation on: Eigen::Matrix4d transformation.
    // The first three elements of std::vector<double> transformation_arr
    // represent the euler angles, the last ones the translation. use
    // source_indices and target_indices to extract point to compute the matrix
    // to be decomposed.
    ///////////////////////////////////////////////////////////////////////////
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity(4, 4);
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 4;
    options.max_num_iterations = 100;

    std::vector<double> transformation_arr(6, 0.0);
    int num_points = source_indices.size();
    // For each point....
    for (int i = 0; i < num_points; i++) {
    }

    return transformation;
}

void Registration::set_transformation(Eigen::Matrix4d init_transformation) {
    transformation_ = init_transformation;
}

Eigen::Matrix4d Registration::get_transformation() { return transformation_; }

double Registration::compute_rmse() {
    open3d::geometry::KDTreeFlann target_kd_tree(target_);
    open3d::geometry::PointCloud source_clone = source_;
    source_clone.Transform(transformation_);
    int num_source_points = source_clone.points_.size();
    Eigen::Vector3d source_point;
    std::vector<int> idx(1);
    std::vector<double> dist2(1);
    double mse;
    for (size_t i = 0; i < num_source_points; ++i) {
        source_point = source_clone.points_[i];
        target_kd_tree.SearchKNN(source_point, 1, idx, dist2);
        mse = mse * i / (i + 1) + dist2[0] / (i + 1);
    }
    return sqrt(mse);
}

void Registration::write_tranformation_matrix(std::string filename) {
    std::ofstream outfile(filename);
    if (outfile.is_open()) {
        outfile << transformation_;
        outfile.close();
    }
}

void Registration::save_merged_cloud(std::string filename) {
    // clone input
    open3d::geometry::PointCloud source_clone = source_;
    open3d::geometry::PointCloud target_clone = target_;

    // paint them
    Eigen::Vector3d color_s;
    Eigen::Vector3d color_t;
    color_s << 1, 0.706, 0;
    color_t << 0, 0.651, 0.929;

    target_clone.PaintUniformColor(color_t);
    source_clone.PaintUniformColor(color_s);

    source_clone.Transform(transformation_);

    open3d::geometry::PointCloud merged = target_clone + source_clone;
    open3d::io::WritePointCloud(filename, merged);
}
