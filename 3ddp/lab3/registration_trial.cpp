#include <chrono>
#include <iostream>

#include "Registration.h"

int main(int argc, char *argv[]) {
    if (argc < 4) std::cerr << "Usage <source pc> <target pc> <mode>";
    std::string mode = argv[3];
    Registration registration(argv[1], argv[2]);
    registration.draw_registration_result();

    std::chrono::steady_clock::time_point begin =
        std::chrono::steady_clock::now();
    registration.execute_icp_registration(0.2, 100, 1e-6, mode);
    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();
    std::cout << "\nElapsed time = "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                       begin)
                         .count() /
                     1000.0
              << "[s]" << std::endl;

    registration.draw_registration_result();
    std::cout << "RMSE: " << registration.compute_rmse() << std::endl;
    registration.write_tranformation_matrix("transformation.txt");
    registration.save_merged_cloud("merged_registered_cloud.ply");
    return 0;
}
