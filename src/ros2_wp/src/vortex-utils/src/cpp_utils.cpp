#include "vortex_utils/cpp_utils.hpp"

namespace vortex_utils {
double ssa(const double angle) {
    double result = fmod(angle + M_PI, 2 * M_PI);
    double angle_ssa = result < 0 ? result + M_PI : result - M_PI;
    return angle_ssa;
}

Matrix3d skew_symmetric(const Eigen::Vector3d& vector) {
    Matrix3d skew_symmetric_matrix;
    skew_symmetric_matrix << 0, -vector.z(), vector.y(), vector.z(), 0,
        -vector.x(), -vector.y(), vector.x(), 0;
    return skew_symmetric_matrix;
}

Eigen::Vector3d quat_to_euler(const Eigen::Quaterniond& q) {
    Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);
    return euler_angles;
}

}  // namespace vortex_utils
