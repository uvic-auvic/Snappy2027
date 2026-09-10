#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace snappy_control {
inline Eigen::Quaterniond orientation_from_rpy(double roll, double pitch, double yaw) {
    if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw))
        throw std::invalid_argument("Orientation target must be finite");
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).normalized();
}

inline Eigen::Vector3d body_rotation_error(const Eigen::Quaterniond& current,
                                          const Eigen::Quaterniond& target) {
    Eigen::Quaterniond error = (current.conjugate() * target).normalized();
    if (error.w() < 0) error.coeffs() *= -1;
    const Eigen::AngleAxisd rotation(error);
    return rotation.angle() * rotation.axis();
}

inline int8_t thrust_to_command(double force) {
    if (!std::isfinite(force)) throw std::invalid_argument("Non-finite motor force");
    return static_cast<int8_t>(std::clamp(force * (force > 0 ? 20.0 : 25.0), -100.0, 100.0));
}
} // namespace snappy_control
