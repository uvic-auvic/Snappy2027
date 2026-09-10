#include "thruster_allocator.hpp"
#include <algorithm>
#include <stdexcept>


ThrusterAllocator::ThrusterAllocator()
    : configuration_(Eigen::MatrixXd()), min_thrust_(Eigen::VectorXd()), max_thrust_(Eigen::VectorXd()) {}

ThrusterAllocator::ThrusterAllocator(const Eigen::MatrixXd &configuration,
                                       const Eigen::VectorXd &min_thrust,
                                       const Eigen::VectorXd &max_thrust)
    : configuration_(configuration) {
    int thruster_count = configuration_.cols(); // Number of thrusters in the configuration matrix

    // Check if the provided min_thrust and max_thrust vectors have the correct size
    if (min_thrust.size() == thruster_count && max_thrust.size() == thruster_count) {
        min_thrust_ = min_thrust;
        max_thrust_ = max_thrust;
    } else if (min_thrust.size() == 0 && max_thrust.size() == 0) {
        // If the sizes are not correct, initialize with default values (e.g., -1 to 1)
        min_thrust_ = Eigen::VectorXd::Constant(thruster_count, -1.0);
        max_thrust_ = Eigen::VectorXd::Constant(thruster_count, 1.0);
    } else {
        throw std::invalid_argument("Motor limit count must match configuration columns");
    }
    validate();
}

ThrusterAllocator::ThrusterAllocator(const Eigen::MatrixXd &configuration,
                                        const float min_thrust,
                                        const float max_thrust)
    : configuration_(configuration) {
    int thruster_count = configuration_.cols();
    min_thrust_ = Eigen::VectorXd::Constant(thruster_count, min_thrust);
    max_thrust_ = Eigen::VectorXd::Constant(thruster_count, max_thrust);
    validate();
}

void ThrusterAllocator::validate() const {
    if (configuration_.rows() != 6 || configuration_.cols() < 6 || !configuration_.allFinite() ||
        min_thrust_.size() != configuration_.cols() || max_thrust_.size() != configuration_.cols() ||
        !min_thrust_.allFinite() || !max_thrust_.allFinite() ||
        (min_thrust_.array() >= 0).any() || (max_thrust_.array() <= 0).any() ||
        configuration_.completeOrthogonalDecomposition().rank() != 6)
        throw std::invalid_argument("Allocation requires a finite full-rank six-axis matrix and motor limits spanning zero");
}

Eigen::VectorXd ThrusterAllocator::getThrusts_(const Eigen::VectorXd &wrench) const {
    if (wrench.size() != configuration_.rows() || !wrench.allFinite())
        throw std::invalid_argument("Invalid requested wrench");
    // Calculate the pseudo-inverse of the configuration matrix to find the thrusts that achieve the desired wrench
    Eigen::MatrixXd inverse = configuration_.completeOrthogonalDecomposition().pseudoInverse();

    // Calculate the thrusts by multiplying the pseudo-inverse with the desired wrench
    Eigen::VectorXd allocation = inverse * wrench;

    return allocation;
}

float ThrusterAllocator::getMaxSaturationRatio_(const Eigen::VectorXd &thrusts) const {
    float max_ratio = 0.0;

    // Iterate through each thruster and calculate the saturation ratio based on the min and max thrust limits
    for (int i = 0; i < thrusts.size(); ++i) {
        float ratio = 0.0;
        if (thrusts[i] > max_thrust_[i]) {
            ratio = thrusts[i] / max_thrust_[i];
        } else if (thrusts[i] < min_thrust_[i]) {
            ratio = thrusts[i] / min_thrust_[i];
        }
        max_ratio = std::max(max_ratio, ratio);
    }
    return max_ratio;
}

Eigen::VectorXd ThrusterAllocator::allocate(const Eigen::VectorXd &wrench) const {
    validate();
    // Calculate the thrusts based on the desired wrench
    Eigen::VectorXd thrusts = getThrusts_(wrench);

    // Calculate the maximum saturation ratio and scale the thrusts accordingly to ensure they are within the limits
    float max_saturation_ratio = getMaxSaturationRatio_(thrusts);
    if (max_saturation_ratio > 1.0) {
        thrusts /= max_saturation_ratio; // Scale down the thrusts to fit within the limits
    }
    return thrusts;
}

Eigen::VectorXd ThrusterAllocator::allocate_depth_priority(const Eigen::VectorXd &wrench) const {
    if (wrench.size() != 6) throw std::invalid_argument("Expected six wrench components");
    Eigen::VectorXd depth_wrench = Eigen::VectorXd::Zero(6);
    depth_wrench[2] = wrench[2];
    return allocate_with_priority(wrench, depth_wrench);
}

Eigen::VectorXd ThrusterAllocator::allocate_with_priority(
    const Eigen::VectorXd &wrench, const Eigen::VectorXd &priority_wrench) const {
    validate();
    if (wrench.size() != 6 || priority_wrench.size() != 6 ||
        !wrench.allFinite() || !priority_wrench.allFinite())
        throw std::invalid_argument("Requested and priority wrenches must be finite six-vectors");
    const Eigen::MatrixXd inverse = configuration_.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd baseline = inverse * priority_wrench;
    // Reduce heave only if the pure-heave solution itself exceeds motor limits.
    double depth_scale = 1.0;
    for (Eigen::Index i = 0; i < baseline.size(); ++i) {
        if (baseline[i] > max_thrust_[i])
            depth_scale = std::min(depth_scale, max_thrust_[i] / baseline[i]);
        else if (baseline[i] < min_thrust_[i])
            depth_scale = std::min(depth_scale, min_thrust_[i] / baseline[i]);
    }
    baseline *= depth_scale;

    const Eigen::VectorXd secondary = inverse * (wrench - priority_wrench);
    double secondary_scale = 1.0;
    for (Eigen::Index i = 0; i < secondary.size(); ++i) {
        if (secondary[i] > 1e-12)
            secondary_scale = std::min(secondary_scale, (max_thrust_[i] - baseline[i]) / secondary[i]);
        else if (secondary[i] < -1e-12)
            secondary_scale = std::min(secondary_scale, (min_thrust_[i] - baseline[i]) / secondary[i]);
    }
    secondary_scale = std::clamp(secondary_scale, 0.0, 1.0);
    return baseline + secondary_scale * secondary;
}

Eigen::MatrixXd ThrusterAllocator::get_configuration() const {
    return configuration_;
}

Eigen::VectorXd ThrusterAllocator::get_min_thrust() const {
    return min_thrust_;
}

Eigen::VectorXd ThrusterAllocator::get_max_thrust() const {
    return max_thrust_;
}

void ThrusterAllocator::set_configuration(const Eigen::MatrixXd &configuration) {
    ThrusterAllocator checked(configuration, min_thrust_, max_thrust_);
    *this = checked;
}

void ThrusterAllocator::set_min_thrust(const Eigen::VectorXd &min_thrust) {
    ThrusterAllocator checked(configuration_, min_thrust, max_thrust_);
    *this = checked;
}

void ThrusterAllocator::set_max_thrust(const Eigen::VectorXd &max_thrust) {
    ThrusterAllocator checked(configuration_, min_thrust_, max_thrust);
    *this = checked;
}

void ThrusterAllocator::set_min_thrust(const float min_thrust) {
    set_min_thrust(Eigen::VectorXd::Constant(configuration_.cols(), min_thrust));
}

void ThrusterAllocator::set_max_thrust(const float max_thrust) {
    set_max_thrust(Eigen::VectorXd::Constant(configuration_.cols(), max_thrust));
}
