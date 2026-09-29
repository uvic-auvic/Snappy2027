#include "thruster_allocator.hpp"


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
    } else {
        // If the sizes are not correct, initialize with default values (e.g., -1 to 1)
        min_thrust_ = Eigen::VectorXd::Constant(thruster_count, -1.0);
        max_thrust_ = Eigen::VectorXd::Constant(thruster_count, 1.0);
    }
}

ThrusterAllocator::ThrusterAllocator(const Eigen::MatrixXd &configuration,
                                        const float min_thrust,
                                        const float max_thrust)
    : configuration_(configuration) {
    int thruster_count = configuration_.cols();
    min_thrust_ = Eigen::VectorXd::Constant(thruster_count, min_thrust);
    max_thrust_ = Eigen::VectorXd::Constant(thruster_count, max_thrust);
}

Eigen::VectorXd ThrusterAllocator::getThrusts_(const Eigen::VectorXd &wrench) const {
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
    // Calculate the thrusts based on the desired wrench
    Eigen::VectorXd thrusts = getThrusts_(wrench);

    // Calculate the maximum saturation ratio and scale the thrusts accordingly to ensure they are within the limits
    float max_saturation_ratio = getMaxSaturationRatio_(thrusts);
    if (max_saturation_ratio > 1.0) {
        thrusts /= max_saturation_ratio; // Scale down the thrusts to fit within the limits
    }
    return thrusts;
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
    configuration_ = configuration;
}

void ThrusterAllocator::set_min_thrust(const Eigen::VectorXd &min_thrust) {
    min_thrust_ = min_thrust;
}

void ThrusterAllocator::set_max_thrust(const Eigen::VectorXd &max_thrust) {
    max_thrust_ = max_thrust;
}

void ThrusterAllocator::set_min_thrust(const float min_thrust) {
    min_thrust_ = Eigen::VectorXd::Constant(configuration_.rows(), min_thrust);
}

void ThrusterAllocator::set_max_thrust(const float max_thrust) {
    max_thrust_ = Eigen::VectorXd::Constant(configuration_.rows(), max_thrust);
}
