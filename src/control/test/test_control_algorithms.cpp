#include <cmath>
#include <iostream>
#include <stdexcept>

#include "pid.hpp"
#include "thruster_allocator.hpp"
#include "control_math.hpp"
#include <limits>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

bool near(double actual, double expected, double tolerance = 1e-5) {
    return std::abs(actual - expected) < tolerance;
}

Eigen::MatrixXd vehicle_configuration() {
    Eigen::MatrixXd matrix(6, 8);
    matrix <<
        0, 0, 1, 0, 0, 0, 1, 0,
        -1, 0, 0, 0, 1, 0, 0, 0,
        0, 1, 0, 1, 0, 1, 0, 1,
        .1301, .1653, 0, .1653, -.1301, -.1648, 0, -.1648,
        0, .3024, -.0158, -.2977, 0, -.2977, -.0159, .3024,
        -.3041, 0, -.2739, 0, -.3121, 0, .2734, 0;
    return matrix;
}

void check_limits(const Eigen::VectorXd& forces) {
    require(forces.allFinite(), "Allocation contains non-finite forces");
    require((forces.array() >= -4.0 - 1e-8).all(), "Reverse thrust limit exceeded");
    require((forces.array() <= 5.0 + 1e-8).all(), "Forward thrust limit exceeded");
}

void test_derivative_transition() {
    PID pid(0.0f, 0.0f, 1.725f);
    require(near(pid.update(0.0f, .02f), 0), "Initial derivative is not suppressed");
    // Controller supplies negative error, with PID internal target held at zero.
    pid.reset_derivative();
    require(near(pid.update(-1.5707963f, .02f), 0), "90-degree task step caused derivative kick");
    require(near(pid.update(-1.5707963f, .02f), 0), "Derivative kick delayed by one update");
    require(pid.update(-1.47f, .02f) < -8, "Derivative did not resume after transition");

    PID internal_target(0, 0, 1);
    internal_target.update(0, .02f);
    internal_target.set_target(1);
    require(near(internal_target.update(0, .02f), 0), "set_target caused derivative kick");
    internal_target.set_target(1);
    require(near(internal_target.update(.1f, .02f), -5), "Repeated target suppressed real derivative");
}

void test_integral_and_feedback() {
    PID depth(0, 1, 0);
    for (int i = 0; i < 10; ++i) depth.update(-1, .1f);
    depth.reset_derivative();
    require(near(depth.update(0, .02f), 1), "Derivative reset erased depth compensation");

    for (float direction : {-1.0f, 1.0f}) {
        PID saturated(0, 1, 0);
        for (int i = 0; i < 100; ++i) {
            saturated.update(-direction, .02f);
            saturated.apply_output_feedback(0);
        }
        require(near(saturated.update(0, .02f), 0), "Integral wound up against motor saturation");
    }

    // Integration that reduces an existing saturated command must be retained.
    PID unwinding(0, 1, 0);
    unwinding.update(-10, 1);
    for (int i = 0; i < 5; ++i) {
        unwinding.update(1, 1);
        unwinding.apply_output_feedback(0);
    }
    require(near(unwinding.update(0, .02f), 5), "Saturation prevented integral unwinding");

    PID local_clamp(200, 1, 0);
    for (int i = 0; i < 100; ++i) local_clamp.update(-1, .02f);
    require(near(local_clamp.update(0, .02f), 0), "Integral wound up against PID output clamp");
}

void test_allocation() {
    const auto matrix = vehicle_configuration();
    ThrusterAllocator allocator(matrix, -4.0f, 5.0f);
    Eigen::VectorXd wrench(6);
    wrench << .2, -.1, 5, .1, -.1, .1;
    auto forces = allocator.allocate_depth_priority(wrench);
    require((matrix * forces - wrench).norm() < 1e-8, "Unsaturated wrench was changed");
    require((forces - allocator.allocate(wrench)).norm() < 1e-8, "Unsaturated allocation changed");

    for (int axis : {0, 1, 3, 4, 5}) {
        for (double heave : {-5.0, 5.0}) {
            for (double demand : {-100.0, 100.0}) {
                wrench.setZero();
                wrench[2] = heave;
                wrench[axis] = demand;
                forces = allocator.allocate_depth_priority(wrench);
                check_limits(forces);
                require(near((matrix * forces)[2], heave), "Saturated maneuver stole heave thrust");
                require(std::abs((matrix * forces)[axis]) < std::abs(demand), "Maneuver was not limited");
            }
        }
    }

    for (double heave : {-100.0, 100.0}) {
        wrench.setZero();
        wrench[2] = heave;
        wrench[3] = 100;
        forces = allocator.allocate_depth_priority(wrench);
        check_limits(forces);
        require((matrix * forces)[2] * heave > 0, "Infeasible heave changed direction");
        require(std::abs((matrix * forces)[2]) < std::abs(heave), "Infeasible heave was not limited");
    }

    // Large yaw + heave PID request: feedback must exclude the +5 feedforward.
    PID depth(0, 1, 0);
    PID yaw(20, 1, 0);
    for (int i = 0; i < 100; ++i) {
        const float heave_output = depth.update(-1, .02f);
        const float yaw_output = yaw.update(-1, .02f);
        wrench.setZero();
        wrench[2] = heave_output + 5;
        wrench[5] = yaw_output;
        const Eigen::VectorXd achieved = matrix * allocator.allocate_depth_priority(wrench);
        depth.apply_output_feedback(achieved[2] - 5);
        yaw.apply_output_feedback(achieved[5]);
    }
    require(near(depth.update(0, .02f), 2, 1e-4), "Yaw saturation incorrectly inhibited depth integration");
    require(near(yaw.update(0, .02f), 0), "Yaw integral wound up during allocation");

    // Even the priority axis must stop integrating when heave itself is infeasible.
    PID saturated_depth(30, 1, 0);
    for (int i = 0; i < 100; ++i) {
        wrench.setZero();
        wrench[2] = saturated_depth.update(-1, .02f) + 5;
        const Eigen::VectorXd achieved = matrix * allocator.allocate_depth_priority(wrench);
        saturated_depth.apply_output_feedback(achieved[2] - 5);
    }
    require(near(saturated_depth.update(0, .02f), 0), "Depth integral wound up at heave limit");
}

void test_frames_and_validation() {
    const auto matrix = vehicle_configuration();
    ThrusterAllocator allocator(matrix, -4.0f, 5.0f);
    const double pi = std::acos(-1.0);
    // At 90-degree yaw, a world-X rotation is a negative body-Y rotation.
    const auto current = snappy_control::orientation_from_rpy(0, 0, pi / 2);
    const Eigen::Quaterniond target = Eigen::Quaterniond(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitX())) * current;
    const auto error = snappy_control::body_rotation_error(current, target);
    require((error - Eigen::Vector3d(0, -.2, 0)).norm() < 1e-8, "Torque error is not in body frame");
    auto negative_target = target;
    negative_target.coeffs() *= -1;
    require((snappy_control::body_rotation_error(current, negative_target) - error).norm() < 1e-8,
            "Quaternion sign changed rotation error");
    for (double roll : {0.0, pi / 4, pi / 2}) {
        const auto attitude = snappy_control::orientation_from_rpy(roll, .2, .7);
        Eigen::VectorXd priority = Eigen::VectorXd::Zero(6);
        priority.head<3>() = attitude.conjugate() * Eigen::Vector3d(0, 0, 3);
        Eigen::VectorXd desired = priority;
        desired.tail<3>() = Eigen::Vector3d(30, -20, 100);
        const auto forces = allocator.allocate_with_priority(desired, priority);
        check_limits(forces);
        const Eigen::VectorXd body = matrix * forces;
        const Eigen::Vector3d world = attitude * Eigen::Vector3d(body.head<3>());
        require((world - Eigen::Vector3d(0, 0, 3)).norm() < 1e-8, "Tilted maneuver lost world-vertical force");
    }
    allocator.set_min_thrust(-3.0f);
    allocator.set_max_thrust(4.0f);
    require(allocator.get_min_thrust().size() == 8 && allocator.get_max_thrust().size() == 8,
            "Scalar limits use axis count instead of motor count");
    require(snappy_control::thrust_to_command(1000) == 100, "Motor conversion overflow");
    bool threw = false;
    try { snappy_control::thrust_to_command(std::numeric_limits<double>::quiet_NaN()); }
    catch (const std::invalid_argument&) { threw = true; }
    require(threw, "NaN motor force was accepted");
    threw = false;
    try {
        Eigen::VectorXd bad = Eigen::VectorXd::Zero(6);
        bad[2] = std::numeric_limits<double>::infinity();
        allocator.allocate_depth_priority(bad);
    } catch (const std::invalid_argument&) { threw = true; }
    require(threw, "Non-finite wrench was accepted");
}
} // namespace

int main() {
    try {
        test_derivative_transition();
        test_integral_and_feedback();
        test_allocation();
        test_frames_and_validation();
        std::cout << "Control algorithm regression tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
