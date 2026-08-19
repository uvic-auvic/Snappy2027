#ifndef THRUSTER_ALLOCATOR_H
#define THRUSTER_ALLOCATOR_H

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

/*
 * ThrusterAllocator class for allocating thruster forces based on a desired wrench (force and torque).
 * The class takes a configuration matrix that maps thruster forces to the resulting wrench on the vehicle.
 * It also allows for specifying minimum and maximum thrust limits for each thruster.
 */
class ThrusterAllocator {
    private:
        Eigen::MatrixXd configuration_; // Configuration matrix mapping thruster forces to wrench
        Eigen::VectorXd min_thrust_; // Minimum thrust limits for each thruster
        Eigen::VectorXd max_thrust_; // Maximum thrust limits for each thruster



        /*
         * Calculates the thrusts for each thruster based on the desired wrench.
         * @param wrench The desired wrench (force and torque) to be achieved.
         * @return The calculated thrusts for each thruster.
         */
        Eigen::VectorXd getThrusts_(const Eigen::VectorXd &wrench) const;
        /*
         * Calculates the maximum saturation ratio among all thrusters.
         * @param thrusts The calculated thrusts for each thruster.
         * @return The maximum saturation ratio
         */
        float getMaxSaturationRatio_(const Eigen::VectorXd &thrusts) const;

    public:
        /*
         * Constructor for the ThrusterAllocator class.
         * @param configuration The configuration matrix mapping thruster forces to wrench.
         * @param min_thrust The minimum thrust limits for each thruster, either a vector matching the number of thrusters or a scalar.
         * @param max_thrust The maximum thrust limits for each thruster, either a vector matching the number of thrusters or a scalar.
         */
        ThrusterAllocator();
        ThrusterAllocator(const Eigen::MatrixXd &configuration,
                                   const Eigen::VectorXd &min_thrust = Eigen::VectorXd(),
                                   const Eigen::VectorXd &max_thrust = Eigen::VectorXd());
        ThrusterAllocator(const Eigen::MatrixXd &configuration,
                                   const float min_thrust,
                                   const float max_thrust);

        /*
         * Allocates thruster forces based on a desired wrench.
         * @param wrench The desired wrench (force and torque) to be achieved.
         * @return The allocated thruster forces.
         */
        Eigen::VectorXd allocate(const Eigen::VectorXd &wrench) const;
        Eigen::MatrixXd get_configuration() const;
        Eigen::VectorXd get_min_thrust() const;
        Eigen::VectorXd get_max_thrust() const;

        void set_configuration(const Eigen::MatrixXd &configuration);
        void set_min_thrust(const Eigen::VectorXd &min_thrust);
        void set_max_thrust(const Eigen::VectorXd &max_thrust);
        void set_min_thrust(const float min_thrust);
        void set_max_thrust(const float max_thrust);

};

#endif
