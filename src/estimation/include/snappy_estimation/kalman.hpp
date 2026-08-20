// Kalman Filter header file
// Fuses two IMUs, a depth sensor,and a DVL to estimate position, velocity, and orientation

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

using namespace Eigen;

class KalmanFilter
{
public:
    
    KalmanFilter();

    // - x0: Initial state [pos(3), vel(3), quat(4), accel_bias(3)] = 13x1
    // - P0: Initial error-state covariance 12x12
    // - Q: Process noise covariance 6x6 [accel_noise(0:3), accel_bias_walk(3:6)]
    // - R_imu1: IMU1 measurement noise covariance (accel gravity vector, low trust)
    // - R_depth: Depth sensor measurement noise covariance
    // - q_imu1_to_body: rotation from IMU1 frame to the filter body frame
    // - q_imu2_to_body: rotation from IMU2 frame to the filter body frame
    // - R_dvl_vel: DVL body-frame velocity measurement noise covariance 3x3
    KalmanFilter(VectorXd x0Input, MatrixXd P0Input, MatrixXd QInput,
                 MatrixXd R_imu1Input, MatrixXd R_depthInput,
                 Quaterniond q_imu1_to_body = Quaterniond::Identity(),
                 Quaterniond q_imu2_to_body = Quaterniond::Identity(),
                 MatrixXd R_dvl_velInput = MatrixXd::Identity(3, 3) * 0.01);


    // Predict step: propagate pos/vel from IMU2 accel, update orientation from IMU2 quat.
    void predict(const Eigen::Vector3d& accel, const Eigen::Quaterniond& quat_imu2, double dt);

    // IMU1 combined low-trust predict+update: dead-reckons candidate position/
    // velocity (accel kinematics) and orientation (gyro integration) over dt,
    // then applies their residuals as ONE weak measurement update with large R
    // (small Kalman gain). IMU2/DVL/depth remain the strong corrections.
    void updateIMU1(const Vector3d& accel, const Vector3d& gyro, double dt);

    // Update step: correct state estimate using depth sensor
    void updateDepth(double depth_measured);

    // Update step: DVL velocity + position update.
    // v_measured: DVL body-frame velocity. p_measured: world-frame position.
    // R_vel / R_pos: 3x3 noise covariances for velocity and position respectively.
    void updateDVL(const Vector3d& v_measured, const Vector3d& p_measured,
                   const Matrix3d& R_vel, const Matrix3d& R_pos);

    // Update step: DVL velocity only (frame-safe). v_measured is the DVL/body-frame
    // velocity; it is rotated into the world frame by the current attitude and used
    // to correct the velocity state (which in turn bounds position drift via predict).
    // Uses R_dvl_vel_ (constructor arg or setDVLVelocityMeasurementNoise).
    void updateDVLVelocity(const Vector3d& v_measured);

    // For testing using csv files
    static MatrixXd openData(const std::string& fileToOpen);

    // Getters for state vector components
    Vector3d getPosition() const;
    Vector3d getVelocity() const;
    Quaterniond getOrientation() const;

    // Get full state vector
    VectorXd getState() const { return x; }
    
    // Get covariance matrix
    MatrixXd getCovariance() const { return P; }

    // Setters for noise covariances and initial conditions
    void setIMU1MeasurementNoise(const MatrixXd& R_imu1Input);
    void setDepthMeasurementNoise(const MatrixXd& R_depthInput);
    void setDVLVelocityMeasurementNoise(const MatrixXd& R_dvl_vel);
    void setProcessNoise(const MatrixXd& QInput);
    void setInitialCovariance(const MatrixXd& P0Input);
    void setIMU1ToBodyRotation(const Quaterniond& q_imu1_to_body);
    void setIMU2ToBodyRotation(const Quaterniond& q_imu2_to_body);

    void setIMU2OrientationNoise(const Matrix3d& R) { R_imu2_orient_ = R; }
    void setOrientationProcessVar(double v)         { orient_process_var_ = v; }



    // Reset/reinitialize the filter
    void reset(const VectorXd& x0, const MatrixXd& P0);

    // Fixed rotation from the Xsens ENU (z-up) reference into z-down NED
    // (x north, y east, z down): 180 deg about the x+y diagonal
    // (east->world y, north->world x, up->world -z). This is NOT a mounting
    // rotation: the Xsens quaternion RELATES its sensor frame to an ENU world,
    // and this constant re-references its world side. q_imu2_to_body_ fixes
    // the sensor/mount side. Both are needed:
    //   q_world<-body = q_ned_to_world_ * q_enu_to_world_ * q_enu<-imu2 * q_imu2<-body^-1
    // Public and shared so predict() and try_initialize() (state_estimator.cpp)
    // can never disagree about the world frame.
    inline static const Quaterniond q_enu_to_world_ = Quaterniond(
        0.0, 0.7071067811865476, 0.7071067811865476, 0.0);  // (w, x, y, z)

    // Mission-frame alignment, assigned once by try_initialize(): Rz(-yaw0),
    // so world +x = vehicle heading at frame init, +y right of it, +z down.
    // predict() applies it on the world side of every IMU2 fusion, so a
    // constant compass offset in the Xsens yaw cancels out of the filter.
    // Identity until frame init = world stays NED.
    Quaterniond q_ned_to_world_ = Quaterniond::Identity();

private:
    // State vector
    VectorXd x;

    // Error-state covariance matrix
    MatrixXd P;

    // Process noise covariance 
    MatrixXd Q;

    // Measurement noise covariances
    MatrixXd R_imu1_;      // low trust orientation (3x3, gravity vector)
    MatrixXd R_depth;
    MatrixXd R_dvl_vel_ = MatrixXd::Identity(3, 3) * 0.01;  // DVL body-frame velocity (3x3)

    // Low-trust variances for the combined IMU1 predict+update in updateIMU1().
    // Deliberately large => small Kalman gain: the IMU1 dead-reckoned candidates
    // only weakly correct the filter between IMU2 samples.
    double imu1_pos_var_    = 10.0;  // m²
    double imu1_vel_var_    = 10.0;  // (m/s)²
    double imu1_orient_var_ = 1.0;   // rad²

    // High-trust IMU2 quaternion orientation update (rad²) used in predict().
    // Small => IMU2 dominates attitude (gain near 1).
    Matrix3d R_imu2_orient_ = Matrix3d::Identity() * 1e-4;

    // Rotation from IMU1 frame to filter body frame (used in updateIMU1)
    Quaterniond q_imu1_to_body_ = Quaterniond::Identity();

    // Rotation from IMU2 frame to filter body frame (used in predict)
    Quaterniond q_imu2_to_body_ = Quaterniond::Identity();

    // Orientation process noise (rad²/s) added to P(6:9,6:9) each predict(). Lets the
    // orientation covariance re-grow between updates so the IMU2 fusion and the
    // low-trust IMU1 corrections keep a non-zero Kalman gain.
    double orient_process_var_ = 1e-2;

    // Gravity vector in the z-DOWN world frame (added to q*f in predict())
    const Vector3d gravity = Vector3d(0, 0, 9.81);

    // Computes error-state transition Jacobian F
    MatrixXd computeF(double dt, const Vector3d& accel) const;

    // Computes process noise mapping G (15x12)
    MatrixXd computeG() const;

    // Normalize quaternion in state vector
    void normalizeQuaternion();

    // Helper function: skew-symmetric matrix from 3-vector
    Matrix3d skew(const Vector3d& v) const;

    // Inject error-state correction into nominal state
    void injectErrorState(const VectorXd& dx);

    // Generic ESKF measurement update with residual and error-state Jacobian H
    void updateErrorState(const VectorXd& residual, const MatrixXd& H, const MatrixXd& R);

    // Orientation measurement update: pulls the state quaternion toward q_meas
    // (a measured body-in-world attitude) with measurement noise R.
    void updateOrientation(const Quaterniond& q_meas, const Matrix3d& R);

};

#endif // KALMANFILTER_H
