// Kalman Filter Implementation
// Using an Error-state Extended Kalman Filter
// https://notanymike.github.io/Error-State-Extended-Kalman-Filter/
// Good paper for quaternion EKF: https://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf


/*
    ENU -- East-North-Up ()
    NED -- North-East-Down
    ---- Frame conventions ----
    World frame = MISSION frame, anchored at frame init:
      +x = vehicle heading at init, +y = right of that, +z down.
    Depth below the surface maps to POSITIVE world z: moving 2 m down => z = +2.
    Example: (1.0, 2.0, 9.0) = 1 m forward and 2 m right of the starting pose, 9 m below the surface.

     - World frame: Position (0, 0, depth) at t = 0 (depth sensor value)
     - World frame: Orientation yaw = 0 at t = 0 by construction (roll/pitch as measured)
    Body frame: +x forward, +y right, +z down (matches the world at init when level).


    The IMU2/Xsens quaternion is referenced to an ENU (z-up) world, so predict()
    re-references it with two world-side rotations before fusing:
      1. q_enu_to_world_ (fixed): ENU -> z-down NED (east->world y, north->x, up->-z)
      2. q_ned_to_world_ (Rz(-yaw0), set once at frame init): NED -> mission frame.
         This also absorbs any constant compass offset in the Xsens yaw.

    - IMU2 orientation:
        - physically mounted upside down: rotate 180 deg about the Y axis to reach the
          body frame (x fwd, y right, z down): q_imu2_to_body = (w=0, x=0, y=1, z=0)

    - IMU1: sensor z points UP, so it needs 180 deg about the X axis: q_imu1_to_body = (w=0, x=1, y=0, z=0)

    - DVL: the driver reports velocity with fully negated sign; the node negates
      all three components to get body FRD (verified in-water 2026-07-13).

*/




#include "snappy_estimation/kalman.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

using namespace Eigen;

KalmanFilter::KalmanFilter()
{
    // Default constructor with zero state and identity covariances
    x = VectorXd::Zero(13); // [pos(3), vel(3), quat(4), accel_bias(3)]
    x(6) = 1.0;  // identity quaternion [w, x, y, z]
    P = MatrixXd::Identity(12, 12);
    Q = MatrixXd::Identity(6, 6);
    R_imu1_ = MatrixXd::Identity(3, 3) * 10.0;
    R_depth = MatrixXd::Identity(1, 1);
}

KalmanFilter::KalmanFilter(VectorXd x0Input, MatrixXd P0Input, MatrixXd QInput,
                           MatrixXd R_imu1Input, MatrixXd R_depthInput,
                           Quaterniond q_imu1_to_body, Quaterniond q_imu2_to_body, MatrixXd R_dvl_velInput)
{
    x = x0Input;
    P = P0Input;
    Q = QInput;
    R_imu1_ = R_imu1Input;
    R_depth = R_depthInput;
    R_dvl_vel_ = R_dvl_velInput;
    q_imu1_to_body_ = q_imu1_to_body.normalized();
    q_imu2_to_body_ = q_imu2_to_body.normalized();

    normalizeQuaternion();
    std::cout << "Kalman Filter initialized with " << x.size() << " states" << std::endl;
}


// Predict step: IMU2 accel and quat. used to predict postion, velocity, and orientation. Accel bias is constant.
void KalmanFilter::predict(const Eigen::Vector3d& accel, const Eigen::Quaterniond& quat_imu2, double dt)
{
    Quaterniond q(x(6), x(7), x(8), x(9));
    q.normalize();

    Vector3d accel_bias = x.segment<3>(10);

    Vector3d accel_body = q_imu2_to_body_ * (accel - accel_bias);

    Vector3d accel_world = q * accel_body + gravity;

    Vector3d position = x.segment<3>(0);
    Vector3d velocity = x.segment<3>(3);
    position += velocity * dt + 0.5 * accel_world * dt * dt;
    velocity += accel_world * dt;

    x.segment<3>(0) = position;
    x.segment<3>(3) = velocity;

    // ---- 2. Covariance propagation (Sola 7.3) ----
    MatrixXd F   = computeF(dt, accel);
    MatrixXd Phi = MatrixXd::Identity(12, 12) + F * dt;
    MatrixXd G   = computeG();
    P = Phi * P * Phi.transpose() + G * Q * G.transpose() * dt;

    P.block<3,3>(6,6) += Matrix3d::Identity() * orient_process_var_ * dt;
    P = 0.5 * (P + P.transpose());

    // Since quaternion is a direct measurement, use an update step here. (May want to split this later)
    // Chain: q_world<-body = q_ned_to_world_ * q_enu_to_world_ * q_enu<-imu2 * q_imu2<-body.
    // The Xsens quaternion is body-in-ENU (z-up); q_enu_to_world_ converts its
    // reference into z-down NED, then q_ned_to_world_ (Rz(-yaw0), set at frame
    // init) re-references NED into the mission frame (+x = heading at init).
    
    Quaterniond q_meas = (q_ned_to_world_ * q_enu_to_world_ * quat_imu2.normalized()
                          * q_imu2_to_body_.conjugate()).normalized();

    updateOrientation(q_meas, R_imu2_orient_);
}


void KalmanFilter::updateOrientation(const Quaterniond& q_meas, const Matrix3d& R)
{
    Quaterniond q(x(6), x(7), x(8), x(9));
    q.normalize();

    // Body-frame error rotation from estimate to measurement: q_meas = q * dq
    Quaterniond dq = q.conjugate() * q_meas.normalized();
    if (dq.w() < 0.0) dq.coeffs() *= -1.0;   // shortest-path (avoid the ±q ambiguity)
    Vector3d residual = 2.0 * dq.vec();        // small-angle body-frame residual

    MatrixXd H = MatrixXd::Zero(3, 12);
    H.block<3,3>(0, 6) = Matrix3d::Identity();
    updateErrorState(residual, H, R);
}




// IMU1 low-trust update: the residual is what IMU1 says changed over dt minus
// what the state says (dead-reckoned candidate minus current state, which
// simplifies to the deltas below). Large R => small Kalman gain, so this only
// weakly corrects the filter; IMU2/DVL/depth stay the strong corrections.
void KalmanFilter::updateIMU1(const Vector3d& accel_raw, const Vector3d& gyro_raw, double dt)
{
    if (dt <= 0.0) return;

    Quaterniond q(x(6), x(7), x(8), x(9));
    q.normalize();

    // Gyro: body-frame rotation over dt — already the small-angle orientation

    Vector3d dtheta = (q_imu1_to_body_ * gyro_raw) * dt;

    // Accel: same z-DOWN world convention as predict(): a = q*f + g
    Vector3d accel_world = q * (q_imu1_to_body_ * accel_raw) + gravity;

    Vector3d velocity = x.segment<3>(3);

    VectorXd residual(9);
    residual.segment<3>(0) = velocity * dt + 0.5 * accel_world * dt * dt;  // = p_cand - p
    residual.segment<3>(3) = accel_world * dt;                             // = v_cand - v
    residual.segment<3>(6) = dtheta;

    MatrixXd H = MatrixXd::Zero(9, 12);
    H.block<3,3>(0, 0) = Matrix3d::Identity();  // observes position error states
    H.block<3,3>(3, 3) = Matrix3d::Identity();  // observes velocity error states
    H.block<3,3>(6, 6) = Matrix3d::Identity();  // observes orientation error states

    updateErrorState(residual, H, R_imu1_);
}



void KalmanFilter::updateDepth(double depth_measured)
{
    // We may have to apply a sin(pitch) to the depth measurement if the vehicle is not level
    VectorXd residual(1);
    residual << (depth_measured - x(2));

    if (depth_measured < -0.2) {
       return;
    }

    MatrixXd H = MatrixXd::Zero(1, 12);
    H(0, 2) = 1.0;

    updateErrorState(residual, H, R_depth);
}

// DVL velocity update
// may want a position update too (add later if needed)
void KalmanFilter::updateDVLVelocity(const Vector3d& v_measured)
{
    Quaterniond q(x(6), x(7), x(8), x(9));
    q.normalize();

    Vector3d v_world = q * v_measured;          // body -> world

    VectorXd residual(3);
    residual = v_world - x.segment<3>(3);

    MatrixXd H = MatrixXd::Zero(3, 12);
    H.block<3,3>(0, 3) = Matrix3d::Identity();  // observes velocity error states

    updateErrorState(residual, H, R_dvl_vel_);
}


// Solà 7.3 "The error-state Kalman filter" — update step
void KalmanFilter::updateErrorState(const VectorXd& residual, const MatrixXd& H, const MatrixXd& R)
{
    // Solà 7.3 innovation covariance: S = H*P*H^T + R
    // Total uncertainty in the residual = filter uncertainty + sensor noise
    MatrixXd S = H * P * H.transpose() + R;

    // Solà 7.3 Kalman gain: K = P*H^T * S^-1
    // Weights how much to trust the measurement vs. the current estimate
    MatrixXd K = P * H.transpose() * S.inverse();

    // Solà 7.3 error state correction: dx = K * residual
    // Then inject into nominal state (additive for pos/vel/bias, multiplicative for quaternion)
    VectorXd dx = K * residual;
    injectErrorState(dx);

    // Solà 7.3 Joseph-form covariance update: P = (I-KH)*P*(I-KH)^T + K*R*K^T
    MatrixXd I = MatrixXd::Identity(12, 12);
    MatrixXd I_KH = I - K * H;
    P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
    P = 0.5 * (P + P.transpose());  // fixes rounding errors
}


Vector3d KalmanFilter::getPosition() const
{
    return x.segment<3>(0);
}

Vector3d KalmanFilter::getVelocity() const
{
    return x.segment<3>(3);
}

Quaterniond KalmanFilter::getOrientation() const
{
    // State stores quaternion as [w, x, y, z] at indices 6-9
    return Quaterniond(x(6), x(7), x(8), x(9));
}

void KalmanFilter::setIMU1MeasurementNoise(const MatrixXd& R_imu1Input)
{
    R_imu1_ = R_imu1Input;
}

void KalmanFilter::setDepthMeasurementNoise(const MatrixXd& R_depthInput)
{
    R_depth = R_depthInput;
}

void KalmanFilter::setDVLVelocityMeasurementNoise(const MatrixXd& R_dvl_vel)
{
    R_dvl_vel_ = R_dvl_vel;
}

void KalmanFilter::setProcessNoise(const MatrixXd& QInput)
{
    Q = QInput;
}

void KalmanFilter::setInitialCovariance(const MatrixXd& P0Input)
{
    P = P0Input;
}

void KalmanFilter::setIMU1ToBodyRotation(const Quaterniond& q_imu1_to_body)
{
    q_imu1_to_body_ = q_imu1_to_body.normalized();
}

void KalmanFilter::setIMU2ToBodyRotation(const Quaterniond& q_imu2_to_body)
{
    q_imu2_to_body_ = q_imu2_to_body.normalized();
}

void KalmanFilter::reset(const VectorXd& x0, const MatrixXd& P0)
{
    x = x0;
    P = P0;
    normalizeQuaternion();

    std::cout << "Kalman Filter reset" << std::endl;
}


void KalmanFilter::normalizeQuaternion()
{
    Vector4d quat = x.segment<4>(6);
    double norm = quat.norm();
    if (norm > 1e-9) {
        x.segment<4>(6) = quat / norm;
    } else {
        x(6) = 1.0;
        x(7) = 0.0;
        x(8) = 0.0;
        x(9) = 0.0;
    }
}

// F: Jacobian of the error-state transition function (Sola 7.3).
// Orientation rows are zero — orientation comes from direct IMU2 injection.
// accel_bias is treated as being in the IMU2 frame (consistent with predict()).
MatrixXd KalmanFilter::computeF(double dt, const Vector3d& accel) const
{
    (void)dt;
    MatrixXd F = MatrixXd::Zero(12, 12);

    Quaterniond q(x(6), x(7), x(8), x(9));
    q.normalize();
    Matrix3d Rwb = q.toRotationMatrix();
    Matrix3d R_imu2_body = q_imu2_to_body_.toRotationMatrix();

    Vector3d accel_bias = x.segment<3>(10);
    Vector3d accel_body = R_imu2_body * (accel - accel_bias);  // bias in IMU2 frame, same as predict()

    F.block<3,3>(0, 3) = Matrix3d::Identity();       // dp/dv
    F.block<3,3>(3, 6) = -Rwb * skew(accel_body);    // dv/dtheta
    F.block<3,3>(3, 9) = -Rwb * R_imu2_body;         // dv/db_a (bias in IMU2 frame)

    return F;
}
// G maps process noise (Q, 6x6) into the error-state space (12 dims).
// Q column layout: accel_noise(0-2), accel_bias_walk(3-5)
MatrixXd KalmanFilter::computeG() const
{
    MatrixXd G = MatrixXd::Zero(12, 6);

    Quaterniond q(x(6), x(7), x(8), x(9));
    q.normalize();
    Matrix3d Rwb = q.toRotationMatrix();

    G.block<3,3>(3, 0) = -Rwb;                 // accel noise → velocity
    G.block<3,3>(9, 3) = Matrix3d::Identity();  // accel bias random walk

    return G;
}
// Helper for computeF
Matrix3d KalmanFilter::skew(const Vector3d& v) const
{
    Matrix3d S;
    S <<     0.0, -v.z(),  v.y(),
          v.z(),    0.0, -v.x(),
         -v.y(),  v.x(),   0.0;
    return S;
}

// updates the nominal state with the error-state correction dx
void KalmanFilter::injectErrorState(const VectorXd& dx)
{
    x.segment<3>(0) += dx.segment<3>(0); // p = p + dp
    x.segment<3>(3) += dx.segment<3>(3); // v = v + dv

    Vector3d dtheta = dx.segment<3>(6);
    Quaterniond dq(
        1.0,
        0.5 * dtheta.x(),
        0.5 * dtheta.y(),
        0.5 * dtheta.z()
    );

    Quaterniond q(x(6), x(7), x(8), x(9));
    q = q * dq;
    q.normalize();
    x.segment<4>(6) << q.w(), q.x(), q.y(), q.z();

    x.segment<3>(10) += dx.segment<3>(9);  // accel_bias
}
