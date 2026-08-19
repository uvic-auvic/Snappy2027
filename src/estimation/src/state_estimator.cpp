// this file is going to contain the code for the state estimator
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

#include "snappy_interfaces/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "snappy_estimation/kalman.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class StateEstimator : public rclcpp::Node
{
public:
    // Output files
    std::fstream imu1_file;
    std::fstream imu2_file;
    std::fstream depth_file;
    std::fstream dvl_file;
    std::fstream kalman_file;

    // name files different every run
    std::string time_now = std::to_string(rclcpp::Clock().now().nanoseconds());
    std::string imu1_filename   = "imu1_"   + time_now + ".csv";
    std::string imu2_filename   = "imu2_"   + time_now + ".csv";
    std::string depth_filename  = "depth_"  + time_now + ".csv";
    std::string dvl_filename    = "dvl_"    + time_now + ".csv";
    std::string kalman_filename = "kalman_" + time_now + ".csv";


    //change in time between imu messages
    double last_time_imu1_sec_ = 0.0;
    double last_time_imu2_sec_ = 0.0;
    KalmanFilter kf;


    StateEstimator() : Node("state_estimator")
    {
        //RCLCPP_INFO(this->get_logger(), "State Estimator starting...");

        // imu1_file: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        // imu2_file: accel_x, accel_y, accel_z, quat_x, quat_y, quat_z, quat_w

        imu1_file.open(imu1_filename, std::fstream::out);
        imu2_file.open(imu2_filename, std::fstream::out);
        depth_file.open(depth_filename, std::fstream::out);
        dvl_file.open(dvl_filename, std::fstream::out);
        kalman_file.open(kalman_filename, std::fstream::out); // output of kalman filter

        // Create QoS profile matching RealSense camera publisher
        // RealSense uses: Best Effort reliability + Volatile durability
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()  // Use Best Effort (not Reliable)
            .durability_volatile();  // Volatile durability

        // Primary: unified IMU topic (gyro + accel combined) from camera
        imu_sub1_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/d455/camera/camera/imu",
            qos,
            std::bind(&StateEstimator::imu1_callback, this, _1));

        // From on-board IMU (Xsens MTi driver)
        imu_sub2_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data",
            qos,
            std::bind(&StateEstimator::imu2_callback, this, _1));

        depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/depth_data",
            10,
            std::bind(&StateEstimator::depth_callback, this, _1));


        dvl_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/waterlinked_dvl_driver/odom", 10,
            std::bind(&StateEstimator::dvl_callback, this, _1));



        accel_bias_init_ << 0.0, 0.0, 0.0;

        // Default state before frame initialization fires (filter not yet active)
        VectorXd x0 = VectorXd::Zero(13);
        x0(6) = 1.0;  // identity quaternion [w, x, y, z]
        x0.segment<3>(10) = accel_bias_init_;

        // Error-state covariance: [δp(3), δv(3), δθ(3), δb_a(3)] = 12 dims

        P_init_ = MatrixXd::Identity(12, 12);
        P_init_.block<3,3>(0,0) *= 0.01;  // position uncertainty of initial guess
        P_init_.block<3,3>(3,3) *= 0.01;   // velocity uncertainty of initial guess
        P_init_.block<3,3>(6,6) *= 0.01;  // orientation error uncertainty of initial guess
        P_init_.block<3,3>(9,9) *= 0.01;  // accel bias uncertainty of initial guess

        // Q: 6x6 — two active noise sources: IMU2 accel noise and accel bias random walk
        MatrixXd Q_init = MatrixXd::Zero(6, 6);
        Q_init.block<3,3>(0,0) = Matrix3d::Identity() * 0.01;   // accel noise → velocity growth
        Q_init.block<3,3>(3,3) = Matrix3d::Identity() * 1e-5;   // accel bias random walk (fast enough to converge in-run)
        kf.setProcessNoise(Q_init);

        // These show how much we trust each sensor. Smaller values = more trust, larger values = less trust.
        MatrixXd R_imu1_init = MatrixXd::Identity(3, 3) * 1.0;   // IMU1 gravity update noise (low trust)
        MatrixXd R_depth_init = MatrixXd::Identity(1, 1) * 2.5e-4; // depth sensor noise
        MatrixXd R_dvl_vel_ = MatrixXd::Identity(3, 3) * 0.01; // DVL velocity noise

        kf.setIMU1MeasurementNoise(R_imu1_init);
        kf.setDepthMeasurementNoise(R_depth_init);
        kf.setDVLVelocityMeasurementNoise(R_dvl_vel_);

        q_imu2_to_body_node_ = Quaterniond(0, 0, 1, 0);  // w, x, y, z; 180° around y
        kf.setIMU2ToBodyRotation(q_imu2_to_body_node_);

        kf.setIMU1ToBodyRotation(Quaterniond(0, 1, 0, 0));

        kf.reset(x0, P_init_);



        publisher_ = this->create_publisher<snappy_interfaces::msg::Pose>(
            "state_estimator/state", 10
        );

        //RCLCPP_INFO(this->get_logger(), "State Estimator subscribed to:");
        //RCLCPP_INFO(this->get_logger(), "  - /camera/camera/imu");
        //RCLCPP_INFO(this->get_logger(), "  - /imu/data");
        //RCLCPP_INFO(this->get_logger(), "  - depth_data");
        //RCLCPP_INFO(this->get_logger(), "  - /waterlinked_dvl_driver/odom");
        //RCLCPP_INFO(this->get_logger(), "Waiting for IMU data...");

        // Timer to check status
        check_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&StateEstimator::check_status, this));
    }

private:
    void imu1_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!imu_received_) {
            //RCLCPP_INFO(this->get_logger(), "✅ First IMU1 message received!");
            imu_received_ = true;
        }
        const double now_sec = rclcpp::Time(msg->header.stamp).seconds();
        if (!imu1_initialized_) {
            imu1_initialized_ = true;
            last_time_imu1_sec_ = now_sec;
        }
        imu_count_++;

        const double dt = now_sec - last_time_imu1_sec_;
        last_time_imu1_sec_ = now_sec;
        if(dt > 0.001 && frame_initialized_)
        {
            Vector3d accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            kf.updateIMU1(accel, gyro, dt);
        }


        // Print combined IMU data every 20 messages
        if (imu_count_ % 20 == 0) {
            //RCLCPP_INFO(this->get_logger(),
              //  "[IMU 1] Gyro: [%.3f, %.3f, %.3f] rad/s | Accel: [%.3f, %.3f, %.3f] m/s²",
                //msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
                //msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        }

        // Write all data to file
        imu1_file << rclcpp::Time(msg->header.stamp).nanoseconds() << ","
                << msg->linear_acceleration.x << ","
                << msg->linear_acceleration.y << ","
                << msg->linear_acceleration.z << ","
                << msg->angular_velocity.x << ","
                << msg->angular_velocity.y << ","
                << msg->angular_velocity.z << std::endl;


    }

    void imu2_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        //pose message
        auto pose = snappy_interfaces::msg::Pose();


        if (!imu_received_) {
            //RCLCPP_INFO(this->get_logger(), "✅ First IMU2 message received!");
            imu_received_ = true;
        }
        imu2_count_++;
        const double now_sec = rclcpp::Time(msg->header.stamp).seconds();

        if (!imu2_initialized_) {
            imu2_initialized_ = true;
            last_time_imu2_sec_ = now_sec;

        }

        // wait until we have received the first IMU2 message to initialize the filter to set the right starting frame
        if (!init_imu2_ready_) {
            init_imu2_quat_ = Quaterniond(msg->orientation.w, msg->orientation.x,
                                          msg->orientation.y, msg->orientation.z);
            init_imu2_ready_ = true;
            
            try_initialize();
        }

        
        const double dt = now_sec - last_time_imu2_sec_;
        last_time_imu2_sec_ = now_sec;

        if (!frame_initialized_) {
            //RCLCPP_INFO(this->get_logger(), "Waiting for frame initialization...");
            return;
        }

        if (dt > 0.001 && frame_initialized_)
        {
            Vector3d accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            Quaterniond quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
            kf.predict(accel, quat, dt);
        }

        pose.position.x = kf.getPosition().x();
        pose.position.y = kf.getPosition().y();
        pose.position.z = kf.getPosition().z();
        pose.orientation.w = kf.getOrientation().w();
        pose.orientation.x = kf.getOrientation().x();
        pose.orientation.y = kf.getOrientation().y();
        pose.orientation.z = kf.getOrientation().z();

        publisher_->publish(pose);
        
        // Print combined IMU data every 20 messages
        if (imu2_count_ % 20 == 0) {
            //RCLCPP_INFO(this->get_logger(),
             //   "[IMU 2] Orient: [%.3f, %.3f, %.3f, %.3f] rad | Accel: [%.3f, %.3f, %.3f] m/s²",
              //  msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
              //  msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

//        RCLCPP_INFO(this->get_logger(), "  State Estimate: Pos=[%.2f, %.2f, %.2f] Vel=[%.2f, %.2f, %.2f] Ori=[%.2f, %.2f, %.2f, %.2f], Pitch=%.2f, Roll=%.2f, Yaw=%.2f",
  //          kf.getPosition().x(), kf.getPosition().y(), kf.getPosition().z(),
    //        kf.getVelocity().x(), kf.getVelocity().y(), kf.getVelocity().z(),
      //      kf.getOrientation().x(), kf.getOrientation().y(), kf.getOrientation().z(), kf.getOrientation().w(),
        //    getPitch(kf.getOrientation()), getRoll(kf.getOrientation()), getYaw(kf.getOrientation()));
        }


        // Write all data to file
        imu2_file << rclcpp::Time(msg->header.stamp).nanoseconds() << ","
                << msg->linear_acceleration.x << ","
                << msg->linear_acceleration.y << ","
                << msg->linear_acceleration.z << ","
                << msg->orientation.x << ","
                << msg->orientation.y << ","
                << msg->orientation.z << ","
                << msg->orientation.w << std::endl;

         kalman_file << rclcpp::Time(msg->header.stamp).nanoseconds() << ","
                    << kf.getPosition().x() << ","
                    << kf.getPosition().y() << ","
                    << kf.getPosition().z() << ","
                    << kf.getVelocity().x() << ","
                    << kf.getVelocity().y() << ","
                    << kf.getVelocity().z() << ","
                    << kf.getOrientation().w() << ","
                    << kf.getOrientation().x() << ","
                    << kf.getOrientation().y() << ","
                    << kf.getOrientation().z() << std::endl;

    }

   void depth_callback(const std_msgs::msg::Float32 & msg)
    {
        float depth_data = msg.data;
        //RCLCPP_INFO(this->get_logger(), "Depth data received: %f", depth_data);

        // Buffer first depth to seed z0 for world frame initialization
        if (!init_depth_ready_) {
            init_depth_ = depth_data;
            init_depth_ready_ = true;
            try_initialize();
        }

        if (!frame_initialized_) {
            //RCLCPP_INFO(this->get_logger(), "Waiting for frame initialization...");
            return;
        }
        kf.updateDepth(depth_data);

        // Write depth data to file (std_msgs::Float32 carries no header stamp)
        depth_file << depth_data << std::endl;
    }

    void dvl_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!dvl_received_) {
            //RCLCPP_INFO(this->get_logger(), "First DVL message received!");
            dvl_received_ = true;
        }
        dvl_count_++;
        if (!frame_initialized_) return;

        const float epsilon = 1e-6f;
        


        Vector3d v_body(-msg->twist.twist.linear.x,
                        -msg->twist.twist.linear.y,
                        -msg->twist.twist.linear.z);

        if (abs(v_body.x()) < epsilon && abs(v_body.y()) < epsilon && abs(v_body.z()) < epsilon) return; // skip bad samples
        kf.updateDVLVelocity(v_body);

        if (dvl_count_ % 20 == 0) {
            //RCLCPP_INFO(this->get_logger(),
              //  "[DVL] v_body=[%.3f, %.3f, %.3f] m/s | Vel est=[%.2f, %.2f, %.2f]",
                //v_body.x(), v_body.y(), v_body.z(),
                //kf.getVelocity().x(), kf.getVelocity().y(), kf.getVelocity().z());
        }

        // write to dvl file
        dvl_file << rclcpp::Time(msg->header.stamp).nanoseconds() << ","
                 << v_body.x() << ","
                 << v_body.y() << ","
                 << v_body.z() << std::endl;

    }


    void try_initialize()
    {
        if (frame_initialized_ || !init_depth_ready_ || !init_imu2_ready_) return;

        // May need to apply a sin(pitch) to the depth measurement if the vehicle is not level
        double z0 = init_depth_;

        // Initial orientation: yaw = 0 in the mission frame (+x = starting
        // heading). yaw0 below is measured only so it can be cancelled.

        // The Xsens quaternion answers "how is the SENSOR rotated relative to an
        // ENU (z-up) world?" — our filter world is z-DOWN, so BOTH sides of that
        // relationship need fixing before it can be used:
        //   q_world<-body = q_enu_to_world * quat_imu2 * q_imu2_to_body^-1
        //                   (world side)                 (sensor/mount side)
        // The mount flip alone is NOT enough: without the ENU term the at-rest
        // attitude comes out 180° flipped (filter thinks it's upside down) and
        // gravity stops cancelling (+19.6 m/s² phantom accel). Uses the same
        // shared constant predict() fuses with (see kalman.h).
        Quaterniond q_body0 = (KalmanFilter::q_enu_to_world_ * init_imu2_quat_.normalized()
                               * q_imu2_to_body_node_.conjugate()).normalized();


        // Formula from stack overflow:  atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
        const double yaw0 = std::atan2(
            2.0 * (q_body0.w() * q_body0.z() + q_body0.x() * q_body0.y()),
            1.0 - 2.0 * (q_body0.y() * q_body0.y() + q_body0.z() * q_body0.z()));

        // Mission frame: Rz(-yaw0) cancels the starting NED yaw, so world
        // +x = vehicle heading at init, +y right of it, +z down. predict()
        // applies the same rotation to every later IMU2 fusion via
        // kf.q_ned_to_world_, so a constant compass offset cancels everywhere.
        const Quaterniond q_align(AngleAxisd(-yaw0, Vector3d::UnitZ()));
        kf.q_ned_to_world_ = q_align;

        // Startup attitude in the mission frame: yaw exactly 0, roll/pitch as
        // measured (the first IMU2 fusion in predict() would snap them in anyway).
        Quaterniond q0 = (q_align * q_body0).normalized();

        VectorXd x0 = VectorXd::Zero(13);
        x0(2) = z0;           // depth at startup: (0, 0, depth)
        x0(6) = q0.w();       // yaw-zero startup quaternion [w, x, y, z]
        x0(7) = q0.x();
        x0(8) = q0.y();
        x0(9) = q0.z();
        x0.segment<3>(10) = accel_bias_init_;

        kf.reset(x0, P_init_);
        frame_initialized_ = true;
        
        //RCLCPP_INFO(this->get_logger(),
          //  "KF mission frame initialized: pos=(0, 0, %.3f) m, +x = initial heading  [depth=%.3f m, NED yaw0=%.1f deg]",
            //z0, init_depth_, yaw0 * 180.0 / M_PI);
    }

    // RADIANS in — do not pass degrees. ZYX composition q = Rz(yaw)*Ry(pitch)*Rx(roll)
    Quaterniond getQuaternionFromYawPitchRollRadians(double yaw_rad, double pitch_rad, double roll_rad) {
        tf2::Quaternion q_tf;
        q_tf.setRPY(roll_rad, pitch_rad, yaw_rad);  // tf2 argument order is (roll, pitch, yaw), radians
        return Quaterniond(q_tf.w(), q_tf.x(), q_tf.y(), q_tf.z());
    }

    void check_status()
    {
        if (!imu_received_ && !gyro_received_ && !accel_received_) {
            //RCLCPP_WARN(this->get_logger(),
              //  "❌ No IMU data received yet. Total: IMU=%d, Gyro=%d, Accel=%d",
                //imu_count_, gyro_count_, accel_count_);
        } else {
            //RCLCPP_INFO(this->get_logger(),
              //  "✅ Receiving - IMU:%d Gyro:%d Accel:%d msgs",
                //imu_count_, gyro_count_, accel_count_);
        }
    }

    double getYaw(const Quaterniond& q) {
        return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }
    double getPitch(const Quaterniond& q) {
        return std::asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
    }
    double getRoll(const Quaterniond& q) {
        return std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                          1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub1_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub2_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_subscription_;
    rclcpp::Publisher<snappy_interfaces::msg::Pose>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr check_timer_;

    bool imu_received_ = false;
    bool imu1_initialized_ = false;
    bool imu2_initialized_ = false;
    bool gyro_received_ = false;
    bool accel_received_ = false;
    bool dvl_received_ = false;
    int imu_count_ = 0;
    int imu2_count_ = 0;
    int gyro_count_ = 0;
    int accel_count_ = 0;
    int dvl_count_ = 0;

    // Deferred world-frame initialization: wait for first depth + first IMU2
    bool frame_initialized_ = false;
    bool init_depth_ready_  = false;
    bool init_imu2_ready_   = false;
    double init_depth_ = 0.0;
    Quaterniond init_imu2_quat_;
    Quaterniond q_imu2_to_body_node_;      // local copy for try_initialize()
    Vector3d accel_bias_init_ = Vector3d::Zero();
    MatrixXd P_init_;                      // initial error-state covariance, built once in the constructor
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
