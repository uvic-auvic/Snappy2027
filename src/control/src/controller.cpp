// this file will have all the controller logic for the submarine
// Motor command publisher — publishes Float32MultiArray on /motor_cmd
// data[0] = MotorSelect (0–255, cast to uint8 on the STM32 side) 11111111 (255) = all 8 motors
// data[1] = Speed       (–100.0 to 100.0)

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <utility>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "snappy_interfaces/msg/task.hpp"
#include "snappy_interfaces/msg/pose.hpp"
#include "snappy_interfaces/msg/thruster_command.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <queue>
#include <vector>

#include "pid.hpp"
#include "thruster_allocator.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Controller : public rclcpp::Node {
    public:
        Controller() : Node("controller"),
            pid_x_(0.0f, 0.0f, 0.0f),
            pid_y_(0.0f, 0.0f, 0.0f),
            pid_z_(0.0f, 0.0f, 0.0f),
            pid_roll_(0.0f, 0.0f, 0.0f),
            pid_pitch_(0.0f, 0.0f, 0.0f),
            pid_yaw_(0.0f, 0.0f, 0.0f)
         {
             declare_parameter("target_position", std::vector<double>{0.0, 0.0, 1.0});
             declare_parameter("target_roll", 0.0);
             declare_parameter("target_pitch", 0.0);
             declare_parameter("target_yaw", 0.0);
             declare_parameter("kill_timeout_s", 30.0);
             // 1. Declare the PID parameters with fallback defaults
             declare_parameter("pid_x", std::vector<double>{0.0, 0.0, 0.0});
             declare_parameter("pid_y", std::vector<double>{0.0, 0.0, 0.0});
             declare_parameter("pid_z", std::vector<double>{0.0, 0.0, 0.0});
             declare_parameter("pid_roll", std::vector<double>{0.0, 0.0, 0.0});
             declare_parameter("pid_pitch", std::vector<double>{0.0, 0.0, 0.0});
             declare_parameter("pid_yaw", std::vector<double>{0.0, 0.0, 0.0});

             // 2. Retrieve the arrays from the YAML file
             std::vector<double> px = get_parameter("pid_x").as_double_array();
             std::vector<double> py = get_parameter("pid_y").as_double_array();
             std::vector<double> pz = get_parameter("pid_z").as_double_array();
             std::vector<double> proll = get_parameter("pid_roll").as_double_array();
             std::vector<double> ppitch = get_parameter("pid_pitch").as_double_array();
             std::vector<double> pyaw = get_parameter("pid_yaw").as_double_array();

             // 3. Re-instantiate the PID objects with the live YAML values
             pid_x_ = PID(px[0], px[1], px[2]);
             pid_y_ = PID(py[0], py[1], py[2]);
             pid_z_ = PID(pz[0], pz[1], pz[2]);
             pid_roll_ = PID(proll[0], proll[1], proll[2]);
             pid_pitch_ = PID(ppitch[0], ppitch[1], ppitch[2]);
             pid_yaw_ = PID(pyaw[0], pyaw[1], pyaw[2]);

             std::vector<double> target_position_vec = get_parameter("target_position").as_double_array();
             target_position = Eigen::Vector3d(target_position_vec[0], target_position_vec[1], target_position_vec[2]);

             double roll = get_parameter("target_roll").as_double();
             double pitch = get_parameter("target_pitch").as_double();
             double yaw = get_parameter("target_yaw").as_double();

             current_position = Eigen::Vector3d(0.0, 0.0, 0.0);
             current_orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

             flag_ = 0;

             target_orientation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

            // Configuration of motors on the AUV
            // Rows: Fx, Fy, Fz, Tx, Ty, Tz
            // Columns: Thrusters
            configuration = Eigen::MatrixXd(6, 8);
            configuration <<0,    0,    1,    0,    0,    0,    1,    0,
-1,    0,    0,    0,    1,    0,    0,    0,
0,    1,    0,    1,    0,    1,    0,    1,
0.1301,    0.1653,    0,    0.1653,    -0.1301,    -0.1648,    0,    -0.1648,
0,    0.3024,    -0.0158,    -0.2977,    0,    -0.2977,    -0.0159,    0.3024,
-0.3041,    0,    -0.2739,    0,    -0.3121,    0,    0.2734,    0; 

            // // Allocate thrusters based on the configuration matrix
            // // Blue Robotics T200 thrusters can achieve ~5.0 kgf backwards and 5.0 kgf forwards
            thruster_allocator = ThrusterAllocator(configuration, -4.0, 5.0);

            //publish motor command
            // Match the STM32 micro-ROS subscription: same type AND best-effort QoS.
            motor_publisher_ = create_publisher<snappy_interfaces::msg::ThrusterCommand>(
                "/motor_cmd", rclcpp::QoS(10).best_effort());

            const double kill_timeout_s = get_parameter("kill_timeout_s").as_double();
            if (kill_timeout_s <= 0.0) {
                throw std::invalid_argument("kill_timeout_s must be greater than zero");
            }
            kill_deadline_ = std::chrono::steady_clock::now()
                + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(kill_timeout_s));
            RCLCPP_WARN(
                get_logger(),
                "Thruster kill timer armed: all thrusters will stop in %.1f seconds",
                kill_timeout_s);

            // Publish state status to planner
            status_publisher_ = this->create_publisher<std_msgs::msg::String>("/controller/status", 10);

            // Publish trajectory vectors to state estimator
            trajectory_publisher_ = this->create_publisher<snappy_interfaces::msg::Pose>("/controller/trajectory", 10);

            // Receive tasks from planner. Transient local matches the
            // planner's latched publisher, so the current task is delivered
            // even if this node starts after it was published.
            task_subscription_ = this->create_subscription<snappy_interfaces::msg::Task>(
                "/planner/task", rclcpp::QoS(1).transient_local(),
                std::bind(&Controller::task_callback, this, _1));

            // Ack completed task seqs back to the planner
            task_done_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
                "/controller/task_done", 10);

            // Receive states from state estimator
            state_subscription_ = this->create_subscription<snappy_interfaces::msg::Pose>(
               "state_estimator/state", 10, std::bind(&Controller::state_callback, this, _1));

             //Receive dpeth value from pressure node
             //depth_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
              //"depth_data", 10, std::bind(&Controller::depth_callback, this, _1));

            //get yaw info from the imu
            // imu_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            //     "/filter/euler", 200, std::bind(&Controller::imu_callback, this, _1));
            //currently state estimator does not publish state. maybe parse through IMU..

            // imu_d455_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            //   "/d455/imu", 10, std::bind(&Controller::imu_d455_subscription_callback, this, _1));

            // Receive DVL pose
            // dvl_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            //     "/waterlinked_dvl_driver/odom", 10, std::bind(&Controller::dvl_callback, this, _1));

            // Publish every 100ms ??
            //status_timer_ = this->create_wall_timer(
            //    100ms, std::bind(&Controller::status_callback, this));
            //trajectory_timer_ = this->create_wall_timer(
            //    100ms, std::bind(&Controller::trajectory_callback, this));


            timer_ = this->create_wall_timer(20ms, std::bind(&Controller::timer_callback, this));
            //RCLCPP_INFO(this->get_logger(), "Timer Node started");

        }

    private:
        void publish_zero_thrust() {
            snappy_interfaces::msg::ThrusterCommand msg{};
            msg.header.stamp = now();
            msg.thruster_mask = 255;
            msg.thrust_pct.fill(0);
            motor_publisher_->publish(msg);
        }

        void timer_callback() {
            if (kill_latched_ || std::chrono::steady_clock::now() >= kill_deadline_) {
                if (!kill_latched_) {
                    kill_latched_ = true;
                    RCLCPP_ERROR(
                        get_logger(),
                        "Thruster kill timer expired; all motor commands are now latched at zero");
                }
                publish_zero_thrust();
                return;
            }

            std::pair<Eigen::Vector3d, Eigen::Quaterniond> trajectory = generate_trajectory();

            // Ack the current task once its tracking error (position error
            // norm + orientation error angle) drops under error_threshold.
            if (current_task_ && in_progress_) {
                double error = trajectory.first.norm()
                    + std::abs(Eigen::AngleAxisd(trajectory.second).angle());
		//RCLCPP_INFO(this->get_logger(), "Error: %0.3f", error);
                if (error < error_threshold) {
                    auto done_msg = std_msgs::msg::Int32();
                    done_msg.data = current_task_->seq;
                    task_done_publisher_->publish(done_msg);
                    in_progress_ = false;
                    //RCLCPP_INFO(this->get_logger(), "Task %d done (error %.4f)",
              //          current_task_->seq, error);
                }
            }

            Eigen::Vector3d euler = trajectory.second.toRotationMatrix().eulerAngles(0, 1, 2);

            //this is used to calculate the w value needed
            //RCLCPP_INFO(this->get_logger(), "Trajectory [X, Y, Z, R, P, Y]: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
               // trajectory.first[0], trajectory.first[1], trajectory.first[2], euler[0], euler[1], euler[2]);

            // this is w aka the wrench value, it is a 6x1 vector
            Eigen::VectorXd wrench = generate_wrench(trajectory.first, trajectory.second);
            //RCLCPP_INFO(this->get_logger(), "Wrench     [X, Y, Z, Y, P, R]: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
              //  wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]);

            // this calculates the thrust that gets sent to every thruster
            std::vector<int8_t> allocation = allocate_thrusters(wrench);
            //RCLCPP_INFO(this->get_logger(), "Allocation [ Thrusters  1-8 ]: %d, %d, %d, %d, %d, %d, %d, %d",
              //  allocation[0], allocation[1], allocation[2], allocation[3], allocation[4], allocation[5], allocation[6], allocation[7]);

            snappy_interfaces::msg::ThrusterCommand msg{};
            msg.thruster_mask = 255;
            for (size_t i = 0; i < msg.thrust_pct.size(); ++i) {
                msg.thrust_pct[i] = allocation[i];
            }
            motor_publisher_->publish(msg);
        }

        // Given the target and current poses in global space, determine the vector between them in local space
        std::pair<Eigen::Vector3d, Eigen::Quaterniond> generate_trajectory() {
            //Eigen::Vector3d current_position(0.0, 0.0, current_depth);
            //Eigen::Quaterniond current_orientation;
            //current_orientation =
                //Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                //Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                // Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

            //Eigen::Vector3d relative_position = current_orientation * (target_position - current_position);
            Eigen::Quaterniond relative_orientation = target_orientation * current_orientation.inverse();

	    if (relative_orientation.w() < 0) {
		relative_orientation.coeffs() *= -1;
	    }

            std::pair<Eigen::Vector3d, Eigen::Quaterniond> result;

            result.first = target_position - current_position;//relative_position;
            result.second = relative_orientation;//relative_orientation;

            return result;
        }

        // Given the relative position and orientation of the target, generate the wrench to move in that direction
        Eigen::VectorXd generate_wrench(Eigen::Vector3d relative_position, Eigen::Quaterniond relative_orientation) {
            Eigen::AngleAxisd aa(relative_orientation);

            Eigen::Vector3d trajectory = aa.angle() * aa.axis();  
		
	    //Eigen::Vector3d euler = relative_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
            
	    float roll = trajectory[0];
            float pitch = trajectory[1];
            float yaw = trajectory[2];//2.0f * std::atan2(relative_orientation.z(), relative_orientation.w());//euler[2];


	    /*
	    double pi = EIGEN_PI;

	    if (roll > pi) {
		roll -= 2*pi;
	    }

	    if (pitch > pi) {
		pitch -= 2*pi;
	    }

	    if (yaw > pi) {
		yaw -= 2*pi;
	    }
	    */
            //double pi = EIGEN_PI;

            //if(yaw < -pi) {
            //    yaw += 2*pi;
            //}else if (yaw > pi) {
            //    yaw -= 2*pi;
            //}

            //if(pitch < -pi) {
            //    pitch += 2*pi;
            //}else if (pitch > pi) {
            //    pitch -= 2*pi;
            //}

            //if(roll < -pi) {
            //    roll += 2*pi;
            //}else if (roll > pi) {
            //    roll -= 2*pi;
            //}

            // Relative position of AUV from target is negated
            float thrust_x = pid_x_.update(-relative_position[0]);
            float thrust_y = pid_y_.update(-relative_position[1]);
            float thrust_z = pid_z_.update(-relative_position[2]);

            float thrust_yaw = pid_yaw_.update(-yaw);
            float thrust_pitch = pid_pitch_.update(-pitch);
            float thrust_roll = pid_roll_.update(-roll);

            // Create wrench vector to be returned
            //float thrust_pitch = pid_pitch_.update(-pitch);
            //float thrust_roll = pid_roll_.update(-roll);

            // Create wrench vector to be returned
            Eigen::VectorXd wrench(6);
            wrench << thrust_x, thrust_y, thrust_z + 5.0f, thrust_roll, thrust_pitch, thrust_yaw;

            return wrench;
        }

        std::vector<int8_t> allocate_thrusters(Eigen::VectorXd wrench) {
            Eigen::VectorXd allocation = thruster_allocator.allocate(wrench);

            std::vector<int8_t> allocation_array;

            // Turn the allocation vector into an array of int8_t for the motorboard
            double* allocation_data = allocation.data();

            for (int i = 0; i < allocation.size(); i++) {
                double force = allocation_data[i];
                allocation_array.push_back(thrust_to_speed(force)); // Convert max = 5 (kgf) to max = 100 (speed)
            }

            return allocation_array;
        }

        // Determines the speed (-100, 100) from thrust (-4, 5)
        int8_t thrust_to_speed(double thrust) {
            if (thrust > 0) {
                return static_cast<int8_t>(thrust * 20.0);
            } else {
                return static_cast<int8_t>(thrust * 25.0);
            }
        }

        void set_x(float x, bool local = true) {
            if (local) {
                Eigen::Quaterniond body_rotation;
                body_rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

                Eigen::Vector3d local_target = body_rotation * target_position;
                local_target[0] = x;
                target_position = body_rotation.inverse() * local_target;
            } else {
                target_position[0] = x;
            }
        }

        void set_y(float y, bool local = true) {
            if (local) {
                Eigen::Quaterniond body_rotation;
                body_rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

                Eigen::Vector3d local_target = body_rotation * target_position;
                local_target[1] = y;
                target_position = body_rotation.inverse() * local_target;
            } else {
                target_position[1] = y;
            }
        }

        void set_z(float z, bool local = false) {
            if (local) {
                Eigen::Quaterniond body_rotation;
                body_rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

                Eigen::Vector3d local_target = body_rotation * target_position;
                local_target[2] = z;
                target_position = body_rotation.inverse() * local_target;
            } else {
                target_position[2] = z;
            }
        }

        void set_yaw(float yaw, bool absolute = false) {
            Eigen::Vector3d euler = target_orientation.toRotationMatrix().eulerAngles(0, 1, 2);

            if (absolute) {
                euler[2] = yaw;
            } else {
                euler[2] += yaw;
            }

            target_orientation =
                Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());

            // Normalize to prevent numerical drift
            target_orientation.normalize();
        }

        void set_pitch(float pitch, bool absolute = false) {
            Eigen::Vector3d euler = target_orientation.toRotationMatrix().eulerAngles(0, 1, 2);

            if (absolute) {
                euler[1] = pitch;
            } else {
                euler[1] += pitch;
            }

            target_orientation =
                Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());

            // Normalize to prevent numerical drift
            target_orientation.normalize();
        }

        void set_roll(float roll, bool absolute = false) {
            Eigen::Vector3d euler = target_orientation.toRotationMatrix().eulerAngles(0, 1, 2);

            if (absolute) {
                euler[0] = roll;
            } else {
                euler[0] += roll;
            }

            target_orientation =
                Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());

            // Normalize to prevent numerical drift
            target_orientation.normalize();
        }

        void depth_callback(const std_msgs::msg::Float32 & msg) {
	    if(msg.data < 0){
		return;
	    }else {
		    current_position[2] = msg.data;
	    }
        }

        void imu_callback(const geometry_msgs::msg::Vector3Stamped & msg) {
            //RCLCPP_INFO(this->get_logger(), "Received IMU data: roll=%.2f, pitch=%.2f, yaw=%.2f", msg.vector.x, msg.vector.y, msg.vector.z);
            //rollroll
    	    roll = msg.vector.x * EIGEN_PI / 180;
            //pitch
            pitch = msg.vector.y * EIGEN_PI / 180;
            //yaw
            yaw = msg.vector.z * EIGEN_PI / 180;



    	    if (flag_ == 0) {
                flag_ = 1;
    		// // first reference of yaw
    		//pid_yaw_.set_target(yaw);
                set_yaw(yaw);
          }
        }


// pose:
//   pose:
//     position:
//       x: 0.21390331654677197
//       y: 0.15011672381170452
//       z: 0.031298197173737205
//     orientation:
//       x: 0.5008356545579112
//       y: -0.4569927882399792
//       z: 0.6090910863871614
//       w: -0.41149639986749026

        void dvl_callback(const nav_msgs::msg::Odometry & msg) {
            x = msg.pose.pose.position.x;
            y = msg.pose.pose.position.y;

            float orientationX = msg.pose.pose.orientation.x;
            float orientationY = msg.pose.pose.orientation.y;
            float orientationZ = msg.pose.pose.orientation.z;
            float orientationW = msg.pose.pose.orientation.w;

            Eigen::Quaterniond q(orientationW, orientationX, orientationY, orientationZ);
            q.normalize();
            Eigen::Matrix3d R = q.toRotationMatrix();

            Eigen::Vector3d euler = R.eulerAngles(2,1,0);

            //yaw = euler[0];
       	    //if (dvl_first_ == 0) {
          		//dvl_first_ = 1;
          		// first reference of yaw
          		//set_yaw(yaw);
          		//pid_y_.set_target(y);
          		//pid_x_.set_target(x);
            //}
        }

        void task_callback(const snappy_interfaces::msg::Task & msg) {
            target_position.x() = msg.x;
            target_position.y() = msg.y;
//	    target_position.z() = msg.z;
            // Depth is pinned: target_position[2] comes from the
            // target_position param at construction — never from a task.

            target_orientation = Eigen::AngleAxisd(msg.roll, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(msg.pitch, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(msg.yaw, Eigen::Vector3d::UnitZ());

            current_task_ = msg;
            in_progress_ = true;
            //RCLCPP_INFO(this->get_logger(), "Received task %d: x=%.2f y=%.2f roll=%.2f pitch=%.2f yaw=%.2f",
              //  msg.seq, msg.x, msg.y, msg.roll, msg.pitch, msg.yaw);
        }

        void state_callback(const snappy_interfaces::msg::Pose & msg) {
           /* current_position = Eigen::Vector3d(
                 msg.position.x,
                 msg.position.y,
                 msg.position.z);
            */ 
           
            current_position.x() = target_position.x();
            current_position.y() = target_position.y();
            current_position.z() = msg.position.z;    

            current_orientation = Eigen::Quaterniond(
                msg.orientation.w,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z);
        }

        rclcpp::Publisher<snappy_interfaces::msg::ThrusterCommand>::SharedPtr motor_publisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
        rclcpp::Publisher<snappy_interfaces::msg::Pose>::SharedPtr trajectory_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr task_done_publisher_;
        rclcpp::Subscription<snappy_interfaces::msg::Task>::SharedPtr task_subscription_;
        rclcpp::Subscription<snappy_interfaces::msg::Pose>::SharedPtr state_subscription_;
    	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_subscription_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_subscription_;
    	rclcpp::TimerBase::SharedPtr status_timer_;
        rclcpp::TimerBase::SharedPtr trajectory_timer_;

        PID pid_x_;
        PID pid_y_;
        PID pid_z_;
        PID pid_roll_;
        PID pid_pitch_;
        PID pid_yaw_;

        std::queue<snappy_interfaces::msg::Task> tasks_;
        std::optional<snappy_interfaces::msg::Task> current_task_;
        bool in_progress_ = false; // set to true in task_callback, reset to false once the task is acked

        geometry_msgs::msg::Point position_current_;
        std::optional<geometry_msgs::msg::Point> position_target_;
        geometry_msgs::msg::Quaternion orientation_current_;
        std::optional<geometry_msgs::msg::Quaternion> orientation_target_;
        const double error_threshold = 0.2;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::chrono::steady_clock::time_point kill_deadline_;
        bool kill_latched_ = false;

        int flag_;

        int dvl_first_;
        int timer_first_;
        int state_;

    	float pitch = 0.0f;
    	float yaw = 0.0f;
    	float roll = 0.0f;
    	float x = 0.0f;
    	float y = 0.0f;
    	float current_depth = 0.0f;

        Eigen::Vector3d current_position;
        Eigen::Quaterniond current_orientation;

    	Eigen::Vector3d target_position;
    	Eigen::Quaterniond target_orientation;

    	Eigen::MatrixXd configuration;
    	ThrusterAllocator thruster_allocator;
};



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
