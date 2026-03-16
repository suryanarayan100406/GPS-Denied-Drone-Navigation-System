/**
 * @file pid_controller.cpp
 * @brief Cascade PID Controller for GPS-Denied Drone
 * 
 * Outer Loop: Altitude (Z) -> Target Thrust
 * Outer Loop: Position (X, Y) -> Target Attitude (Roll, Pitch)
 * Inner Loop: Attitude (Roll, Pitch, Yaw) -> Motor Mixing
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <algorithm>

using std::placeholders::_1;

class PIDController : public rclcpp::Node {
public:
    PIDController() : Node("pid_controller") {
        // --- Parameters Initialization ---
        // Altitude PID Gains
        this->declare_parameter("kp_alt", 2.0);
        this->declare_parameter("ki_alt", 0.05);
        this->declare_parameter("kd_alt", 1.0);
        
        // Attitude PID Gains (Roll/Pitch)
        this->declare_parameter("kp_att", 1.5);
        this->declare_parameter("ki_att", 0.01);
        this->declare_parameter("kd_att", 0.2);
        
        // Yaw PID Gains
        this->declare_parameter("kp_yaw", 1.0);
        this->declare_parameter("ki_yaw", 0.0);
        this->declare_parameter("kd_yaw", 0.1);

        // --- Publishers and Subscribers ---
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&PIDController::imu_callback, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PIDController::odom_callback, this, _1));
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&PIDController::cmd_vel_callback, this, _1));
            
        motor_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_mix", 10);
        pid_telemetry_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pid_telemetry", 10);
        
        // --- Control Loop Timer (50Hz) ---
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&PIDController::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "Cascade PID Controller Initialized.");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Store orientation and angular velocity for Inner Loop
        // Simplified mapping for the sake of structure
        current_roll_rate_ = msg->angular_velocity.x;
        current_pitch_rate_ = msg->angular_velocity.y;
        current_yaw_rate_ = msg->angular_velocity.z;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Store current altitude for Outer Loop
        current_altitude_ = msg->pose.pose.position.z;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Accept navigation targets from Nav2
        target_vel_x_ = msg->linear.x;
        target_vel_y_ = msg->linear.y;
        target_yaw_rate_ = msg->angular.z;
        // Adjust target altitude based on linear.z
        target_altitude_ += msg->linear.z * 0.02; // Integrating Z commanded velocity
    }

    void control_loop() {
        // ------------------------------------------
        // Gain Scheduling based on Altitude Zone
        // ------------------------------------------
        double alt_kp = this->get_parameter("kp_alt").as_double();
        if (current_altitude_ < 0.5) {
            alt_kp *= 1.2; // Increase gain in ground effect zone
        }

        // ------------------------------------------
        // Outer Loop Z: Altitude to Base Thrust
        // ------------------------------------------
        double alt_error = target_altitude_ - current_altitude_;
        alt_integral_ += alt_error * 0.02;
        // Anti-windup
        alt_integral_ = std::clamp(alt_integral_, -2.0, 2.0);
        double alt_derivative = (alt_error - prev_alt_error_) / 0.02;
        prev_alt_error_ = alt_error;
        
        double base_thrust = (alt_kp * alt_error) + 
                             (this->get_parameter("ki_alt").as_double() * alt_integral_) + 
                             (this->get_parameter("kd_alt").as_double() * alt_derivative) + 
                             9.81; // Add hover offset (gravity compensation)

        // ------------------------------------------
        // Publish Telemetry
        // ------------------------------------------
        auto telemetry_msg = std_msgs::msg::Float32MultiArray();
        telemetry_msg.data = {
            static_cast<float>(alt_error),
            static_cast<float>(alt_integral_),
            static_cast<float>(alt_derivative)
        };
        pid_telemetry_pub_->publish(telemetry_msg);

        // ------------------------------------------
        // Motor Mixing mapping would go here
        // ------------------------------------------
        auto motor_msg = std_msgs::msg::Float32MultiArray();
        motor_msg.data = {
            static_cast<float>(base_thrust), // M1
            static_cast<float>(base_thrust), // M2
            static_cast<float>(base_thrust), // M3
            static_cast<float>(base_thrust)  // M4
        };
        motor_pub_->publish(motor_msg);
    }

    // State Variables
    double current_altitude_ = 0.0;
    double current_roll_rate_ = 0.0;
    double current_pitch_rate_ = 0.0;
    double current_yaw_rate_ = 0.0;

    // Target Variables
    double target_altitude_ = 1.0; 
    double target_vel_x_ = 0.0;
    double target_vel_y_ = 0.0;
    double target_yaw_rate_ = 0.0;

    // PID Errors
    double prev_alt_error_ = 0.0;
    double alt_integral_ = 0.0;

    // ROS2 Components
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pid_telemetry_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting PID Controller Node...");
    auto node = std::make_shared<PIDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
