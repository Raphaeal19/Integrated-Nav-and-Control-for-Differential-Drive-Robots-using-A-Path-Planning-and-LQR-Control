#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <algorithm>

// State of the mobile robot
struct State
{
    double x;
    double y;
    double theta;

    State() {}
    State(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

// Input of the mobile robot
struct Input
{
    double v;
    double w;

    Input() {}
    Input(double v_, double w_) : v(v_), w(w_) {}
};

class MobileRobotController : public rclcpp::Node
{
public:
    MobileRobotController() : Node("mobile_robot_controller")
    {
        // Subscribe to robot pose topic
        robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MobileRobotController::robotPoseCallback, this, std::placeholders::_1));

        local_pose_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_plan", 12, std::bind(&MobileRobotController::localPoseCallback, this, std::placeholders::_1));

        // Publish control input topic
        control_input_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        A_ << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

        Q_ << 0.8, 0, 0,
            0, 0.8, 0,
            0, 0, 0.8;

        R_ << 0.01, 0,
            0, 0.01;

        desired_state_ = {10, 0.0, 0}; // x, y, and theta
        dt_ = 0.03;
        tolerance = 0.005;
        end_controller = false;
        max_linear_velocity = 5.0;
        max_angular_velocity = M_PI / 2;

        // Start control loop timer
        control_loop_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), std::bind(&MobileRobotController::controlLoopCallback, this));
    }

private:
    // Callback to get current state of robot pose
    void robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        actual_state_ = State(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    }

    Eigen::Matrix<double, 3, 2> getB(double yaw, double dt)
    {
        Eigen::Matrix<double, 3, 2> B; // Input model
        B << std::cos(yaw) * dt, 0,
            std::sin(yaw) * dt, 0,
            0, dt;

        return B;
    }

    void pubVel(double v, double w)
    {
        geometry_msgs::msg::Twist vel;
        vel.linear.x = v;
        vel.angular.z = w;
        control_input_pub_->publish(vel);
    }

    Input LQR(Eigen::Vector3d state_error, Eigen::Matrix3d Q, Eigen::Matrix2d R, Eigen::Matrix3d A, Eigen::Matrix<double, 3, 2> B)
    {
        uint8_t N = 50; // number of iteration
        std::vector<Eigen::MatrixXd> P(N + 1);
        Eigen::MatrixXd Qf = Q;
        P[N] = Qf;

        for (uint8_t i = N; i >= 1; --i)
        {
            auto Y = R + B.transpose() * P[i] * B;
            // Compute the SVD decomposition of Y
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
            // Compute the pseudo-inverse of Y
            Eigen::MatrixXd Yinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();

            P[i - 1] = Q + A.transpose() * P[i] * A - (A.transpose() * P[i] * B) * Yinv * (B.transpose() * P[i] * A);
        }

        std::vector<Eigen::MatrixXd> K(N);
        std::vector<Eigen::Vector2d> u(N);

        for (uint8_t i = 0; i <= N - 1; ++i)
        {
            auto Y = R + B.transpose() * P[i + 1] * B;
            // Compute the SVD decomposition of Y
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
            // Compute the pseudo-inverse of Y
            Eigen::MatrixXd Yinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();

            K[i] = Yinv * B.transpose() * P[i + 1] * A;
            u[i] = -K[i] * state_error;
        }

        Input u_optimal = {u[N - 1](0), u[N - 1](1)};

        return u_optimal; // return optimal control input
    }

    // Callback for control loop timer
    void controlLoopCallback()
    {
        if (end_controller == false)
        { // if robot has not reached goal
            Eigen::Matrix<double, 3, 2> B = getB(actual_state_.theta, dt_);
            Eigen::Vector3d x_actual(actual_state_.x, actual_state_.y, actual_state_.theta);
            Eigen::Vector3d x_desired(desired_state_.x, desired_state_.y, desired_state_.theta);
            state_error_ = x_actual - x_desired;
            // RCLCPP_INFO(rclcpp::get_logger("mobile_robot_controller"), "Actual State: (%f, %f, %f)", actual_state_.x, actual_state_.y, actual_state_.theta);
            // RCLCPP_INFO(rclcpp::get_logger("mobile_robot_controller"), "State Error: (%f, %f, %f)", state_error_[0], state_error_[1], state_error_[2]);

            Input u = LQR(state_error_, Q_, R_, A_, B);
            u.v = std::clamp(u.v, -max_linear_velocity, max_linear_velocity);
            u.w = std::clamp(u.w, -max_angular_velocity, max_angular_velocity);
            double state_error_magnitude = state_error_.norm();
            
            if (u.v < -max_linear_velocity || u.v > max_linear_velocity || double(u.v) != u.v)
            {
                u = Input(0.2, u.w);
            }
            if (u.w < -max_angular_velocity || u.w > max_angular_velocity || double(u.w) != u.w)
            {
                u = Input(u.v, 0.2);
            }
            if (state_error_magnitude > 100)
            {
                u = Input(0.3, 0.3);
            }

            pubVel(u.v, u.w);

            if (state_error_magnitude < tolerance)
            {
                // RCLCPP_INFO(rclcpp::get_logger("mobile_robot_controller"), "Error low, Goal Reached: %f", state_error_magnitude);
                pubVel(0, 0);
                end_controller = true;
            }
        }

        else
        {
            RCLCPP_INFO(rclcpp::get_logger("LQR"), "Goal reached!");
            control_loop_timer_->cancel();
            return;
        }
    }

    void localPoseCallback(const nav_msgs::msg::Path::ConstPtr &msg_local_pose)
    {
        // RCLCPP_INFO(rclcpp::get_logger("mobile_robot_controller"), "Local Pose: (%f, %f, %f)", msg_local_pose->poses[9].pose.position.x, msg_local_pose->poses[9].pose.position.y, msg_local_pose->poses[9].pose.position.z);
        if (end_controller == false)
        {
            tf2::Quaternion quat(msg_local_pose->poses[9].pose.orientation.x, msg_local_pose->poses[9].pose.orientation.y, msg_local_pose->poses[9].pose.orientation.z, msg_local_pose->poses[9].pose.orientation.w);
            tf2::Matrix3x3 mat(quat);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);
            desired_state_ = State(msg_local_pose->poses[9].pose.position.x, msg_local_pose->poses[9].pose.position.y, yaw);
            // RCLCPP_INFO(rclcpp::get_logger("mobile_robot_controller"), "Inside local Desired: (%f, %f, %f)", desired_state_.x, desired_state_.y, desired_state_.theta);
        }
    }

    // Subscriptions and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_input_pub_;

    // Timer for control loop
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    // LQR Matrices
    Eigen::Matrix3d A_; // State model
    Eigen::Matrix3d Q_; // State cost
    Eigen::Matrix2d R_; // Input cost
    Eigen::Vector3d state_error_;

    double dt_;       // sample time
    double tolerance; // goal tolerance
    bool end_controller;
    double max_linear_velocity;
    double max_angular_velocity;

    // Current robot state and desired state
    State actual_state_;
    State desired_state_;
};

int main(int argc, char **argv)
{
    try
    {
        rclcpp::init(argc, argv);
        auto controller = std::make_shared<MobileRobotController>();
        rclcpp::spin(controller);
        rclcpp::shutdown();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("mobile_robot_controller"), "Error: " << e.what());
    }

    return 0;
}
