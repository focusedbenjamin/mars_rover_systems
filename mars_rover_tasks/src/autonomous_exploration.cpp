#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class AutonomousExploration : public rclcpp::Node
{
public:
    AutonomousExploration() : Node("autonomous_exploration")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&AutonomousExploration::scan_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&AutonomousExploration::odom_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:

    // ================= ODOMETRY =================
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        distance_ = std::sqrt(current_x_ * current_x_ + current_y_ * current_y_);

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw_);
    }

    // ================= LASER =================
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int size = msg->ranges.size();

        front_ = *std::min_element(msg->ranges.begin() + size/3,
                                   msg->ranges.begin() + 2*size/3);

        left_ = *std::min_element(msg->ranges.begin() + 2*size/3,
                                  msg->ranges.end());

        right_ = *std::min_element(msg->ranges.begin(),
                                   msg->ranges.begin() + size/3);

        decision_engine();
    }

    // ================= BRAIN =================
    void decision_engine()
    {
        geometry_msgs::msg::Twist cmd;

        //  boundary control
        if (distance_ > 5.0)
        {
            cmd = return_to_origin();
        }
        // obstacle avoidance
        else if (front_ < 1.0 || left_ < 1.0 || right_ < 1.0)
        {
            cmd = avoid_obstacles();
        }
        //  explore
        else
        {
            cmd.linear.x = 0.5;
        }

        cmd_pub_->publish(cmd);
    }

    // ================= OBSTACLE AVOIDANCE =================
    geometry_msgs::msg::Twist avoid_obstacles()
    {
        geometry_msgs::msg::Twist cmd;

        if (front_ < 1.0)
            cmd.angular.z = 0.5;
        else if (left_ < 1.0)
            cmd.angular.z = -0.5;
        else if (right_ < 1.0)
            cmd.angular.z = 0.5;

        return cmd;
    }

    // ================= RETURN TO ORIGIN =================
    geometry_msgs::msg::Twist return_to_origin()
    {
        geometry_msgs::msg::Twist cmd;

        double desired_yaw = std::atan2(-current_y_, -current_x_);
        double yaw_error = desired_yaw - yaw_;

        yaw_error = std::fmod(yaw_error + M_PI, 2 * M_PI) - M_PI;

        if (std::abs(yaw_error) > 0.1)
        {
            cmd.angular.z = (yaw_error > 0) ? 0.5 : -0.5;
        }
        else
        {
            cmd.linear.x = 0.5;
        }

        return cmd;
    }

    // ================= VARIABLES =================
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    double front_{10}, left_{10}, right_{10};

    double current_x_{0}, current_y_{0};
    double yaw_{0};
    double distance_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousExploration>());
    rclcpp::shutdown();
    return 0;
}