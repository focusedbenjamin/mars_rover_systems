#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <cmath>
#include <string>

class TopicsQuizNode : public rclcpp::Node
{
public:
    TopicsQuizNode() : Node("topics_quiz_node")
    {
        // Publishers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/mars_rover_status", 10);

        // Subscribers
        mission_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/nasa_mission", 10,
            std::bind(&TopicsQuizNode::mission_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&TopicsQuizNode::odom_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&TopicsQuizNode::scan_callback, this, std::placeholders::_1));

        // Timer loop (control loop)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TopicsQuizNode::control_loop, this));

        // Home position
        home_x_ = 0.0;
        home_y_ = 0.0;

        pickup_x_ = -2.342;
        pickup_y_ = -2.432;

        goal_tolerance_ = 0.1;

        target_set_ = false;
        obstacle_ahead_ = false;

        RCLCPP_INFO(this->get_logger(), "Topics Quiz Node Initialized");
    }

private:

    // ===== CALLBACKS =====

    void mission_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = msg->data;

        if (cmd == "Go-Home")
        {
            set_goal(home_x_, home_y_, "Go-Home");
        }
        else if (cmd == "Go-Pickup")
        {
            set_goal(pickup_x_, pickup_y_, "Go-Pickup");
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);

        // Check goal reached
        if (target_set_)
        {
            double dist = distance(current_x_, current_y_, goal_x_, goal_y_);

            if (dist < goal_tolerance_)
            {
                stop_robot();
                target_set_ = false;

                publish_status("goal-reached");
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
            }
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        obstacle_ahead_ = false;

        int start = msg->ranges.size() * 0.45;
        int end   = msg->ranges.size() * 0.55;

        for (int i = start; i < end; i++)
        {
            if (msg->ranges[i] < 0.8)
            {
                obstacle_ahead_ = true;
                return;
            }
        }
    }

    // ===== CONTROL LOOP =====

    void control_loop()
    {
        if (!target_set_)
        {
            stop_robot();
            return;
        }

        if (obstacle_ahead_)
        {
            avoid_obstacle();
            return;
        }

        move_to_goal();
    }

    // ===== MOVEMENT =====

    void move_to_goal()
    {
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;

        double angle_to_goal = atan2(dy, dx);
        double distance_to_goal = sqrt(dx*dx + dy*dy);

        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.5 * distance_to_goal;
        cmd.angular.z = 2.0 * (angle_to_goal - current_yaw_);

        cmd_pub_->publish(cmd);
    }

    void avoid_obstacle()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.8;
        cmd_pub_->publish(cmd);

        publish_status("avoiding-obstacle");
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    // ===== HELPERS =====

    void set_goal(double x, double y, const std::string & name)
    {
        if (distance(current_x_, current_y_, x, y) < goal_tolerance_)
        {
            RCLCPP_INFO(this->get_logger(), "%s already reached.", name.c_str());
            return;
        }

        goal_x_ = x;
        goal_y_ = y;
        target_set_ = true;

        RCLCPP_INFO(this->get_logger(), "New mission: %s", name.c_str());
        publish_status("mission-started");
    }

    double distance(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    void publish_status(const std::string & msg)
    {
        std_msgs::msg::String status;
        status.data = msg;
        status_pub_->publish(status);
    }

    // ===== VARIABLES =====

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double current_x_, current_y_, current_yaw_;

    double goal_x_, goal_y_;
    double home_x_, home_y_;
    double pickup_x_, pickup_y_;

    double goal_tolerance_;
    bool target_set_;
    bool obstacle_ahead_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicsQuizNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}