#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

class ObstacleDetectorNode : public rclcpp::Node
{
public:
    ObstacleDetectorNode(const std::string& node_name = "obstacle_detector_node")
        : Node(node_name), node_name_(node_name)
    {
        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_scan",
            qos,
            std::bind(&ObstacleDetectorNode::laserscan_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "%s Ready...", node_name_.c_str());
    }

private:
    std::string node_name_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;

    // ================================
    // SAFE MIN FUNCTION (handles inf/NaN)
    // ================================
    float getMinInRange(const sensor_msgs::msg::LaserScan::SharedPtr msg, int start, int end)
    {
        float min_val = std::numeric_limits<float>::infinity();

        for (int i = start; i < end && i < (int)msg->ranges.size(); i++)
        {
            float value = msg->ranges[i];

            if (std::isnan(value) || std::isinf(value))
                continue;

            min_val = std::min(min_val, value);
        }

        return min_val;
    }

    // ================================
    // MAIN CALLBACK
    // ================================
    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const float THRESHOLD = 0.8;
        const int total = msg->ranges.size();
        const int sector = total / 6;

        // ================================
        // SECTOR DEFINITIONS (RIGHT → LEFT)
        // ================================
        int rr_s = 0,        rr_e = sector;
        int r_s  = rr_e,     r_e  = r_s + sector;
        int fr_s = r_e,      fr_e = fr_s + sector;
        int fl_s = fr_e,     fl_e = fl_s + sector;
        int l_s  = fl_e,     l_e  = l_s + sector;
        int lr_s = l_e,      lr_e = total;

        // ================================
        // MIN DISTANCES PER SECTOR
        // ================================
        float right_rear  = getMinInRange(msg, rr_s, rr_e);
        float right       = getMinInRange(msg, r_s, r_e);
        float front_right = getMinInRange(msg, fr_s, fr_e);
        float front_left  = getMinInRange(msg, fl_s, fl_e);
        float left        = getMinInRange(msg, l_s, l_e);
        float left_rear   = getMinInRange(msg, lr_s, lr_e);

        // ================================
        // LOG OUTPUT (REQUIRED FORMAT)
        // ================================
        RCLCPP_INFO(this->get_logger(), "Right_Rear: %.2f meters", right_rear);
        RCLCPP_INFO(this->get_logger(), "Right: %.2f meters", right);
        RCLCPP_INFO(this->get_logger(), "Front_Right: %.2f meters", front_right);
        RCLCPP_INFO(this->get_logger(), "Front_Left: %.2f meters", front_left);
        RCLCPP_INFO(this->get_logger(), "Left: %.2f meters", left);
        RCLCPP_INFO(this->get_logger(), "Left_Rear: %.2f meters", left_rear);

        // ================================
        // DETECTION FLAGS
        // ================================
        bool obs_fl = front_left  < THRESHOLD;
        bool obs_fr = front_right < THRESHOLD;
        bool obs_l  = left        < THRESHOLD;
        bool obs_r  = right       < THRESHOLD;

        // ================================
        // ACTION DECISION SYSTEM
        // ================================
        std::string action;

        if (obs_fl && obs_fr)
        {
            action = "Selected Turn Arbitrary Direction Right";
        }
        else if (obs_fl)
        {
            action = "Turn Right";
        }
        else if (obs_fr)
        {
            action = "Turn Left";
        }
        else if (obs_l)
        {
            action = "Go Forwards turning slightly right";
        }
        else if (obs_r)
        {
            action = "Go Forwards turning slightly left";
        }
        else
        {
            action = "Go Forwards";
        }

        RCLCPP_INFO(this->get_logger(), "Suggested action: %s", action.c_str());
    }
};

// ================================
// MAIN FUNCTION
// ================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}