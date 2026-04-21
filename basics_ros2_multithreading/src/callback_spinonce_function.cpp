#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <functional>

class RobotStatus : public rclcpp::Node {
public:
    RobotStatus() : Node("robot_status"), 
                    time_robot_on_(0.0),
                    timer_period_(0.1),
                    main_task_period_(1.0),
                    shutdown_flag_(false),
                    robot_status_({"Robot Booting Up...", "Robot Ready...", "Robot ShuttingDown..."}) {
        
        timer_counter_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period_), 
            std::bind(&RobotStatus::timer_counter, this));
        
        main_task_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(main_task_period_), 
            std::bind(&RobotStatus::main_task, this));
    }
    
    void robot_message(const std::string& text, const std::string& robot_name = "Robot-1") {
        RCLCPP_INFO(this->get_logger(), "%s: %s", robot_name.c_str(), text.c_str());
    }
    
    void timer_counter() {
        time_robot_on_ += timer_period_;
        RCLCPP_INFO(this->get_logger(), "Updated Time Robot On=%.1f", time_robot_on_);
    }
    
    void main_task() {
        if (shutdown_flag_) {
            return;  // Skip the main task if shutting down
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        if (!robot_status_.empty()) {
            std::string status = robot_status_.front();
            robot_status_.erase(robot_status_.begin());
            robot_message(status);
            
            if (status.find("ShuttingDown") != std::string::npos) {
                RCLCPP_INFO(this->get_logger(), "Shutting down node...");
                shutdown_flag_ = true;  // Set the shutdown flag
            } else {
                RCLCPP_INFO(this->get_logger(), "Continue....");
            }
        }
    }
    
    bool is_shutdown_requested() const {
        return shutdown_flag_;
    }
    
private:
    double time_robot_on_;
    double timer_period_;
    double main_task_period_;
    bool shutdown_flag_;
    std::vector<std::string> robot_status_;
    rclcpp::TimerBase::SharedPtr timer_counter_timer_;
    rclcpp::TimerBase::SharedPtr main_task_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto robot_status_node = std::make_shared<RobotStatus>();
    
    // Use spin_some inside a loop instead of spin
    while (rclcpp::ok()) {
        rclcpp::spin_some(robot_status_node);  // Handle callbacks
        if (robot_status_node->is_shutdown_requested()) {
            RCLCPP_INFO(robot_status_node->get_logger(), "Node has been shut down.");
            rclcpp::shutdown();  // Shutdown ROS2 client library
            break;  // Break out of the loop to end the script
        }
        
        // Small sleep to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    return 0;
}