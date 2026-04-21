#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

void robot_message(const std::string& text, const std::string& robot_name = "Robot-1") {
    std::cout << robot_name << ": " << text << std::endl;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    double period = 1.0;
    robot_message("Robot Booting Up...");
    std::this_thread::sleep_for(std::chrono::duration<double>(period));
    
    robot_message("Robot Ready...");
    std::this_thread::sleep_for(std::chrono::duration<double>(period));
    
    robot_message("Robot ShuttingDown...");
    
    rclcpp::shutdown();
    return 0;
}