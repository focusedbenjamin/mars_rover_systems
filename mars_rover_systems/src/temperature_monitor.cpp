#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <random>

class TemperatureMonitor : public rclcpp::Node
{
public:
    TemperatureMonitor() : Node("temperature_monitor_node")
    {
        // Random generator setup
        std::random_device rd;
        gen_ = std::mt19937(rd());
        temp_dist_ = std::uniform_real_distribution<double>(20.0, 100.0);

        // Timer (1 second)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TemperatureMonitor::monitor_temperature, this)
        );

        RCLCPP_INFO(this->get_logger(), "Temperature Monitor Node Started");
    }

private:
    void monitor_temperature()
    {
        double temperature = temp_dist_(gen_);

        if (temperature > 70.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "⚠️ HIGH TEMPERATURE: %.2f °C", temperature);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "Temperature: %.2f °C", temperature);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> temp_dist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperatureMonitor>());
    rclcpp::shutdown();
    return 0;
}