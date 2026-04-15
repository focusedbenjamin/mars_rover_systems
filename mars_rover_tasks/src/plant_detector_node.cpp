#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>  // Include String message type
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "mars_rover_tasks/plant_detector.hpp"  // Include the AI class

class PlantDetectorNode : public rclcpp::Node
{
public:
    PlantDetectorNode() : Node("plant_detector_node")
    {
        // Initialize the PlantDetector
        std::string path_to_model = "/home/user/ros2_ws/src/basic_ros2_extra_files/plant_detector/best_plant_detector_model.pth";
        plant_detector_ = std::make_unique<PlantDetector>(path_to_model);
        
        // Subscribe to the image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/leo/camera/image_raw",
            10,
            std::bind(&PlantDetectorNode::image_callback, this, std::placeholders::_1));

        // Initialize the Publisher for plant detection results
        publisher_ = this->create_publisher<std_msgs::msg::String>("/plant_detector", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS Image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        // Convert BGR to RGB
        cv::Mat rgb_image;
        cv::cvtColor(cv_ptr->image, rgb_image, cv::COLOR_BGR2RGB);
        
        // Use the PlantDetector to make a prediction
        float prediction = plant_detector_->predict(rgb_image);
        
        // Determine the result message based on the prediction
        std::string result;
        if (prediction > 0.5)
        {
            result = "Plant detected with confidence: " + std::to_string(prediction);
            RCLCPP_WARN(this->get_logger(), "%s", result.c_str());
        }
        else
        {
            result = "No plant detected. Confidence: " + std::to_string(1.0 - prediction);
            RCLCPP_INFO(this->get_logger(), "%s", result.c_str());
        }
        
        // Publish the result as a String message
        auto msg_out = std_msgs::msg::String();
        msg_out.data = result;
        publisher_->publish(msg_out);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::unique_ptr<PlantDetector> plant_detector_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto plant_detector_node = std::make_shared<PlantDetectorNode>();

    rclcpp::spin(plant_detector_node);

    // Shutdown explicitly
    rclcpp::shutdown();
    return 0;
}