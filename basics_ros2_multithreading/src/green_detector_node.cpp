#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/callback_group.hpp>
#include <chrono>
#include <thread>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class PlantDetectorNode : public rclcpp::Node
{
public:
  PlantDetectorNode(const std::string & image_topic_name = "/leo/camera/image_raw")
  : Node("plant_detector_node"), image_topic_name_(image_topic_name)
  {
    // Create two mutually exclusive callback groups
    mutuallyexclusive_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    mutuallyexclusive_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    // Set up subscription options to use the first mutually exclusive callback group
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = mutuallyexclusive_group_1_;
    
    // Create subscription for camera images with first mutually exclusive callback group
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_name_, 10, 
      std::bind(&PlantDetectorNode::listener_callback, this, _1),
      sub_options);
    
    // Create publisher for velocity commands
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Create service for plant detection with second mutually exclusive callback group
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "detect_plants", 
      std::bind(&PlantDetectorNode::detect_plants_callback, this, _1, _2),
      rmw_qos_profile_services_default,
      mutuallyexclusive_group_2_);
    
    RCLCPP_INFO(this->get_logger(), "1- PlantDetectorNode READY...");
  }

  cv::Mat get_latest_image() const
  {
    return last_image_.clone();
  }

private:
  void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Commented out to reduce log output and see service calls clearly
    // RCLCPP_INFO(this->get_logger(), "Receiving video frame");
    
    try {
      // Convert ROS image to OpenCV image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      last_image_ = cv_ptr->image.clone();
      image_received_ = true;
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void detect_plants_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // Unused parameter
    
    auto start_time = this->get_clock()->now();
    auto duration_10_sec = rclcpp::Duration::from_seconds(10.0);
    
    bool plant_found = false;

    // Start moving the robot
    publish_velocity(0.0, -0.2);
    
    auto delta_time = this->get_clock()->now() - start_time;

    while (delta_time < duration_10_sec && !plant_found) {
      delta_time = this->get_clock()->now() - start_time;

      if (image_received_ && !last_image_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Processing Image for Plants...");
        
        // Simple plant detection based on green color analysis
        cv::Mat hsv_image;
        cv::cvtColor(last_image_, hsv_image, cv::COLOR_BGR2HSV);
        
        // Define range for green color
        cv::Scalar lower_green(35, 40, 40);
        cv::Scalar upper_green(85, 255, 255);
        
        cv::Mat green_mask;
        cv::inRange(hsv_image, lower_green, upper_green, green_mask);
        
        // Calculate percentage of green pixels
        int green_pixels = cv::countNonZero(green_mask);
        int total_pixels = green_mask.rows * green_mask.cols;
        double green_percentage = (double)green_pixels / total_pixels;

        if (green_percentage > 0.15) {  // Threshold for plant detection
          publish_velocity(0.0, 0.0);
          RCLCPP_INFO(this->get_logger(), "Plant Detected! Green percentage: %.2f", green_percentage);
          plant_found = true;
        } else {
          RCLCPP_INFO(this->get_logger(), "No Plant Detected. Green percentage: %.2f", green_percentage);
        }
      }

      std::this_thread::sleep_for(100ms);
    }

    // Stop the robot
    publish_velocity(0.0, 0.0);
    response->success = plant_found;
    response->message = "Plant detection completed";
  }
  
  void publish_velocity(double linear, double angular)
  {
    auto vel_msg = geometry_msgs::msg::Twist();
    vel_msg.linear.x = linear;
    vel_msg.angular.z = angular;
    cmd_vel_publisher_->publish(vel_msg);
  }

  std::string image_topic_name_;
  rclcpp::CallbackGroup::SharedPtr mutuallyexclusive_group_1_;
  rclcpp::CallbackGroup::SharedPtr mutuallyexclusive_group_2_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  cv::Mat last_image_;
  bool image_received_ = false;
};

class GreenDetectorNode : public rclcpp::Node
{
public:
  GreenDetectorNode(std::shared_ptr<PlantDetectorNode> plant_detect_node)
  : Node("green_detector_node"), plant_detect_node_(plant_detect_node)
  {
    // Create service for green detection
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "/green_detector", 
      std::bind(&GreenDetectorNode::detect_green_detector_callback, this, _1, _2));
    
    RCLCPP_INFO(this->get_logger(), "2- GreenDetectorNode READY...");
  }

private:
  void detect_green_detector_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // Unused parameter
    
    cv::Mat cv_image = get_image();
    auto result = detect_green_detector(cv_image);
    response->success = result.first;
    response->message = result.second;
  }
  
  cv::Mat get_image()
  {
    return plant_detect_node_->get_latest_image();
  }

  std::pair<bool, std::string> detect_green_detector(const cv::Mat& cv_image)
  {
    if (cv_image.empty()) {
      return {false, "No image available"};
    }

    // Convert the image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(cv_image, hsv_image, cv::COLOR_BGR2HSV);

    // Define the lower and upper bounds for green color in HSV
    cv::Scalar lower_green(34, 84, 0);   // Adjusted lower bound
    cv::Scalar upper_green(179, 255, 255); // Adjusted upper bound

    // Create a mask to threshold the image to detect green
    cv::Mat mask;
    cv::inRange(hsv_image, lower_green, upper_green, mask);

    // Visualize the green part in the original image
    cv::Mat green_detected;
    cv::bitwise_and(cv_image, cv_image, green_detected, mask);

    // Show window until 'q' is pressed
    cv::imshow("Mask", mask);
    cv::imshow("Detected Green", green_detected);
    
    // Wait for key press (blocking)
    while (true) {
      if (cv::waitKey(1) & 0xFF == 'q') {
        break;
      }
    }

    cv::destroyAllWindows();

    return {true, "Green"};
  }

  std::shared_ptr<PlantDetectorNode> plant_detect_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto plant_detector_node = std::make_shared<PlantDetectorNode>();
  auto green_detector_node = std::make_shared<GreenDetectorNode>(plant_detector_node);
  
  // Use MultiThreadedExecutor with 2 threads
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(plant_detector_node);
  executor.add_node(green_detector_node);
  
  try {
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}