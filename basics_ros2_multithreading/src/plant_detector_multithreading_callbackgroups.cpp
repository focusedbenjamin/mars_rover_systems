#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/callback_group.hpp"
#include <chrono>
#include <thread>
#include <memory>
#include <fstream>
#include <iomanip>
#include <filesystem>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class PlantDetectorNode : public rclcpp::Node
{
public:
  PlantDetectorNode(const std::string & image_topic_name = "/leo/camera/image_raw")
  : Node("plant_detector_node"), image_topic_name_(image_topic_name)
  {
    // Define the output directory
    output_dir_ = "/home/simulations/ros2_sims_ws/src/cpp_callback_visual_scripts/";

    // Open timing log files with full path
    listener_callback_log_.open(output_dir_ + "mutuallyexclusive_listener_callback_timings.csv");
    detect_plants_callback_log_.open(output_dir_ + "mutuallyexclusive_detect_plants_callback_timings.csv");

    // Write CSV headers
    listener_callback_log_ << "timestamp,duration_ms,callback_type\n";
    detect_plants_callback_log_ << "timestamp,duration_ms,callback_type\n";

    // Create two mutually exclusive callback groups
    mutuallyexclusive_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    mutuallyexclusive_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    // Set up subscription options to use the first mutually exclusive callback group
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = mutuallyexclusive_group_1_;
    
    // Create subscription for camera images with first mutually exclusive callback group
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_name_, 10, 
      std::bind(&PlantDetectorNode::listener_callback_wrapper, this, _1),
      sub_options);
    
    // Create publisher for velocity commands
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Create service for plant detection with second mutually exclusive callback group
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "detect_plants", 
      std::bind(&PlantDetectorNode::detect_plants_callback_wrapper, this, _1, _2),
      rmw_qos_profile_services_default,
      mutuallyexclusive_group_2_);
    
    RCLCPP_INFO(this->get_logger(), "Plant Detector Node initialized with MutuallyExclusiveCallbackGroups");
    RCLCPP_INFO(this->get_logger(), "Callback timing instrumentation enabled");
    RCLCPP_INFO(this->get_logger(), "CSV logs will be saved to: %s", output_dir_.c_str());
    RCLCPP_INFO(this->get_logger(), "Files: mutuallyexclusive_listener_callback_timings.csv, mutuallyexclusive_detect_plants_callback_timings.csv");
  }

  ~PlantDetectorNode() {
    if (listener_callback_log_.is_open())
      listener_callback_log_.close();
    if (detect_plants_callback_log_.is_open())
      detect_plants_callback_log_.close();

    // Generate summary
    generate_timing_summary();
  }

private:
  // Wrapper functions that add timing instrumentation
  void listener_callback_wrapper(const sensor_msgs::msg::Image::SharedPtr msg) {
    auto start_time = std::chrono::high_resolution_clock::now();
    uint64_t timestamp = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            start_time.time_since_epoch())
            .count());

    // Call the actual callback
    listener_callback(msg);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time)
                        .count() /
                    1000.0; // Convert to milliseconds

    // Log timing data
    listener_callback_log_ << timestamp << "," << duration << ",mutuallyexclusive_listener_callback\n";
    listener_callback_log_.flush();

    // Store for analysis
    listener_callback_timings_.push_back({timestamp, duration});
  }

  void detect_plants_callback_wrapper(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    auto start_time = std::chrono::high_resolution_clock::now();
    uint64_t timestamp = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            start_time.time_since_epoch())
            .count());

    // Call the actual callback
    detect_plants_callback(request, response);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time)
                        .count() /
                    1000.0; // Convert to milliseconds

    // Log timing data
    detect_plants_callback_log_ << timestamp << "," << duration << ",mutuallyexclusive_detect_plants_callback\n";
    detect_plants_callback_log_.flush();

    // Store for analysis
    detect_plants_callback_timings_.push_back({timestamp, duration});
  }

  void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Receiving video frame");
    
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

  void generate_timing_summary() {
    RCLCPP_INFO(this->get_logger(), "=== MUTUALLY EXCLUSIVE CALLBACK TIMING SUMMARY ===");

    if (!listener_callback_timings_.empty()) {
      double avg_listener = 0,
             min_listener = listener_callback_timings_[0].duration,
             max_listener = 0;
      for (const auto &timing : listener_callback_timings_) {
        avg_listener += timing.duration;
        min_listener = std::min(min_listener, timing.duration);
        max_listener = std::max(max_listener, timing.duration);
      }
      avg_listener /= listener_callback_timings_.size();

      RCLCPP_INFO(
          this->get_logger(),
          "mutuallyexclusive_listener_callback: %zu calls, avg=%.2fms, min=%.2fms, max=%.2fms",
          listener_callback_timings_.size(), avg_listener, min_listener,
          max_listener);
    }

    if (!detect_plants_callback_timings_.empty()) {
      double avg_detect = 0,
             min_detect = detect_plants_callback_timings_[0].duration,
             max_detect = 0;
      for (const auto &timing : detect_plants_callback_timings_) {
        avg_detect += timing.duration;
        min_detect = std::min(min_detect, timing.duration);
        max_detect = std::max(max_detect, timing.duration);
      }
      avg_detect /= detect_plants_callback_timings_.size();

      RCLCPP_INFO(this->get_logger(),
                  "mutuallyexclusive_detect_plants_callback: %zu calls, avg=%.2fms, min=%.2fms, "
                  "max=%.2fms",
                  detect_plants_callback_timings_.size(), avg_detect,
                  min_detect, max_detect);
    }

    RCLCPP_INFO(this->get_logger(), "CSV files saved to: %s",
                output_dir_.c_str());
    RCLCPP_INFO(this->get_logger(), "Files: mutuallyexclusive_listener_callback_timings.csv, "
                                    "mutuallyexclusive_detect_plants_callback_timings.csv");
  }

  std::string image_topic_name_;
  std::string output_dir_;
  rclcpp::CallbackGroup::SharedPtr mutuallyexclusive_group_1_;
  rclcpp::CallbackGroup::SharedPtr mutuallyexclusive_group_2_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  cv::Mat last_image_;
  bool image_received_ = false;

  // Timing instrumentation variables
  std::ofstream listener_callback_log_;
  std::ofstream detect_plants_callback_log_;

  struct TimingData {
    uint64_t timestamp;
    double duration;
  };

  std::vector<TimingData> listener_callback_timings_;
  std::vector<TimingData> detect_plants_callback_timings_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto plant_detector_node = std::make_shared<PlantDetectorNode>();
  
  // Use MultiThreadedExecutor with 2 threads
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(plant_detector_node);
  
  try {
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(plant_detector_node->get_logger(), "Exception: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}