#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char** argv){

rclcpp::init(argc, argv);
std::cout<<"Shutting down Mars rover 1..." << std::endl;

rclcpp::shutdown();
return 0;


}