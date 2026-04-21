cmake_minimum_required(VERSION 3.8)
project(basics_ros2_multithreading)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# === EXECUTABLES ===

add_executable(function src/function.cpp)
ament_target_dependencies(function rclcpp)

add_executable(callback_function src/callback_function.cpp)
ament_target_dependencies(callback_function rclcpp)

add_executable(callback_spinonce_function src/callback_spinonce_function.cpp)
ament_target_dependencies(callback_spinonce_function rclcpp)

add_executable(plant_detector src/plant_detector.cpp)
ament_target_dependencies(plant_detector 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  std_srvs 
  cv_bridge 
  OpenCV
)

# === INSTALL ===
install(TARGETS
  function
  callback_function
  callback_spinonce_function
  plant_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()