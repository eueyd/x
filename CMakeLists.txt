cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(image_subscriber src/image_subscriber.cpp)
ament_target_dependencies(image_subscriber rclcpp sensor_msgs cv_bridge image_transport)

add_executable(enemy_position_subscriber src/enemy_position_subscriber.cpp)
ament_target_dependencies(enemy_position_subscriber 
  rclcpp geometry_msgs tf2_ros tf2_geometry_msgs visualization_msgs)

install(TARGETS
  image_subscriber
  enemy_position_subscriber
  DESTINATION lib/${cpp_pubsub})

ament_package()
