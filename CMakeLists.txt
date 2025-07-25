cmake_minimum_required(VERSION 3.5)
project(euroc_ros2_bag_converter)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wextra -Wpedantic)

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


add_executable(euroc_ros2_bag_converter_node src/euroc_ros2_bag_converter.cpp src/euroc_ros2_bag_converter_node.cpp)
ament_target_dependencies(euroc_ros2_bag_converter_node rclcpp rosbag2_cpp sensor_msgs geometry_msgs tf2_msgs cv_bridge OpenCV nav_msgs)
install(TARGETS euroc_ros2_bag_converter_node DESTINATION lib/${PROJECT_NAME})


# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS euroc_ros2_bag_converter_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()


