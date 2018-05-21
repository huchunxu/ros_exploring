cmake_minimum_required(VERSION 3.5)
project(ros2_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(ros2_talker src/ros2_talker.cpp)
ament_target_dependencies(ros2_talker rclcpp std_msgs)

add_executable(ros2_listerner src/ros2_listerner.cpp)
ament_target_dependencies(ros2_listerner rclcpp std_msgs)

add_executable(ros2_server src/ros2_server.cpp)
ament_target_dependencies(ros2_server example_interfaces rclcpp)

add_executable(ros2_client src/ros2_client.cpp)
ament_target_dependencies(ros2_client example_interfaces rclcpp)

install(TARGETS
    ros2_talker
    ros2_listerner
    ros2_server
    ros2_client
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
