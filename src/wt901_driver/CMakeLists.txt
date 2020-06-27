cmake_minimum_required(VERSION 3.5)
project(wt901_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAEK_CXX_FLAGS "-O2 -Wall -Wextra -Wpedantic")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_message REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(converter src/converter.cpp)
ament_target_dependencies(converter rclcpp serial_message sensor_msgs)

install(TARGETS converter DESTINATION lib/${PROJECT_NAME})

ament_package()
