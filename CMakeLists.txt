cmake_minimum_required(VERSION 3.8)
project(robotino_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(
  include
  /opt/robotino/include
)

# Find the Robotino libraries
find_library(REC_ROBOTINO_API2_LIBRARY rec_robotino_api2 HINTS /opt/robotino/lib)
find_library(REC_ROBOTINO_RPC_LIBRARY rec_robotino_rpc HINTS /opt/robotino/lib)

if(NOT REC_ROBOTINO_API2_LIBRARY)
  message(FATAL_ERROR "rec_robotino_api2 library not found in /opt/robotino/lib")
endif()
if(NOT REC_ROBOTINO_RPC_LIBRARY)
  message(FATAL_ERROR "rec_robotino_rpc library not found in /opt/robotino/lib")
endif()

# Add executable
add_executable(robotino_publisher
  src/main.cpp
  src/robotino_publisher.cpp
)

# Link libraries
target_link_libraries(robotino_publisher
  ${REC_ROBOTINO_API2_LIBRARY}
  ${REC_ROBOTINO_RPC_LIBRARY}
)

# ROS 2 dependencies
ament_target_dependencies(robotino_publisher
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
)

# Install instructions
install(TARGETS robotino_publisher DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_package()
