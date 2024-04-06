cmake_minimum_required(VERSION 3.8)
project(ros2_pca9685)


include(FetchContent)
if (NOT TARGET PiPCA9685)
  set (PYTHON OFF CACHE INTERNAL "Turn off python for PCA9685")
  message(STATUS "${PROJECT_NAME}: `PCA9685` targets not found. Attempting to fetch contents...")
  FetchContent_Declare(
    PCA9685
    GIT_REPOSITORY https://github.com/incebellipipo/PCA9685.git
    GIT_TAG        master
  )
  FetchContent_MakeAvailable(PCA9685)
else()
  message(STATUS "PCA9685: `PCA9685` targets found.")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_control REQUIRED)

message(STATUS "PCA9685_SOURCE_DIR: ${PCA9685_SOURCE_DIR}")

include_directories(
  include
)


add_library(ros2_pca9685
  src/ros2_pca9685.cpp
)
target_link_libraries(ros2_pca9685
  PCA9685
)
ament_target_dependencies(ros2_pca9685
  rclcpp
  tf2
  sensor_msgs
  std_msgs
  geometry_msgs
  ros2_control
)

add_executable(ros2_pca9685_node
  src/ros2_pca9685_node.cpp
)
ament_target_dependencies(ros2_pca9685_node
  rclcpp
  tf2
  sensor_msgs
  std_msgs
  geometry_msgs
  ros2_control
)
target_link_libraries(ros2_pca9685_node
  ros2_pca9685
)

install(
  DIRECTORY
    launch
    param
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  TARGETS
    ros2_pca9685
    ros2_pca9685_node
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
