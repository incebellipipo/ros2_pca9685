cmake_minimum_required(VERSION 3.8)
project(ros2_pca9685)


include(FetchContent)
if (NOT TARGET PiPCA9685)
  set (PYTHON OFF CACHE INTERNAL "Turn off python for PiPCA9685")
  message(STATUS "${PROJECT_NAME}: `PiPCA9685` targets not found. Attempting to fetch contents...")
  FetchContent_Declare(
    PiPCA9685
    GIT_REPOSITORY https://github.com/barulicm/PiPCA9685.git
    GIT_TAG        master
  )
  FetchContent_MakeAvailable(PiPCA9685)
else()
  message(STATUS "PiPCA9685: `PiPCA9685` targets found.")
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

message(STATUS "PiPCA9685_SOURCE_DIR: ${PiPCA9685_SOURCE_DIR}")

include_directories(
  include
  ${PiPCA9685_SOURCE_DIR}/include
)


add_library(ros2_pca9685
  src/ros2_pca9685.cpp
)
target_link_libraries(ros2_pca9685
  PiPCA9685
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
