#include "ros2_pca9685/ros2_pca9685.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PWMDriver>());
  rclcpp::shutdown();
  return 0;
}