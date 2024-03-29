#include "ros2_pca9685/ros2_pca9685.hpp"

PWMDriver::PWMDriver() : Node("cybership_servos")
{
    pca_ = std::make_shared<PiPCA9685::PCA9685>();
    pca_->set_pwm_freq(60.0);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&PWMDriver::timer_callback, this));

    for(int i = 0; i < 12 ; i++) {
        subs_[i] = this->create_subscription<std_msgs::msg::Int32>(
            "servo_" + std::to_string(i), 10,
            [this, i](const std_msgs::msg::Int32::SharedPtr msg) {
                pca_->set_pwm(0, i, msg->data);
            });
    }
}

void PWMDriver::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Timer callback");
}


