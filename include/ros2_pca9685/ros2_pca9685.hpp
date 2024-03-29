#pragma once

#include "rclcpp/rclcpp.hpp"
#include "PiPCA9685/PCA9685.h"
#include "memory"
#include "map"
#include "std_msgs/msg/Float3232.hpp"

class PWMDriver : public rclcpp::Node
{

private:

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<PiPCA9685::PCA9685> pca_;

    std::map<int,rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> subs_;

    void timer_callback();

public:
    PWMDriver();

    void init_servo(int channel, int min, int max);

};