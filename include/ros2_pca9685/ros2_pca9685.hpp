#pragma once

#include "memory"
#include "map"
#include "string"
#include "vector"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "PiPCA9685/PCA9685.h"

class PWMDriver : public rclcpp::Node
{

private:

    std::shared_ptr<PiPCA9685::PCA9685> m_pca;

    std::map<
        int,
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
    > m_subs;

    void timer_callback();

    struct ChannelConfig {
        int channel;
        std::string channel_name;
        std::string topic_name;
    };

    std::vector<ChannelConfig> m_channel_configs;

    float m_pwm_freq;

    void f_param_digest();

public:
    PWMDriver();

    void init_servo(int channel, int min, int max);

};