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

    std::vector<
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
    > m_subs;

    void timer_callback();

    struct ChannelConfig {
        int channel;
        std::string channel_name;
        std::string pwm_topic_name;
        std::string scaled_topic_name;
        float min;
        float max;
        float center;
    };

    std::vector<ChannelConfig> m_channel_configs;

    float m_pwm_freq;

    void f_param_digest();

    void f_init_servo(const ChannelConfig *  channel_config);

    void f_filtered_cmd(const ChannelConfig * channel_config, float value);

    void f_pwm_callback(const std_msgs::msg::Float32::SharedPtr msg, ChannelConfig * channel_config);

    void f_scaled_callback(const std_msgs::msg::Float32::SharedPtr msg, ChannelConfig * channel_config);

    static float linear_interpolate(float x, float x0, float x1, float y0, float y1){
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    };

public:
    PWMDriver();


};