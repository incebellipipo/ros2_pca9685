#include "ros2_pca9685/ros2_pca9685.hpp"

PWMDriver::PWMDriver() :
    Node("cybership_servos",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true))
{
    this->f_param_digest();

    m_pca = std::make_shared<PiPCA9685::PCA9685>();
    m_pca->set_pwm_freq(m_pwm_freq);

    for(auto & channel_config : m_channel_configs){
        auto sub = this->create_subscription<std_msgs::msg::Float32>(
            channel_config.topic_name, 10,
            [this, channel_config](const std_msgs::msg::Float32::SharedPtr msg) {
                m_pca->set_pwm_ms(channel_config.channel, msg->data);
            });
        m_subs[channel_config.channel] = sub;
    }
}

void PWMDriver::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Timer callback");
}


void PWMDriver::f_param_digest()
{
    std::cout << "reading the parameters" << std::endl;
    std::map<std::string, rclcpp::Parameter> parameter_map;

    this->get_parameters("channels", parameter_map);
    std::set<std::string> channel_names;
    for(auto & key_value : parameter_map){
        size_t pos = key_value.first.find('.');
        auto channel_name = key_value.first.substr(0, pos);
        channel_names.insert(channel_name);
    }

    for(auto & channel_name : channel_names){
        std::cout << "channel name: " << channel_name << std::endl;

        ChannelConfig channel_config;
        this->get_parameter("channels." + channel_name + ".channel", channel_config.channel);
        this->get_parameter("channels." + channel_name + ".channel_name", channel_config.channel_name);
        this->get_parameter("channels." + channel_name + ".topic_name", channel_config.topic_name);

        m_channel_configs.push_back(channel_config);
    }

    m_pwm_freq = this->declare_parameter("pwm_freq", 60.0);

}