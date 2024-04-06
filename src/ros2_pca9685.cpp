#include "ros2_pca9685/ros2_pca9685.hpp"

PWMDriver::PWMDriver() :
    Node("cybership_servos",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true))
{

    // Read the parameters
    this->f_param_digest();

    // Initialize the PCA9685
    m_pca = std::make_shared<PCA9685::PCA9685>();
    m_pca->set_pwm_freq(m_pwm_freq);

    // Initialize the servos
    for(auto & channel_config : m_channel_configs){
        this->f_init_servo(&channel_config);
    }

    for(auto & channel_config : m_channel_configs){
        auto pwm_callback = std::bind(
            &PWMDriver::f_pwm_callback,
            this,
            std::placeholders::_1,
            &channel_config);
        auto pwm_sub = this->create_subscription<std_msgs::msg::Float32>(
            channel_config.pwm_topic_name, 10, [pwm_callback, channel_config](const std_msgs::msg::Float32::SharedPtr msg){
                pwm_callback(msg, &channel_config);
            }
        );
        m_subs.push_back(pwm_sub);

        auto scaled_callback = std::bind(
            &PWMDriver::f_scaled_callback,
            this,
            std::placeholders::_1,
            &channel_config);
        auto scaled_sub = this->create_subscription<std_msgs::msg::Float32>(
            channel_config.scaled_topic_name, 10, [scaled_callback, channel_config](const std_msgs::msg::Float32::SharedPtr msg){
                scaled_callback(msg, &channel_config);
            }
        );
        m_subs.push_back(scaled_sub);

    }
}

void PWMDriver::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Timer callback");
}

void PWMDriver::f_pwm_callback(const std_msgs::msg::Float32::SharedPtr msg, ChannelConfig * channel_config)
{
    float value = msg->data;
    this->f_filtered_cmd(channel_config, value);
}

void PWMDriver::f_scaled_callback(const std_msgs::msg::Float32::SharedPtr msg, ChannelConfig * channel_config)
{
    float value = msg->data;

    auto linear_interpolate = [](float x, float x0, float x1, float y0, float y1){
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    };

    if(value > 0.0 ) {
        value = linear_interpolate(value, channel_config->scale_center, channel_config->scale_max, channel_config->pulse_center, channel_config->pulse_max);
    } else {
        value = linear_interpolate(value, channel_config->scale_center, channel_config->scale_min, channel_config->pulse_center, channel_config->pulse_min);
    }

    this->f_filtered_cmd(channel_config, value);
}

void PWMDriver::f_filtered_cmd(const ChannelConfig * channel_config, float value)
{
    if(channel_config->channel < 0){
        // Print an error message and return
        RCLCPP_ERROR(this->get_logger(), "Channel not set for %s", channel_config->channel_name.c_str());
        return;
    }
    if(value < channel_config->pulse_min){
        value = channel_config->pulse_min;
    } else if(value > channel_config->pulse_max){
        value = channel_config->pulse_max;
    }

    m_pca->set_pwm_ms(channel_config->channel, value);
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

        ChannelConfig channel_config;
        // TODO: replace it with declare parameter
        this->get_parameter_or<int>("channels." + channel_name + ".channel", channel_config.channel, -1);
        this->get_parameter_or<std::string>("channels." + channel_name + ".channel_name", channel_config.channel_name);
        this->get_parameter_or<std::string>("channels." + channel_name + ".pwm_topic", channel_config.pwm_topic_name, std::string("/" + channel_name + "/pwm"));
        this->get_parameter_or<std::string>("channels." + channel_name + ".scaled_topic", channel_config.scaled_topic_name, std::string("/" + channel_name + "/scaled"));
        this->get_parameter_or<float>("channels." + channel_name + ".pulse_duration.min", channel_config.pulse_min, 1.0);
        this->get_parameter_or<float>("channels." + channel_name + ".pulse_duration.max", channel_config.pulse_max, 2.0);
        this->get_parameter_or<float>("channels." + channel_name + ".scale.min", channel_config.scale_min, -1.0);
        this->get_parameter_or<float>("channels." + channel_name + ".scale.max", channel_config.scale_max, 1.0);
        this->get_parameter_or<float>("channels." + channel_name + ".pulse_duration.center", channel_config.pulse_center, (channel_config.pulse_min + channel_config.pulse_max) / 2.0);
        this->get_parameter_or<float>("channels." + channel_name + ".scale.center", channel_config.scale_center, (channel_config.scale_min + channel_config.scale_max) / 2.0);


        m_channel_configs.push_back(channel_config);
    }

    this->get_parameter_or<float>("frequency", m_pwm_freq, 50.0);

}

void PWMDriver::f_init_servo(const ChannelConfig * channel_config)
{
    m_pca->set_pwm_ms(channel_config->channel, channel_config->pulse_center);
}