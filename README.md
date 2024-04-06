# PCA9685 ROS2 Driver


## Configuration

- `frequency`:
    - Optional: True
    - Default: 50Hz
    - Type: Float
    - Unit: Hertz
    - Description: Defines the frequency of PWM signal
- `channels`:
    - Optional: False
    - Default: []
    - Type: Complex
    - Description: List of PWM signal descriptions

- `channel.{channel_name}`
    - Type: Complex
    - Description: Holds the information about the channels

- `channel.{channel_name}.pwm_topic`
    - Type: String, ROS2 Topic name
    - Description: Subscriber for pwm signal with interface type `std_msgs/msg/Float32`. Accepts in PWM signals as pulse duration in milliseconds. Limited by `channel.{channel_name}.pulse_duration.{min,max}`
- `channel.{channel_name}.pulse_duration.min`
    - Type: Float
    - Unit: Milliseconds
    - Description: Minimum pulse duration for operation
- `channel.{channel_name}.pulse_duration.max`
    - Type: Float
    - Unit: Milliseconds
    - Description: max pulse duration for operation

- `channel.{channel_name}.pulse_duration.center`
    - Type: Float
    - Optional: True
    - Unit: Milliseconds
    - Default: `(channel.{channel_name}.pwm_topic.max + channel.{channel_name}.pwm_topic.min) / 2`
    - Description: Center pulse duration.
- `channel.{channel_name}.scaled_topic`
    - Type: String, ROS2 Topic name
    - Description: Subscriber for scaled signal with interface type `std_msgs/msg/Float32`. Accepts in scaled control inputs limited by `channel.{channel_name}.scale.{min,max}`
- `channel.{channel_name}.scale.min`
    - Type: Float
    - Unit: Unitless
    - Description: Minimum scale
- `channel.{channel_name}.scale.max`
    - Type: Float
    - Unit: Unitless
    - Description: Max scale
- `channel.{channel_name}.scale.center`
    - Type: Float
    - Optional: True
    - Unit: Unitless
    - Default: `(channel.{channel_name}.scale.max + channel.{channel_name}.scale.min) / 2`
    - Description: Center scaled signal.





```yaml
/**:
  ros__parameters:
    frequency: 50.0
    channels:
      test_servo:
        # The model of the servo is Futaba: FP-S148
        # Read the specs for pulse-width and write it accordingly
        pwm_topic: /test_servo/pwm
        scaled_topic: /test_servo/scaled
        channel: 14
        pulse_duration:
          center: 1.44
          min: 0.550
          max: 2.330
        scale:
          center: 0.0
          min: -1.0
          max: 1.0
```