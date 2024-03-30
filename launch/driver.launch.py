import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_pca9685 = os.path.join(
        get_package_share_directory('ros2_pca9685'),
        'param',
        'config.yaml'
    )

    node_pca9685 = launch_ros.actions.Node(
        package='ros2_pca9685',
        executable='ros2_pca9685_node',
        name='ros2_pca9685_node',
        parameters=[config_pca9685],
        output='screen'
    )

    ld = launch.LaunchDescription()

    ld.add_action(node_pca9685)

    return ld