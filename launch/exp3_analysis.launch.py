import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the config file
    config_path = os.path.join(
        get_package_share_directory('suave_planta'),
        'config',
        'exp3_analysis_config.yml'
    )

    # Launch the suave_runner node with the parameters loaded from YAML
    return LaunchDescription([
        Node(
            package='suave_runner',
            executable='statistical_analysis',
            name='analysis',
            output='screen',
            parameters=[config_path],
        )
    ])
