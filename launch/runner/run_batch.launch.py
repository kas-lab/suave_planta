# Copyright 2026 KAS Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run the configured PLANTA batch with an optional explicit output path."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Configure a fresh or resumed batch without starting analysis."""
    share = Path(get_package_share_directory('suave_planta'))
    config = share / 'config' / 'runner' / 'batch_campaigns.yml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'batch_dir', default_value='', description='New batch directory; '
            'empty creates ~/suave/results/batches/batch_YYYYMMDD_HHMMSS'),
        DeclareLaunchArgument(
            'resume_state_file', default_value='', description='Existing '
            'batch state.json to resume; leave batch_dir empty when resuming'),
        Node(
            package='suave_runner', executable='run_batch',
            name='run_batch_node', output='screen',
            parameters=[str(config), {
                'batch_dir': ParameterValue(
                    LaunchConfiguration('batch_dir'), value_type=str),
                'resume_state_file': ParameterValue(
                    LaunchConfiguration('resume_state_file'), value_type=str),
            }]),
    ])
