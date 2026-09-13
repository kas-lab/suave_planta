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

"""Analyze a batch directory produced by run_batch.launch.py."""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration


def _check_exit(event, context):
    if event.returncode != 0:
        raise RuntimeError(
            'Batch Wilcoxon analysis failed with exit code '
            f'{event.returncode}')
    return []


def _launch_analysis(context):
    batch_dir = Path(LaunchConfiguration('batch_dir').perform(
        context)).expanduser().resolve()
    if not (batch_dir / 'campaigns').is_dir():
        raise ValueError(
            f'{batch_dir}: expected the batch root containing campaigns/, '
            'as printed by run_batch.launch.py')
    command = [
        sys.executable, '-m',
        'suave_runner.analysis.wilcoxon_analysis_batch', str(batch_dir),
        '--correction', LaunchConfiguration('correction').perform(context),
    ]
    for argument, option in [('output_root', '--output'),
                             ('config_dir', '--config-dir')]:
        value = LaunchConfiguration(argument).perform(context)
        if value:
            command.extend([option, str(Path(value).expanduser())])
    process = ExecuteProcess(
        cmd=command, name='batch_wilcoxon_analysis', output='screen')
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=process, on_exit=_check_exit)),
        process,
    ]


def generate_launch_description():
    """Declare batch input and optional analysis overrides."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'batch_dir', description='Batch root from run_batch.launch.py '
            '(normally ~/suave/results/batches/batch_YYYYMMDD_HHMMSS)'),
        DeclareLaunchArgument(
            'output_root', default_value='', description='Output root; '
            'defaults to <batch_dir>/campaings_results'),
        DeclareLaunchArgument(
            'config_dir', default_value='', description='Runner config '
            'directory if config paths recorded in state.json have moved'),
        DeclareLaunchArgument(
            'correction', default_value='holm', choices=['holm', 'none'],
            description='Correction separately within each campaign'),
        OpaqueFunction(function=_launch_analysis),
    ])
