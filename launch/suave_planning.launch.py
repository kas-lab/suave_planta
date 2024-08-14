# Copyright 2024 Gustavo Rezende Silva
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
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
# from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # stdout_linebuf_envvar = SetEnvironmentVariable(
    #     'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    mission_type = LaunchConfiguration('mission_type')
    result_filename = LaunchConfiguration('result_filename')

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Desired mission type' +
                    '[time_constrained_mission or const_dist_mission]'
    )

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='',
        description='Name of the results file'
    )

    pkg_suave_planning = get_package_share_directory(
        'suave_planning')
    suave_planning_base_launch_path = os.path.join(
        pkg_suave_planning,
        'launch',
        'suave_base.launch.py')

    mission_config = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml'
    )

    suave_planning_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_planning_base_launch_path),
        launch_arguments={
            'mission_type': mission_type,
            'result_filename': result_filename,
            'mission_config': mission_config,
        }.items()
    )

    suave_planning_path = get_package_share_directory('suave_planning')
    plansys_path = get_package_share_directory('plansys2_bringup')
    plansys2_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                plansys_path,
                'launch',
                'plansys2_bringup_launch_distributed.py')),
            launch_arguments={
                'model_file':
                    suave_planning_path + '/pddl/suave_domain_created.pddl',
                'problem_file':
                    suave_planning_path + '/pddl/suave_problem_created.pddl',
                'params_file':
                    suave_planning_path + '/config/plansys2_params.yaml',
            }.items()
        )

    suave_planning_controller_node = Node(
        package='suave_planning',
        executable='suave_plansys_controller',
        # parameters=[mission_config]
    )

    start_robot_pddl_action_node = Node(
        package='suave_planning',
        executable='action_start_robot',
        # parameters=[mission_config]
        parameters=[{'action_name': 'start_robot'}]
    )

    search_pipeline_pddl_action_node = Node(
        package='suave_planning',
        executable='action_search_pipeline',
        # parameters=[mission_config]
        parameters=[{'action_name': 'search_pipeline'}]
    )

    inspect_pipeline_pddl_action_node = Node(
        package='suave_planning',
        executable='action_inspect_pipeline',
        # parameters=[mission_config]
        parameters=[{'action_name': 'inspect_pipeline'}]
    )

    return LaunchDescription([
        mission_type_arg,
        result_filename_arg,
        suave_planning_base,
        plansys2_bringup,
        suave_planning_controller_node,
        start_robot_pddl_action_node,
        search_pipeline_pddl_action_node,
        inspect_pipeline_pddl_action_node,
    ])
