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
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    silent = LaunchConfiguration('silent')
    silent_arg = DeclareLaunchArgument(
        'silent',
        default_value='false',
        description='Suppress all output (launch logs + node logs)'
    )
    def configure_logging(context, *args, **kwargs):
        if silent.perform(context) == 'true':
            import logging
            logging.getLogger().setLevel(logging.CRITICAL)
        return []

    result_path = LaunchConfiguration('result_path')
    result_path_arg = DeclareLaunchArgument(
        'result_path',
        default_value='~/suave/results',
        description='Path where to save the results'
    )

    result_filename = LaunchConfiguration('result_filename')
    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='',
        description='Name of the results file'
    )

    pkg_suave_planta = get_package_share_directory(
        'suave_planta')
    suave_planta_base_launch_path = os.path.join(
        pkg_suave_planta,
        'launch',
        'suave_base.launch.py')

    mission_config_default = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml'
    )

    mission_config = LaunchConfiguration('mission_config')
    mission_config_arg = DeclareLaunchArgument(
        'mission_config',
        default_value=mission_config_default,
        description='Mission config full path'
    )

    suave_planta_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_planta_base_launch_path),
        launch_arguments={
            'result_path': result_path,
            'result_filename': result_filename,
            'mission_config': mission_config,
        }.items()
    )

    suave_planta_path = get_package_share_directory('suave_planta')
    suave_ontology_path = os.path.join(
        suave_planta_path,
        'owl',
        'suave.owl'
    )
    suave_pddl_path = os.path.join(
        suave_planta_path,
        'pddl'
    )
    suave_domain_path = os.path.join(
        suave_pddl_path,
        'suave_domain.pddl'
    )
    suave_domain_out_path = os.path.join(
        suave_pddl_path,
        'suave_domain_created.pddl'
    )
    suave_problem_path = os.path.join(
        suave_pddl_path,
        'suave_problem.pddl'
    )
    suave_problem_out_path = os.path.join(
        suave_pddl_path,
        'suave_problem_created.pddl'
    )

    owl_to_pddl = Node(
        package='owl_to_pddl',
        executable='owl_to_pddl.py',
        parameters=[{
            'owl_file': suave_ontology_path,
            'in_domain_file': suave_domain_path,
            'out_domain_file': suave_domain_out_path,
            'in_problem_file': suave_problem_path,
            'out_problem_file': suave_problem_out_path,
            'add_numbers': True,
            'replace_output': True,
            'ignore_data_properties': False
        }]
    )

    plansys_path = get_package_share_directory('plansys2_bringup')
    plansys2_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                plansys_path,
                'launch',
                'plansys2_bringup_launch_distributed.py')),
            launch_arguments={
                'model_file':
                    suave_domain_out_path,
                'problem_file':
                    suave_problem_out_path,
                'params_file':
                    suave_planta_path + '/config/plansys2_params.yaml',
            }.items()
        )

    suave_planta_controller_node = Node(
        package='suave_planta',
        executable='suave_planta_controller',
        name="mission_node",
        parameters=[mission_config],
    )

    start_robot_pddl_action_node = Node(
        package='suave_planta',
        executable='action_start_robot',
        name='start_robot_pddl_action_node',
        parameters=[{'action_name': 'start_robot'}]
    )

    search_pipeline_pddl_action_node = Node(
        package='suave_planta',
        executable='action_search_pipeline',
        name='search_pipeline_pddl_action_node',
        parameters=[{'action_name': 'search_pipeline'}]
    )

    inspect_pipeline_pddl_action_node = Node(
        package='suave_planta',
        executable='action_inspect_pipeline',
        name='inspect_pipeline_pddl_action_node',
        parameters=[{'action_name': 'inspect_pipeline'}]
    )

    reconfigure_pddl_action_node = Node(
        package='suave_planta',
        executable='action_reconfigure',
        name='reconfigure_pddl_action_node',
        parameters=[{'action_name': 'reconfigure1'}]
    )
    
    reconfigure2_pddl_action_node = Node(
        package='suave_planta',
        executable='action_reconfigure',
        name='reconfigure_pddl_action_node',
        parameters=[{'action_name': 'reconfigure2'}]
    )

    return LaunchDescription([
        result_path_arg,
        result_filename_arg,
        silent_arg,
        OpaqueFunction(function=configure_logging),
        mission_config_arg,
        suave_planta_base,
        owl_to_pddl,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=owl_to_pddl,
                on_exit=[plansys2_bringup],
            )
        ),
        suave_planta_controller_node,
        start_robot_pddl_action_node,
        search_pipeline_pddl_action_node,
        inspect_pipeline_pddl_action_node,
        reconfigure_pddl_action_node,
        reconfigure2_pddl_action_node
    ])
