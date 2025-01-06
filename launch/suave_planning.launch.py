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

from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


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
    suave_ontology_path = os.path.join(
        suave_planning_path,
        'owl',
        'suave.owl'
    )
    suave_pddl_path = os.path.join(
        suave_planning_path,
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

    owl_to_pddl_path = get_package_share_directory('owl_to_pddl')
    owl_to_pdd_jar = os.path.join(
        owl_to_pddl_path,
        'build/libs/dlToPlanning-1.0-SNAPSHOT-all.jar'
    )
    owl_to_pddl = ExecuteProcess(
        cmd=[
            'java',
            '-jar',
            owl_to_pdd_jar,
            '--owl=' + suave_ontology_path,
            '--tBox',
            '--inDomain=' + suave_domain_path,
            '--outDomain=' + suave_domain_out_path,
            '--aBox',
            '--inProblem=' + suave_problem_path,
            '--outProblem=' + suave_problem_out_path,
            '--add-num-comparisons',
            '--replace-output'
        ])
    # owl_to_pddl = Node(
    #     package='owl_to_pddl',
    #     executable='owl_to_pddl.py',
    #     arguments=[
    #         '--owl=' + suave_ontology_path,
    #         '--tBox',
    #         '--inDomain=' + suave_domain_path,
    #         '--outDomain=' + suave_domain_out_path,
    #         '--aBox',
    #         '--inProblem=' + suave_problem_path,
    #         '--outProblem=' + suave_problem_out_path,
    #         '--add-num-comparisons',
    #         '--replace-output'
    #     ]
    # )

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
                    suave_planning_path + '/config/plansys2_params.yaml',
            }.items()
        )

    suave_planning_controller_node = Node(
        package='suave_planning',
        executable='suave_plansys_controller',
    )

    start_robot_pddl_action_node = Node(
        package='suave_planning',
        executable='action_start_robot',
        name='start_robot_pddl_action_node',
        parameters=[{'action_name': 'start_robot'}]
    )

    search_pipeline_pddl_action_node = Node(
        package='suave_planning',
        executable='action_search_pipeline',
        name='search_pipeline_pddl_action_node',
        parameters=[{'action_name': 'search_pipeline'}]
    )

    inspect_pipeline_pddl_action_node = Node(
        package='suave_planning',
        executable='action_inspect_pipeline',
        name='inspect_pipeline_pddl_action_node',
        parameters=[{'action_name': 'inspect_pipeline'}]
    )

    reconfigure_pddl_action_node = Node(
        package='suave_planning',
        executable='action_reconfigure',
        name='reconfigure_pddl_action_node',
        parameters=[{'action_name': 'reconfigure'}]
    )

    return LaunchDescription([
        mission_type_arg,
        result_filename_arg,
        suave_planning_base,
        owl_to_pddl,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=owl_to_pddl,
                on_exit=[plansys2_bringup],
            )
        ),
        suave_planning_controller_node,
        start_robot_pddl_action_node,
        search_pipeline_pddl_action_node,
        inspect_pipeline_pddl_action_node,
        reconfigure_pddl_action_node,
    ])
