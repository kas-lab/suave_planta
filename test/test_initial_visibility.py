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

"""Exercise the controller's visibility gate through ROS services."""

from pathlib import Path
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from plansys2_msgs.srv import (
    AffectParam, GetDomain, GetPlan, GetProblem, GetProblemGoal, GetStates,
    UpdateNodes,
)
import pytest
import rclpy
from rclpy.node import Node


class PlanningServices:
    """Provide controlled PlanSys2 responses to the real controller process."""

    def __init__(self, node):
        """Create services with lifecycle readiness initially withheld."""
        self.node = node
        self.active = False
        self.reject_update = False
        self.predicates = []
        self.plans = []
        self.predicate_reads = 0
        self.lifecycle_reads = 0
        for name in ('domain_expert', 'problem_expert', 'planner', 'executor'):
            node.create_service(GetState, name + '/get_state', self.get_state)
        for service_type, name, callback in (
            (AffectParam, 'problem_expert/add_problem_instance', self.success),
            (GetStates, 'problem_expert/get_problem_predicates',
             self.get_predicates),
            (UpdateNodes, 'problem_expert/update_problem_predicates',
             self.update_predicates),
            (GetDomain, 'domain_expert/get_domain', self.success),
            (GetProblem, 'problem_expert/get_problem', self.get_problem),
            (GetPlan, 'planner/get_plan', self.get_plan),
            (GetProblemGoal, 'problem_expert/get_problem_goal', self.success),
        ):
            node.create_service(service_type, name, callback)

    def get_state(self, request, response):
        """Expose a controllable lifecycle startup delay."""
        self.lifecycle_reads += 1
        response.current_state.id = (
            State.PRIMARY_STATE_ACTIVE if self.active
            else State.PRIMARY_STATE_INACTIVE)
        return response

    def success(self, request, response):
        """Acknowledge a supporting request."""
        response.success = True
        return response

    def get_predicates(self, request, response):
        """Return the predicates committed by earlier updates."""
        self.predicate_reads += 1
        response.success = True
        response.states = self.predicates
        return response

    def update_predicates(self, request, response):
        """Commit the update unless the test injects a failure."""
        response.success = not self.reject_update
        if response.success:
            self.predicates = [
                p for p in self.predicates if p not in request.remove_nodes]
            self.predicates.extend(request.add_nodes)
        else:
            response.error_info = 'Injected update failure'
        return response

    def get_problem(self, request, response):
        """Serialize committed predicates for inspection at plan time."""
        response.success = True
        response.problem = '\n'.join(
            '(' + p.name + ' ' + ' '.join(a.name for a in p.parameters) + ')'
            for p in self.predicates)
        return response

    def get_plan(self, request, response):
        """Record the problem without starting vehicle actions."""
        self.plans.append(request.problem)
        response.success = False
        response.error_info = 'Test stops at the planning boundary'
        return response


@pytest.fixture
def controller(tmp_path):
    rclpy.init()
    namespace = '/visibility_test_' + uuid.uuid4().hex
    node = Node('planning_services', namespace=namespace)
    services = PlanningServices(node)
    publisher = node.create_publisher(DiagnosticArray, 'diagnostics', 10)
    executable = (Path(get_package_prefix('suave_planta')) / 'lib' /
                  'suave_planta' / 'suave_planta_controller')
    log_path = tmp_path / 'controller.log'
    with log_path.open('w') as output:
        process = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__ns:=' + namespace,
            '-r', '/diagnostics:=' + namespace + '/diagnostics',
        ], stdout=output, stderr=subprocess.STDOUT)
        try:
            yield node, services, publisher, process, log_path
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


def spin_until(controller, condition, timeout=10):
    """Service ROS requests until an observable condition holds."""
    node, _, _, _, log_path = controller
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition():
            return
    pytest.fail('Timed out waiting for controller:\n' + log_path.read_text())


def publish_qa(publisher, key, value):
    """Publish one observation using the monitor's diagnostic format."""
    publisher.publish(DiagnosticArray(status=[DiagnosticStatus(
        message='QA status', values=[KeyValue(key=key, value=value)])]))


@pytest.mark.parametrize('visibility_before_readiness', [False, True])
def test_first_plan_uses_applied_visibility(
        controller, visibility_before_readiness):
    """Require visibility for either ordering of monitor and PlanSys2 startup."""
    _, services, publisher, process, log_path = controller
    spin_until(controller, lambda: publisher.get_subscription_count() > 0)
    spin_until(controller, lambda: services.lifecycle_reads >= 4)
    if visibility_before_readiness:
        publish_qa(publisher, 'water_visibility', '2.5')
        reads = services.lifecycle_reads
        spin_until(controller, lambda: services.lifecycle_reads >= reads + 4)
        assert services.predicate_reads == 0
        assert services.plans == []

    services.active = True
    if not visibility_before_readiness:
        spin_until(controller, lambda: 'Waiting for initial water visibility'
                   in log_path.read_text())
        # An unrelated QA must not release the planning gate.
        publish_qa(publisher, 'battery_level', '1.0')
        spin_until(controller, lambda: services.predicate_reads > 0)
        assert services.plans == []
        publish_qa(publisher, 'water_visibility', '2.5')

    spin_until(controller, lambda: bool(services.plans))
    assert process.poll() is None
    problem = services.plans[0]
    assert '(qa_has_value obs_water_visibility 2.50_decimal)' in problem
    assert '(lessthan 2.50_decimal 3.25_decimal)' in problem
    assert '(lessthan 2.25_decimal 2.50_decimal)' in problem


def test_rejected_visibility_update_cannot_release_planning(controller):
    """Fail explicitly instead of planning after an unsuccessful update."""
    _, services, publisher, process, log_path = controller
    services.active = True
    services.reject_update = True
    spin_until(controller, lambda: 'Waiting for initial water visibility'
               in log_path.read_text())
    spin_until(controller, lambda: publisher.get_subscription_count() > 0)
    publish_qa(publisher, 'water_visibility', '2.5')
    spin_until(controller, lambda: process.poll() is not None)
    assert process.returncode != 0
    assert services.plans == []
    assert 'Failed to apply initial water visibility' in log_path.read_text()
