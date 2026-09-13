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

"""Check portable analysis paths and batch-launch argument forwarding."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


ROOT = Path(__file__).parents[1]
CAMPAIGNS = ['exp1', 'exp2', 'exp3',
             'extended_exp1', 'extended_exp2', 'extended_exp3']


def _load(relative, monkeypatch):
    path = ROOT / 'launch' / relative
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if hasattr(module, 'get_package_share_directory'):
        monkeypatch.setattr(module, 'get_package_share_directory',
                            lambda package: str(ROOT))
    return module


def _context(description, overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


@pytest.mark.parametrize('campaign', CAMPAIGNS)
def test_single_launch_resolves_sorted_paths(
        campaign, tmp_path, monkeypatch):
    """Use portable sorted inputs and an overridable output root."""
    module = _load(f'analysis/{campaign}_analysis.launch.py', monkeypatch)
    description = module.generate_launch_description()
    context = _context(description, {'output_root': str(tmp_path)})
    node = next(action for action in description.entities
                if isinstance(action, Node))
    parameter_file = node._Node__parameters[0]
    try:
        parameters = evaluate_parameters(context, [parameter_file])
        data = yaml.safe_load(Path(parameters[0]).read_text())
        params = data['/wilcoxon_analysis']['ros__parameters']
        assert params['correction'] == 'holm'
        assert params['filename'] == f'{campaign}_wilcoxon'
        expected_type = ('suave_extended'
                         if campaign.startswith('extended') else 'suave')
        experiment = campaign.split('_')[-1]
        assert Path(params['result_path']) == tmp_path / expected_type / experiment
        entries = [json.loads(item) for item in params['data_files']]
        assert len(entries) == (3 if expected_type == 'suave_extended' else 6)
        for entry in entries:
            source = Path(entry['data_file'])
            assert source.parent == ROOT / 'results' / expected_type / experiment
            assert source.name.endswith('_sorted.csv')
        assert 'rosa_bt' in {entry['managing_system'] for entry in entries}
    finally:
        parameter_file.cleanup()


def test_batch_launch_passes_runner_batch_root(tmp_path, monkeypatch):
    """Forward the batch root and overrides to the installed module."""
    (tmp_path / 'campaigns').mkdir()
    module = _load('analysis/batch_analysis.launch.py', monkeypatch)
    context = _context(module.generate_launch_description(), {
        'batch_dir': str(tmp_path), 'correction': 'none',
        'output_root': str(tmp_path / 'output'),
        'config_dir': str(tmp_path / 'configs'),
    })
    actions = module._launch_analysis(context)
    process = next(action for action in actions
                   if isinstance(action, ExecuteProcess))
    command = [perform_substitutions(
        context, normalize_to_list_of_substitutions(part))
        for part in process.cmd]
    assert command[1:] == [
        '-m', 'suave_runner.analysis.wilcoxon_analysis_batch', str(tmp_path),
        '--correction', 'none', '--output', str(tmp_path / 'output'),
        '--config-dir', str(tmp_path / 'configs')]


def test_batch_launch_reports_wrong_directory(tmp_path, monkeypatch):
    """Reject a folder that is not the batch runner's output root."""
    module = _load('analysis/batch_analysis.launch.py', monkeypatch)
    context = _context(module.generate_launch_description(), {
        'batch_dir': str(tmp_path)})
    with pytest.raises(ValueError, match='expected the batch root'):
        module._launch_analysis(context)
    with pytest.raises(RuntimeError, match='exit code 1'):
        module._check_exit(SimpleNamespace(returncode=1), context)
    assert module._check_exit(SimpleNamespace(returncode=0), context) == []


def test_runner_forwards_batch_directory(tmp_path, monkeypatch):
    """Allow the runner and analysis launches to use the same chosen path."""
    module = _load('runner/run_batch.launch.py', monkeypatch)
    description = module.generate_launch_description()
    context = _context(description, {'batch_dir': str(tmp_path)})
    node = next(action for action in description.entities
                if isinstance(action, Node))
    params = evaluate_parameters(context, node._Node__parameters)[1]
    assert params['batch_dir'] == str(tmp_path)
    assert params['resume_state_file'] == ''
