#!/usr/bin/env python3
"""Run every SUAVE PLANTA experiment campaign sequentially."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import TextIO

import yaml


CAMPAIGN_CONFIGS = {
    'exp1_suave_runner.launch.py': 'runner/exp1_runner_config.yml',
    'exp2_suave_runner.launch.py': 'runner/exp2_runner_config.yml',
    'exp3_suave_runner.launch.py': 'runner/exp3_runner_config.yml',
    'extended_exp1_suave_runner.launch.py':
        'runner/extended_exp1_runner_config.yml',
    'extended_exp2_suave_runner.launch.py':
        'runner/extended_exp2_runner_config.yml',
    'extended_exp3_suave_runner.launch.py':
        'runner/extended_exp3_runner_config.yml',
}
CAMPAIGNS = tuple(CAMPAIGN_CONFIGS)

active_process: subprocess.Popen[str] | None = None
received_signal: int | None = None
signal_count = 0


def timestamp() -> str:
    """Return a readable local timestamp."""
    return datetime.now().astimezone().isoformat(timespec='seconds')


def emit(message: str, log: TextIO | None = None) -> None:
    """Print a timestamped orchestration message and optionally log it."""
    line = f'[{timestamp()}] {message}'
    print(line, flush=True)
    if log is not None:
        log.write(line + '\n')
        log.flush()


def write_state(state_file: Path, state: dict) -> None:
    """Atomically persist campaign state."""
    state['updated_at'] = timestamp()
    temporary_file = state_file.with_name(f'.{state_file.name}.tmp')
    temporary_file.write_text(
        json.dumps(state, indent=2) + '\n', encoding='utf-8')
    os.replace(temporary_file, state_file)


def load_state(state_file: Path) -> dict:
    """Load and minimally validate a prior campaign state file."""
    try:
        state = json.loads(state_file.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(
            f'cannot read state file {state_file}: {error}') from error

    if state.get('schema_version') != 1:
        raise ValueError(f'unsupported state-file schema in {state_file}')
    if not isinstance(state.get('campaigns'), dict):
        raise ValueError(f'invalid campaign data in {state_file}')

    for campaign in CAMPAIGNS:
        state['campaigns'].setdefault(campaign, {'status': 'pending'})
    return state


def new_state() -> dict:
    """Create state for a fresh six-campaign batch."""
    return {
        'schema_version': 1,
        'created_at': timestamp(),
        'updated_at': timestamp(),
        'campaign_order': list(CAMPAIGNS),
        'campaigns': {
            campaign: {'status': 'pending'} for campaign in CAMPAIGNS
        },
    }


def signal_handler(signum: int, _frame: object) -> None:
    """Forward termination to the complete active ros2 process group."""
    global received_signal, signal_count
    signal_count += 1
    if received_signal is None:
        received_signal = signum

    process = active_process
    if process is None or process.poll() is not None:
        return

    forwarded_signal = signum if signal_count == 1 else signal.SIGKILL
    try:
        os.killpg(process.pid, forwarded_signal)
    except ProcessLookupError:
        pass


def check_ros_environment() -> tuple[Path | None, str]:
    """Return the installed PLANTA config directory when ROS is ready."""
    if shutil.which('ros2') is None:
        return None, 'ros2 is unavailable; source the ROS workspace first'

    suave_planta_prefix = None
    for package in ('suave_planta', 'suave_runner'):
        check = subprocess.run(
            ['ros2', 'pkg', 'prefix', package],
            capture_output=True,
            text=True,
            check=False,
        )
        if check.returncode != 0:
            return None, f'ROS package {package!r} is not discoverable'
        if package == 'suave_planta':
            suave_planta_prefix = Path(check.stdout.strip())

    assert suave_planta_prefix is not None
    config_dir = (
        suave_planta_prefix / 'share' / 'suave_planta' / 'config')
    missing_configs = [
        name for name in CAMPAIGN_CONFIGS.values()
        if not (config_dir / name).is_file()
    ]
    if missing_configs:
        return None, (
            f'missing installed runner config: {missing_configs[0]}; '
            'rebuild the workspace')

    return config_dir, ''


def expected_runs(config_file: Path) -> int:
    """Return the total number of runs declared in a runner config."""
    try:
        config = yaml.safe_load(config_file.read_text(encoding='utf-8'))
        experiments = config[
            '/suave_runner_node']['ros__parameters']['experiments']
        return sum(
            int(json.loads(experiment).get('num_runs', 1))
            for experiment in experiments
        )
    except (OSError, KeyError, TypeError, ValueError,
            json.JSONDecodeError, yaml.YAMLError) as error:
        raise ValueError(
            f'cannot read experiments from {config_file}: {error}') from error


def completed_runs(result_path: Path) -> int:
    """Count successful run markers in a runner result directory."""
    return sum(1 for _ in result_path.glob('run_*_*.done'))


def run_campaign(
        config_file: Path,
        result_path: Path,
        log_file: Path) -> tuple[int, float]:
    """Run one runner config and mirror its combined output into a log."""
    global active_process
    command = [
        'ros2', 'run', 'suave_runner', 'suave_runner',
        '--ros-args',
        '--params-file', str(config_file),
        '-p', f'resume_result_path:={result_path}',
    ]
    started_at = time.monotonic()
    process: subprocess.Popen[str] | None = None

    try:
        with log_file.open('a', encoding='utf-8') as campaign_log:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding='utf-8',
                errors='replace',
                bufsize=1,
                start_new_session=True,
            )
            active_process = process
            if received_signal is not None:
                os.killpg(process.pid, received_signal)
            assert process.stdout is not None
            for line in process.stdout:
                sys.stdout.write(line)
                sys.stdout.flush()
                campaign_log.write(line)
                campaign_log.flush()
            return_code = process.wait()
    finally:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        active_process = None

    return return_code, time.monotonic() - started_at


def parse_arguments() -> argparse.Namespace:
    """Parse command-line options."""
    parser = argparse.ArgumentParser(
        description='Run all six SUAVE PLANTA experiment campaigns in order.')
    destination = parser.add_mutually_exclusive_group()
    destination.add_argument(
        '--batch-dir',
        type=Path,
        help='directory for new-batch state and orchestration logs',
    )
    destination.add_argument(
        '--resume',
        type=Path,
        metavar='STATE_FILE',
        help='resume a batch, skipping campaigns marked completed',
    )
    parser.add_argument(
        '--fail-fast',
        action='store_true',
        help=(
            'stop after a campaign process fails; incomplete campaigns '
            'always continue'),
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='print the launch commands without executing or writing files',
    )
    return parser.parse_args()


def prepare_batch(arguments: argparse.Namespace) -> tuple[Path, Path, dict]:
    """Create a new batch directory or load an existing checkpoint."""
    if arguments.resume is not None:
        state_file = arguments.resume.expanduser().resolve()
        if not state_file.is_file():
            raise ValueError(f'state file does not exist: {state_file}')
        return state_file.parent, state_file, load_state(state_file)

    if arguments.batch_dir is not None:
        batch_dir = arguments.batch_dir.expanduser().resolve()
    else:
        name = datetime.now().strftime('all_experiments_%Y%m%d_%H%M%S')
        batch_dir = (Path('~/suave/results/batches').expanduser() / name)

    if batch_dir.exists():
        raise ValueError(
            f'batch directory already exists: {batch_dir}; use --resume')
    batch_dir.mkdir(parents=True)
    state_file = batch_dir / 'state.json'
    state = new_state()
    write_state(state_file, state)
    return batch_dir, state_file, state


def print_summary(state: dict, log: TextIO) -> None:
    """Print the status of every campaign."""
    emit('Campaign summary:', log)
    for campaign in CAMPAIGNS:
        status = state['campaigns'][campaign].get('status', 'unknown')
        emit(f'  {campaign}: {status}', log)


def main() -> int:
    """Run or resume the complete experiment batch."""
    arguments = parse_arguments()

    if arguments.dry_run:
        for campaign in CAMPAIGNS:
            config_file = (
                Path('<suave_planta_share>') / 'config' /
                CAMPAIGN_CONFIGS[campaign])
            result_path = (
                Path('<batch_dir>') / 'campaigns' /
                campaign.removesuffix('_suave_runner.launch.py'))
            command = [
                'ros2', 'run', 'suave_runner', 'suave_runner',
                '--ros-args',
                '--params-file', str(config_file),
                '-p', f'resume_result_path:={result_path}',
            ]
            print(shlex.join(command))
        return 0

    config_dir, error = check_ros_environment()
    if config_dir is None:
        print(f'Error: {error}', file=sys.stderr)
        return 2

    try:
        run_counts = {
            campaign: expected_runs(
                config_dir / CAMPAIGN_CONFIGS[campaign])
            for campaign in CAMPAIGNS
        }
    except ValueError as error:
        print(f'Error: {error}', file=sys.stderr)
        return 2

    try:
        batch_dir, state_file, state = prepare_batch(arguments)
    except ValueError as error:
        print(f'Error: {error}', file=sys.stderr)
        return 2

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    overall_log_file = batch_dir / 'run_all_experiments.log'
    with overall_log_file.open('a', encoding='utf-8') as overall_log:
        emit(f'Batch directory: {batch_dir}', overall_log)
        emit(f'Checkpoint: {state_file}', overall_log)

        for campaign in CAMPAIGNS:
            campaign_state = state['campaigns'][campaign]
            if campaign_state.get('status') == 'completed':
                emit(f'Skipping completed campaign: {campaign}', overall_log)
                continue

            if received_signal is not None:
                break

            campaign_name = campaign.removesuffix(
                '_suave_runner.launch.py')
            result_path_value = campaign_state.get('result_path')
            if result_path_value:
                result_path = Path(result_path_value).expanduser().resolve()
            else:
                result_path = (
                    batch_dir / 'campaigns' / campaign_name).resolve()

            config_file = config_dir / CAMPAIGN_CONFIGS[campaign]
            campaign_state.update({
                'status': 'running',
                'started_at': timestamp(),
                'finished_at': None,
                'return_code': None,
                'elapsed_seconds': None,
                'error': None,
                'config_file': str(config_file),
                'result_path': str(result_path),
                'expected_runs': run_counts[campaign],
                'completed_runs': completed_runs(result_path),
            })
            write_state(state_file, state)
            emit(f'Starting campaign: {campaign}', overall_log)
            emit(
                f'Resumable results: {result_path}',
                overall_log,
            )

            log_name = campaign.removesuffix('.launch.py') + '.log'
            try:
                return_code, elapsed = run_campaign(
                    config_file, result_path, batch_dir / log_name)
            except OSError as error:
                campaign_state.update({
                    'status': 'failed',
                    'finished_at': timestamp(),
                    'error': str(error),
                    'completed_runs': completed_runs(result_path),
                })
                write_state(state_file, state)
                emit(
                    f'Could not run campaign {campaign}: {error}',
                    overall_log,
                )
                if arguments.fail_fast:
                    break
                continue

            campaign_state.update({
                'finished_at': timestamp(),
                'return_code': return_code,
                'elapsed_seconds': round(elapsed, 3),
                'completed_runs': completed_runs(result_path),
            })

            if received_signal is not None:
                campaign_state['status'] = 'interrupted'
                write_state(state_file, state)
                emit(f'Interrupted during campaign: {campaign}', overall_log)
                break

            all_runs_completed = (
                campaign_state['completed_runs'] ==
                campaign_state['expected_runs'])
            if return_code == 0 and all_runs_completed:
                campaign_state['status'] = 'completed'
                write_state(state_file, state)
                emit(
                    f'Completed campaign: {campaign} ({elapsed / 3600:.2f} h)',
                    overall_log,
                )
                continue

            campaign_state['status'] = (
                'incomplete' if return_code == 0 else 'failed')
            write_state(state_file, state)
            if return_code == 0:
                emit(
                    f'Campaign incomplete: {campaign_state["completed_runs"]}/'
                    f'{campaign_state["expected_runs"]} successful runs',
                    overall_log,
                )
            else:
                emit(
                    f'Campaign failed with exit code {return_code}: '
                    f'{campaign}',
                    overall_log,
                )
            if return_code != 0 and arguments.fail_fast:
                break

        print_summary(state, overall_log)

        if received_signal is not None:
            emit(
                f'Batch interrupted; resume with: {sys.argv[0]} '
                f'--resume {state_file}',
                overall_log,
            )
            return 128 + received_signal
        incomplete_campaigns = [
            campaign for campaign in CAMPAIGNS
            if state['campaigns'][campaign].get('status') == 'incomplete'
        ]
        failed_campaigns = [
            campaign for campaign in CAMPAIGNS
            if state['campaigns'][campaign].get('status') == 'failed'
        ]
        pending_campaigns = [
            campaign for campaign in CAMPAIGNS
            if state['campaigns'][campaign].get('status') == 'pending'
        ]
        if incomplete_campaigns or failed_campaigns or pending_campaigns:
            emit(
                'Batch finished with '
                f'{len(incomplete_campaigns)} incomplete, '
                f'{len(failed_campaigns)} failed, and '
                f'{len(pending_campaigns)} pending campaign(s).',
                overall_log,
            )
            emit(
                f'Retry unfinished runs with: {sys.argv[0]} '
                f'--resume {state_file}',
                overall_log,
            )
            return 1

        emit('All six campaigns completed successfully.', overall_log)
        return 0


if __name__ == '__main__':
    sys.exit(main())
