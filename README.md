# suave_planta

[![DOI](https://zenodo.org/badge/842123765.svg)](https://doi.org/10.5281/zenodo.15874043)


A PDDL-based managing system for SUAVE.

This repo contains the experimental setup used in the paper "Plan your Self-Adaptation! – Efficient Task and Architecture Co-adaptation Planning for Robots".

The remainder of this README explains how to reproduce the experiments.

## Docker

The first step is to build the `suave_planta` docker image, if you want to have it locally:

```Bash
docker build -t suave_planta .
```

Run docker image:

Run docker image without web interface (with nvidia)(don't forget to install the [docker-nvidia-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html))

```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_planta -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v $HOME/suave/results:/home/ubuntu-user/suave/results -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro ghcr.io/kas-lab/suave_planta:main
```

Without a nvidia GPU:
```Bash
docker run -it --rm --name suave_planta -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v $HOME/suave/results:/home/ubuntu-user/suave/results -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro ghcr.io/kas-lab/suave_planta:main
```


**Note:** If you want to use the host machine display, run the following command before running the docker container:
```Bash
xhost +
```

### Development

To develop `suave_planta` with docker, you will need to fetch the repos you want to develop into your host machine and mount the relevant directories inside the container.

For example:

Mount `$HOME/suave/results`, `$HOME/suave_ws/src/suave`, `$HOME/suave_ws/src/suave_planta`, and `$HOME/suave_ws/src/ros2_planning_system`
```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_planta -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/suave/results:/home/ubuntu-user/suave/results -v $HOME/suave_ws/src/suave_planta:/home/ubuntu-user/suave_ws/src/suave_planta -v $HOME/suave_ws/src/ros2_planning_system:/home/ubuntu-user/suave_ws/src/plansys2 -v $HOME/suave_ws/src/suave:/home/ubuntu-user/suave_ws/src/suave suave_planta
```

```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_planta -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/suave/results:/home/ubuntu-user/suave/results -v $PWD/src/suave_planta:/home/ubuntu-user/suave_ws/src/suave_planta -v $PWD/src/ros2_planning_system:/home/ubuntu-user/suave_ws/src/plansys2 -v $PWD/src/suave:/home/ubuntu-user/suave_ws/src/suave suave_planta
```

```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_planta -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/suave/results:/home/ubuntu-user/suave/results -v $PWD:/home/ubuntu-user/suave_ws/src/suave_planta suave_planta
```


## Run SUAVE with PLANTA

Before requesting its first plan, PLANTA waits for PlanSys2 to become active and
for the first water-visibility observation to be applied to the planning problem.
The SUAVE monitor publishes the initial visibility during vehicle startup and
holds the visibility schedule at time zero until GUIDED. This prevents planning
with unknown visibility while preserving the timing of visibility changes.
If applying the initial observation fails, the controller reports a failure
instead of planning with missing visibility.

### Batch runner

To run all six experiment campaigns (`exp1`-`exp3` and `extended_exp1`-`extended_exp3`) sequentially in one go, use the generic `run_batch` node from `suave_runner`, configured through [batch_campaigns.yml](config/runner/batch_campaigns.yml):

```Bash
ros2 launch suave_planta run_batch.launch.py
```

**Note:** running all six campaigns back-to-back takes several hours to a few days, depending on the machine and `num_runs` per experiment — plan to leave it running unattended (e.g. in `screen`/`tmux`) rather than waiting on it.

This creates a timestamped batch directory under `~/suave/results/batches/` (e.g. `batch_20260913_104200/`) containing:
- `state.json` — the batch checkpoint, tracking each campaign's status (`pending`, `running`, `completed`, `incomplete`, `failed`, or `interrupted`)
- `run_batch.log` — the overall orchestration log
- `<campaign_name>.log` — combined stdout/stderr for each campaign (e.g. `exp1.log`)
- `campaigns/<campaign_name>/` — each campaign's own results directory, in the same layout `suave_runner` produces for a single campaign

**Checking whether a batch needs to be resumed:** a batch finished cleanly if `run_batch.log` ends with `All campaigns completed successfully.`. If it stops early — Ctrl+C, a crash, a failed campaign — the log ends instead with a `Batch interrupted; ...` or `Batch finished with N incomplete, ...` line followed by the exact resume command to use. You can also check campaign-by-campaign at any time by inspecting `state.json`:

```Bash
grep '"status"' ~/suave/results/batches/<batch_dir>/state.json
```

Any campaign not marked `"status": "completed"` still needs to (re)run.

**Resuming an interrupted batch:** pass the batch's `state.json` as the `resume_state_file` parameter; campaigns already marked `completed` are skipped, and an `incomplete` campaign picks up from its last saved run instead of starting over:

```Bash
ros2 run suave_runner run_batch --ros-args -p resume_state_file:=~/suave/results/batches/<batch_dir>/state.json
```

**Note:** to run a custom subset or order of campaigns, edit [batch_campaigns.yml](config/runner/batch_campaigns.yml) (or point `run_batch` at your own copy via `--params-file`) before starting a new batch. `-p fail_fast:=true` stops the batch as soon as a campaign fails instead of continuing to the next one, and `-p dry_run:=true` prints the `ros2 run` command for each campaign without executing anything.

### With Individual Runner

#### SUAVE
You can run it with the launchfile:

Experiment 1:
```Bash
ros2 launch suave_planta exp1_suave_runner.launch.py
```

Experiment 2:
```Bash
ros2 launch suave_planta exp2_suave_runner.launch.py
```

Experiment 3:
```Bash
ros2 launch suave_planta exp3_suave_runner.launch.py
```

**Note:** with the default configuration and launch files above, the results will be save in the directory `~/suave/results`

#### SUAVE extended

Experiment 1:
```Bash
ros2 launch suave_planta extended_exp1_suave_runner.launch.py
```

Experiment 2:
```Bash
ros2 launch suave_planta extended_exp2_suave_runner.launch.py
```

Experiment 3:
```Bash
ros2 launch suave_planta extended_exp3_suave_runner.launch.py
```

**Note:** with the default configuration and launch files above, the results will be save in the directory `~/suave/results`

### Custom runner config

**Changing experiments config:** Simply create a new configuration file for the runner, for example by modifying the [exp3_runner_config.yml](config/runner/exp3_runner_config.yml) file. Then, pass it to the `suave_runner`node, for example, by modifying the [exp3_suave_runner.launch.py](launch/runner/exp3_suave_runner.launch.py) launch file.

Alternatively, you can run the `suave_runner` node directly with the parameters you want. Check some examples below:

PLANTA:
```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiment_logging:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_planta suave_planta.launch.py\", \
      \"num_runs\": 1, \
      \"adaptation_manager\": \"planta\", \
      \"mission_name\": \"suave\"}"
  ]'
```

SUAVE extended with PLANTA:
```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiment_logging:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_planta suave_planta_extended.launch.py\", \
      \"num_runs\": 20, \
      \"adaptation_manager\": \"planta\", \
      \"mission_name\": \"suave_extended\"}"
  ]'
```

SUAVE with no managing subsystem:
```bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiment_logging:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_none suave_none.launch.py\", \
      \"num_runs\": 1, \
      \"adaptation_manager\": \"none\", \
      \"mission_name\": \"suave\"}"
  ]'
```

### Manually

Run ardusub:
```Bash
sim_vehicle.py -L RATBeach -v ArduSub  --model=JSON --console
```

Run the simulation:
```Bash
ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0
```

Run PLANTA:
```Bash
ros2 launch suave_planta suave_planta.launch.py
```

## Run the experimental analysis

The launches below analyze the bundled, run-matched `*_sorted.csv` files
using the paired Wilcoxon signed-rank test with Holm correction. Standard
SUAVE inputs are in `results/suave/exp1`, `exp2`, and `exp3`; extended inputs
are in the corresponding `results/suave_extended/` folders. These launches
replace the old Mann-Whitney configurations that referenced raw CSVs no
longer present in `results/`.

Paths are resolved from the installed `suave_planta` package rather than a
particular user's home or workspace path. With a symlink installation,
the input files point back to the source package's bundled results. Rebuild
after updating the launch/config files and result files:

```bash
colcon build --symlink-install --packages-select suave_runner suave_planta
source install/setup.bash
```

### SUAVE

Experiment 1:
```Bash
ros2 launch suave_planta exp1_analysis.launch.py
```

Experiment 2:
```Bash
ros2 launch suave_planta exp2_analysis.launch.py
```

Experiment 3:
```Bash
ros2 launch suave_planta exp3_analysis.launch.py
```

Outputs are written beside the inputs by default, using the prefix
`expN_wilcoxon` (or `extended_expN_wilcoxon`). To keep the bundled outputs
intact, supply a separate output root, for example:

```bash
ros2 launch suave_planta exp1_analysis.launch.py output_root:=/tmp/planta_analysis
```

This writes under `/tmp/planta_analysis/suave/exp1/`. The extended launch
uses `suave_extended/expN/` under the selected root.

### SUAVE extended

Experiment 1:
```Bash
ros2 launch suave_planta extended_exp1_analysis.launch.py
```

Experiment 2:
```Bash
ros2 launch suave_planta extended_exp2_analysis.launch.py
```

Experiment 3:
```Bash
ros2 launch suave_planta extended_exp3_analysis.launch.py
```

For example, save extended experiment 1 outputs to another directory:

```bash
ros2 launch suave_planta extended_exp1_analysis.launch.py \
  output_root:=/tmp/planta_analysis
```

### Custom analysis

Edit the [analysis configs](config/analysis/) to choose the sorted method
CSV files. The YAML node namespace is `/wilcoxon_analysis`. File paths use
`$(var results_root)` and `$(var output_root)` substitutions expanded by the
launch files; these configs are intended to be used through the launches.

Each single-experiment launch accepts:

| Argument | Default | Purpose |
|---|---|---|
| `results_root` | Installed package's `results/` | Root containing `suave/` and `suave_extended/` |
| `output_root` | Same as `results_root` | Root for generated CSVs in the same experiment layout |
| `correction` | `holm` | `holm` or explicit `none` |

Both metrics exclude a pair when either method did not find the pipeline.
Search time uses the `less` alternative and distance uses `greater` for
row method minus column method. Holm correction pools the computed tests
across both metrics within that experiment. Use `p_adjusted` in
`*_wilcoxon_results.csv` for corrected comparisons; the two secondary
matrices and the node's console significance labels use raw p-values.


### Batch analysis and LaTeX tables

`run_batch.launch.py` writes a batch root containing `state.json` and
`campaigns/<experiment>/`. By default it creates
`~/suave/results/batches/batch_YYYYMMDD_HHMMSS` and prints the exact batch
directory. Pass that same root to the new analysis launch:

```bash
ros2 launch suave_planta run_batch.launch.py
# After the run finishes, use the batch directory printed by the runner:
ros2 launch suave_planta batch_analysis.launch.py \
  batch_dir:=/home/ubuntu-user/suave/results/batches/batch_YYYYMMDD_HHMMSS
```

Alternatively, choose an explicit directory for both commands. Run analysis
after the batch completes:

```bash
ros2 launch suave_planta run_batch.launch.py \
  batch_dir:=/home/ubuntu-user/suave/results/batches/my_batch
ros2 launch suave_planta batch_analysis.launch.py \
  batch_dir:=/home/ubuntu-user/suave/results/batches/my_batch
```

The runner requires a new directory for a fresh batch. To resume an existing
batch, leave `batch_dir` empty and supply
`resume_state_file:=/path/to/batch/state.json` instead.

The analysis launch accepts the **batch root**, not its `campaigns/` child
and not the bundled `suave_planta/results/` tree. It automatically sorts raw
CSV files using their original completion markers, then runs paired Wilcoxon
analysis. The input directory must come from a completed matching campaign;
incomplete runs and unsafe reconstruction are reported as errors.

| Batch analysis argument | Default | Purpose |
|---|---|---|
| `batch_dir` | Required | Exact output root of the batch runner |
| `output_root` | `<batch_dir>/campaings_results` | Root for analysis outputs |
| `config_dir` | Config paths recorded in `state.json` | Override with original configs if those paths moved |
| `correction` | `holm` | Correction separately within each experiment |

Results go to `<batch_dir>/campaings_results/<experiment>/wilcoxon_analysis/`.
For the older batch discussed in this project, the same launch works with
`batch_dir:=/home/ubuntu-user/suave/results/batches/all_experiments_20260904_093305`.
If necessary, add `config_dir:=/path/to/original/campaign/configs`; do not
substitute configs from a different campaign.

Q-Q plots and LaTeX tables remain available through the batch scripts:

```bash
python3 src/suave/suave_runner/suave_runner/analysis/qq_plot_batch.py /path/to/batch
python3 src/suave/suave_runner/suave_runner/latex/wilcoxon_latex_tables_batch.py /path/to/batch
```

Q-Q plots use `campaings_results/<experiment>/q-q-plots/`. Tables read the
existing Wilcoxon analysis and use
`campaigns_latex_tables/wilcoxon_analysis/`, with Holm-adjusted p-values by
default when the analysis used Holm correction. The standalone analysis
command is still available in `suave_runner/suave_runner/analysis/`.

See [suave_runner's guide](../suave/suave_runner/README.md) for details.

## OWL to PDDL

If you want to try the OWL to PDDL conversion separately, check the examples below.

With ROS:
```bash
ros2 run owl_to_pddl owl_to_pddl.py --ros-args -p owl_file:=owl/suave.owl -p in_domain_file:=pddl/suave_domain.pddl -p out_domain_file:=pddl/suave_domain_created.pddl -p in_problem_file:=pddl/suave_problem.pddl -p out_problem_file:=pddl/suave_problem_created.pddl -p add_numbers:=true -p replace_output:=true
```

SUAVE extended
```bash
ros2 run owl_to_pddl owl_to_pddl.py --ros-args -p owl_file:=owl/suave_extended.owl -p in_domain_file:=pddl/suave_domain_extended.pddl -p out_domain_file:=pddl/suave_domain_extended_created.pddl -p in_problem_file:=pddl/suave_problem_extended.pddl -p out_problem_file:=pddl/suave_problem_extended_created.pddl -p add_numbers:=true -p replace_output:=true
```

Without ROS:
```bash
OWLToPDDL.sh --owl=owl/suave.owl --tBox --inDomain=pddl/suave_domain.pddl --outDomain=pddl/suave_domain_created.pddl --aBox --inProblem=pddl/suave_problem.pddl --outProblem=pddl/suave_problem_created.pddl --add-num-comparisons --replace-output
```

SUAVE extended
```bash
OWLToPDDL.sh --owl=owl/suave_extended.owl --tBox --inDomain=pddl/suave_domain_extended.pddl --outDomain=pddl/suave_domain_extended_created.pddl --aBox --inProblem=pddl/suave_problem_extended.pddl --outProblem=pddl/suave_problem_extended_created.pddl --add-num-comparisons --replace-output
```

## Planning with fast downward

If you want to run the planner individually, check the examples below.

```bash
ros2 run downward_ros fast-downward.py pddl/suave_domain_created.pddl pddl/suave_problem_created.pddl --search 'astar(blind())'
```

SUAVE extended:
```bash
ros2 run downward_ros fast-downward.py pddl/suave_domain_extended_created.pddl pddl/suave_problem_extended_created.pddl --search 'astar(blind())'
```

## Count PDDL complexity

```bash
ros2 run owl_to_pddl count_pddl_complexity.py pddl/suave_domain_created.pddl pddl/suave_problem_created.pddl
```

```bash
ros2 run owl_to_pddl count_pddl_complexity.py pddl/suave_domain_extended_created.pddl pddl/suave_problem_extended_created.pddl
```
