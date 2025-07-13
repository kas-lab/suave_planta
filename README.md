# suave_planta

[![DOI](https://zenodo.org/badge/842123765.svg)](https://doi.org/10.5281/zenodo.15874043)


A PDDL-based managing system for SUAVE.

This repo contains the experimental setup used in the paper "Plan your Self-Adaptation! – Efficient Task and Architecture Co-adaptation Planning for Robots".

The remainder of this README explains how to reproduce the experiments.

## Docker

The first step is to run the `suave_planta` docker image.

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

## Run SUAVE with PLANTA

### With Runner

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

**Changing experiments config:** Simply create a new configuration file for the runner, for example by modifying the [exp3_runner_config.yml](config/exp3_runner_config.yml) file. Then, pass it to the `suave_runner`node, for example, by modifying the [exp3_suave_runner.launch.py](config/exp3_suave_runner.launch.py) launch file.

Alternatively, you can run the `suave_runner` node directly with the parameters you want. Check some examples below:

PLANTA:
```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiment_logging:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_planta suave_planta.launch.py\", \
      \"num_runs\": 20, \
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

To reproduce exactly the analysis done for the paper, run the commands below.

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

**Note:** the results are saved in the results folder in this package

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

**Note:** the results are saved in the results folder in this package

### Custom analysis

**Changing analysis config:** Simply create a new configuration file for the analysis node, for example by modifying the [exp3_analysis_config.yml](config/exp3_analysis_config.yml) file. Then, pass it to the `statistical_analysis` node, for example, by modifying the [exp3_analysis.launch.py](config/exp3_analysis.launch.py) launch file.


## OWL to PDDL

If you want to try the OWL to PDDL conversion separetly, check the examples below.

With ROS:
```bash
ros2 run owl_to_pddl owl_to_pddl.py --ros-args -p owl_file:=owl/suave.owl -p in_domain_file:=pddl/suave_domain.pddl -p out_domain_file:=pddl/suave_domain_created.pddl -p in_problem_file:=pddl/suave_problem.pddl -p out_problem_file:=pddl/suave_problem_created.pddl
```

SUAVE extended
```bash
ros2 run owl_to_pddl owl_to_pddl.py --ros-args -p owl_file:=owl/suave_extended.owl -p in_domain_file:=pddl/suave_domain_extended.pddl -p out_domain_file:=pddl/suave_domain_extended_created.pddl -p in_problem_file:=pddl/suave_problem_extended.pddl -p out_problem_file:=pddl/suave_problem_extended_created.pddl
```

Without ROS:
```bash
OWLToPDDL.sh --owl=owl/suave.owl --tBox --inDomain=pddl/suave_domain.pddl --outDomain=pddl/suave_domain_test.pddl --aBox --inProblem=pddl/suave_problem.pddl --outProblem=pddl/suave_problem_test.pddl --add-num-comparisons --replace-output
```

SUAVE extended
```bash
OWLToPDDL.sh --owl=owl/suave_extended.owl --tBox --inDomain=pddl/suave_domain_extended.pddl --outDomain=pddl/suave_domain_extended_test.pddl --aBox --inProblem=pddl/suave_problem_extended.pddl --outProblem=pddl/suave_problem_extended_test.pddl --add-num-comparisons --replace-output
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
