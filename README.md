# suave_planta

A PDDL-based managing system for SUAVE

## Docker

Run docker image without web interface (with nvidia)(don't forget to install the [docker-nvidia-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html))

```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_planta -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v $HOME/suave/results:/home/ubuntu-user/suave/results -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro ghcr.io/kas-lab/suave_planta:main
```

**DEV**
```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_planta -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/suave_ws/src/suave_planta:/home/ubuntu-user/suave_ws/src/suave_planta ghcr.io/kas-lab/suave_planta:latest
```

```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_planta -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v $HOME/suave/results:/home/ubuntu-user/suave/results -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/suave_ws/src/suave_planta:/home/ubuntu-user/suave_ws/src/suave_planta -v $HOME/suave_ws/src/ros2_planning_system:/home/ubuntu-user/suave_ws/src/plansys2 suave_planta
```

## Run SUAVE with PLANTA

### With Runner

You can run it with the launchfile:

```Bash
ros2 launch suave_planta suave_runner_launch.py
```

**Changing experiments config:** Simply change the [runner_config.yml](config/runner_config.yml) file

Or you can run it with the suave_runner ros node:
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

### Manually

```Bash
sim_vehicle.py -L RATBeach -v ArduSub  --model=JSON --console
```

```Bash
ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0
```

```Bash
ros2 launch suave_planta suave_planta.launch.py
```


## OWL to PDDL

With ROS:
```bash
ros2 run owl_to_pddl owl_to_pddl.py --ros-args -p owl_file:=owl/suave.owl -p in_domain_file:=pddl/suave_domain.pddl -p out_domain_file:=pddl/suave_domain_created.pddl -p in_problem_file:=pddl/suave_problem.pddl -p out_problem_file:=pddl/suave_problem_created.pddl
```

Without ROS:
```bash
OWLToPDDL.sh --owl=owl/suave.owl --tBox --inDomain=pddl/suave_domain.pddl --outDomain=pddl/suave_test.pddl --aBox --inProblem=pddl/suave_problem.pddl --outProblem=pddl/suave_test.pddl --ignore-data-props --add-num-comparisons
```

## Planning with fast downward

```bash
ros2 run downward_ros fast-downward.py pddl/suave_domain_created.pddl pddl/suave_problem_created.pddl --search 'astar(blind())'
```
