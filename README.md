# suave_planning
A PDDL-based managing system for SUAVE

## OWL to PDDL

<!-- ```bash
ros2 run owl_to_pddl owl_to_pddl.py --ros-args -p --owl=owl/suave.owl --tBox --inDomain=pddl/suave_domain.pddl --outDomain=pddl/suave_domain_created.pddl --aBox --inProblem=pddl/suave_problem.pddl --outProblem=pddl/suave_problem_created.pddl --add-num-comparisons --replace-output
``` -->
```bash
ros2 run owl_to_pddl owl_to_pddl.py --ros-args -p owl_file:=owl/suave.owl -p in_domain_file:=pddl/suave_domain.pddl -p out_domain_file:=pddl/suave_domain_created.pddl -p in_problem_file:=pddl/suave_problem.pddl -p out_problem_file:=pddl/suave_problem_created.pddl
```

## Planning with fast downward

```bash
ros2 run downward_ros fast-downward.py pddl/suave_domain_created.pddl pddl/suave_problem_created.pddl --search 'astar(blind())'
```

## SUAVE

```bash
sim_vehicle.py -L RATBeach -v ArduSub  --model=JSON --console
```

```bash
ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0
```

```bash
ros2 launch suave_planning suave_planning.launch.py
```
