# suave_planning
A PDDL-based managing system for SUAVE

## OWL to PDDL

```bash
ros2 run owl_to_pddl owl_to_pddl.py --owl=owl/suave.owl --tBox --inDomain=pddl/suave_domain.pddl --outDomain=pddl/suave_domain_created.pddl --aBox --inProblem=pddl/suave_problem.pddl --outProblem=pddl/suave_problem_created.pddl --add-num-comparisons --replace-output
```
## Planning with fast downward

```bash
ros2 run downward_ros fast-downward.py --alias lama-first pddl/suave_domain_created.pddl pddl/suave_problem_created.pddl
```
