## SUAVE experiments

time_limit: 200 (make it spend less time inspecting the pipeline)
# Experiment 1 (Thruster failure time variation)
N=20 runs 

# Find a closer waypoint to the pipeline
x = 18 + random(-1,1)
y = -2 + random(+1,1)

Water visibility = 2.5

## Changes (when it occurs(same for all managing subsystems))
/thruster_monitor:
  ros__parameters:
    thruster_events: 
    - (1,failure,35) 
    - (3,failure,35) # change time between every 5 runs

# Experiment 2 (Water visibility variation)
N=20 runs

# Find a closer waypoint to the pipeline
x = 18 + random(-1,1)
y = -2 + random(+1,1)

# Change parameters (random phase shift, and fix period in a higher value)
/water_visibility_observer_node:
  ros__parameters:
    qa_publishing_period: 1.0
    water_visibility_period: 80 # Water visibility period in seconds
    water_visibility_min: 1.25 # Minimum value for water visibility
    water_visibility_max: 3.75 # Maximum value for water visibility
    water_visibility_sec_shift: 0.0 # Water visibility seconds shift to left


# Experiment 3 (Both variation)
N=20 runs 

# Find a closer waypoint to the pipeline
x = 18 + random(-1,1)
y = -2 + random(+1,1)

# Change parameters (random phase shift, and fix period in a higher value)
/water_visibility_observer_node:
  ros__parameters:
    qa_publishing_period: 1.0
    water_visibility_period: 80 # Water visibility period in seconds
    water_visibility_min: 1.25 # Minimum value for water visibility
    water_visibility_max: 3.75 # Maximum value for water visibility
    water_visibility_sec_shift: 0.0 # Water visibility seconds shift to left

## Changes (when it occurs(same for all managing subsystems))
/thruster_monitor:
  ros__parameters:
    thruster_events: 
    - (1,failure,35) 
    - (3,failure,35) # change time between every 5 runs

## Analysis implementation locations

The configs in `analysis/` select the bundled `*_sorted.csv` files under
`results/suave/expN/` and `results/suave_extended/expN/`. Their node namespace
is `/wilcoxon_analysis`, and their matching launches run the installed
`suave_runner` paired Wilcoxon executable with Holm correction by default.

Paths use `$(var results_root)` and `$(var output_root)`, expanded by
`ParameterFile(..., allow_substs=True)` in the launch files. Use the matching
launch rather than passing these YAML files directly to `ros2 run`.

For new runner batches, use `batch_analysis.launch.py batch_dir:=<batch_root>`.
It consumes the `state.json` and `campaigns/` structure produced by
`run_batch.launch.py`, sorting raw CSVs before analysis. The batch's original
runner configs establish the completion-marker indices; these single-run
analysis configs are not used by batch analysis.

See the [analysis guide](../README.md#run-the-experimental-analysis) for
launch arguments and rebuild commands.
