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