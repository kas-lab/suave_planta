---
name: rebuild-owl-to-pddl-jar
description: Use before running or debugging anything that goes through owl_to_pddl (suave_planta.launch.py, OWLToPDDL.sh, the owl_to_pddl ROS node) when Kotlin/Java sources under src/owl_to_pddl/src or build.gradle.kts have changed more recently than the built JAR. The JAR is a separate build step from colcon and is easy to forget, so ontology/domain changes silently don't take effect until it's rebuilt.
---

# Rebuild the OWL-to-PDDL JAR

`owl_to_pddl` wraps a Kotlin/Gradle JAR (`dlToPlanning-1.0-SNAPSHOT-all.jar`)
that does the actual OWL -> PDDL conversion. `colcon build` does **not**
build this JAR -- it's a separate Gradle build step, so a change to the
Kotlin/Java sources or to `pddl.pddl4j`/`owlapi` handling has no effect on a
launch until the JAR is rebuilt by hand.

## When this bites

`suave_planta.launch.py` (and any other launch that runs `owl_to_pddl`
before PlanSys2 bringup) will run happily against a **stale** JAR with no
error -- it just silently keeps producing the old `*_created.pddl` output.
If you changed anything under `src/owl_to_pddl/src/` or
`src/owl_to_pddl/build.gradle.kts` and the generated PDDL doesn't reflect
it, rebuild the JAR before debugging further.

## Check whether a rebuild is needed

```bash
cd /home/gus/ros_workspaces/planta_ws/src/owl_to_pddl
find src/main build.gradle.kts -newer build/libs/dlToPlanning-1.0-SNAPSHOT-all.jar 2>/dev/null
```

Any output means a source file is newer than the built JAR -- rebuild. No
output (or the JAR doesn't exist yet) is also a rebuild signal the first
time.

## Rebuild

```bash
cd /home/gus/ros_workspaces/planta_ws/src/owl_to_pddl
./gradlew shadowJar
```

This writes `build/libs/dlToPlanning-1.0-SNAPSHOT-all.jar`. The ROS node
expects it at the colcon-install location,
`install/owl_to_pddl/share/owl_to_pddl/build/libs/dlToPlanning-1.0-SNAPSHOT-all.jar`
(per `CLAUDE.md`). With `colcon build --symlink-install`, this is normally
already a symlink back to `src/owl_to_pddl/build/libs/`, so the Gradle
rebuild alone is usually enough -- confirm with:

```bash
ls -la /home/gus/ros_workspaces/planta_ws/install/owl_to_pddl/share/owl_to_pddl/build/libs/dlToPlanning-1.0-SNAPSHOT-all.jar
```

If it's a real file rather than a symlink (or missing), rebuild the package
itself so the install step re-links it:

```bash
cd /home/gus/ros_workspaces/planta_ws
colcon build --symlink-install --packages-select owl_to_pddl
```

## After rebuilding

Relaunch whatever calls `owl_to_pddl` (e.g.
`ros2 launch suave_planta suave_planta.launch.py`) to regenerate
`*_created.pddl` from the new JAR -- per the `block_generated_pddl` hook and
`CLAUDE.md`, never hand-edit the generated PDDL files directly.
