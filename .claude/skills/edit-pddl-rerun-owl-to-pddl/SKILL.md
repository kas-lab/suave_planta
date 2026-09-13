---
name: edit-pddl-rerun-owl-to-pddl
description: Use when changing suave_planta's PDDL action logic (preconditions/effects in suave_domain.pddl / suave_domain_extended.pddl) or the OWL ontologies (owl/suave.owl / owl/suave_extended.owl) and need the *_created.pddl files owl_to_pddl produces to reflect it. Covers which files are hand-written vs generated, the exact OWLToPDDL.sh invocation for both suave and suave_extended, and verifying the result with Fast Downward standalone instead of a full ROS launch.
---

# Edit PDDL/OWL and Regenerate via owl_to_pddl

## Hand-written source vs generated output

| Hand-written (edit these) | Generated (never edit) |
|---|---|
| `pddl/suave_domain.pddl`, `pddl/suave_domain_extended.pddl` | `pddl/suave_domain_created.pddl`, `pddl/suave_domain_extended_created.pddl` |
| `pddl/suave_problem.pddl`, `pddl/suave_problem_extended.pddl` | `pddl/suave_problem_created.pddl`, `pddl/suave_problem_extended_created.pddl` |
| `owl/suave.owl`, `owl/suave_extended.owl` | (consumed to produce the `*_created.pddl` above) |

A `PreToolUse` hook (`.claude/hooks/block_generated_pddl.py`, matches
`*_created.pddl`) rejects `Edit`/`Write` on the generated files outright --
if you hit it, you edited the wrong file. Edit the hand-written domain,
problem, or OWL file instead and regenerate.

**Which one to edit:** action preconditions/effects (new guards, new
actions) go in the hand-written `suave_domain*.pddl`. Architecture/QA facts
-- new individuals, new component/function/FD relationships, new classes or
properties for those individuals -- go in the OWL files. You often need
both for one change (e.g. a new marker class in OWL referenced by a new
precondition clause in the hand-written domain).

## Regenerating

```bash
cd /home/gus/ros_workspaces/planta_ws/src/suave_planta

# suave (base)
/home/gus/ros_workspaces/planta_ws/src/owl_to_pddl/OWLToPDDL.sh \
  --owl=owl/suave.owl --tBox \
  --inDomain=pddl/suave_domain.pddl --outDomain=pddl/suave_domain_created.pddl \
  --aBox --inProblem=pddl/suave_problem.pddl --outProblem=pddl/suave_problem_created.pddl \
  --add-num-comparisons --replace-output

# suave_extended
/home/gus/ros_workspaces/planta_ws/src/owl_to_pddl/OWLToPDDL.sh \
  --owl=owl/suave_extended.owl --tBox \
  --inDomain=pddl/suave_domain_extended.pddl --outDomain=pddl/suave_domain_extended_created.pddl \
  --aBox --inProblem=pddl/suave_problem_extended.pddl --outProblem=pddl/suave_problem_extended_created.pddl \
  --add-num-comparisons --replace-output
```

`OWLToPDDL.sh` is a thin wrapper (`java -jar build/libs/dlToPlanning-1.0-SNAPSHOT-all.jar "$@"`)
-- no ROS environment needs to be sourced to run it. `WARNING: can not parse
TBox axiom DataPropertyRange(...)` lines on stdout are pre-existing noise
from the imported `pddl_tomasys` ontology's datatype property ranges, not a
sign your change failed.

If `src/owl_to_pddl/src/main/kotlin/**` or `build.gradle.kts` changed (not
just the `.owl`/`.pddl` files), the JAR itself is stale -- use the
`rebuild-owl-to-pddl-jar` skill first, or this regenerates against old
translator logic silently.

## Verifying without a full ROS launch

Running `ros2 launch suave_planta suave_planta.launch.py` also regenerates
these files (it runs `owl_to_pddl` before PlanSys2 bringup via
`OnProcessExit`), but spinning up Ignition/PlanSys2 just to check a PDDL
edit is slow. Run Fast Downward standalone against the regenerated files
instead:

```bash
source /home/gus/ros_workspaces/planta_ws/install/setup.bash

ros2 run downward_ros fast-downward.py \
  src/suave_planta/pddl/suave_domain_created.pddl \
  src/suave_planta/pddl/suave_problem_created.pddl \
  --search 'astar(blind())'

# and for the extended domain:
ros2 run downward_ros fast-downward.py \
  src/suave_planta/pddl/suave_domain_extended_created.pddl \
  src/suave_planta/pddl/suave_problem_extended_created.pddl \
  --search 'astar(blind())'
```

Check both that a plan is still found (the edit didn't over-constrain
things into unsolvability) and that the printed action sequence has the
ordering/behavior you intended -- `Plan length`/`Plan cost` alone don't
tell you that.

To confirm a new OWL individual/class/property assertion actually made it
into the generated PDDL, grep for it directly rather than trusting the
regeneration silently worked:

```bash
grep -n "<YourNewPredicateOrClass>" src/suave_planta/pddl/suave_domain_created.pddl
grep -n "<YourNewPredicateOrClass>" src/suave_planta/pddl/suave_problem_created.pddl
```

## Gotchas learned the hard way in this codebase

- **`or` in action preconditions doesn't work here**, even though
  `:disjunctive-preconditions` is declared in `:requirements`. Every
  existing precondition in `suave_domain*.pddl` is built purely from
  `not`/`and`/`exists`/`=` -- follow that convention and rewrite any `(or A
  B)` via De Morgan's law as `(not (and (not A) (not B)))` instead of
  trusting the declared requirement.
- **No SWRL rule is needed for simple structural facts.** `owl_to_pddl`
  auto-generates a bare predicate *and* an `inferred-<CapitalizedName>`
  derived wrapper for every OWL class and object property it encounters in
  an ABox assertion -- e.g. marking three individuals with a brand-new
  `rdf:type` class (no `owl:Class` declaration needed, no property
  declaration needed) is enough to get both `(YourClass ?x)` and
  `(inferred-YourClass ?x)` for free. Reach for a SWRL rule only when you
  need actual conditional inference, not just "these individuals share a
  tag."
- **OWL property characteristics (`owl:SymmetricProperty`,
  `owl:TransitiveProperty`, `owl:InverseOf`, ...) are not honored.** The
  translator (`OntologyTranslator.kt`) has no `OWLReasoner` -- it lifts
  each asserted axiom syntactically, it does not compute semantic closure.
  If you need a relation to hold in both directions, assert both directions
  explicitly in the `.owl` file.
- `reconfigure1`/`reconfigure2`'s `?f`/`?fd_goal`/`?fd_initial` parameters
  are untyped PDDL variables (`Function`/`FunctionDesign` are unary
  predicates, not PDDL `:types`), so named constants (e.g. `f_follow_pipeline`,
  `fd_unground`) can be referenced directly by `=`-equality in a hand-written
  precondition without touching the ontology at all -- useful for a fast,
  throwaway test of a guard before committing to modeling it in OWL.
