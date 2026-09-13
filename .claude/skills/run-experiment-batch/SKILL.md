---
name: run-experiment-batch
description: Use when the user wants to run all (or several) suave_planta experiment campaigns in one go, check whether a previous batch finished or needs resuming, or resume an interrupted/crashed batch. Covers the generic run_batch node and its state.json checkpoint -- do not hand-roll a loop over the individual exp*_suave_runner.launch.py files instead.
---

# Run a suave_planta Experiment Batch

`suave_planta` has two ways to run a full campaign sweep. This skill covers
the generic, checkpointed one (`run_batch`); use it instead of manually
looping over the six individual `exp*_suave_runner.launch.py` /
`extended_exp*_suave_runner.launch.py` launch files.

(The older `run_all_experiments.py` script at the package root still exists
and does the same six campaigns with its own state file -- it's kept for
whoever still uses it directly, but `run_batch` is the maintained,
package-generic mechanism and the one this skill drives.)

## Starting a batch

```bash
ros2 launch suave_planta run_batch.launch.py
```

This runs `exp1`-`exp3` and `extended_exp1`-`extended_exp3` sequentially,
per `suave_planta/config/runner/batch_campaigns.yml`, into a new timestamped
directory under `~/suave/results/batches/` (e.g. `batch_20260913_104200/`).

**This takes hours to a few days** depending on `num_runs` per experiment --
launch it in `screen`/`tmux` and don't wait on it in the foreground.

To run a different or narrower set of campaigns, don't edit
`batch_campaigns.yml` in place unless that's actually the intent -- pass a
different manifest instead:

```bash
ros2 run suave_runner run_batch \
  --ros-args --params-file /path/to/custom_batch_campaigns.yml
```

## Checking whether a batch is done or needs resuming

Every batch directory contains:

- `state.json` -- per-campaign status: `pending`, `running`, `completed`,
  `incomplete`, `failed`, or `interrupted`
- `run_batch.log` -- the overall orchestration log
- `<campaign_name>.log` -- e.g. `exp1.log`, per-campaign combined output
- `campaigns/<campaign_name>/` -- that campaign's own results directory

Fastest check -- tail the overall log:

```bash
tail -n 5 ~/suave/results/batches/<batch_dir>/run_batch.log
```

- Ends with `All campaigns completed successfully.` -> done, nothing to do.
- Ends with `Batch interrupted; ...` or `Batch finished with N incomplete,
  ...` -> not done; that line also prints the exact resume command.

Or check per-campaign status directly:

```bash
grep '"status"' ~/suave/results/batches/<batch_dir>/state.json
```

Any campaign not `"completed"` still needs to (re)run.

## Resuming

```bash
ros2 run suave_runner run_batch \
  --ros-args -p resume_state_file:=~/suave/results/batches/<batch_dir>/state.json
```

Campaigns already `completed` are skipped; an `incomplete` campaign resumes
from its last saved run rather than restarting from scratch (it reuses
`suave_runner`'s own `resume_result_path` mechanism under the hood -- see
`suave_runner/README.md`'s "Resuming a crashed campaign" section for what
that guarantees and doesn't).

Do not confuse this with a single campaign's `resume_result_path` parameter
-- `resume_state_file` operates one level up, at the whole-batch level.

## Other run_batch parameters

- `-p fail_fast:=true` -- stop the batch as soon as one campaign fails,
  instead of continuing to the next.
- `-p dry_run:=true` -- print the `ros2 run suave_runner suave_runner ...`
  command for each campaign without executing anything; useful to sanity
  check a new or edited manifest before committing hours to it.

Full details: `suave_planta/README.md`'s "Batch runner" section and
`suave/suave_runner/README.md`'s "Batch runner" section (the underlying
node's own docs).
