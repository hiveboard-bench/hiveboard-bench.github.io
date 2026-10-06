# Simulation workflows

These workflows use [the Newton-based Isaac Lab installation](/simulation/isaac-lab), checked at simulation commit [`e009735`](https://github.com/EESC-LabRoM/isaaclab-hiveboard/commit/e00973501f1fa466b6aceba8a6c31dca2875065d). Complete the dependency installation and USD generation first. Run commands from the simulation repository root.

## Edit a task's command sequence

The Viser editor lets you adjust target poses, rotation axes, screw travel, gripper commands, and the tool center point (TCP).

```bash
uv run python scripts/command_edit.py \
  --task Isaac-HiveBoard-Franka-BallValve-Play-v0 \
  --port 9080
```

Open `http://localhost:9080` in a browser on the same computer.

1. Select a command and adjust its position, orientation, or reference frame.
2. Set motion speeds, tolerances, gripper state, and hold time as needed.
3. Check the TCP offset relative to the end-effector body.
4. Preview the sequence and inspect the inverse-kinematics residuals.
5. Click **Save setup**.
6. Run the saved sequence with physics before collecting data.

The editor preview is kinematic. Contact forces, object motion, collisions, and cuRobo plans must be checked in the simulation run.

### Saved setups

By default, the editor and player look for `configs/<task>.json`. A `-Play-v0` task falls back to its base task's file when it has no file of its own. An explicit `--setup FILE` selects a different file.

The editor saves to the loaded file, or to `configs/<task>.json` for a new setup. Use `--out FILE` to choose another output path. Retain the saved JSON with the experiment.

For example, save a Spot setup to a known path:

```bash
uv run python scripts/command_edit.py \
  --task Isaac-HiveBoard-Spot-BallValve-Play-v0 \
  --out configs/spot-ball-valve-review.json --port 9080
```

After clicking **Save setup**, run:

```bash
uv run python scripts/play.py \
  --task Isaac-HiveBoard-Spot-BallValve-Play-v0 \
  --setup configs/spot-ball-valve-review.json \
  physics=newton_mjwarp --visualizer newton
```

Use `--no-setup` with `play.py` to inspect the command sequence defined in the task code.

## Record several environments

The batch recorder runs each selected environment in a separate process and writes MP4 files, logs, and a `summary.json` under `videos/environments/<timestamp>/`.

Preview the default selection:

```bash
uv run python scripts/record_all_envs.py --list
```

The default selection contains `-Play` environments. Add `--all` to include other variants. Since light-bulb environments have no `-Play` variant, select one explicitly or use `--all`.

```bash
uv run python scripts/record_all_envs.py \
  --task Isaac-HiveBoard-Anymal-Lamp-v0 --renderer newton
```

Filter by robot and inspect the commands before recording:

```bash
uv run python scripts/record_all_envs.py --match Franka --dry-run
uv run python scripts/record_all_envs.py --match Franka --renderer newton
```

The batch recorder defaults to RTX rendering. Select `--renderer newton` for rasterized output. Both `ffmpeg` and `ffprobe` must be on `PATH`. Frame rates are derived separately for each task to preserve simulated timing.

Inspect the recordings and task outcomes before treating them as successful demonstrations.

## Record episodes for learning

### Player recordings

When a task has a recorder configured, `play.py` writes HDF5 episodes and prints the output path. The standard recorder defaults to `logs/recorded_datasets/`. The player exports successful and unsuccessful episodes. Use `--no-dataset` when only visualization or video is needed.

The imitation-learning collector below uses its own recorder and HDF5 layout. Select the recorder according to the downstream training code.

### Imitation learning

The repository includes demonstration collection, behavior cloning, DAgger, and policy evaluation in [`scripts/imitation/`](https://github.com/EESC-LabRoM/isaaclab-hiveboard/tree/e00973501f1fa466b6aceba8a6c31dca2875065d/scripts/imitation).

Install the additional dependencies:

```bash
uv sync --python 3.12 --extra imitation
```

Use `uv run --extra imitation` for the following commands to keep the extra dependencies installed.

The collector requires a `bc` observation group. Training also requires a registered `robomimic_bc_cfg_entry_point`. At the documented commit, these training configurations are registered for Spot light bulb, Spot ball valve, and ANYmal ball valve. Other registered simulation tasks need the corresponding learning configuration before using this pipeline.

Collect 50 successful Spot ball-valve demonstrations:

```bash
uv run --extra imitation python scripts/imitation/collect_demos.py \
  --task Isaac-HiveBoard-Spot-BallValve-v0 \
  --num_demos 50 --num_envs 1 \
  --dataset_name spot_ball_valve_50
```

The output is `logs/imitation/datasets/spot_ball_valve_50.hdf5`. The script refuses to overwrite an existing dataset. It stores observations at `data/demo_<i>/obs/<key>` and actions at `data/demo_<i>/actions`. Add `--keep_failed` to write rejected episodes to a separate file.

The collector loads saved command setups using the same task lookup as the player. Pass `--setup FILE` to use a specific setup. Its option to skip saved setups is spelled `--no_setup`.

Train a behavior-cloning policy:

```bash
uv run --extra imitation python scripts/imitation/train_bc.py \
  --task Isaac-HiveBoard-Spot-BallValve-v0 \
  --dataset logs/imitation/datasets/spot_ball_valve_50.hdf5
```

Training artifacts are written under `logs/imitation/runs/`. Evaluate a checkpoint by replacing the path below with the generated checkpoint:

```bash
uv run --extra imitation python scripts/imitation/eval_policy.py \
  --task Isaac-HiveBoard-Spot-BallValve-v0 \
  --checkpoint /path/to/checkpoint.pth --episodes 25
```

Use `--expert` in place of `--checkpoint` to evaluate the scripted controller. Check the actual observation fields before transferring a learned policy to a robot. Object states available from the simulator may require sensing or estimation on the physical setup.

For DAgger options, see `scripts/imitation/dagger.py --help` through the same `uv run --extra imitation python` command.

### Reinforcement learning

ANYmal RL environments are registered for these tasks:

| Benchmark task | Training task ID |
|---|---|
| Ball valve | `Isaac-HiveBoard-Anymal-BallValve-RL-v0` |
| Gate valve (small) | `Isaac-HiveBoard-Anymal-SmallValve-RL-v0` |
| Thread (M30) | `Isaac-HiveBoard-Anymal-M30Thread-RL-v0` |
| Circuit breaker | `Isaac-HiveBoard-Anymal-CircuitBreaker-RL-v0` |

Each has an `-RL-Play-v0` variant. The repository provides expert-bank generation, PPO teacher training, student training or distillation, and checkpoint evaluation in [`scripts/rl/`](https://github.com/EESC-LabRoM/isaaclab-hiveboard/tree/e00973501f1fa466b6aceba8a6c31dca2875065d/scripts/rl).

Follow the [`rl-*` recipes in the justfile](https://github.com/EESC-LabRoM/isaaclab-hiveboard/blob/e00973501f1fa466b6aceba8a6c31dca2875065d/justfile) for the full sequence. `RL_TOOL` selects `BallValve`, `SmallValve`, `M30Thread`, or `CircuitBreaker`. Inspect the selected task's expert-bank path, reset distribution, observation groups, and success condition before starting a run. The default training recipes use many parallel environments, so choose counts that fit the available GPU memory.

**The RL success conditions can differ from the benchmark protocol.** At the documented commit, the [small gate-valve RL environment](https://github.com/EESC-LabRoM/isaaclab-hiveboard/blob/e00973501f1fa466b6aceba8a6c31dca2875065d/source/isaaclab_hiveboard/isaaclab_hiveboard/tasks/anymal/small_valve_rl/env.py) targets a quarter turn. The [benchmark task](/benchmark/tasks#small-gate-valve) requires one full stem turn. Report the configured goal with learning results and validate the complete benchmark motion before submitting a benchmark evaluation.

## Simulation datasets and DataHive

Use [DataHive](/guides/datahive) for its recording integrations, episode annotations, validation, and dataset submission workflow. The simulation scripts above export their own HDF5 datasets. Using the same benchmark does not establish file-format compatibility. Check observation names, action units, coordinate frames, timestamps, and metadata before converting data between pipelines.

A [learning-dataset contribution](/contribute/evaluations) has no fixed episode count. Include the simulator commit, task configuration, controller or policy, reset distribution, and outcome labels with simulated data. Keep this separate from a scored benchmark submission, which requires **13 conditions with five trials each**.
