# Simulation workflows

These workflows use [the Newton-based Isaac Lab installation](/simulation/isaac-lab), checked at simulation commit [`1d7a042`](https://github.com/hiveboard-bench/isaaclab-hiveboard/commit/1d7a0421154af19883eadc0e5d3b12eda2cb2a00). Complete the dependency installation and USD generation first. Run commands from the simulation repository root.

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

The default selection contains `-Play` environments, including the light-bulb tasks for all three robots. Add `--all` to include other variants. To record only the ANYmal light-bulb task:

```bash
uv run python scripts/record_all_envs.py \
  --task Isaac-HiveBoard-Anymal-Lamp-Play-v0 --renderer newton
```

Filter by robot and inspect the commands before recording:

```bash
uv run python scripts/record_all_envs.py --match Franka --dry-run
uv run python scripts/record_all_envs.py --match Franka --renderer newton
```

The batch recorder defaults to RTX rendering. Select `--renderer newton` for rasterized output. Both `ffmpeg` and `ffprobe` must be on `PATH`. Frame rates are derived separately for each task to preserve simulated timing.

Inspect the recordings and task outcomes before treating them as successful demonstrations. For light bulb, check the [configured screw travel and success condition](/simulation/isaac-lab#light-bulb-success-condition).

## Record episodes for learning

### Player recordings

When a task has a recorder configured, `play.py` writes HDF5 episodes and prints the output path. The standard recorder defaults to `logs/recorded_datasets/`. The player exports successful and unsuccessful episodes. Use `--no-dataset` when only visualization or video is needed.

Inspect outcome labels before selecting episodes for learning. Check the HDF5 schema against the intended training code.

## Policy training

Reinforcement and imitation learning now live in [`EESC-LabRoM/hiveboard-rl`](https://github.com/EESC-LabRoM/hiveboard-rl), checked at commit [`34a11c8`](https://github.com/EESC-LabRoM/hiveboard-rl/commit/34a11c865923aedc3727eb019a8912401c241625). The core `isaaclab-hiveboard` repository provides environments, scripted controllers, command editing, and recording. Its former `scripts/rl/`, `scripts/imitation/`, and `imitation` dependency extra have been removed.

The learning repository installs its own pinned `isaaclab-hiveboard` submodule. With `just` and `uv` installed, start in a separate checkout:

```bash
git clone https://github.com/EESC-LabRoM/hiveboard-rl.git
cd hiveboard-rl
just setup
```

Generate USD assets inside `dependencies/isaaclab-hiveboard` using that checkout's generation recipes, or use `just sync-assets /path/to/isaaclab-hiveboard` to copy generated assets from a checkout matching the pinned core version. Then run `just list-envs` to inspect the combined core and learning registry. Keep the learning repository commit and its core submodule commit with experiment records.

### Reinforcement learning

The learning package registers ANYmal `-RL-v0` and `-RL-Play-v0` tasks for `BallValve`, `SmallValve`, `M30Thread`, and `CircuitBreaker`.

The maintainers identify **cuRobo trajectory bank → PPO student → evaluation** as the current working workflow. The bank supplies reset states, tracking rewards, and deviation terminations. The actor uses deployable observations; the critic also receives privileged simulator state.

Follow the [learning README](https://github.com/EESC-LabRoM/hiveboard-rl/blob/34a11c865923aedc3727eb019a8912401c241625/README.md#usage) and [justfile](https://github.com/EESC-LabRoM/hiveboard-rl/blob/34a11c865923aedc3727eb019a8912401c241625/justfile) for `rl-bank`, `rl-student-ppo`, `rl-eval`, and `rl-play`. Select the same `RL_TOOL` for bank generation, training, and evaluation. Set parallel environment counts to fit the available GPU memory.

Inspect each task's success condition and reset distribution before training. Report the configured goal and validate the complete [benchmark motion](/benchmark/tasks) before submitting an evaluation.

### Imitation learning

Behavior cloning, DAgger, teacher training, and student distillation remain alternative workflows in the learning repository. Its `il-collect`, `il-train`, `il-dagger`, and `il-eval` recipes use the imitation-learning scripts and their dataset layout. Follow the [learning repository instructions](https://github.com/EESC-LabRoM/hiveboard-rl/blob/34a11c865923aedc3727eb019a8912401c241625/README.md#usage) for these workflows; a core player recording is not automatically a compatible training dataset.

## Simulation datasets and DataHive

Use [DataHive](/guides/datahive) for its recording integrations, episode annotations, validation, and dataset submission workflow. The simulation player exports its own HDF5 datasets. Using the same benchmark does not establish file-format compatibility. Check observation names, action units, coordinate frames, timestamps, and metadata before converting data between pipelines.

A [learning-dataset contribution](/contribute/evaluations) has no fixed episode count. Include the simulator commit, task configuration, controller or policy, reset distribution, and outcome labels with simulated data. Keep this separate from a scored benchmark submission, which requires **13 conditions with five trials each**.
