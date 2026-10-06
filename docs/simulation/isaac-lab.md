# Isaac Lab integration

The [`EESC-LabRoM/isaaclab-hiveboard`](https://github.com/EESC-LabRoM/isaaclab-hiveboard) repository provides HiveBoard environments for Spot with arm, Franka FR3, and ANYmal with DynaArm. The current implementation uses Isaac Lab with **Newton MJWarp**. The standard simulation and recording workflows run without Isaac Sim.

This guide follows `master` at commit [`e009735`](https://github.com/EESC-LabRoM/isaaclab-hiveboard/commit/e00973501f1fa466b6aceba8a6c31dca2875065d), checked on 6 October 2026. For command editing, datasets, and policy training, see [Simulation workflows](/simulation/workflows).

## Requirements

Use the dependencies specified by the repository's [`pyproject.toml`](https://github.com/EESC-LabRoM/isaaclab-hiveboard/blob/e00973501f1fa466b6aceba8a6c31dca2875065d/pyproject.toml) and Git submodules.

| Component | Current configuration |
|---|---|
| Python | 3.12 |
| Isaac Lab | Source checkout in `dependencies/IsaacLab`, pinned to `78b12aed` |
| Physics | Newton 1.6.0 with the `newton_mjwarp` preset |
| Package manager | `uv` |
| Motion planning | cuRobo from `dependencies/curobo`, included in the project dependencies |
| Video encoding | `ffmpeg` and `ffprobe` on `PATH` |
| Task editor | Web browser for the local Viser interface |

The GPU workflows use CUDA dependencies. RTX video rendering requires compatible NVIDIA hardware. The player also accepts `--device cpu` for CPU runs, subject to the selected task and planner.

The earlier Python 3.11, Isaac Sim 5.1.0, and Isaac Lab 2.3.2 instructions describe [commit `769d034`](https://github.com/EESC-LabRoM/isaaclab-hiveboard/tree/769d034d9b5bfd71e34310a5f6fd2b5c1628c6d9). Use that checkout to reproduce experiments from the earlier implementation.

## Installation

Clone the repository and its pinned submodules:

```bash
git clone --branch master --recurse-submodules https://github.com/EESC-LabRoM/isaaclab-hiveboard.git
cd isaaclab-hiveboard
uv sync --python 3.12
```

After updating an existing checkout, synchronize its dependencies:

```bash
git submodule update --init --recursive
uv sync --python 3.12
```

Run commands from the repository root through `uv run`. Keep the Isaac Lab submodule at the commit recorded by the HiveBoard checkout.

### Generate the Newton assets

The generated Newton USD files are build outputs and must be created on a fresh clone. Initializing the asset submodules alone does not generate them.

First, check which outputs are missing:

```bash
uv run python scripts/generate_newton_usd.py --verify-only
```

A failed check is expected before generation. Install [urdf-usd-converter](https://github.com/newton-physics/urdf-usd-converter) in a separate Python environment, then pass that environment's Python executable to the generator:

```bash
uv venv --python 3.12 .venv-uuc
uv pip install --python .venv-uuc/bin/python urdf-usd-converter
uv run python scripts/generate_newton_usd.py \
  --uuc-python .venv-uuc/bin/python
uv run python scripts/generate_newton_usd.py --verify-only
```

Record the converter version with the experiment. The generator also accepts `--assets` to rebuild one mechanism, for example `--assets lamp`.

For ANYmal, generate and verify the robot assembly as well:

```bash
uv run python scripts/generate_anymal_newton_usd.py
uv run python scripts/generate_anymal_newton_usd.py --verify-only
```

The ANYmal generator downloads robot and gripper assets when they are absent, so its first run requires internet access.

## Available tasks

Task IDs have the form `Isaac-HiveBoard-<Robot>-<Tool>-v0`. Robot tokens are case-sensitive: `Spot`, `Franka`, and `Anymal`.

The table maps the benchmark terminology to the identifiers in [the task registry](https://github.com/EESC-LabRoM/isaaclab-hiveboard/blob/e00973501f1fa466b6aceba8a6c31dca2875065d/source/isaaclab_hiveboard/isaaclab_hiveboard/tasks/__init__.py). A check mark means that an environment is registered for that robot.

| Benchmark task | Tool token | Spot | Franka FR3 | ANYmal |
|---|---|:---:|:---:|:---:|
| Ball valve | `BallValve` | ✓ | ✓ | ✓ |
| Gate valve (small) | `SmallValve` | ✓ | ✓ | ✓ |
| Gate valve (large) | `HighTorqueValve` | ✓ | ✓ | ✓ |
| Circuit breaker | `CircuitBreaker` | ✓ | ✓ | ✓ |
| Button | `Button` | ✓ | ✓ | ✓ |
| Lock and key | `Key` | ✓ | ✓ | ✓ |
| Drawer | `Drawer` | ✓ | ✓ | ✓ |
| Thread (M8) | `M8Thread` | ✓ | ✓ | ✓ |
| Thread (M30) | `M30Thread` | ✓ | ✓ | ✓ |
| Peg insertion | `PegInsertion` | ✓ | ✓ | ✓ |
| Shock absorber | `ShockAbsorber` | ✓ | ✓ | ✓ |
| Light bulb | `Lamp` | ✓ | ✓ | ✓ |

Each listed task except `Lamp` also has a `-Play-v0` variant. For example, the Franka ball-valve play environment is `Isaac-HiveBoard-Franka-BallValve-Play-v0`. The former `Franka-LeverValve` identifier is no longer registered.

List the registered environments without launching a simulation:

```bash
uv run python scripts/record_all_envs.py --all --list
uv run python scripts/record_all_envs.py --all --list --match Franka
```

The registry also includes `CuroboValve` planning examples, `BenchValve` joint-trajectory playback, diagnostic tasks, and [RL environments](/simulation/workflows#reinforcement-learning). Registration alone does not establish task success or compliance with every benchmark condition. In particular, the two physical ball-valve conditions require an explicit simulation configuration and validation of their resistance.

## Run an environment

Run the Spot ball-valve play environment:

```bash
uv run python scripts/play.py \
  --task Isaac-HiveBoard-Spot-BallValve-Play-v0 \
  physics=newton_mjwarp --visualizer newton
```

For the Franka shock-absorber task with TCP tracking diagnostics:

```bash
uv run python scripts/play.py \
  --task Isaac-HiveBoard-Franka-ShockAbsorber-Play-v0 \
  physics=newton_mjwarp --visualizer newton --pose-debug
```

Run a bounded simulation without a viewer:

```bash
uv run python scripts/play.py \
  --task Isaac-HiveBoard-Spot-BallValve-Play-v0 \
  physics=newton_mjwarp --visualizer none \
  --device cuda:0 --max-steps 600 --no-dataset
```

| Option | Purpose |
|---|---|
| `--num_envs N` | Number of parallel environments |
| `--seed N` | Environment reset seed |
| `--max-steps N` | Stop after a fixed number of environment steps |
| `--duration S` | Limit the run to simulated seconds, while allowing success or failure to end it earlier |
| `--pose-debug` | Print TCP tracking errors |
| `--contact-debug` | Print contact forces for tasks with the required gripper sensors |
| `--setup FILE` | Load a saved command sequence and TCP settings |
| `--no-setup` | Use the command sequence defined in the task code |
| `--no-dataset` | Disable HDF5 episode recording |

The `--duration` option changes the handling of the task's episode time limit. Use the task configuration and [benchmark protocol](/benchmark/protocol) to set evaluation timeouts.

If `just` is installed, the repository provides shortcuts such as `just play Franka ShockAbsorber`. Use `play.py` directly for `Lamp`, which has no `-Play-v0` variant.

## Record an example video

```bash
uv run python scripts/play.py \
  --task Isaac-HiveBoard-Spot-BallValve-Play-v0 \
  physics=newton_mjwarp --visualizer none \
  --video --video-renderer newton --max-steps 600 \
  --video-folder videos/spot-ball-valve --no-dataset
```

Use `--video-renderer rtx` for path-traced output. These Newton recording commands do not require the previous Isaac Sim `--enable_cameras` setup. The default playback frame rate is derived from the simulation timestep and control decimation.

For batch recording, saved command setups, and dataset collection, continue to [Simulation workflows](/simulation/workflows).

## Reporting simulation results

Record:

- the full simulation repository commit and `git submodule status --recursive`
- the Python environment, physics backend, renderer, and asset-converter version
- the task ID, robot, end-effector, and saved command setup
- physics timestep, control decimation, solver settings, and any contact or joint overrides
- observations, actions, reset distributions, seeds, success checks, and time limits
- the number of trials and whether control was scripted or learned.

Check the configured motion against [How to perform each task](/benchmark/tasks). Some training environments use simplified goals, including the small gate-valve RL task described in [Simulation workflows](/simulation/workflows#reinforcement-learning). A benchmark evaluation still requires all **13 conditions with five trials each**.

Report simulated and physical results separately. Document which effects were modeled, including friction, contact, attachment release, and component damage.
