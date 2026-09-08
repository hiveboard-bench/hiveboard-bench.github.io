# Isaac Lab integration

The [`EESC-LabRoM/isaaclab-hiveboard`](https://github.com/EESC-LabRoM/isaaclab-hiveboard) repository provides Isaac Lab environments for running HiveBoard tasks with Spot, Franka, and ANYmal platforms.

The instructions below correspond to the repository's `master` branch at commit [`769d034`](https://github.com/EESC-LabRoM/isaaclab-hiveboard/commit/769d034d9b5bfd71e34310a5f6fd2b5c1628c6d9).

## Requirements

The versions are pinned in the repository's [`pyproject.toml`](https://github.com/EESC-LabRoM/isaaclab-hiveboard/blob/master/pyproject.toml):

| Component | Version |
|---|---|
| Python | 3.11 |
| Isaac Sim | 5.1.0 |
| Isaac Lab | 2.3.2.post1 |
| Package manager | `uv` |

The project also installs CuRobo and uses Git submodules for the HiveBoard assets, CuRobo, and the DynaArm description.

## Installation

Clone the repository with all submodules:

```bash
git clone --recurse-submodules https://github.com/EESC-LabRoM/isaaclab-hiveboard.git
cd isaaclab-hiveboard
uv sync
```

For an existing clone created without `--recurse-submodules`, initialize the dependencies before running an environment:

```bash
git submodule update --init --recursive
uv sync
```

Run commands through `uv run` so that the pinned project environment is used.

## Verify the installation

List all registered HiveBoard environments:

```bash
uv run python scripts/list_envs.py
```

The list can be filtered by robot or mechanism:

```bash
uv run python scripts/list_envs.py --keyword Franka
uv run python scripts/list_envs.py --keyword BallValve
```

## Benchmark environments

The following primary task IDs are registered in [`tasks/__init__.py`](https://github.com/EESC-LabRoM/isaaclab-hiveboard/blob/master/source/isaaclab_hiveboard/isaaclab_hiveboard/tasks/__init__.py):

| Task ID | Platform | Mechanism |
|---|---|---|
| `Isaac-HiveBoard-Spot-BallValve-v0` | Spot with arm | Ball valve |
| `Isaac-HiveBoard-Spot-CircuitBreaker-v0` | Spot with arm | Circuit breaker |
| `Isaac-HiveBoard-Spot-HighTorqueValve-v0` | Spot with arm | High-torque gate valve |
| `Isaac-HiveBoard-Spot-SmallValve-v0` | Spot with arm | Small gate valve |
| `Isaac-HiveBoard-Spot-Lamp-v0` | Spot with arm | Threaded lamp |
| `Isaac-HiveBoard-Franka-LeverValve-v0` | Franka | Ball valve |
| `Isaac-HiveBoard-Franka-CircuitBreaker-v0` | Franka | Circuit breaker |
| `Isaac-HiveBoard-Franka-Lamp-v0` | Franka | Threaded lamp |
| `Isaac-HiveBoard-Anymal-BallValve-v0` | ANYmal with DynaArm | Ball valve |

The repository also registers camera, play, robot-only, and gripper-only configurations for debugging. Use `scripts/list_envs.py` for the complete current list. Legacy aliases remain registered for backwards compatibility; new experiments should use the `Isaac-HiveBoard-*` identifiers.

## Run an environment

Run the Spot ball-valve task interactively:

```bash
uv run python scripts/play.py \
  --task "Isaac-HiveBoard-Spot-BallValve-v0" \
  --orbit
```

Run the Franka circuit-breaker task with pose diagnostics:

```bash
uv run python scripts/play.py \
  --task "Isaac-HiveBoard-Franka-CircuitBreaker-v0" \
  --pose-debug
```

For a bounded headless run on the first CUDA device:

```bash
uv run python scripts/play.py \
  --task "Isaac-HiveBoard-Spot-BallValve-v0" \
  --headless \
  --device cuda:0 \
  --max-steps 600
```

Common options supported by `scripts/play.py` include `--num_envs`, `--device`, `--headless`, `--video`, `--fast`, `--pose-debug`, and `--max-steps`.

## Record an example video

Record one headless episode:

```bash
uv run python scripts/play.py \
  --task "Isaac-HiveBoard-Spot-BallValve-v0" \
  --video \
  --headless \
  --enable_cameras \
  --device cuda:0
```

To run the repository's predefined validation set, use:

```bash
./scripts/record_all_examples.sh --device cuda:0
```

Videos are written under `videos/` by default. The batch script records the Spot and Franka examples defined in [`record_all_examples.sh`](https://github.com/EESC-LabRoM/isaaclab-hiveboard/blob/master/scripts/record_all_examples.sh).

## Collect demonstrations

The current collector records successful fixed-base Spot ball-valve demonstrations with relative end-effector control and exports HDF5 datasets:

```bash
uv run python scripts/collect_demos.py \
  --headless \
  --device cuda:0 \
  --num_demos 10 \
  --dataset_name spot_ball_valve_10
```

Datasets are stored in `logs/recorded_datasets/` unless `--dataset_dir` is supplied. The collector also supports `--num_envs`, `--video`, `--keep_failed_demos`, and an optional reachable reset-state cache through `--reset_state_cache_path`.

## Repository structure

```text
isaaclab-hiveboard/
├── dependencies/
│   ├── HiveBoard/          # Benchmark meshes and articulated assets
│   ├── curobo/             # CuRobo submodule
│   └── duatic_dynaarm/     # DynaArm description
├── scripts/
│   ├── list_envs.py
│   ├── play.py
│   ├── collect_demos.py
│   └── record_all_examples.sh
└── source/isaaclab_hiveboard/
    └── isaaclab_hiveboard/
        ├── assets/
        ├── mdp/
        ├── tasks/
        └── utils/
```

## Reporting simulation results

Simulation results should identify:

- the full `isaaclab-hiveboard` commit;
- the HiveBoard submodule commit;
- Isaac Sim and Isaac Lab versions;
- the task ID, robot, and end-effector;
- the number of parallel environments and evaluation seeds;
- any changes to task configuration, observations, actions, reset distributions, or success conditions; and
- whether results were produced with a scripted controller, collected demonstrations, or a learned policy.

Do not combine simulation and physical-board results into one aggregate score. Contact, friction, printed-part variation, press-fit release, and component damage are not necessarily represented by the simulation.
