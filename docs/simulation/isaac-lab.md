# Isaac Lab integration

The [`hiveboard-bench/isaaclab-hiveboard`](https://github.com/hiveboard-bench/isaaclab-hiveboard) repository provides HiveBoard environments for Spot with arm, Franka FR3, and ANYmal with DynaArm. The current implementation uses Isaac Lab with **Newton MJWarp**. The standard simulation and recording workflows run without Isaac Sim.

This guide follows `master` at commit [`1d7a042`](https://github.com/hiveboard-bench/isaaclab-hiveboard/commit/1d7a0421154af19883eadc0e5d3b12eda2cb2a00), checked on 8 October 2026. For command editing, episode recording, and the separate learning repository, see [Simulation workflows](/simulation/workflows).

## Simulation videos

Four examples from the [Isaac Lab video gallery](https://hiveboard-bench.github.io/#Videos), featuring two tasks each with ANYmal D and Franka FR3:

<div class="simulation-video-grid">
  <figure>
    <video controls playsinline preload="none" poster="/images/anymal_d_isaac_ball_valve.webp" aria-label="ANYmal D operating a ball valve">
      <source src="https://github.com/hiveboard-bench/hiveboard-bench.github.io/releases/download/v1.0-v1.0-videos/anymal_d_isaac_ball_valve.mp4" type="video/mp4">
    </video>
    <figcaption>ANYmal D · Ball valve</figcaption>
  </figure>
  <figure>
    <video controls playsinline preload="none" poster="/images/anymal_d_isaac_button.webp" aria-label="ANYmal D pressing a button">
      <source src="https://github.com/hiveboard-bench/hiveboard-bench.github.io/releases/download/v1.0-v1.0-videos/anymal_d_isaac_button.mp4" type="video/mp4">
    </video>
    <figcaption>ANYmal D · Button</figcaption>
  </figure>
  <figure>
    <video controls playsinline preload="none" poster="/images/fr3_isaac_circuit.webp" aria-label="Franka FR3 operating a circuit breaker">
      <source src="https://github.com/hiveboard-bench/hiveboard-bench.github.io/releases/download/v1.0-v1.0-videos/fr3_isaac_circuit.mp4" type="video/mp4">
    </video>
    <figcaption>Franka FR3 · Circuit breaker</figcaption>
  </figure>
  <figure>
    <video controls playsinline preload="none" poster="/images/fr3_isaac_big_gate_valve.webp" aria-label="Franka FR3 operating a large gate valve">
      <source src="https://github.com/hiveboard-bench/hiveboard-bench.github.io/releases/download/v1.0-v1.0-videos/fr3_isaac_big_gate_valve.mp4" type="video/mp4">
    </video>
    <figcaption>Franka FR3 · Gate valve (large)</figcaption>
  </figure>
</div>

## Simulation videos

Four examples from the [Isaac Lab video gallery](https://hiveboard-bench.github.io/#Videos), featuring two tasks each with ANYmal D and Franka FR3:

<div class="simulation-video-grid">
  <figure>
    <video controls playsinline preload="none" poster="/images/anymal_d_isaac_ball_valve.webp" aria-label="ANYmal D operating a ball valve">
      <source src="https://github.com/hiveboard-bench/hiveboard-bench.github.io/releases/download/v1.0-v1.0-videos/anymal_d_isaac_ball_valve.mp4" type="video/mp4">
    </video>
    <figcaption>ANYmal D · Ball valve</figcaption>
  </figure>
  <figure>
    <video controls playsinline preload="none" poster="/images/anymal_d_isaac_button.webp" aria-label="ANYmal D pressing a button">
      <source src="https://github.com/hiveboard-bench/hiveboard-bench.github.io/releases/download/v1.0-v1.0-videos/anymal_d_isaac_button.mp4" type="video/mp4">
    </video>
    <figcaption>ANYmal D · Button</figcaption>
  </figure>
  <figure>
    <video controls playsinline preload="none" poster="/images/fr3_isaac_circuit.webp" aria-label="Franka FR3 operating a circuit breaker">
      <source src="https://github.com/hiveboard-bench/hiveboard-bench.github.io/releases/download/v1.0-v1.0-videos/fr3_isaac_circuit.mp4" type="video/mp4">
    </video>
    <figcaption>Franka FR3 · Circuit breaker</figcaption>
  </figure>
  <figure>
    <video controls playsinline preload="none" poster="/images/fr3_isaac_big_gate_valve.webp" aria-label="Franka FR3 operating a large gate valve">
      <source src="https://github.com/hiveboard-bench/hiveboard-bench.github.io/releases/download/v1.0-v1.0-videos/fr3_isaac_big_gate_valve.mp4" type="video/mp4">
    </video>
    <figcaption>Franka FR3 · Gate valve (large)</figcaption>
  </figure>
</div>

## Requirements

Use the dependencies specified by the repository's [`pyproject.toml`](https://github.com/hiveboard-bench/isaaclab-hiveboard/blob/1d7a0421154af19883eadc0e5d3b12eda2cb2a00/pyproject.toml) and Git submodules.

| Component | Current configuration |
|---|---|
| Python | 3.12 |
| Isaac Lab | Source checkout in `dependencies/IsaacLab`, pinned to `78b12aed` |
| Physics | Newton 1.6.1 with the `newton_mjwarp` preset |
| Package manager | `uv` |
| Motion planning | cuRobo from `dependencies/curobo`, included in the project dependencies |
| Video encoding | `ffmpeg` and `ffprobe` on `PATH` |
| Task editor | Web browser for the local Viser interface |

The GPU workflows use CUDA dependencies. RTX video rendering requires compatible NVIDIA hardware. The player also accepts `--device cpu` for CPU runs, subject to the selected task and planner.

## Installation

Clone the repository and its pinned submodules:

```bash
git clone --branch master --recurse-submodules https://github.com/hiveboard-bench/isaaclab-hiveboard.git
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

After updating an existing checkout, regenerate the key and drawer assets. The key now uses separate key and lock USDs; the drawer uses a fixed housing and a free box guided by contact. Regeneration also applies the revised collision geometry and contact materials:

```bash
uv run python scripts/generate_newton_usd.py \
  --assets key --uuc-python .venv-uuc/bin/python
uv run python scripts/generate_newton_usd.py \
  --assets drawer --uuc-python .venv-uuc/bin/python
uv run python scripts/generate_newton_usd.py --verify-only
```

For ANYmal, generate and verify the robot assembly as well:

```bash
uv run python scripts/generate_anymal_newton_usd.py
uv run python scripts/generate_anymal_newton_usd.py --verify-only
```

The ANYmal generator downloads robot and gripper assets when they are absent, so its first run requires internet access.

## Heuristic task status {#available-tasks}

Task IDs have the form `Isaac-HiveBoard-<Robot>-<Tool>-v0`. Robot tokens are case-sensitive: `Spot`, `Franka`, and `Anymal`.

The table reports the heuristic status given in the [simulation README at commit `1d7a042`](https://github.com/hiveboard-bench/isaaclab-hiveboard/blob/1d7a0421154af19883eadc0e5d3b12eda2cb2a00/README.md#available-tasks).

**✓**: working heuristic reported by the maintainers. **✗**: no working heuristic currently reported. All listed robot–task combinations have registered environments. The symbols describe the supplied heuristic controllers.

| Benchmark task | Tool token | Spot | Franka FR3 | ANYmal |
|---|---|:---:|:---:|:---:|
| Ball valve | `BallValve` | ✓ | ✓ | ✓ |
| Gate valve (small) | `SmallValve` | ✓ | ✓ | ✓ |
| Gate valve (large) | `HighTorqueValve` | ✓ | ✓ | ✓ |
| Circuit breaker | `CircuitBreaker` | ✓ | ✓ | ✓ |
| Button | `Button` | ✓ | ✓ | ✓ |
| Lock and key | `Key` | ✓ | ✓ | ✓ |
| Drawer | `Drawer` | ✗ | ✓ | ✓ |
| Thread (M8) | `M8Thread` | ✗ | ✓ | ✓ |
| Thread (M30) | `M30Thread` | ✗ | ✓ | ✓ |
| Peg insertion | `PegInsertion` | ✗ | ✗ | ✗ |
| Shock absorber | `ShockAbsorber` | ✗ | ✗ | ✗ |
| Light bulb | `Lamp` | ✓ | ✓ | ✓ |

These statuses apply to the supplied task configurations. They do not quantify success rates across randomized resets. For benchmark evaluations, check the configured motion and success condition against [How to perform each task](/benchmark/tasks).

Each listed task has a `-Play-v0` variant, including `Lamp` for Spot, Franka, and ANYmal. For example, the Franka ball-valve play environment is `Isaac-HiveBoard-Franka-BallValve-Play-v0`. The former `Franka-LeverValve` identifier is no longer registered.

List the registered environments without launching a simulation:

```bash
uv run python scripts/record_all_envs.py --all --list
uv run python scripts/record_all_envs.py --all --list --match Franka
```

The registry also includes `CuroboValve` planning examples, `BenchValve` joint-trajectory playback, and diagnostic tasks. [RL environments](/simulation/workflows#reinforcement-learning) are registered by the separate `hiveboard-rl` package. Registration alone does not establish task success or compliance with every benchmark condition. In particular, the two physical ball-valve conditions require an explicit simulation configuration and validation of their resistance.

### Lock-and-key setup

The [key scene](https://github.com/hiveboard-bench/isaaclab-hiveboard/blob/1d7a0421154af19883eadc0e5d3b12eda2cb2a00/source/isaaclab_hiveboard/isaaclab_hiveboard/tasks/scenes/key.py) starts with the key rigidly attached to the robot's hand. The supplied sequence approaches the lock, inserts the key, and turns the plug. It does not evaluate picking up a loose key or retaining it through a frictional grasp.

The [success check](https://github.com/hiveboard-bench/isaaclab-hiveboard/blob/1d7a0421154af19883eadc0e5d3b12eda2cb2a00/source/isaaclab_hiveboard/isaaclab_hiveboard/tasks/anymal/key/configs/terminations.py) requires the command sequence to finish and the plug angle to lie between 80° and 95°. Record the pre-held key condition with results and compare it with the [benchmark task](/benchmark/tasks).

### Drawer removal

The Franka and ANYmal tasks use a free drawer box inside a kinematic housing. The box can slide out and detach through contact dynamics. Their [removal success check](https://github.com/hiveboard-bench/isaaclab-hiveboard/blob/1d7a0421154af19883eadc0e5d3b12eda2cb2a00/source/isaaclab_hiveboard/isaaclab_hiveboard/tasks/anymal/drawer/slide.py) requires the command sequence to finish, at least 5 cm of outward displacement, and both drawer shafts to clear the housing guides. A removed drawer may fall after release without losing success solely because its height changes.

Spot still has no working drawer heuristic reported. Its configured sliding success check differs from the removal check used by Franka and ANYmal.

### Light-bulb success condition

The [light-bulb success check](https://github.com/hiveboard-bench/isaaclab-hiveboard/blob/1d7a0421154af19883eadc0e5d3b12eda2cb2a00/source/isaaclab_hiveboard/isaaclab_hiveboard/mdp/terminations.py) requires the command sequence to finish and the bulb to reach the axial position specified by its total screw travel, with a 0.5 mm tolerance. The target is limited by the seated position.

A short command sequence can therefore succeed with the bulb only partly threaded. For the [benchmark light-bulb task](/benchmark/tasks#light-bulb-and-socket), configure the full travel and check that the bulb is seated. Retain the command setup with the reported result.

## Run an environment

Run the Spot ball-valve play environment:

```bash
uv run python scripts/play.py \
  --task Isaac-HiveBoard-Spot-BallValve-Play-v0 \
  physics=newton_mjwarp --visualizer newton
```

For the Franka circuit-breaker task with TCP tracking diagnostics:

```bash
uv run python scripts/play.py \
  --task Isaac-HiveBoard-Franka-CircuitBreaker-Play-v0 \
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

If `just` is installed, the repository provides shortcuts such as `just play Franka CircuitBreaker` and `just play Franka Lamp`.

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

Check the configured motion against [How to perform each task](/benchmark/tasks). Simulation success checks are specific to each environment and command setup. A benchmark evaluation requires all **13 conditions with five trials each**.

Report simulated and physical results separately. Document which effects were modeled, including friction, contact, attachment release, and component damage.
