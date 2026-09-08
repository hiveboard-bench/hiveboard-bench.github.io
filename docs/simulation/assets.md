# Simulation assets

The main HiveBoard repository includes articulated digital assets for the physical mechanisms. Use these files when integrating HiveBoard into an existing simulator or robotics stack.

Browse the [`Simulation/` directory](https://github.com/EESC-LabRoM/HiveBoard/tree/main/Simulation).

## Available mechanism groups

| Folder | Mechanisms |
|---|---|
| `Button/` | Covered push button |
| `Circuit Breaker/` | Toggle mechanism |
| `Drawer/` | Sliding drawer |
| `Honeycomb/` | Base geometry |
| `Key/` | Lock-and-key mechanism |
| `Lamp/` | Threaded light-bulb mechanism |
| `Peg Insertion/` | Threaded peg and socket |
| `Shock Absorber/` | Pin-insertion assembly |
| `Threads/` | M8 and M30 fasteners |
| `Valves/` | Ball and gate valves |

## Model contents

Depending on the mechanism, the simulation package provides:

- visual meshes;
- collision meshes;
- revolute, continuous, and prismatic joints;
- joint ranges and limits;
- nominal mass and inertia; and
- URDF or USD exports.

Threaded motion is represented by coupled rotational and translational joints where a native helical joint is unavailable. This reproduces the advance of a threaded component while keeping the asset portable across simulators.

## URDF and USD assets

| Goal | Starting point |
|---|---|
| Load or visualize a mechanism in an existing stack | URDF or USD assets in the main repository |
| Inspect kinematics or create a custom environment | Articulated asset plus your simulator's loader |
| Train and evaluate policies in Isaac Lab | Dedicated Isaac Lab repository |
| Compare simulation with the printed board | Record model commit and locally identified physical parameters |

## Physical parameters

The provided masses, inertias, friction values, and contact properties are nominal. They are not system-identified values for every printed instance.

Printer calibration, filament, layer orientation, surface wear, sanding, and mechanism assembly can all change physical behavior. If close sim-to-real correspondence is required:

1. print and assemble the target mechanism;
2. measure its range and actuation resistance;
3. identify the relevant simulation parameters;
4. store the overrides separately from the original asset; and
5. report both the asset commit and the identified values.

## Validation before use

Before training or evaluation, check that:

- every referenced mesh resolves;
- the rest configuration has no unintended self-intersection;
- joint axes and ranges match the physical mechanism;
- collision geometry is sufficiently accurate for the task;
- units and scale are correct; and
- a full actuation cycle completes without instability.

A simulation experiment must define observations, actions, reset states, success checks, termination conditions, and evaluation seeds.
