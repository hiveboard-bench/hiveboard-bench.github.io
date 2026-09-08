# Benchmark overview

HiveBoard evaluates manipulation of mechanisms with constrained motion. A platform may need to rotate a valve about a fixed axis, keep a screw engaged with its thread, align and insert a key, or complete a sequence of dependent actions.

<figure class="doc-image">
  <img src="/images/attachments-overview.png" alt="HiveBoard attachments grouped by manipulation category">
  <figcaption>The HiveBoard attachments shown on the honeycomb base and as isolated mechanisms.</figcaption>
</figure>

## Hardware and platform support

### Reproducible hardware

All structural and functional parts are designed for consumer-grade fused-deposition modelling using PLA. The shared press-fit geometry allows laboratories to reproduce the same board without a specialized fixture.

### Modular tasks

The seven-cell honeycomb base accepts interchangeable attachments. Modules can be swapped without changing the base, and new tasks can reuse the same mounting interface.

### Embodiment-independent evaluation

The protocol treats a platform as an end-effector plus the system that positions and commands it. This keeps the benchmark applicable to fixed-base manipulators, quadruped manipulators, wearable devices, prosthetic hands, and exoskeletons.

### Simulation models

The repository includes articulated assets with visual and collision meshes, joint limits, and nominal physical properties. These assets support inspection and integration in common robotics simulators; a separate repository contains the Isaac Lab implementation.

## Task families

| Family | What it tests | Attachments |
|---|---|---|
| Torque | Rotational manipulation and force transmission | Ball valve, ball valve with friction ring, small and large gate valves, circuit breaker |
| Precision | Alignment, threading, insertion, and controlled rotation | Light bulb, M8 thread, M30 thread, threaded peg insertion |
| Composed assembly | Ordered sequences with stage-wise progress | Covered button, lock and key, drawer, shock absorber |

## What the benchmark reports

For each attachment, the protocol records:

- task outcome and completion time;
- attempts and regrasps;
- prehensile or non-prehensile strategy;
- last completed stage for composed tasks; and
- one primary failure cause for unsuccessful trials.

HiveBoard is intended to characterize task performance and failure modes. The validation protocol does not, by itself, define a universal ranking of platforms or control interfaces.

## Repository files

The current files are maintained in the [HiveBoard parts and protocol repository](https://github.com/EESC-LabRoM/HiveBoard):

- `STL/` — printable parts;
- `CAD/` — editable source geometry;
- `Simulation/` — URDF and USD assets;
- `Documentation/` — protocol and trial templates.
