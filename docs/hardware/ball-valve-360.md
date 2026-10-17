---
description: A freely rotating ball-valve variant for policy development, data collection, and controller debugging.
---

# Ball valve — 360° variant

The 360° variant allows the lever to rotate freely through a complete revolution. It is intended for learning-based methods and controller debugging, where commands may overshoot the usual 90° motion and break the standard mechanism at its rotation stop. Allowing continued rotation reduces this source of damage.

The **standard Ball valve task remains a 90° rotation from closed to open**. The variant adds no scored condition to the benchmark.

## Files

Download the models from [`CAD/Valves/Lever Valve/Ball Valve 360/`](https://github.com/hiveboard-bench/HiveBoard/tree/main/CAD/Valves/Lever%20Valve/Ball%20Valve%20360).

| File | Purpose |
|---|---|
| `ball_valve_360_base.obj` | Separate base body |
| `ball_valve_360_lever.obj` | Separate lever body |
| `ball_valve_360_assembly.obj` | Reference assembly |
| `ball_valve_360_assembly.mtl` | Shared OBJ display materials |
| `ball_valve_360_assembly.usdz` | Packaged assembly with original USD materials |

The base and lever were extracted from the supplied assembly without changing their geometry. Keep the shared MTL beside all three OBJ files. These are mesh exports. STEP and STL files are not included for this variant.

## Scale and assembly

| Format or dimension | Setting |
|---|---|
| OBJ coordinates | Centimetres |
| OBJ imported into millimetre-based software | Multiply coordinates by 10 |
| OBJ imported into a metre-based simulator | Multiply coordinates by 0.01 |
| USDZ units | Millimetres, declared by `metersPerUnit = 0.001` |
| Mounting-base width | 88.1 mm |
| Assembly bounding box in the exported pose | Approximately 162.3 × 76.3 × 66.1 mm |

1. Import the base and lever separately and check the dimensions.
2. Print them using the [recommended PLA settings](/hardware/printing) as a starting point.
3. Remove support material and stringing from mating surfaces.
4. Assemble the parts in the orientation shown by the reference model.
5. Check a full revolution by hand and leave the lever's swept path clear of adjacent attachments and the fixture.
6. Check friction-ring fit separately if a ring will be used.

The variant addresses damage caused by rotation beyond the quarter-turn stop. Inspect the printed parts and mounting interface during development, since other contact loads can still damage them.

## Task target and reporting

For a quarter-turn task, mark the initial and target orientations and use the existing 90° criterion. A freely rotating mechanism still needs a defined target pose.

Record the following with the experiment:

- hardware variant: `ball_valve_360`;
- asset release or commit;
- initial and target orientations;
- friction-ring configuration, if used;
- relevant printing or assembly changes.

In the [Evaluation Runner](/benchmark/evaluation-runner), enter the variant and setup details in **Calibration notes**, which are included in `platform.md`. Identify the variant in reported results because its permitted travel differs from the standard valve. Keep the existing task IDs and timeouts for benchmark records. Training or dataset episodes with another target, such as a full revolution, must describe that target separately.

The benchmark submission requirement remains **13 conditions with five trials each**.

## Data collection and simulation

The variant can be used during [DataHive collection](/guides/datahive). Include the hardware variant and task target in the dataset metadata so episodes can be interpreted correctly.

The supplied USDZ contains geometry and materials, with no physics joints or rigid-body configuration. A simulator integration must define and validate the rotation axis, joint type, collision shapes, masses, and friction. Existing Isaac Lab environments continue to use their released assets. Importing this USDZ alone does not change their valve model or joint limits.

See [Simulation assets](/simulation/assets) for integration and physical-parameter checks.
