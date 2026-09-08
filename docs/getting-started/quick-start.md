# Getting started

Build and evaluate the printed board, or load the articulated models in a simulator.

## Build the physical benchmark

1. Review the [printing requirements](/hardware/printing).
2. Print one base cell and one attachment to check the press fit.
3. Print and [assemble the full benchmark](/hardware/assembly).
4. Review all 13 conditions in the [module reference](/hardware/modules), including both ball-valve configurations.
5. Follow the [evaluation protocol](/benchmark/protocol).
6. Record five trials per condition with the [Evaluation Runner](/benchmark/evaluation-runner) or [CSV or XLSX template](/benchmark/logging).

## Evaluate an existing board

1. Check every attachment for damage, free motion, and secure seating.
2. Fix the base to a rigid surface and document its orientation.
3. Read the task-specific success criteria and timeouts.
4. Complete familiarization before recording.
5. Run five recorded trials for all 13 conditions. Record every trial with an external camera.
6. Check the trial entries and include the setup photograph, platform description, and recordings with the results.

Go directly to the [evaluation protocol](/benchmark/protocol).

## Work in simulation

- **Articulated assets:** use the URDF or USD files from the main repository in your own simulator or robotics stack.
- **Isaac Lab environment:** use the dedicated repository to train and evaluate policies in Isaac Lab.

Read [Simulation assets](/simulation/assets) before selecting the implementation. The physical and simulation parameters should be reported separately because friction, contact, and press-fit behavior vary between printed instances.

## Minimum reporting package

A physical benchmark result should identify:

- the HiveBoard release or commit;
- printer, material, and any post-processing;
- board mounting orientation;
- robot and end-effector;
- control interface or policy;
- the completed trial log; and
- any broken, replaced, or reseated parts.

Include `setup.jpg` and all 65 trial recordings. The runner exports the trial log and platform description, and lists the required video filenames. Incomplete sessions can be saved and resumed; a submission must contain all 65 trials.
