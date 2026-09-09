# Add a new attachment

An attachment is part of the benchmark only when its physical geometry, simulation model, task definition, and evaluation metadata agree.

For the [ADL and bimanual open calls](/contribute/), contact the authors before starting the design. Read the [submission requirements and credit policy](/contribute/requirements) alongside this technical guide.

## 1. Reuse the mounting interface

Start from the existing press-fit geometry in the CAD repository. Do not redraw the interface from measurements when an editable source is available.

The attachment should:

- seat fully in a standard honeycomb cell;
- resist the expected manipulation load;
- remain removable without damaging the base;
- avoid interfering with neighboring cells; and
- preserve a repeatable initial state.

For an attachment wider than one cell, document exactly which adjacent cells must remain free.

## 2. Define the task

Write the task definition before producing the final model:

- initial state;
- required manipulation sequence;
- observable success state;
- timeout;
- number and names of intermediate stages, if any;
- permitted reset procedure; and
- likely failure modes.

A success condition must be measurable across different platforms without requiring access to internal robot state.

## 3. Produce the physical files

Provide:

- editable CAD source;
- STL files with descriptive names;
- a complete bill of printed components;
- print orientation and support requirements;
- assembly instructions; and
- a photograph or render of the assembled mechanism.

If the mechanism requires non-printed hardware, list the exact specification and a generally available alternative.

## 4. Produce the articulated model

The simulation asset should include:

- visual and collision geometry;
- named joints and links;
- joint axes, limits, and initial values;
- mass and inertia values with their derivation;
- contact and friction assumptions; and
- a smoke test that drives the mechanism through its full range.

Keep identified parameters separate from nominal defaults when they depend on a particular printed instance.

## 5. Extend the protocol

Propose a stable lowercase `attachment_id` and supply the task metadata for review. Once the attachment is accepted into a versioned benchmark extension, coordinate the following updates with the maintainers:

- the module reference;
- the success and timeout table;
- stage numbering, if applicable;
- `trials.csv` and `trials.xlsx`;
- logging validation; and
- the reproducibility checklist.

The current benchmark still requires all 13 conditions, with five trials each. A proposed attachment does not replace an existing condition or change the current runner's required trial count.

## 6. Submit the contribution

Include physical and simulation evidence in the pull request:

- assembled-module image;
- manual actuation through the full range;
- successful fit in the HiveBoard base;
- simulation load test; and
- one completed example trial row.
