---
description: Contribute HiveBoard manipulation datasets with robot states, actions, camera observations, and task outcomes for learning-based methods.
---

# Open call: datasets for learning-based manipulation

We invite laboratories and individual researchers to collect and contribute **HiveBoard manipulation datasets for training and evaluating learning-based methods**. We seek demonstrations and robot rollouts containing camera observations, robot states, executed control commands, and task outcomes. These data can support imitation learning, visuomotor policy learning, and learning from both successful and unsuccessful attempts.

Robot grippers, dexterous hands, teleoperated and autonomous manipulators, and prosthetic hands are welcome. Physical and simulated evaluations must be identified and submitted separately.

**There is no fixed episode count or requirement to cover all 13 conditions for this call.** Larger collections are encouraged. Contributors can focus on an agreed subset of tasks and collect as many useful demonstrations and rollouts as possible. Include unsuccessful attempts as well as successes, and report the number of episodes per condition. A high success rate is not a requirement for contributing data.

We seek learning data with the robot and sensor streams described below. A formal benchmark evaluation is optional and can accompany the dataset as a separate submission.

## Before starting

Read the [evaluation protocol](/benchmark/protocol) and [task definitions](/hardware/modules). If you need to print the board, start with the [printing](/hardware/printing) and [assembly](/hardware/assembly) guides.

**Contact [ricardo.godoy@usp.br](mailto:ricardo.godoy@usp.br?subject=HiveBoard%20learning%20dataset%20contribution) before collection** to agree on the tasks, recorded signals, synchronization, file formats, and intended data reuse. Include your robot, end-effector, control interface, cameras and other sensors, board orientation, and whether the data are physical or simulated. Discuss unavailable signals or modified mechanisms at this stage.

## Data to record

Record the streams at their native acquisition rates and provide timestamps so observations, measured states, and issued commands can be aligned. State which signals are measured, estimated, or commanded. Preserve the raw data and document any filtering or resampling in derived files.

| Data | What to include |
|---|---|
| Robot states | Joint positions and velocities, end-effector pose, and gripper opening or hand joint states. Include measured effort, torque, or motor current where available. Specify joint order, units, and coordinate frames. |
| Actions and commands | The commands actually sent to the robot or controller, their timestamps, and control mode. State whether actions are joint targets, Cartesian targets or increments, velocities, torques, or gripper commands. Include scaling, limits, and the command frame. |
| Camera observations | Timestamped RGB streams from wrist-mounted and external cameras used during collection. Include depth when available, with its units, scale, invalid-value convention, and RGB alignment. Document image resolution and encoding. |
| Other sensors | Force/torque, tactile, proprioceptive, or interface signals where available, with channel definitions, units, and timestamps. For prosthetic systems, describe the recorded hand states and input signals. |
| Timing and calibration | Sampling rates, clock sources, timestamp meaning, synchronization method, known offsets or delays, and dropped samples. Include camera intrinsics, distortion parameters, extrinsics, and frame transforms used to interpret the observations. |
| Episode metadata | Episode ID, task/condition ID, platform configuration, asset version, initial state, start/end boundaries, outcome, failure cause, and correspondence with the benchmark trial record where applicable. Label resets, interventions, and aborted episodes. |

For simulation, also record the simulator and asset versions, physics settings, seeds, and any randomization used. Identify privileged simulator state separately from the observations available to a policy.

The baseline is camera observations, robot states, and timestamped actions. Additional sensing is welcome. If your platform cannot expose a baseline signal, discuss the limitation with the organizers before collection. Do not infer control commands from measured motion and present them as recorded actions.

## Collect the episodes

1. Document the platform configuration, HiveBoard version, board orientation, printing settings, and any post-processing or protocol deviations.
2. Define the tasks, initial states, reset procedures, and episode boundaries. Identify changes in platform configuration, control method, or task difficulty in the metadata.
3. Collect demonstrations or robot rollouts with synchronized observations, states, and commands. Larger datasets and variation in starting states, viewpoints, operators, and strategies are welcome; document the variation used.
4. Preserve successes, failures, timeouts, interventions, and aborted episodes, and label them accurately. Distinguish the ball-valve configurations and other task variants.
5. Record task outcomes and failure causes for each episode. Where applicable, use the stage, attempt, and regrasp definitions in the [logging instructions](/benchmark/logging).

Identify the demonstration source or policy used for each episode and any data used to train or tune it. Preserve session boundaries so related trajectories can be grouped when constructing training and test splits. If you also report benchmark performance, keep those scored trials separate from training data.

**Record physical episodes with an external camera**, keeping the board, end-effector, and relevant task state visible. Record the complete episode without cuts and align the video with the episode timestamps. For simulated data, include rendered observations and a way to inspect task execution.

## Required submission

| Material | Requirement |
|---|---|
| Episode records | One entry per episode with condition, outcome, duration, data-file references, and counts summarized per task |
| Platform description | `platform.md` identifying the robot, end-effector, control interface, evaluation mode, mounting, and relevant setup details |
| Setup image | `setup.jpg` showing the experimental setup; use a view of the simulated setup for simulation |
| Recordings | External-camera video for physical episodes and rendered observations for simulation, mapped to episode IDs and timestamps |
| Learning data | Timestamped robot states, actions, camera streams, and other recorded sensors, grouped into episodes |
| Dataset documentation | Signal definitions, units, frames, rates, synchronization and calibration details, missing signals, collection procedure, and data reuse terms |
| Episode index and reader | An index mapping learning-data episodes to trial IDs, task conditions, outcomes, and files; a script or notebook that loads an episode and aligns observations, states, and actions |
| Contact and permission | Contact name, email, institution or Independent, package link, and proposed data reuse and publication terms, supplied by email |

**Record the learning data through your experimental system.** The Evaluation Runner does not acquire robot telemetry, camera streams, or control commands, and is not required for this dataset call.

ROS bags, HDF5, Zarr, or an existing documented learning-dataset format can be discussed with the organizers. Include a small loading example with dependency versions. Video plus a trial CSV alone is insufficient for the learning-data call.

Organize the learning data, calibration, episode index, reader, setup description, and dataset README in a documented folder or archive. If the data are too large for one archive, use a shared folder with a file index and checksums covering all data shards.

## Send the data for review

Upload the completed package to storage you control and give **ricardo.godoy@usp.br** download access. Keep the package private during review.

Email [ricardo.godoy@usp.br](mailto:ricardo.godoy@usp.br?subject=HiveBoard%20learning%20dataset%20submission) directly with your name, institution or Independent, dataset link, platform description, tasks covered, episode counts, and recorded modalities. Include known limitations and proposed data reuse terms. **Dataset submissions do not go through the runner's 65-trial submission form.**

The organizers will review the dataset structure, signal definitions, alignment, loading example, and recordings, and request corrections if needed. Public release follows review and agreement on publication and reuse permissions. Contact details and private download links will remain private.

## Authorship and acknowledgments

Contributors who provide a **substantial, accepted dataset** may be eligible for authorship on future HiveBoard dataset papers that use and report their contribution.

The organizers will consider the number of usable episodes, data quality and completeness, coverage of tasks and platforms, and work on collection, calibration, curation, and validation. There is no fixed episode threshold that guarantees authorship. Discuss the planned contribution and expected credit with the authors before collection.

Authorship is assessed for each paper and each contributor. Eligible contributors should also participate in drafting or substantive revision, approve the final manuscript, and take responsibility for their contribution. Dataset acceptance does not guarantee authorship on every future paper or establish author order.

Smaller contributions may be acknowledged with the contributor's consent. Record each person's role in the dataset documentation so credit reflects the work performed.

## Optional benchmark evaluation

If you also want to report a formal benchmark evaluation, use the [Evaluation Runner](/benchmark/evaluation-runner) and submit **all 13 conditions with five trials each, for 65 trials total**, following the [evaluation submission process](/benchmark/results#submit-for-organizer-review). That requirement applies to benchmark evaluations only, not to this learning-dataset call.

## Data reuse

Agree on the dataset's reuse license with the organizers before public release, including permission to train and evaluate learning-based methods under the agreed noncommercial terms. Document third-party restrictions and permission to share identifiable people or other personal data in recordings. The runner's permission to publish approved benchmark results does not by itself assign a license to a learning dataset.
