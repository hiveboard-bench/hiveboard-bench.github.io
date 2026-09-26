---
description: Collect, annotate, validate, and share HiveBoard learning episodes with DataHive, including data collected during benchmark evaluations.
---

# Data collection with DataHive

[DataHive](https://github.com/hiveboard-bench/DataHive) is a toolkit for collecting HiveBoard manipulation episodes with robot states, control commands, camera recordings, and task annotations. It provides a local browser interface, a Python recording API, validation tools, and upload to a configured Hugging Face dataset repository.

**We encourage labs to collect learning data while running a benchmark evaluation.** Recording the same trials can provide both performance measurements and trajectories for subsequent research. Dataset collection is optional: a benchmark submission still requires all 13 conditions with five trials each, while the [learning-dataset call](/contribute/evaluations) has no fixed episode count or requirement to cover every condition.

## Choose the workflow

| Tool | Purpose | Output |
|---|---|---|
| [Website Evaluation Runner](/benchmark/evaluation-runner) | Time trials, enter outcomes, and prepare a benchmark submission in the browser | Benchmark trial CSV, platform description, and submission package |
| DataHive local Runner and Annotate pages | Organize collection sessions, connect recorded states, commands, and videos to trials, and annotate and validate episodes | HDF5 episodes, camera MP4 files, and an extended annotation CSV |

The website Evaluation Runner does not acquire robot or camera streams. DataHive requires a recording integration with your experimental system; installing the toolkit alone does not connect it to a robot.

## Lab registration and credentials

All manipulation data contributed across laboratories is centralized directly in the official **HiveBoard** repository on Hugging Face (`HiveBoard/<lab_id>`). Labs do not need to create or host separate repositories; all contributions always go directly to our official benchmark Hub storage.

To receive authorization and write access to your designated repository:

1. **Submit the registration form**: Fill out the [DataHive Lab Registration Form](https://docs.google.com/forms/d/e/1FAIpQLScfDJvfcweYjv8bLDEALWOJk1q4mW3xOeB52aUey1VWWp_pxQ/viewform?usp=dialog) with your lab name, institution, robotic platform, and contact email.
2. **Receive credentials**: The HiveBoard team will register your laboratory and provide:
   - Your unique **Lab ID** (`lab_id`, e.g. `labrom`, `cmu_bimanual`);
   - An authentication **Hugging Face write token** allowing DataHive to upload directly to `HiveBoard/<lab_id>`.
3. **Direct upload destination**: All episodes uploaded through DataHive go straight to `HiveBoard/<lab_id>`. This ensures standardized tracking, dataset indexing, and automated aggregation for future benchmark releases.

## Install and configure

Use Python 3.10 or later in a dedicated environment. Install `datahive-tools`:

```bash
pip install datahive-tools
pip install "datahive-tools[video]"   # includes OpenCV for video integrity checks
```

Initialize your lab configuration:

```bash
datahive init
```

The CLI prompts for:
- `Lab ID`: your assigned lab identifier from the registration form;
- `Hugging Face token`: your write token issued by the HiveBoard team.

`init` verifies your token against the Hugging Face Hub (via `whoami()`), writes your configuration to `~/.datahive/config.yaml` with secure `0600` file permissions, and automatically sets the remote repository destination to our official repository `HiveBoard/<lab_id>`.

Next, create the robot profile skeleton:

```bash
datahive new-profile
```

Fill in `samples/robot_profile.yaml` before recording. Describe the robot, end-effector, control mode, state and command conventions, joint order, recording rate, and cameras. See the upstream [robot-profile reference](https://github.com/hiveboard-bench/DataHive/blob/main/src/datahive/skills/datahive-data-prep/reference/robot-profile.md) for field definitions.

From the same working directory, start the interface:

```bash
datahive interface --port 8000
```

Open `http://127.0.0.1:8000` on that computer. Use **Runner** to plan a session or **Annotate** to inspect existing episodes.

## Record a session

Select the task conditions and collection mode. The Runner defaults to five trials per selected condition. Dataset collections can use a subset of conditions and additional sessions; the benchmark's 65-trial requirement is not a dataset-size limit.

| Mode | Recording procedure |
|---|---|
| Manual | Use the countdown and stopwatch, enter the outcome, and attach the HDF5 episode and one MP4 per camera listed in the robot profile. Your experimental system records these files. |
| Automatic | Connect your recording/control script through `CollectClient`. The script receives the queued task, writes states and commands through `EpisodeWriter`, attaches camera recordings, and returns the episode for annotation. The evaluator still determines the outcome. |

Follow the upstream [data format](https://github.com/hiveboard-bench/DataHive/blob/main/src/datahive/skills/datahive-data-prep/reference/layout-and-format.md) when preparing files. For automatic collection, use the [robot integration guide](https://github.com/hiveboard-bench/DataHive/blob/main/src/datahive/skills/datahive-auto-collect/reference/adapting.md). Robot drivers, command generation, and camera acquisition remain part of the lab's integration.

::: tip Reference integration: Franka Research 3
For a complete example to use as a base, see [`fr3_datahive`](https://github.com/hiveboard-bench/fr3_datahive) for a Dockerized setup with ROS 1 Noetic, teleoperation, and DataHive recording on a Franka Research 3 (FR3).
:::

**Record physical trials with an external camera**, with the board, end-effector, and task state visible throughout. Include that camera in the profile when storing its recording with the DataHive episode. For simulation, record rendered camera observations and identify the simulator and configuration.

### Check the recording setup first

DataHive currently expects aligned state and command arrays in HDF5, a proprioception rate of at least 100 Hz, and one MP4 per configured camera. Its video checks require matching camera resolution, frame rate, and frame count; each image dimension must be 180–1280 pixels, and video duration must match the episode within 0.5 seconds. Check the upstream format reference when choosing acquisition settings.

Record and validate a short pilot episode before starting the collection. Preserve the original streams and timestamps and document any conversion. If your system cannot meet these constraints, discuss an alternative format with the organizers before collection. The dataset call also welcomes depth, force/torque, tactile, and other signals; agree on supplemental files for modalities outside DataHive's current episode format.

### Optional board localization

The [ArUco cell](/hardware/aruco-cell) can provide a visual reference for board pose during collection. Retain the camera calibration, measured marker size, marker-to-board transform, and timestamped images used for detection. Store any estimated poses with their frame conventions and timestamps. Marker detection and pose estimation are part of the lab's recording integration. Keep calibration records with the session metadata and agree on supplemental files with the organizers where needed.

## Annotate and validate

The local interface supports review and validation. The equivalent command sequence for an existing episode is:

```bash
datahive check EPISODE_ID
datahive annotate EPISODE_ID
datahive validate EPISODE_ID
```

Replace `EPISODE_ID` with the recorded episode identifier. `check` inspects recording integrity before annotation. `annotate` records task outcome and related fields; `validate` checks the episode and annotation together. Inspect both errors and warnings.

Use the [task definitions](/benchmark/tasks) and [counting conventions](/benchmark/logging) for outcomes, stages, attempts, and regrasps. Preserve unsuccessful episodes and label interventions, resets, and aborted recordings. A benchmark trial may contain several attempts or regrasps; these do not become separate scored trials.

## Collect during a benchmark evaluation

1. Test recording and synchronization before the scored trials. Keep familiarization and training episodes identifiable and separate from the evaluation set.
2. Follow the [evaluation protocol](/benchmark/protocol), including all 13 conditions, five trials per condition, fixed control settings, neutral starting poses, resets, and task timeouts. Use the required separate blocks for the two ball-valve configurations.
3. Record the same trial's states, commands, and camera streams. Maintain an index linking its DataHive session and episode IDs to the benchmark `trial_id` and `attachment_id`; session trial numbers may follow a different order from the benchmark template.
4. Time the benchmark trial from the first commanded task motion until success, a safety stop, or timeout. Save a completion time only for successful trials. A recording may include time before or after the task, so its total duration is not automatically the benchmark completion time.
5. Prepare the benchmark log and [submission package](/benchmark/results) separately from the learning dataset. Report every scored trial, including failures, and retain the external-camera recording for each one.

DataHive's `trials.csv` contains extra annotation fields and is **not a drop-in replacement for the benchmark CSV**. For the benchmark submission, use the website Evaluation Runner or transfer the scored results into the [benchmark template](/benchmark/logging), preserving its condition-to-trial mapping and filling all required counts and stages. Keep a correspondence to the original episode IDs. Do not copy DataHive's operator and annotator names into the public benchmark files.

To generate the benchmark submission ZIP, enter the scored results and platform details in the website Evaluation Runner. It does not currently import DataHive CSV files. The template can be used to prepare the records before entry.

Passing DataHive validation checks an episode's format and annotations; it does not establish that all 65 benchmark trials or the full evaluation protocol have been completed. Data collected during evaluation can be contributed for future training, but must remain excluded from training or tuning the policy whose performance those trials report.

## Upload to Hugging Face Hub

DataHive synchronizes local manipulation sessions directly with our official Hugging Face dataset repository (`HiveBoard/<lab_id>`). Labs do not create external repositories; all data contributes directly to the central HiveBoard repository.

### Upload a single episode

After reviewing and validating an episode, upload it using the CLI:

```bash
datahive upload EPISODE_ID
```

Before uploading, DataHive runs a preflight check to confirm that the episode status is `validated` and directory structure constraints are satisfied. When confirmed, DataHive:
- uploads the trajectory array to `{session_id}/episodes/{episode_id}.h5`;
- uploads each camera recording to `{session_id}/episodes/{camera_name}.mp4`;
- uploads `{session_id}/setup.jpg` if present;
- automatically downloads and merges `{session_id}/trials.csv` directly in the HiveBoard repository, keeping remote records synchronized with local additions;
- marks the episode status as `uploaded` in the local index (`samples/.datahive/index.sqlite`).

Use `--yes` (`-y`) to skip the confirmation prompt, or `--force` to re-upload an already uploaded episode.

### Synchronize all sessions (`datahive sync`)

To reconcile your entire local `samples/` directory with our Hugging Face repository in one command:

```bash
datahive sync
```

`sync` scans all sessions under `samples/`, detects any validated episodes that have not yet been uploaded, pushes them directly to `HiveBoard/<lab_id>`, and merges all corresponding `trials.csv` files on the Hub.

Use `--dry-run` to preview what would be uploaded without transferring files:

```bash
datahive sync --dry-run
```

### Uploading from the web interface

Both operations are also accessible in the local web interface (`datahive interface`):
- In the **Annotate** view, click the **Upload** button on any validated episode card to upload it immediately to `HiveBoard/<lab_id>`.
- In the header, click **Sync** to reconcile all sessions with the Hub.

### Remote repository layout

All contributed episodes are organized inside our official dataset repository:

```text
HiveBoard/<lab_id>/                  # Official HiveBoard dataset repository
├── 2026-09-20_session_01/
│   ├── episodes/
│   │   ├── episode_0001.h5          # States, commands, and joint telemetry
│   │   ├── camera_wrist.mp4         # Synchronized camera video
│   │   ├── camera_external.mp4      # External view video
│   │   └── ...
│   ├── trials.csv                   # Session trials and outcome annotations
│   └── setup.jpg                    # Platform setup photo
└── ...
```

### Submitting for organizer review

Because all uploads go directly to our official HiveBoard repository:
- You do not need to configure external storage, manage third-party repositories, or transfer archives manually.
- When you finish collecting a session or dataset, simply email **ricardo.godoy@usp.br** with your `lab_id`, platform description, and covered tasks, as detailed in the [dataset submission instructions](/contribute/evaluations#send-the-data-for-review). The HiveBoard team will review the uploaded episodes directly in `HiveBoard/<lab_id>`.

Record the DataHive package version or commit with the dataset. DataHive builds on the architecture and workflows of [Oopsie Data](https://github.com/oopsie-data); see its [README](https://github.com/hiveboard-bench/DataHive#readme) for project credits and current usage.
