---
description: Convert datasets recorded outside DataHive into the HiveBoard episode format, including the 100 Hz resampling rule and the annotation policy.
---

# Converting existing datasets to the HiveBoard format

HiveBoard's learning-dataset call accepts episodes in the **`datahive_episode_v1`** format written by [DataHive](/guides/datahive). If your episodes were recorded with another stack — LeRobot, a custom HDF5 writer, a ROS bag — they can still contribute. This page is the specification for the converter in `tools/dataset_conversion/` and for the output it produces.

::: warning This targets `datahive_episode_v1`
DataHive writes `datahive_episode_v1`. The OOPSIE project's `oopsiedata_format_v1` is a **different** schema with different field names, action keys and annotation taxonomy. The converter's structure (helper-based writer, per-episode isolation, validation-first loop) follows the [OOPSIE conversion guide](https://oopsie-data.com/conversion/), but the data written here is `datahive_episode_v1` and is validated by `datahive validate`.
:::

## When to use the converter

Use it when you already have episodes and want them accepted by the HiveBoard learning-data pipeline without re-recording. DataHive remains the recommended collection path — recording through it gives you the format for free and keeps annotations in the same tool.

Supported today:

| Source | Adapter | Status |
|---|---|---|
| LeRobot Dataset `v3.x` | `tools/dataset_conversion/sources/lerobot_v3.py` | Implemented and tested on `victorvanhalst/hive_botao_100` |
| RLDS, ROS bag, custom HDF5 | — | Add one module under `sources/`; the conversion core is unchanged |

## Output: the samples root

The converter writes a standard DataHive samples root, so every existing command works on the result:

```text
<samples>/
  robot_profile.yaml
  <session_id>/
    trials_<session_id>.csv
    episodes/
      <episode_id>.h5
      <episode_id>_cam_wrist.mp4
      <episode_id>_cam_external.mp4
  conversion_report.json
```

Identifiers:

- `session_id` — `{lab_id}_{platform_id}_{YYYYMMDD}` unless `--session-id` is given.
- `trial_id` — `1..N` in source-episode order.
- `episode_id` — `{session_id}_t{trial_id}`, globally unique under the samples root.

## Episode schema

```text
/
  [attrs] lab_id, platform_id, session_id, episode_id, trial_id, task_ids (JSON),
          schema_version="datahive_episode_v1", created_at, low_level.mode,
          cameras (JSON), collection_mode, policy, _json_fields, + profile snapshot
  observations/
    robot_states/timestamp         (T,1) float64, strictly increasing seconds
    robot_states/joint_position    (T,5) float64, ARM joints only
    robot_states/ee_state          (T,1) float64, gripper state
    video_paths/<camera>           string, filename inside the episode directory
  actions/
    timestamp                      (T,1) float64
    joint_position                 (T,5) float64
    gripper_position               (T,1) float64
  episode_annotations/<annotator>/ attributes mirroring the trials row
```

Every dataset carries a `provenance` attribute, `measured` or `estimated`.

**`joint_position` holds arm joints only.** The gripper is not a column of it; it lives in `ee_state` (observation) and `gripper_position` (action). `manipulator.joint_names` and `action_joint_names` therefore list the arm joints, and `end_effector.actuated_dof` describes the gripper.

## Source to target mapping

| Target | LeRobot v3.0 source |
|---|---|
| `robot_states/joint_position` | `observation.state[:, :-1]` — five arm joints |
| `robot_states/ee_state` | `observation.state[:, -1:]` — gripper column |
| `actions/joint_position` | `action[:, :-1]` |
| `actions/gripper_position` | `action[:, -1:]` |
| timestamps | uniform grid at the output rate, starting at 0 |
| `video_paths/wrist`, `video_paths/external` | per-episode segment sliced from the shard for `observation.images.wrist` / `.external` |
| `task_ids` | `--attachment-id` (HiveBoard attachment id, e.g. `button`) |
| trials `attachment_id` | the same value |
| profile `control_freq` | `--resample-hz` (default 100) |
| profile camera `fps` | source fps (25 here) |

Joint names come from `meta/info.json` (`features["observation.state"]["names"]`, with the `.pos` suffix stripped). The adapter requires the gripper to be the last entry and refuses the dataset otherwise, because silently mislabelling a joint is worse than failing.

## The 100 Hz requirement

`datahive validate` treats proprioception below **99 Hz** as an error (`MIN_SAMPLE_RATE_HZ = 100.0`). Most third-party datasets are 25–30 Hz.

The converter resamples **proprioception and actions** onto a uniform grid at `--resample-hz` (default 100) by linear interpolation:

```text
duration = (T_source - 1) / source_fps
T_output = round(duration * resample_hz) + 1
```

First and last samples are preserved exactly, so start and end poses are untouched. Cameras are **not** resampled: the validator treats camera fps as independent of the control rate, so videos keep the source frame rate and only their duration must match the episode (within 0.5 s). This avoids a four-fold frame-duplication re-encode.

::: warning Resampling is interpolation, not measurement
`provenance` is a dataset-level attribute, so a resampled array is written wholly as `estimated` — original and synthesised ticks cannot be distinguished inside the file. If 100 Hz fidelity matters for your scientific question, collect at 100 Hz rather than relying on the upsample.
:::

`--resample-hz` below 100 is refused unless `--allow-low-rate` is passed. That flag writes the data at its native rate; `datahive validate` will then reject it. It exists for archival, not for submission.

## Robot profile

`robot_profile.yaml` is written once per samples root and snapshotted into every episode. Generate it from a bundled profile:

```bash
.venv/Scripts/python.exe tools/convert_dataset.py profile \
    --samples out/ --profile so101
```

The bundled SO-101 profile (`tools/dataset_conversion/profiles/so101.yaml`) declares:

- `control_freq: 100` — the converted rate, not the source rate.
- `action_space: [joint_position, gripper_position]`.
- five arm joints plus `end_effector: {type: gripper, actuated_dof: 1, command_modality: position}`.
- `low_level.mode: stock` — mandatory; the header check fails without it.
- cameras `wrist` and `external` at the recorded resolution and fps.
- `units_and_frames` recording that LeRobot's SO-101 `*.pos` readings are **normalized servo units, not radians**. The converter never rescales values.

To convert a different robot, add a profile with matching `manipulator.joint_names`, `action_joint_names`, `control_freq` and `cameras`, then pass `--profile <name-or-path>`.

## Annotations

`datahive_episode_v1` has no unlabelled state: every episode needs a trials row, and DataHive's own guidance is that annotation is human judgement. The converter therefore never guesses silently.

Supply a sidecar CSV keyed by source episode index. Generate the skeleton first — it pre-fills `strategy` and `stage_reached` and leaves `outcome` for you:

```bash
.venv/Scripts/python.exe tools/convert_dataset.py annotations-template \
    --source victorvanhalst/hive_botao_100 --stage-reached 2 --output labels.csv
```

The filled file looks like:

```csv
episode_index,outcome,strategy,stage_reached,failure_cause,failure_cause_detail,severity,n_attempts,n_regrasps,notes
0,success,prehensile,2,,,,,,
1,fail,prehensile,1,slip,,minor,2,1,gripper slipped on the cover
```

| Column | Required | Allowed values |
|---|---|---|
| `episode_index` | yes | source episode index, unique |
| `outcome` | yes | `success`, `fail`, `timeout`, `safety_stop` |
| `strategy` | optional | `prehensile`, `non_prehensile`; defaults to `--strategy` |
| `stage_reached` | composed assemblies only | integer ≥ 0; **forbidden** for other tasks |
| `failure_cause` | non-success | `grasp_geometry`, `kinematic_limit`, `perception`, `slip`, `force_limit`, `control_precision`, `other` |
| `failure_cause_detail` | `failure_cause=other` | free text |
| `severity` | optional, non-success | `minor`, `moderate`, `critical` |
| `n_attempts`, `n_regrasps` | optional | integer ≥ 0 |
| `notes` | optional | free text |

Rows whose `outcome` is still blank are treated as not yet annotated, so you can convert a subset (`--limit`) before the whole sheet is filled. A selected episode with no filled row is an error.

Rules enforced before anything is written:

- A successful trial must not carry `failure_cause`, `severity` or `failure_cause_detail`.
- A non-successful trial must carry `failure_cause`; `other` requires detail.
- `completion_time_s` is filled from the episode's own duration, never from the CSV.
- Composed assemblies (`button`, `lock`, `drawer`, `shock_absorber`) need `stage_reached`; simple attachments must not have it.

Without a CSV the run refuses (exit code 2). If you genuinely have no labels, `--assume-success` records `outcome=success` with `--strategy` and, for composed tasks, `--stage-reached`, and writes this into the row's notes:

> `[conversion] outcome=success assumed: no annotation sidecar was supplied and --assume-success was set. Verify labels before submission.`

The run banner and the report both count assumed episodes. **An assumed label is not a label.**

## Video

Per camera and episode, the converter slices the source segment, re-encodes to H.264 High/`yuv420p`/CRF 19 with `+faststart`, keeps the source frame rate, and forces the segment to exactly the source frame count (trimming, or cloning the last frame inside the validator's tolerance). Sides are capped at 1280 px and must be at least 180 px. Segments are located by accumulating each episode's `length` within its shard, because LeRobot packs whole episodes and concatenates them; nominal `from_timestamp` values are used only as a cross-check.

`--video-mode copy` stream-copies the source codec. It is much faster and it is **not** suitable for submission: AV1 output may not open in OpenCV or a browser. The converter refuses to mark copied output as browser-playable.

## Usage

```bash
# 0. dependencies (once)
.venv/Scripts/python.exe -m pip install -r tools/requirements-dataset-conversion.txt

# 1. write the robot profile into the samples root
.venv/Scripts/python.exe tools/convert_dataset.py profile \
    --samples out/ --profile so101

# 2. see what the adapter reads, before writing anything
.venv/Scripts/python.exe tools/convert_dataset.py probe \
    --source victorvanhalst/hive_botao_100 --metadata-only

# 3. create the annotation sheet, one row per source episode
.venv/Scripts/python.exe tools/convert_dataset.py annotations-template \
    --source victorvanhalst/hive_botao_100 --stage-reached 2 --output labels.csv
#    ... then fill in the outcome column

# 4. rehearse
.venv/Scripts/python.exe tools/convert_dataset.py convert \
    --source victorvanhalst/hive_botao_100 --samples out/ \
    --lab-id my_lab --operator-name ana --annotator-name ana \
    --attachment-id button --strategy prehensile --stage-reached 2 \
    --annotations labels.csv --limit 3 --dry-run

# 5. convert, validating each episode as it is written
.venv/Scripts/python.exe tools/convert_dataset.py convert \
    --source victorvanhalst/hive_botao_100 --samples out/ \
    --lab-id my_lab --operator-name ana --annotator-name ana \
    --attachment-id button --strategy prehensile --stage-reached 2 \
    --annotations labels.csv
```

**No labels yet?** Omit `--annotations` and add `--assume-success` (plus `--stage-reached` for composed tasks). Every row is then stamped as an assumption; see [Annotations](#annotations).

`--source` accepts a local directory or a Hugging Face `owner/name` repo id (downloaded into `--cache-dir`, default `.cache/datasets`). `--start-index` and `--limit` select source episodes; `--overwrite` redoes an episode; a re-run leaves completed episodes untouched.

Exit codes: `0` no failures (skips allowed), `1` at least one episode failed, `2` configuration refusal. A **skip** is an episode that is legitimately out of scope — under one second, no video, unresolvable segment — and is listed with its reason in `conversion_report.json`.

## Loading a converted episode

The submission requirements ask for a reader alongside the data. This is enough to align states, actions and video:

```python
import h5py, cv2, json

EPISODE = "out/my_lab_so101_follower_so101_gripper_20261117/my_lab_so101_follower_so101_gripper_20261117_t1"

with h5py.File(f"{EPISODE}.h5", "r") as f:
    states = f["observations/robot_states/joint_position"][:]   # (T, 5)
    gripper = f["observations/robot_states/ee_state"][:]        # (T, 1)
    actions = f["actions/joint_position"][:]                    # (T, 5)
    timestamps = f["observations/robot_states/timestamp"][:, 0]
    cameras = {k: f["observations/video_paths"][k][()].decode()
               for k in f["observations/video_paths"]}
    camera_meta = {c["name"]: c for c in json.loads(f.attrs["cameras"])}

fps = camera_meta["wrist"]["fps"]
video = cv2.VideoCapture(f"{EPISODE.rsplit('/', 1)[0]}/{cameras['wrist']}")
frame_at = lambda i: video.read()[1]   # frame index i, at the camera's own fps
```

States, actions and timestamps share one index; camera frames run at the camera fps declared in the header.

## Validation

Every converted episode is validated as it is written (`--no-validate` disables this). To re-check independently:

```bash
.venv/Scripts/datahive validate <episode_id> --samples out/
.venv/Scripts/datahive check --samples out/ --json
```

A valid episode reports a sample rate of 100.0 Hz, no timestamp gaps, and one entry per declared camera. `datahive upload` will then accept it like any DataHive-recorded episode.

## Limitations

- **No `language_instruction`.** `datahive_episode_v1` has no free-text instruction field; the task lives in `task_ids` and the annotation. Name the HiveBoard task in your annotation notes if you need it recoverable.
- **No additional sensor streams.** Depth, force/torque and tactile data are not carried by this converter.
- **No cartesian, velocity or base actions from LeRobot.** The adapter reads joint-position + gripper only; use a different adapter for other action spaces.
- **Source units are not normalised.** Values are written as recorded; read `units_and_frames`.
- **Upsampling adds no information.** 25 Hz data becomes a 100 Hz file, not a 100 Hz recording.
- **One profile per samples root.** Converting two different robots means two samples roots.

## Appendix: `hive_botao_100` probe

`probe --metadata-only` on `victorvanhalst/hive_botao_100`:

```text
version    v3.0   fps=25.0   robot=so_follower
episodes   100   frames=30358
cameras    wrist<-observation.images.wrist, external<-observation.images.external
joints     ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
tasks      {0: 'Open the cover and press the button'}
columns    data/chunk_index, data/file_index, dataset_from_index, dataset_to_index,
           episode_index, length, tasks,
           videos/observation.images.<cam>/{chunk_index,file_index,from_timestamp,to_timestamp}
episode0   length=263 dataset=[0, 263)  wrist/file_index=0  from=0.0  to=10.52
```

Timestamps in `meta/episodes` are seconds (`to - from = length / fps`), and `length` matches the data rows for every episode. State and action values span roughly −103 to 98, consistent with normalized servo units rather than radians.

Converting all 100 episodes with `--assume-success --attachment-id button --stage-reached 2`:

```text
written=100 exists=0 skipped=0 failed=0
episodes checked: 100  ok: 100  failed: 0
sample rates: [100.0]
total timestamp gaps: 0
total output steps: 121132
distinct warnings: 0
```

121,132 is exactly `sum(round((length - 1) / 25 * 100) + 1)` over the source episodes, so no frames were dropped or invented beyond the interpolation. Source episodes run 5.64–18.16 s, inside the schema's 1–600 s bounds. The 917 MB source produced 540 MB of output (100 HDF5 episodes plus 200 H.264 videos at 640×480/25 fps), in about five minutes on CPU.
