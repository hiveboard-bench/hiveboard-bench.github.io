# Evaluation protocol

This protocol characterizes robot-side performance across HiveBoard attachments. It is suitable for teleoperated arms, wearable interfaces, exoskeletons, and autonomous policies, provided that the control mode is held constant across the recorded trials being compared.

The source protocol is available in [`Documentation/PROTOCOL.md`](https://github.com/EESC-LabRoM/HiveBoard/blob/main/Documentation/PROTOCOL.md). Submissions through this documentation require all 13 conditions, with five recorded trials per condition (65 trials).

## Required material

- One printed HiveBoard base.
- The complete attachment set, including the four-piece friction-ring accessory for the ball-valve ring condition.
- One manipulation platform: an end-effector plus the system that positions and commands it.
- The [Evaluation Runner](/benchmark/evaluation-runner) or supplied trial logging template.
- An external camera with a view of the board and end-effector.

## Prepare the setup

1. Fix the board to a rigid surface within the platform's working volume.
2. Record whether it is mounted horizontally or vertically.
3. Fully seat every attachment before starting.
4. Position the platform so that the board is not moved during a trial.
5. Save a photograph of the complete setup as `setup.jpg`.
6. Create `platform.md` with the end-effector, control interface or policy, mounting orientation, control mode, and relevant calibration notes.
7. Record the printer, material, print settings, and post-processing. The runner includes these fields in `platform.md`.

## Familiarization

Run unrecorded practice trials for each attachment until completion times stabilize. Begin recording after three consecutive practice trials finish within 20% of each other or within the timeout, whichever comes first.

Practice trials are not included in the submitted result.

## Recorded trials

For each platform and attachment pair:

1. Run **five recorded trials**.
2. Start the end-effector from the same neutral pose.
3. Start the external camera before the countdown. When using the runner, command the first task motion when the countdown reaches zero; this is when timing starts. For manual logging, start timing with the first commanded task motion.
4. Stop timing when the success criterion is met, a safety event occurs, or the timeout expires.
5. Reset the mechanism and neutral pose before the next trial.

Run the ball valve as two separate blocks: five trials without a friction ring and five with the ring fitted.

Do not tune parameters or change the control mode between recorded trials in the same condition. A restarted approach inside a trial is counted as another attempt or regrasp; it is not a new trial.

## Outcomes

Use exactly one of these values:

| Outcome | Meaning |
|---|---|
| `success` | The full success criterion was completed within the timeout |
| `fail` | The trial ended unsuccessfully before the timeout |
| `timeout` | The success criterion was not completed before the limit |
| `safety_stop` | The trial ended because of a safety event |

Only successful trials receive a `completion_time_s` value.

## Failure causes

Every unsuccessful trial must contain one dominant failure cause.

| Cause | Use when |
|---|---|
| `grasp_geometry` | The end-effector cannot engage the part in a stable grasp |
| `kinematic_limit` | The platform cannot produce the motion required by the task |
| `perception` | Alignment or state could not be judged reliably |
| `slip` | A grasp was acquired and then lost |
| `force_limit` | Available grasp or actuation force was insufficient |
| `control_precision` | Control resolution was too coarse for the task |
| `other` | None of the listed causes fits; explain in `notes` |

Choose one primary cause even when several factors contributed.

## Broken or displaced parts

If a part breaks or leaves its cell:

1. stop the trial if continuation is unsafe;
2. record the trial as it occurred;
3. describe the event in `notes`;
4. reseat or replace the part; and
5. continue with the next scheduled trial.

Do not silently replace the affected row with a rerun.

## Reproducibility checklist

- [ ] The HiveBoard release or commit is recorded.
- [ ] Printing parameters and post-processing are recorded.
- [ ] All attachments are fully seated.
- [ ] Familiarization is complete for every attachment.
- [ ] Five recorded trials exist for all 13 conditions (65 trials total).
- [ ] Timeouts and task criteria match the module reference.
- [ ] Every unsuccessful trial has one failure cause.
- [ ] Attempts, regrasps, and strategy are present on every row.
- [ ] Composed tasks include `stage_reached`.
- [ ] Broken or reseated parts are described.
- [ ] `setup.jpg` and `platform.md` accompany the log.
- [ ] One external-camera recording accompanies each trial, using the filenames in `recording-instructions.md`.
- [ ] No operator names or personal identifiers appear in the files.
