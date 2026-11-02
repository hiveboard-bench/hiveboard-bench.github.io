---
title: How to perform each task
description: Starting states, actions, success criteria, timeouts, and reset instructions for all 13 HiveBoard evaluation conditions.
---

# How to perform each task

Select a task below for its starting state, required actions, success criterion, and reset. Record **five trials for each of the 13 conditions (65 trials total)**. The two ball-valve configurations are separate conditions.

| Task | What to do | Timeout |
|---|---|---:|
| [Ball valve](#ball-valve) | Rotate the handle 90° from closed to open | 60 s |
| [Ball valve + ring](#ball-valve-with-friction-ring) | Perform the same rotation with the ring fitted | 90 s |
| [Gate valve (small)](#small-gate-valve) | Rotate the stem one full turn from closed | 90 s |
| [Gate valve (large)](#large-gate-valve) | Rotate the stem one full turn from closed | 120 s |
| [Circuit breaker](#circuit-breaker) | Move the toggle to the opposite state and hold it | 60 s |
| [Light bulb](#light-bulb-and-socket) | Thread the bulb into the socket until seated | 120 s |
| [Thread (M8)](#m8-threaded-fastener) | Align the nut with the screw and thread it along the available length | 120 s |
| [Thread (M30)](#m30-threaded-fastener) | Align the nut with the screw and thread it along the available length | 120 s |
| [Peg insertion](#threaded-peg-insertion) | Align and thread the free peg into the socket until seated | 120 s |
| [Button](#covered-button) | Open the cover, then press the button until actuation | 60 s |
| [Lock and key](#lock-and-key) | Grasp the key, insert it vertically, and rotate to unlock | 180 s |
| [Drawer](#sliding-drawer) | Grasp the drawer, align it with its housing, and insert it fully | 120 s |
| [Shock absorber](#shock-absorber) | Grasp the loose part, align its hole with the pin, and insert the part fully onto the pin | 180 s |

Task names follow Table 2 of the paper. See [Terminology](/reference/terminology) to match them to trial IDs and older file names.

## Before every trial

Fully seat the attachment and restore the starting state described below. For loose parts, document their starting pose and use the same pose between trials. For Thread (M8) and Thread (M30), start with the nut completely removed from the fixed screw. Alignment and initial thread engagement are part of the task.

Start the end-effector from the same neutral pose for the platform. Record the complete physical trial with an external camera. Start timing with the first commanded task motion; when using the [Evaluation Runner](/benchmark/evaluation-runner), begin that motion when the countdown reaches zero. Stop at success, a safety stop, or the timeout.

The timed sequence includes the initial approach and grasp. For the key, drawer, and shock absorber, grasping remains stage 1. For the bulb, nuts, and peg, picking up the loose part is also included in the trial time.

For familiarization, outcome labels, and handling broken or displaced parts, follow the [evaluation protocol](/benchmark/protocol). Example videos remain available in the runner.

## Ball valve

Task ID: `valve_ball`

| Step | Instruction |
|---|---|
| Starting state | Handle closed; no friction ring fitted; attachment fully seated. |
| Action | Engage the handle and rotate it 90° toward the open position. |
| Success | The handle reaches the open position after a 90° rotation. |
| Timeout | 60 s. |
| Reset | Return the handle to closed and check attachment seating. |

The [360° variant](/hardware/ball-valve-360) allows overshoot without reaching a quarter-turn stop during development. The task target remains 90°. Record which variant is used and mark the initial and target orientations.

## Ball valve + ring {#ball-valve-with-friction-ring}

Task ID: `valve_ball_ring`

| Step | Instruction |
|---|---|
| Starting state | Friction ring fitted; handle closed; attachment fully seated. Document which ring is used. |
| Action | Engage the handle and rotate it 90° toward the open position with the ring fitted. |
| Success | The handle reaches the open position after a 90° rotation. |
| Timeout | 90 s. |
| Reset | Keep the same ring fitted, return the handle to closed, and check attachment seating. |

## Gate valve (small) {#small-gate-valve}

Task ID: `valve_gate_small`

| Step | Instruction |
|---|---|
| Starting state | Stem in the closed starting position. Identify its initial orientation so one full turn can be observed. |
| Action | Turn the valve to rotate the stem through one complete revolution from closed. |
| Success | The stem completes one full turn (360°). Full opening travel is not required. |
| Timeout | 90 s. |
| Reset | Reverse the motion to restore the closed starting position and initial orientation. Keep the board position unchanged. |

## Gate valve (large) {#large-gate-valve}

Task ID: `valve_gate_large`

| Step | Instruction |
|---|---|
| Starting state | Stem in the closed starting position. Identify its initial orientation so one full turn can be observed. |
| Action | Turn the valve to rotate the stem through one complete revolution from closed. |
| Success | The stem completes one full turn (360°). Full opening travel is not required. |
| Timeout | 120 s. |
| Reset | Reverse the motion to restore the closed starting position and initial orientation. Keep the board position unchanged. |

## Circuit breaker

Task ID: `circuit_breaker`

| Step | Instruction |
|---|---|
| Starting state | Toggle in the documented initial state. |
| Action | Move the toggle to the opposite state and hold it there. |
| Success | The toggle has changed state and is held in that state. |
| Timeout | 60 s. |
| Reset | Return the toggle to the initial state and check that it moves freely. |

## Light bulb {#light-bulb-and-socket}

Task ID: `light_bulb`

| Step | Instruction |
|---|---|
| Starting state | Bulb removed from the socket and placed in its documented starting pose; thread clear. |
| Action | Align the bulb with the socket and rotate it to engage and advance the thread. |
| Success | The bulb is fully threaded until seated. |
| Timeout | 120 s. |
| Reset | Unscrew and remove the bulb, restore its starting pose, and inspect the thread. |

## Thread (M8) {#m8-threaded-fastener}

Task ID: `thread_m8`

| Step | Instruction |
|---|---|
| Starting state | Nut completely removed from the fixed screw, with no thread engagement. Document the nut's starting pose and keep it consistent across trials. |
| Action | Align the nut with the screw, engage the thread, and rotate the nut along the available length. |
| Success | The nut is fully threaded along the available length of the screw. |
| Timeout | 120 s. |
| Reset | Fully unthread and remove the nut, restore its starting pose, and inspect both threads. |

## Thread (M30) {#m30-threaded-fastener}

Task ID: `thread_m30`

| Step | Instruction |
|---|---|
| Starting state | Nut completely removed from the fixed screw, with no thread engagement. Document the nut's starting pose and keep it consistent across trials. |
| Action | Align the nut with the screw, engage the thread, and rotate the nut along the available length. |
| Success | The nut is fully threaded along the available length of the screw. |
| Timeout | 120 s. |
| Reset | Fully unthread and remove the nut, restore its starting pose, and inspect both threads. |

The runner's M8 and M30 images show the assembled mechanism, with the nut on the screw. These images do not define the required starting state. Begin each new trial with the nut completely clear of the screw. Identify earlier trials that began with the nut engaged when reporting results.

## Peg insertion {#threaded-peg-insertion}

Task ID: `peg_insertion`

| Step | Instruction |
|---|---|
| Starting state | Socket empty; free 8 mm threaded peg in its documented pose next to the socket. |
| Action | Pick up the peg, align it with the socket, and rotate it to engage and advance the thread. |
| Success | The peg is threaded into the socket until seated. Simply placing it in the opening is insufficient. |
| Timeout | 120 s. |
| Reset | Unscrew and remove the peg, leave the socket empty, and restore the peg's starting pose. |

## Button {#covered-button}

Task ID: `button`

| Step | Instruction |
|---|---|
| Starting state | Cover closed; button returned to its unpressed state. |
| Stage 1 | Open the cover to expose the button. |
| Stage 2 | Press the button until actuation. |
| Success | Both stages complete within the timeout. |
| Timeout | 60 s for the complete sequence. |
| Reset | Release the button, confirm that it returns, and close the cover. |

Record `stage_reached = 0` if no stage completes, `1` if only the cover is opened, or `2` when the button is also actuated.

## Lock and key

Task ID: `lock`

| Step | Instruction |
|---|---|
| Starting state | Lock in its locked initial state; key removed and placed in its documented starting pose. |
| Stage 1 | Grasp the key. |
| Stage 2 | Insert the key vertically into the lock. |
| Stage 3 | Rotate the key to unlock. |
| Success | The lock is unlocked after all three stages. |
| Timeout | 180 s for the complete sequence. |
| Reset | Restore the lock's initial state, remove the key, and return the key to its starting pose. |

Record the last completed stage: `0`, `1`, `2`, or `3`. Inserting the key without unlocking is stage `2`, not full success.

## Drawer {#sliding-drawer}

Task ID: `drawer`

| Step | Instruction |
|---|---|
| Starting state | Drawer completely removed from its housing and placed in its documented starting pose. Housing fixed to the board and empty. |
| Stage 1 | Grasp the drawer. |
| Stage 2 | Align the drawer with the housing opening. |
| Stage 3 | Insert the drawer fully into the housing. |
| Success | The drawer is correctly seated in the fully inserted position. |
| Timeout | 120 s, including the initial approach and grasp. |
| Reset | Remove the drawer completely from its housing and restore its documented starting pose. |

Record the last completed stage: `0`, `1`, `2`, or `3`. Record `2` if the drawer is aligned but only partly inserted. Full success requires complete insertion. The task ends with the drawer inserted.

Identify earlier trials that used an open-and-close sequence in `notes` when reporting results.

## Shock absorber

Task ID: `shock_absorber`

**Grasp the loose part of the shock absorber and insert it onto the pin.** The loose part moves while the pin stays in place. The complete sequence has three stages and a single 180 s timeout.

| Step | Instruction |
|---|---|
| Starting state | Loose part removed from the pin and placed in its documented starting pose. Leave the pin in place and check that the attachment is seated in both occupied board cells. |
| Stage 1 — grasp | Approach and grasp the loose part of the shock absorber with the end-effector. |
| Stage 2 — align | Align the hole in the grasped part with the pin. |
| Stage 3 — insert | Move the aligned part onto the pin until it is fully inserted. |
| Success | All three stages are complete, with the loose part fully inserted onto the pin. |
| Timeout | 180 s total, beginning with the first commanded task motion. Do not restart the timer between stages. |
| Reset | Remove the loose part from the pin and return it to its starting pose. Leave the pin in place and check attachment seating in both board cells. Restore the end-effector's neutral pose before the next trial. |

Record the last **completed** stage:

| `stage_reached` | What was completed |
|---|---|
| `0` | The loose part was not grasped. |
| `1` | The loose part was grasped, but alignment with the pin was not completed. |
| `2` | The hole in the part was aligned with the pin, but the part was not fully inserted onto it. This includes partial insertion. |
| `3` | The part was fully inserted onto the pin. |

For example, if the part is aligned but only partly inserted onto the pin when 180 s expires, record `outcome = timeout`, `stage_reached = 2`, and leave `completion_time_s` blank. Also record attempts, regrasps, strategy, and the dominant failure cause. See [trial logging](/benchmark/logging) for the field definitions.

See the [source protocol](https://github.com/hiveboard-bench/HiveBoard/blob/main/Documentation/PROTOCOL.md#5-per-attachment-success-criteria-and-timeouts) for success criteria and timeouts. The protocol does not specify a numeric insertion depth, holding duration, or loose-part pose. Document the starting pose and use the same setup for every trial.
