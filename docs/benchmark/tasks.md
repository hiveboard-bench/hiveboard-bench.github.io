---
title: How to perform each task
description: Starting states, actions, success criteria, timeouts, and reset instructions for all 13 HiveBoard evaluation conditions.
---

# How to perform each task

Select a task below for its starting state, required actions, success criterion, and reset. Record **five trials for each of the 13 conditions (65 trials total)**. The two ball-valve configurations are separate conditions.

| Task | What to do | Timeout |
|---|---|---:|
| [Ball valve](#ball-valve) | Rotate the handle 90° from closed to open | 60 s |
| [Ball valve with friction ring](#ball-valve-with-friction-ring) | Perform the same rotation with the ring fitted | 90 s |
| [Small gate valve](#small-gate-valve) | Rotate the stem one full turn from closed | 90 s |
| [Large gate valve](#large-gate-valve) | Rotate the stem one full turn from closed | 120 s |
| [Circuit breaker](#circuit-breaker) | Move the toggle to the opposite state and hold it | 60 s |
| [Light bulb and socket](#light-bulb-and-socket) | Thread the bulb into the socket until seated | 120 s |
| [M8 threaded fastener](#m8-threaded-fastener) | Thread the bolt along the available length | 120 s |
| [M30 threaded fastener](#m30-threaded-fastener) | Thread the bolt along the available length | 120 s |
| [Threaded peg insertion](#threaded-peg-insertion) | Align and thread the free peg into the socket until seated | 120 s |
| [Covered button](#covered-button) | Open the cover, then press the button until actuation | 60 s |
| [Lock and key](#lock-and-key) | Grasp the key, insert it vertically, and rotate to unlock | 180 s |
| [Sliding drawer](#sliding-drawer) | Grasp the handle, pull open, then push closed | 120 s |
| [Shock absorber](#shock-absorber) | Grasp the pin, align it with the hole, and insert it fully | 180 s |

## Before every trial

Fully seat the attachment and restore the starting state described below. For loose parts, document their starting pose and use the same pose between trials. Where initial thread engagement is part of the setup, document and restore that engagement; this guide does not introduce a new number of starting turns.

Start the end-effector from the same neutral pose for the platform. Record the complete physical trial with an external camera. Start timing with the first commanded task motion; when using the [Evaluation Runner](/benchmark/evaluation-runner), begin that motion when the countdown reaches zero. Stop at success, a safety stop, or the timeout.

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

## Ball valve with friction ring

Task ID: `valve_ball_ring`

| Step | Instruction |
|---|---|
| Starting state | Friction ring fitted; handle closed; attachment fully seated. Document which ring is used. |
| Action | Engage the handle and rotate it 90° toward the open position with the ring fitted. |
| Success | The handle reaches the open position after a 90° rotation. |
| Timeout | 90 s. |
| Reset | Keep the same ring fitted, return the handle to closed, and check attachment seating. |

## Small gate valve

Task ID: `valve_gate_small`

| Step | Instruction |
|---|---|
| Starting state | Stem in the closed starting position. Identify its initial orientation so one full turn can be observed. |
| Action | Turn the valve to rotate the stem through one complete revolution from closed. |
| Success | The stem completes one full turn (360°). Full opening travel is not required. |
| Timeout | 90 s. |
| Reset | Reverse the motion to restore the closed starting position and initial orientation. Keep the board position unchanged. |

## Large gate valve

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

## Light bulb and socket

Task ID: `light_bulb`

| Step | Instruction |
|---|---|
| Starting state | Bulb removed from the socket and placed in its documented starting pose; thread clear. |
| Action | Align the bulb with the socket and rotate it to engage and advance the thread. |
| Success | The bulb is fully threaded until seated. |
| Timeout | 120 s. |
| Reset | Unscrew and remove the bulb, restore its starting pose, and inspect the thread. |

## M8 threaded fastener

Task ID: `thread_m8`

| Step | Instruction |
|---|---|
| Starting state | Bolt at the documented initial thread engagement; thread clear. |
| Action | Rotate the bolt to advance it along the thread. |
| Success | The bolt is fully threaded along the available length. |
| Timeout | 120 s. |
| Reset | Return the bolt to its initial engagement and inspect the thread. |

## M30 threaded fastener

Task ID: `thread_m30`

| Step | Instruction |
|---|---|
| Starting state | Bolt at the documented initial thread engagement; thread clear. |
| Action | Rotate the bolt to advance it along the thread. |
| Success | The bolt is fully threaded along the available length. |
| Timeout | 120 s. |
| Reset | Return the bolt to its initial engagement and inspect the thread. |

## Threaded peg insertion

Task ID: `peg_insertion`

| Step | Instruction |
|---|---|
| Starting state | Socket empty; free 8 mm threaded peg in its documented pose next to the socket. |
| Action | Pick up the peg, align it with the socket, and rotate it to engage and advance the thread. |
| Success | The peg is threaded into the socket until seated. Simply placing it in the opening is insufficient. |
| Timeout | 120 s. |
| Reset | Unscrew and remove the peg, leave the socket empty, and restore the peg's starting pose. |

## Covered button

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

## Sliding drawer

Task ID: `drawer`

| Step | Instruction |
|---|---|
| Starting state | Drawer fully closed. |
| Stage 1 | Grasp the handle. |
| Stage 2 | Pull the drawer open. |
| Stage 3 | Push the drawer closed again. |
| Success | The drawer has been opened and then closed after grasping the handle. |
| Timeout | 120 s for the complete sequence. |
| Reset | Return the drawer to the fully closed position. |

Record the last completed stage: `0`, `1`, `2`, or `3`. Opening the drawer without closing it is stage `2`, not full success.

## Shock absorber

Task ID: `shock_absorber`

**The task is to pick up the loose pin and insert it fully into the shock absorber's hole.** The complete sequence has three stages and a single 180 s timeout.

| Step | Instruction |
|---|---|
| Starting state | Pin removed from the hole and placed in its documented starting pose. Check that the attachment is seated in both occupied board cells. |
| Stage 1 — grasp | Approach and grasp the loose pin with the end-effector. |
| Stage 2 — align | Bring the pin to the hole and align its insertion axis with the hole. |
| Stage 3 — insert | Advance the aligned pin into the hole until it is fully inserted. |
| Success | All three stages are complete, with the pin fully inserted. |
| Timeout | 180 s total, beginning with the first commanded task motion. Do not restart the timer between stages. |
| Reset | Remove the pin, return it to its starting pose, and check attachment seating in both board cells. Restore the end-effector's neutral pose before the next trial. |

Record the last **completed** stage:

| `stage_reached` | What was completed |
|---|---|
| `0` | The pin was not grasped. |
| `1` | The pin was grasped, but alignment was not completed. |
| `2` | The pin was aligned with the hole, but full insertion was not completed. This includes a partially inserted pin. |
| `3` | The pin was fully inserted. |

For example, if the pin is aligned but only partly inserted when 180 s expires, record `outcome = timeout`, `stage_reached = 2`, and leave `completion_time_s` blank. Also record attempts, regrasps, strategy, and the primary failure cause. See [trial logging](/benchmark/logging) for the field definitions.

The task definitions and timeouts follow the [source protocol](https://github.com/EESC-LabRoM/HiveBoard/blob/main/Documentation/PROTOCOL.md#5-per-attachment-success-criteria-and-timeouts); reset instructions follow the Evaluation Runner. The protocol does not specify a numeric insertion depth, holding duration, or loose-part pose. Document the physical setup rather than introducing a different threshold for each trial.
