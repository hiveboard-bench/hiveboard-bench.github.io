# Module reference

HiveBoard contains 13 scored attachment configurations across three task families. The ball valve is evaluated both without and with a friction ring, producing two separate trial sets.

<figure class="doc-image">
  <img src="/images/attachments-overview.png" alt="Overview of the thirteen HiveBoard attachment configurations">
  <figcaption>Attachment overview. Each mechanism uses the shared honeycomb mounting interface.</figcaption>
</figure>

## Torque tasks

| ID | Attachment | Required outcome | Timeout |
|---|---|---|---:|
| `valve_ball` | Ball valve | Rotate the handle 90° from closed to open | 60 s |
| `valve_ball_ring` | Ball valve with friction ring | Rotate the handle 90° with the ring fitted | 90 s |
| `valve_gate_small` | Small gate valve | Complete one full turn of the stem | 90 s |
| `valve_gate_large` | Large gate valve | Complete one full turn of the stem | 120 s |
| `circuit_breaker` | Circuit breaker | Move the toggle to the other state and hold it | 60 s |

The gate-valve criterion is one complete stem rotation, not full travel. The number of turns required for full travel can vary with the printed thread pitch.

## Precision tasks

| ID | Attachment | Required outcome | Timeout |
|---|---|---|---:|
| `light_bulb` | Light bulb and socket | Thread the bulb until it is seated | 120 s |
| `thread_m8` | M8 threaded fastener | Thread the bolt along the available length | 120 s |
| `thread_m30` | M30 threaded fastener | Thread the bolt along the available length | 120 s |
| `peg_insertion` | Threaded peg insertion | Thread the free peg into the empty socket until seated | 120 s |

The peg task uses a threaded 8 mm pin. It is not a clearance-fit insertion task.

## Composed assembly tasks

These tasks are scored by the last completed stage as well as by full success.

| ID | Attachment | Stage 1 | Stage 2 | Stage 3 | Timeout |
|---|---|---|---|---|---:|
| `button` | Covered button | Open cover | Press button | — | 60 s |
| `lock` | Lock and key | Grasp key | Insert key vertically | Rotate to unlock | 180 s |
| `drawer` | Sliding drawer | Grasp handle | Pull open | Push closed | 120 s |
| `shock_absorber` | Shock absorber | Grasp pin | Align with hole | Insert fully | 180 s |

Record `0` when no stage is completed. A full success reaches the final stage listed for the attachment.

## Printable files

The module folders and their components are available under [`STL/`](https://github.com/EESC-LabRoM/HiveBoard/tree/main/STL). The same high-level folder names are used under `CAD/` and `Simulation/` where the corresponding assets are available.
