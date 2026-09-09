# Record trials

Use the supplied spreadsheet during experiments and the CSV for scripts or data exchange.

The [Evaluation Runner](/benchmark/evaluation-runner) provides a guided timer and produces the same CSV format directly in the browser. It can also save transferable sessions, validate a complete evaluation, and generate the final submission package.

<ul class="doc-links">
  <li><a href="https://github.com/EESC-LabRoM/HiveBoard/raw/main/Documentation/trials.xlsx"><strong>Download the XLSX template</strong></a> — for manual entry, with all trial rows pre-populated.</li>
  <li><a href="https://github.com/EESC-LabRoM/HiveBoard/raw/main/Documentation/trials.csv"><strong>Download the CSV template</strong></a> — for scripts, version control, and automated validation.</li>
</ul>

Do not change `trial_id` or `attachment_id`. Fill the remaining columns while running the trials.

## Column reference

| Column | Required entry |
|---|---|
| `lab_id` | Stable lowercase identifier for the laboratory |
| `platform_id` | End-effector and control-interface combination, using lowercase and underscores |
| `date` | `YYYY-MM-DD` |
| `outcome` | `success`, `fail`, `timeout`, or `safety_stop` |
| `failure_cause` | Required when outcome is not `success`; blank otherwise |
| `completion_time_s` | Decimal seconds for successful trials only |
| `n_attempts` | Discrete task attempts, counted from 1 |
| `n_regrasps` | Release-and-grasp events, counted from 0 |
| `stage_reached` | Last completed stage, only for composed tasks |
| `strategy` | `prehensile` or `non_prehensile` |
| `notes` | Deviations, adjustments, damage, or other concise observations |

## Counting conventions

### Attempts

An attempt ends when the current approach is abandoned and a new approach begins. Corrective motion within one continuous approach does not create another attempt. Every trial has at least one attempt.

### Regrasps

Count each time the end-effector releases a part and closes on it again. A trial with one grasp and no release records `0`.

### Strategy

Use `prehensile` when the end-effector closes around and holds the part. Use `non_prehensile` for pushing, pressing, or engaging a feature without an enclosing grasp.

### Stages

Enter the number of the last completed stage, not the stage where the attempt stopped. Enter `0` if stage 1 was not completed.

## Valid example

```csv
trial_id,lab_id,platform_id,attachment_id,date,outcome,failure_cause,completion_time_s,n_attempts,n_regrasps,stage_reached,strategy,notes
1,usp_crob,franka_2f85,valve_gate_small,2026-04-26,success,,34.2,1,0,,prehensile,
2,usp_crob,franka_2f85,valve_gate_small,2026-04-26,fail,slip,,3,2,,prehensile,handle slipped
56,usp_crob,franka_2f85,lock,2026-04-26,success,,142.3,2,1,3,prehensile,
```

## Common mistakes

| Avoid | Use instead |
|---|---|
| Milliseconds or values with `s` appended | Decimal seconds as a plain number |
| `timeout` written in the time column | Set outcome to `timeout` and leave time blank |
| Regrasps counted from 1 | Count release-and-grasp events from 0 |
| Blank counts on failed trials | Record attempts, regrasps, and strategy on every row |
| Blank stage on a failed composed task | Record the last completed stage, including `0` |
| Replacing a damaged-part trial with a clean rerun | Preserve the trial and explain the event in `notes` |

See the [column-by-column instructions](https://github.com/EESC-LabRoM/HiveBoard/blob/main/Documentation/HOW_TO_FILL_TRIALS.md) for the full reference.
