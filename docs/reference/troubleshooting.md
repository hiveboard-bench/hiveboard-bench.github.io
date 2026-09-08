# Troubleshooting

## An attachment does not fit

Print one cell and one attachment base using the same material and profile. Check for elephant's foot, warping, or excess material before modifying the CAD. If sanding is required, apply it consistently and record it.

## A threaded component binds

Remove stringing and debris, then test the matching calibration piece. Keep the part aligned with the thread axis and do not force initial engagement. Record sanding or lubrication because either can change task resistance.

## A module moves during a trial

Stop if continuation is unsafe. Preserve the trial result, describe the displacement in `notes`, and reseat the module before continuing. For repeated release, verify cell tolerances and mounting orientation.

## A printed part breaks

Record the trial as it occurred. Photograph the damage when useful, replace the component with the same print profile, and note the replacement. Do not substitute an unreported rerun.

## Completion time appears as a date

Format the spreadsheet column as a plain number and enter decimal seconds without units. Use `12.9`, not `12.9 s` or a clock value.

## Failed trials have no time

This is expected. `completion_time_s` is only populated for successful trials. Use `outcome=timeout` for a trial that reaches the time limit.

## Simulation geometry loads at the wrong scale

Verify the asset units and the loader's unit conversion. Check both visual and collision geometry before changing the global scene scale.

## A simulation joint moves in the wrong direction

Inspect the joint axis, parent-child order, and initial pose. Compare the full motion with the physical mechanism, not only the rendered rest configuration.

## Simulation contact is unstable

Begin with one environment and a conservative timestep. Inspect collision geometry, penetration at reset, solver settings, drive gains, and mass ratios. Treat the supplied physical properties as nominal rather than identified values.

## An Isaac Lab asset cannot be found

The simulation repository uses Git submodules. From the repository root, initialize or update them with:

```bash
git submodule update --init --recursive
```

The `dependencies/HiveBoard`, `dependencies/curobo`, and `dependencies/duatic_dynaarm` directories should then contain their respective repositories.

## An Isaac Lab task is not registered

Run the environment-listing script inside the project environment:

```bash
uv run python scripts/list_envs.py
```

Use the exact `Isaac-HiveBoard-*` identifier printed by the script. If imports fail, run `uv sync` from the repository root and continue using `uv run` for subsequent commands.
