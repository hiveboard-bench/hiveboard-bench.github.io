# Print the parts

The printable files are stored in the [`STL/` directory](https://github.com/EESC-LabRoM/HiveBoard/tree/main/STL). Editable sources are available separately in [`CAD/`](https://github.com/EESC-LabRoM/HiveBoard/tree/main/CAD).

::: tip Check the fit first
Before printing the full set, print one honeycomb cell and one attachment base. Verify that the attachment seats fully, remains stable during manipulation, and can be removed without damaging either part.
:::

## Printer requirements

| Item | Recommendation |
|---|---|
| Process | Consumer-grade FDM |
| Material | PLA, 1.75 mm |
| Minimum build area | 300 × 300 mm for the complete base |
| Nozzle | 0.4 mm |
| Layer height | 0.20 mm |
| Wall count | 4 |
| Top / bottom layers | 5 / 5 |
| Print speed | 50 mm/s |
| Nozzle temperature | 200–220 °C |
| Bed temperature | 50–60 °C |
| Cooling | 100% |
| Adhesion | Skirt or brim |
| Supports | Only where required |

Smaller variants of the honeycomb panel are included in the repository, but the full benchmark geometry is designed around a 300 × 300 mm build area.

## Infill by part type

| Part type | Infill |
|---|---:|
| Base structure | 20% |
| Mechanical parts | 25% |
| Torque components | 30% |
| Threads | 25% |
| Decorative covers | 15% |

## Orientation

| Component | Orientation |
|---|---|
| Threads and screws | Vertical |
| Nuts | Flat |
| Valves | Handle up |
| Drawer | Largest face on the bed |
| Pegs | Vertical |
| Shock absorber parts | Sideways |

## Print order

For a new printer or filament batch, use this order:

1. Print one honeycomb cell and one simple module base.
2. Check the press fit and dimensional consistency.
3. Print the M8 and M30 test pieces before the complete threaded modules.
4. Print the base panels.
5. Print the remaining functional mechanisms.

This order catches tolerance and thread problems before the longest prints.

## Post-processing

- Remove stringing and support material without rounding functional edges.
- Check that mating faces seat fully.
- Test threads by hand before mounting the module.
- Apply light sanding only when printer calibration produces tight surfaces.
- Record any sanding or dimensional adjustment in the trial notes.

Do not lubricate or modify friction surfaces without reporting the change. These changes can alter the task difficulty.

## Files to download

| Group | Repository folder |
|---|---|
| Honeycomb base | [`STL/Honeycomb`](https://github.com/EESC-LabRoM/HiveBoard/tree/main/STL/Honeycomb) |
| Valves | [`STL/Valves`](https://github.com/EESC-LabRoM/HiveBoard/tree/main/STL/Valves) |
| Threads | [`STL/Threads`](https://github.com/EESC-LabRoM/HiveBoard/tree/main/STL/Threads) |
| Other mechanisms | [`STL/`](https://github.com/EESC-LabRoM/HiveBoard/tree/main/STL) |

The repository README also links to the current module-specific printing guide.
