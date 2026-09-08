# Repository map

HiveBoard currently separates presentation, physical assets, and the Isaac Lab implementation.

| Repository | Role | Use it for |
|---|---|---|
| [`EESC-LabRoM/HiveBoard`](https://github.com/EESC-LabRoM/HiveBoard) | Benchmark assets | STL, CAD, URDF/USD, protocol, and trial templates |
| [`EESC-LabRoM/isaaclab-hiveboard`](https://github.com/EESC-LabRoM/isaaclab-hiveboard) | Isaac Lab implementation | Environments, task configuration, training, and simulation evaluation |
| [`hiveboard-bench/hiveboard-bench.github.io`](https://github.com/hiveboard-bench/hiveboard-bench.github.io) | Project website | Paper presentation, videos, results, and interactive viewer |
| [`ricardovgodoy/hiveboard-docs`](https://github.com/ricardovgodoy/hiveboard-docs) | Documentation prototype | Review and refinement before migration to the official website |

## Source files

To avoid conflicting instructions:

- physical geometry and the trial protocol belong to the main HiveBoard repository;
- executable Isaac Lab details belong to the Isaac Lab repository;
- the project website presents the research; and
- this repository contains the documentation and evaluation runner.

The documentation links to the corresponding source files in each repository.

## Version an experiment

Record a release tag when available. Otherwise record a full commit hash for every repository used. A complete experiment identifier may therefore include:

```text
HiveBoard assets: <release or commit>
Isaac Lab integration: <release or commit>
Documentation: <release or commit>
```

Avoid reporting only `main`, since its content can change after the experiment.
