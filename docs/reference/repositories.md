# Repository map

HiveBoard maintains separate repositories for benchmark assets, simulation, policy learning, data collection, and the website.

| Repository | Role | Use it for |
|---|---|---|
| [`hiveboard-bench/HiveBoard`](https://github.com/hiveboard-bench/HiveBoard) | Benchmark assets | STL, CAD, URDF/USD, protocol, and trial templates |
| [`hiveboard-bench/isaaclab-hiveboard`](https://github.com/hiveboard-bench/isaaclab-hiveboard) | Isaac Lab implementation | Environments, task configuration, scripted controllers, and simulation recording |
| [`EESC-LabRoM/hiveboard-rl`](https://github.com/EESC-LabRoM/hiveboard-rl) | Policy learning | Reinforcement and imitation learning using a pinned Isaac Lab integration submodule |
| [`hiveboard-bench/DataHive`](https://github.com/hiveboard-bench/DataHive) | Data collection toolkit | Recording integration, local Runner, episode annotation and validation, and Hugging Face upload; start with the [DataHive guide](/guides/datahive) |
| [`hiveboard-bench/fr3_datahive`](https://github.com/hiveboard-bench/fr3_datahive) | Reference integration | Franka Research 3 example integration and Docker environment for DataHive |
| [`hiveboard-bench/hiveboard-bench.github.io`](https://github.com/hiveboard-bench/hiveboard-bench.github.io) | Project website and documentation | Paper presentation, videos, results, interactive viewer, and `docs/` with the documentation and Evaluation Runner |

The earlier [`ricardovgodoy/hiveboard-docs`](https://github.com/ricardovgodoy/hiveboard-docs) prototype has been superseded by `docs/` in the project website repository.

## Source files

To avoid conflicting instructions:

- physical geometry and the trial protocol belong to the main HiveBoard repository;
- executable Isaac Lab details belong to the Isaac Lab repository;
- policy training, expert banks, and learned-policy evaluation belong to `hiveboard-rl`;
- DataHive's episode format, recording API, and collection tools belong to the DataHive repository;
- the Franka Research 3 reference integration belongs to the `fr3_datahive` repository;
- the project website presents the research; and
- this repository contains the documentation and evaluation runner.

The documentation links to the corresponding source files in each repository.

## Version an experiment

Record a release tag when available. Otherwise record a full commit hash for every repository used. A complete experiment identifier may therefore include:

```text
HiveBoard assets: <release or commit>
Isaac Lab integration: <release or commit>
HiveBoard RL (if used): <release or commit, plus core submodule commit>
DataHive (if used): <package version or commit>
Documentation: <release or commit>
```

Avoid reporting only `main`, since its content can change after the experiment.
