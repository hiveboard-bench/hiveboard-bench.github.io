# HiveBoard

HiveBoard is an open, modular, fully 3D-printable benchmark for manipulation of functional mechanisms. Its interchangeable attachments cover rotational, threaded, insertion, linear, and multi-stage tasks. The same board and protocol can be used with robot grippers, dexterous hands, teleoperated systems, and worn prosthetic hands.

For the paper, demonstration videos, and reported results, see the [HiveBoard project website](https://hiveboard-bench.github.io).

## Documentation

- [Benchmark overview](/getting-started/overview)
- [Getting started](/getting-started/quick-start)
- [3D printing](/hardware/printing)
- [Assembly and mounting](/hardware/assembly)
- [Module reference](/hardware/modules)
- [Evaluation Runner](/benchmark/evaluation-runner)
- [Evaluation protocol](/benchmark/protocol)
- [Simulation assets](/simulation/assets)

The printable parts, CAD models, articulated assets, protocol, and trial templates are maintained in the [HiveBoard repository](https://github.com/EESC-LabRoM/HiveBoard). The Isaac Lab environments are maintained in a [separate repository](https://github.com/EESC-LabRoM/isaaclab-hiveboard).

## Open calls for contributions

We invite laboratories and individual researchers to [contribute datasets for learning-based manipulation](/contribute/evaluations), including robot states, actions, camera observations, timestamps, calibration, and task outcomes. There is no fixed episode count; larger collections of demonstrations and rollouts are encouraged.

We welcome functional assets for [activities of daily living](/contribute/adl) and [bimanual manipulation](/contribute/bimanual). Contact the authors before starting a design to check its suitability and avoid duplicating work.

Contributors who deliver at least one accepted, functional asset with the complete files and validation evidence may be eligible for authorship on future papers that use the contribution. Smaller contributions may be acknowledged. See the [requirements and credit policy](/contribute/requirements).

## Citation

If you use HiveBoard in your research, please cite the project paper:

```bibtex
@article{hiveboard2026,
  title   = {HiveBoard: An Open, Modular, 3D-Printed Benchmark of Industrial
             Mechanisms for Robotic and Prosthetic Manipulation},
  author  = {Godoy, Ricardo V. and de Souza, Enzo F. and de Lange, Rudy De-Xin and
             Negri, Juliano and Marsicano, Joao A. and van Halst, Victor and others},
  journal = {Under review},
  year    = {2026},
  url     = {https://github.com/EESC-LabRoM/HiveBoard}
}
```

The citation metadata may change while the paper is under review. See the [citation page](/reference/citation) before preparing a publication.
