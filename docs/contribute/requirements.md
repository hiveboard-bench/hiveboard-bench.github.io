---
description: Required files, validation evidence, and contributor credit for the HiveBoard asset calls.
---

# Contribution requirements and credit

These requirements apply to the [ADL](/contribute/adl) and [bimanual manipulation](/contribute/bimanual) calls. Discuss the proposal with the authors before starting detailed design. Use the [attachment guide](/guides/new-attachment) for technical integration.

## Required files and evidence

A complete asset contribution must let another laboratory fabricate, assemble, simulate, and operate the mechanism.

| Component | Required material |
|---|---|
| Editable design | Native CAD source and a neutral exchange format such as STEP; dimensions and mounting geometry |
| Printable parts | STL files for every printed component, with units, quantities, orientation, and printing settings |
| Assembly | Bill of materials, specifications for non-printed parts, illustrated assembly instructions, and an assembled-prototype photograph |
| Simulation | Articulated URDF and/or USD model in the format agreed with the authors; all referenced visual and collision meshes; joint axes, limits, initial values, and documented physical parameters |
| Simulation check | Loading instructions, software versions, and a runnable script or reproducible procedure that exercises the moving parts |
| Task definition | Initial state, action sequence, success criterion, timeout, intermediate stages where needed, failure conditions, and reset procedure |
| Physical validation | External-camera video showing mounting, full mechanism operation, successful task execution, and reset; both hands visible for a bimanual task |
| Trial example | At least one completed example trial record, identifying the mechanism version, performer or platform, and any known limitations |
| Attribution and provenance | Contributor names and roles, sources of reused material, and the existing licenses or permissions for those materials |

Identify measured physical parameters separately from nominal simulation values. A render alone does not demonstrate a functional asset. Label human demonstrations and robot trials accurately; discuss the available validation platform with the authors during the proposal stage.

## Submission and review

1. **Propose the task.** Send the idea and sketch to [ricardo.godoy@usp.br](mailto:ricardo.godoy@usp.br). The authors check scope, overlap, feasibility, and the intended contribution.
2. **Agree on the deliverables.** Confirm mounting, simulation format, validation evidence, and applicable contribution licenses before preparing the final package.
3. **Build and validate.** Complete the files above and document any unresolved limitations.
4. **Submit for review.** Send a repository or file-package link to the authors. They will coordinate the pull request to the appropriate HiveBoard repository. Include a short file index and the contributor roles.
5. **Address review comments.** Acceptance depends on function, reproducibility, task relevance, and file completeness. The maintainers coordinate integration into a versioned extension.

The example record validates the proposed task description; it does not constitute a complete benchmark evaluation. Evaluations of the current benchmark still require **all 13 conditions and five trials per condition**.

## Authorship and acknowledgments

Anyone who contributes **at least one accepted, functional asset with all required files and validation evidence** may be eligible for authorship on future HiveBoard papers that use and report that contribution.

Authorship is assessed for each paper. Eligible contributors should also participate in drafting or substantive revision of the paper, approve its final version, and take responsibility for their contribution. Asset acceptance does not guarantee authorship on every future HiveBoard paper or establish author order. The authors will discuss the expected role when planning the relevant paper.

Smaller contributions, such as design suggestions, documentation corrections, or limited testing, may be credited in the acknowledgments with the contributor's consent. These contributions are also welcome. Keep a record of each person's work so credit reflects the contribution made.
