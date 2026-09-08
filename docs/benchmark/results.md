# Report results

Report performance per attachment and preserve the trial-level data. Aggregate values alone hide the failure modes and stage progression that make the benchmark informative.

## Recommended metrics

| Metric | Calculation | Report for |
|---|---|---|
| Success rate | Successful trials ÷ recorded trials | Every attachment |
| Completion time | Median and individual successful-trial times | Every attachment with successes |
| Attempts | Median and individual counts | Every attachment |
| Regrasps | Median and individual counts | Every attachment |
| Stage completion | Fraction reaching each stage | Composed tasks |
| Failure distribution | Count by primary failure cause | Unsuccessful trials |
| Strategy distribution | Count of prehensile and non-prehensile trials | Every attachment |

With five trials per attachment, always show the numerator and denominator—for example, `4/5 (80%)`—rather than only a percentage.

## Keep conditions separate

Do not merge results across:

- different end-effectors;
- different control interfaces or policies;
- horizontal and vertical board mounting;
- ball valve trials with and without the friction ring;
- materially different print settings or modified parts; or
- simulation and physical trials.

## Accompanying files

A complete submission contains 65 trials: five for each of the 13 conditions. Include these files:

```text
result-package/
├── trials.csv
├── setup.jpg
├── platform.md
├── manifest.json
├── session.json
├── recording-instructions.md
└── videos/              # one recording per trial
```

The platform description identifies the robot, end-effector, control method, board orientation, calibration notes, HiveBoard version, printing parameters, post-processing, and any protocol deviations.

The [Evaluation Runner](/benchmark/evaluation-runner) generates the ZIP after all 65 trial entries and required setup details pass validation. Attach a JPEG setup photograph to include it as `setup.jpg`, or add it after extracting the ZIP. Add the recordings to `videos/` using the filenames in `recording-instructions.md`.

“Trial records complete” refers to the log and setup fields. It does not confirm that all supporting files are present or that an organizer has reviewed the outcomes. The manifest lists the expected recordings and whether the setup photograph was included when the ZIP was generated.

## Submit for organizer review

Submissions are reviewed by email. There is no upload server or automatic approval.

1. Complete all 65 trial entries and the required platform details in the [Evaluation Runner](/benchmark/evaluation-runner).
2. Download and extract the results ZIP. Add `setup.jpg` and the 65 external-camera recordings using the filenames in `recording-instructions.md`. For simulated evaluations, provide screen recordings of the complete trials.
3. Recompress the completed folder and upload it to storage you control. Give **ricardo.godoy@usp.br** download access and keep the package private during review. Keep the link available until review is complete.
4. In the runner's Review step, select **Submit for review**. Enter your contact name, email, institution (or Independent), and the package link. Robot, end-effector, and control interface are taken from the evaluation setup.
5. Confirm file completeness and permission to publish the approved trial data, platform description, setup photograph, and recordings. You must have permission to share the materials, including permission from identifiable people in the media.
6. Select **Prepare submission email**, review the draft, and send it from your email application. If no email application opens, copy the draft and send it to **ricardo.godoy@usp.br**. Opening a draft does not send it or confirm receipt.

Contact details and the private package link are included only in the email draft, not in the exported results or session backups. They must be entered again after reloading the page. Results can still be downloaded without submitting or granting publication permission.

### Organizer review

The organizer checks the 13 conditions and five records per condition, platform metadata, photograph, recording filenames, and the recorded outcomes against the video evidence. Browser validation is a preliminary check; it does not replace this review.

Corrections are requested by email. Keep the same submission ID when supplying a corrected package and describe the changes. Publication follows approval and requires the submitter's recorded permission.

Before publishing, prepare a separate copy containing only the approved trial data, platform description, photograph, and recordings. Check free-text notes and media for personal information. Do not publish contact details, private download links, correspondence, or the organizer's review notes. Public result pages and rankings will be added separately.

## Interpretation

HiveBoard exposes different constraints across attachments. A low score can indicate incompatible grasp geometry, insufficient force, kinematic limits, perception difficulty, slip, or insufficient control precision. Report these causes instead of interpreting every unsuccessful trial as the same type of failure.

Comparisons between platforms should state which factors differ. The protocol characterizes the complete manipulation platform and should not be used to attribute a result to the gripper, robot, controller, or operator in isolation without a controlled experiment.
