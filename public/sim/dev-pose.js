/*
 * The pose the simulator must adopt when the trajectory editor takes hold of a
 * key ("dev hold" -- see hiveboard-sim.html's devHold/devPose).
 *
 * Extracted from teleportSample() so the contract is unit-testable: the spec is
 * that after this call the robot ALREADY stands at the key's pose, because the
 * very next thing the caller does is capture the gripper-preview baseline
 * (setSample() -> captureGripperPreviewPose()).  Anything left un-written here
 * is a pose the robot has not reached yet, and gets baked in as a wrong
 * baseline.
 */

/**
 * Write the dev pose for `sampleIndex` into `qpos` (the live mjData.qpos).
 *
 * @param {Float64Array} qpos        live MuJoCo qpos buffer
 * @param {object} task              trajectory: qpos / grip / spin / stateQpos
 * @param {number} sampleIndex       sample to adopt
 * @param {object} layout
 * @param {number} layout.arm            number of arm joints (indices 0..arm-1)
 * @param {number} layout.gripQposIndex  qpos address of the gripper jaw, or -1
 * @param {number} layout.spinIndex      qpos address of the spin dof, or -1
 */
export function writeDevPose(qpos, task, sampleIndex, { arm, gripQposIndex, spinIndex }) {
  const state = task.stateQpos?.[sampleIndex];

  if (state && state.length === qpos.length) {
    qpos.set(state);
    return;
  }

  const pose = task.qpos?.[sampleIndex];
  if (!pose) return;

  for (let i = 0; i < arm; i++) qpos[i] = pose[i];

  /*
   * The key owns the jaw and the spin dof as well, and this branch is the only
   * place they can come from: a task with no full state carries them solely in
   * task.grip / task.spin.  macao.traj.json ships stateQpos: [], so before this
   * the jaw kept whatever the PREVIOUS selection left it at -- mj_forward()
   * froze that stale value, setSample() captured the preview baseline from it,
   * and only then did applyControl() servo the jaw to task.grip[sampleIndex].
   * The preview was therefore defined from a pose the robot had not reached.
   */
  const grip = task.grip?.[sampleIndex];
  if (gripQposIndex >= 0 && Number.isFinite(grip)) qpos[gripQposIndex] = grip;
  if (spinIndex >= 0) qpos[spinIndex] = task.spin || 0;
}
