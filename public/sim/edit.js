/**
 * Trajectory editor overlay. TEMPORARY dev tool -- see tools/traj_edit.py.
 *
 * The widget already draws the path the fingertips will take. This puts the
 * handful of keyframes that path was authored from on top of it as beads,
 * lets one be dragged, and hands the result back to tools/traj_edit.py, which
 * re-solves the task with the real IK and the real acceptance replay and
 * sends the new trajectory straight back into the running scene. What you see
 * play back after a drag is what the next build will produce.
 *
 * Only loaded when the page is opened with ?edit=1, and only useful while the
 * editor server is running:
 *
 *     python3 tools/traj_edit.py
 *     http://localhost:5173/sim/hiveboard-sim.html?edit=1
 *
 * To remove the editor, delete this file and the `?edit=1` block at the
 * bottom of hiveboard-sim.html (tools/traj_edit.py lists the rest).
 */

const API = new URLSearchParams(location.search).get('api') || 'http://localhost:8770';

// How far an arrow key moves a bead: a centimetre is about the smallest
// change worth looking at, and shift drops it to a millimetre for the last
// bit of a grasp.
const NUDGE = 0.01;
const FINE = 0.001;

export function attach(sim) {
  const { THREE } = sim;

  const state = {
    robot: null,
    module: null,
    keys: [],
    joints: [],
    jointRanges: [],
    gripRange: [0, 1],
    objectJoints: [],
    objectRanges: [],
    editMode: 'arm',
    edits: {},          // module -> { keys: { index: {dpos, secs, grip, angle} }, added: [ {id, after, pos, dpos, finger, approach, grip, secs, transit} ] }
    selected: null,
    busy: false,
    loaded: false,      // false while the solver is unreachable, so we retry
    link: true,         // drag beads that sit on top of each other together
    auto: true,         // re-solve as soon as a bead is dropped
    holdAfterSolve: false,
    manualMode: false,  // lamp keyframes are authored directly, not by IK
  };

  const markers = new THREE.Group();
  markers.name = 'trajectory editor';
  sim.scene.add(markers);

  const ui = buildPanel();
  document.body.appendChild(ui.root);

  // ── talking to the solver ────────────────────────────────────────────────
  async function api(path, payload, signal) {
    const opts = { signal };
    if (payload) {
      opts.method = 'POST';
      opts.headers = { 'Content-Type': 'application/json' };
      opts.body = JSON.stringify(payload);
    }
    let resp;
    try {
      resp = await fetch(API + path, opts);
    } catch (err) {
      if (err.name === 'AbortError') throw err;
      // fetch() reports every network failure as the same bare "Failed to
      // fetch", which is no help at all when the answer is almost always
      // that the solver is not running.
      state.loaded = false;
      throw new Error(`no answer from ${API} — is \`python3 tools/traj_edit.py\` running?`);
    }
    const data = await resp.json().catch(() => ({ error: `HTTP ${resp.status}` }));
    if (data.error) throw new Error(data.error);
    return data;
  }

  function moduleEdits(module = state.module) {
    if (!state.edits[module]) state.edits[module] = { keys: {}, added: [] };
    if (!state.edits[module].keys) state.edits[module].keys = {};
    if (!state.edits[module].added) state.edits[module].added = [];
    return state.edits[module];
  }

  function editsFor(module) {
    return moduleEdits(module);
  }

  function getAddedKey(id) {
    const added = moduleEdits().added;
    return added.find((k) => String(k.id) === String(id));
  }

  function editFor(index) {
    const addedKey = getAddedKey(index);
    if (addedKey) return addedKey;

    const keys = moduleEdits().keys;
    const strIdx = String(index);
    if (!keys[strIdx]) keys[strIdx] = {};
    return keys[strIdx];
  }

  function addPose() {
    if (!state.module || !state.keys.length) return;

    let afterKey = null;
    if (state.selected !== null) {
      afterKey = state.keys.find((k) => String(k.index) === String(state.selected));
    }
    if (!afterKey) {
      const candidates = state.keys.filter((k) => !k.home && !k.off);
      afterKey = candidates[candidates.length - 1] || state.keys[0];
    }

    const afterIndex = afterKey ? afterKey.index : 0;
    const afterPosIdx = state.keys.indexOf(afterKey);
    const nextKey = afterPosIdx >= 0 && afterPosIdx + 1 < state.keys.length
      ? state.keys[afterPosIdx + 1]
      : null;

    let newPos;
    if (afterKey && afterKey.pos && nextKey && nextKey.pos && !nextKey.off && dist(afterKey.pos, nextKey.pos) > 0.005) {
      newPos = [
        (afterKey.pos[0] + nextKey.pos[0]) / 2,
        (afterKey.pos[1] + nextKey.pos[1]) / 2,
        (afterKey.pos[2] + nextKey.pos[2]) / 2,
      ];
    } else if (afterKey && afterKey.pos) {
      const app = afterKey.approach || [0, 0, -1];
      newPos = [
        afterKey.pos[0] - app[0] * 0.03,
        afterKey.pos[1] - app[1] * 0.03,
        afterKey.pos[2] - app[2] * 0.03,
      ];
    } else {
      newPos = [0, 0, 0.5];
    }

    const finger = afterKey && afterKey.finger ? [...afterKey.finger] : [0, 1, 0];
    const approach = afterKey && afterKey.approach ? [...afterKey.approach] : [0, 0, -1];
    const grip = afterKey && afterKey.grip !== null && afterKey.grip !== undefined
      ? afterKey.grip
      : 0.0;
    const secs = 1.0;

    const medits = moduleEdits();
    let maxNum = 0;
    for (const k of medits.added) {
      const m = String(k.id).match(/^a(\d+)$/);
      if (m) maxNum = Math.max(maxNum, parseInt(m[1], 10));
    }
    const newId = `a${maxNum + 1}`;

    const roundNum = (v, n = 5) => Math.round(v * 10 ** n) / 10 ** n;
    const midpoint = (a, b) => a && b
      ? a.map((v, i) => roundNum((v + b[i]) / 2))
      : a ? [...a] : null;
    const newQpos = midpoint(afterKey && afterKey.qpos, nextKey && nextKey.qpos);
    const newObjectQpos = midpoint(afterKey && afterKey.objectQpos,
                                   nextKey && nextKey.objectQpos);

    const newAddedKey = {
      id: newId,
      index: newId,
      added: true,
      kind: 'pose',
      off: false,
      home: false,
      draggable: true,
      after: String(afterIndex),
      pos: [roundNum(newPos[0]), roundNum(newPos[1]), roundNum(newPos[2])],
      finger: [roundNum(finger[0]), roundNum(finger[1]), roundNum(finger[2])],
      approach: [roundNum(approach[0]), roundNum(approach[1]), roundNum(approach[2])],
      grip: roundNum(grip, 4),
      secs: roundNum(secs, 2),
      transit: false,
      qpos: newQpos,
      originalQpos: newQpos ? [...newQpos] : null,
      objectQpos: newObjectQpos,
      originalObjectQpos: newObjectQpos ? [...newObjectQpos] : null,
    };

    medits.added.push(newAddedKey);
    state.selected = newId;
    sim.setPlaying(false);
    solve(undefined, undefined, state.module === 'lamp');
  }

  function removeAdded(id) {
    const medits = moduleEdits();
    const idx = medits.added.findIndex((k) => String(k.id) === String(id));
    if (idx >= 0) {
      medits.added.splice(idx, 1);
      if (String(state.selected) === String(id)) {
        state.selected = null;
      }
      solve();
    }
  }

  /**
   * Switch a keyframe out of the path, or back into it.
   *
   * The step before it then runs straight on to whatever came next -- which
   * is the quickest way to find out whether a standoff, a settling beat or a
   * whole arc was doing anything for the motion.
   */
  function toggle(index) {
    const key = state.keys.find((k) => String(k.index) === String(index));
    if (!key) return;
    if (key.added) {
      removeAdded(index);
      return;
    }
    if (String(index) === '0') return;         // the first key is the start pose
    const edit = editFor(index);
    if (edit.off) delete edit.off; else edit.off = true;
    prune(index);
    solve();
  }

  /** Drop an edit that no longer says anything, so /save stays readable. */
  function prune(index) {
    const addedKey = getAddedKey(index);
    if (addedKey) {
      if (addedKey.dpos && addedKey.dpos.every((v) => Math.abs(v) < 1e-9)) {
        delete addedKey.dpos;
      }
      return;
    }
    const keys = moduleEdits().keys;
    const strIdx = String(index);
    const edit = keys[strIdx];
    if (!edit) return;
    if (edit.dpos && edit.dpos.every((v) => Math.abs(v) < 1e-9)) delete edit.dpos;
    if (!Object.keys(edit).length) delete keys[strIdx];
  }

  /**
   * Pick up whatever edits are on file, then solve this task from source.
   *
   * Solving rather than trusting the published trajectory: it may have been
   * built before a later change to the robot's config, in which case the
   * beads and the path the widget is drawing would disagree with each other.
   * What the editor shows is always what today's sources produce.
   */
  async function load() {
    const targetRobot = state.robot;
    const targetModule = state.module;
    if (!targetRobot) return;
    status('loading…');
    try {
      const data = await api(`/state?robot=${targetRobot}`);
      if (state.robot !== targetRobot) return;
      state.edits = data.edits || {};
      state.joints = data.joints || [];
      state.jointRanges = data.jointRanges || [];
      state.gripRange = data.gripRange || [0, 1];
      state.loaded = true;
      if (data.modules && data.modules.length && !data.modules.includes(state.module)) {
        const currentActive = sim.task ? sim.task.watch.split('_')[0] : null;
        state.module = (currentActive && data.modules.includes(currentActive))
          ? currentActive
          : data.modules[0];
      }
      state.manualMode = state.module === 'lamp'
        && Array.isArray(data.manualKeys) && data.manualKeys.length > 0;
      if (state.manualMode) state.keys = data.manualKeys;
      // Restore a saved hand-authored lamp path directly. Other tasks still
      // start from the normal solver and their saved edits.
      await solve(state.module, state.robot, state.manualMode);
    } catch (err) {
      if (err.name !== 'AbortError') {
        status(err.message, false);
      }
    }
  }

  let solveTimer = null;
  let solveSeq = 0;
  let activeAbortController = null;

  function solveSoon(delay = 350) {
    clearTimeout(solveTimer);
    solveTimer = setTimeout(() => solve(), delay);
  }

  async function solve(targetModule = state.module, targetRobot = state.robot, forceManual = false) {
    clearTimeout(solveTimer);
    if (!targetModule || !targetRobot) return;

    const mySeq = ++solveSeq;
    if (activeAbortController) {
      activeAbortController.abort();
    }
    activeAbortController = new AbortController();
    const signal = activeAbortController.signal;

    state.busy = true;
    const manual = (forceManual || state.manualMode) && targetModule === 'lamp';
    if (manual) state.manualMode = true;
    if (manual) {
      sim.setPlaying(false);
      sim.setTrajectoryPending(true);
    }
    status('solving…');
    try {
      const result = await api(manual ? '/manual' : '/solve', manual
        ? { robot: targetRobot, module: targetModule, keys: state.keys,
            spin: sim.task ? sim.task.spin || 0 : 0 }
        : { robot: targetRobot, module: targetModule,
            edits: editsFor(targetModule), spin: sim.task ? sim.task.spin || 0 : 0 }, signal);

      if (mySeq !== solveSeq) return;

      const resultModule = result.module || targetModule;
      if (result.joints) state.joints = result.joints;
      if (result.jointRanges) state.jointRanges = result.jointRanges;
      if (result.gripRange) state.gripRange = result.gripRange;
      if (result.objectJoints) state.objectJoints = result.objectJoints;
      if (result.objectRanges) state.objectRanges = result.objectRanges;

      // Always write the solved trajectory into its dedicated module slot in sim.trajectories
      if (result.traj && sim.trajectories && sim.trajectories[resultModule]) {
        sim.replaceTrajectory(resultModule, result.traj);
      }

      // ONLY update the active scene if the active task is still this exact module and robot
      const currentActiveModule = sim.task ? sim.task.watch.split('_')[0] : null;
      const currentActiveRobot = sim.robot ? sim.robot.name : null;
      if (currentActiveRobot === targetRobot && currentActiveModule === resultModule) {
        state.keys = result.keys || [];
        if (result.traj) {
          sim.setTrajectoryPending(false);
          sim.resetScene();
          sim.drawPath();
          const selected = state.keys.find((k) => String(k.index) === String(state.selected));
          if (state.holdAfterSolve && selected && selected.sample !== undefined) {
            sim.setPlaying(false);
            sim.setSample(selected.sample);
          } else {
            sim.setPlaying(true);
          }
          state.holdAfterSolve = false;
        }
        status(result.why, result.ok);
      }
    } catch (err) {
      if (err.name === 'AbortError') return;
      if (mySeq === solveSeq) {
        status(err.message, false);
        if (manual) sim.setTrajectoryPending(false);
      }
    } finally {
      if (mySeq === solveSeq) {
        state.busy = false;
        draw();
        render();
      }
    }
  }

  async function save() {
    if (state.busy) return;
    state.busy = true;
    status('saving…');
    try {
      const result = await api('/save', {
        robot: state.robot,
        edits: state.edits,
        manualKeys: state.module === 'lamp' ? state.keys : null,
      });
      if (state.module === 'lamp' && state.keys.length) state.manualMode = true;
      if (result.trajectory && state.module === 'lamp') {
        sim.replaceTrajectory('lamp', result.trajectory);
        sim.setTrajectoryPending(false);
        sim.resetScene();
        sim.drawPath();
      }
      const missed = result.report.filter((r) => !r.ok).map((r) => r.module);
      status(missed.length
        ? `saved to ${result.saved} — still missing: ${missed.join(', ')}`
        : `saved to ${result.saved} — all tasks pass`, !missed.length);
    } catch (err) {
      status(err.message, false);
    } finally {
      state.busy = false;
    }
  }

  // ── beads ────────────────────────────────────────────────────────────────
  const geoPose = new THREE.SphereGeometry(0.011, 18, 12);
  // Bigger than a pose bead: an arc ends where the next key starts, so the
  // two sit on top of each other and the arc has to show past it.
  const geoArc = new THREE.OctahedronGeometry(0.019);
  const colours = { pose: 0x2563eb, arc: 0xea580c, home: 0x94a3b8, added: 0x059669, selected: 0xdc2626 };

  function draw() {
    for (const child of [...markers.children]) {
      child.material.dispose();      // the two geometries are shared, not per bead
      markers.remove(child);
    }
    // Draw unselected first, then selected on top so selected bead is always rendered in front
    const sortedKeys = [...state.keys].sort((a, b) => {
      const aSel = String(a.index) === String(state.selected);
      const bSel = String(b.index) === String(state.selected);
      return aSel === bSel ? 0 : aSel ? 1 : -1;
    });

    for (const key of sortedKeys) {
      if (key.off) continue;
      const isSel = String(key.index) === String(state.selected);
      const isArc = key.kind === 'arc';
      const colour = isSel ? colours.selected
        : key.home ? colours.home
        : key.added ? colours.added
        : colours[key.kind];
      const mesh = new THREE.Mesh(
        isArc ? geoArc : geoPose,
        new THREE.MeshBasicMaterial({
          color: colour,
          transparent: true,
          opacity: isSel ? 1.0 : isArc ? 0.65 : 0.85,
          depthTest: false,
        }));
      mesh.renderOrder = isSel ? 20 : isArc ? 12 : 10;
      mesh.position.copy(sim.toThree(key.pos));
      mesh.userData.index = key.index;
      markers.add(mesh);
    }
  }

  /** Every key sitting on the same point as this one -- a grasp is two or three. */
  function stacked(index) {
    const here = state.keys.find((k) => String(k.index) === String(index));
    if (!state.link || !here) return [index];
    if (!here.pos) return [index];
    return state.keys
      .filter((k) => k.draggable && dist(k.pos, here.pos) < 1e-4)
      .map((k) => k.index);
  }

  function dist(a, b) {
    return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  }

  /** Add a world-space offset (MuJoCo axes) to a key's edit. */
  function move(index, delta) {
    for (const i of stacked(index)) {
      const edit = editFor(i);
      const dpos = edit.dpos || [0, 0, 0];
      edit.dpos = [dpos[0] + delta[0], dpos[1] + delta[1], dpos[2] + delta[2]];
      prune(i);
    }
  }

  /** three.js is Y-up, MuJoCo is Z-up: the widget's toThree(), backwards. */
  function toMujoco(v) {
    return [v.x, -v.z, v.y];
  }

  // ── dragging ─────────────────────────────────────────────────────────────
  const ray = new THREE.Raycaster();
  const plane = new THREE.Plane();
  const pointer = new THREE.Vector2();
  const hit = new THREE.Vector3();
  const drag = { index: null, start: new THREE.Vector3(), origin: new THREE.Vector3() };

  function picks(event) {
    const rect = sim.renderer.domElement.getBoundingClientRect();
    pointer.set((event.clientX - rect.left) / rect.width * 2 - 1,
                -(event.clientY - rect.top) / rect.height * 2 + 1);
    ray.setFromCamera(pointer, sim.camera);
    return ray.intersectObjects(markers.children, false);
  }

  function onPointerDown(event) {
    if (!state.keys.length || event.button !== 0) return;
    const hits = picks(event);
    if (!hits.length) return;

    const hitMesh = hits[0].object;
    const hitPos = hitMesh.position;

    // Find all keys whose 3D bead sits in this cluster (within 25mm)
    const cluster = state.keys.filter(
      (k) => !k.off && k.pos && sim.toThree(k.pos).distanceTo(hitPos) < 0.025
    );

    let index;
    if (cluster.length > 1) {
      // If one of the beads in this cluster is already selected, cycle to the next one!
      const curIdx = cluster.findIndex((k) => String(k.index) === String(state.selected));
      if (curIdx >= 0) {
        index = cluster[(curIdx + 1) % cluster.length].index;
      } else {
        index = hitMesh.userData.index !== undefined ? hitMesh.userData.index : cluster[0].index;
      }
    } else {
      index = hitMesh.userData.index;
    }

    select(index);
    const key = state.keys.find((k) => String(k.index) === String(index));
    if (!key || !key.draggable) return;

    // Take the gesture off OrbitControls, which listens on the same canvas.
    event.stopPropagation();
    event.preventDefault();
    sim.controls.enabled = false;
    drag.index = index;
    drag.origin.copy(hitPos);
    plane.setFromNormalAndCoplanarPoint(
      sim.camera.getWorldDirection(new THREE.Vector3()), drag.origin);
    ray.ray.intersectPlane(plane, drag.start);
    sim.renderer.domElement.setPointerCapture(event.pointerId);
  }

  function onPointerMove(event) {
    if (drag.index === null) return;
    const rect = sim.renderer.domElement.getBoundingClientRect();
    pointer.set((event.clientX - rect.left) / rect.width * 2 - 1,
                -(event.clientY - rect.top) / rect.height * 2 + 1);
    ray.setFromCamera(pointer, sim.camera);
    if (!ray.ray.intersectPlane(plane, hit)) return;

    const delta = hit.clone().sub(drag.start);
    if (event.shiftKey) delta.setX(0).setZ(0);          // straight up and down
    else if (event.ctrlKey || event.metaKey) delta.setY(0);   // along the board
    for (const i of stacked(drag.index)) {
      const mesh = markers.children.find((m) => String(m.userData.index) === String(i));
      if (mesh) mesh.position.copy(drag.origin).add(delta);
    }
  }

  function onPointerUp(event) {
    if (drag.index === null) return;
    const mesh = markers.children.find((m) => String(m.userData.index) === String(drag.index));
    const delta = mesh ? mesh.position.clone().sub(drag.origin) : new THREE.Vector3();
    const idx = drag.index;
    drag.index = null;
    sim.controls.enabled = true;
    sim.renderer.domElement.releasePointerCapture?.(event.pointerId);
    if (delta.lengthSq() < 1e-10) return;
    move(idx, toMujoco(delta));
    render();
    if (state.auto) solve(); else draw();
  }

  function onKeyDown(event) {
    if (/^(INPUT|TEXTAREA|SELECT)$/.test(event.target.tagName)) return;
    if (event.key === 's' || event.key === 'S') return void solve();
    if (event.key === 'a' || event.key === 'A') return void addPose();
    if (event.key === '[' || event.key === ']') {
      event.preventDefault();
      const liveKeys = state.keys.filter((k) => !k.off);
      if (!liveKeys.length) return;
      const curIdx = liveKeys.findIndex((k) => String(k.index) === String(state.selected));
      let nextIdx;
      if (event.key === ']') {
        nextIdx = curIdx >= 0 ? (curIdx + 1) % liveKeys.length : 0;
      } else {
        nextIdx = curIdx > 0 ? curIdx - 1 : liveKeys.length - 1;
      }
      select(liveKeys[nextIdx].index);
      return;
    }
    if (state.selected === null) return;
    if (event.key === 'd' || event.key === 'D') return void toggle(state.selected);
    const key = state.keys.find((k) => String(k.index) === String(state.selected));
    if (!key || !key.draggable) return;

    const step = event.shiftKey ? FINE : NUDGE;
    const right = new THREE.Vector3().setFromMatrixColumn(sim.camera.matrixWorld, 0);
    const up = new THREE.Vector3().setFromMatrixColumn(sim.camera.matrixWorld, 1);
    const into = sim.camera.getWorldDirection(new THREE.Vector3());
    const moves = {
      ArrowLeft: right.clone().multiplyScalar(-step),
      ArrowRight: right.clone().multiplyScalar(step),
      ArrowUp: (event.altKey ? into : up).clone().multiplyScalar(step),
      ArrowDown: (event.altKey ? into : up).clone().multiplyScalar(-step),
    };
    if (!moves[event.key]) return;
    event.preventDefault();
    move(state.selected, toMujoco(moves[event.key]));
    render();
    if (state.auto) solveSoon(); else draw();
  }

  const canvas = sim.renderer.domElement;
  canvas.addEventListener('pointerdown', onPointerDown, true);
  canvas.addEventListener('pointermove', onPointerMove);
  window.addEventListener('pointerup', onPointerUp);
  window.addEventListener('keydown', onKeyDown);

  // ── panel ────────────────────────────────────────────────────────────────
  function select(index) {
    // Selecting a pose is an inspection action: stop the running trajectory
    // and jump there immediately, before rendering the controls.
    if (activeAbortController) {
      activeAbortController.abort();
      activeAbortController = null;
      solveSeq++;
      state.busy = false;
    }
    state.selected = index;
    const key = state.keys.find((k) => String(k.index) === String(index));
    if (key && key.sample !== undefined && !key.off) {
      sim.setPlaying(false);
      sim.setSample(key.sample);
    }
    draw();
    render();
  }

  function status(text, ok) {
    ui.status.textContent = text;
    ui.status.style.color = ok === undefined ? '#475569' : ok ? '#15803d' : '#b91c1c';
  }

  function num(value, step, onChange) {
    const input = document.createElement('input');
    input.type = 'number';
    input.step = step;
    input.value = value;
    input.addEventListener('change', () => {
      onChange(parseFloat(input.value));
      render();
      if (state.auto) solve();
    });
    return input;
  }

  function renderJointControls(key) {
    ui.joints.replaceChildren();
    if (!key || key.off || !key.qpos || !state.joints.length) return;

    const title = document.createElement('div');
    title.textContent = state.editMode === 'arm'
      ? 'arm points — move sliders, then press Go'
      : 'lamp points — move sliders, then press Go';
    title.style.marginTop = '8px';
    title.style.color = '#475569';
    ui.joints.appendChild(title);

    const modes = document.createElement('div');
    modes.style.margin = '6px 0';
    for (const [mode, text] of [['arm', 'Arm points'], ['object', 'Lamp points']]) {
      const button = document.createElement('button');
      button.type = 'button';
      button.textContent = text;
      button.style.marginRight = '5px';
      button.style.background = state.editMode === mode ? '#dbeafe' : '#fff';
      button.addEventListener('click', (event) => {
        event.stopPropagation();
        state.editMode = mode;
        render();
      });
      modes.appendChild(button);
    }
    ui.joints.appendChild(modes);

    if (state.editMode === 'object') {
      const values = key.objectQpos;
      if (!values || !state.objectJoints.length) return;
      const original = key.originalObjectQpos || values;
      const grid = document.createElement('div');
      grid.style.display = 'grid';
      grid.style.gridTemplateColumns = '1fr 1fr';
      grid.style.gap = '4px 8px';
      state.objectJoints.forEach((joint, i) => {
        const label = document.createElement('label');
        label.textContent = joint;
        const input = document.createElement('input');
        input.type = 'range';
        input.min = state.objectRanges[i][0];
        input.max = state.objectRanges[i][1];
        input.step = '0.001';
        input.value = values[i];
        const output = document.createElement('output');
        output.textContent = Number(values[i]).toFixed(4);
        input.addEventListener('input', () => {
          const next = [...key.objectQpos];
          next[i] = parseFloat(input.value);
          key.objectQpos = next;
          editFor(key.index).objectQpos = next;
          output.textContent = next[i].toFixed(4);
          sim.setEditorObjectPose(next);
        });
        const reset = document.createElement('button');
        reset.type = 'button'; reset.textContent = '↺';
        reset.title = 'Reset this lamp joint';
        reset.addEventListener('click', (event) => {
          event.stopPropagation();
          const next = [...key.objectQpos]; next[i] = original[i];
          key.objectQpos = next; editFor(key.index).objectQpos = next;
          input.value = next[i]; output.textContent = Number(next[i]).toFixed(4);
          sim.setEditorObjectPose(next);
        });
        label.append(input, output, reset); grid.appendChild(label);
      });
      ui.joints.appendChild(grid);
      return;
    }

    const gripLabel = document.createElement('label');
    gripLabel.textContent = 'gripper';
    const gripInput = document.createElement('input');
    gripInput.type = 'range';
    gripInput.min = state.gripRange[0];
    gripInput.max = state.gripRange[1];
    gripInput.step = '0.001';
    gripInput.value = key.grip;
    const gripOutput = document.createElement('output');
    gripOutput.textContent = Number(key.grip).toFixed(4);
    gripOutput.style.marginLeft = '8px';
    gripInput.addEventListener('input', () => {
      const value = parseFloat(gripInput.value);
      editFor(key.index).grip = value;
      key.grip = value;
      gripOutput.textContent = value.toFixed(4);
      sim.setEditorPose(key.qpos, value);
      // In hand-authored mode the playback trajectory is built from the
      // pose rows, so keep it synchronized with the live gripper preview.
      if (state.module === 'lamp') {
        state.manualMode = true;
        solveSoon(180);
      }
    });
    const gripReset = document.createElement('button');
    gripReset.type = 'button';
    gripReset.textContent = '↺';
    gripReset.title = 'Reset gripper to its original pose value';
    gripReset.style.padding = '1px 5px';
    gripReset.style.marginLeft = '6px';
    gripReset.addEventListener('click', (event) => {
      event.stopPropagation();
      const value = key.originalGrip ?? key.grip;
      editFor(key.index).grip = value;
      key.grip = value;
      gripInput.value = value;
      gripOutput.textContent = Number(value).toFixed(4);
      sim.setEditorPose(key.qpos, value);
      if (state.module === 'lamp') {
        state.manualMode = true;
        solveSoon(180);
      }
    });
    gripLabel.append(gripInput, gripOutput, gripReset);
    ui.joints.appendChild(gripLabel);

    const grid = document.createElement('div');
    grid.style.display = 'grid';
    grid.style.gridTemplateColumns = '1fr 1fr';
    grid.style.gap = '4px 8px';
    state.joints.forEach((joint, i) => {
      if (i >= key.qpos.length) return;
      const label = document.createElement('label');
      label.textContent = joint;
      label.style.marginTop = '4px';
      const range = state.jointRanges[i] || [-3.14, 3.14];
      const input = document.createElement('input');
      input.type = 'range';
      input.min = range[0];
      input.max = range[1];
      input.step = '0.001';
      input.value = key.qpos[i];
      input.title = 'MuJoCo joint position';
      const value = document.createElement('output');
      value.textContent = Number(key.qpos[i]).toFixed(4);
      value.style.display = 'block';
      value.style.color = '#0f172a';
      input.addEventListener('input', () => {
        const edit = editFor(key.index);
        const qpos = [...key.qpos];
        const value = parseFloat(input.value);
        if (!Number.isFinite(value)) return;
        qpos[i] = value;
        edit.qpos = qpos;
        key.qpos[i] = value;
        output.textContent = value.toFixed(4);
        sim.setEditorPose(qpos, key.grip);
      });
      const output = value;
      label.appendChild(input);
      label.appendChild(output);
      const reset = document.createElement('button');
      reset.type = 'button';
      reset.textContent = '↺';
      reset.title = 'Reset this joint to its original pose value';
      reset.style.padding = '1px 5px';
      reset.addEventListener('click', (event) => {
        event.stopPropagation();
        const original = key.originalQpos || key.qpos;
        const qpos = [...key.qpos];
        qpos[i] = original[i];
        editFor(key.index).qpos = qpos;
        key.qpos[i] = original[i];
        input.value = original[i];
        output.textContent = Number(original[i]).toFixed(4);
        sim.setEditorPose(qpos, key.grip);
      });
      label.appendChild(reset);
      grid.appendChild(label);
    });
    ui.joints.appendChild(grid);
  }

  function render() {
    const edits = state.module ? moduleEdits().keys : {};
    ui.title.textContent = `${state.robot || '—'} · ${state.module || '—'}`;
    ui.rows.replaceChildren();

    for (const key of state.keys) {
      const edit = editFor(key.index);
      const isSelected = String(key.index) === String(state.selected);
      const row = document.createElement('div');
      row.className = 'row' + (isSelected ? ' on' : '');
      row.addEventListener('click', (e) => {
        // The whole row is a pose selector. Keep number/range inputs and the
        // enable/disable button independent so editing them does not jump the
        // current pose unexpectedly.
        if (!e.target.closest('input, button')) select(key.index);
      });

      const tag = document.createElement('span');
      tag.className = 'tag';
      tag.textContent = `${key.index}`;
      tag.style.background = key.home ? '#94a3b8'
        : key.kind === 'arc' ? '#ea580c'
        : key.added ? '#059669'
        : '#2563eb';
      row.appendChild(tag);

      const what = document.createElement('span');
      what.className = 'what';
      what.textContent = key.off ? 'off' : key.home ? 'home'
        : key.added ? 'added'
        : key.kind === 'arc' ? 'arc' : 'pose';
      if (key.transit) what.textContent += ' ·';        // transit: may be missed
      row.appendChild(what);

      if (key.off) {
        // Nothing to show: the key is out of the path, so it has no timing,
        // no grip and nowhere it puts the hand until it is switched back on.
        const gone = document.createElement('span');
        gone.className = 'gone';
        gone.textContent = 'not in the path';
        gone.style.gridColumn = 'span 4';
        row.appendChild(gone);
      } else {
        row.appendChild(num(key.secs, 0.1, (v) => {
          // Manual lamp trajectories consume timing directly from each pose;
          // keep the displayed keyframe and the edit record in sync.
          key.secs = v;
          editFor(key.index).secs = v;
        }));
        row.appendChild(num(key.grip, 0.05, (v) => { editFor(key.index).grip = v; }));
        if (key.kind === 'arc') {
          // How far it sweeps, and how far it climbs while sweeping. On the
          // lamp the second one is the whole task: the hand has to walk up
          // the thread at the pitch the bulb actually rides, or it drags the
          // glass against its own thread instead of following it.
          row.appendChild(num(key.angle, 0.05, (v) => { editFor(key.index).angle = v; }));
          // In millimetres, like every other distance the panel shows; the
          // solver wants metres.
          row.appendChild(num(Math.round(key.rise * 1e5) / 100, 1,
                              (v) => { editFor(key.index).rise = v / 1000; }));
        } else {
          const moved = document.createElement('span');
          moved.className = 'moved';
          moved.textContent = edit.dpos
            ? `${(Math.hypot(...edit.dpos) * 1000).toFixed(0)}mm`
            : '';
          row.appendChild(moved);
          row.appendChild(document.createElement('span'));
        }
      }

      const power = document.createElement('button');
      power.className = 'power';
      power.textContent = key.off ? '+' : '×';
      power.title = key.added ? 'delete this added pose'
        : key.off ? 'put this keyframe back in the path'
        : 'take this keyframe out of the path (D)';
      power.disabled = String(key.index) === '0';
      power.addEventListener('click', (e) => {
        e.stopPropagation();
        toggle(key.index);
      });
      row.appendChild(power);

      if (key.off) row.classList.add('dead');
      ui.rows.appendChild(row);
    }

    const key = state.selected !== null
      ? state.keys.find((k) => String(k.index) === String(state.selected))
      : null;
    const edit = key ? editFor(key.index) : null;
    ui.detail.textContent = key
      ? `#${key.index}${key.added ? ' (added pose)' : ''} at ${key.pos ? key.pos.map((v) => v.toFixed(3)).join(', ') : '—'}` +
        (edit && edit.dpos
          ? `   moved ${edit.dpos.map((v) => (v * 1000).toFixed(0)).join(', ')} mm`
          : '')
      : 'pick a bead on the path, or a row here';
    renderJointControls(key);
  }

  function buildPanel() {
    const style = document.createElement('style');
    style.textContent = `
      #traj-edit { position: fixed; top: 10px; right: 10px; width: 384px; z-index: 60;
        font: 12px/1.35 ui-monospace, SFMono-Regular, Menlo, monospace; color: #0f172a;
        background: #fff; border: 1px solid #cbd5e1; border-radius: 10px;
        box-shadow: 0 8px 24px rgba(15,23,42,.14); max-height: 92vh; overflow: auto; }
      #traj-edit h4 { margin: 0; padding: 8px 10px; font-size: 12px; letter-spacing: .04em;
        text-transform: uppercase; border-bottom: 1px solid #e2e8f0; background: #f8fafc;
        border-radius: 10px 10px 0 0; }
      #traj-edit .body { padding: 8px 10px 10px; }
      #traj-edit .head, #traj-edit .row { display: grid; gap: 6px; align-items: center;
        grid-template-columns: 18px 46px 1fr 1fr 1.15fr 1.15fr 18px; padding: 2px; border-radius: 5px; }
      #traj-edit .head { color: #64748b; font-size: 10px; text-transform: uppercase; }
      #traj-edit .row:hover { background: #f1f5f9; cursor: pointer; }
      #traj-edit .row.on { background: #fef3c7; }
      #traj-edit .tag { display: inline-block; width: 18px; text-align: center;
        color: #fff; border-radius: 4px; font-size: 10px; padding: 1px 0; }
      #traj-edit .what { color: #475569; white-space: nowrap; }
      #traj-edit .moved { color: #b45309; text-align: right; padding-right: 4px; }
      #traj-edit .gone { color: #94a3b8; font-style: italic; }
      #traj-edit .row.dead .tag, #traj-edit .row.dead .what { opacity: .45; }
      #traj-edit .row.dead .what { text-decoration: line-through; }
      #traj-edit .power { padding: 0; width: 18px; height: 18px; line-height: 1;
        border: 1px solid #e2e8f0; background: #fff; color: #64748b; border-radius: 4px; }
      #traj-edit .power:hover:not(:disabled) { color: #b91c1c; border-color: #fca5a5; }
      #traj-edit .power:disabled { opacity: .3; cursor: default; }
      #traj-edit input[type=number] { width: 100%; font: inherit; padding: 1px 3px;
        border: 1px solid #e2e8f0; border-radius: 4px; background: #fff; }
      #traj-edit .detail { margin: 6px 0; color: #475569; min-height: 16px; }
      #traj-edit .status { margin: 6px 0; min-height: 30px; color: #475569; }
      #traj-edit .buttons { display: flex; gap: 6px; flex-wrap: wrap; }
      #traj-edit button { font: inherit; padding: 4px 8px; border-radius: 6px; cursor: pointer;
        border: 1px solid #cbd5e1; background: #fff; }
      #traj-edit button:hover { border-color: #94a3b8; }
      #traj-edit button.primary { background: #0f172a; color: #fff; border-color: #0f172a; }
      #traj-edit label { display: block; color: #475569; margin-top: 6px; }
      #traj-edit .help { margin-top: 8px; color: #64748b; font-size: 10px; line-height: 1.5;
        border-top: 1px solid #e2e8f0; padding-top: 6px; }
      #traj-edit input[type=range] { width: 100%; }
    `;
    document.head.appendChild(style);

    const root = document.createElement('div');
    root.id = 'traj-edit';
    root.innerHTML = `
      <h4>trajectory editor</h4>
      <div class="body">
        <div class="detail" id="te-title"></div>
        <div class="head"><span></span><span>key</span><span>secs</span><span>grip</span><span>angle/Δ</span><span>rise mm</span><span></span></div>
      <div id="te-rows"></div>
        <div class="detail" id="te-joints"></div>
        <div class="detail" id="te-detail"></div>
        <div class="status" id="te-status"></div>
        <div class="buttons">
          <button class="primary" id="te-solve">Go / Use hand poses (S)</button>
          <button id="te-save">Save</button>
          <button id="te-add">+ Add pose (A)</button>
          <button id="te-undo">Reset key</button>
          <button id="te-clear">Reset task</button>
        </div>
        <label><input type="checkbox" id="te-auto" checked> re-solve on drop</label>
        <label><input type="checkbox" id="te-link" checked> move stacked beads together</label>
        <label>scrub <input type="range" id="te-scrub" min="0" max="1" value="0"></label>
        <div class="help">
          drag a bead to move that keyframe · click overlapping beads to cycle selection ([ and ] step through) ·
          shift = straight up and down · ctrl = along the board · arrows nudge the selected bead 10 mm
          (shift 1 mm, alt = towards camera) · + Add pose (A) inserts a new pose · × removes an added pose
          or toggles key (D), + puts it back · orange beads are arcs: change their angle instead
        </div>
      </div>`;

    return {
      root,
      title: root.querySelector('#te-title'),
      rows: root.querySelector('#te-rows'),
      joints: root.querySelector('#te-joints'),
      detail: root.querySelector('#te-detail'),
      status: root.querySelector('#te-status'),
      scrub: root.querySelector('#te-scrub'),
    };
  }

  ui.root.querySelector('#te-solve').addEventListener('click', () => {
    state.holdAfterSolve = true;
    solve(undefined, undefined, true);
  });
  ui.root.querySelector('#te-save').addEventListener('click', () => save());
  ui.root.querySelector('#te-add').addEventListener('click', () => addPose());
  ui.root.querySelector('#te-auto').addEventListener('change', (e) => {
    state.auto = e.target.checked;
  });
  ui.root.querySelector('#te-link').addEventListener('change', (e) => {
    state.link = e.target.checked;
  });
  ui.root.querySelector('#te-undo').addEventListener('click', () => {
    if (state.selected === null) return;
    const addedKey = getAddedKey(state.selected);
    if (addedKey) {
      removeAdded(state.selected);
      return;
    }
    for (const i of stacked(state.selected)) delete moduleEdits().keys[i];
    solve();
  });
  ui.root.querySelector('#te-clear').addEventListener('click', () => {
    state.edits[state.module] = { keys: {}, added: [] };
    solve();
  });
  ui.scrub.addEventListener('input', () => {
    sim.setPlaying(false);
    sim.setSample(parseInt(ui.scrub.value, 10));
  });

  let lastTry = 0;

  function reload() {
    lastTry = Date.now();
    load().catch((err) => status(err.message, false));
  }

  function syncTask() {
    const robot = sim.robot ? sim.robot.name : null;
    const module = sim.task ? sim.task.watch.split('_')[0] : null;
    if (robot !== state.robot || module !== state.module) {
      state.robot = robot;
      state.module = module;
      state.selected = null;
      state.keys = [];
      draw();
      render();
      if (robot && module) reload();
    }
  }

  // Fast response to UI clicks on task list and robot picker
  document.addEventListener('click', (e) => {
    if (e.target.closest('#task-list') || e.target.closest('#platform-picker')) {
      setTimeout(syncTask, 0);
    }
  });

  // The widget owns which robot and task are showing, and says so only by
  // changing what it is playing -- so watch rather than reach in and hook it.
  setInterval(() => {
    syncTask();
    if (state.robot && state.module && !state.loaded && !state.busy) {
      // The solver was not up when this task came round. Keep trying, so
      // starting it is all it takes -- no reloading the page.
      if (Date.now() - lastTry > 3000) reload();
    }
    if (sim.task && drag.index === null && document.activeElement !== ui.scrub) {
      ui.scrub.max = sim.task.qpos.length - 1;
      ui.scrub.value = sim.sample;
    }
  }, 100);

  render();
  status(`editor ready — ${API}`);
}
