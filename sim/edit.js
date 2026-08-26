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
    edits: {},          // module -> { keys: { index: {dpos, secs, grip, angle} } }
    selected: null,
    busy: false,
    loaded: false,      // false while the solver is unreachable, so we retry
    link: true,         // drag beads that sit on top of each other together
    auto: true,         // re-solve as soon as a bead is dropped
  };

  const markers = new THREE.Group();
  markers.name = 'trajectory editor';
  sim.scene.add(markers);

  const ui = buildPanel();
  document.body.appendChild(ui.root);

  // ── talking to the solver ────────────────────────────────────────────────
  async function api(path, payload) {
    const opts = payload
      ? { method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload) }
      : {};
    let resp;
    try {
      resp = await fetch(API + path, opts);
    } catch (_) {
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

  function editsFor(module) {
    if (!state.edits[module]) state.edits[module] = { keys: {} };
    if (!state.edits[module].keys) state.edits[module].keys = {};
    return state.edits[module];
  }

  function editFor(index) {
    const keys = editsFor(state.module).keys;
    if (!keys[index]) keys[index] = {};
    return keys[index];
  }

  /**
   * Switch a keyframe out of the path, or back into it.
   *
   * The step before it then runs straight on to whatever came next -- which
   * is the quickest way to find out whether a standoff, a settling beat or a
   * whole arc was doing anything for the motion.
   */
  function toggle(index) {
    const key = state.keys[index];
    if (!key || index === 0) return;         // the first key is the start pose
    const edit = editFor(index);
    if (edit.off) delete edit.off; else edit.off = true;
    prune(index);
    solve();
  }

  /** Drop an edit that no longer says anything, so /save stays readable. */
  function prune(index) {
    const keys = editsFor(state.module).keys;
    const edit = keys[index];
    if (!edit) return;
    if (edit.dpos && edit.dpos.every((v) => Math.abs(v) < 1e-9)) delete edit.dpos;
    if (!Object.keys(edit).length) delete keys[index];
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
    status('loading…');
    const data = await api(`/state?robot=${state.robot}`);
    state.edits = data.edits || {};
    state.loaded = true;
    await solve();
  }

  let solveTimer = null;

  function solveSoon(delay = 350) {
    clearTimeout(solveTimer);
    solveTimer = setTimeout(solve, delay);
  }

  async function solve() {
    clearTimeout(solveTimer);
    if (state.busy || !state.module) return;
    state.busy = true;
    status('solving…');
    try {
      const result = await api('/solve', {
        robot: state.robot,
        module: state.module,
        edits: editsFor(state.module),
        spin: sim.task ? sim.task.spin || 0 : 0,
      });
      state.keys = result.keys;
      if (result.traj) {
        // The widget holds this object by reference, so writing the new
        // samples into it swaps the trajectory under the running scene.
        Object.assign(sim.trajectories[state.module], result.traj);
        sim.resetScene();
        sim.drawPath();
        sim.setPlaying(true);
      }
      status(result.why, result.ok);
    } catch (err) {
      status(err.message, false);
    } finally {
      state.busy = false;
      draw();
      render();
    }
  }

  async function save() {
    if (state.busy) return;
    state.busy = true;
    status('saving…');
    try {
      const result = await api('/save', { robot: state.robot, edits: state.edits });
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
  const colours = { pose: 0x2563eb, arc: 0xea580c, home: 0x94a3b8, selected: 0xdc2626 };

  function draw() {
    for (const child of [...markers.children]) {
      child.material.dispose();      // the two geometries are shared, not per bead
      markers.remove(child);
    }
    for (const key of state.keys) {
      if (key.off) continue;
      const colour = key.index === state.selected ? colours.selected
        : key.home ? colours.home : colours[key.kind];
      const mesh = new THREE.Mesh(
        key.kind === 'arc' ? geoArc : geoPose,
        new THREE.MeshBasicMaterial({
          color: colour, transparent: true,
          opacity: key.index === state.selected ? 1 : 0.75,
          depthTest: false,
        }));
      mesh.renderOrder = 10;
      mesh.position.copy(sim.toThree(key.pos));
      mesh.userData.index = key.index;
      markers.add(mesh);
    }
  }

  /** Every key sitting on the same point as this one -- a grasp is two or three. */
  function stacked(index) {
    const here = state.keys[index];
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

  function pick(event) {
    const rect = sim.renderer.domElement.getBoundingClientRect();
    pointer.set((event.clientX - rect.left) / rect.width * 2 - 1,
                -(event.clientY - rect.top) / rect.height * 2 + 1);
    ray.setFromCamera(pointer, sim.camera);
    return ray.intersectObjects(markers.children, false)[0];
  }

  function onPointerDown(event) {
    if (!state.keys.length || event.button !== 0) return;
    const found = pick(event);
    if (!found) return;
    const index = found.object.userData.index;
    select(index);
    if (!state.keys[index].draggable) return;

    // Take the gesture off OrbitControls, which listens on the same canvas.
    event.stopPropagation();
    event.preventDefault();
    sim.controls.enabled = false;
    drag.index = index;
    drag.origin.copy(found.object.position);
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
      const mesh = markers.children.find((m) => m.userData.index === i);
      if (mesh) mesh.position.copy(drag.origin).add(delta);
    }
  }

  function onPointerUp(event) {
    if (drag.index === null) return;
    const mesh = markers.children.find((m) => m.userData.index === drag.index);
    const delta = mesh.position.clone().sub(drag.origin);
    drag.index = null;
    sim.controls.enabled = true;
    sim.renderer.domElement.releasePointerCapture?.(event.pointerId);
    if (delta.lengthSq() < 1e-10) return;
    move(mesh.userData.index, toMujoco(delta));
    render();
    if (state.auto) solve(); else draw();
  }

  function onKeyDown(event) {
    if (/^(INPUT|TEXTAREA|SELECT)$/.test(event.target.tagName)) return;
    if (event.key === 's' || event.key === 'S') return void solve();
    if (state.selected === null) return;
    if (event.key === 'd' || event.key === 'D') return void toggle(state.selected);
    const key = state.keys[state.selected];
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
    state.selected = index;
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

  function render() {
    const edits = state.module ? editsFor(state.module).keys : {};
    ui.title.textContent = `${state.robot || '—'} · ${state.module || '—'}`;
    ui.rows.replaceChildren();

    for (const key of state.keys) {
      const edit = edits[key.index] || {};
      const row = document.createElement('div');
      row.className = 'row' + (key.index === state.selected ? ' on' : '');
      row.addEventListener('click', (e) => {
        if (e.target === row || e.target.className === 'tag') select(key.index);
      });

      const tag = document.createElement('span');
      tag.className = 'tag';
      tag.textContent = `${key.index}`;
      tag.style.background = key.home ? '#94a3b8'
        : key.kind === 'arc' ? '#ea580c' : '#2563eb';
      row.appendChild(tag);

      const what = document.createElement('span');
      what.className = 'what';
      what.textContent = key.off ? 'off' : key.home ? 'home'
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
        row.appendChild(num(key.secs, 0.1, (v) => { editFor(key.index).secs = v; }));
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
      power.title = key.off ? 'put this keyframe back in the path'
        : 'take this keyframe out of the path (D)';
      power.disabled = key.index === 0;
      power.addEventListener('click', () => toggle(key.index));
      row.appendChild(power);

      if (key.off) row.classList.add('dead');
      ui.rows.appendChild(row);
    }

    const key = state.selected !== null ? state.keys[state.selected] : null;
    const edit = key ? edits[key.index] : null;
    ui.detail.textContent = key
      ? `#${key.index} at ${key.pos.map((v) => v.toFixed(3)).join(', ')}` +
        (edit && edit.dpos
          ? `   moved ${edit.dpos.map((v) => (v * 1000).toFixed(0)).join(', ')} mm`
          : '')
      : 'pick a bead on the path, or a row here';
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
        <div class="detail" id="te-detail"></div>
        <div class="status" id="te-status"></div>
        <div class="buttons">
          <button class="primary" id="te-solve">Solve (S)</button>
          <button id="te-save">Save</button>
          <button id="te-undo">Reset key</button>
          <button id="te-clear">Reset task</button>
        </div>
        <label><input type="checkbox" id="te-auto" checked> re-solve on drop</label>
        <label><input type="checkbox" id="te-link" checked> move stacked beads together</label>
        <label>scrub <input type="range" id="te-scrub" min="0" max="1" value="0"></label>
        <div class="help">
          drag a bead to move that keyframe · shift = straight up and down ·
          ctrl = along the board · arrows nudge the selected bead 10 mm
          (shift 1 mm, alt = towards the camera) · × takes a keyframe out of
          the path (D), + puts it back · orange beads are arcs: they follow
          the joint, so change their angle instead
        </div>
      </div>`;

    return {
      root,
      title: root.querySelector('#te-title'),
      rows: root.querySelector('#te-rows'),
      detail: root.querySelector('#te-detail'),
      status: root.querySelector('#te-status'),
      scrub: root.querySelector('#te-scrub'),
    };
  }

  ui.root.querySelector('#te-solve').addEventListener('click', () => solve());
  ui.root.querySelector('#te-save').addEventListener('click', () => save());
  ui.root.querySelector('#te-auto').addEventListener('change', (e) => {
    state.auto = e.target.checked;
  });
  ui.root.querySelector('#te-link').addEventListener('change', (e) => {
    state.link = e.target.checked;
  });
  ui.root.querySelector('#te-undo').addEventListener('click', () => {
    if (state.selected === null) return;
    for (const i of stacked(state.selected)) delete editsFor(state.module).keys[i];
    solve();
  });
  ui.root.querySelector('#te-clear').addEventListener('click', () => {
    state.edits[state.module] = { keys: {} };
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

  // The widget owns which robot and task are showing, and says so only by
  // changing what it is playing -- so watch rather than reach in and hook it.
  setInterval(() => {
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
    } else if (robot && module && !state.loaded && !state.busy) {
      // The solver was not up when this task came round. Keep trying, so
      // starting it is all it takes -- no reloading the page.
      if (Date.now() - lastTry > 3000) reload();
    }
    if (sim.task && drag.index === null && document.activeElement !== ui.scrub) {
      ui.scrub.max = sim.task.qpos.length - 1;
      ui.scrub.value = sim.sample;
    }
  }, 250);

  render();
  status(`editor ready — ${API}`);
}
