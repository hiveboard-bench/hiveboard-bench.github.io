import assert from 'node:assert/strict';
import { readFileSync, existsSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { fileURLToPath } from 'node:url';
import { test } from 'node:test';

const root = new URL('../', import.meta.url);
const read = (path) => readFileSync(new URL(path, root));
const json = (path) => JSON.parse(read(path));
const models = 'public/sim/models/';

test('every mesh referenced by the viewer is shipped with its compressed fallback', () => {
  const paths = [...read('viewer.js').toString().matchAll(/['"]\.\/(models\/[^'"]+\.(?:stl|obj))['"]/g)];
  assert.ok(paths.length >= 30);
  for (const [, path] of paths) {
    const raw = read(`public/${path}`);
    if (path.endsWith('.stl')) {
      assert.deepEqual(gunzipSync(read(`public/${path}.gz`)), raw, path);
    }
  }
});

test('all simulation archives match the source files used by local tools', () => {
  const manifest = json(models + 'manifest.json');
  assert.equal(new Set(manifest).size, manifest.length);
  for (const name of ['manifest.json', ...manifest]) {
    assert.ok(!name.includes('\\') && !name.includes('..'), name);
    assert.deepEqual(gunzipSync(read(models + name + '.gz')), read(models + name), name);
  }
});

test('the catalogue retains all platforms and their published trajectories', () => {
  const robots = json(models + 'robots.json');
  assert.deepEqual(robots.map(r => r.name).sort(), ['anymal', 'fr3', 'macao', 'so101', 'spot']);
  const manifest = new Set(json(models + 'manifest.json'));
  for (const robot of robots) {
    assert.ok(manifest.has(robot.scene) && manifest.has(robot.traj), robot.name);
    const trajectories = json(models + robot.traj);
    for (const name of robot.tasks) {
      const task = trajectories[name];
      assert.ok(task?.qpos.length > 1, `${robot.name}/${name}`);
      assert.equal(task.qpos.length, task.grip.length);
      assert.equal(task.qpos.length, task.tcp.length);
      for (const pose of task.qpos) {
        assert.equal(pose.length, robot.arm);
        assert.ok(pose.every(Number.isFinite));
      }
    }
    for (const ext of ['urdf', 'xrdf']) {
      const path = new URL(`tools/cumotion_resources/${robot.name}_cumotion/${robot.name}.${ext}`, root);
      assert.ok(existsSync(path), fileURLToPath(path));
    }
  }
});
