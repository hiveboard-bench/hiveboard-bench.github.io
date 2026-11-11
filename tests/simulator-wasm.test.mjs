import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { dirname } from 'node:path';
import { gunzipSync } from 'node:zlib';
import { test } from 'node:test';
import loadMujoco from '../public/sim/vendor/mujoco.js';

const models = new URL('../public/sim/models/', import.meta.url);
const readArchive = (name) => gunzipSync(readFileSync(new URL(name + '.gz', models)));

test('the shipped browser MuJoCo engine loads and steps every published robot', async () => {
  const mujoco = await loadMujoco({
    wasmBinary: gunzipSync(readFileSync(new URL('../public/sim/vendor/mujoco.wasm.gz', import.meta.url))),
  });
  for (const name of JSON.parse(readArchive('manifest.json'))) {
    const path = '/working/' + name;
    mujoco.FS.mkdirTree(dirname(path));
    mujoco.FS.writeFile(path, readArchive(name));
  }
  for (const robot of JSON.parse(readArchive('robots.json'))) {
    const model = mujoco.MjModel.from_xml_path('/working/' + robot.scene);
    const data = new mujoco.MjData(model);
    try {
      mujoco.mj_resetDataKeyframe(model, data, 0);
      mujoco.mj_forward(model, data);
      for (let i = 0; i < 5; i++) mujoco.mj_step(model, data);
      assert.ok(Array.from(data.qpos).every(Number.isFinite), robot.name);
      assert.ok(Array.from(data.xpos).every(Number.isFinite), robot.name);
      assert.ok(model.nq > robot.arm, robot.name);
    } finally {
      data.delete();
      model.delete();
    }
  }
});
