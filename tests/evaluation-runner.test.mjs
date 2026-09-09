import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import vm from 'node:vm'
import { computed, reactive, ref, watch, nextTick } from 'vue'
import JSZip from 'jszip'

// Exercise the actual component functions with a controllable clock and downloads.
// No browser rendering is needed for these data-integrity regressions.
const source = readFileSync(new URL('../docs/.vitepress/theme/components/EvaluationRunner.vue', import.meta.url), 'utf8')
  .split('<script setup>')[1].split('</script>')[0].replace(/^import .*$/gm, '')
function runner() {
  let now = 1000
  const downloads = []
  const context = vm.createContext({
    computed, reactive, ref, JSZip, Blob, Uint8Array, console,
    onMounted() {}, onUnmounted() {}, watch,
    performance: { now: () => now },
    window: { setInterval: () => 1, clearInterval() {}, setTimeout: fn => fn(), confirm: () => true, localStorage: { removeItem() {} } },
    document: { createElement: () => ({ click() {} }) },
    URL: class extends URL { static createObjectURL(blob) { downloads.push(blob); return 'blob:test' } static revokeObjectURL() {} }
  })
  vm.runInContext(source, context)
  const api = vm.runInContext(`({session, trials, tasks, timerState, elapsedMs, trialForm, selectedTaskId, step,
    submissionReady, metadataComplete, setupPhoto, recordErrors, readSession, readSetup,
    importSession, startTimer, startCountdown, updateTimerOnce, finishTiming, resetTimer,
    saveTrial, error, sessionPayload, profilePayload, downloadPackage, manifestPayload,
    handleKey, exampleVideoTask, trialPending, openTask, contact, publicationConsent, evidenceConfirmed,
    submissionDraft, submissionMailto, prepareSubmissionEmail, submissionEmailErrors, validPackageUrl})`, context)
  Object.assign(api.session, {
    submission_id: 'lab_robot_20260907_abcdef', evaluation_mode: 'physical', lab_id: 'lab', platform_id: 'robot',
    robot_model: 'FR3', end_effector: '2F85', control_method: 'Teleoperation', board_orientation: 'horizontal',
    hiveboard_version: 'test-version', date: '2026-09-07', printer: 'Test printer', material: 'PLA',
    print_settings: '0.2 mm layers, 25% infill', post_processing: 'None'
  })
  return { ...api, clock: value => { now = value }, downloads }
}
function record(r, task = r.tasks[0], id = 1) {
  return { trial_id: id, lab_id: 'lab', platform_id: 'robot', attachment_id: task.id, date: '2026-09-07',
    outcome: 'success', failure_cause: '', completion_time_s: '10.00', n_attempts: 1, n_regrasps: 0,
    stage_reached: task.stages ? task.stages.length : '', strategy: 'prehensile', notes: '' }
}
function fill(r) {
  r.trials.value = r.tasks.flatMap((task, index) => Array.from({length: 5}, (_, i) => record(r, task, index * 5 + i + 1)))
}

test('completion time remains fixed after stopping, switching tabs, and saving', () => {
  const r = runner()
  r.startTimer()
  r.clock(6500)
  r.finishTiming()
  assert.equal(r.elapsedMs.value, 5500)
  r.clock(29000)
  r.updateTimerOnce()
  assert.equal(r.elapsedMs.value, 5500)
  r.saveTrial()
  assert.equal(r.trials.value[0].completion_time_s, '5.50')
})

test('late manual stop is a timeout even if the interval callback was delayed', () => {
  const r = runner()
  r.startTimer()
  r.clock(81000)
  r.finishTiming()
  assert.equal(r.trialForm.outcome, 'timeout')
  assert.equal(r.elapsedMs.value, 60000)
  r.trialForm.failure_cause = 'slip'
  r.saveTrial()
  assert.equal(r.trials.value[0].completion_time_s, '')
})

test('navigation cannot discard an active or unrecorded trial; video Space cannot start the timer', () => {
  const r = runner()
  r.step.value = 'trial'
  r.exampleVideoTask.value = r.tasks[0]
  r.handleKey({ code: 'Space', target: {tagName: 'VIDEO'}, preventDefault() {} })
  assert.equal(r.timerState.value, 'idle')
  r.startTimer()
  r.openTask(r.tasks[1].id)
  assert.equal(r.selectedTaskId.value, r.tasks[0].id)
  r.finishTiming()
  assert.equal(r.trialPending.value, true)
  r.openTask(r.tasks[1].id)
  assert.equal(r.selectedTaskId.value, r.tasks[0].id)
})

test('validation rejects missing, fractional, non-finite, and out-of-range entries', () => {
  const r = runner()
  const good = record(r)
  assert.equal(r.recordErrors(good).length, 0)
  for (const patch of [
    { n_attempts: undefined }, { n_regrasps: '' }, { n_regrasps: null }, { n_attempts: 1.5 },
    { n_regrasps: -1 }, { n_attempts: 'abc' }, { n_attempts: true }, { trial_id: 0 },
    { completion_time_s: '' }, { completion_time_s: null }, { completion_time_s: Infinity },
    { completion_time_s: 61 }, { strategy: '' }, { strategy: 'unknown' },
    { failure_cause: 'slip' }, { platform_id: 'other' }, { date: '2026-02-30' }
  ]) assert.ok(r.recordErrors({...good, ...patch}).length, JSON.stringify(patch))
  const task = r.tasks.find(task => task.stages)
  for (const stage of ['', undefined, null, 'abc', 0.5, task.stages.length - 1]) {
    assert.ok(r.recordErrors({...record(r, task), stage_reached: stage}).length)
  }
  const failed = {...good, outcome: 'fail', completion_time_s: '', failure_cause: 'slip'}
  assert.equal(r.recordErrors(failed).length, 0)
  assert.ok(r.recordErrors({...failed, failure_cause: 'other'}).length)
  assert.ok(r.recordErrors({...failed, failure_cause: 'unknown'}).length)
})

test('saving applies the same validation as imports and rejects empty regrasp counts', () => {
  const r = runner()
  r.startTimer()
  r.clock(3000)
  r.finishTiming()
  r.trialForm.n_regrasps = ''
  r.saveTrial()
  assert.equal(r.trials.value.length, 0)
  assert.match(r.error.value, /Regrasps/)
})

test('all 13 conditions and exactly five valid trials are needed for the ZIP', () => {
  const r = runner()
  fill(r)
  assert.equal(r.submissionReady.value, true)
  const last = r.trials.value.pop()
  assert.equal(r.submissionReady.value, false)
  r.trials.value.push(last)
  r.trials.value.push({...last, trial_id: 66})
  assert.equal(r.submissionReady.value, false)
  r.trials.value.pop()
  r.trials.value[0].strategy = ''
  assert.equal(r.submissionReady.value, false)
  r.trials.value[0].strategy = 'prehensile'
  r.session.printer = ''
  assert.equal(r.submissionReady.value, false)
})

test('invalid session import is atomic; supported old backups retain their IDs and records', async () => {
  const r = runner()
  fill(r)
  const original = JSON.stringify(r.sessionPayload())
  const invalid = JSON.parse(original)
  invalid.trials[0].completion_time_s = 900
  await r.importSession({target: {value: 'test', files: [{text: async () => JSON.stringify(invalid)}]}})
  assert.equal(r.trials.value[0].completion_time_s, '10.00')
  assert.match(r.error.value, /Session not imported/)
  const old = JSON.parse(original)
  for (const key of ['printer', 'material', 'print_settings', 'post_processing', 'calibration_notes']) delete old.session[key]
  const restored = r.readSession(old)
  assert.equal(restored.records.length, 65)
  assert.equal(restored.imported.submission_id, r.session.submission_id)
  assert.equal(restored.imported.printer, '')
  for (const mutate of [d => d.schema_version = '99', d => d.session.lab_id = {},
    d => d.session.submission_id = '../other', d => d.trials[1].trial_id = 1]) {
    const data = JSON.parse(original); mutate(data)
    assert.throws(() => r.readSession(data))
  }
})

test('ZIP contains consistent records, printing details, and truthful supporting-file status', async () => {
  const r = runner()
  fill(r)
  await r.downloadPackage()
  const zip = await JSZip.loadAsync(await r.downloads[0].arrayBuffer())
  const prefix = r.session.submission_id + '/'
  const manifest = JSON.parse(await zip.file(prefix + 'manifest.json').async('string'))
  assert.equal(manifest.summary.total_trials, 65)
  assert.equal(manifest.supporting_files.setup_photo.included, false)
  assert.equal(manifest.supporting_files.recordings_included, false)
  assert.equal(manifest.recordings.length, 65)
  assert.equal(new Set(manifest.recordings.map(x => x.expected_filename)).size, 65)
  assert.equal((await zip.file(prefix + 'trials.csv').async('string')).trim().split('\n').length, 66)
  assert.match(await zip.file(prefix + 'platform.md').async('string'), /Test printer/)
  const restored = JSON.parse(await zip.file(prefix + 'session.json').async('string'))
  assert.equal(r.readSession(restored).records.length, 65)
  assert.equal(zip.file(prefix + 'setup.jpg'), null)
  r.setupPhoto.value = new Blob([new Uint8Array([255, 216, 255, 217])], {type: 'image/jpeg'})
  await r.downloadPackage()
  const withPhoto = await JSZip.loadAsync(await r.downloads[1].arrayBuffer())
  assert.ok(withPhoto.file(prefix + 'setup.jpg'))
  const withPhotoManifest = JSON.parse(await withPhoto.file(prefix + 'manifest.json').async('string'))
  assert.equal(withPhotoManifest.supporting_files.setup_photo.included, true)
})

test('email requires a complete evaluation, contact, evidence and explicit permission', async () => {
  const r = runner()
  fill(r)
  Object.assign(r.contact, { name: 'Researcher & Co', email: 'private@example.org', institution: 'Test institute', package_url: 'https://example.org/package?token=private&part=1' })
  await nextTick()
  r.prepareSubmissionEmail()
  assert.equal(r.submissionDraft.value, '')
  r.publicationConsent.value = true
  r.evidenceConfirmed.value = true
  await nextTick()
  r.prepareSubmissionEmail()
  assert.match(r.submissionDraft.value, /Trial records: 65/)
  assert.match(r.submissionDraft.value, /Publication permission: I authorize/)
  const mailto = new URL(r.submissionMailto.value)
  assert.equal(mailto.pathname, 'ricardo.godoy@usp.br')
  assert.equal(mailto.searchParams.get('body'), r.submissionDraft.value)
  r.trials.value.pop()
  await nextTick()
  assert.equal(r.submissionDraft.value, '')
  assert.equal(r.publicationConsent.value, false)
  r.prepareSubmissionEmail()
  assert.equal(r.submissionDraft.value, '')
  for (const link of ['javascript:alert(1)', 'http://example.org', 'https://user:pass@example.org', 'invalid']) assert.equal(r.validPackageUrl(link), false)
})

test('private contact and package links never enter exported sessions, profiles or ZIP', async () => {
  const r = runner()
  fill(r)
  Object.assign(r.contact, { name: 'Private contact marker', email: 'private@example.org', institution: 'Private institution marker', package_url: 'https://example.org/private-package-marker' })
  for (const payload of [r.sessionPayload(), r.profilePayload(), r.manifestPayload()]) {
    assert.doesNotMatch(JSON.stringify(payload), /private@example|Private contact marker|private-package-marker|Private institution marker/)
  }
  await r.downloadPackage()
  const zip = await JSZip.loadAsync(await r.downloads[0].arrayBuffer())
  for (const file of Object.values(zip.files).filter(file => !file.dir)) {
    assert.doesNotMatch(await file.async('string'), /private@example|Private contact marker|private-package-marker|Private institution marker/)
  }
  assert.match(await zip.file(r.session.submission_id + '/submission-instructions.md').async('string'), /Opening a draft does not send it/)
})
