<script setup>
import { computed, onMounted, onUnmounted, reactive, ref, watch } from 'vue'
import { withBase } from 'vitepress'
import JSZip from 'jszip'

const STORAGE_KEY = 'hiveboard-evaluation-runner-v1'
const REQUIRED_TRIALS = 5
const SCHEMA_VERSION = '1.0'
const PROFILE_FIELDS = [
  'evaluation_mode', 'lab_id', 'platform_id', 'robot_model', 'end_effector',
  'control_method', 'board_orientation', 'hiveboard_version',
  'printer', 'material', 'print_settings', 'post_processing', 'calibration_notes'
]
const CSV_COLUMNS = [
  'trial_id', 'lab_id', 'platform_id', 'attachment_id', 'date', 'outcome',
  'failure_cause', 'completion_time_s', 'n_attempts', 'n_regrasps',
  'stage_reached', 'strategy', 'notes'
]
const VIDEO_BASE = 'https://github.com/hiveboard-bench/hiveboard-bench.github.io/releases/download/v1.0-v1.0-videos'

const tasks = [
  {
    id: 'valve_ball', name: 'Ball valve', family: 'Torque', timeout: 60,
    image: '/images/tasks/torque_valve_noFriction_3d.png',
    video: `${VIDEO_BASE}/s010_valve_ball.mp4`, videoPlatform: 'Platform B · LeRobot SO-101',
    success: 'Rotate the handle 90° from the closed state to the open state.',
    reset: 'Return the handle to the closed position and confirm that the attachment is fully seated.'
  },
  {
    id: 'valve_ball_ring', name: 'Ball valve with friction ring', family: 'Torque', timeout: 90,
    image: '/images/tasks/torque_valve_noFriction_3d.png',
    video: `${VIDEO_BASE}/anymal_valve_lever_hard_vr.mp4`, videoPlatform: 'Platform C · ANYmal with DynaArm (VR)',
    success: 'Rotate the handle 90° from closed to open with the friction ring fitted.',
    reset: 'Fit the friction ring, return the handle to closed, and confirm that the attachment is seated.'
  },
  {
    id: 'valve_gate_small', name: 'Small gate valve', family: 'Torque', timeout: 90,
    image: '/images/tasks/small_valve_3d.png',
    video: `${VIDEO_BASE}/spot_small_valve.mp4`, videoPlatform: 'Platform A · Spot with Spot Arm',
    success: 'Complete one full turn of the valve stem.',
    reset: 'Return the stem to the marked initial orientation without changing the board position.'
  },
  {
    id: 'valve_gate_large', name: 'Large gate valve', family: 'Torque', timeout: 120,
    image: '/images/tasks/big_valve_3d.png',
    video: `${VIDEO_BASE}/spot_big_valve.mp4`, videoPlatform: 'Platform A · Spot with Spot Arm',
    success: 'Complete one full turn of the valve stem.',
    reset: 'Return the stem to the marked initial orientation without changing the board position.'
  },
  {
    id: 'circuit_breaker', name: 'Circuit breaker', family: 'Torque', timeout: 60,
    image: '/images/tasks/switch_3d.png',
    video: `${VIDEO_BASE}/spot_circuit_breaker.mp4`, videoPlatform: 'Platform A · Spot with Spot Arm',
    success: 'Move the toggle to the opposite state and hold it there.',
    reset: 'Return the toggle to its initial state and confirm that it moves freely.'
  },
  {
    id: 'light_bulb', name: 'Light bulb and socket', family: 'Precision', timeout: 120,
    image: '/images/tasks/lamp_3d.png',
    video: `${VIDEO_BASE}/spot_light_bulb.mp4`, videoPlatform: 'Platform A · Spot with Spot Arm',
    success: 'Thread the bulb into the socket until it is seated.',
    reset: 'Remove the bulb, restore the documented starting pose, and inspect the thread.'
  },
  {
    id: 'thread_m8', name: 'M8 threaded fastener', family: 'Precision', timeout: 120,
    image: '/images/tasks/m8_3d.png',
    video: `${VIDEO_BASE}/s010_m8_exp5.mp4`, videoPlatform: 'Platform B · LeRobot SO-101',
    success: 'Thread the bolt along the available length.',
    reset: 'Return the bolt to the documented initial engagement and check that the thread is clear.'
  },
  {
    id: 'thread_m30', name: 'M30 threaded fastener', family: 'Precision', timeout: 120,
    image: '/images/tasks/m30_3d.png',
    video: `${VIDEO_BASE}/s010_m30_exp5.mp4`, videoPlatform: 'Platform B · LeRobot SO-101',
    success: 'Thread the bolt along the available length.',
    reset: 'Return the bolt to the documented initial engagement and check that the thread is clear.'
  },
  {
    id: 'peg_insertion', name: 'Threaded peg insertion', family: 'Precision', timeout: 120,
    image: '/images/tasks/peg_and_hole_3d.png',
    video: `${VIDEO_BASE}/macao_peg_and_hole.mp4`, videoPlatform: 'Platform D · Macao prosthetic hand',
    success: 'Thread the free 8 mm peg into the empty socket until it is seated.',
    reset: 'Remove the peg and return it to the initial pose next to the empty socket.'
  },
  {
    id: 'button', name: 'Covered button', family: 'Composed assembly', timeout: 60,
    image: '/images/tasks/button_3d.png',
    video: `${VIDEO_BASE}/spot_button.mp4`, videoPlatform: 'Platform A · Spot with Spot Arm',
    success: 'Open the cover and press the button.',
    reset: 'Close the cover and confirm that the button has returned.',
    stages: ['Open cover', 'Press button']
  },
  {
    id: 'lock', name: 'Lock and key', family: 'Composed assembly', timeout: 180,
    image: '/images/tasks/key_3d.png',
    video: `${VIDEO_BASE}/spot_key.mp4`, videoPlatform: 'Platform A · Spot with Spot Arm',
    success: 'Grasp the key, insert it vertically, and rotate it to unlock.',
    reset: 'Remove the key, return the lock to its initial state, and restore the key pose.',
    stages: ['Grasp key', 'Insert key vertically', 'Rotate to unlock']
  },
  {
    id: 'drawer', name: 'Sliding drawer', family: 'Composed assembly', timeout: 120,
    image: '/images/tasks/box_3d.png',
    video: `${VIDEO_BASE}/macao_box.mp4`, videoPlatform: 'Platform D · Macao prosthetic hand',
    success: 'Grasp the handle, pull the drawer open, and push it closed.',
    reset: 'Return the drawer to the fully closed initial position.',
    stages: ['Grasp handle', 'Pull open', 'Push closed']
  },
  {
    id: 'shock_absorber', name: 'Shock absorber', family: 'Composed assembly', timeout: 180,
    image: '/images/tasks/spring_3d.png',
    video: `${VIDEO_BASE}/macao_spring.mp4`, videoPlatform: 'Platform D · Macao prosthetic hand',
    success: 'Grasp the pin, align it with the hole, and insert it fully.',
    reset: 'Remove the pin, restore its starting pose, and check both occupied board cells.',
    stages: ['Grasp pin', 'Align with hole', 'Insert fully']
  }
]

const emptySession = () => ({
  submission_id: '',
  evaluation_mode: 'physical',
  lab_id: '',
  platform_id: '',
  robot_model: '',
  end_effector: '',
  control_method: '',
  board_orientation: 'horizontal',
  hiveboard_version: '',
  printer: '',
  material: '',
  print_settings: '',
  post_processing: '',
  calibration_notes: '',
  date: ''
})

const emptyTrialForm = () => ({
  outcome: 'success',
  failure_cause: '',
  n_attempts: 1,
  n_regrasps: 0,
  stage_reached: '',
  strategy: 'prehensile',
  notes: ''
})

const step = ref('setup')
const session = reactive(emptySession())
const selectedTaskId = ref(tasks[0].id)
const trials = ref([])
const timerState = ref('idle')
const countdown = ref(5)
const elapsedMs = ref(0)
const trialForm = reactive(emptyTrialForm())
const error = ref('')
const exampleVideoTask = ref(null)
const profileInput = ref(null)
const sessionInput = ref(null)
const transferMessage = ref('')
const transferIsError = ref(false)
const packageState = ref('idle')
const mounted = ref(false)
const setupPhoto = ref(null)
const storageError = ref('')
const submissionOpen = ref(false)
const contact = reactive({ name: '', email: '', institution: '', package_url: '' })
const publicationConsent = ref(false)
const evidenceConfirmed = ref(false)
const submissionMessage = ref('')
const submissionDraft = ref('')
const ORGANIZER_EMAIL = 'ricardo.godoy@usp.br'
const PUBLICATION_PERMISSION = 'I authorize the HiveBoard organizers to publish the approved trial data, platform description, setup photograph, and trial recordings. I have permission to share these materials, including permission from identifiable people in the photographs or videos. My contact name and email are for organizer correspondence and must not be published.'

function validPackageUrl(value) {
  try {
    const url = new URL(value)
    return url.protocol === 'https:' && Boolean(url.hostname) && !url.username && !url.password
  } catch (_) { return false }
}

function submissionEmailErrors() {
  const issues = []
  if (!submissionReady.value) issues.push('Complete all 65 valid trial records and required platform details.')
  if (!hasText(contact.name)) issues.push('Enter a contact name.')
  if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(contact.email)) issues.push('Enter a valid contact email.')
  if (!hasText(contact.institution)) issues.push('Enter your institution, or Independent if unaffiliated.')
  if (!validPackageUrl(contact.package_url)) issues.push('Enter an HTTPS download link for the completed package.')
  if (!evidenceConfirmed.value) issues.push('Confirm that the shared package includes setup.jpg and all 65 recordings.')
  if (!publicationConsent.value) issues.push('Permission to publish approved results is required for submission. You can still download your results without granting permission.')
  return issues
}

function prepareSubmissionEmail() {
  submissionDraft.value = ''
  const issues = submissionEmailErrors()
  submissionMessage.value = issues.join(' ')
  if (issues.length) return
  submissionDraft.value = `Hello Ricardo,

Please review my HiveBoard evaluation.

Submission ID: ${ensureSubmissionId()}
Contact name: ${contact.name.trim()}
Contact email: ${contact.email.trim()}
Institution: ${contact.institution.trim()}
Robot: ${session.robot_model}
End-effector: ${session.end_effector}
Control interface: ${session.control_method}
Evaluation mode: ${session.evaluation_mode}
Board orientation: ${session.board_orientation}
Evaluation date: ${session.date}
Trial records: 65 (13 conditions, five trials each)

Package link: ${contact.package_url.trim()}

I confirm that this package includes setup.jpg and all 65 trial recordings with the filenames specified by the runner.

Publication permission: ${PUBLICATION_PERMISSION}
Permission recorded at: ${new Date().toISOString()}

Please contact me if corrections are needed. I understand that submission does not imply approval and that only approved results will be published.
`
}

const submissionMailto = computed(() => `mailto:${ORGANIZER_EMAIL}?subject=${encodeURIComponent(`HiveBoard evaluation ${session.submission_id}`)}&body=${encodeURIComponent(submissionDraft.value)}`)

function resetSubmission() {
  submissionOpen.value = false
  Object.assign(contact, { name: '', email: '', institution: '', package_url: '' })
  publicationConsent.value = false
  evidenceConfirmed.value = false
  submissionMessage.value = ''
  submissionDraft.value = ''
}
let ticker = null
let startMark = 0
let countdownEnd = 0

const currentTask = computed(() => tasks.find(task => task.id === selectedTaskId.value) || tasks[0])
const currentTrials = computed(() => trials.value.filter(trial => trial.attachment_id === currentTask.value.id))
const currentTrialNumber = computed(() => Math.min(currentTrials.value.length + 1, REQUIRED_TRIALS))
const elapsedSeconds = computed(() => elapsedMs.value / 1000)
const displayTime = computed(() => formatTime(elapsedMs.value))
const hasText = value => typeof value === 'string' && value.trim().length > 0
const isNumber = value => (typeof value === 'number' || hasText(value)) && Number.isFinite(Number(value))
const isCount = (value, minimum) => isNumber(value) && Number.isInteger(Number(value)) && Number(value) >= minimum
const validDate = value => typeof value === 'string' && /^\d{4}-\d{2}-\d{2}$/.test(value) && Number.isFinite(Date.parse(value)) && new Date(value).toISOString().slice(0, 10) === value
function validSetup(value) {
  return ['lab_id', 'platform_id', 'robot_model', 'end_effector', 'control_method', 'hiveboard_version'].every(field => hasText(value[field])) &&
    ['lab_id', 'platform_id'].every(field => /^[a-z0-9_]+$/.test(value[field])) &&
    ['physical', 'simulation'].includes(value.evaluation_mode) && ['horizontal', 'vertical'].includes(value.board_orientation)
}
const sessionReady = computed(() => validSetup(session) && validDate(session.date))
const profileReady = computed(() => validSetup(session))
const metadataComplete = computed(() => sessionReady.value && (session.evaluation_mode !== 'physical' ||
  ['printer', 'material', 'print_settings', 'post_processing'].every(field => hasText(session[field]))))
const trialPending = computed(() => ['running', 'countdown', 'form'].includes(timerState.value))

function recordErrors(trial, setup = session) {
  if (!trial || typeof trial !== 'object') return ['Invalid trial record.']
  const issues = []
  const task = tasks.find(candidate => candidate.id === trial.attachment_id)
  if (!task) return ['Unknown attachment.']
  if (!isCount(trial.trial_id, 1)) issues.push('Trial ID must be a positive integer.')
  if (!['success', 'fail', 'timeout', 'safety_stop'].includes(trial.outcome)) issues.push('Select a trial outcome.')
  if (!isCount(trial.n_attempts, 1)) issues.push('Attempts must be an integer of at least 1.')
  if (!isCount(trial.n_regrasps, 0)) issues.push('Regrasps must be a non-negative integer.')
  if (!['prehensile', 'non_prehensile'].includes(trial.strategy)) issues.push('Select a manipulation strategy.')
  if (trial.outcome === 'success') {
    if (!isNumber(trial.completion_time_s) || Number(trial.completion_time_s) < 0 || Number(trial.completion_time_s) > task.timeout) issues.push(`Completion time must be between 0 and ${task.timeout} s.`)
    if (trial.failure_cause !== '') issues.push('Successful trials must have a blank failure cause.')
  } else {
    if (!['grasp_geometry', 'kinematic_limit', 'perception', 'slip', 'force_limit', 'control_precision', 'other'].includes(trial.failure_cause)) issues.push('Select a primary failure cause.')
    if (trial.completion_time_s !== '') issues.push('Unsuccessful trials must have a blank completion time.')
    if (trial.failure_cause === 'other' && !hasText(trial.notes)) issues.push('Explain the failure in Notes.')
  }
  if (task.stages) {
    if (!isCount(trial.stage_reached, 0) || Number(trial.stage_reached) > task.stages.length) issues.push('Enter the last completed stage.')
    else if (trial.outcome === 'success' && Number(trial.stage_reached) !== task.stages.length) issues.push('Success requires completion of every stage.')
  } else if (trial.stage_reached !== '') issues.push('Leave stage blank for this task.')
  if (typeof trial.notes !== 'string') issues.push('Notes must be text.')
  if (!validDate(trial.date) || trial.date !== setup.date || trial.lab_id !== setup.lab_id || trial.platform_id !== setup.platform_id) issues.push('Trial date and platform identifiers must match the session.')
  return issues
}
const invalidRecords = computed(() => trials.value.flatMap(trial => recordErrors(trial).map(message => `Trial ${trial?.trial_id ?? '?'}: ${message}`)))
const taskTrialCounts = computed(() => Object.fromEntries(
  tasks.map(task => [task.id, trials.value.filter(trial => trial.attachment_id === task.id).length])
))
const validationChecks = computed(() => {
  const trialIds = trials.value.map(trial => Number(trial.trial_id))
  return [
    { label: 'Experimental setup recorded', passed: metadataComplete.value },
    { label: `All ${tasks.length} conditions recorded`, passed: tasks.every(task => taskTrialCounts.value[task.id] > 0) },
    { label: `${REQUIRED_TRIALS} trials recorded for every condition`, passed: tasks.every(task => taskTrialCounts.value[task.id] === REQUIRED_TRIALS) },
    { label: 'Trial IDs are unique', passed: trialIds.length > 0 && new Set(trialIds).size === trialIds.length },
    { label: 'Trial entries valid', passed: trials.value.length > 0 && invalidRecords.value.length === 0 }
  ]
})
const submissionReady = computed(() => validationChecks.value.every(check => check.passed))

function formatTime(ms) {
  const total = Math.max(0, ms) / 1000
  const minutes = Math.floor(total / 60)
  const seconds = (total % 60).toFixed(2).padStart(5, '0')
  return `${String(minutes).padStart(2, '0')}:${seconds}`
}

function clearTicker() {
  if (ticker) window.clearInterval(ticker)
  ticker = null
}

function beep(frequency = 660, duration = 0.08) {
  try {
    const AudioContext = window.AudioContext || window.webkitAudioContext
    const context = new AudioContext()
    const oscillator = context.createOscillator()
    const gain = context.createGain()
    oscillator.frequency.value = frequency
    gain.gain.setValueAtTime(0.08, context.currentTime)
    gain.gain.exponentialRampToValueAtTime(0.001, context.currentTime + duration)
    oscillator.connect(gain).connect(context.destination)
    oscillator.start()
    oscillator.stop(context.currentTime + duration)
    oscillator.addEventListener('ended', () => context.close())
  } catch (_) {}
}

function ensureSubmissionId() {
  if (session.submission_id) return session.submission_id
  const datePart = (session.date || new Date().toISOString().slice(0, 10)).replaceAll('-', '')
  const randomPart = typeof crypto !== 'undefined' && crypto.getRandomValues
    ? Array.from(crypto.getRandomValues(new Uint8Array(3)), value => value.toString(16).padStart(2, '0')).join('')
    : Math.random().toString(16).slice(2, 8).padEnd(6, '0')
  session.submission_id = `${session.lab_id.trim()}_${session.platform_id.trim()}_${datePart}_${randomPart}`
  return session.submission_id
}

function startSession() {
  error.value = ''
  session.lab_id = session.lab_id.trim()
  session.platform_id = session.platform_id.trim()
  if (!sessionReady.value) {
    error.value = 'Complete all required session fields before continuing.'
    return
  }
  const identifierPattern = /^[a-z0-9_]+$/
  if (!identifierPattern.test(session.lab_id.trim()) || !identifierPattern.test(session.platform_id.trim())) {
    error.value = 'Laboratory ID and platform ID may contain only lowercase letters, numbers, and underscores.'
    return
  }
  ensureSubmissionId()
  step.value = 'task'
}

function openTask(taskId) {
  if (trialPending.value || !sessionReady.value) return
  selectedTaskId.value = taskId
  resetTimer()
  step.value = 'trial'
}

function startCountdown() {
  if (timerState.value !== 'idle' || !sessionReady.value) return
  ensureSubmissionId()
  if (currentTrials.value.length >= REQUIRED_TRIALS) {
    step.value = 'review'
    return
  }
  error.value = ''
  countdown.value = 5
  timerState.value = 'countdown'
  countdownEnd = performance.now() + 5000
  beep(520)
  clearTicker()
  ticker = window.setInterval(() => {
    countdown.value = Math.max(0, Math.ceil((countdownEnd - performance.now()) / 1000))
    if (countdown.value > 0) {
      beep(520)
    } else {
      clearTicker()
      startTimer()
    }
  }, 1000)
}

function startTimer() {
  timerState.value = 'running'
  elapsedMs.value = 0
  startMark = performance.now()
  beep(880, 0.16)
  ticker = window.setInterval(updateTimer, 50)
}

function updateTimer() {
  if (timerState.value !== 'running') return
  elapsedMs.value = performance.now() - startMark
  if (elapsedSeconds.value >= currentTask.value.timeout) {
    elapsedMs.value = currentTask.value.timeout * 1000
    finishTiming(true)
  }
}

function finishTiming(timedOut = false) {
  if (timerState.value !== 'running') return
  updateTimerOnce()
  clearTicker()
  timedOut = timedOut || elapsedSeconds.value >= currentTask.value.timeout
  if (timedOut) elapsedMs.value = currentTask.value.timeout * 1000
  startMark = 0
  Object.assign(trialForm, emptyTrialForm(), { outcome: timedOut ? 'timeout' : 'success' })
  timerState.value = 'form'
  beep(timedOut ? 360 : 740, 0.18)
}

function updateTimerOnce() {
  if (timerState.value === 'running') elapsedMs.value = Math.min(performance.now() - startMark, currentTask.value.timeout * 1000)
}

function cancelCountdown() {
  clearTicker()
  timerState.value = 'idle'
  countdown.value = 5
}

function resetTimer() {
  clearTicker()
  timerState.value = 'idle'
  countdown.value = 5
  elapsedMs.value = 0
  startMark = 0
  Object.assign(trialForm, emptyTrialForm())
  error.value = ''
}

function saveTrial() {
  error.value = ''
  if (timerState.value !== 'form' || !sessionReady.value || currentTrials.value.length >= REQUIRED_TRIALS) return
  const unsuccessful = trialForm.outcome !== 'success'
  if (unsuccessful && !trialForm.failure_cause) {
    error.value = 'Select one primary failure cause.'
    return
  }
  if (currentTask.value.stages && trialForm.stage_reached === '') {
    error.value = 'Record the last completed stage, including 0 if no stage was completed.'
    return
  }
  const nextId = trials.value.reduce((max, trial) => Math.max(max, Number(trial.trial_id)), 0) + 1
  const record = {
    trial_id: nextId,
    lab_id: session.lab_id.trim(),
    platform_id: session.platform_id.trim(),
    attachment_id: currentTask.value.id,
    date: session.date,
    outcome: trialForm.outcome,
    failure_cause: unsuccessful ? trialForm.failure_cause : '',
    completion_time_s: trialForm.outcome === 'success' ? elapsedSeconds.value.toFixed(2) : '',
    n_attempts: trialForm.n_attempts,
    n_regrasps: trialForm.n_regrasps,
    stage_reached: currentTask.value.stages ? trialForm.stage_reached : '',
    strategy: trialForm.strategy,
    notes: trialForm.notes.trim()
  }
  const issues = recordErrors(record)
  if (issues.length) { error.value = issues.join(' '); return }
  record.n_attempts = Number(record.n_attempts)
  record.n_regrasps = Number(record.n_regrasps)
  if (currentTask.value.stages) record.stage_reached = Number(record.stage_reached)
  trials.value.push(record)
  resetTimer()
  if (currentTrials.value.length >= REQUIRED_TRIALS) step.value = 'review'
}

function removeTrial(trialId) {
  if (!window.confirm('Remove this trial from the session?')) return
  trials.value = trials.value.filter(trial => trial.trial_id !== trialId)
}

function csvEscape(value) {
  const text = String(value ?? '')
  return /[",\r\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text
}

function downloadBlob(filename, blob) {
  const url = URL.createObjectURL(blob)
  const anchor = document.createElement('a')
  anchor.href = url
  anchor.download = filename
  anchor.click()
  window.setTimeout(() => URL.revokeObjectURL(url), 0)
}

function downloadFile(filename, content, type) {
  downloadBlob(filename, new Blob([content], { type }))
}

function buildCsv() {
  const rows = trials.value
    .slice()
    .sort((a, b) => a.trial_id - b.trial_id)
    .map(trial => CSV_COLUMNS.map(column => csvEscape(trial[column])).join(','))
  return [CSV_COLUMNS.join(','), ...rows].join('\n') + '\n'
}

function buildPlatformMarkdown() {
  return `# HiveBoard platform description

- Submission ID: ${ensureSubmissionId()}
- Evaluation mode: ${session.evaluation_mode}
- Laboratory ID: ${session.lab_id}
- Platform ID: ${session.platform_id}
- Robot: ${session.robot_model}
- End-effector: ${session.end_effector}
- Control method or interface: ${session.control_method}
- Board orientation: ${session.board_orientation}
- HiveBoard version or commit: ${session.hiveboard_version}
- Evaluation date: ${session.date}
- Printer: ${session.evaluation_mode === 'physical' ? session.printer : 'Not applicable (simulation)'}
- Material: ${session.evaluation_mode === 'physical' ? session.material : 'Not applicable (simulation)'}
- Print settings: ${session.evaluation_mode === 'physical' ? session.print_settings : 'Not applicable (simulation)'}
- Post-processing: ${session.evaluation_mode === 'physical' ? session.post_processing : 'Not applicable (simulation)'}
- Calibration notes: ${session.calibration_notes || 'Not provided'}
`
}

function sessionPayload() {
  return {
    type: 'hiveboard-evaluation-session',
    schema_version: SCHEMA_VERSION,
    exported_at: new Date().toISOString(),
    session: { ...session, submission_id: ensureSubmissionId() },
    trials: trials.value.slice().sort((a, b) => a.trial_id - b.trial_id),
    selected_task_id: selectedTaskId.value
  }
}

function profilePayload() {
  return {
    type: 'hiveboard-platform-profile',
    schema_version: SCHEMA_VERSION,
    exported_at: new Date().toISOString(),
    profile: Object.fromEntries(PROFILE_FIELDS.map(field => [field, session[field]]))
  }
}

function videoFilename(trial) {
  return `${ensureSubmissionId()}_trial_${String(trial.trial_id).padStart(3, '0')}_${trial.attachment_id}.mp4`
}

function buildRecordingInstructions() {
  const rows = trials.value
    .slice()
    .sort((a, b) => a.trial_id - b.trial_id)
    .map(trial => `| ${trial.trial_id} | \`${trial.attachment_id}\` | \`${videoFilename(trial)}\` |`)
    .join('\n')
  return `# Trial recording instructions

Submission ID: \`${ensureSubmissionId()}\`

1. ${session.evaluation_mode === 'simulation' ? 'Start a screen recording of the simulation before starting the runner countdown.' : 'Start the external camera before starting the runner countdown.'}
2. Record the complete trial without cuts.
3. Keep the HiveBoard, robot and end-effector, and final task state visible.
4. Save one MP4 file for each row in \`trials.csv\`.
5. Rename each file exactly as listed below and place it in the package's \`videos/\` directory before submission.

| Trial | Attachment | Required filename |
| ---: | --- | --- |
${rows}
`
}

function manifestPayload() {
  const sortedTrials = trials.value.slice().sort((a, b) => a.trial_id - b.trial_id)
  return {
    schema_version: SCHEMA_VERSION,
    benchmark: 'HiveBoard',
    submission_id: ensureSubmissionId(),
    created_at: new Date().toISOString(),
    trial_records_complete: submissionReady.value,
    supporting_files: {
      setup_photo: { filename: 'setup.jpg', included: Boolean(setupPhoto.value) },
      recordings_included: false,
      instructions: 'Add the listed recordings and any missing setup.jpg before submission.'
    },
    evaluation: {
      mode: session.evaluation_mode,
      lab_id: session.lab_id,
      platform_id: session.platform_id,
      hiveboard_version: session.hiveboard_version,
      board_orientation: session.board_orientation,
      date: session.date,
      required_trials_per_condition: REQUIRED_TRIALS
    },
    files: {
      trials: 'trials.csv',
      platform_description: 'platform.md',
      session_backup: 'session.json',
      recording_instructions: 'recording-instructions.md',
      submission_instructions: 'submission-instructions.md',
      videos_directory: 'videos/'
    },
    summary: {
      conditions_evaluated: new Set(sortedTrials.map(trial => trial.attachment_id)).size,
      total_trials: sortedTrials.length,
      successful_trials: sortedTrials.filter(trial => trial.outcome === 'success').length
    },
    recordings: sortedTrials.map(trial => ({
      trial_id: trial.trial_id,
      attachment_id: trial.attachment_id,
      expected_filename: videoFilename(trial)
    }))
  }
}

function downloadCsv() {
  downloadFile('trials.csv', buildCsv(), 'text/csv;charset=utf-8')
}

function downloadPlatform() {
  downloadFile('platform.md', buildPlatformMarkdown(), 'text/markdown;charset=utf-8')
}

function downloadProfile() {
  error.value = ''
  transferMessage.value = ''
  transferIsError.value = false
  if (!profileReady.value) {
    error.value = 'Complete the platform fields before exporting a profile.'
    return
  }
  downloadFile(`hiveboard-profile_${session.platform_id}.json`, JSON.stringify(profilePayload(), null, 2) + '\n', 'application/json')
}

function downloadSession() {
  transferIsError.value = false
  if (!sessionReady.value) {
    transferMessage.value = 'Complete the session metadata before exporting a session.'
    transferIsError.value = true
    return
  }
  downloadFile(`${ensureSubmissionId()}_session.json`, JSON.stringify(sessionPayload(), null, 2) + '\n', 'application/json')
}

function chooseProfileFile() {
  profileInput.value?.click()
}

function chooseSessionFile() {
  sessionInput.value?.click()
}

function readSetup(data) {
  if (!data || typeof data !== 'object' || Array.isArray(data)) throw new Error('Invalid setup')
  const result = emptySession()
  for (const field of Object.keys(result)) {
    if (data[field] !== undefined) {
      if (typeof data[field] !== 'string') throw new Error('Invalid setup field')
      result[field] = data[field].trim()
    }
  }
  if (!validSetup(result) || (result.submission_id && !/^[a-z0-9_]+$/.test(result.submission_id))) throw new Error('Invalid setup')
  return result
}

function readSession(data) {
  if (!data || data.type !== 'hiveboard-evaluation-session' || data.schema_version !== SCHEMA_VERSION || !Array.isArray(data.trials)) throw new Error('Unsupported session format')
  const imported = readSetup(data.session)
  if (!validDate(imported.date)) throw new Error('Invalid date')
  const records = data.trials.map(trial => {
    const issues = recordErrors(trial, imported)
    if (issues.length) throw new Error(`Trial ${trial?.trial_id ?? '?'}: ${issues.join(' ')}`)
    return Object.fromEntries(CSV_COLUMNS.map(field => [field, trial[field]]))
  })
  if (new Set(records.map(trial => Number(trial.trial_id))).size !== records.length ||
    tasks.some(task => records.filter(trial => trial.attachment_id === task.id).length > REQUIRED_TRIALS)) throw new Error('Duplicate IDs or too many trials for a condition')
  return { imported, records }
}

async function importProfile(event) {
  error.value = ''
  transferMessage.value = ''
  transferIsError.value = false
  const file = event.target.files?.[0]
  event.target.value = ''
  if (!file) return
  if (trials.value.length) {
    error.value = 'Start a new session before importing a different platform profile.'
    return
  }
  try {
    const data = JSON.parse(await file.text())
    if (data.type !== 'hiveboard-platform-profile' || data.schema_version !== SCHEMA_VERSION) throw new Error('Invalid profile')
    const imported = readSetup(data.profile)
    Object.assign(session, Object.fromEntries(PROFILE_FIELDS.map(field => [field, imported[field]])), { submission_id: '' })
    transferMessage.value = `Profile loaded: ${session.platform_id || file.name}`
  } catch (_) {
    error.value = 'This file is not a valid HiveBoard platform profile.'
  }
}

async function importSession(event) {
  error.value = ''
  transferMessage.value = ''
  transferIsError.value = false
  const file = event.target.files?.[0]
  event.target.value = ''
  if (!file) return
  if (trials.value.length && !window.confirm('Replace the current session and its recorded trials?')) return
  try {
    const data = JSON.parse(await file.text())
    const { imported, records } = readSession(data)
    Object.assign(session, imported)
    resetSubmission()
    trials.value = records
    setupPhoto.value = null
    selectedTaskId.value = tasks.some(task => task.id === data.selected_task_id) ? data.selected_task_id : tasks[0].id
    resetTimer()
    if (sessionReady.value) ensureSubmissionId()
    step.value = trials.value.length ? 'review' : (sessionReady.value ? 'task' : 'setup')
    transferMessage.value = `Session loaded: ${session.submission_id || file.name}`
  } catch (problem) {
    error.value = `Session not imported. ${problem.message}`
  }
}

async function attachSetupPhoto(event) {
  const file = event.target.files?.[0]
  event.target.value = ''
  if (!file) return
  transferMessage.value = ''
  transferIsError.value = false
  try {
    const signature = new Uint8Array(await file.slice(0, 3).arrayBuffer())
    if (signature[0] !== 255 || signature[1] !== 216 || signature[2] !== 255) throw new Error('Select a JPEG image for setup.jpg.')
    setupPhoto.value = file
    transferMessage.value = 'Setup photograph attached. It will be included as setup.jpg in the ZIP.'
  } catch (problem) {
    transferMessage.value = problem.message
    transferIsError.value = true
  }
}

async function downloadPackage() {
  transferMessage.value = ''
  transferIsError.value = false
  if (!submissionReady.value) {
    transferMessage.value = 'Complete the experimental setup and record five valid trials for all 13 conditions.'
    transferIsError.value = true
    return
  }
  packageState.value = 'building'
  try {
    const submissionId = ensureSubmissionId()
    const zip = new JSZip()
    const root = zip.folder(submissionId)
    root.file('trials.csv', buildCsv())
    root.file('platform.md', buildPlatformMarkdown())
    root.file('manifest.json', JSON.stringify(manifestPayload(), null, 2) + '\n')
    root.file('session.json', JSON.stringify(sessionPayload(), null, 2) + '\n')
    root.file('recording-instructions.md', buildRecordingInstructions())
    root.file('submission-instructions.md', '# Submit for organizer review\n\nAdd setup.jpg and all 65 recordings listed in recording-instructions.md. Recompress the completed folder and upload it to storage you control. Give ricardo.godoy@usp.br download access; do not make the package publicly accessible before review.\n\nReturn to the runner, import session.json if needed, and select Submit for review. Enter your contact details and package link, confirm the supporting files and publication permission, then prepare and send the email. Opening a draft does not send it.\n\nThe organizer checks the records and recordings, requests corrections by email, and approves results before publication. Contact details, private download links, and correspondence must not be published.\n')
    if (setupPhoto.value) root.file('setup.jpg', await setupPhoto.value.arrayBuffer())
    root.file('README.md', `# HiveBoard results\n\nSubmission ID: ${submissionId}\n\nTrial records: 65/65 complete.\n\n${setupPhoto.value ? 'setup.jpg is included.' : 'Add a photograph of the complete setup as setup.jpg.'}\nAdd the 65 recordings listed in recording-instructions.md to videos/.\nThe runner has not uploaded these files or reviewed the recordings.\n`)
    root.folder('videos').file('README.md', 'Place the trial MP4 files listed in ../recording-instructions.md in this directory before submission. Use external-camera recordings for physical trials and screen recordings for simulated trials.\n')
    const blob = await zip.generateAsync({ type: 'blob', compression: 'DEFLATE', compressionOptions: { level: 6 } })
    downloadBlob(`${submissionId}.zip`, blob)
    transferMessage.value = `Results downloaded. Add the 65 recordings to videos/${setupPhoto.value ? '.' : ' and a setup photograph as setup.jpg.'}`
  } catch (_) {
    transferMessage.value = 'The submission package could not be created. Export the session and try again.'
    transferIsError.value = true
  } finally {
    packageState.value = 'idle'
  }
}

function newSession() {
  if (trials.value.length && !window.confirm('Clear the saved session and all recorded trials?')) return
  clearTicker()
  Object.assign(session, emptySession(), { date: new Date().toISOString().slice(0, 10) })
  trials.value = []
  resetSubmission()
  setupPhoto.value = null
  selectedTaskId.value = tasks[0].id
  step.value = 'setup'
  transferMessage.value = ''
  transferIsError.value = false
  packageState.value = 'idle'
  resetTimer()
  window.localStorage.removeItem(STORAGE_KEY)
}

function openExampleVideo(task) {
  exampleVideoTask.value = task
}

function closeExampleVideo() {
  exampleVideoTask.value = null
}

function handleKey(event) {
  if (event.key === 'Escape' && exampleVideoTask.value) {
    event.preventDefault()
    closeExampleVideo()
    return
  }
  if (exampleVideoTask.value) return
  const target = event.target
  if (target && ['INPUT', 'SELECT', 'TEXTAREA', 'BUTTON'].includes(target.tagName)) return
  if (event.code !== 'Space' || step.value !== 'trial') return
  event.preventDefault()
  if (timerState.value === 'idle') startCountdown()
  else if (timerState.value === 'running') finishTiming(false)
}

onMounted(() => {
  session.date = new Date().toISOString().slice(0, 10)
  try {
    const saved = JSON.parse(window.localStorage.getItem(STORAGE_KEY) || 'null')
    if (saved?.session) {
      for (const field of Object.keys(emptySession())) {
        if (typeof saved.session[field] === 'string') session[field] = saved.session[field]
      }
    }
    if (Array.isArray(saved?.trials)) trials.value = saved.trials.filter(trial => trial && typeof trial === 'object')
    if (tasks.some(task => task.id === saved?.selectedTaskId)) selectedTaskId.value = saved.selectedTaskId
    if (['setup', 'task', 'trial', 'review'].includes(saved?.step)) step.value = saved.step
    if (trials.value.length && sessionReady.value) ensureSubmissionId()
  } catch (_) {}
  mounted.value = true
  window.addEventListener('keydown', handleKey)
  document.addEventListener('visibilitychange', updateTimerOnce)
})

watch(
  () => ({ session: { ...session }, trials: trials.value }),
  () => { evidenceConfirmed.value = false; publicationConsent.value = false; submissionDraft.value = '' },
  { deep: true }
)

watch(
  () => ({ ...contact, consent: publicationConsent.value, evidence: evidenceConfirmed.value }),
  () => { submissionDraft.value = ''; submissionMessage.value = '' },
  { deep: true }
)

watch(
  () => ({ session: { ...session }, trials: trials.value, selectedTaskId: selectedTaskId.value, step: step.value }),
  value => {
    if (mounted.value) {
      try { window.localStorage.setItem(STORAGE_KEY, JSON.stringify(value)); storageError.value = '' }
      catch (_) { storageError.value = 'This browser could not save the session. Export a session backup before closing the page.' }
    }
  },
  { deep: true }
)

onUnmounted(() => {
  clearTicker()
  window.removeEventListener('keydown', handleKey)
  document.removeEventListener('visibilitychange', updateTimerOnce)
})
</script>

<template>
  <div class="runner">
    <p v-if="storageError" class="form-error" role="alert">{{ storageError }}</p>
    <nav class="runner-steps" aria-label="Evaluation progress">
      <button v-for="(label, key) in { setup: '1. Setup', task: '2. Task', trial: '3. Trial', review: '4. Review' }"
        :key="key" :class="{ active: step === key }" type="button"
        :disabled="trialPending || (key !== 'setup' && !sessionReady)" @click="step = key">
        {{ label }}
      </button>
    </nav>

    <section v-if="step === 'setup'" class="runner-section">
      <div class="section-heading">
        <div>
          <h2>Experimental setup</h2>
        </div>
        <p>Required fields are marked with an asterisk.</p>
      </div>

      <div class="transfer-tools">
        <div>
          <strong>Platform profile</strong>
          <span>Reuse the same robot and interface configuration.</span>
          <div>
            <button class="secondary compact" type="button" @click="chooseProfileFile">Import profile</button>
            <button class="secondary compact" type="button" :disabled="!profileReady" @click="downloadProfile">Export profile</button>
          </div>
        </div>
        <div>
          <strong>Evaluation session</strong>
          <span>Continue a session exported from another browser.</span>
          <div><button class="secondary compact" type="button" @click="chooseSessionFile">Import session</button></div>
        </div>
      </div>
      <input ref="profileInput" class="visually-hidden" type="file" accept="application/json,.json" @change="importProfile">
      <input ref="sessionInput" class="visually-hidden" type="file" accept="application/json,.json" @change="importSession">

      <p v-if="trials.length">The platform and date are fixed for these trials. Start a new session to change them.</p>
      <fieldset class="form-grid setup-fields" :disabled="trials.length > 0">
        <label>Evaluation mode *
          <select v-model="session.evaluation_mode">
            <option value="physical">Physical board</option>
            <option value="simulation">Simulation</option>
          </select>
        </label>
        <label>Evaluation date *
          <input v-model="session.date" type="date">
        </label>
        <label>Laboratory ID *
          <input v-model="session.lab_id" placeholder="usp_crob" pattern="[a-z0-9_]+">
          <span>Lowercase letters, numbers and underscores.</span>
        </label>
        <label>Platform ID *
          <input v-model="session.platform_id" placeholder="franka_2f85" pattern="[a-z0-9_]+">
          <span>Identifier for the end-effector and control-interface combination.</span>
        </label>
        <label>Robot model *
          <input v-model="session.robot_model" placeholder="Franka Research 3">
        </label>
        <label>End-effector *
          <input v-model="session.end_effector" placeholder="Robotiq 2F-85">
        </label>
        <label>Control method or interface *
          <input v-model="session.control_method" placeholder="Cartesian teleoperation">
        </label>
        <label>Board orientation *
          <select v-model="session.board_orientation">
            <option value="horizontal">Horizontal</option>
            <option value="vertical">Vertical</option>
          </select>
        </label>
        <label class="wide">HiveBoard release or commit *
          <input v-model="session.hiveboard_version" placeholder="Release tag or full commit hash">
        </label>
      </fieldset>
      <div class="form-grid setup-details">
        <template v-if="session.evaluation_mode === 'physical'">
          <label>Printer *<input v-model="session.printer" placeholder="Manufacturer and model"></label>
          <label>Material *<input v-model="session.material" placeholder="Filament type and manufacturer"></label>
          <label class="wide">Print settings *<textarea v-model="session.print_settings" rows="2" placeholder="Nozzle, layer height, walls, infill, and part orientation"></textarea></label>
          <label class="wide">Post-processing *<textarea v-model="session.post_processing" rows="2" placeholder="Sanding, lubrication, dimensional adjustments, or None"></textarea></label>
        </template>
        <label class="wide">Calibration notes<input v-model="session.calibration_notes" placeholder="Relevant calibration or setup changes"></label>
      </div>
      <p v-if="error" class="form-error" role="alert">{{ error }}</p>
      <p v-if="transferMessage" class="form-success" role="status">{{ transferMessage }}</p>
      <div class="actions"><button class="primary" type="button" @click="startSession">Select task</button></div>
    </section>

    <section v-else-if="step === 'task'" class="runner-section">
      <div class="section-heading">
        <div><p class="eyebrow">Task selection</p><h2>Select an attachment</h2></div>
        <p>Each condition requires five recorded trials.</p>
      </div>
      <div class="task-grid">
        <button v-for="task in tasks" :key="task.id" class="task-card" type="button" @click="openTask(task.id)">
          <img :src="withBase(task.image)" :alt="`${task.name} render`">
          <span class="task-card-body">
            <strong>{{ task.name }}</strong>
            <span>{{ task.family }} · {{ task.timeout }} s</span>
            <span class="task-count">{{ trials.filter(t => t.attachment_id === task.id).length }}/5 trials</span>
          </span>
        </button>
      </div>
      <div class="actions split">
        <button class="secondary" type="button" @click="step = 'setup'">Edit setup</button>
        <button v-if="trials.length" class="secondary" type="button" @click="step = 'review'">Review {{ trials.length }} recorded trials</button>
      </div>
    </section>

    <section v-else-if="step === 'trial'" class="runner-section">
      <div class="trial-heading">
        <div>
          <p class="eyebrow">{{ currentTask.family }} · {{ currentTask.timeout }} second timeout</p>
          <h2>{{ currentTask.name }}</h2>
          <p>Trial {{ currentTrialNumber }} of {{ REQUIRED_TRIALS }}</p>
        </div>
        <button class="secondary compact" type="button" :disabled="trialPending" @click="step = 'task'">Change task</button>
      </div>

      <aside class="recording-reminder" role="note">
        <strong>External recording required.</strong>
        Record the complete trial with an external camera. Keep the board, robot and end-effector, and final task state visible.
      </aside>

      <div class="trial-layout">
        <div class="task-reference">
          <img :src="withBase(currentTask.image)" :alt="`${currentTask.name} reference render`">
          <dl>
            <div><dt>Success</dt><dd>{{ currentTask.success }}</dd></div>
            <div v-if="currentTask.stages"><dt>Stages</dt><dd>{{ currentTask.stages.join(' → ') }}</dd></div>
            <div><dt>Reset</dt><dd>{{ currentTask.reset }}</dd></div>
          </dl>
          <div class="reference-links">
            <button class="video-link" type="button" @click="openExampleVideo(currentTask)">
              Watch example trial <span>{{ currentTask.videoPlatform }}</span>
            </button>
            <a href="https://hiveboard-bench.github.io/#Simulation-Compatibility" target="_blank" rel="noreferrer noopener">Open the interactive simulation ↗</a>
          </div>
        </div>

        <div class="timer-panel" :class="timerState">
          <p v-if="timerState === 'countdown'" class="countdown-label">Starting in</p>
          <div v-if="timerState === 'countdown'" class="countdown" aria-live="assertive">{{ countdown }}</div>
          <div v-else class="stopwatch" aria-live="polite">{{ displayTime }}</div>
          <p v-if="timerState === 'idle'">Begin the task when the countdown reaches zero.</p>
          <p v-else-if="timerState === 'running'">Timer running · automatic timeout at {{ currentTask.timeout }} seconds</p>
          <p v-else-if="timerState === 'form'">Enter the trial outcome.</p>

          <div v-if="timerState === 'idle'" class="timer-actions">
            <button class="primary large" type="button" @click="startCountdown">Start countdown</button>
            <span>Keyboard: Space</span>
          </div>
          <div v-else-if="timerState === 'countdown'" class="timer-actions">
            <button class="secondary" type="button" @click="cancelCountdown">Cancel</button>
          </div>
          <div v-else-if="timerState === 'running'" class="timer-actions">
            <button class="stop large" type="button" @click="finishTiming(false)">Stop timer</button>
            <span>Keyboard: Space</span>
          </div>
        </div>
      </div>

      <form v-if="timerState === 'form'" class="trial-form" @submit.prevent="saveTrial">
        <h3>Trial record</h3>
        <div class="form-grid">
          <label>Outcome *
            <select v-model="trialForm.outcome">
              <option value="success">Success</option>
              <option value="fail">Fail</option>
              <option value="timeout">Timeout</option>
              <option value="safety_stop">Safety stop</option>
            </select>
          </label>
          <label v-if="trialForm.outcome === 'success'">Completion time
            <input :value="elapsedSeconds.toFixed(2)" type="text" readonly>
            <span>Seconds, recorded by the timer.</span>
          </label>
          <label v-else>Primary failure cause *
            <select v-model="trialForm.failure_cause">
              <option value="" disabled>Select one</option>
              <option value="grasp_geometry">Grasp geometry</option>
              <option value="kinematic_limit">Kinematic limit</option>
              <option value="perception">Perception</option>
              <option value="slip">Slip</option>
              <option value="force_limit">Force limit</option>
              <option value="control_precision">Control precision</option>
              <option value="other">Other</option>
            </select>
          </label>
          <label>Attempts *
            <input v-model.number="trialForm.n_attempts" type="number" min="1" step="1">
          </label>
          <label>Regrasps *
            <input v-model.number="trialForm.n_regrasps" type="number" min="0" step="1">
          </label>
          <label>Strategy *
            <select v-model="trialForm.strategy">
              <option value="prehensile">Prehensile</option>
              <option value="non_prehensile">Non-prehensile</option>
            </select>
          </label>
          <label v-if="currentTask.stages">Last completed stage *
            <select v-model="trialForm.stage_reached">
              <option value="" disabled>Select stage</option>
              <option :value="0">0 — No stage completed</option>
              <option v-for="(stageName, index) in currentTask.stages" :key="stageName" :value="index + 1">{{ index + 1 }} — {{ stageName }}</option>
            </select>
          </label>
          <label class="wide">Notes
            <textarea v-model="trialForm.notes" rows="3" placeholder="Deviation, adjustment, damage, or other observation"></textarea>
          </label>
        </div>
        <p v-if="error" class="form-error" role="alert">{{ error }}</p>
        <div class="actions split">
          <button class="secondary" type="button" @click="resetTimer">Discard timing</button>
          <button class="primary" type="submit">Save trial</button>
        </div>
      </form>

      <div v-if="currentTrials.length" class="recorded-summary">
        <strong>{{ currentTrials.length }}/5 recorded</strong>
        <span v-for="n in REQUIRED_TRIALS" :key="n" :class="{ filled: n <= currentTrials.length }"></span>
      </div>
    </section>

    <section v-else class="runner-section">
      <div class="section-heading">
        <div><h2>Recorded trials</h2></div>
        <p>Export the session to keep a backup or continue on another computer.</p>
      </div>

      <div v-if="!trials.length" class="empty-state">
        <p>No trials have been recorded.</p>
        <button class="primary" type="button" @click="step = 'task'">Select a task</button>
      </div>
      <template v-else>
        <div class="submission-identity">
          <span>Submission ID</span>
          <code>{{ session.submission_id }}</code>
        </div>
        <div class="review-stats">
          <div><strong>{{ trials.length }}</strong><span>Total trials</span></div>
          <div><strong>{{ trials.filter(t => t.outcome === 'success').length }}</strong><span>Successful</span></div>
          <div><strong>{{ new Set(trials.map(t => t.attachment_id)).size }}</strong><span>Conditions</span></div>
        </div>
        <div class="table-wrap">
          <table>
            <thead><tr><th>Trial</th><th>Attachment</th><th>Outcome</th><th>Time (s)</th><th>Attempts</th><th>Regrasps</th><th></th></tr></thead>
            <tbody>
              <tr v-for="trial in trials" :key="trial.trial_id">
                <td>{{ trial.trial_id }}</td><td><code>{{ trial.attachment_id }}</code></td><td>{{ trial.outcome }}</td>
                <td>{{ trial.completion_time_s || '—' }}</td><td>{{ trial.n_attempts }}</td><td>{{ trial.n_regrasps }}</td>
                <td><button class="text-button" type="button" @click="removeTrial(trial.trial_id)">Remove</button></td>
              </tr>
            </tbody>
          </table>
        </div>
        <div class="actions split wrap">
          <div><button class="secondary" type="button" @click="step = 'task'">Record another task</button></div>
          <div class="download-actions">
            <button class="secondary" type="button" @click="downloadSession">Export session</button>
            <button class="secondary" type="button" @click="downloadPlatform">Download platform.md</button>
            <button class="secondary" type="button" @click="downloadCsv">Download trials.csv</button>
          </div>
        </div>

        <section class="readiness-panel" aria-labelledby="readiness-title">
          <div class="readiness-heading">
            <div>
              <p class="eyebrow">Submission package</p>
              <h3 id="readiness-title">Check trial records</h3>
            </div>
            <strong :class="submissionReady ? 'ready' : 'incomplete'">{{ submissionReady ? 'Trial records complete' : 'Trial records incomplete' }}</strong>
          </div>
          <ul class="validation-list">
            <li v-for="check in validationChecks" :key="check.label" :class="{ passed: check.passed }">
              <span aria-hidden="true">{{ check.passed ? '✓' : '—' }}</span>{{ check.label }}
            </li>
          </ul>
          <ul v-if="invalidRecords.length" class="form-error"><li v-for="issue in invalidRecords" :key="issue">{{ issue }}</li></ul>
          <div class="condition-progress">
            <div v-for="task in tasks" :key="task.id" :class="{ complete: taskTrialCounts[task.id] === REQUIRED_TRIALS }">
              <span>{{ task.name }}</span>
              <strong>{{ taskTrialCounts[task.id] }}/{{ REQUIRED_TRIALS }}</strong>
            </div>
          </div>
          <p>The ZIP contains the trial log, platform description, manifest, session backup, and recording filenames.</p>
          <h3>Supporting files</h3>
          <label>Setup photograph (JPEG)<input type="file" accept="image/jpeg,.jpg,.jpeg" @change="attachSetupPhoto"></label>
          <p>{{ setupPhoto ? 'setup.jpg attached; it will be included in the ZIP.' : 'setup.jpg missing; attach it here or add it to the downloaded folder.' }} The photograph is not stored in session backups; attach it again after reloading or importing a session.</p>
          <p>Add all 65 {{ session.evaluation_mode === 'simulation' ? 'screen recordings' : 'external-camera recordings' }} to <code>videos/</code> after extracting the ZIP. The runner checks trial entries; it does not assess task success or inspect recordings.</p>
          <p v-if="transferMessage" :class="transferIsError ? 'form-error' : 'form-success'" role="status">{{ transferMessage }}</p>
          <div class="package-action">
            <span v-if="!submissionReady">Record five valid trials per condition and complete the experimental setup before downloading the complete results.</span>
            <button class="primary" type="button" :disabled="!submissionReady || packageState === 'building'" @click="downloadPackage">
              {{ packageState === 'building' ? 'Creating ZIP…' : 'Download results (.zip)' }}
            </button>
          </div>
        </section>
        <section class="readiness-panel" aria-labelledby="submission-title">
          <h3 id="submission-title">Submit for organizer review</h3>
          <p>Submissions are reviewed by email. Download your results, add the photograph and recordings, and share the completed package with the organizer.</p>
          <button class="secondary" type="button" :disabled="!submissionReady" @click="submissionOpen = true">Submit for review</button>
          <p v-if="!submissionReady">Complete the required setup fields and all 65 valid trial records to continue.</p>
          <div v-if="submissionOpen">
            <ol>
              <li>Download and extract the results ZIP above.</li>
              <li>Add <code>setup.jpg</code> and the 65 external-camera recordings to <code>videos/</code>, using the filenames in <code>recording-instructions.md</code>. For simulated evaluations, provide screen recordings showing each complete trial.</li>
              <li>Recompress the completed folder and upload it to storage you control. Give <strong>ricardo.godoy@usp.br</strong> download access. Keep the package private during review and keep the link available until the review is complete.</li>
            </ol>
            <p>Contact details and the private download link appear only in the email draft. They are not saved in this browser's session, exported profiles, or results ZIP. Re-enter them if you reload the page.</p>
            <form @submit.prevent="prepareSubmissionEmail">
              <div class="form-grid">
                <label>Contact name *<input v-model="contact.name" autocomplete="name" required maxlength="150"></label>
                <label>Contact email *<input v-model="contact.email" type="email" autocomplete="email" required maxlength="254"></label>
                <label>Institution *<input v-model="contact.institution" autocomplete="organization" required maxlength="200" placeholder="Institution, or Independent"></label>
                <label>Completed package link *<input v-model="contact.package_url" type="url" required maxlength="2000" placeholder="https://…"></label>
              </div>
              <p>Robot: <strong>{{ session.robot_model }}</strong> · End-effector: <strong>{{ session.end_effector }}</strong> · Control interface: <strong>{{ session.control_method }}</strong></p>
              <label class="submission-check"><input v-model="evidenceConfirmed" type="checkbox">I checked that the shared package contains the setup photograph and all 65 recordings, and that the organizer can download it.</label>
              <label class="submission-check"><input v-model="publicationConsent" type="checkbox">{{ PUBLICATION_PERMISSION }}</label>
              <p>You may download and keep your results without submitting or granting publication permission.</p>
              <p v-if="submissionMessage" class="form-error" role="alert">{{ submissionMessage }}</p>
              <button class="primary" type="submit" :disabled="!submissionReady">Prepare submission email</button>
            </form>
            <div v-if="submissionDraft && submissionReady" class="submission-draft">
              <h4>Email draft — not sent</h4>
              <p>Review the text and send it using your email application. If the email link does not work, copy the draft below and send it to <strong>ricardo.godoy@usp.br</strong>.</p>
              <label>Draft email<textarea :value="submissionDraft" readonly rows="15"></textarea></label>
              <a :href="submissionMailto">Open draft in email application</a>
              <p>The organizer will check the records and recordings and contact you if corrections are needed. Only approved results will be published. This page does not send email, upload files, or confirm receipt.</p>
            </div>
          </div>
        </section>
      </template>
      <div class="new-session"><button class="text-button" type="button" @click="newSession">Start a new session</button></div>
    </section>
  </div>

  <Teleport to="body">
    <div v-if="exampleVideoTask" class="video-modal-backdrop" @click.self="closeExampleVideo">
      <section class="video-modal" role="dialog" aria-modal="true" aria-labelledby="example-video-title">
        <header>
          <div>
            <p>Example trial · {{ exampleVideoTask.videoPlatform }}</p>
            <h2 id="example-video-title">{{ exampleVideoTask.name }}</h2>
          </div>
          <button class="video-close" type="button" aria-label="Close example video" @click="closeExampleVideo">Close</button>
        </header>
        <video :key="exampleVideoTask.video" controls autoplay muted playsinline preload="metadata" controlslist="nodownload">
          <source :src="exampleVideoTask.video" type="video/mp4">
          Your browser does not support HTML video.
        </video>
      </section>
    </div>
  </Teleport>
</template>

<style scoped>
.submission-check { display: flex; align-items: flex-start; gap: .65rem; margin: 1rem 0; }
.runner .submission-check input { width: 1.1rem; height: 1.1rem; min-height: 0; padding: 0; flex: 0 0 auto; margin-top: .3rem; }
.submission-draft { margin-top: 1.5rem; }
.submission-draft textarea { width: 100%; }
.runner { margin: 1.5rem 0 3rem; border: 1px solid #dfe2e5; background: #fff; color: #24292f; }
.runner-steps { display: grid; grid-template-columns: repeat(4, 1fr); border-bottom: 1px solid #dfe2e5; background: #f7f8fa; }
.runner-steps button { padding: .8rem .6rem; border: 0; border-right: 1px solid #dfe2e5; background: transparent; color: #57606a; font: inherit; font-size: .86rem; text-align: center; }
.runner-steps button:last-child { border-right: 0; }
.runner-steps button.active { box-shadow: inset 0 -3px #23527c; background: #fff; color: #183b5b; font-weight: 600; }
.runner-steps button:disabled { color: #9ca3af; cursor: not-allowed; }
.runner-section { padding: 1.5rem; }
.section-heading, .trial-heading { display: flex; align-items: flex-start; justify-content: space-between; gap: 2rem; margin-bottom: 1.5rem; }
.section-heading > p { max-width: 420px; margin: .25rem 0 0; color: #57606a; font-size: .9rem; }
.eyebrow { margin: 0 0 .25rem; color: #57606a; font-size: .76rem; font-weight: 700; letter-spacing: .06em; text-transform: uppercase; }
.runner h2 { margin: 0; padding: 0; border: 0; font-size: 1.35rem; }
.runner h3 { margin-top: 0; font-size: 1.08rem; }
.trial-heading p:last-child { margin: .35rem 0 0; color: #57606a; }
.transfer-tools { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: .8rem; margin-bottom: 1.25rem; }
.transfer-tools > div { display: grid; gap: .35rem; padding: .85rem 1rem; border: 1px solid #dfe2e5; background: #fafbfc; }
.transfer-tools strong { font-size: .88rem; }
.transfer-tools span { color: #57606a; font-size: .78rem; }
.transfer-tools > div > div { display: flex; flex-wrap: wrap; gap: .5rem; margin-top: .3rem; }
.visually-hidden { position: absolute; overflow: hidden; width: 1px; height: 1px; padding: 0; border: 0; clip: rect(0 0 0 0); white-space: nowrap; }
.form-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 1rem 1.25rem; }
.setup-fields { min-width: 0; margin: 0; padding: 0; border: 0; }
.setup-fields:disabled input, .setup-fields:disabled select { background: #f1f3f5; }
.setup-details { margin-top: 1rem; }
label { display: flex; flex-direction: column; gap: .38rem; color: #34414d; font-size: .88rem; font-weight: 600; }
label span { color: #6e7781; font-size: .76rem; font-weight: 400; }
input, select, textarea { box-sizing: border-box; width: 100%; min-height: 42px; padding: .58rem .68rem; border: 1px solid #c7cdd3; border-radius: 2px; background: #fff; color: #24292f; font: inherit; font-size: 1rem; }
textarea { resize: vertical; }
input:focus, select:focus, textarea:focus, button:focus-visible { outline: 3px solid rgba(35, 82, 124, .18); outline-offset: 1px; border-color: #23527c; }
.wide { grid-column: 1 / -1; }
.actions { display: flex; justify-content: flex-end; gap: .7rem; margin-top: 1.25rem; }
.actions.split { justify-content: space-between; }
.actions.wrap { flex-wrap: wrap; }
.download-actions { display: flex; flex-wrap: wrap; gap: .7rem; }
button { cursor: pointer; }
button.primary, button.secondary, button.stop { min-height: 42px; padding: .55rem 1rem; border-radius: 2px; font: inherit; font-size: .9rem; font-weight: 600; }
button.primary { border: 1px solid #23527c; background: #23527c; color: #fff; }
button.primary:hover { background: #183b5b; }
button.primary:disabled, button.secondary:disabled { border-color: #c7cdd3; background: #e9ecef; color: #7a838c; cursor: not-allowed; }
button.secondary { border: 1px solid #aeb6bf; background: #fff; color: #263746; }
button.secondary:hover { background: #f3f4f6; }
button.stop { border: 1px solid #9b2c2c; background: #9b2c2c; color: #fff; }
button.large { min-width: 190px; min-height: 50px; font-size: 1rem; }
button.compact { min-height: 36px; padding: .35rem .75rem; }
.form-error { margin: 1rem 0 0; padding: .65rem .8rem; border-left: 3px solid #b42318; background: #fff1f0; color: #7a271a; font-size: .88rem; }
.form-success { margin: 1rem 0 0; padding: .65rem .8rem; border-left: 3px solid #1a7f37; background: #f0fff4; color: #14532d; font-size: .88rem; }
.recording-reminder { margin: -0.25rem 0 1.25rem; padding: .8rem 1rem; border: 1px solid #d4b106; border-left: 4px solid #d4b106; background: #fffbe6; color: #4d3f00; font-size: .88rem; line-height: 1.5; }
.recording-reminder strong { margin-right: .25rem; }
.task-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: .85rem; }
.task-card { overflow: hidden; padding: 0; border: 1px solid #d6d9dd; border-radius: 2px; background: #fff; color: inherit; text-align: left; }
.task-card:hover { border-color: #23527c; box-shadow: 0 2px 8px rgba(31, 41, 51, .08); }
.task-card img { display: block; width: 100%; aspect-ratio: 16 / 8; object-fit: cover; border-bottom: 1px solid #e5e7eb; background: #f7f8fa; }
.task-card-body { display: grid; gap: .2rem; padding: .72rem .78rem .8rem; }
.task-card-body strong { font-size: .92rem; }
.task-card-body > span { color: #6e7781; font-size: .78rem; }
.task-card-body .task-count { margin-top: .25rem; color: #23527c; font-weight: 600; }
.trial-layout { display: grid; grid-template-columns: minmax(0, 1.05fr) minmax(320px, .95fr); gap: 1.25rem; }
.task-reference { padding: 1rem; border: 1px solid #dfe2e5; background: #fafbfc; }
.task-reference img { display: block; width: 100%; aspect-ratio: 16 / 8.5; object-fit: cover; border: 1px solid #e1e4e8; background: #fff; }
.task-reference dl { margin: 1rem 0; }
.task-reference dl div { display: grid; grid-template-columns: 76px 1fr; gap: .75rem; padding: .58rem 0; border-bottom: 1px solid #e1e4e8; }
.task-reference dt { font-size: .78rem; font-weight: 700; text-transform: uppercase; }
.task-reference dd { margin: 0; font-size: .88rem; line-height: 1.55; }
.reference-links { display: grid; gap: .45rem; }
.task-reference a { font-size: .84rem; font-weight: 600; }
.task-reference a span { color: #57606a; font-size: .78rem; font-weight: 400; }
.video-link { width: fit-content; padding: 0; border: 0; background: transparent; color: #23527c; font: inherit; font-size: .84rem; font-weight: 600; text-align: left; text-decoration: underline; }
.video-link span { color: #57606a; font-size: .78rem; font-weight: 400; }
.video-modal-backdrop { position: fixed; z-index: 1000; inset: 0; display: flex; padding: 1.5rem; align-items: center; justify-content: center; background: rgba(13, 23, 33, .82); }
.video-modal { overflow: hidden; width: min(960px, 100%); max-height: calc(100vh - 3rem); border: 1px solid #cfd5db; border-radius: 2px; background: #fff; box-shadow: 0 20px 60px rgba(0, 0, 0, .35); color: #24292f; }
.video-modal header { display: flex; padding: .9rem 1rem; align-items: center; justify-content: space-between; gap: 1rem; }
.video-modal header p { margin: 0 0 .15rem; color: #57606a; font-size: .76rem; font-weight: 600; letter-spacing: .04em; text-transform: uppercase; }
.video-modal header h2 { margin: 0; border: 0; font-size: 1.1rem; }
.video-modal video { display: block; width: 100%; max-height: calc(100vh - 12rem); aspect-ratio: 16 / 9; background: #000; }
.video-close { min-height: 38px; padding: .4rem .75rem; border: 1px solid #aeb6bf; border-radius: 2px; background: #fff; color: #263746; font: inherit; font-size: .84rem; font-weight: 600; }
.video-close:hover { background: #f3f4f6; }
.video-caption { margin: 0; padding: .7rem 1rem .8rem; color: #57606a; font-size: .8rem; }
.timer-panel { display: flex; min-height: 330px; padding: 1.25rem; border: 1px solid #cfd5db; align-items: center; justify-content: center; flex-direction: column; text-align: center; }
.timer-panel.running { border-color: #23527c; background: #f7fbff; }
.stopwatch, .countdown { color: #172b3a; font-variant-numeric: tabular-nums; line-height: 1; }
.stopwatch { font-family: var(--vp-font-family-mono); font-size: clamp(2.7rem, 7vw, 4.4rem); letter-spacing: -.06em; }
.countdown { font-size: 6rem; font-weight: 600; }
.countdown-label { margin-bottom: .6rem; text-transform: uppercase; letter-spacing: .08em; }
.timer-panel > p { max-width: 360px; color: #57606a; font-size: .86rem; }
.timer-actions { display: flex; margin-top: 1.2rem; align-items: center; gap: .8rem; flex-direction: column; }
.timer-actions span { color: #6e7781; font-size: .76rem; }
.trial-form { margin-top: 1.25rem; padding: 1.25rem; border: 1px solid #dfe2e5; background: #fafbfc; }
.recorded-summary { display: flex; margin-top: 1rem; align-items: center; gap: .45rem; color: #57606a; font-size: .82rem; }
.recorded-summary strong { margin-right: .3rem; }
.recorded-summary span { width: 24px; height: 5px; background: #dfe2e5; }
.recorded-summary span.filled { background: #23527c; }
.review-stats { display: grid; grid-template-columns: repeat(3, 1fr); margin-bottom: 1rem; border: 1px solid #dfe2e5; }
.review-stats div { display: flex; padding: 1rem; border-right: 1px solid #dfe2e5; flex-direction: column; }
.review-stats div:last-child { border-right: 0; }
.review-stats strong { font-size: 1.5rem; }
.review-stats span { color: #6e7781; font-size: .78rem; }
.submission-identity { display: flex; margin-bottom: 1rem; padding: .75rem 1rem; border: 1px solid #dfe2e5; align-items: center; justify-content: space-between; gap: 1rem; background: #fafbfc; }
.submission-identity span { color: #57606a; font-size: .78rem; font-weight: 700; letter-spacing: .04em; text-transform: uppercase; }
.submission-identity code { overflow-wrap: anywhere; font-size: .82rem; }
.table-wrap { overflow-x: auto; border: 1px solid #dfe2e5; }
.table-wrap table { display: table; width: 100%; margin: 0; border: 0; font-size: .84rem; }
.table-wrap th, .table-wrap td { white-space: nowrap; }
.text-button { padding: .2rem; border: 0; background: transparent; color: #23527c; font: inherit; font-size: .82rem; text-decoration: underline; }
.empty-state { padding: 2rem; border: 1px dashed #b8c0c8; text-align: center; }
.readiness-panel { margin-top: 1.5rem; padding: 1.25rem; border: 1px solid #cfd5db; background: #fafbfc; }
.readiness-heading { display: flex; margin-bottom: 1rem; align-items: flex-start; justify-content: space-between; gap: 1rem; }
.readiness-heading h3 { margin: 0; }
.readiness-heading > strong { padding: .25rem .55rem; border: 1px solid; border-radius: 999px; font-size: .75rem; }
.readiness-heading > strong.ready { border-color: #75b798; background: #ecfdf3; color: #146c43; }
.readiness-heading > strong.incomplete { border-color: #c7cdd3; background: #f1f3f5; color: #57606a; }
.validation-list { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: .45rem 1rem; margin: 0 0 1rem; padding: 0; list-style: none; }
.validation-list li { display: flex; align-items: center; gap: .45rem; color: #6e7781; font-size: .84rem; }
.validation-list li > span { display: inline-grid; width: 20px; height: 20px; border: 1px solid #c7cdd3; border-radius: 50%; place-items: center; font-size: .72rem; }
.validation-list li.passed { color: #146c43; }
.validation-list li.passed > span { border-color: #75b798; background: #ecfdf3; }
.condition-progress { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); margin: 1rem 0; border-top: 1px solid #dfe2e5; border-left: 1px solid #dfe2e5; }
.condition-progress > div { display: flex; min-width: 0; padding: .55rem .65rem; border-right: 1px solid #dfe2e5; border-bottom: 1px solid #dfe2e5; align-items: center; justify-content: space-between; gap: .5rem; background: #fff; }
.condition-progress span { overflow: hidden; color: #57606a; font-size: .76rem; text-overflow: ellipsis; white-space: nowrap; }
.condition-progress strong { color: #7a271a; font-size: .76rem; font-variant-numeric: tabular-nums; }
.condition-progress .complete strong { color: #146c43; }
.readiness-panel > p { color: #57606a; font-size: .82rem; line-height: 1.55; }
.package-action { display: flex; margin-top: 1rem; padding-top: 1rem; border-top: 1px solid #dfe2e5; align-items: center; justify-content: space-between; gap: 1rem; }
.package-action > span { color: #6e7781; font-size: .78rem; }
.new-session { margin-top: 1.4rem; padding-top: .8rem; border-top: 1px solid #e5e7eb; text-align: right; }

@media (max-width: 900px) {
  .task-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .trial-layout { grid-template-columns: 1fr; }
  .condition-progress { grid-template-columns: repeat(2, minmax(0, 1fr)); }
}
@media (max-width: 640px) {
  .runner-section { padding: 1rem; }
  .runner-steps button { padding: .7rem .25rem; font-size: .74rem; }
  .section-heading, .trial-heading { flex-direction: column; gap: .7rem; }
  .form-grid, .task-grid, .transfer-tools, .validation-list, .condition-progress { grid-template-columns: 1fr; }
  .wide { grid-column: auto; }
  .actions.split { align-items: stretch; flex-direction: column; }
  .download-actions { flex-direction: column; }
  .review-stats { grid-template-columns: 1fr; }
  .review-stats div { border-right: 0; border-bottom: 1px solid #dfe2e5; }
  .review-stats div:last-child { border-bottom: 0; }
  .submission-identity, .package-action { align-items: stretch; flex-direction: column; }
  .video-modal-backdrop { padding: .75rem; }
}
</style>
