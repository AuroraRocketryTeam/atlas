const pages = [...document.querySelectorAll('.page')];
const navButtons = [...document.querySelectorAll('nav button')];
const token = document.getElementById('token');
const progress = document.getElementById('progress');
const otaStatus = document.getElementById('otaStatus');
const actionStatus = document.getElementById('actionStatus');
const drop = document.getElementById('drop');
const firmware = document.getElementById('firmware');
const fileName = document.getElementById('fileName');
const fsmState = document.getElementById('fsmState');
const hilMode = document.getElementById('hilMode');
const fsmAdvance = document.getElementById('fsmAdvance');
const simulationWarning = document.getElementById('simulationWarning');
const logOutput = document.getElementById('logOutput');
const testLogOutput = document.getElementById('testLogOutput');
const logFilter = document.getElementById('logFilter');
const pauseLogsButton = document.getElementById('pauseLogs');

let tests = [];
let runtimeConfig = {};
let runtimeSchema = { fields: [] };
let runtimeValidation = { items: [] };
let selectedFile = null;
let latestStatus = null;
let sdkconfigText = null;
let sdkconfigLoading = false;
let logsPaused = false;
let otaUploadActive = false;
let lastLogSeq = 0;
let logLines = [];
let authQueue = Promise.resolve();
const pollBusy = {
  status: false,
  live: false,
  ota: false,
  tests: false,
  logs: false,
};
const MAX_CLIENT_LOG_LINES = 1000;

token.value = localStorage.getItem('atlas_ground_token') || '';

function saveToken() {
  localStorage.setItem('atlas_ground_token', token.value.trim());
}

function bytesToHex(bytes) {
  return [...bytes].map(b => b.toString(16).padStart(2, '0')).join('');
}

function utf8(value) {
  return new TextEncoder().encode(String(value || ''));
}

function rotr(x, n) {
  return (x >>> n) | (x << (32 - n));
}

function sha256Bytes(input) {
  const K = [
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
  ];
  const H = [0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19];
  const bytes = input instanceof Uint8Array ? input : new Uint8Array(input);
  const bitLen = bytes.length * 8;
  const msgLen = (((bytes.length + 9 + 63) >> 6) << 6);
  const msg = new Uint8Array(msgLen);
  msg.set(bytes);
  msg[bytes.length] = 0x80;
  const hi = Math.floor(bitLen / 0x100000000);
  const lo = bitLen >>> 0;
  msg[msgLen - 8] = (hi >>> 24) & 255;
  msg[msgLen - 7] = (hi >>> 16) & 255;
  msg[msgLen - 6] = (hi >>> 8) & 255;
  msg[msgLen - 5] = hi & 255;
  msg[msgLen - 4] = (lo >>> 24) & 255;
  msg[msgLen - 3] = (lo >>> 16) & 255;
  msg[msgLen - 2] = (lo >>> 8) & 255;
  msg[msgLen - 1] = lo & 255;

  const w = new Uint32Array(64);
  for (let offset = 0; offset < msg.length; offset += 64) {
    for (let i = 0; i < 16; i++) {
      const j = offset + i * 4;
      w[i] = ((msg[j] << 24) | (msg[j + 1] << 16) | (msg[j + 2] << 8) | msg[j + 3]) >>> 0;
    }
    for (let i = 16; i < 64; i++) {
      const s0 = (rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >>> 3)) >>> 0;
      const s1 = (rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >>> 10)) >>> 0;
      w[i] = (w[i - 16] + s0 + w[i - 7] + s1) >>> 0;
    }
    let [a,b,c,d,e,f,g,h] = H;
    for (let i = 0; i < 64; i++) {
      const S1 = (rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25)) >>> 0;
      const ch = ((e & f) ^ (~e & g)) >>> 0;
      const t1 = (h + S1 + ch + K[i] + w[i]) >>> 0;
      const S0 = (rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22)) >>> 0;
      const maj = ((a & b) ^ (a & c) ^ (b & c)) >>> 0;
      const t2 = (S0 + maj) >>> 0;
      h = g; g = f; f = e; e = (d + t1) >>> 0; d = c; c = b; b = a; a = (t1 + t2) >>> 0;
    }
    H[0] = (H[0] + a) >>> 0; H[1] = (H[1] + b) >>> 0; H[2] = (H[2] + c) >>> 0; H[3] = (H[3] + d) >>> 0;
    H[4] = (H[4] + e) >>> 0; H[5] = (H[5] + f) >>> 0; H[6] = (H[6] + g) >>> 0; H[7] = (H[7] + h) >>> 0;
  }
  const out = new Uint8Array(32);
  H.forEach((v, i) => {
    out[i * 4] = (v >>> 24) & 255;
    out[i * 4 + 1] = (v >>> 16) & 255;
    out[i * 4 + 2] = (v >>> 8) & 255;
    out[i * 4 + 3] = v & 255;
  });
  return out;
}

function concatBytes(a, b) {
  const out = new Uint8Array(a.length + b.length);
  out.set(a, 0);
  out.set(b, a.length);
  return out;
}

async function sha256Hex(data) {
  let bytes;
  if (data instanceof ArrayBuffer) bytes = new Uint8Array(data);
  else if (data instanceof Blob) bytes = new Uint8Array(await data.arrayBuffer());
  else bytes = utf8(data);
  return bytesToHex(sha256Bytes(bytes));
}

async function hmacSha256Hex(secret, message) {
  let key = utf8(secret);
  if (key.length > 64) key = sha256Bytes(key);
  const ipad = new Uint8Array(64);
  const opad = new Uint8Array(64);
  for (let i = 0; i < 64; i++) {
    const b = key[i] || 0;
    ipad[i] = b ^ 0x36;
    opad[i] = b ^ 0x5c;
  }
  return bytesToHex(sha256Bytes(concatBytes(opad, sha256Bytes(concatBytes(ipad, utf8(message))))));
}

async function getNonce() {
  const res = await fetch('/api/auth/nonce', { cache: 'no-store' });
  const body = await res.json();
  if (!res.ok || !body.ok || !body.nonce) throw new Error('Failed to obtain auth nonce');
  return body.nonce;
}

function enqueueAuth(work) {
  const run = authQueue.catch(() => {}).then(work);
  authQueue = run.catch(() => {});
  return run;
}

async function buildSignedHeaders(path, method, bodySha256, extra = {}) {
  const secret = token.value.trim();
  if (!secret) throw new Error('Missing HMAC shared secret');
  const nonce = await getNonce();
  const confirm = extra['X-Confirm'] || extra['x-confirm'] || '';
  const canonical = `${method}\n${path}\n${bodySha256}\n${nonce}\n${confirm}`;
  const signature = await hmacSha256Hex(secret, canonical);
  return {
    ...extra,
    'X-Auth-Nonce': nonce,
    'X-Auth-Body-SHA256': bodySha256,
    'X-Auth-Signature': signature,
  };
}

async function signedHeaders(path, method, bodySha256, extra = {}) {
  return enqueueAuth(() => buildSignedHeaders(path, method, bodySha256, extra));
}

async function authFetch(path, options = {}) {
  return enqueueAuth(async () => {
    const method = (options.method || 'GET').toUpperCase();
    const body = options.body || '';
    const bodySha256 = await sha256Hex(body);
    const signed = await buildSignedHeaders(path, method, bodySha256, options.headers || {});
    return fetch(path, { ...options, method, headers: signed, cache: 'no-store' });
  });
}

async function api(path, options = {}) {
  try {
    const res = await authFetch(path, options);
    const text = await res.text();
    let body;
    try { body = JSON.parse(text); } catch { body = { ok: res.ok, text }; }
    if (!res.ok) body.ok = false;
    return body;
  } catch (err) {
    return { ok: false, error: err.message || String(err) };
  }
}

async function apiText(path, options = {}) {
  const res = await authFetch(path, options);
  const text = await res.text();

  if (!res.ok) {
    throw new Error(text || `HTTP ${res.status}`);
  }

  return text;
}

function esc(value) {
  return String(value == null ? '' : value).replace(/[&<>"']/g, c => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
  }[c]));
}

function fmtBytes(value) {
  const n = Number(value || 0);
  if (n >= 1024 * 1024) return `${(n / 1024 / 1024).toFixed(1)} MB`;
  if (n >= 1024) return `${(n / 1024).toFixed(1)} KB`;
  return `${n} B`;
}

function fmtMs(ms) {
  const s = Math.floor(Number(ms || 0) / 1000);
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  return `${h}h ${m}m ${s % 60}s`;
}

function showActionStatus(value) {
  actionStatus.classList.remove('hidden');
  actionStatus.textContent = typeof value === 'string' ? value : JSON.stringify(value, null, 2);
}

function show(page) {
  pages.forEach(p => p.classList.toggle('hidden', p.id !== page));
  navButtons.forEach(b => b.classList.toggle('active', b.dataset.page === page));
  if (page === 'tests' && tests.length === 0) loadTests();
  if (page === 'health') loadHealth();
  if (page === 'config') loadConfig();
  if (page === 'info') loadInfo();
  if (page === 'live') loadLive();
  if (page === 'ota') refreshOta();
  if (page === 'logs') loadLogs();
}

function runPoll(name, visible, work) {
  if (!visible || pollBusy[name]) return;
  pollBusy[name] = true;
  Promise.resolve()
    .then(work)
    .catch(() => {})
    .finally(() => { pollBusy[name] = false; });
}

function row(label, value) {
  return `<div class="info-row"><span>${esc(label)}</span><strong>${esc(value)}</strong></div>`;
}

function panel(title, content) {
  return `<section class="panel"><h2>${esc(title)}</h2>${content}</section>`;
}

function checklistFailures(checklist) {
  return (checklist.items || []).filter(item => !item.ok);
}

function checklistSummary(checklist) {
  const failures = checklistFailures(checklist);
  if (failures.length === 0) return 'Pre-launch checklist passed.';
  return failures.map(item => `${item.label || item.key}: ${item.message || item.severity || 'not ready'}`).join('\n');
}

function checklistPanel(checklist) {
  if (!checklist || !Array.isArray(checklist.items)) {
    return panel('Pre-launch Checklist', '<p class="bad">Checklist unavailable.</p>');
  }
  const rows = checklist.items.map(item => `
    <div class="checklist-item ${item.ok ? 'ok' : 'bad'}">
      <span>${item.ok ? 'OK' : 'BLOCKED'}</span>
      <div>
        <strong>${esc(item.label || item.key)}</strong>
        <small>${esc(item.message || '')}</small>
      </div>
    </div>`).join('');
  return panel('Pre-launch Checklist', `
    <div class="checklist-status ${checklist.ok ? 'ok' : 'bad'}">${checklist.ok ? 'Ready for launch transition' : 'Manual review required'}</div>
    <div class="checklist-list">${rows}</div>
  `);
}

function updateFsmBar(status) {
  latestStatus = status;
  const state = (status && status.fsm_state) || 'UNKNOWN';
  fsmState.textContent = state;
  fsmState.className = `state-pill state-${state.toLowerCase()}`;

  const hil = (status && status.hil) || {};
  const hasHilMode = typeof hil.simulation === 'boolean';
  if (hasHilMode) {
    const simulation = hil.simulation === true;
    const support = hil.support === true;
    const modeLabel = simulation ? 'SIMULATION MODE' : 'FLIGHT MODE';
    hilMode.textContent = support && !simulation ? `${modeLabel} / HIL READY` : modeLabel;
    hilMode.className = `mode-pill ${simulation ? 'mode-simulation' : 'mode-flight'}${support && !simulation ? ' mode-hil-ready' : ''}`;
    simulationWarning.classList.toggle('hidden', !simulation);
  } else {
    hilMode.textContent = 'UNKNOWN';
    hilMode.className = 'mode-pill mode-unknown';
    simulationWarning.classList.add('hidden');
  }

  if (state === 'GROUND_SERVICES') {
    fsmAdvance.textContent = 'Go To Ready For Launch';
    fsmAdvance.classList.remove('hidden');
    fsmAdvance.classList.add('danger-action');
  } else {
    fsmAdvance.classList.add('hidden');
    fsmAdvance.classList.remove('danger-action');
  }
}

async function pollStatus() {
  if (otaUploadActive) return;
  if (pollBusy.status) return;
  pollBusy.status = true;
  let s;
  try {
    s = await api('/api/status');
  } finally {
    pollBusy.status = false;
  }
  if (!s || !s.ok) return;

  updateFsmBar(s);

  // Optional: update only small fields if you give them IDs.
  // Do not replace document.getElementById('info').innerHTML here.
}

async function loadInfo() {
  const [s, checklist] = await Promise.all([
    api('/api/status'),
    api('/api/prelaunch/checklist')
  ]);
  if (!s.ok) {
    document.getElementById('info').innerHTML = panel('Info', `<p class="bad">Unauthorized or unavailable.</p><pre>${esc(JSON.stringify(s, null, 2))}</pre>`);
    return;
  }
  updateFsmBar(s);

  const app = s.app || {};
  const chip = s.chip || {};
  const network = s.network || {};
  const reset = s.reset || {};
  const flash = s.flash || {};
  const memory = s.memory || {};
  const partitions = s.partitions || {};
  const running = partitions.running || {};
  const boot = partitions.configured_boot || {};
  const security = s.security || {};
  const runtime = s.runtime_config || {};
  const featurePills = (chip.features || []).map(f => `<span>${esc(f)}</span>`).join('');
  document.getElementById('info').innerHTML = `
    <section class="hero">
      <div>
        <h2>${esc(chip.model)} flight computer</h2>
        <p>${esc(app.project_name)} ${esc(app.version)} - ${esc(network.ssid)} - ${esc(network.ip)}</p>
      </div>
      <div class="feature-pills">${featurePills}</div>
    </section>
    <div class="info-grid">
      ${panel('Firmware', [
        row('Project', app.project_name),
        row('Version', app.version),
        row('Build', `${app.date || ''} ${app.time || ''}`),
        row('ESP-IDF', app.idf_version),
        row('Uptime', fmtMs(s.uptime_ms)),
        row('Reset', `${reset.reason} (${reset.code})`)
      ].join(''))}
      ${panel('Chip', [
        row('Model', chip.model),
        row('Revision', chip.revision),
        row('CPU cores', chip.cores),
        row('Flash size', fmtBytes(flash.size_bytes)),
        row('Flash ID', flash.id)
      ].join(''))}
      ${panel('Memory', [
        row('Heap total', fmtBytes(memory.heap_total)),
        row('Heap free', fmtBytes(memory.heap_free)),
        row('Heap low water', fmtBytes(memory.heap_min_free)),
        row('PSRAM total', fmtBytes(memory.psram_total)),
        row('PSRAM free', fmtBytes(memory.psram_free))
      ].join(''))}
      ${panel('Partitions', [
        row('Running', `${running.label} @ 0x${Number(running.address || 0).toString(16)}`),
        row('Running size', fmtBytes(running.size)),
        row('Boot', `${boot.label} @ 0x${Number(boot.address || 0).toString(16)}`),
        row('Boot size', fmtBytes(boot.size))
      ].join(''))}
      ${panel('Network', [
        row('SSID', network.ssid),
        row('IP address', network.ip),
        row('MAC', network.mac)
      ].join(''))}
      ${panel('Security', [
        row('Secure boot', security.secure_boot ? 'enabled' : 'disabled'),
        row('Flash encryption', security.flash_encryption ? 'enabled' : 'disabled'),
        row('Config schema', runtime.schema_version),
        row('Config revision', runtime.config_revision)
      ].join(''))}
      ${checklistPanel(checklist)}
    </div>
    <section class="panel sdkconfig-panel">
      <div class="section-head">
        <h2>SDK Config Snapshot</h2>
        <button id="loadSdkconfig">Load sdkconfig</button>
      </div>
      <pre id="sdkconfigBox">${esc(
        sdkconfigLoading ? 'Loading sdkconfig...' :
        sdkconfigText ? sdkconfigText :
        'Not loaded.'
      )}</pre>
    </section>`;
  document.getElementById('loadSdkconfig').addEventListener('click', loadSdkconfig);
}

function healthItem(name, item, detail) {
  item = item || {};
  const ok = Object.prototype.hasOwnProperty.call(item, 'size_ok')
    ? item.size_ok === true
    : (item.expected === false && item.present === false ? true : (item.status ? item.status === 'ok' : item.present === true));
  const state = ok ? 'OK' : 'X';
  const status = item.status || (item.present ? 'present' : 'not_present');
  return `<div class="sensor-card ${ok ? 'ok' : 'bad'}">
    <div class="sensor-head"><h3>${esc(name)}</h3><span>${state} ${esc(status)}</span></div>
    ${detail}
  </div>`;
}

async function loadHealth() {
  const h = await api('/api/health');
  if (!h.ok) {
    document.getElementById('healthContent').innerHTML = panel('System Health', '<p class="bad">Unauthorized or unavailable.</p>');
    return;
  }
  const sensors = h.sensors || {};
  const memory = h.memory || {};
  const psram = memory.psram || {};
  const sd = (h.storage || {}).sd || {};
  document.getElementById('healthContent').innerHTML = `
    <div class="sensor-grid">
      ${healthItem('PSRAM', psram, [
        row('Total', fmtBytes(psram.total_bytes)),
        row('Free', fmtBytes(psram.free_bytes))
      ].join(''))}
      ${healthItem('Internal Flash', h.internal_flash, [
        row('Detected', fmtBytes((h.internal_flash || {}).size_bytes)),
        row('Expected', fmtBytes((h.internal_flash || {}).expected_size_bytes)),
        row('16 MB match', (h.internal_flash || {}).size_ok ? 'yes' : 'no')
      ].join(''))}
      ${healthItem('External Flash', h.external_flash, [
        row('Mounted', (h.external_flash || {}).present ? 'yes' : 'no')
      ].join(''))}
      ${healthItem('BNO055 IMU', sensors.imu_bno055, [row('Read status', (sensors.imu_bno055 || {}).status)].join(''))}
      ${healthItem('MS5611 Barometer 1', sensors.barometer_ms5611_primary, [row('Read status', (sensors.barometer_ms5611_primary || {}).status)].join(''))}
      ${healthItem('MS5611 Barometer 2', sensors.barometer_ms5611_secondary, [row('Read status', (sensors.barometer_ms5611_secondary || {}).status)].join(''))}
      ${healthItem('LIS3DHTR Accelerometer', sensors.accelerometer_lis3dhtr, [row('Read status', (sensors.accelerometer_lis3dhtr || {}).status)].join(''))}
      ${healthItem('GPS', sensors.gps, [row('Read status', (sensors.gps || {}).status)].join(''))}
      ${healthItem('SD Card', sd, [row('Mounted', sd.present ? 'yes' : 'no')].join(''))}
    </div>`;
}

async function loadSdkconfig() {
  const box = document.getElementById('sdkconfigBox');

  try {
    sdkconfigLoading = true;
    if (box) box.textContent = 'Loading sdkconfig...';

    sdkconfigText = await apiText('/api/config/sdkconfig');

    const newBox = document.getElementById('sdkconfigBox');
    if (newBox) newBox.textContent = sdkconfigText;
  } catch (err) {
    sdkconfigText = err.message || String(err);

    const newBox = document.getElementById('sdkconfigBox');
    if (newBox) newBox.textContent = sdkconfigText;
  } finally {
    sdkconfigLoading = false;
  }
}

function filteredLogs(text) {
  const filter = (logFilter && logFilter.value || '').trim().toLowerCase();
  if (!filter) return text;
  return text.split('\n').filter(line => line.toLowerCase().includes(filter)).join('\n');
}

function isNearBottom(output) {
  if (!output) return false;
  return output.scrollHeight - output.scrollTop - output.clientHeight < 24;
}

function scrollToBottom(output) {
  if (output) output.scrollTop = output.scrollHeight;
}

function renderLogs(output = logOutput, options = {}) {
  if (!output) return;
  const shouldStick = options.forceBottom || isNearBottom(output);
  output.textContent = filteredLogs(logLines.join('\n')) || 'No log lines captured yet.';
  if (shouldStick) scrollToBottom(output);
}

async function loadLogs(options = {}) {
  if (pollBusy.logs && !options.force) return;
  pollBusy.logs = true;
  const output = options.output || logOutput;
  if (logsPaused && output === logOutput && !options.force) {
    pollBusy.logs = false;
    return;
  }
  try {
    const path = `/api/logs?since=${lastLogSeq}`;
    const res = await authFetch(path);
    const text = await res.text();
    if (!res.ok) throw new Error(text || `HTTP ${res.status}`);

    const latest = Number(res.headers.get('X-Log-Latest-Seq') || lastLogSeq);
    if (Number.isFinite(latest) && latest >= lastLogSeq) lastLogSeq = latest;

    const newLines = text.split('\n').filter(line => line.length > 0);
    if (newLines.length > 0) {
      logLines.push(...newLines);
      if (logLines.length > MAX_CLIENT_LOG_LINES) {
        logLines = logLines.slice(logLines.length - MAX_CLIENT_LOG_LINES);
      }
    }
    renderLogs(output, { forceBottom: options.forceBottom });
  } catch (err) {
    output.textContent = err.message || String(err);
  } finally {
    pollBusy.logs = false;
  }
}

function clearLogsView() {
  // logsPaused = false;
  // pauseLogsButton.textContent = 'Resume';
  logLines = [];
  renderLogs(logOutput, { forceBottom: true });
}

async function copyTextFrom(output) {
  const text = output ? output.textContent : '';
  if (!text) return;
  try {
    await navigator.clipboard.writeText(text);
  } catch {
    const area = document.createElement('textarea');
    area.value = text;
    document.body.appendChild(area);
    area.select();
    document.execCommand('copy');
    document.body.removeChild(area);
  }
}

function sensorCard(name, sensor, values) {
  sensor = sensor || {};
  const ok = sensor.status === 'ok';
  const status = ok ? 'OK' : (sensor.status || 'missing');
  return `<div class="sensor-card ${ok ? 'ok' : 'bad'}">
    <div class="sensor-head"><h3>${esc(name)}</h3><span>${ok ? 'OK' : 'X'} ${esc(status)}</span></div>
    ${values.map(([k, v]) => row(k, v)).join('')}
  </div>`;
}

async function loadLive() {
  const s = await api('/api/live-data');
  if (!s.ok) {
    document.getElementById('liveContent').innerHTML = panel('Live Data', `<p class="bad">Unauthorized or unavailable.</p>`);
    return;
  }
  updateFsmBar({ ...(latestStatus || {}), fsm_state: s.fsm_state || 'UNKNOWN' });
  const flight = s.flight || {};
  const calibration = s.calibration || {};
  const sensors = s.sensors || {};
  const imu = sensors.imu || {};
  const barometer = sensors.barometer || {};
  const gps = sensors.gps || {};
  document.getElementById('liveContent').innerHTML = `
    <div class="metrics">
      <div><span>Height</span><strong>${Number(flight.height_m || 0).toFixed(2)} m</strong></div>
      <div><span>Vertical speed</span><strong>${Number(flight.vertical_speed_mps || 0).toFixed(2)} m/s</strong></div>
      <div><span>Rising</span><strong>${flight.is_rising ? 'yes' : 'no'}</strong></div>
      <div><span>Calibrated</span><strong>${calibration.imu ? 'yes' : 'no'}</strong></div>
    </div>
    <div class="sensor-grid">
      ${sensorCard('IMU', imu, [
        ['Acceleration X', Number(imu.ax || 0).toFixed(4)],
        ['Acceleration Y', Number(imu.ay || 0).toFixed(4)],
        ['Acceleration Z', Number(imu.az || 0).toFixed(4)],
        ['Temperature', `${Number(imu.temperature_c || 0).toFixed(1)} C`]
      ])}
      ${sensorCard('Barometer', barometer, [
        ['Pressure', Number(barometer.pressure || 0).toFixed(2)],
        ['Temperature', `${Number(barometer.temperature_c || 0).toFixed(1)} C`],
        ['Zeroed', calibration.barometer ? 'yes' : 'no'],
        ['Samples', calibration.barometer_samples || 0]
      ])}
      ${sensorCard('GPS', gps, [
        ['Fix', gps.fix ? 'yes' : 'no'],
        ['Satellites', gps.satellites || 0],
        ['Latitude', Number(gps.lat || 0).toFixed(7)],
        ['Longitude', Number(gps.lon || 0).toFixed(7)]
      ])}
    </div>`;
}

async function refreshOta() {
  const s = await api('/api/ota/status');
  progress.value = s.progress || 0;
  otaStatus.textContent = JSON.stringify(s, null, 2);
}

function configInput(key, value) {
  const meta = runtimeSchema.fields.find(field => field.key === key) || { key, label: key, editable: false, group: 'Read-only Firmware' };
  const locked = runtimeConfig.config_locked && meta.locked_after_ready;
  const editable = meta.editable && !locked;
  const issue = (runtimeValidation.items || []).find(item => item.field === key || item.field.endsWith(`.${key}`));
  const classes = [
    'config-field',
    editable ? 'editable' : 'readonly',
    locked ? 'locked' : '',
    issue && issue.severity === 'error' ? 'invalid' : '',
    issue && issue.severity === 'warning' ? 'warning' : ''
  ].filter(Boolean).join(' ');
  const badges = [
    !editable ? `<em class="badge readonly">${locked ? 'locked' : 'read-only'}</em>` : ''
  ].join('');
  const disabled = editable ? '' : ' disabled';
  let input;
  if (typeof value === 'boolean') {
    input = `<input data-config="${esc(key)}" type="checkbox" ${value ? 'checked' : ''}${disabled}>`;
  } else if (typeof value === 'number') {
    const min = Object.prototype.hasOwnProperty.call(meta, 'min') ? ` min="${esc(meta.min)}"` : '';
    const max = Object.prototype.hasOwnProperty.call(meta, 'max') ? ` max="${esc(meta.max)}"` : '';
    input = `<input data-config="${esc(key)}" type="number" step="any"${min}${max} value="${esc(value)}"${disabled}${editable ? ' required' : ''}>`;
  } else {
    input = `<input data-config="${esc(key)}" type="text" value="${esc(value)}"${disabled}>`;
  }
  return `<label class="${classes}">
    <span class="config-label"><strong>${esc(meta.label || key)}</strong>${badges}</span>
    <span class="config-control">${input}${meta.unit ? `<small>${esc(meta.unit)}</small>` : ''}</span>
    <span class="config-desc">${esc(meta.description || '')}</span>
    ${issue ? `<span class="config-message">${esc(issue.message)}</span>` : ''}
  </label>`;
}

async function loadConfig() {
  const [cfg, schema, validation] = await Promise.all([
    api('/api/config/runtime'),
    api('/api/config/schema'),
    api('/api/config/validation')
  ]);
  runtimeConfig = cfg;
  runtimeSchema = schema.ok ? schema : { fields: [] };
  runtimeValidation = validation.ok === false || validation.ok === true ? validation : { items: [] };
  const groups = (runtimeSchema.fields || []).reduce((acc, field) => {
    if (!acc[field.group]) acc[field.group] = [];
    acc[field.group].push(field);
    return acc;
  }, {});
  document.getElementById('configEditor').innerHTML = Object.entries(groups).map(([group, fields]) =>
    `<section class="config-group"><h3>${esc(group)}</h3><div class="config-grid">
      ${fields.map(field => configInput(field.key, runtimeConfig[field.key])).join('')}
    </div></section>`
  ).join('');
  const locked = runtimeConfig.config_locked ? 'LOCKED FOR FLIGHT' : 'Editable in Ground Services';
  const valid = runtimeValidation.ok ? 'valid' : 'invalid';
  document.getElementById('configStatus').textContent = `${locked} - ${valid}\n${JSON.stringify(runtimeValidation, null, 2)}`;
  document.getElementById('unlockConfig').disabled = !runtimeConfig.config_locked;
}

async function saveConfig() {
  const inputs = [...document.querySelectorAll('[data-config]')];
  const invalid = inputs.find(input => !input.disabled && !input.checkValidity());
  if (invalid) {
    invalid.reportValidity();
    return;
  }
  const updated = { ...runtimeConfig };
  inputs.forEach(input => {
    if (input.disabled) return;
    const key = input.dataset.config;
    if (input.type === 'checkbox') updated[key] = input.checked;
    else if (input.type === 'number') updated[key] = Number(input.value);
    else updated[key] = input.value;
  });
  const s = await api('/api/config/runtime', {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(updated)
  });
  document.getElementById('configStatus').textContent = JSON.stringify(s, null, 2);
  loadConfig();
}

async function resetConfig() {
  const s = await api('/api/config/reset-defaults', { method: 'POST' });
  document.getElementById('configStatus').textContent = JSON.stringify(s, null, 2);
  loadConfig();
}

async function unlockConfig() {
  const prompt = 'This clears only the RuntimeConfig lock bit after recovery/inspection.\nThe saved mission configuration remains in NVS and is not reset or erased.\n\nType UNLOCK_AFTER_RECOVERY to unlock editing.';
  if (window.prompt(prompt) !== 'UNLOCK_AFTER_RECOVERY') return;
  const s = await api('/api/config/unlock-after-recovery', {
    method: 'POST',
    headers: { 'X-Confirm': 'UNLOCK_AFTER_RECOVERY' },
    body: JSON.stringify({ confirm: 'UNLOCK_AFTER_RECOVERY' })
  });
  document.getElementById('configStatus').textContent = JSON.stringify(s, null, 2);
  showActionStatus(s);
  loadConfig();
  loadInfo();
}

function testCard(t) {
  const confirm = t.destructive ? ` data-confirm="${esc(t.confirmation || '')}"` : '';
  return `<div class="test-row">
    <div>
      <div class="test-name">${esc(t.name)}</div>
      <div class="test-desc">${esc(t.description)}</div>
      ${t.destructive ? '<div class="danger">Requires confirmation</div>' : ''}
    </div>
    <button data-test="${esc(t.id)}"${confirm}>Run</button>
  </div>`;
}

async function loadTests() {
  const s = await api('/api/tests');
  if (!s.available) {
    document.getElementById('testGroups').innerHTML = '<div class="panel">Test runner unavailable</div>';
    return;
  }
  tests = s.tests || [];
  const groups = tests.reduce((acc, t) => {
    if (!acc[t.group]) acc[t.group] = [];
    acc[t.group].push(t);
    return acc;
  }, {});
  document.getElementById('testGroups').innerHTML = Object.entries(groups).map(([group, items]) =>
    `<section class="test-group"><h2>${esc(group)}</h2>${items.map(testCard).join('')}</section>`
  ).join('');
  document.querySelectorAll('[data-test]').forEach(button => button.addEventListener('click', startTest));
  refreshTests();
}

async function refreshTests() {
  const s = await api('/api/tests/status');
  document.getElementById('testStatus').innerHTML = `<strong>${esc(s.state || 'unknown')}</strong> ${esc(s.active_name || '')} ${esc(s.message || '')}`;
  const prompt = document.getElementById('testPrompt');
  prompt.classList.toggle('hidden', !s.waiting_for_verdict);
  document.getElementById('testPromptText').textContent = s.prompt || '';
  document.querySelectorAll('[data-test-verdict]').forEach(button => {
    button.disabled = !s.waiting_for_verdict;
  });
  const testLogPanel = document.getElementById('testLogPanel');
  const showTestLogs = s.running || s.waiting_for_verdict;
  const testLogWasHidden = testLogPanel.classList.contains('hidden');
  testLogPanel.classList.toggle('hidden', !showTestLogs);
  if (showTestLogs) loadLogs({ output: testLogOutput, forceBottom: testLogWasHidden });
}

async function startTest(event) {
  const button = event.currentTarget;
  const id = Number(button.dataset.test);
  const confirm = button.dataset.confirm;
  if (confirm && window.prompt(`Type ${confirm} to run this test`) !== confirm) return;
  const s = await api('/api/tests/start', {
    method: 'POST',
    headers: confirm ? { 'X-Confirm': confirm } : {},
    body: JSON.stringify({ id })
  });
  document.getElementById('testStatus').textContent = JSON.stringify(s, null, 2);
  refreshTests();
}

async function sendVerdict(verdict) {
  document.querySelectorAll('[data-test-verdict]').forEach(button => {
    button.disabled = true;
  });
  const headersExtra = verdict === 'reboot' ? { 'X-Confirm': 'REBOOT_FROM_TEST' } : {};
  const s = await api('/api/tests/verdict', {
    method: 'POST',
    headers: headersExtra,
    body: JSON.stringify({ verdict })
  });
  document.getElementById('testStatus').textContent = JSON.stringify(s, null, 2);
  refreshTests();
}

async function uploadFirmware() {
  const file = selectedFile || firmware.files[0];
  if (!file) return;
  if (!file.name.endsWith('.bin') && !window.confirm('The selected file does not end with .bin. Continue?')) return;
  if (!window.confirm('Start OTA upload? The image will be written to the inactive OTA partition.')) return;

  otaStatus.textContent = 'Computing firmware SHA-256...';
  let fileHash;
  let signed;
  try {
    fileHash = await sha256Hex(file);
    signed = await signedHeaders('/api/ota/upload', 'POST', fileHash, {
      'Content-Type': 'application/octet-stream',
      'X-Confirm': 'START_OTA',
      'X-Firmware-Name': file.name,
      'X-Firmware-SHA256': fileHash,
    });
  } catch (err) {
    otaStatus.textContent = err.message || String(err);
    return;
  }

  const xhr = new XMLHttpRequest();
  otaUploadActive = true;
  xhr.open('POST', '/api/ota/upload', true);
  Object.entries(signed).forEach(([key, value]) => xhr.setRequestHeader(key, value));
  xhr.upload.onprogress = event => {
    if (event.lengthComputable) progress.value = Math.round((event.loaded / event.total) * 100);
  };
  xhr.onload = () => {
    otaUploadActive = false;
    try { otaStatus.textContent = JSON.stringify(JSON.parse(xhr.responseText), null, 2); }
    catch { otaStatus.textContent = xhr.responseText; }
    refreshOta();
  };
  xhr.onerror = () => {
    otaUploadActive = false;
    otaStatus.textContent = 'Upload failed';
    refreshOta();
  };
  xhr.onabort = xhr.onerror;
  xhr.ontimeout = xhr.onerror;
  xhr.send(file);
}

async function advanceFsm() {
  if (!latestStatus) {
    const status = await api('/api/status');
    if (status.ok) updateFsmBar(status);
  }
  if (!latestStatus || latestStatus.fsm_state !== 'GROUND_SERVICES') {
    showActionStatus('Ready-for-launch is only available in GROUND_SERVICES.');
    return;
  }
  let checklist = await api('/api/prelaunch/checklist');
  if (!checklist.ok) {
    const summary = checklistSummary(checklist);
    const prompt = `Pre-launch checklist is not complete:\n\n${summary}\n\nType READY_FOR_LAUNCH to manually override these checklist blockers and lock the flight configuration.`;
    showActionStatus(`Pre-launch checklist is not complete:\n${summary}`);
    if (window.prompt(prompt) !== 'READY_FOR_LAUNCH') return;
    const override = await api('/api/fsm/ready-for-launch', {
      method: 'POST',
      headers: { 'X-Confirm': 'READY_FOR_LAUNCH' },
      body: JSON.stringify({ override: 'READY_FOR_LAUNCH' })
    });
    showActionStatus(override);
    loadInfo();
    return;
  }
  const prompt = 'This will lock the flight configuration.\nAfter this point, mission parameters cannot be edited until the allowed recovery/reset path.\nConfirm that the pre-launch checklist is complete.\n\nType READY_FOR_LAUNCH to lock and arm.';
  if (window.prompt(prompt) !== 'READY_FOR_LAUNCH') return;
  const s = await api('/api/fsm/ready-for-launch', {
    method: 'POST',
    headers: { 'X-Confirm': 'READY_FOR_LAUNCH' },
    body: ''
  });
  showActionStatus(s);
  loadInfo();
}

navButtons.forEach(b => b.addEventListener('click', () => show(b.dataset.page)));
token.addEventListener('input', saveToken);
token.addEventListener('change', () => { loadInfo(); loadLive(); loadConfig(); refreshOta(); loadTests(); });
firmware.addEventListener('change', () => {
  selectedFile = firmware.files[0] || null;
  fileName.textContent = selectedFile ? `${selectedFile.name} (${fmtBytes(selectedFile.size)})` : 'No file selected';
});
['dragenter', 'dragover'].forEach(eventName => {
  drop.addEventListener(eventName, event => { event.preventDefault(); drop.classList.add('drag'); });
});
['dragleave', 'drop'].forEach(eventName => {
  drop.addEventListener(eventName, event => { event.preventDefault(); drop.classList.remove('drag'); });
});
drop.addEventListener('drop', event => {
  const file = event.dataTransfer.files[0];
  if (!file) return;
  selectedFile = file;
  fileName.textContent = `${file.name} (${fmtBytes(file.size)})`;
});
document.getElementById('upload').addEventListener('click', uploadFirmware);
document.getElementById('reboot').addEventListener('click', async () => {
  const s = await api('/api/ota/reboot', {
    method: 'POST',
    headers: { 'X-Confirm': 'REBOOT_TO_NEW_FIRMWARE' },
    body: 'REBOOT_TO_NEW_FIRMWARE'
  });
  otaStatus.textContent = JSON.stringify(s, null, 2);
});
fsmAdvance.addEventListener('click', advanceFsm);
document.getElementById('testPassed').addEventListener('click', () => sendVerdict('passed'));
document.getElementById('testRetry').addEventListener('click', () => sendVerdict('retry'));
document.getElementById('testExit').addEventListener('click', () => sendVerdict('exit'));
document.getElementById('testReboot').addEventListener('click', () => {
  if (window.prompt('Type REBOOT_FROM_TEST to reboot') === 'REBOOT_FROM_TEST') sendVerdict('reboot');
});
document.addEventListener('keydown', event => {
  if (document.getElementById('tests').classList.contains('hidden')) return;
  if (event.defaultPrevented || event.ctrlKey || event.metaKey || event.altKey) return;
  const tag = (event.target && event.target.tagName || '').toLowerCase();
  if (tag === 'input' || tag === 'textarea' || tag === 'select') return;
  const key = event.key.toLowerCase();
  if (key === 'p' && !document.getElementById('testPassed').disabled) {
    event.preventDefault();
    sendVerdict('passed');
  } else if (key === 'r' && !document.getElementById('testRetry').disabled) {
    event.preventDefault();
    sendVerdict('retry');
  } else if (key === 'e' && !document.getElementById('testExit').disabled) {
    event.preventDefault();
    sendVerdict('exit');
  }
});
document.getElementById('saveConfig').addEventListener('click', saveConfig);
document.getElementById('resetConfig').addEventListener('click', resetConfig);
document.getElementById('unlockConfig').addEventListener('click', unlockConfig);

setInterval(() => runPoll('live', !otaUploadActive && !document.getElementById('live').classList.contains('hidden'), loadLive), 1000);
setInterval(() => runPoll('ota', !otaUploadActive && !document.getElementById('ota').classList.contains('hidden'), refreshOta), 2000);
setInterval(() => runPoll('tests', !otaUploadActive && !document.getElementById('tests').classList.contains('hidden'), refreshTests), 1500);
setInterval(() => { if (!otaUploadActive && !document.getElementById('logs').classList.contains('hidden')) loadLogs(); }, 1500);
setInterval(pollStatus, 5000);
document.getElementById('refreshLogs').addEventListener('click', loadLogs);
pauseLogsButton.addEventListener('click', () => {
  logsPaused = !logsPaused;
  pauseLogsButton.textContent = logsPaused ? 'Resume' : 'Pause';
  if (!logsPaused) loadLogs({ force: true });
});
document.getElementById('clearLogs').addEventListener('click', clearLogsView);
document.getElementById('copyLogs').addEventListener('click', () => copyTextFrom(logOutput));
document.getElementById('scrollLogsBottom').addEventListener('click', () => { scrollToBottom(logOutput); });
document.getElementById('refreshTestLogs').addEventListener('click', () => loadLogs({ output: testLogOutput, force: true, forceBottom: true }));
document.getElementById('copyTestLogs').addEventListener('click', () => copyTextFrom(testLogOutput));
logFilter.addEventListener('input', () => renderLogs(logOutput));
show('info');
