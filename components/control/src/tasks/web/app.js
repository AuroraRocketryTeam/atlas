const pages = [...document.querySelectorAll('.page')];
const navButtons = [...document.querySelectorAll('nav button')];
const protectedNavButtons = [...document.querySelectorAll('nav button[data-auth="required"]')];
const pageRoutes = {
  info: '/', health: '/health', live: '/live-data', config: '/config', ota: '/ota',
  tests: '/tests', logs: '/serial-monitor', files: '/files'
};
const routePages = Object.fromEntries(Object.entries(pageRoutes).map(([page, route]) => [route, page]));
const token = document.getElementById('token');
const authenticateButton = document.getElementById('authenticate');
const authState = document.getElementById('authState');
const progress = document.getElementById('progress');
const otaStatus = document.getElementById('otaStatus');
const actionStatus = document.getElementById('actionStatus');
const drop = document.getElementById('drop');
const firmware = document.getElementById('firmware');
const fileName = document.getElementById('fileName');
const fsmState = document.getElementById('fsmState');
const hilMode = document.getElementById('hilMode');
const fsmAdvance = document.getElementById('fsmAdvance');
const readyConfigGuard = document.getElementById('readyConfigGuard');
const simulationWarning = document.getElementById('simulationWarning');
const logOutput = document.getElementById('logOutput');
const testLogOutput = document.getElementById('testLogOutput');
const logFilter = document.getElementById('logFilter');
const pauseLogsButton = document.getElementById('pauseLogs');
const pauseTestLogsButton = document.getElementById('pauseTestLogs');

let tests = [];
let runtimeConfig = {};
let runtimeSchema = { fields: [] };
let runtimeValidation = { items: [] };
let persistedConfigSignature = '';
let configDirty = false;
let selectedFile = null;
let latestStatus = null;
let sdkconfigText = null;
let sdkconfigLoading = false;
let taskSnapshot = null;
let logsPaused = false;
let testLogsPaused = false;
let closeTestLogWhenFinished = false;
let otaUploadActive = false;
let lastLogSeq = 0;
let logLines = [];
let testLogLines = [];
let testLogVisible = false;
let authQueue = Promise.resolve();
let isAuthenticated = false;
const pollBusy = {
  status: false,
  live: false,
  health: false,
  ota: false,
  tests: false,
  logs: false,
};
const MAX_CLIENT_LOG_LINES = 1000;
const LIVE_HISTORY_SAMPLES = 120;
const liveHistory = [];
let targetAttitudeQuaternion = { w: 1, x: 0, y: 0, z: 0 };
let displayedAttitudeQuaternion = { w: 1, x: 0, y: 0, z: 0 };
let targetAttitudeAcceleration = [0, 0, 0];
let displayedAttitudeAcceleration = [0, 0, 0];
let lastLoRaTxSuccess = 0;
let loRaTxPulseUntil = 0;
let liveSocket = null;
let liveReconnectTimer = null;
let liveReconnectDelayMs = 1500;
let logSocket = null;
let logReconnectTimer = null;
let logReconnectDelayMs = 1500;
const WS_RECONNECT_MAX_MS = 10000;
const nextReconnectDelay = delay => Math.min(delay * 2, WS_RECONNECT_MAX_MS);
console.assert([1500, 3000, 6000, 10000].map(nextReconnectDelay).join() === '3000,6000,10000,10000',
  'WebSocket reconnect backoff check failed');
const DEFAULT_ATTITUDE_MOUNTING = { x: '-x', y: '-y', z: '+z' };
let attitudeMounting = loadAttitudeMounting();
let attitudeMountingDraft = { ...attitudeMounting };

token.value = localStorage.getItem('atlas_ground_token') || '';

function saveToken() {
  localStorage.setItem('atlas_ground_token', token.value.trim());
}

function setAuthState(authenticated, message = '') {
  isAuthenticated = authenticated;
  document.body.classList.toggle('authenticated', authenticated);
  authState.className = `auth-state ${authenticated ? 'operator' : 'guest'}`;
  authState.textContent = message || (authenticated ? 'OPERATOR · controls unlocked' : 'VIEWER · monitoring only');
  authenticateButton.textContent = authenticated ? 'Lock Controls' : 'Unlock Controls';
  protectedNavButtons.forEach(button => { button.disabled = !authenticated; });
  if (!authenticated) {
    const current = pages.find(page => !page.classList.contains('hidden'));
    if (current && protectedNavButtons.some(button => button.dataset.page === current.id)) show('info');
  }
  if (latestStatus) updateFsmBar(latestStatus);
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

async function authenticateOperator() {
  authState.textContent = 'AUTHENTICATING…';
  try {
    const res = await authFetch('/api/config/schema');
    if (!res.ok) throw new Error('Authentication failed');
    setAuthState(true);
  } catch (err) {
    setAuthState(false, `VIEWER · ${err.message || 'authentication failed'}`);
  }
}

async function authFetch(path, options = {}) {
  return enqueueAuth(async () => {
    const method = (options.method || 'GET').toUpperCase();
    const body = options.body || '';
    const bodySha256 = await sha256Hex(body);
    const headers = await buildSignedHeaders(path, method, bodySha256, options.headers || {});
    const res = await fetch(path, { ...options, method, headers, cache: 'no-store' });
    if (res.status === 401) setAuthState(false, 'VIEWER · authentication failed');
    return res;
  });
}

async function publicApi(path) {
  try {
    const res = await fetch(path, { cache: 'no-store' });
    const text = await res.text();
    let body;
    try { body = JSON.parse(text); } catch { body = { ok: res.ok, text }; }
    if (!res.ok) body.ok = false;
    return body;
  } catch (err) {
    return { ok: false, error: err.message || String(err) };
  }
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

async function publicText(path) {
  const res = await fetch(path, { cache: 'no-store' });
  const text = await res.text();
  if (!res.ok) throw new Error(text || `HTTP ${res.status}`);
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

async function loadFiles() {
  const content = document.getElementById('filesContent');
  const status = document.getElementById('filesStatus');
  content.innerHTML = '<p>Loading files…</p>';
  status.classList.add('hidden');
  const result = await api('/api/files');
  if (!result.ok) {
    content.innerHTML = `<p class="bad">${esc(result.error || 'Unable to list files')}</p>`;
    return;
  }
  const files = result.files || [];
  content.innerHTML = files.length ? `<div class="config-table-wrap"><table class="config-table file-table">
    <thead><tr><th>Name</th><th>Size</th><th>Actions</th></tr></thead>
    <tbody>${files.map(file => `<tr><td><strong>${esc(file.name)}</strong>${file.latest ? ' <em class="badge latest">Latest</em>' : ''}</td><td>${esc(fmtBytes(file.size))}</td><td>
      <button type="button" data-file-download="${esc(file.name)}">Download</button>
      <button type="button" class="danger-action" data-file-delete="${esc(file.name)}">Delete</button>
    </td></tr>`).join('')}</tbody></table></div>` : '<p>No stored files.</p>';
}

async function downloadStoredFile(name) {
  const status = document.getElementById('filesStatus');
  status.classList.remove('hidden');
  status.textContent = `Downloading ${name}…`;
  const path = `/api/files/download?name=${encodeURIComponent(name)}`;
  try {
    const response = await authFetch(path);
    if (!response.ok) throw new Error((await response.json()).error || `HTTP ${response.status}`);
    const url = URL.createObjectURL(await response.blob());
    const link = document.createElement('a');
    link.href = url;
    link.download = name;
    document.body.appendChild(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
    status.textContent = `Downloaded ${name}`;
  } catch (error) {
    status.textContent = `Download failed: ${error.message || error}`;
  }
}

async function deleteStoredFile(name) {
  if (window.prompt(`Type DELETE to permanently remove ${name}`) !== 'DELETE') return;
  const path = `/api/files?name=${encodeURIComponent(name)}`;
  const result = await api(path, { method: 'DELETE', headers: { 'X-Confirm': 'DELETE_FILE' } });
  const status = document.getElementById('filesStatus');
  status.classList.remove('hidden');
  status.textContent = result.ok ? `Deleted ${name}` : `Delete failed: ${result.error || 'unknown error'}`;
  if (result.ok) loadFiles();
}

function showActionStatus(value) {
  actionStatus.classList.remove('hidden');
  actionStatus.textContent = typeof value === 'string' ? value : JSON.stringify(value, null, 2);
}

function show(page, updateHistory = true) {
  const targetButton = navButtons.find(button => button.dataset.page === page);
  if (targetButton && targetButton.dataset.auth === 'required' && !isAuthenticated) return false;
  pages.forEach(p => p.classList.toggle('hidden', p.id !== page));
  navButtons.forEach(b => b.classList.toggle('active', b.dataset.page === page));
  const route = pageRoutes[page] || '/';
  if (updateHistory && location.pathname !== route) history.pushState({ page }, '', route);
  if (page === 'tests' && tests.length === 0) loadTests();
  if (page === 'health') loadHealth();
  if (page === 'config') loadConfig();
  if (page === 'files') loadFiles();
  if (page === 'info') loadInfo();
  if (page === 'live') {
    disconnectLogSocket(() => {
      if (!document.getElementById('live').classList.contains('hidden')) connectLiveSocket();
    });
    if (!liveSocket || liveSocket.readyState !== WebSocket.OPEN) loadLive();
  } else if (page === 'logs' || page === 'tests') {
    disconnectLiveSocket(() => {
      if (logStreamVisible()) connectLogSocket();
    });
  } else {
    disconnectLiveSocket();
    disconnectLogSocket();
  }
  if (page === 'ota') refreshOta();
  return true;
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
  const rows = checklist.items.map(item => {
    const warning = !item.ok && item.severity === 'warning';
    return `
    <div class="checklist-item ${item.ok ? 'ok' : (warning ? 'warning' : 'bad')}">
      <span>${item.ok ? 'OK' : (warning ? 'REVIEW' : 'BLOCKED')}</span>
      <div>
        <strong>${esc(item.label || item.key)}</strong>
        <small>${esc(item.message || '')}</small>
      </div>
    </div>`;
  }).join('');
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
  const hasHilMode = typeof hil.enabled === 'boolean';
  if (hasHilMode) {
    const simulation = hil.enabled === true;
    const modeLabel = simulation ? 'SIMULATION MODE' : 'FLIGHT MODE';
    hilMode.textContent = modeLabel;
    hilMode.className = `mode-pill ${simulation ? 'mode-simulation' : 'mode-flight'}`;
    simulationWarning.classList.toggle('hidden', !simulation);
  } else {
    hilMode.textContent = 'UNKNOWN';
    hilMode.className = 'mode-pill mode-unknown';
    simulationWarning.classList.add('hidden');
  }

  if (state === 'GROUND_SERVICES' && isAuthenticated) {
    fsmAdvance.textContent = 'Go To Ready For Launch';
    fsmAdvance.classList.remove('hidden');
    fsmAdvance.classList.add('danger-action');
    fsmAdvance.disabled = configDirty;
    fsmAdvance.title = configDirty ? 'Save configuration changes before READY FOR LAUNCH.' : '';
    readyConfigGuard.classList.toggle('hidden', !configDirty);
  } else {
    fsmAdvance.classList.add('hidden');
    fsmAdvance.classList.remove('danger-action');
    fsmAdvance.disabled = false;
    fsmAdvance.title = '';
    readyConfigGuard.classList.add('hidden');
  }
}

async function pollStatus() {
  if (otaUploadActive) return;
  if (pollBusy.status) return;
  pollBusy.status = true;
  let s;
  try {
    s = await publicApi('/api/status');
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
    publicApi('/api/status'),
    publicApi('/api/prelaunch/checklist')
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

function taskTable(tasks, includeHandle = false) {
  if (!tasks.length) return '<p class="muted">No task data available.</p>';
  const headers = includeHandle
    ? '<tr><th>Name</th><th>Handle</th><th>#</th><th>State</th><th>Priority</th><th>Base</th><th>Core</th><th>Stack HWM</th></tr>'
    : '<tr><th>Type</th><th>Task</th><th>Stack HWM</th></tr>';
  const rows = tasks.map(task => includeHandle
    ? `<tr><td>${esc(task.name || '')}</td><td>${esc(task.handle || '')}</td><td>${esc(String(task.number ?? ''))}</td><td>${esc(task.state || '')}</td><td>${esc(String(task.priority ?? ''))}</td><td>${esc(String(task.base_priority ?? ''))}</td><td>${esc(String(task.core ?? ''))}</td><td>${fmtBytes(task.stack_high_water_bytes)}</td></tr>`
    : `<tr><td>${esc(task.type || '')}</td><td>${esc(task.name || '')}</td><td>${fmtBytes(task.stack_high_water_bytes)}</td></tr>`).join('');
  return `<div class="config-table-wrap"><table class="config-table"><thead>${headers}</thead><tbody>${rows}</tbody></table></div>`;
}

async function captureTaskSnapshot() {
  taskSnapshot = await publicApi('/api/health/tasks');
  loadHealth();
}

async function loadHealth() {
  const h = await publicApi('/api/health');
  if (!h.ok) {
    document.getElementById('healthContent').innerHTML = panel('System Health', '<p class="bad">Unauthorized or unavailable.</p>');
    return;
  }
  const sensors = h.sensors || {};
  const memory = h.memory || {};
  const internal = memory.internal || {};
  const psram = memory.psram || {};
  const sd = (h.storage || {}).sd || {};
  const ground = h.ground_services || {};
  const http = ground.http || {};
  const websocket = ground.websocket || {};
  const broadcast = ground.broadcast || {};
  const httpd = ground.httpd || {};
  const lora = h.lora || {};
  const managedTasks = h.tasks || [];
  const snapshotBody = taskSnapshot
    ? (taskSnapshot.ok
      ? taskTable(taskSnapshot.tasks || [], true)
      : `<p class="bad">${esc(taskSnapshot.error || 'Task snapshot unavailable.')}</p>`)
    : '<p class="muted">On-demand only: captures all FreeRTOS tasks once.</p>';
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
      ${(sensors.barometer_ms5611_secondary || {}).status === 'not_present' ? '' : healthItem('MS5611 Barometer 2', sensors.barometer_ms5611_secondary, [row('Read status', (sensors.barometer_ms5611_secondary || {}).status)].join(''))}
      ${healthItem('LIS3DHTR Accelerometer', sensors.accelerometer_lis3dhtr, [row('Read status', (sensors.accelerometer_lis3dhtr || {}).status)].join(''))}
      ${healthItem('GPS', sensors.gps, [row('Read status', (sensors.gps || {}).status)].join(''))}
      ${healthItem('LoRa E220', lora, [
        row('Messages sent / failed', `${lora.tx_success || 0} / ${lora.tx_failures || 0}`),
        row('Last successful TX', lora.last_success_ms ? `${lora.last_success_ms} ms` : 'none yet')
      ].join(''))}
      ${healthItem('SD Card', sd, [row('Mounted', sd.present ? 'yes' : 'no')].join(''))}
    </div>
    ${panel('Resource Monitor', [
      row('HTTP clients', `${http.clients || 0} / ${http.capacity || 0}`),
      row('WebSockets', `${websocket.clients || 0} / ${websocket.capacity || 0} (${websocket.live_data || 0} live, ${websocket.logs || 0} logs)`),
      row('WS failures / slow drops', `${websocket.send_failures || 0} / ${websocket.slow_client_drops || 0}`),
      row('WS limit rejects', websocket.limit_rejects || 0),
      row('Broadcast queue failures / coalesced', `${broadcast.queue_failures || 0} / ${broadcast.coalesced || 0}`),
      row('Internal heap free / largest', `${fmtBytes(internal.free_bytes)} / ${fmtBytes(internal.largest_free_block_bytes)}`),
      row('Minimum internal heap', fmtBytes(internal.minimum_free_bytes)),
      row('HTTPD stack HWM', fmtBytes(httpd.stack_high_water_bytes)),
      row('SoftAP stations', ground.softap_stations || 0),
      row('OTA state', ground.ota_state || 'unknown'),
      '<h3>Managed task stack high-water marks</h3>',
      taskTable(managedTasks),
      '<div class="section-head"><h3>FreeRTOS task snapshot</h3><button id="captureTaskSnapshot">Capture snapshot</button></div>',
      snapshotBody
    ].join(''))}`;
  document.getElementById('captureTaskSnapshot').addEventListener('click', captureTaskSnapshot);
}

async function loadSdkconfig() {
  const box = document.getElementById('sdkconfigBox');

  try {
    sdkconfigLoading = true;
    if (box) box.textContent = 'Loading sdkconfig...';

    sdkconfigText = await publicText('/api/info/sdkconfig');

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

function logStreamVisible() {
  return ['logs', 'tests'].some(id => !document.getElementById(id).classList.contains('hidden'));
}

function currentLogOutput() {
  if (!document.getElementById('tests').classList.contains('hidden')) return testLogOutput;
  if (!document.getElementById('logs').classList.contains('hidden')) return logOutput;
  return null;
}

function renderLogs(output = logOutput, options = {}) {
  if (!output) return;
  if (output === testLogOutput && testLogsPaused && !options.force) return;
  const shouldStick = options.forceBottom || isNearBottom(output);
  const text = (output === testLogOutput ? testLogLines.join('\n') : filteredLogs(logLines.join('\n')));
  output.textContent = text || 'No log lines captured yet.';
  if (shouldStick) scrollToBottom(output);
}

async function loadLogs(options = {}) {
  const output = options.output || logOutput;
  if (logSocket && logSocket.readyState === WebSocket.OPEN) {
    renderLogs(output, { force: options.force, forceBottom: options.forceBottom });
    return;
  }
  if (pollBusy.logs && !options.force) return;
  pollBusy.logs = true;
  if (logsPaused && output === logOutput && !options.force) {
    pollBusy.logs = false;
    return;
  }
  try {
    const path = `/api/logs?since=${lastLogSeq}`;
    const res = await fetch(path, { cache: 'no-store' });
    const text = await res.text();
    if (!res.ok) throw new Error(text || `HTTP ${res.status}`);

    const latest = Number(res.headers.get('X-Log-Latest-Seq') || lastLogSeq);
    if (Number.isFinite(latest) && latest >= lastLogSeq) lastLogSeq = latest;

    const newLines = text.split('\n').filter(line => line.length > 0);
    if (newLines.length > 0) {
      logLines.push(...newLines);
      if (testLogVisible) testLogLines.push(...newLines);
      if (logLines.length > MAX_CLIENT_LOG_LINES) {
        logLines = logLines.slice(logLines.length - MAX_CLIENT_LOG_LINES);
      }
      if (testLogLines.length > MAX_CLIENT_LOG_LINES) testLogLines = testLogLines.slice(-MAX_CLIENT_LOG_LINES);
    }
    renderLogs(output, { force: options.force, forceBottom: options.forceBottom });
  } catch (err) {
    output.textContent = err.message || String(err);
  } finally {
    pollBusy.logs = false;
  }
}

function parseLogFrame(data) {
  const split = data.indexOf('\n');
  const latest = Number(data.slice(0, split));
  return split >= 0 && Number.isFinite(latest) ? { latest, text: data.slice(split + 1) } : null;
}

console.assert(parseLogFrame('42\nline\n').latest === 42, 'Log WebSocket frame parser check failed');

function connectLogSocket() {
  if (logSocket && logSocket.readyState <= WebSocket.OPEN) return;
  clearTimeout(logReconnectTimer);
  const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const socket = new WebSocket(`${protocol}//${location.host}/ws/logs?since=${lastLogSeq}`);
  logSocket = socket;
  socket.onopen = () => {
    if (socket !== logSocket) return;
    logReconnectDelayMs = 1500;
    const output = currentLogOutput();
    if (output && (output !== logOutput || !logsPaused)) renderLogs(output);
  };
  socket.onmessage = event => {
    if (socket !== logSocket) return;
    const frame = parseLogFrame(event.data);
    if (!frame || frame.latest <= lastLogSeq) return;
    lastLogSeq = frame.latest;
    const newLines = frame.text.split('\n').filter(line => line.length > 0);
    logLines.push(...newLines);
    if (testLogVisible) testLogLines.push(...newLines);
    if (logLines.length > MAX_CLIENT_LOG_LINES) logLines = logLines.slice(-MAX_CLIENT_LOG_LINES);
    if (testLogLines.length > MAX_CLIENT_LOG_LINES) testLogLines = testLogLines.slice(-MAX_CLIENT_LOG_LINES);
    const output = currentLogOutput();
    if (output && (output !== logOutput || !logsPaused)) renderLogs(output);
  };
  socket.onclose = () => {
    if (socket !== logSocket) return;
    logSocket = null;
    if (logStreamVisible()) {
      const delay = logReconnectDelayMs;
      logReconnectDelayMs = nextReconnectDelay(logReconnectDelayMs);
      logReconnectTimer = setTimeout(connectLogSocket, delay);
    }
  };
  socket.onerror = () => socket.close();
}

function disconnectLogSocket(onClosed) {
  clearTimeout(logReconnectTimer);
  logReconnectTimer = null;
  const socket = logSocket;
  logSocket = null;
  if (!socket) {
    if (onClosed) onClosed();
    return;
  }
  if (onClosed) socket.addEventListener('close', onClosed, { once: true });
  if (socket.readyState < WebSocket.CLOSING) socket.close();
  else if (socket.readyState === WebSocket.CLOSED && onClosed) onClosed();
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

function rotateByQuaternion(vector, quaternion) {
  let w = Number(quaternion.w);
  let x = Number(quaternion.x);
  let y = Number(quaternion.y);
  let z = Number(quaternion.z);
  const norm = Math.hypot(w, x, y, z) || 1;
  w /= norm; x /= norm; y /= norm; z /= norm;
  const [vx, vy, vz] = vector;
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return [
    vx + w * tx + (y * tz - z * ty),
    vy + w * ty + (z * tx - x * tz),
    vz + w * tz + (x * ty - y * tx)
  ];
}

function normalizedVector(vector) {
  const length = Math.hypot(...vector) || 1;
  return vector.map(value => value / length);
}

function crossProduct(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

const ATTITUDE_CAMERA_FORWARD = normalizedVector([2, 3, 1.5]);
const ATTITUDE_CAMERA_RIGHT = normalizedVector(crossProduct([0, 0, 1], ATTITUDE_CAMERA_FORWARD));
const ATTITUDE_CAMERA_UP = crossProduct(ATTITUDE_CAMERA_FORWARD, ATTITUDE_CAMERA_RIGHT);

function attitudeMountingDeterminant(mapping) {
  if (!mapping || !['x', 'y', 'z'].every(axis => /^[+-][xyz]$/.test(mapping[axis]))) return 0;
  const matrix = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  ['x', 'y', 'z'].forEach((bodyAxis, column) => {
    const mapped = mapping[bodyAxis];
    matrix['xyz'.indexOf(mapped[1])][column] = mapped[0] === '-' ? -1 : 1;
  });
  return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
    matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
    matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

function validAttitudeMounting(mapping) {
  return attitudeMountingDeterminant(mapping) === 1;
}

function loadAttitudeMounting() {
  try {
    const stored = JSON.parse(localStorage.getItem('atlas_attitude_mounting'));
    return validAttitudeMounting(stored) ? stored : { ...DEFAULT_ATTITUDE_MOUNTING };
  } catch (_) {
    return { ...DEFAULT_ATTITUDE_MOUNTING };
  }
}

function bodyToSensor(vector, mapping = attitudeMounting) {
  const sensor = [0, 0, 0];
  ['x', 'y', 'z'].forEach((bodyAxis, index) => {
    const mapped = mapping[bodyAxis];
    sensor['xyz'.indexOf(mapped[1])] = (mapped[0] === '-' ? -1 : 1) * vector[index];
  });
  return sensor;
}

function attitudeGeometry(quaternion) {
  const cameraForward = ATTITUDE_CAMERA_FORWARD;
  const cameraRight = ATTITUDE_CAMERA_RIGHT;
  const cameraUp = ATTITUDE_CAMERA_UP;
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const project = (value, origin = [230, 165], scale = 92) => {
    const depth = dot(value, cameraForward);
    const perspective = 3.6 / (3.6 - depth);
    return [origin[0] + scale * dot(value, cameraRight) * perspective, origin[1] - scale * dot(value, cameraUp) * perspective];
  };
  const projectXZ = ([x, , z]) => [382 + 31 * x, 76 - 31 * z];
  const projectYZ = ([, y, z]) => [382 + 31 * y, 202 - 31 * z];
  const point = value => value.map(n => n.toFixed(1)).join(',');
  const rotated = value => rotateByQuaternion(bodyToSensor(value), quaternion);
  const bodyAxes = [
    ['X', '#ef4444', rotated([1, 0, 0])],
    ['Y', '#22c55e', rotated([0, 1, 0])],
    ['Z / NOSE', '#3b82f6', rotated([0, 0, 1.25])]
  ];
  const center = project([0, 0, 0]);
  const inertialAcceleration = rotateByQuaternion(displayedAttitudeAcceleration, quaternion);
  const accelerationEnd = project(inertialAcceleration.map(value => value / 9.80665));
  const accelerationMagnitude = Math.hypot(...displayedAttitudeAcceleration) / 9.80665;
  const segments = 8;
  const ring = z => Array.from({ length: segments }, (_, index) => {
    const angle = index * Math.PI * 2 / segments;
    return rotated([0.22 * Math.cos(angle), 0.22 * Math.sin(angle), z]);
  });
  const bottom = ring(-0.9), shoulder = ring(0.62), nose = rotated([0, 0, 1.35]);
  const mesh = [];
  const addFace = (kind, vertices) => mesh.push({ kind, vertices });
  for (let index = 0; index < segments; index++) {
    const next = (index + 1) % segments;
    addFace(`rocket-body rocket-face-${index % 2}`, [bottom[index], bottom[next], shoulder[next], shoulder[index]]);
    addFace(`rocket-cone rocket-face-${index % 2}`, [shoulder[index], shoulder[next], nose]);
  }
  addFace('rocket-base', bottom);
  for (let index = 0; index < 3; index++) {
    const angle = index * Math.PI * 2 / 3;
    const radial = distance => [distance * Math.cos(angle), distance * Math.sin(angle)];
    const root = radial(0.2), tip = radial(0.52);
    addFace(`rocket-fin rocket-fin-${index}`, [
      rotated([root[0], root[1], -0.88]), rotated([tip[0], tip[1], -1.05]),
      rotated([tip[0], tip[1], -0.48]), rotated([root[0], root[1], -0.38])
    ]);
  }
  const renderFaces = (projector, depthVector) => mesh.map(face => ({
    kind: face.kind,
    depth: face.vertices.reduce((sum, value) => sum + dot(value, depthVector), 0) / face.vertices.length,
    points: face.vertices.map(value => point(projector(value))).join(' ')
  })).sort((a, b) => a.depth - b.depth);

  return {
    project, bodyAxes, center, accelerationEnd, accelerationMagnitude,
    faces: renderFaces(project, cameraForward),
    xzFaces: renderFaces(projectXZ, [0, 1, 0]),
    yzFaces: renderFaces(projectYZ, [1, 0, 0])
  };
}

function rocketFacesSvg(faces) {
  return faces.map(face => `<polygon points="${face.points}" class="${face.kind}"/>`).join('');
}

function attitudeAlignmentControls() {
  const options = ['+x', '-x', '+y', '-y', '+z', '-z'];
  return `<div class="attitude-alignment"><strong>Board mounting</strong><small>Map rocket body axes to sensor axes</small><div>${['x', 'y', 'z'].map(axis =>
    `<label>Body ${axis.toUpperCase()}<select class="attitude-axis-map" data-body-axis="${axis}">${options.map(value =>
      `<option value="${value}"${attitudeMountingDraft[axis] === value ? ' selected' : ''}>Sensor ${value.toUpperCase()}</option>`).join('')}</select></label>`
  ).join('')}</div><button type="button" id="applyAttitudeAlignment">Apply alignment</button><span id="attitudeAlignmentStatus">Changes are staged until applied.</span></div>`;
}

function attitudeSvg() {
  const { project, bodyAxes, center, faces, xzFaces, yzFaces, accelerationEnd, accelerationMagnitude } = attitudeGeometry(displayedAttitudeQuaternion);
  const worldAxes = [
    ['+X', '#ef4444', [1, 0, 0]], ['+Y', '#22c55e', [0, 1, 0]], ['+Z', '#3b82f6', [0, 0, 1]]
  ];
  const worldOrigin = [62, 260];

  return `<div class="attitude-view">
    <svg viewBox="0 0 460 320" role="img" aria-label="Live rocket attitude with world and body reference axes">
      <defs>
        <filter id="rocketShadow"><feDropShadow dx="0" dy="5" stdDeviation="6" flood-opacity=".24"/></filter>
        <marker id="accelerationArrow" markerWidth="8" markerHeight="8" refX="6" refY="3" orient="auto"><path d="M0,0 L0,6 L7,3 z"/></marker>
      </defs>
      <g>
        <rect width="460" height="165" class="horizon-sky"/>
        <rect y="165" width="460" height="155" class="horizon-earth"/>
        <line x1="0" y1="165" x2="460" y2="165" class="horizon-line"/>
        <line x1="205" y1="125" x2="255" y2="125" class="horizon-mark"/>
        <line x1="190" y1="205" x2="270" y2="205" class="horizon-mark"/>
      </g>
      <circle cx="${center[0]}" cy="${center[1]}" r="108" class="attitude-orbit"/>
      <g id="attitudeRocket" filter="url(#rocketShadow)">${rocketFacesSvg(faces)}</g>
      <line id="attitudeAcceleration" x1="${center[0]}" y1="${center[1]}" x2="${accelerationEnd[0]}" y2="${accelerationEnd[1]}" class="acceleration-vector" marker-end="url(#accelerationArrow)"/>
      <text id="attitudeAccelerationLabel" x="${accelerationEnd[0] + 7}" y="${accelerationEnd[1] - 7}" class="acceleration-label">${accelerationMagnitude.toFixed(2)} g</text>
      ${bodyAxes.map(([label, color, value], index) => {
        const end = project(value);
        return `<line id="bodyAxis${index}" x1="${center[0]}" y1="${center[1]}" x2="${end[0]}" y2="${end[1]}" stroke="${color}" class="body-axis"/><text id="bodyLabel${index}" x="${end[0] + 5}" y="${end[1] - 5}" fill="${color}" class="axis-label">B${label}</text>`;
      }).join('')}
      ${worldAxes.map(([label, color, value]) => {
        const projected = project(value, worldOrigin, 42);
        return `<line x1="${worldOrigin[0]}" y1="${worldOrigin[1]}" x2="${projected[0]}" y2="${projected[1]}" stroke="${color}" class="world-axis"/><text x="${projected[0] + 4}" y="${projected[1] - 3}" fill="${color}" class="axis-label">I${label}</text>`;
      }).join('')}
      <circle cx="${worldOrigin[0]}" cy="${worldOrigin[1]}" r="3" class="axis-origin"/>
      <g class="orthographic-view"><rect x="338" y="18" width="88" height="116"/><text x="346" y="33">X–Z</text><line x1="346" y1="76" x2="418" y2="76"/><line x1="382" y1="38" x2="382" y2="122"/><g id="attitudeRocketXZ">${rocketFacesSvg(xzFaces)}</g></g>
      <g class="orthographic-view"><rect x="338" y="144" width="88" height="116"/><text x="346" y="159">Y–Z</text><line x1="346" y1="202" x2="418" y2="202"/><line x1="382" y1="164" x2="382" y2="248"/><g id="attitudeRocketYZ">${rocketFacesSvg(yzFaces)}</g></g>
    </svg>
    <div class="attitude-legend"><span><i class="axis-x"></i>Body X</span><span><i class="axis-y"></i>Body Y</span><span><i class="axis-z"></i>Body Z / nose</span></div>
  </div>`;
}

function animateAttitude() {
  let target = targetAttitudeQuaternion;
  const current = displayedAttitudeQuaternion;
  if (current.w * target.w + current.x * target.x + current.y * target.y + current.z * target.z < 0) {
    target = { w: -target.w, x: -target.x, y: -target.y, z: -target.z };
  }
  const blend = 0.16;
  const next = {
    w: current.w + (target.w - current.w) * blend,
    x: current.x + (target.x - current.x) * blend,
    y: current.y + (target.y - current.y) * blend,
    z: current.z + (target.z - current.z) * blend
  };
  const norm = Math.hypot(next.w, next.x, next.y, next.z) || 1;
  displayedAttitudeQuaternion = { w: next.w / norm, x: next.x / norm, y: next.y / norm, z: next.z / norm };
  displayedAttitudeAcceleration = displayedAttitudeAcceleration.map((value, index) => value + (targetAttitudeAcceleration[index] - value) * blend);

  const rocket = document.getElementById('attitudeRocket');
  if (rocket) {
    const { project, bodyAxes, faces, xzFaces, yzFaces, accelerationEnd, accelerationMagnitude } = attitudeGeometry(displayedAttitudeQuaternion);
    rocket.innerHTML = rocketFacesSvg(faces);
    document.getElementById('attitudeRocketXZ').innerHTML = rocketFacesSvg(xzFaces);
    document.getElementById('attitudeRocketYZ').innerHTML = rocketFacesSvg(yzFaces);
    const acceleration = document.getElementById('attitudeAcceleration');
    const accelerationLabel = document.getElementById('attitudeAccelerationLabel');
    acceleration.setAttribute('x2', accelerationEnd[0]); acceleration.setAttribute('y2', accelerationEnd[1]);
    accelerationLabel.setAttribute('x', accelerationEnd[0] + 7); accelerationLabel.setAttribute('y', accelerationEnd[1] - 7);
    accelerationLabel.textContent = `${accelerationMagnitude.toFixed(2)} g`;
    bodyAxes.forEach(([, , value], index) => {
      const end = project(value);
      const axis = document.getElementById(`bodyAxis${index}`);
      const label = document.getElementById(`bodyLabel${index}`);
      if (axis) { axis.setAttribute('x2', end[0]); axis.setAttribute('y2', end[1]); }
      if (label) { label.setAttribute('x', end[0] + 5); label.setAttribute('y', end[1] - 5); }
    });
  }
  requestAnimationFrame(animateAttitude);
}

function addLiveHistorySample(values) {
  liveHistory.push({ time: Date.now(), ...values });
  if (liveHistory.length > LIVE_HISTORY_SAMPLES) liveHistory.shift();
}

function trendChart(title, unit, series, decimals = 2) {
  const width = 320, height = 112, left = 8, right = 8, top = 8, bottom = 10;
  const allValues = series.flatMap(item => liveHistory.map(sample => Number(sample[item.key])).filter(Number.isFinite));
  if (allValues.length === 0) return '';
  let min = Math.min(...allValues), max = Math.max(...allValues);
  // Pad relative to the span actually observed, never to the absolute value:
  // a 1%-of-magnitude floor would swamp a few Pa of change on a ~101325 Pa
  // baseline and flatten the pressure trace. Only a completely flat series
  // falls back to a magnitude-derived epsilon so it still has a visible band.
  const span = max - min;
  const padding = span > 0 ? span * 0.12 : Math.max(Math.abs(max) * 1e-4, 0.01);
  min -= padding; max += padding;
  const x = index => left + index * (width - left - right) / Math.max(liveHistory.length - 1, 1);
  const y = value => top + (max - value) * (height - top - bottom) / (max - min);
  const paths = series.map(item => {
    const points = liveHistory.map((sample, index) => [x(index), Number(sample[item.key])]).filter(([, value]) => Number.isFinite(value));
    if (!points.length) return '';
    const path = points.map(([px, value], index) => `${index ? 'L' : 'M'}${px.toFixed(1)},${y(value).toFixed(1)}`).join(' ');
    const [lastX, lastValue] = points[points.length - 1];
    return `<path d="${path}" stroke="${item.color}"/><circle cx="${lastX.toFixed(1)}" cy="${y(lastValue).toFixed(1)}" r="3.5" fill="${item.color}"/>`;
  }).join('');
  const latest = liveHistory[liveHistory.length - 1];
  const legend = series.map(item => {
    const value = Number(latest[item.key]);
    return `<span style="color:${item.color}">${esc(item.label)} <strong>${Number.isFinite(value) ? value.toFixed(decimals) : '—'}</strong></span>`;
  }).join('');
  return `<section class="trend-card"><div class="trend-head"><strong>${esc(title)}</strong><small>${esc(unit)}</small></div><div class="trend-legend">${legend}</div><svg viewBox="0 0 ${width} ${height}" preserveAspectRatio="none"><line x1="${left}" y1="${height / 2}" x2="${width - right}" y2="${height / 2}" class="trend-grid"/>${paths}</svg></section>`;
}

const attitudeRotationCheck = rotateByQuaternion([1, 0, 0], { w: Math.SQRT1_2, x: 0, y: 0, z: Math.SQRT1_2 });
console.assert(Math.abs(attitudeRotationCheck[0]) < 1e-6 && Math.abs(attitudeRotationCheck[1] - 1) < 1e-6, 'Quaternion attitude rotation check failed');
const cameraHandednessCheck = crossProduct(ATTITUDE_CAMERA_RIGHT, ATTITUDE_CAMERA_UP);
console.assert(cameraHandednessCheck.every((value, index) => Math.abs(value - ATTITUDE_CAMERA_FORWARD[index]) < 1e-6), 'Camera right-hand-rule check failed');
console.assert(validAttitudeMounting(DEFAULT_ATTITUDE_MOUNTING), 'Default PCB attitude mapping check failed');
console.assert(!validAttitudeMounting({ x: '+x', y: '+y', z: '-z' }), 'Left-handed attitude mapping check failed');
console.assert(bodyToSensor([1, 2, 3], DEFAULT_ATTITUDE_MOUNTING).join(',') === '-1,-2,3', 'Body-to-sensor axis mapping check failed');
requestAnimationFrame(animateAttitude);

function renderLive(s) {
  if (!s.ok) {
    document.getElementById('liveContent').innerHTML = panel('Live Data', `<p class="bad">Live data unavailable.</p>`);
    return;
  }
  updateFsmBar({ ...(latestStatus || {}), fsm_state: s.fsm_state || 'UNKNOWN' });
  const flight = s.flight || {};
  const calibration = s.calibration || {};
  const sensors = s.sensors || {};
  const imu = sensors.imu || {};
  const accelerometer = sensors.accelerometer || {};
  const barometer = sensors.barometer || {};
  const gps = sensors.gps || {};
  const imuCalibration = imu.calibration || {};
  const orientation = imu.orientation_deg || {};
  const angularVelocity = imu.angular_velocity_rad_s || {};
  const acceleration = imu.acceleration_m_s2 || {};
  const linearAcceleration = imu.linear_acceleration_m_s2 || {};
  const gravity = imu.gravity_m_s2 || {};
  const magnetometer = imu.magnetometer_ut || {};
  const lora = ((s.telemetry || {}).lora || {});
  const loraTxPulse = performance.now() < loRaTxPulseUntil;
  addLiveHistorySample({
    attitudeX: Number(orientation.x), attitudeY: Number(orientation.y), attitudeZ: Number(orientation.z),
    imuAx: Number(acceleration.x), imuAy: Number(acceleration.y), imuAz: Number(acceleration.z),
    lisAx: Number(accelerometer.x), lisAy: Number(accelerometer.y), lisAz: Number(accelerometer.z),
    gyroX: Number(angularVelocity.x), gyroY: Number(angularVelocity.y), gyroZ: Number(angularVelocity.z),
    pressure: Number(barometer.pressure), gpsAltitude: Number(gps.alt)
  });
  if (document.activeElement && document.activeElement.closest('.attitude-alignment')) return;
  document.getElementById('liveContent').innerHTML = `
    <div class="metrics">
      <div><span>Height</span><strong>${Number(flight.height_m || 0).toFixed(2)} m</strong></div>
      <div><span>Vertical speed</span><strong>${Number(flight.vertical_speed_mps || 0).toFixed(2)} m/s</strong></div>
      <div><span>Rising</span><strong>${flight.is_rising ? 'yes' : 'no'}</strong></div>
      <div><span>Calibrated</span><strong>${calibration.imu ? 'yes' : 'no'}</strong></div>
      <div class="${loraTxPulse ? 'tx-pulse' : ''}"><span>LoRa</span><strong>${lora.available ? (loraTxPulse ? 'TX pulse' : 'ready') : 'unavailable'}</strong></div>
    </div>
    <div class="trend-grid-layout">
      ${trendChart('Attitude', 'deg', [{ key: 'attitudeX', label: 'X', color: '#ef4444' }, { key: 'attitudeY', label: 'Y', color: '#22c55e' }, { key: 'attitudeZ', label: 'Z', color: '#3b82f6' }], 1)}
      ${trendChart('BNO055 acceleration', 'm/s2', [{ key: 'imuAx', label: 'X', color: '#ef4444' }, { key: 'imuAy', label: 'Y', color: '#22c55e' }, { key: 'imuAz', label: 'Z', color: '#3b82f6' }], 2)}
      ${trendChart('LIS3DHTR acceleration', 'm/s2', [{ key: 'lisAx', label: 'X', color: '#ef4444' }, { key: 'lisAy', label: 'Y', color: '#22c55e' }, { key: 'lisAz', label: 'Z', color: '#3b82f6' }], 2)}
      ${trendChart('Angular velocity', 'rad/s', [{ key: 'gyroX', label: 'X', color: '#ef4444' }, { key: 'gyroY', label: 'Y', color: '#22c55e' }, { key: 'gyroZ', label: 'Z', color: '#3b82f6' }], 2)}
      ${trendChart('Barometer pressure', 'Pa', [{ key: 'pressure', label: 'P', color: '#a855f7' }], 1)}
      ${trendChart('GPS altitude', 'm ASL', [{ key: 'gpsAltitude', label: 'Altitude', color: '#f59e0b' }], 1)}
    </div>
    <div class="attitude-grid">
      ${panel('Rocket Attitude', attitudeSvg())}
      ${panel('Attitude & Calibration', [
        row('Euler X / heading', `${Number(orientation.x || 0).toFixed(2)} deg`),
        row('Euler Y / roll', `${Number(orientation.y || 0).toFixed(2)} deg`),
        row('Euler Z / pitch', `${Number(orientation.z || 0).toFixed(2)} deg`),
        row('Calibration SYS / GYR / ACC / MAG', `${imuCalibration.system || 0} / ${imuCalibration.gyro || 0} / ${imuCalibration.accelerometer || 0} / ${imuCalibration.magnetometer || 0}`),
        row('Angular velocity', `${Number(angularVelocity.x || 0).toFixed(3)}, ${Number(angularVelocity.y || 0).toFixed(3)}, ${Number(angularVelocity.z || 0).toFixed(3)} rad/s`),
        attitudeAlignmentControls()
      ].join(''))}
    </div>
    <div class="trend-grid-layout">
      ${trendChart('Attitude', 'deg', [{ key: 'attitudeX', label: 'X', color: '#ef4444' }, { key: 'attitudeY', label: 'Y', color: '#22c55e' }, { key: 'attitudeZ', label: 'Z', color: '#3b82f6' }], 1)}
      ${trendChart('BNO055 acceleration', 'm/s2', [{ key: 'imuAx', label: 'X', color: '#ef4444' }, { key: 'imuAy', label: 'Y', color: '#22c55e' }, { key: 'imuAz', label: 'Z', color: '#3b82f6' }], 2)}
      ${trendChart('LIS3DHTR acceleration', 'm/s2', [{ key: 'lisAx', label: 'X', color: '#ef4444' }, { key: 'lisAy', label: 'Y', color: '#22c55e' }, { key: 'lisAz', label: 'Z', color: '#3b82f6' }], 2)}
      ${trendChart('Angular velocity', 'rad/s', [{ key: 'gyroX', label: 'X', color: '#ef4444' }, { key: 'gyroY', label: 'Y', color: '#22c55e' }, { key: 'gyroZ', label: 'Z', color: '#3b82f6' }], 2)}
      ${trendChart('Barometer pressure', 'Pa', [{ key: 'pressure', label: 'P', color: '#a855f7' }], 1)}
      ${trendChart('GPS altitude', 'm ASL', [{ key: 'gpsAltitude', label: 'Altitude', color: '#f59e0b' }], 1)}
    </div>
    <div class="attitude-grid">
      ${panel('Rocket Attitude', attitudeSvg())}
      ${panel('Attitude & Calibration', [
        row('Euler X / heading', `${Number(orientation.x || 0).toFixed(2)} deg`),
        row('Euler Y / roll', `${Number(orientation.y || 0).toFixed(2)} deg`),
        row('Euler Z / pitch', `${Number(orientation.z || 0).toFixed(2)} deg`),
        row('Calibration SYS / GYR / ACC / MAG', `${imuCalibration.system || 0} / ${imuCalibration.gyro || 0} / ${imuCalibration.accelerometer || 0} / ${imuCalibration.magnetometer || 0}`),
        row('Angular velocity', `${Number(angularVelocity.x || 0).toFixed(3)}, ${Number(angularVelocity.y || 0).toFixed(3)}, ${Number(angularVelocity.z || 0).toFixed(3)} rad/s`),
        attitudeAlignmentControls()
      ].join(''))}
    </div>
    <div class="sensor-grid">
      ${sensorCard('IMU', imu, [
        ['Acceleration', `${Number(acceleration.x || 0).toFixed(3)}, ${Number(acceleration.y || 0).toFixed(3)}, ${Number(acceleration.z || 0).toFixed(3)} m/s2`],
        ['Linear acceleration', `${Number(linearAcceleration.x || 0).toFixed(3)}, ${Number(linearAcceleration.y || 0).toFixed(3)}, ${Number(linearAcceleration.z || 0).toFixed(3)} m/s2`],
        ['Gravity', `${Number(gravity.x || 0).toFixed(3)}, ${Number(gravity.y || 0).toFixed(3)}, ${Number(gravity.z || 0).toFixed(3)} m/s2`],
        ['Magnetometer', `${Number(magnetometer.x || 0).toFixed(2)}, ${Number(magnetometer.y || 0).toFixed(2)}, ${Number(magnetometer.z || 0).toFixed(2)} uT`],
        ['Temperature', `${Number(imu.temperature_c || 0).toFixed(1)} C`]
      ])}
      ${sensorCard('LIS3DHTR Accelerometer', accelerometer, [
        ['Acceleration X', `${Number(accelerometer.x || 0).toFixed(4)} m/s2`],
        ['Acceleration Y', `${Number(accelerometer.y || 0).toFixed(4)} m/s2`],
        ['Acceleration Z', `${Number(accelerometer.z || 0).toFixed(4)} m/s2`]
      ])}
      ${sensorCard('Barometer', barometer, [
        ['Pressure', `${Number(barometer.pressure || 0).toFixed(2)} Pa`],
        ['Temperature', `${Number(barometer.temperature_c || 0).toFixed(1)} C`],
        ['Zeroed', calibration.barometer ? 'yes' : 'no'],
        ['Samples', calibration.barometer_samples || 0]
      ])}
      ${sensorCard('GPS', gps, [
        ['Fix', gps.fix ? 'yes' : 'no'],
        ['Satellites', gps.satellites || 0],
        ['Latitude', Number(gps.lat || 0).toFixed(7)],
        ['Longitude', Number(gps.lon || 0).toFixed(7)],
        ['Altitude', `${Number(gps.alt || 0).toFixed(2)} m`],
        ['Ground speed', `${Number(gps.ground_speed_mps || 0).toFixed(2)} m/s`],
        ['HDOP', Number(gps.hdop || 0).toFixed(2)]
      ])}
    </div>`;
}

function ingestLive(s) {
  if (s && s.ok) {
    const imu = ((s.sensors || {}).imu || {});
    const acceleration = imu.acceleration_m_s2 || {};
    targetAttitudeQuaternion = imu.quaternion || targetAttitudeQuaternion;
    targetAttitudeAcceleration = [Number(acceleration.x) || 0, Number(acceleration.y) || 0, Number(acceleration.z) || 0];
    const loraSuccess = Number((((s.telemetry || {}).lora || {}).tx_success) || 0);
    if (loraSuccess > lastLoRaTxSuccess) loRaTxPulseUntil = performance.now() + 400;
    lastLoRaTxSuccess = loraSuccess;
  }
  if (!document.getElementById('live').classList.contains('hidden')) renderLive(s);
}

async function loadLive() {
  try {
    const response = await fetch('/api/live-data', { cache: 'no-store' });
    ingestLive(await response.json());
  } catch (_) {
    ingestLive({ ok: false });
  }
}

function connectLiveSocket() {
  if (liveSocket && liveSocket.readyState <= WebSocket.OPEN) return;
  clearTimeout(liveReconnectTimer);
  const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const socket = new WebSocket(`${protocol}//${location.host}/ws/live-data`);
  liveSocket = socket;
  socket.onopen = () => {
    if (socket === liveSocket) liveReconnectDelayMs = 1500;
  };
  socket.onmessage = event => {
    if (socket !== liveSocket || document.getElementById('live').classList.contains('hidden')) return;
    try { ingestLive(JSON.parse(event.data)); } catch (_) {}
  };
  socket.onclose = () => {
    if (socket !== liveSocket) return;
    liveSocket = null;
    if (!document.getElementById('live').classList.contains('hidden')) {
      const delay = liveReconnectDelayMs;
      liveReconnectDelayMs = nextReconnectDelay(liveReconnectDelayMs);
      liveReconnectTimer = setTimeout(connectLiveSocket, delay);
    }
  };
  socket.onerror = () => socket.close();
}

function disconnectLiveSocket(onClosed) {
  clearTimeout(liveReconnectTimer);
  liveReconnectTimer = null;
  const socket = liveSocket;
  liveSocket = null;
  if (!socket) {
    if (onClosed) onClosed();
    return;
  }
  if (onClosed) socket.addEventListener('close', onClosed, { once: true });
  if (socket.readyState < WebSocket.CLOSING) socket.close();
  else if (socket.readyState === WebSocket.CLOSED && onClosed) onClosed();
}

async function refreshOta() {
  const s = await api('/api/ota/status');
  progress.value = s.progress || 0;
  if (s.state === 'ready_to_reboot') {
    otaStatus.textContent = `Firmware validated — ready to reboot\n${JSON.stringify(s, null, 2)}`;
  } else if (s.state === 'failed' || s.ok === false) {
    otaStatus.textContent = `OTA failed\n${JSON.stringify(s, null, 2)}`;
  } else {
    otaStatus.textContent = JSON.stringify(s, null, 2);
  }
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
    <span class="config-desc">${esc(meta.description || '')}${meta.source ? ` · Source: ${esc(meta.source)}` : ''}</span>
    ${issue ? `<span class="config-message">${esc(issue.message)}</span>` : ''}
  </label>`;
}

function configReadOnlyTable(fields) {
  return `<div class="config-table-wrap"><table class="config-table">
    <thead><tr><th>Parameter</th><th>Value</th><th>Source</th></tr></thead>
    <tbody>${fields.map(field => {
      const value = runtimeConfig[field.key];
      const display = `${value === undefined ? '—' : value}${field.unit ? ` ${field.unit}` : ''}`;
      return `<tr><td><strong>${esc(field.label || field.key)}</strong><small>${esc(field.description || '')}</small></td><td>${esc(display)}</td><td>${esc(field.source || 'unknown')}</td></tr>`;
    }).join('')}</tbody>
  </table></div>`;
}

function configSignature(values) {
  const editable = (runtimeSchema.fields || []).filter(field => field.editable);
  return JSON.stringify(Object.fromEntries(editable.map(field => [field.key, values[field.key]])));
}

function editedConfigValues() {
  const values = { ...runtimeConfig };
  document.querySelectorAll('[data-config]').forEach(input => {
    if (input.disabled) return;
    const key = input.dataset.config;
    values[key] = input.type === 'checkbox' ? input.checked
      : input.type === 'number' ? Number(input.value) : input.value;
  });
  return values;
}

function setConfigSaveState(message = '', state = '') {
  const button = document.getElementById('saveConfig');
  const feedback = document.getElementById('configSaveState');
  button.classList.toggle('config-dirty', configDirty);
  button.textContent = configDirty ? 'Save Changes' : 'Save';
  feedback.textContent = message;
  feedback.className = `config-save-state ${state}`;
}

function updateConfigDirtyState() {
  if (!persistedConfigSignature) return;
  configDirty = configSignature(editedConfigValues()) !== persistedConfigSignature;
  setConfigSaveState(configDirty ? 'Unsaved changes' : '');
  if (latestStatus) updateFsmBar(latestStatus);
}
async function loadConfig() {
  document.getElementById('configStatus').classList.remove('config-save-success');
  const [cfg, schema, validation] = await Promise.all([
    api('/api/config/runtime'),
    api('/api/config/schema'),
    api('/api/config/validation')
  ]);
  runtimeConfig = cfg;
  runtimeSchema = schema.ok ? schema : { fields: [] };
  runtimeValidation = validation.ok === false || validation.ok === true ? validation : { items: [] };
  const editableFields = (runtimeSchema.fields || []).filter(field => field.editable);
  const editableGroups = editableFields.reduce((acc, field) => {
    if (!acc[field.group]) acc[field.group] = [];
    acc[field.group].push(field);
    return acc;
  }, {});
  const readOnlyGroups = (runtimeSchema.fields || []).filter(field => !field.editable).reduce((acc, field) => {
    if (!acc[field.group]) acc[field.group] = [];
    acc[field.group].push(field);
    return acc;
  }, {});
  document.getElementById('configEditor').innerHTML = `
    ${Object.entries(editableGroups).map(([group, fields]) => `<section class="config-group"><h3>${esc(group)}</h3><p class="config-intro">Editable before the flight lock.</p><div class="config-grid">
      ${fields.map(field => configInput(field.key, runtimeConfig[field.key])).join('')}
    </div></section>`).join('')}
    <section class="config-group"><h3>Configuration inspection</h3><p class="config-intro">Compiled, hardware, and algorithm values are read-only. Expand a table only when needed.</p>
      ${Object.entries(readOnlyGroups).map(([group, fields]) => `<details class="config-inspection"><summary>${esc(group)} <span>${fields.length}</span></summary>${configReadOnlyTable(fields)}</details>`).join('')}
    </section>`;
  const locked = runtimeConfig.config_locked ? 'LOCKED FOR FLIGHT' : 'Editable in Ground Services';
  const valid = runtimeValidation.ok ? 'valid' : 'invalid';
  document.getElementById('configStatus').textContent = `${locked} - ${valid} - schema v${runtimeConfig.schema_version}, revision ${runtimeConfig.config_revision}\n${JSON.stringify(runtimeValidation, null, 2)}`;
  document.getElementById('unlockConfig').classList.toggle('hidden', runtimeConfig.config_locked !== true);
  persistedConfigSignature = configSignature(runtimeConfig);
  configDirty = false;
  setConfigSaveState();
  if (latestStatus) updateFsmBar(latestStatus);
}

async function saveConfig() {
  const inputs = [...document.querySelectorAll('[data-config]')];
  const invalid = inputs.find(input => !input.disabled && !input.checkValidity());
  if (invalid) {
    invalid.reportValidity();
    return;
  }
  const updated = editedConfigValues();
  const s = await api('/api/config/runtime', {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(updated)
  });
  const status = document.getElementById('configStatus');
  if (s.ok) {
    await loadConfig();
    status.textContent = `✓ Configuration saved to NVS.\n${status.textContent}`;
    status.classList.add('config-save-success');
    setConfigSaveState('Saved ✓', 'success');
  } else {
    status.textContent = JSON.stringify(s, null, 2);
    setConfigSaveState('Save failed — changes not saved', 'error');
  }
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
  if (closeTestLogWhenFinished && !s.running && !s.waiting_for_verdict) {
    closeTestLogWhenFinished = false;
    testLogVisible = false;
    testLogsPaused = false;
    pauseTestLogsButton.textContent = 'Pause';
  }
  if (s.running || s.waiting_for_verdict) testLogVisible = true;
  const testLogPanel = document.getElementById('testLogPanel');
  const showTestLogs = testLogVisible || s.running || s.waiting_for_verdict;
  const testLogWasHidden = testLogPanel.classList.contains('hidden');
  testLogPanel.classList.toggle('hidden', !showTestLogs);
  if (showTestLogs) loadLogs({ output: testLogOutput, forceBottom: testLogWasHidden });
}

async function startTest(event) {
  const button = event.currentTarget;
  const id = Number(button.dataset.test);
  const confirm = button.dataset.confirm;
  if (confirm && window.prompt(`Type ${confirm} to run this test`) !== confirm) return;
  testLogLines = [];
  testLogVisible = true;
  testLogsPaused = false;
  closeTestLogWhenFinished = false;
  pauseTestLogsButton.textContent = 'Pause';
  const testConsole = document.querySelector('.test-console');
  document.getElementById('testLogPanel').classList.remove('hidden');
  renderLogs(testLogOutput, { force: true, forceBottom: true });
  testConsole.scrollIntoView({ behavior: 'smooth', block: 'start' });
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
  if (s.ok && verdict === 'passed') closeTestLogWhenFinished = true;
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
    if (!event.lengthComputable) return;
    const percent = Math.round((event.loaded / event.total) * 100);
    progress.value = percent;
    otaStatus.textContent = percent < 100
      ? `Uploading firmware... ${percent}%`
      : 'Upload complete — validating firmware...';
  };
  xhr.upload.onload = () => { otaStatus.textContent = 'Upload complete — validating firmware...'; };
  xhr.onload = () => {
    otaUploadActive = false;
    try {
      const result = JSON.parse(xhr.responseText);
      otaStatus.textContent = xhr.status >= 200 && xhr.status < 300 && result.ok
        ? `Firmware validated — ready to reboot\n${JSON.stringify(result, null, 2)}`
        : `OTA failed\n${JSON.stringify(result, null, 2)}`;
    } catch {
      otaStatus.textContent = `OTA failed\n${xhr.responseText}`;
    }
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
  if (configDirty) {
    showActionStatus('READY FOR LAUNCH is blocked until configuration changes are saved.');
    return;
  }
  let checklist = await api('/api/prelaunch/checklist');
  if (!checklist.ok) {
    const summary = checklistSummary(checklist);
    showActionStatus(`Pre-launch checklist is not complete:\n${summary}`);
    return;
  }
  const loraUnavailable = (checklist.items || []).some(item => item.key === 'lora_available' && !item.ok);
  const confirmation = loraUnavailable ? 'READY_FOR_LAUNCH_WITHOUT_LORA' : 'READY_FOR_LAUNCH';
  const prompt = loraUnavailable
    ? 'LoRa telemetry is unavailable. Continuing requires an explicit operator override.\n\nType READY_FOR_LAUNCH_WITHOUT_LORA to lock and arm without LoRa.'
    : 'This will lock the flight configuration.\nAfter this point, mission parameters cannot be edited until the allowed recovery/reset path.\nConfirm that the pre-launch checklist is complete.\n\nType READY_FOR_LAUNCH to lock and arm.';
  if (window.prompt(prompt) !== confirmation) return;
  const s = await api('/api/fsm/ready-for-launch', {
    method: 'POST',
    headers: { 'X-Confirm': confirmation },
    body: ''
  });
  showActionStatus(s);
  loadInfo();
}

navButtons.forEach(b => b.addEventListener('click', () => show(b.dataset.page)));
window.addEventListener('popstate', () => {
  const page = routePages[location.pathname] || 'info';
  if (!show(page, false)) show('info', false);
});
token.addEventListener('input', () => {
  saveToken();
  setAuthState(false);
});
authenticateButton.addEventListener('click', () => {
  if (isAuthenticated) {
    setAuthState(false);
  } else {
    authenticateOperator();
  }
});
token.addEventListener('keydown', event => {
  if (event.key === 'Enter' && !isAuthenticated) authenticateOperator();
});
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
document.getElementById('refreshFiles').addEventListener('click', loadFiles);
document.getElementById('filesContent').addEventListener('click', event => {
  const download = event.target.closest('[data-file-download]');
  if (download) downloadStoredFile(download.dataset.fileDownload);
  const remove = event.target.closest('[data-file-delete]');
  if (remove) deleteStoredFile(remove.dataset.fileDelete);
});
document.getElementById('reboot').addEventListener('click', async () => {
  const prompt = 'The board will reboot into the uploaded firmware.\n\nType REBOOT_TO_NEW_FIRMWARE to continue.';
  if (window.prompt(prompt) !== 'REBOOT_TO_NEW_FIRMWARE') return;
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
  }
});
document.getElementById('saveConfig').addEventListener('click', saveConfig);
document.getElementById('resetConfig').addEventListener('click', resetConfig);
document.getElementById('unlockConfig').addEventListener('click', unlockConfig);
document.addEventListener('input', event => {
  if (event.target.matches('[data-config]')) updateConfigDirtyState();
});
document.addEventListener('change', event => {
  if (event.target.matches('[data-config]')) updateConfigDirtyState();
  if (!event.target.classList.contains('attitude-axis-map')) return;
  attitudeMountingDraft[event.target.dataset.bodyAxis] = event.target.value;
  const status = document.getElementById('attitudeAlignmentStatus');
  if (status) status.textContent = validAttitudeMounting(attitudeMountingDraft) ? 'Right-handed mapping ready to apply.' : 'Choose a unique, right-handed axis mapping.';
});
document.addEventListener('click', event => {
  if (event.target.id !== 'applyAttitudeAlignment') return;
  const status = document.getElementById('attitudeAlignmentStatus');
  if (!validAttitudeMounting(attitudeMountingDraft)) {
    if (status) status.textContent = 'Choose a unique, right-handed axis mapping.';
    return;
  }
  attitudeMounting = { ...attitudeMountingDraft };
  localStorage.setItem('atlas_attitude_mounting', JSON.stringify(attitudeMounting));
  if (status) status.textContent = 'Saved in this browser.';
});

setInterval(() => runPoll('live', !otaUploadActive && !document.getElementById('live').classList.contains('hidden') && (!liveSocket || liveSocket.readyState !== WebSocket.OPEN), loadLive), 1000);
setInterval(() => runPoll('health', !otaUploadActive && !document.getElementById('health').classList.contains('hidden'), loadHealth), 3000);
setInterval(() => runPoll('ota', !otaUploadActive && !document.getElementById('ota').classList.contains('hidden'), refreshOta), 2000);
setInterval(() => runPoll('tests', !otaUploadActive && !document.getElementById('tests').classList.contains('hidden'), refreshTests), 1500);
setInterval(() => {
  if (!otaUploadActive && logStreamVisible() &&
      (!logSocket || logSocket.readyState > WebSocket.OPEN)) {
    loadLogs({ output: currentLogOutput() });
  }
}, 1500);
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
pauseTestLogsButton.addEventListener('click', () => {
  testLogsPaused = !testLogsPaused;
  pauseTestLogsButton.textContent = testLogsPaused ? 'Resume' : 'Pause';
  if (!testLogsPaused) renderLogs(testLogOutput, { force: true, forceBottom: true });
});
document.getElementById('clearTestLogs').addEventListener('click', () => {
  testLogLines = [];
  renderLogs(testLogOutput, { force: true, forceBottom: true });
});
document.getElementById('copyTestLogs').addEventListener('click', () => copyTextFrom(testLogOutput));
logFilter.addEventListener('input', () => renderLogs(logOutput));
setAuthState(false);
let initialPage = routePages[location.pathname] || 'info';
if (!show(initialPage, false)) initialPage = 'info';
history.replaceState({ page: initialPage }, '', pageRoutes[initialPage]);
