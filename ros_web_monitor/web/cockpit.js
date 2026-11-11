'use strict';
const VEH_GEAR = { 0: 'P', 5: 'D', 6: 'N', 7: 'R' };
const $ = (sel) => document.querySelector(/^[#.[]/.test(sel) ? sel : `#${sel}`);
const CAMS = [['Front_120_Camera', 'Front 120'], ['Front_60_Camera', 'Front 60'], ['Front_30_Camera', 'Front 30'], ['Rear_Camera', 'Rear'],
  ['Left_Front_Camera', 'Left Front'], ['Right_Front_Camera', 'Right Front'], ['Left_Rear_Camera', 'Left Rear'], ['Right_Rear_Camera', 'Right Rear']];
const GROUPS = [['cameras', '카메라'], ['lidar', '라이다'], ['radar', '레이더'], ['localization', '측위 (GNSS)'], ['vehicle_can', '차량 CAN'], ['perception', '인지']];
const KEY_TOPICS = ['/Front_120_Camera/image_raw/compressed', '/Multi_Lidar_pointcloud', '/front_radar_objects', '/gpsfix', '/acu_ck2_ver2',
  '/object_3d/Front_120_Camera/result_image/compressed', '/road_perception/status'];
const HISTORY_SEC = 60;
const LIDAR_MAX_POINTS = 5000;
const LIDAR_ZOOM_MIN = 0.45, LIDAR_ZOOM_MAX = 30, LIDAR_ZOOM_DEFAULT = 3.2, LIDAR_PITCH_DEFAULT = 1.08;
const SPEED_GAUGE_MAX = 120;
const STALE_SEC = 3;
const lidarDefault = () => ({ yaw: -({ '+x': 0, '+y': Math.PI / 2, '-x': Math.PI, '-y': -Math.PI / 2 }[GEOM.forward] ?? Math.PI / 2), pitch: LIDAR_PITCH_DEFAULT, zoom: LIDAR_ZOOM_DEFAULT, panX: 0, panY: 0 });
const SRC_NAMES = { object: '객체 인지', road: '도로 인지', raw: '카메라 영상' };
const GEOM = (() => {
  const d = { forward: '+y', cx: 0.2, cy: 1.6, length: 8.2, width: 3.0, height: 2.6, radarFrontX: 0, radarRearX: 0 };
  try { return { ...d, ...(JSON.parse(localStorage.getItem('rosWebVehicleGeom') || 'null') || {}) }; } catch (_) { return d; }
})();
const ui = (() => {
  const d = { cam: 'Front_120_Camera', src: 'object', lidar: null };
  try { return { ...d, ...(JSON.parse(localStorage.getItem('rosWebCockpit') || 'null') || {}) }; } catch (_) { return d; }
})();
const saveUi = () => { try { localStorage.setItem('rosWebCockpit', JSON.stringify(ui)); } catch (_) { /* 저장 불가 */ } };

let last = null, rectCamera = false, ports = [], lastDraw = 0;
const history = [];
const logItems = [];
let prevActive = null;

// 기존 화면 3D 주행 코드(app.js 의 @cockpit-share 구간)가 쓰는 전역
const vehicleGeom = GEOM;
const PROX_WARN_M = 7.0;
const state = { visualization: null, vehicle: null, vehiclePlay: null, gps: null, gpsPlay: null, playNamespace: '/bagplay' };
const view = { mode: 'live', vizTab: 'drive3d', cameraLayouts: {}, vizLayers: (() => {
  try { return JSON.parse(localStorage.getItem('rosWebVizLayers') || 'null') || []; } catch (_) { return []; }
})() };
const vizStats = document.createElement('span');
function isPlayTopic(name) { return typeof name === 'string' && name.startsWith(`${state.playNamespace}/`); }
function isClusterPolygon(m) { return /\/cluster_boxes$/.test(m.topic) && m.type === 4; }
function clusterBoxShadowed(viz, box) {
  return /\/cluster_detections$/.test(box.topic)
    && (viz.markers || []).some((m) => isClusterPolygon(m) && isPlayTopic(m.topic) === isPlayTopic(box.topic));
}
function logicalTopic(name) { return isPlayTopic(name) ? name.slice(state.playNamespace.length) : name; }
function effectiveTopic(topic) { return topic && view.mode === 'play' ? state.playNamespace + topic : topic; }
// 실시간 뷰와 Play 뷰는 /bagplay 접두어로만 구분한다
const inMode = (name) => isPlayTopic(name) === (view.mode === 'play');
const modeTopics = (s) => (s.topics || []).filter((t) => inMode(t.name));
const modeGps = (s) => (view.mode === 'play' ? s.gps_play : s.gps);
const escapeHtml = (v) => String(v).replace(/[&<>"']/g, (c) => `&#${c.charCodeAt(0)};`);
function vehHeading() { return { '+x': 0, '+y': Math.PI / 2, '-x': Math.PI, '-y': -Math.PI / 2 }[vehicleGeom.forward] ?? Math.PI / 2; }
function toVeh(x, y) {
  const h = vehHeading(), c = Math.cos(h), sn = Math.sin(h), dx = x - vehicleGeom.cx, dy = y - vehicleGeom.cy;
  return [c * dx + sn * dy, -sn * dx + c * dy];
}
function insideVehicle(x, y) { const [f, l] = toVeh(x, y); return Math.abs(f) <= vehicleGeom.length / 2 && Math.abs(l) <= vehicleGeom.width / 2; }
function vehEdgeDist(x, y) { const [f, l] = toVeh(x, y); return Math.hypot(Math.max(0, Math.abs(f) - vehicleGeom.length / 2), Math.max(0, Math.abs(l) - vehicleGeom.width / 2)); }
function openCamZoom(topic) {
  const cam = String(topic).split('/').filter(Boolean)[0];
  if (CAMS.some((c) => c[0] === cam)) { ui.cam = cam; saveUi(); updateCams(); }
}
const openVizSettingsModal = () => {};
const setViewPreset = () => {};
let vizRaf = false, driveReady = false;
function renderVisualization() { if (driveReady && state.visualization) renderDriveView(); }
function scheduleViz() {
  if (vizRaf) return;
  vizRaf = true;
  requestAnimationFrame(() => { vizRaf = false; renderVisualization(); });
}
async function loadDriveCore() {
  const text = await (await fetch('/app.js')).text();
  const a = text.indexOf('// @cockpit-share-begin'), b = text.indexOf('// @cockpit-share-end');
  if (a < 0 || b < a) throw new Error('공유 코드 표시를 찾지 못했습니다');
  const script = document.createElement('script');
  script.textContent = text.slice(a, b);
  document.head.appendChild(script);
  view.vizTab = 'drive3d';    // 공유 코드가 저장값으로 덮어쓰므로 다시 고정
  driveSettings.camMode = 'off';   // 대시보드에는 썸네일이 있어 타일을 그리지 않음 (저장하지 않으므로 기존 화면 설정은 그대로)
  document.querySelectorAll('#drivePresets [data-dpreset]').forEach((x) => x.classList.toggle('active', x.dataset.dpreset === driveCam.preset));
  driveReady = true;
}

function fit(cv) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2), r = cv.getBoundingClientRect();
  const w = Math.max(1, Math.round(r.width * dpr)), h = Math.max(1, Math.round(r.height * dpr));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const g = cv.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, r.width, r.height);
  return [g, r.width, r.height];
}
function noData(g, w, h, text) {
  g.fillStyle = '#8d9aa8'; g.font = '14px system-ui, sans-serif'; g.textAlign = 'center';
  g.fillText(text, w / 2, h / 2); g.textAlign = 'left';
}

function streamUrl(topic, id, extra) {
  const port = ports.length ? ports[id % ports.length] : null;
  const base = port ? `${location.protocol}//${location.hostname}:${port}` : '';
  return `${base}/stream?topic=${encodeURIComponent(topic)}&id=ck${id}${extra || ''}`;
}
function setSrc(img, url) { if (img.dataset.url !== url) { img.dataset.url = url; img.src = url; } }
function clearStreams() { document.querySelectorAll('#mainImg, .thumb img').forEach((img) => { img.removeAttribute('src'); img.dataset.url = ''; }); }
function rawTopic(cam) { return effectiveTopic(`/${cam}/image_raw/compressed`); }

function buildThumbs() {
  $('thumbs').innerHTML = '';
  CAMS.forEach(([cam, label]) => {
    const b = document.createElement('button');
    b.type = 'button'; b.className = 'thumb'; b.dataset.cam = cam; b.title = `${label}을 크게 보기`;
    const img = document.createElement('img'); img.alt = `${label} 카메라`;
    const s = document.createElement('span'); s.textContent = label;
    const st = document.createElement('small'); st.textContent = '대기';
    b.append(img, s, st);
    b.addEventListener('click', () => { ui.cam = cam; saveUi(); updateCams(); });
    $('thumbs').appendChild(b);
  });
}
function updateCams() {
  if (!last) return;
  const names = new Set((last.cameras || []).map((c) => c.name));
  const label = (CAMS.find((c) => c[0] === ui.cam) || [0, ui.cam])[1];
  const candidates = { object: effectiveTopic(`/object_3d/${ui.cam}/result_image/compressed`), road: effectiveTopic(`/road_perception/${ui.cam}/result_image/compressed`), raw: rawTopic(ui.cam) };
  let topic = candidates[ui.src], note = '';
  if (ui.src !== 'raw' && !names.has(topic)) { topic = candidates.raw; note = ' (인지 결과 없음, 카메라 영상 표시)'; }
  const rect = topic === candidates.raw && rectCamera ? '&rect=1' : '';
  setSrc($('mainImg'), streamUrl(topic, 0, rect + '&main=1'));
  const inPlay = view.mode === 'play', paused = inPlay && !!(last.bag_play || {}).paused;
  const mainInfo = (last.cameras || []).find((c) => c.name === topic);
  const noSignal = !mainInfo || typeof mainInfo.age_sec !== 'number' || (!paused && mainInfo.age_sec >= STALE_SEC * (inPlay ? 2 : 1));
  $('mainLabel').textContent = `${inPlay ? 'Play · ' : ''}${label} · ${note ? SRC_NAMES.raw : SRC_NAMES[ui.src]}${note}${noSignal ? ' · 신호 없음' : ''}`;
  document.querySelectorAll('#mainSrc button').forEach((b) => b.classList.toggle('on', b.dataset.src === ui.src));
  document.querySelectorAll('.thumb').forEach((t, i) => {
    t.classList.toggle('on', t.dataset.cam === ui.cam);
    setSrc(t.querySelector('img'), streamUrl(rawTopic(t.dataset.cam), i + 1, rectCamera ? '&rect=1' : ''));
    const info = (last.cameras || []).find((c) => c.name === rawTopic(t.dataset.cam));
    const live = !!info && typeof info.age_sec === 'number' && (paused || info.age_sec < STALE_SEC * (inPlay ? 2 : 1));
    t.classList.toggle('stale', !live);
    t.querySelector('small').textContent = live ? (inPlay ? 'PLAY' : 'LIVE') : 'NO SIGNAL';
  });
}

function vehicleData(s) {
  const v = view.mode === 'play' ? s.vehicle_play : s.vehicle, gps = modeGps(s);
  if (v && [v.wheel_fl, v.wheel_fr, v.wheel_rl, v.wheel_rr].some((x) => typeof x === 'number')) {
    const w = [v.wheel_fl, v.wheel_fr, v.wheel_rl, v.wheel_rr].filter((x) => typeof x === 'number');
    return { src: 'CAN', speed: w.reduce((a, b) => a + b, 0) / w.length, steer: typeof v.steer_angle === 'number' ? v.steer_angle : null,
      gear: VEH_GEAR[Math.round(v.gear)] ?? '—', brake: v.brake, soc: v.soc };
  }
  if (gps && typeof gps.speed === 'number') return { src: 'GPS', speed: gps.speed * 3.6, steer: null, gear: null };
  return { src: '', speed: null, steer: null, gear: null };
}
const fix = (x, d = 1, u = '') => (typeof x === 'number' ? `${x.toFixed(d)}${u}` : '—');
function renderVehicle(d, s) {
  $('speed').textContent = d.speed == null ? '—' : Math.abs(d.speed).toFixed(1);
  $('speedSrc').textContent = d.src ? `${d.src} 기준` : '데이터 없음';
  document.querySelectorAll('#prnd i').forEach((el) => el.classList.toggle('on', el.textContent === d.gear));
  $('gFill').style.strokeDasharray = `${d.speed == null ? 0 : Math.min(100, Math.abs(d.speed) / SPEED_GAUGE_MAX * 100)} 100`;
  $('steer').textContent = fix(d.steer, 1, '°');
  $('brake').textContent = fix(d.brake, 1);
  $('soc').textContent = fix(d.soc, 0, '%');
  void s;
}
function li(cls, name, right, ratio) {
  const el = document.createElement('li');
  const dot = document.createElement('b'); dot.className = cls;
  const n = document.createElement('span'); n.textContent = name;
  const r = document.createElement('em'); r.textContent = right;
  const track = document.createElement('u'); const bar = document.createElement('s');
  bar.className = cls; bar.style.width = `${Math.round((ratio || 0) * 100)}%`;
  track.appendChild(bar);
  el.append(dot, n, r, track);
  return el;
}
function groupForTopic(name) {
  const n = logicalTopic(name).toLowerCase();
  if (n.includes('camera') || n.includes('image')) return 'cameras';
  if (n.includes('ouster') || n.includes('velodyne') || n.includes('lidar') || n.includes('point') || n.includes('scan')) return 'lidar';
  if (n.includes('radar')) return 'radar';
  if (n.includes('gps') || n.includes('geodetic') || n.includes('cov') || n.includes('euler')) return 'localization';
  if (n.includes('can') || n.startsWith('/acu')) return 'vehicle_can';
  if (n.includes('object') || n.includes('cluster') || n.includes('detection') || n.includes('track')) return 'perception';
  return '';
}
function renderGroups(s) {
  // 서버 groups 는 실시간과 재생 토픽을 합산하므로 현재 뷰의 토픽으로 다시 센다
  const map = {};
  for (const t of modeTopics(s)) {
    const g = map[groupForTopic(t.name)] ||= { total: 0, active: 0 };
    g.total += 1;
    if (t.active) g.active += 1;
  }
  $('groups').replaceChildren(...GROUPS.map(([k, name]) => {
    const g = map[k];
    if (!g) return li('', name, '토픽 없음');
    const ratio = g.active / Math.max(1, g.total);
    if (g.active === g.total) return li('ok-d', name, `정상 ${g.active}/${g.total}`, ratio);
    if (g.active === 0) return li('bad-d', name, `중단 0/${g.total}`, ratio);
    return li('warn-d', name, `일부 ${g.active}/${g.total}`, ratio);
  }));
}
function renderRes(s) {
  const y = s.system || {}, cpu = (y.cpu || {}).usage_percent, gpu = (y.gpu || {}).usage_percent, ram = (y.memory || {}).usage_percent;
  const temp = (y.thermal || []).reduce((m, t) => Math.max(m, t.c || 0), 0) || null;
  const rows = [['GPU', gpu, '%', 100], ['CPU', cpu, '%', 100], ['RAM', ram, '%', 100], ['TEMP', temp, '°C', 100]];
  $('board').textContent = (y.model || '').replace('NVIDIA ', '').replace(' (Developer kit)', '');
  $('res').replaceChildren(...rows.map(([n, v, u, max]) => {
    const row = document.createElement('div');
    const name = document.createElement('span'); name.textContent = n;
    const track = document.createElement('u'); const bar = document.createElement('s');
    const pct = typeof v === 'number' ? Math.min(100, v / max * 100) : 0;
    bar.style.width = `${pct}%`;
    bar.style.background = pct >= 90 ? 'var(--bad)' : pct >= 70 ? 'var(--warn)' : 'var(--good)';
    track.appendChild(bar);
    const val = document.createElement('b'); val.textContent = typeof v === 'number' ? `${Math.round(v)}${u}` : '—';
    row.append(name, track, val);
    return row;
  }));
}
function renderTopics(s) {
  const map = Object.fromEntries(modeTopics(s).map((t) => [logicalTopic(t.name), t]));
  $('topics').replaceChildren(...KEY_TOPICS.filter((n) => map[n]).map((n) => {
    const t = map[n], el = document.createElement('li');
    const dot = document.createElement('i'); dot.className = t.active ? 'ok-d' : 'bad-d';
    const c = document.createElement('code'); c.textContent = n; c.title = n;
    const r = document.createElement('b'); r.textContent = `${(t.rate_hz || 0).toFixed(1)} Hz`;
    el.append(dot, c, r);
    return el;
  }));
  const now = new Set(modeTopics(s).filter((t) => t.active).map((t) => logicalTopic(t.name)));
  if (prevActive) {
    for (const n of prevActive) if (!now.has(n)) logItems.unshift([s.time, `${n} 끊김`, 'bad-d']);
    for (const n of now) if (!prevActive.has(n)) logItems.unshift([s.time, `${n} 복구`, 'ok-d']);
    logItems.length = Math.min(logItems.length, 100);
  }
  prevActive = now;
  $('log').replaceChildren(...(logItems.length ? logItems : [[null, '페이지를 연 뒤 변화 없음', '']]).map(([t, text, cls]) => {
    const el = document.createElement('li');
    const dot = document.createElement('i'); dot.className = cls;
    const c = document.createElement('code'); c.textContent = text; c.title = text;
    el.append(dot, c);
    if (t) { const m = document.createElement('time'); m.textContent = t.slice(-8); el.appendChild(m); }
    return el;
  }));
}
function renderCounts(s) {
  const v = s.visualization || {}, boxes = (v.boxes || []).filter((b) => inMode(b.topic));
  const radar = (v.radar || []).filter((r) => inMode(r.topic)).reduce((n, r) => n + (r.objects || []).length, 0);
  const rows = [['차량 트랙', boxes.filter((b) => b.kind === 'track').length, '#3ddc84'], ['장애물 (클러스터)', boxes.filter((b) => b.kind === 'detection').length, '#ff7a70'], ['레이더 점', radar, '#b48cff']];
  $('counts').replaceChildren(...rows.map(([n, c, color]) => {
    const el = document.createElement('li');
    el.style.setProperty('--c', color);
    const a = document.createElement('span'); a.textContent = n;
    const b = document.createElement('b'); b.textContent = c;
    el.append(a, b);
    return el;
  }));
}

const HEIGHT_COLORS = ['#2b4bff', '#1f8dff', '#14c8e6', '#22dd88', '#9be22d', '#f2d21f', '#ff8a1f', '#ff3b30'];
function lidarLayer(v) {
  const want = effectiveTopic(v.merge_output), layers = v.point_layers || [];
  const l = layers.find((p) => p.topic === want);
  if (l && l.points && l.points.length) return l;
  // 병합 클라우드가 없으면(병합 전 라이다만 녹화된 bag 등) 병합 좌표로 옮긴 개별 라이다를 합쳐 표시
  const parts = layers.filter((p) => p.merge_part && inMode(p.topic) && p.points && p.points.length);
  return parts.length ? { points: parts.flatMap((p) => p.points), age_sec: Math.min(...parts.map((p) => p.age_sec)) } : null;
}
function lidarProject(x, y, z, w, h) {
  const c = ui.lidar, cy = Math.cos(c.yaw), sy = Math.sin(c.yaw), sp = Math.sin(c.pitch), cp = Math.cos(c.pitch);
  const forward = cy * x - sy * y, side = sy * x + cy * y;
  const depth = 170 / c.zoom + forward * cp - z * sp;      // 기존 RViz 화면(project3D)과 같은 투영
  const scale = Math.min(w, h) * 1.05 / Math.max(20 / Math.max(1, c.zoom / 4), depth);
  return { x: w / 2 - side * scale + c.panX, y: h * 0.55 - (forward * sp + z * cp) * scale + c.panY, ok: depth > 1 };
}
function drawLidar(s) {
  const [g, w, h] = fit($('lidarCv'));
  const v = s.visualization || {}, layer = lidarLayer(v);
  const bg = g.createRadialGradient(w / 2, h * 0.55, 20, w / 2, h * 0.55, Math.max(w, h));
  bg.addColorStop(0, '#0b1724'); bg.addColorStop(0.6, '#070d15'); bg.addColorStop(1, '#04070b');
  g.fillStyle = bg; g.fillRect(0, 0, w, h);
  if (!layer) { $('lidarInfo').textContent = ''; noData(g, w, h, '병합 포인트클라우드 없음'); return; }
  $('lidarInfo').textContent = `${layer.points.length.toLocaleString()}점 · ${fix(layer.age_sec, 1, 's')} 전`;
  const P = (x, y, z) => lidarProject(x, y, z, w, h);
  g.strokeStyle = 'rgba(70,120,200,.28)'; g.lineWidth = 1;
  for (const R of [5, 10, 20, 30, 50]) {
    g.beginPath(); let pen = false;
    for (let a = 0; a <= 72; a += 1) {
      const t = a / 72 * 2 * Math.PI, q = P(GEOM.cx + R * Math.cos(t), GEOM.cy + R * Math.sin(t), 0);
      if (!q.ok) { pen = false; continue; }
      if (pen) g.lineTo(q.x, q.y); else g.moveTo(q.x, q.y);
      pen = true;
    }
    g.stroke();
  }
  const dot = Math.min(3, 1.2 + 0.08 * ui.lidar.zoom);
  const buckets = HEIGHT_COLORS.map(() => []);
  for (const p of layer.points) {
    const q = P(p[0], p[1], p[2] || 0);
    if (!q.ok || q.x < 0 || q.x > w || q.y < 0 || q.y > h) continue;
    buckets[Math.max(0, Math.min(7, Math.floor(((p[2] || 0) + 0.5) / 0.75)))].push(q);
  }
  buckets.forEach((pts, i) => { g.fillStyle = HEIGHT_COLORS[i]; for (const q of pts) g.fillRect(q.x, q.y, dot, dot); });
  const wire = (corners, z0, z1, color) => {
    const lo = corners.map(([x, y]) => P(x, y, z0)), hi = corners.map(([x, y]) => P(x, y, z1));
    if (lo.concat(hi).some((q) => !q.ok)) return;
    g.strokeStyle = color; g.beginPath();
    for (let i = 0; i < 4; i += 1) {
      const j = (i + 1) % 4;
      g.moveTo(lo[i].x, lo[i].y); g.lineTo(lo[j].x, lo[j].y);
      g.moveTo(hi[i].x, hi[i].y); g.lineTo(hi[j].x, hi[j].y);
      g.moveTo(lo[i].x, lo[i].y); g.lineTo(hi[i].x, hi[i].y);
    }
    g.stroke();
  };
  for (const b of v.boxes || []) {
    if ((b.kind !== 'track' && b.kind !== 'detection') || !inMode(b.topic) || clusterBoxShadowed(v, b)) continue;
    const c = Math.cos(b.yaw || 0), sn = Math.sin(b.yaw || 0), hx = (b.sx || 0) / 2, hy = (b.sy || 0) / 2, z = b.z || 0, hz = (b.sz || 0) / 2;
    wire([[hx, hy], [hx, -hy], [-hx, -hy], [-hx, hy]].map(([a, d]) => [b.x + a * c - d * sn, b.y + a * sn + d * c]), z - hz, z + hz, b.kind === 'track' ? '#3ddc84' : '#ff7a70');
  }
  g.strokeStyle = '#ff7a70'; g.lineWidth = 1.2;
  for (const m of (v.markers || []).filter((x) => isClusterPolygon(x) && inMode(x.topic))) {
    g.beginPath(); let pen = false;
    for (const p of m.points) {
      const q = P(p[0], p[1], p[2] || 0);
      if (!q.ok) { pen = false; continue; }
      if (pen) g.lineTo(q.x, q.y); else g.moveTo(q.x, q.y);
      pen = true;
    }
    g.stroke();
  }
  const hd = vehHeading(), ch = Math.cos(hd), sh = Math.sin(hd), hl = GEOM.length / 2, hw = GEOM.width / 2;
  wire([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]].map(([f, l]) => [GEOM.cx + f * ch - l * sh, GEOM.cy + f * sh + l * ch]), 0, GEOM.height, '#ffffff');
}

function drawChart() {
  const [g, w, h] = fit($('chartCv'));
  const now = performance.now() / 1000, L = 40, R = 8, gap = 8, B = 16, ph = (h - B - gap * 2) / 3;
  const speedColor = '#57c7ff', series = [['Speed (km/h)', 'speed', speedColor], ['Steer (°)', 'steer', '#3ddc84'], ['Accel (m/s², 속도 변화)', 'accel', '#f2b441']];
  g.font = '11px system-ui, sans-serif';
  series.forEach(([name, key, color], i) => {
    const top = i * (ph + gap), pts = history.filter((p) => p[key] != null);
    g.strokeStyle = 'rgba(120,150,180,.14)'; g.strokeRect(L, top, w - L - R, ph);
    g.fillStyle = '#c4ced8'; g.fillText(name, L + 6, top + 13);
    if (pts.length < 2) { g.fillStyle = '#8d9aa8'; g.fillText('데이터 없음', L + (w - L - R) / 2 - 28, top + ph / 2 + 4); return; }
    let lo = Math.min(...pts.map((p) => p[key])), hi = Math.max(...pts.map((p) => p[key]));
    if (hi - lo < 1) { const c = (hi + lo) / 2; lo = c - 0.5; hi = c + 0.5; }
    g.fillStyle = '#8d9aa8'; g.fillText(hi.toFixed(hi < 10 ? 1 : 0), 4, top + 11); g.fillText(lo.toFixed(lo > -10 && lo < 10 ? 1 : 0), 4, top + ph - 2);
    g.strokeStyle = color; g.lineWidth = 1.6; g.beginPath();
    pts.forEach((p, n) => {
      const x = L + (1 - (now - p.t) / HISTORY_SEC) * (w - L - R), y = top + ph - (p[key] - lo) / (hi - lo) * ph;
      if (n) g.lineTo(x, y); else g.moveTo(x, y);
    });
    g.stroke();
    const firstX = L + (1 - (now - pts[0].t) / HISTORY_SEC) * (w - L - R), lastP = pts[pts.length - 1];
    const lastX = L + (1 - (now - lastP.t) / HISTORY_SEC) * (w - L - R);
    g.lineTo(lastX, top + ph); g.lineTo(firstX, top + ph); g.closePath();
    const fill = g.createLinearGradient(0, top, 0, top + ph);
    fill.addColorStop(0, `${color}55`); fill.addColorStop(1, `${color}00`);
    g.fillStyle = fill; g.fill();
    g.fillStyle = color; g.font = '600 13px system-ui, sans-serif'; g.textAlign = 'right';
    g.fillText(lastP[key].toFixed(Math.abs(lastP[key]) < 10 ? 2 : 1).replace(/^-(0\.0+)$/, '$1'), w - R - 6, top + 15);
    g.textAlign = 'left'; g.font = '11px system-ui, sans-serif';
  });
  g.fillStyle = '#8d9aa8';
  for (let t = 60; t >= 0; t -= 20) g.fillText(t ? `-${t}s` : '0s', L + (1 - t / HISTORY_SEC) * (w - L - R) - (t ? 10 : 14), h - 3);
}

let map = null, marker = null, trail = null, mapFollow = true;
const MAP_MIN_ZOOM = 17;
function renderMap(s) {
  const gps = modeGps(s);
  const row = (k, val) => {
    const wrap = document.createElement('div');
    const a = document.createElement('dt'); a.textContent = k;
    const b = document.createElement('dd'); b.textContent = val;
    wrap.append(a, b);
    return wrap;
  };
  if (!gps || typeof gps.latitude !== 'number') { $('gps').replaceChildren(row('GPS', '데이터 없음')); return; }
  $('gps').replaceChildren(row('위도', gps.latitude.toFixed(6)), row('경도', gps.longitude.toFixed(6)), row('고도', fix(gps.altitude, 1, ' m')), row('위성', gps.satellites ?? '—'));
  if (typeof L === 'undefined') return;
  const pos = [gps.latitude, gps.longitude];
  if (!map) {
    map = L.map('map', { zoomControl: false, attributionControl: false }).setView(pos, 17);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);
    trail = L.polyline([], { color: '#2f86ff', weight: 4 }).addTo(map);
    marker = L.circleMarker(pos, { radius: 7, color: '#fff', weight: 2, fillColor: '#27c46b', fillOpacity: 1 }).addTo(map);
    map.on('dragstart', () => mapSetFollow(false));   // 지도를 직접 옮기면 따라가기를 멈춤
    $('mapNow').disabled = false;
  }
  marker.setLatLng(pos);
  const t = trail.getLatLngs(), tail = t[t.length - 1];
  if (!tail || map.distance(tail, pos) > 1) { trail.addLatLng(pos); if (t.length > 3000) trail.setLatLngs(t.slice(-3000)); }
  if (mapFollow) map.panTo(pos, { animate: false });
}
function mapSetFollow(on) {
  mapFollow = on;
  $('mapNow').classList.toggle('off', !on);
}
function mapGoCurrent() {
  const gps = last && modeGps(last);
  if (!map || !gps || typeof gps.latitude !== 'number') return;
  mapSetFollow(true);
  map.setView([gps.latitude, gps.longitude], Math.max(map.getZoom(), MAP_MIN_ZOOM), { animate: true });
  renderMap(last);
}

function pushHistory(d) {
  const t = performance.now() / 1000, prev = history[history.length - 1];
  let accel = null;
  if (prev && d.speed != null && prev.speed != null && t > prev.t) accel = 0.7 * (prev.accel || 0) + 0.3 * ((Math.abs(d.speed) - prev.speed) / 3.6 / (t - prev.t));
  history.push({ t, speed: d.speed == null ? null : Math.abs(d.speed), steer: d.steer, accel });
  while (history.length && t - history[0].t > HISTORY_SEC) history.shift();
}
function render() {
  const s = last;
  if (!s) return;
  Object.assign(state, { visualization: s.visualization, vehicle: s.vehicle, vehiclePlay: s.vehicle_play, gps: s.gps, gpsPlay: s.gps_play, playNamespace: s.play_namespace || '/bagplay' });
  if (!modeChecked) { modeChecked = true; if (ui.mode === 'play' && (s.bag_play || {}).playing) { setViewMode('play'); return; } }
  sendVizConfig(s);
  if (!ports.length && s.stream_ports && s.stream_ports.length) ports = s.stream_ports;
  $('clock').textContent = s.time || '--';
  $('rec').hidden = !(s.bag && s.bag.recording);
  updateBagUi(s);
  const d = vehicleData(s);
  pushHistory(d);
  renderVehicle(d, s); renderGroups(s); renderRes(s); renderTopics(s); renderCounts(s); renderMap(s);
  updatePlayTrack(s);
  updateCams();
  drawLidar(s); renderVisualization(); drawChart();
}
function onState(data) {
  last = JSON.parse(data);
  const now = performance.now();
  if (now - lastDraw < 180) return;
  lastDraw = now;
  render();
}

function lidarRedraw() { if (last) drawLidar(last); }
function lidarSave() { saveUi(); document.querySelector('[data-zoom="lidar"] b').textContent = `${Math.round(ui.lidar.zoom / LIDAR_ZOOM_DEFAULT * 100)}%`; }
function lidarZoom(factor) {
  ui.lidar.zoom = Math.min(LIDAR_ZOOM_MAX, Math.max(LIDAR_ZOOM_MIN, ui.lidar.zoom * factor));
  lidarSave(); lidarRedraw();
}
function lidarReset() { ui.lidar = lidarDefault(); lidarSave(); lidarRedraw(); }
function setupLidarControls() {
  ui.lidar = { ...lidarDefault(), ...(ui.lidar || {}) };
  delete ui.zoom;
  const cv = $('lidarCv'), STEP = 1.14;
  let drag = null;
  document.querySelectorAll('[data-zoom="lidar"] button').forEach((b) => b.addEventListener('click', () => {
    if (b.dataset.z === 'reset') lidarReset(); else lidarZoom(b.dataset.z === 'in' ? STEP : 1 / STEP);
  }));
  document.querySelectorAll('#lidarViews button').forEach((b) => b.addEventListener('click', () => {
    const y0 = lidarDefault().yaw, keep = { zoom: ui.lidar.zoom, panX: 0, panY: 0 };
    ui.lidar = { ...keep, ...{ top: { yaw: y0, pitch: Math.PI / 2 }, front: { yaw: y0, pitch: 0.02 }, left: { yaw: y0 + Math.PI / 2, pitch: 0.02 }, right: { yaw: y0 - Math.PI / 2, pitch: 0.02 } }[b.dataset.view] };
    lidarSave(); lidarRedraw();
  }));
  // 기존 RViz 와 같은 조작: 왼쪽 드래그 = 회전, 휠 클릭 드래그 = 이동, 휠 = 확대
  cv.addEventListener('pointerdown', (e) => {
    drag = { mode: e.button === 1 ? 'pan' : 'rotate', x: e.clientX, y: e.clientY };
    if (e.button === 1) e.preventDefault();
    cv.setPointerCapture(e.pointerId);
  });
  cv.addEventListener('pointermove', (e) => {
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y, c = ui.lidar;
    drag.x = e.clientX; drag.y = e.clientY;
    if (drag.mode === 'pan') { c.panX += dx; c.panY += dy; } else {
      c.yaw += dx * 0.008;
      c.pitch = Math.max(0.02, Math.min(Math.PI / 2, c.pitch - dy * 0.006));
    }
    lidarRedraw();
  });
  const end = () => { if (drag) { drag = null; saveUi(); } };
  cv.addEventListener('pointerup', end);
  cv.addEventListener('pointercancel', end);
  cv.addEventListener('mousedown', (e) => { if (e.button === 1) e.preventDefault(); });
  cv.addEventListener('auxclick', (e) => { if (e.button === 1) e.preventDefault(); });
  cv.addEventListener('dblclick', lidarReset);
  cv.addEventListener('wheel', (e) => { e.preventDefault(); lidarZoom(e.deltaY < 0 ? STEP : 1 / STEP); }, { passive: false });
  lidarSave();
}
setupLidarControls();
buildThumbs();
$('mapNow').addEventListener('click', mapGoCurrent);
document.querySelectorAll('#mainSrc button').forEach((b) => b.addEventListener('click', () => { ui.src = b.dataset.src; saveUi(); updateCams(); }));
document.querySelectorAll('#tabs button').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('#tabs button').forEach((x) => x.classList.toggle('on', x === b));
  $('topics').hidden = b.dataset.tab !== 'topics'; $('log').hidden = b.dataset.tab !== 'log';
}));
$('full').addEventListener('click', () => { if (document.fullscreenElement) document.exitFullscreen(); else document.documentElement.requestFullscreen().catch(() => {}); });
window.addEventListener('resize', () => { if (map) map.invalidateSize(); if (last) { drawLidar(last); renderVisualization(); drawChart(); } });
fetch('/api/calibration_apply').then((r) => r.json()).then((j) => { if (j.ok) { rectCamera = !!j.settings.camera; updateCams(); } }).catch(() => {});

// 점 수는 서버 공용 설정. 기존 화면의 RViz 와 같은 상한으로 맞춘다 (범위는 현재 값 유지)
let vizConfigSent = false;
function sendVizConfig(s) {
  if (vizConfigSent || !s.visualization) return;
  vizConfigSent = true;
  fetch('/api/viz-config', { method: 'POST', body: JSON.stringify({ maxPoints: LIDAR_MAX_POINTS, range: s.visualization.range_m }) }).catch(() => {});
}

// ── 실시간/Play 뷰 전환 ──
let modeChecked = false;
function setViewMode(mode) {
  view.mode = mode === 'play' ? 'play' : 'live';
  ui.mode = view.mode; saveUi();
  document.querySelectorAll('#viewMode button').forEach((b) => b.classList.toggle('on', b.dataset.mode === view.mode));
  history.length = 0; prevActive = null;
  clearStreams();   // 새 뷰의 프레임이 없을 때 이전 뷰 영상이 남지 않게 함
  if (trail) trail.setLatLngs([]);
  mapSetFollow(true);
  if (last) render();
}

// ── 안내/확인 창 ──
function ask(text, okLabel = '확인', { danger = false, cancel = true } = {}) {
  const d = $('askDlg');
  if (d.open) d.close('');
  $('askText').textContent = text;
  $('askOk').textContent = okLabel;
  $('askOk').classList.toggle('danger', danger);
  $('askCancel').hidden = !cancel;
  d.returnValue = '';
  d.showModal();
  return new Promise((resolve) => d.addEventListener('close', () => resolve(d.returnValue === 'ok'), { once: true }));
}
const notice = (text) => ask(text, '확인', { cancel: false });
async function postJson(url, body) {
  const opts = { method: 'POST', cache: 'no-store' };
  if (body !== undefined) { opts.headers = { 'Content-Type': 'application/json' }; opts.body = JSON.stringify(body); }
  return (await fetch(url, opts)).json();
}
async function playAction(label, url, body) {
  try {
    const d = await postJson(url, body);
    if (d.ok === false) { await notice(`${label} 실패: ${d.error || '알 수 없는 오류'}`); return null; }
    return d;
  } catch (e) { await notice(`${label} 요청 실패: ${e.message}`); return null; }
}

// ── Bag 녹화 (녹화 대상은 기존 화면 설정값을 그대로 사용) ──
async function toggleBag() {
  const rec = !!(last && last.bag && last.bag.recording), btn = $('bagBtn');
  btn.disabled = true;
  try {
    if (rec) {
      const d = await playAction('Bag 저장', '/api/bag/stop');
      if (d && d.path) await notice(d.mcap_count === 0 ? `Bag 저장됨, 그러나 .mcap 파일이 없습니다 (수신된 메시지 없음 또는 토픽 미선택):\n${d.path}` : `Bag 저장 완료 (mcap ${d.mcap_count}개):\n${d.path}`);
    } else {
      let cfg = { all: true, topics: [] };
      try { cfg = JSON.parse(localStorage.getItem('rosWebBagConfig') || 'null') || cfg; } catch (_) { /* 기본값 사용 */ }
      await playAction('Bag 녹화', '/api/bag/start', { all: !!cfg.all, topics: cfg.all ? [] : (cfg.topics || []), light: cfg.light !== false });
    }
  } finally { btn.disabled = false; }
}

// ── Bag 재생 ──
let playSeeking = false, playBarShown = false;
function fmtClock(sec) {
  const t = Math.max(0, Math.floor(sec || 0));
  return `${Math.floor(t / 60)}:${String(t % 60).padStart(2, '0')}`;
}
function updateBagUi(s) {
  const rec = !!(s.bag && s.bag.recording), bp = s.bag_play || {}, active = !!(bp.playing || bp.starting);
  $('bagBtn').textContent = rec ? '■ Bag Save' : '● Bag';
  $('bagBtn').classList.toggle('recording', rec);
  $('playBtn').textContent = bp.starting ? '⏳ 준비중' : (bp.playing ? '■ Play 중지' : '▶ Play');
  $('playBtn').classList.toggle('playing', !!bp.playing);
  document.querySelector('#viewMode [data-mode="play"]').classList.toggle('has-data', !!bp.playing);
  $('playDlgStop').hidden = !bp.playing;
  if (playBarShown !== active) {
    playBarShown = active;
    $('playBar').hidden = !active;
    if (map) map.invalidateSize();
  }
  if (!active) return;
  $('pbText').textContent = bp.starting ? `bag 재생 준비 중… (${bp.file || ''})` : `${bp.paused ? '일시정지' : '재생 중'}: ${bp.file || ''}`;
  const timed = !!bp.playing && bp.duration_sec > 0 && !!bp.ctrl_available;
  ['pbPause', 'pbSeek', 'pbCur', 'pbDur', 'pbRate'].forEach((id) => { $(id).hidden = !timed; });
  $('pbNext').hidden = !timed || !bp.paused;
  if (!timed) return;
  $('pbPause').textContent = bp.paused ? '▶' : '⏸';
  $('pbDur').textContent = fmtClock(bp.duration_sec);
  if (!playSeeking) {
    $('pbCur').textContent = fmtClock(bp.position_sec);
    $('pbSeek').max = String(bp.duration_sec);
    $('pbSeek').value = String(bp.position_sec || 0);
  }
  if (document.activeElement !== $('pbRate')) $('pbRate').value = String(bp.rate || 1);
}
async function renderPlayFiles() {
  const box = $('playFiles');
  const info = (text) => { const el = document.createElement('p'); el.className = 'play-empty'; el.textContent = text; box.replaceChildren(el); };
  info('불러오는 중…');
  let d;
  try { d = await (await fetch('/api/bag/play/list', { cache: 'no-store' })).json(); } catch (e) { info(`목록 조회 실패: ${e.message}`); return; }
  if (!d.ok) { info(d.error || '목록 조회 실패'); return; }
  if (!(d.files || []).length) { info('재생할 수 있는 bag이 없습니다. Bag 버튼으로 먼저 녹화해 주세요.'); return; }
  const bp = (last && last.bag_play) || {};
  box.replaceChildren(...d.files.map((f) => {
    const b = document.createElement('button');
    const playing = bp.playing && bp.file === f.name;
    b.type = 'button'; b.className = `play-file${playing ? ' on' : ''}`;
    const type = document.createElement('i'); type.textContent = f.type === 'ros1' ? 'ROS1' : 'ROS2';
    const name = document.createElement('b'); name.textContent = playing ? `${f.name} · 재생 중` : f.name;
    const meta = document.createElement('small'); meta.textContent = `${f.size_kb ? `${f.size_kb} KB · ` : ''}${f.modified || ''}`;
    b.append(type, name, meta);
    b.addEventListener('click', () => startPlay(f.name));
    return b;
  }));
}
async function startPlay(name) {
  document.querySelectorAll('#playFiles button').forEach((b) => { b.disabled = true; });
  const d = await playAction('재생', '/api/bag/play/start', { name, loop: $('playLoop').checked });
  if (d) { $('playDlg').close(); setViewMode('play'); } else renderPlayFiles();
}
function stopPlay() { return playAction('재생 중지', '/api/bag/play/stop'); }
function openPlayDlg() { $('playDlg').showModal(); renderPlayFiles(); }

// 재생 중인 bag 의 전체 GPS 경로 (실시간 궤적과 별개로 표시)
let playTrackLine = null, playTrackFile = null, playTrackPending = false, playTrackNext = 0;
function clearPlayTrack() {
  if (playTrackLine && map) map.removeLayer(playTrackLine);
  playTrackLine = null; playTrackFile = null;
}
async function updatePlayTrack(s) {
  const bp = s.bag_play || {}, want = view.mode === 'play' && bp.playing ? bp.file : null;
  if (!want) { if (playTrackFile) clearPlayTrack(); return; }
  if (!map || playTrackPending || playTrackFile === want || performance.now() < playTrackNext) return;
  playTrackPending = true;
  try {
    const t = await (await fetch('/api/bag/play/track', { cache: 'no-store' })).json();
    if (t.file !== want) return;
    if (t.status === 'loading') { playTrackNext = performance.now() + 2000; return; }
    clearPlayTrack();
    playTrackFile = want;
    if (t.status === 'ready' && t.points && t.points.length) {
      playTrackLine = L.polyline(t.points.map((p) => [p[1], p[2]]), { color: '#c084fc', weight: 4, opacity: 0.85 }).addTo(map);
    }
  } catch (_) {
    playTrackNext = performance.now() + 2000;
  } finally { playTrackPending = false; }
}

// ── 종료/재부팅 ──
function freeConnections() {   // 영상·이벤트 연결을 닫아 전원 요청이 반드시 나가게 함
  if (source) source.close();
  clearStreams();
}
async function systemPower(action) {
  const label = action === 'shutdown' ? '종료' : '재부팅', cmd = action === 'shutdown' ? 'shutdown -h now' : 'reboot';
  if (!(await ask(`정말 시스템을 ${label}하시겠습니까?\n모든 센서와 인지 노드가 멈춥니다. 녹화 중인 bag은 먼저 저장됩니다.\n\n명령: ${cmd}`, label, { danger: true }))) return;
  setLink('wait', `${label} 요청 중…`);
  freeConnections();
  await new Promise((r) => setTimeout(r, 150));
  for (let attempt = 1; attempt <= 4; attempt += 1) {
    try {
      const ctrl = new AbortController(), timer = setTimeout(() => ctrl.abort(), 4000);
      const res = await fetch(`/api/system/${action}`, { method: 'POST', signal: ctrl.signal, cache: 'no-store' });
      clearTimeout(timer);
      const d = await res.json();
      if (d && d.ok === false) {
        connectEvents();
        await notice(`${label} 실패:\n${d.error || '권한이 부족합니다.'}`);
      } else setLink('wait', `${label} 중…`);
      return;
    } catch (_) {   // 전원 동작이 시작되어 응답 전에 끊겼거나 연결을 못 잡은 경우
      if (attempt === 4) { setLink('wait', `${label} 중…`); return; }
      await new Promise((r) => setTimeout(r, 400));
    }
  }
}

document.querySelectorAll('#viewMode button').forEach((b) => b.addEventListener('click', () => setViewMode(b.dataset.mode)));
$('bagBtn').addEventListener('click', toggleBag);
$('playBtn').addEventListener('click', () => { if (last && last.bag_play && last.bag_play.playing) stopPlay(); else openPlayDlg(); });
$('pbStop').addEventListener('click', stopPlay);
$('pbPause').addEventListener('click', () => playAction('재생 제어', `/api/bag/play/${last && last.bag_play && last.bag_play.paused ? 'resume' : 'pause'}`));
$('pbNext').addEventListener('click', () => playAction('다음 프레임', '/api/bag/play/next'));
$('pbSeek').addEventListener('input', () => { playSeeking = true; $('pbCur').textContent = fmtClock(Number($('pbSeek').value)); });
$('pbSeek').addEventListener('change', async () => { await playAction('이동', '/api/bag/play/seek', { sec: Number($('pbSeek').value) }); playSeeking = false; });
$('pbRate').addEventListener('change', () => playAction('배속 변경', '/api/bag/play/rate', { rate: Number($('pbRate').value) }));
$('playDlgClose').addEventListener('click', () => $('playDlg').close());
$('playDlgStop').addEventListener('click', async () => { await stopPlay(); renderPlayFiles(); });
$('rebootBtn').addEventListener('click', () => systemPower('reboot'));
$('shutdownBtn').addEventListener('click', () => systemPower('shutdown'));

loadDriveCore().catch((e) => { $('driveCount').textContent = `3D 주행을 불러오지 못했습니다: ${e.message}`; });
const setLink = (cls, text) => { $('link').className = `pill ${cls}`; $('link').textContent = text; };
let source = null;
function connectEvents() {
  source = new EventSource('/events');
  source.onopen = () => setLink('ok', '데이터 수신 중');
  source.onmessage = (e) => { setLink('ok', '데이터 수신 중'); onState(e.data); };
  source.onerror = () => setLink('bad', '연결 끊김');
}
connectEvents();
