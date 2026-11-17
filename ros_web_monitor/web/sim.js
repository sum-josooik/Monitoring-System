'use strict';
// 주행 모델 시뮬레이션 화면 (/sim): sp_sim 노드의 상태(/api/sim)와 모델 입력 영상(/api/sim/preview)을 그린다.
// 좌표: x = 카메라에서 전방 [m], y = 왼쪽 [m], z = 위 [m] (카메라 기준).
const $ = (id) => document.getElementById(id);
const COLORS = ['#38bdf8', '#f472b6', '#a3e635', '#fbbf24', '#c084fc', '#fb7185', '#34d399', '#f97316'];
const WIDE = { f: 455, cx: 256, cy: 151.8 };
const MODEL_HEIGHT = 1.22, LANE_HALF = 0.9;
const esc = (v) => String(v).replace(/[&<>"']/g, (c) => `&#${c.charCodeAt(0)};`);
const fix = (v, d = 1, unit = '') => (typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) + unit : '—');

let sim = null, simAge = null, focus = null, bevRange = 80, bevView = '3d', camEdited = false;
const imgs = { wide: new Image() };
let previewStamp = 0;
const ui = (() => { try { return JSON.parse(localStorage.getItem('rosWebSim') || 'null') || {}; } catch (_) { return {}; } })();
if (ui.focus) focus = ui.focus;
if (ui.range) bevRange = ui.range;
if (ui.view === 'top' || ui.view === '3d') bevView = ui.view;
let camTab = ['cam', 'object', 'road'].includes(ui.camTab) ? ui.camTab : 'cam', camOpen = ui.camOpen !== false;
const saveUi = () => { try { localStorage.setItem('rosWebSim', JSON.stringify({ focus, range: bevRange, view: bevView, layers, camTab, camOpen })); } catch (_) { /* 저장 불가 */ } };
const colorOf = (short) => COLORS[Math.max(0, (sim?.catalog || []).findIndex((m) => m.short_name === short)) % COLORS.length];

async function control(body) {
  try {
    const r = await (await fetch('/api/sim/control', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })).json();
    return r;
  } catch (_) { return { ok: false }; }
}

// ── 모델 목록 ──
function renderModels() {
  const list = $('modelList');
  const cat = sim?.catalog || [];
  if (!cat.length) {
    list.dataset.key = '';
    list.innerHTML = sim
      ? '<li style="grid-template-columns:1fr;cursor:default"><b>설치된 모델이 없습니다<small>위의 ＋ 모델 추가 에서 내려받으세요</small></b></li>'
      : '<li style="grid-template-columns:1fr;cursor:default"><b>sp_sim 노드가 실행 중이 아닙니다<small>ros2 launch sp_sim sp_sim.launch.py</small></b></li>';
    return;
  }
  const key = JSON.stringify(cat) + focus;
  if (list.dataset.key === key) return;
  list.dataset.key = key;
  const LABEL = { running: '실행 중', loading: '불러오는 중', ready: '꺼짐', missing: '파일 없음', error: '오류' };
  list.innerHTML = cat.map((m) => `<li data-short="${esc(m.short_name)}" class="${m.short_name === focus ? 'focus' : ''}" title="${esc(m.error || '눌러서 이 모델의 차선·도로 경계를 표시')}">
    <input type="checkbox" ${m.enabled ? 'checked' : ''} aria-label="${esc(m.name)} 켜기">
    <i style="background:${colorOf(m.short_name)}"></i>
    <b>${esc(m.name)}<small>${esc(m.short_name)}${m.error ? ' · ' + esc(m.error) : ''}</small></b>
    <em class="${m.status}">${LABEL[m.status] || m.status}</em></li>`).join('');
}
$('modelList').addEventListener('click', (e) => {
  const li = e.target.closest('li[data-short]');
  if (!li) return;
  if (e.target.matches('input')) {
    const on = [...document.querySelectorAll('#modelList input:checked')].map((x) => x.closest('li').dataset.short);
    control({ enable: on });
    return;
  }
  focus = li.dataset.short; saveUi(); $('modelList').dataset.key = ''; renderModels(); draw();
});

// ── 카메라 장착 ──
function renderCam() {
  if (!sim?.camera || camEdited) return;
  document.querySelectorAll('#camForm [data-k]').forEach((el) => { if (document.activeElement !== el) el.value = sim.camera[el.dataset.k]; });
  $('camName').textContent = sim.camera_name || '';
}
$('camForm').addEventListener('input', () => { camEdited = true; $('camMsg').textContent = '적용 전'; });
$('camApply').addEventListener('click', async () => {
  const cam = {};
  document.querySelectorAll('#camForm [data-k]').forEach((el) => { cam[el.dataset.k] = el.dataset.k === 'height_mode' ? el.value : Number(el.value); });
  const r = await control({ camera: cam });
  camEdited = false;
  $('camMsg').textContent = r.ok ? (r.running ? '적용됨' : 'sp_sim 노드 응답 없음') : '적용 실패';
});

// ── 입력 영상 위 예측 ──
function drawView(cv, img, K, scale) {
  const g = cv.getContext('2d'), w = cv.width, h = cv.height;
  g.fillStyle = '#05090e'; g.fillRect(0, 0, w, h);
  if (img.complete && img.naturalWidth) g.drawImage(img, 0, 0, w, h);
  else { g.fillStyle = '#8d9aa8'; g.font = '14px system-ui, sans-serif'; g.textAlign = 'center'; g.fillText('모델 입력 영상 대기 중', w / 2, h / 2); g.textAlign = 'left'; }
  // 수평선 (도로와 평행한 방향)
  g.setLineDash([8, 6]); g.strokeStyle = 'rgba(255,220,80,.75)'; g.lineWidth = 1;
  g.beginPath(); g.moveTo(0, K.cy * scale); g.lineTo(w, K.cy * scale); g.stroke(); g.setLineDash([]);
  if (!sim) return;
  const P = (x, y, z) => (x > 0.8 ? [(K.cx - K.f * y / x) * scale, (K.cy - K.f * z / x) * scale] : null);
  const line = (pts, color, width, dash) => {
    g.strokeStyle = color; g.lineWidth = width; g.setLineDash(dash || []); g.beginPath();
    let pen = false;
    for (const p of pts) { if (!p) { pen = false; continue; } if (pen) g.lineTo(p[0], p[1]); else g.moveTo(p[0], p[1]); pen = true; }
    g.stroke(); g.setLineDash([]);
  };
  const xs = sim.x_idxs || [];
  const hCam = sim.camera?.height_mode === 'ground' ? MODEL_HEIGHT : (sim.camera?.height ?? MODEL_HEIGHT);
  for (const [short, r] of Object.entries(sim.results || {})) {
    const color = colorOf(short), isFocus = short === focus;
    if (isFocus) {
      r.lanes.forEach((ln) => { if (ln.prob > 0.25) line(xs.map((x, i) => P(x, ln.y[i], ln.z[i])), `rgba(255,255,255,${0.35 + 0.6 * ln.prob})`, 2 * scale); });
      r.edges.forEach((ed) => line(xs.map((x, i) => P(x, ed.y[i], ed.z[i])), 'rgba(255,90,90,.85)', 2 * scale, [6, 5]));
      // 주행 경로 띠
      const L = [], R = [];
      for (const q of r.path) { L.push(P(q[0], q[1] + LANE_HALF, q[2] - hCam)); R.push(P(q[0], q[1] - LANE_HALF, q[2] - hCam)); }
      const poly = L.filter(Boolean).concat(R.filter(Boolean).reverse());
      if (poly.length > 3) { g.fillStyle = color + '55'; g.beginPath(); poly.forEach((p, i) => (i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1]))); g.closePath(); g.fill(); }
    }
    line(r.path.map((q) => P(q[0], q[1], q[2] - hCam)), color, (isFocus ? 3 : 1.8) * scale);
    const lead = r.leads[0];
    if (lead.prob > 0.5) {
      const p = P(lead.x, lead.y, -hCam);
      if (p) {
        const s = Math.max(8, 700 / lead.x) * scale / 2;
        g.strokeStyle = color; g.lineWidth = 2 * scale; g.strokeRect(p[0] - s, p[1] - s * 1.5, s * 2, s * 1.5);
        if (isFocus) { g.fillStyle = color; g.font = `600 ${12 * scale}px system-ui, sans-serif`; g.fillText(`${lead.x.toFixed(1)} m`, p[0] - s, p[1] - s * 1.5 - 4); }
      }
    }
  }
}

// ── 뷰어가 그릴 센서 자료: 웹 모니터의 3D 표시 자료(/api/viz)를 카메라 기준 좌표로 ──
let viz = null, ports = [], playNs = '/bagplay';
const VEH = { cx: 0.2, cy: 1.6, half: 4.1 };   // 병합 라이다 좌표에서 차량 중심(전방 = +y)과 범퍼까지 거리 — 웹 모니터 기본값과 같음
const HEIGHT_COLORS = ['#2b4bff', '#1f8dff', '#14c8e6', '#22dd88', '#9be22d', '#f2d21f', '#ff8a1f', '#ff3b30'];
const layers = { lidar: true, radar: true, cluster: true, pred: true, ...(ui.layers || {}) };
const camFwd = () => sim?.camera?.forward ?? 3.9;
const isPlay = () => sim?.mode === 'play';
const inMode = (topic) => String(topic).startsWith(`${playNs}/`) === isPlay();
function scene() {
  const fwd = camFwd(), out = { points: [], radar: [], clusters: layers.cluster ? (sim?.sensors?.clusters || []) : [] };
  const v = viz?.visualization;
  if (v && layers.lidar) {
    const all = v.point_layers || [], want = (isPlay() ? playNs : '') + (v.merge_output || '');
    let pts = (all.find((l) => l.topic === want) || {}).points;
    if (!pts || !pts.length) pts = all.filter((l) => l.merge_part && inMode(l.topic)).flatMap((l) => l.points || []);
    out.points = pts.map((q) => [q[1] - VEH.cy - fwd, -(q[0] - VEH.cx), q[2] || 0]);
  }
  if (layers.radar) {
    for (const r of (v && v.radar) || []) {
      if (!inMode(r.topic)) continue;
      const base = (r.side === 'rear' ? -VEH.half : VEH.half) - fwd;
      for (const o of r.objects || []) out.radar.push([base + o[0], o[1], r.side === 'rear']);
    }
    if (!out.radar.length) out.radar = (sim?.sensors?.radar || []).map(([f, l]) => [f, l, false]);
  }
  return out;
}
const heightColor = (z) => HEIGHT_COLORS[Math.max(0, Math.min(7, Math.floor((z + 0.5) / 0.75)))];

// ── 위에서 본 그림 ──
function fitBev() {
  const cv = $('bevCv'), dpr = Math.min(window.devicePixelRatio || 1, 2), rc = cv.getBoundingClientRect();
  const w = Math.max(1, Math.round(rc.width)), h = Math.max(1, Math.round(rc.height));
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
  const g = cv.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  return [g, w, h];
}
function drawBev() { if (bevView === '3d') draw3d(); else drawTop(); }
function drawTop() {
  const [g, w, h] = fitBev();
  g.fillStyle = '#05090e'; g.fillRect(0, 0, w, h);
  const back = Math.max(6, bevRange * 0.12), scale = (h - 30) / (bevRange + back), cx = w / 2, y0 = 15 + bevRange * scale;
  const S = (x, y) => [cx - y * scale, y0 - x * scale];
  // 격자
  const step = bevRange <= 40 ? 5 : bevRange <= 80 ? 10 : 20;
  g.font = '11px system-ui, sans-serif'; g.lineWidth = 1;
  for (let x = 0; x <= bevRange; x += step) {
    const [, sy] = S(x, 0);
    g.strokeStyle = 'rgba(120,160,200,.14)'; g.beginPath(); g.moveTo(0, sy); g.lineTo(w, sy); g.stroke();
    g.fillStyle = '#6f8196'; g.fillText(`${x} m`, 6, sy - 3);
  }
  for (let y = -Math.floor(cx / scale / step) * step; y <= cx / scale; y += step) {
    const [sx] = S(0, y);
    g.strokeStyle = 'rgba(120,160,200,.10)'; g.beginPath(); g.moveTo(sx, 0); g.lineTo(sx, h); g.stroke();
  }
  const sc = scene(), sens = sim?.sensors || {};
  $('bevInfo').textContent = `LiDAR ${sc.points.length.toLocaleString()}점 · 레이더 ${sc.radar.length} · 클러스터 ${sc.clusters.length}`;
  // 라이다 점 (높이별 색)
  for (const [f, l, z] of sc.points) {
    const [px, py] = S(f, l);
    if (px < 0 || px > w || py < 0 || py > h) continue;
    g.fillStyle = heightColor(z); g.fillRect(px, py, 2, 2);
  }
  // 라이다 클러스터
  g.strokeStyle = 'rgba(160,175,190,.8)'; g.fillStyle = 'rgba(160,175,190,.16)';
  for (const [f, l, sx, sy, yaw] of sc.clusters) {
    const c = Math.cos(yaw), s = Math.sin(yaw);
    const pts = [[sx / 2, sy / 2], [sx / 2, -sy / 2], [-sx / 2, -sy / 2], [-sx / 2, sy / 2]].map(([a, b]) => S(f + a * c - b * s, l + a * s + b * c));
    g.beginPath(); pts.forEach((p, i) => (i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1]))); g.closePath(); g.fill(); g.stroke();
  }
  // 레이더
  for (const [f, l, rear] of sc.radar) { const [px, py] = S(f, l); g.fillStyle = rear ? '#ff9f6b' : '#ffc04d'; g.beginPath(); g.arc(px, py, 3, 0, Math.PI * 2); g.fill(); }
  // 센서 기준 선행 거리
  const ref = (d, color, label) => {
    if (typeof d !== 'number') return;
    const [, sy] = S(d, 0);
    g.setLineDash([5, 5]); g.strokeStyle = color; g.beginPath(); g.moveTo(cx - 3.2 * scale, sy); g.lineTo(cx + 3.2 * scale, sy); g.stroke(); g.setLineDash([]);
    g.fillStyle = color; g.fillText(`${label} ${d.toFixed(1)} m`, cx + 3.2 * scale + 5, sy + 4);
  };
  ref(sens.lead_lidar, 'rgba(200,210,220,.95)', '라이다');
  ref(sens.lead_radar, '#ffc04d', '레이더');
  // 자차 (카메라가 원점, 차체 중심은 뒤쪽)
  const fwd = camFwd(), L = 8.2, Wd = 3.0;
  const drawn = typeof busModel !== 'undefined' && L * scale >= 30
    && busModel.draw(g, (f, l, z) => ({ x: cx - l * scale, y: y0 - (f - fwd) * scale, depth: -z }), { L, W: Wd, H: 2.6 });
  if (!drawn) { g.fillStyle = '#e8eef5'; const [bx, by] = S(-fwd + L / 2, Wd / 2); g.fillRect(bx, by, Wd * scale, L * scale); }
  // 모델 예측
  const xs = sim?.x_idxs || [];
  const line = (pts, color, width, dash) => {
    g.strokeStyle = color; g.lineWidth = width; g.setLineDash(dash || []); g.beginPath();
    pts.forEach((p, i) => (i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1]))); g.stroke(); g.setLineDash([]);
  };
  for (const [short, r] of Object.entries((layers.pred && sim?.results) || {})) {
    const color = colorOf(short), isFocus = short === focus;
    if (isFocus) {
      r.lanes.forEach((ln) => { if (ln.prob > 0.25) line(xs.map((x, i) => S(x, ln.y[i])), `rgba(255,255,255,${0.3 + 0.6 * ln.prob})`, 1.6); });
      r.edges.forEach((ed) => line(xs.map((x, i) => S(x, ed.y[i])), 'rgba(255,90,90,.8)', 1.6, [6, 5]));
    }
    line(r.path.map((q) => S(q[0], q[1])), color, isFocus ? 3 : 1.8);
    const lead = r.leads[0];
    if (lead.prob > 0.5) {
      const [px, py] = S(lead.x, lead.y);
      g.fillStyle = color; g.beginPath(); g.moveTo(px, py - 8); g.lineTo(px + 7, py + 5); g.lineTo(px - 7, py + 5); g.closePath(); g.fill();
      if (isFocus) g.fillText(`${lead.x.toFixed(1)} m · ${(lead.prob * 100).toFixed(0)}%`, px + 10, py + 4);
    }
  }
}

// ── 3D 예측 뷰어 (드래그 회전 · 휠 확대). 좌표: x 전방, y 왼쪽, z 위, 원점 = 카메라 아래 노면 ──
const cam3 = { yaw: 0, pitch: 0, dist: 0, tx: 0 };
function cam3Reset() { cam3.yaw = -0.22; cam3.pitch = 0.46; cam3.dist = bevRange * 0.8; cam3.tx = bevRange * 0.34; }
function draw3d() {
  const [g, w, h] = fitBev();
  const bg = g.createLinearGradient(0, 0, 0, h);
  bg.addColorStop(0, '#0a1420'); bg.addColorStop(0.5, '#070d15'); bg.addColorStop(1, '#04070b');
  g.fillStyle = bg; g.fillRect(0, 0, w, h);
  const cp = Math.cos(cam3.pitch), sp = Math.sin(cam3.pitch), cy = Math.cos(cam3.yaw), sy = Math.sin(cam3.yaw);
  const eye = [cam3.tx - cam3.dist * cp * cy, -cam3.dist * cp * sy, cam3.dist * sp];
  const norm = (v) => { const n = Math.hypot(...v) || 1; return v.map((x) => x / n); };
  const f = norm([cam3.tx - eye[0], -eye[1], -eye[2]]);
  const r = norm([f[1], -f[0], 0]);
  const u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]];
  const focal = h * 0.5 / Math.tan(0.45), NEAR = 0.5;
  const C = (x, y, z) => { const d = [x - eye[0], y - eye[1], z - eye[2]]; return [d[0] * r[0] + d[1] * r[1] + d[2] * r[2], d[0] * u[0] + d[1] * u[1] + d[2] * u[2], d[0] * f[0] + d[1] * f[1] + d[2] * f[2]]; };
  const Sc = (c) => [w / 2 + c[0] * focal / c[2], h * 0.52 - c[1] * focal / c[2]];
  const P = (x, y, z) => { const c = C(x, y, z); return c[2] > NEAR ? Sc(c) : null; };
  // 선분 (카메라 앞쪽만 남기고 자름)
  const seg = (a, b) => {
    let p = C(...a), q = C(...b);
    if (p[2] <= NEAR && q[2] <= NEAR) return;
    const cut = (m, n) => { const t = (NEAR - m[2]) / (n[2] - m[2]); return [m[0] + (n[0] - m[0]) * t, m[1] + (n[1] - m[1]) * t, NEAR + 1e-3]; };
    if (p[2] <= NEAR) p = cut(p, q); else if (q[2] <= NEAR) q = cut(q, p);
    const s1 = Sc(p), s2 = Sc(q);
    g.moveTo(s1[0], s1[1]); g.lineTo(s2[0], s2[1]);
  };
  const line = (pts, color, width, dash) => {
    g.strokeStyle = color; g.lineWidth = width; g.setLineDash(dash || []); g.beginPath();
    for (let i = 0; i + 1 < pts.length; i += 1) seg(pts[i], pts[i + 1]);
    g.stroke(); g.setLineDash([]);
  };
  const fill = (pts, color) => {
    const s = pts.map((p) => P(...p));
    if (s.some((p) => !p)) return false;
    g.fillStyle = color; g.beginPath(); s.forEach((p, i) => (i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1]))); g.closePath(); g.fill();
    return true;
  };
  // 노면 격자 + 거리 눈금
  const step = bevRange <= 40 ? 5 : bevRange <= 80 ? 10 : 20, lat = Math.max(20, step * 3), back = -Math.max(20, step * 2);
  g.strokeStyle = 'rgba(90,140,200,.16)'; g.lineWidth = 1; g.beginPath();
  for (let x = back; x <= bevRange + 1e-6; x += step) seg([x, -lat, 0], [x, lat, 0]);
  for (let y = -lat; y <= lat + 1e-6; y += step) seg([back, y, 0], [bevRange, y, 0]);
  g.stroke();
  g.font = '11px system-ui, sans-serif'; g.fillStyle = '#6f8196';
  for (let x = step; x <= bevRange + 1e-6; x += step) { const p = P(x, -lat, 0); if (p) g.fillText(`${x} m`, p[0] + 4, p[1] - 3); }
  const sc = scene(), sens = sim?.sensors || {}, xs = sim?.x_idxs || [];
  $('bevInfo').textContent = `LiDAR ${sc.points.length.toLocaleString()}점 · 레이더 ${sc.radar.length} · 클러스터 ${sc.clusters.length}`;
  const hCam = sim?.camera?.height_mode === 'ground' ? MODEL_HEIGHT : (sim?.camera?.height ?? MODEL_HEIGHT);
  const results = (layers.pred && sim?.results) || {};
  // 라이다 점 (높이별 색) — 자차·상자보다 먼저 그려 겹쳐도 물체가 보이게
  for (const [pf, pl, pz] of sc.points) {
    const c = C(pf, pl, pz);
    if (c[2] <= NEAR) continue;
    const q = Sc(c);
    if (q[0] < 0 || q[0] > w || q[1] < 0 || q[1] > h) continue;
    g.fillStyle = heightColor(pz); g.fillRect(q[0], q[1], 2, 2);
  }
  // 모델 예측 (노면 위)
  for (const [short, res] of Object.entries(results)) {
    const color = colorOf(short), isFocus = short === focus;
    if (isFocus) {
      res.lanes.forEach((ln) => { if (ln.prob > 0.25) line(xs.map((x, i) => [x, ln.y[i], ln.z[i] + hCam + 0.03]), `rgba(255,255,255,${0.3 + 0.6 * ln.prob})`, 2); });
      res.edges.forEach((ed) => line(xs.map((x, i) => [x, ed.y[i], ed.z[i] + hCam + 0.03]), 'rgba(255,90,90,.85)', 2, [7, 6]));
      for (let i = 0; i + 1 < res.path.length; i += 1) {
        const a = res.path[i], b = res.path[i + 1];
        fill([[a[0], a[1] + LANE_HALF, a[2] + 0.02], [b[0], b[1] + LANE_HALF, b[2] + 0.02], [b[0], b[1] - LANE_HALF, b[2] + 0.02], [a[0], a[1] - LANE_HALF, a[2] + 0.02]], color + '40');
      }
    }
    line(res.path.map((q) => [q[0], q[1], q[2] + 0.05]), color, isFocus ? 3 : 1.8);
  }
  // 센서 기준 선행 거리
  const ref = (d, color, label) => {
    if (typeof d !== 'number') return;
    line([[d, -1.6, 0.05], [d, 1.6, 0.05]], color, 1.5, [5, 5]);
    const p = P(d, -1.6, 0.05);
    if (p) { g.fillStyle = color; g.fillText(`${label} ${d.toFixed(1)} m`, p[0] + 6, p[1] + 4); }
  };
  ref(sens.lead_lidar, 'rgba(200,210,220,.95)', '라이다');
  ref(sens.lead_radar, '#ffc04d', '레이더');
  // 입체물: 먼 것부터 (라이다 상자 · 레이더 · 자차 · 모델이 본 선행 물체)
  const items = [];
  const box = (cx, cyy, sx, sy2, yaw, hh, stroke, fillc) => {
    const c = Math.cos(yaw), s = Math.sin(yaw);
    const base = [[sx / 2, sy2 / 2], [sx / 2, -sy2 / 2], [-sx / 2, -sy2 / 2], [-sx / 2, sy2 / 2]].map(([a, b]) => [cx + a * c - b * s, cyy + a * s + b * c]);
    fill(base.map(([x, y]) => [x, y, hh]), fillc);
    g.strokeStyle = stroke; g.lineWidth = 1.2; g.beginPath();
    for (let i = 0; i < 4; i += 1) {
      const a = base[i], b = base[(i + 1) % 4];
      seg([a[0], a[1], 0], [b[0], b[1], 0]); seg([a[0], a[1], hh], [b[0], b[1], hh]); seg([a[0], a[1], 0], [a[0], a[1], hh]);
    }
    g.stroke();
  };
  for (const [cf, cl, sx, sy2, yaw, sz] of sc.clusters) {
    items.push({ d: C(cf, cl, 0.8)[2], draw: () => box(cf, cl, sx, sy2, yaw, sz > 0.2 ? sz : 1.6, 'rgba(170,185,200,.85)', 'rgba(170,185,200,.14)') });
  }
  for (const [rf, rl, rear] of sc.radar) {
    items.push({ d: C(rf, rl, 0.6)[2], draw: () => {
      const a = P(rf, rl, 0), b = P(rf, rl, 0.7);
      if (!a || !b) return;
      const col = rear ? '#ff9f6b' : '#ffc04d';
      g.strokeStyle = col; g.globalAlpha = 0.6; g.lineWidth = 1; g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke(); g.globalAlpha = 1;
      g.fillStyle = col; g.beginPath(); g.arc(b[0], b[1], 3, 0, Math.PI * 2); g.fill();
    } });
  }
  const fwd = camFwd();
  items.push({ d: C(-fwd, 0, 1.3)[2], draw: () => {
    const ok = typeof busModel !== 'undefined' && busModel.draw(g, (bf, bl, bz) => { const c = C(bf - fwd, bl, bz); if (c[2] <= NEAR) return null; const p = Sc(c); return { x: p[0], y: p[1], depth: c[2] }; }, { L: 8.2, W: 3.0, H: 2.6 });
    if (!ok) box(-fwd, 0, 8.2, 3.0, 0, 2.6, '#e8eef5', 'rgba(232,238,245,.25)');
  } });
  for (const [short, res] of Object.entries(results)) {
    const lead = res.leads[0], color = colorOf(short);
    if (lead.prob <= 0.5) continue;
    items.push({ d: C(lead.x, lead.y, 0.8)[2] - 0.5, draw: () => {
      const q = [[lead.x, lead.y + 0.95, 0], [lead.x, lead.y - 0.95, 0], [lead.x, lead.y - 0.95, 1.5], [lead.x, lead.y + 0.95, 1.5]];
      fill(q, color + '33');
      line(q.concat([q[0]]), color, short === focus ? 2.5 : 1.5);
      const p = P(lead.x, lead.y, 1.6);
      if (p && short === focus) { g.fillStyle = color; g.font = '600 12px system-ui, sans-serif'; g.fillText(`${lead.x.toFixed(1)} m · ${(lead.prob * 100).toFixed(0)}%`, p[0] - 30, p[1] - 6); g.font = '11px system-ui, sans-serif'; }
    } });
  }
  items.sort((a, b) => b.d - a.d).forEach((it) => it.draw());
}
{
  const cv = $('bevCv');
  let drag = null;
  cv.addEventListener('pointerdown', (e) => { if (bevView !== '3d') return; drag = [e.clientX, e.clientY]; cv.setPointerCapture(e.pointerId); });
  cv.addEventListener('pointermove', (e) => {
    if (!drag) return;
    cam3.yaw -= (e.clientX - drag[0]) * 0.008;
    cam3.pitch = Math.max(0.06, Math.min(1.5, cam3.pitch + (e.clientY - drag[1]) * 0.006));
    drag = [e.clientX, e.clientY]; drawBev();
  });
  const end = () => { drag = null; };
  cv.addEventListener('pointerup', end); cv.addEventListener('pointercancel', end);
  cv.addEventListener('wheel', (e) => { if (bevView !== '3d') return; e.preventDefault(); cam3.dist = Math.max(6, Math.min(500, cam3.dist * Math.exp(e.deltaY * 0.0012))); drawBev(); }, { passive: false });
  cv.addEventListener('dblclick', () => { if (bevView === '3d') { cam3Reset(); drawBev(); } });
}

// ── Bag 불러오기 · 재생 제어 (웹 모니터의 Bag 재생 기능을 그대로 사용) ──
let play = null, playSeeking = false;
const mmss = (sec) => { const t = Math.max(0, Math.floor(sec || 0)); return `${Math.floor(t / 60)}:${String(t % 60).padStart(2, '0')}`; };
async function playPost(label, url, body) {
  try {
    const d = await (await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body || {}) })).json();
    if (d.ok === false) { window.alert(`${label} 실패: ${d.error || '알 수 없는 오류'}`); return null; }
    return d;
  } catch (e) { window.alert(`${label} 요청 실패: ${e.message}`); return null; }
}
function renderPlayBar() {
  const on = !!(play && (play.playing || play.starting));
  $('playBar').hidden = !on;
  $('playDlgStop').hidden = !on;
  if (!on) return;
  const ctrl = !!play.ctrl_available && play.duration_sec != null;
  $('pbText').textContent = `▶ ${play.file || ''}${play.playing ? (play.paused ? ' · 일시정지' : '') : ' · 시작 중…'}${play.loop ? ' · 반복' : ''}`;
  ['pbPause', 'pbRate'].forEach((id) => { $(id).hidden = !ctrl; });
  ['pbCur', 'pbSeek', 'pbDur'].forEach((id) => { $(id).hidden = play.duration_sec == null; });
  $('pbSeek').disabled = !ctrl;
  $('pbPause').textContent = play.paused ? '▶' : '⏸';
  if (play.duration_sec != null) {
    $('pbCur').textContent = mmss(play.position_sec); $('pbDur').textContent = mmss(play.duration_sec);
    if (!playSeeking) { $('pbSeek').max = String(play.duration_sec); $('pbSeek').value = String(play.position_sec || 0); }
  }
  if (document.activeElement !== $('pbRate')) $('pbRate').value = String(play.rate || 1);
}
async function pollPlay() {
  try { play = await (await fetch('/api/bag/play/status', { cache: 'no-store' })).json(); } catch (_) { play = null; }
  renderPlayBar();
}
async function renderPlayFiles() {
  const box = $('playFiles');
  const info = (text) => { const el = document.createElement('p'); el.className = 'play-empty'; el.textContent = text; box.replaceChildren(el); };
  info('불러오는 중…');
  let d;
  try { d = await (await fetch('/api/bag/play/list', { cache: 'no-store' })).json(); } catch (e) { info(`목록 조회 실패: ${e.message}`); return; }
  if (!d.ok) { info(d.error || '목록 조회 실패'); return; }
  if (!(d.files || []).length) { info('재생할 수 있는 Bag 이 없습니다. 기존 화면의 Bag 버튼으로 먼저 녹화해 주세요.'); return; }
  box.replaceChildren(...d.files.map((file) => {
    const b = document.createElement('button');
    const playing = !!(play && play.playing && play.file === file.name);
    b.type = 'button'; b.className = `play-file${playing ? ' on' : ''}`;
    const type = document.createElement('i'); type.textContent = file.type === 'ros1' ? 'ROS1' : 'ROS2';
    const name = document.createElement('b'); name.textContent = playing ? `${file.name} · 재생 중` : file.name;
    const meta = document.createElement('small'); meta.textContent = `${file.size_kb ? `${file.size_kb} KB · ` : ''}${file.modified || ''}`;
    b.append(type, name, meta);
    b.addEventListener('click', () => startPlay(file.name));
    return b;
  }));
}
async function startPlay(name) {
  document.querySelectorAll('#playFiles button').forEach((b) => { b.disabled = true; });
  const d = await playPost('재생', '/api/bag/play/start', { name, loop: $('playLoop').checked });
  if (d) { $('playDlg').close(); control({ mode: 'play' }); pollPlay(); } else renderPlayFiles();
}
function openPlayDlg() { $('playDlg').showModal(); renderPlayFiles(); }
$('playDlgClose').addEventListener('click', () => $('playDlg').close());
$('playDlgStop').addEventListener('click', async () => { await playPost('재생 중지', '/api/bag/play/stop'); await pollPlay(); renderPlayFiles(); });
$('pbOpen').addEventListener('click', openPlayDlg);
$('pbStop').addEventListener('click', async () => { await playPost('재생 중지', '/api/bag/play/stop'); pollPlay(); });
$('pbPause').addEventListener('click', async () => { await playPost('재생 제어', `/api/bag/play/${play && play.paused ? 'resume' : 'pause'}`); pollPlay(); });
$('pbSeek').addEventListener('input', () => { playSeeking = true; $('pbCur').textContent = mmss(Number($('pbSeek').value)); });
$('pbSeek').addEventListener('change', async () => { await playPost('이동', '/api/bag/play/seek', { sec: Number($('pbSeek').value) }); playSeeking = false; pollPlay(); });
$('pbRate').addEventListener('change', () => playPost('배속 변경', '/api/bag/play/rate', { rate: Number($('pbRate').value) }));

// ── 결과 표 ──
function renderTable() {
  const body = $('resultBody');
  const cat = sim?.catalog || [], res = sim?.results || {}, stats = sim?.stats || {}, sens = sim?.sensors || {};
  const shorts = cat.map((m) => m.short_name).filter((s) => res[s] || stats[s]);
  if (!shorts.length) { body.innerHTML = '<tr><td class="empty" colspan="14">왼쪽에서 모델을 켜면 결과가 여기에 쌓입니다.</td></tr>'; }
  else {
    const td = (text, cls = '') => `<td class="${cls}">${text}</td>`;
    const errCls = (e) => (e == null ? 'none' : e < 2 ? 'good' : e < 5 ? 'warn' : 'bad');
    const rateCls = (r) => (r >= 0.8 ? 'good' : r >= 0.4 ? 'warn' : 'bad');
    body.innerHTML = shorts.map((s) => {
      const r = res[s], st = stats[s], name = (cat.find((m) => m.short_name === s) || {}).name || s;
      const lead = r?.leads[0];
      return `<tr class="${s === focus ? 'focus' : ''}"><td><i style="background:${colorOf(s)}"></i>${esc(name)}</td>`
        + (r ? td(lead.prob > 0.5 ? `${fix(lead.x, 1, ' m')} (${(lead.prob * 100).toFixed(0)}%)` : `없음 (${(lead.prob * 100).toFixed(0)}%)`, lead.prob > 0.5 ? '' : 'none')
            + td(fix(sens.lead_lidar, 1, ' m'), sens.lead_lidar == null ? 'none' : '') + td(fix(sens.lead_radar, 1, ' m'), sens.lead_radar == null ? 'none' : '')
            + td(`${(r.lanes[1].prob * 100).toFixed(0)}% · ${(r.lanes[2].prob * 100).toFixed(0)}%`, rateCls(Math.min(r.lanes[1].prob, r.lanes[2].prob)))
            + td(fix(r.speed[0] * 3.6, 1, ' km/h')) + td(fix(r.infer_ms, 1, ' ms'))
          : '<td class="none" colspan="6" style="text-align:center">꺼져 있음</td>')
        + (st ? td(st.frames.toLocaleString()) + td(`${(st.lane_rate * 100).toFixed(0)}%`, rateCls(st.lane_rate)) + td(`${(st.lead_rate * 100).toFixed(0)}%`)
            + td(st.lead_err_lidar == null ? '—' : `${fix(st.lead_err_lidar, 2, ' m')} (${st.lead_cmp_lidar})`, errCls(st.lead_err_lidar))
            + td(st.lead_err_radar == null ? '—' : `${fix(st.lead_err_radar, 2, ' m')} (${st.lead_cmp_radar})`, errCls(st.lead_err_radar))
            + td(fix(st.path_jitter, 3, ' m'), st.path_jitter == null ? 'none' : '') + td(fix(st.infer_ms, 1, ' ms'))
          : '<td class="none" colspan="7" style="text-align:center">누적 결과 없음</td>') + '</tr>';
    }).join('');
  }
  $('tableInfo').textContent = sim ? `${sim.mode === 'play' ? 'Bag 재생' : '실시간'} · 오차 = |모델 선행 거리 − 센서 거리| 평균 (비교 횟수)` : '';
  $('resultFile').textContent = sim?.results_file ? `결과 파일: ${sim.results_file} (5초마다 저장)` : '';
}
// ── Bag 일괄 평가 (tools/eval_bag.py 결과) ──
let bagPick = null;
function renderBag() {
  const list = sim?.bag_results || [], sel = $('bagSel'), body = $('bagBody');
  const key = list.map((b) => b.bag + b.generated).join('|');
  if (sel.dataset.key !== key) {
    sel.dataset.key = key;
    sel.innerHTML = list.map((b) => `<option value="${esc(b.bag)}">${esc(b.bag)}</option>`).join('');
    if (!list.some((b) => b.bag === bagPick)) bagPick = list[0]?.bag ?? null;
    if (bagPick) sel.value = bagPick;
  }
  sel.hidden = list.length < 2;
  const b = list.find((x) => x.bag === bagPick);
  if (!b) {
    body.innerHTML = '<tr><td class="empty" colspan="9">아직 결과가 없습니다 — python3 ~/captin5_ws/src/sp_sim/tools/eval_bag.py &lt;Bag 폴더&gt;</td></tr>';
    $('bagInfo').textContent = '';
    return;
  }
  const td = (text, cls = '') => `<td class="${cls}">${text}</td>`;
  const rateCls = (r) => (r >= 0.8 ? 'good' : r >= 0.4 ? 'warn' : 'bad');
  body.innerHTML = Object.entries(b.models).map(([s, r]) => `<tr class="${s === focus ? 'focus' : ''}"><td><i style="background:${colorOf(s)}"></i>${esc(r.name)}</td>`
    + td(r.frames.toLocaleString()) + td(`${(r.lane_rate * 100).toFixed(0)}%`, rateCls(r.lane_rate)) + td(`${(r.lead_rate * 100).toFixed(0)}%`)
    + td(r.lead_err_lidar == null ? '—' : `${fix(r.lead_err_lidar, 2, ' m')} (${r.lead_cmp_lidar})`, r.lead_err_lidar == null ? 'none' : r.lead_err_lidar < 2 ? 'good' : r.lead_err_lidar < 5 ? 'warn' : 'bad')
    + td(r.speed_err_gps == null ? '—' : fix(r.speed_err_gps * 3.6, 1, ' km/h'), r.speed_err_gps == null ? 'none' : r.speed_err_gps * 3.6 < 5 ? 'good' : r.speed_err_gps * 3.6 < 12 ? 'warn' : 'bad')
    + td(r.speed_ratio_gps == null ? '—' : `×${fix(r.speed_ratio_gps, 2)}`, r.speed_ratio_gps == null ? 'none' : Math.abs(r.speed_ratio_gps - 1) < 0.15 ? 'good' : 'warn')
    + td(fix(r.path_jitter, 3, ' m'), r.path_jitter == null ? 'none' : '') + td(fix(r.infer_ms, 1, ' ms')) + '</tr>').join('');
  $('bagInfo').textContent = `${b.bag} · ${b.seconds} s 구간(시작 ${b.start} s) · ${b.frames.toLocaleString()} 프레임 · 평가 ${b.generated} · 카메라 높이 ${b.camera.height} m, 숙임 ${b.camera.pitch}° · 모델/GPS 속도비가 1 이 아니면 카메라 높이가 실제와 다를 수 있습니다`;
}
$('bagSel').addEventListener('change', (e) => { bagPick = e.target.value; renderBag(); });

$('resetStats').addEventListener('click', () => { if (window.confirm('이 모드의 누적 결과를 모두 지울까요?')) control({ reset_stats: true }); });

function renderLegend() {
  const items = Object.keys(sim?.results || {}).map((s) => `<span><i style="background:${colorOf(s)}"></i>${esc((sim.catalog.find((m) => m.short_name === s) || {}).name || s)} 경로</span>`);
  items.push('<span><i style="background:#fff"></i>차선 (선택한 모델)</span>', '<span><i style="background:#ff5a5a"></i>도로 경계</span>',
    '<span><i style="background:#ffdc50"></i>수평선</span>', '<span><i style="background:#a0afbe"></i>라이다 클러스터</span>', '<span><i style="background:#ffc04d"></i>레이더</span>');
  $('legend').innerHTML = items.join('');
}

function draw() {
  drawView($('wideCv'), imgs.wide, WIDE, 2);
  drawBev();
}
function render() {
  const link = $('link');
  const alive = sim && simAge != null && simAge < 3;
  link.className = `pill ${alive ? 'ok' : sim ? 'bad' : 'wait'}`;
  link.textContent = alive ? (sim.note || `모델 ${Object.keys(sim.results || {}).length}개 실행 중`) : sim ? 'sp_sim 응답 없음' : 'sp_sim 대기 중';
  if (sim) {
    document.querySelectorAll('#simMode button').forEach((b) => b.classList.toggle('on', b.dataset.mode === sim.mode));
    const run = Object.keys(sim.results || {});
    if (run.length && !run.includes(focus)) { focus = run[0]; $('modelList').dataset.key = ''; }
    const fr = sim.results?.[focus];
    const dt = sim.frame_dt || 0;
    $('viewInfo').textContent = fr ? `자차 속도(모델 추정) ${fix(fr.ego_speed * 3.6, 1, ' km/h')} · 영상 간격 ${fix(dt, 2, ' s')}${dt > 0.3 ? ' (모델 기준 0.2 s 보다 느림)' : ''}${sim.rx_hz != null ? ` · 수신 ${fix(sim.rx_hz, 0, ' Hz')}` : ''}` : '';
  }
  renderModels(); renderCam(); renderTable(); renderBag(); renderLegend(); renderCams();
  drawView($('wideCv'), imgs.wide, WIDE, 2);   // 예측 뷰어는 pollViz 주기에 맞춰 그린다
}

async function poll() {
  try {
    const r = await (await fetch('/api/sim', { cache: 'no-store' })).json();
    sim = r.data || sim; simAge = r.age;
  } catch (_) { simAge = null; }
  render();
}
async function pollPreview() {
  try {
    const r = await (await fetch('/api/sim/preview', { cache: 'no-store' })).json();
    if (r.data && r.data.stamp !== previewStamp) {
      previewStamp = r.data.stamp;
      imgs.wide.src = `data:image/jpeg;base64,${r.data.wide}`;
    }
  } catch (_) { /* 다음 주기에 다시 */ }
}
imgs.wide.onload = draw;
$('simMode').addEventListener('click', (e) => {
  const b = e.target.closest('button[data-mode]');
  if (!b) return;
  control({ mode: b.dataset.mode });
  if (b.dataset.mode === 'play') openPlayDlg();   // Bag 재생: 불러올 Bag 을 고르는 창을 연다
});
$('bevRange').addEventListener('click', (e) => {
  const b = e.target.closest('button[data-r]'); if (!b) return;
  bevRange = Number(b.dataset.r); saveUi(); cam3Reset(); syncRange(); drawBev();
});
$('bevView').addEventListener('click', (e) => {
  const b = e.target.closest('button[data-v]'); if (!b) return;
  bevView = b.dataset.v; saveUi(); syncRange(); drawBev();
});
$('bevReset').addEventListener('click', () => { cam3Reset(); drawBev(); });
function syncRange() {
  document.querySelectorAll('#bevRange button').forEach((b) => b.classList.toggle('on', Number(b.dataset.r) === bevRange));
  document.querySelectorAll('#bevView button').forEach((b) => b.classList.toggle('on', b.dataset.v === bevView));
  $('bevCv').classList.toggle('v3d', bevView === '3d');
  $('bevHint').hidden = bevView !== '3d'; $('bevReset').hidden = bevView !== '3d';
}
cam3Reset();
syncRange();
window.addEventListener('resize', drawBev);

// ── 뷰어 표시 항목 · 센서 자료 받기 ──
function syncLayers() { document.querySelectorAll('#bevLayers button[data-l]').forEach((b) => b.classList.toggle('on', !!layers[b.dataset.l])); }
$('bevLayers').addEventListener('click', (e) => {
  const b = e.target.closest('button[data-l]'); if (!b) return;
  layers[b.dataset.l] = !layers[b.dataset.l]; saveUi(); syncLayers(); drawBev();
});
syncLayers();
async function pollViz() {
  try {
    const r = await (await fetch(`/api/viz?play=${isPlay() ? 1 : 0}`, { cache: 'no-store' })).json();
    viz = r; playNs = r.play_namespace || playNs;
    if (!ports.length && (r.stream_ports || []).length) { ports = r.stream_ports; renderCams(); }
  } catch (_) { /* 웹 모니터가 옛 버전이면 이 경로가 없다 — 라이다 없이 그림 */ }
  drawBev();
}

// ── 카메라 패널: 카메라 8대 + AVM / 객체 인지 / 도로 인지 (웹 모니터의 영상 스트림) ──
const CAMS = [['Front_120_Camera', 'Front 120'], ['Front_60_Camera', 'Front 60'], ['Front_30_Camera', 'Front 30'], ['Rear_Camera', 'Rear'],
  ['Left_Front_Camera', 'Left Front'], ['Right_Front_Camera', 'Right Front'], ['Left_Rear_Camera', 'Left Rear'], ['Right_Rear_Camera', 'Right Rear']];
const OBJECT_CAMS = ['Front_120_Camera', 'Left_Front_Camera', 'Right_Front_Camera', 'Left_Rear_Camera', 'Right_Rear_Camera', 'Rear_Camera'];
function camTiles() {
  const label = (cam) => (CAMS.find((c) => c[0] === cam) || [0, cam])[1];
  if (camTab === 'object') return OBJECT_CAMS.map((cam) => [`/object_3d/${cam}/result_image/compressed`, `${label(cam)} · 객체`]);
  if (camTab === 'road') return CAMS.map(([cam, name]) => [`/road_perception/${cam}/result_image/compressed`, `${name} · 도로`]);
  return CAMS.map(([cam, name]) => [`/${cam}/image_raw/compressed`, name]).concat([['/avm_cam/image_raw', 'AVM']]);
}
let camKey = '';
function renderCams() {
  const grid = $('camGrid');
  document.querySelectorAll('#camTabs button').forEach((b) => b.classList.toggle('on', b.dataset.t === camTab));
  $('camToggle').textContent = camOpen ? '접기' : '펼치기';
  grid.hidden = !camOpen;
  const key = `${camOpen}|${camTab}|${isPlay()}|${ports.join(',')}`;
  if (key === camKey) return;
  camKey = key;
  grid.querySelectorAll('img').forEach((img) => { img.removeAttribute('src'); });   // 이전 스트림 연결을 끊는다
  if (!camOpen) { grid.replaceChildren(); $('camInfo').textContent = '영상 끔'; return; }
  const tiles = camTiles();
  $('camInfo').textContent = `${tiles.length}개 · ${isPlay() ? 'Bag 재생 영상' : '실시간'} · 누르면 크게`;
  grid.replaceChildren(...tiles.map(([topic, name], i) => {
    const fig = document.createElement('figure'); fig.className = 'cam-tile';
    const img = document.createElement('img'); img.alt = name; img.loading = 'lazy';
    const port = ports.length ? ports[i % ports.length] : null;
    const base = port ? `${location.protocol}//${location.hostname}:${port}` : '';
    img.src = `${base}/stream?topic=${encodeURIComponent((isPlay() ? playNs : '') + topic)}&id=sm${i}`;
    img.addEventListener('error', () => fig.classList.add('off'));
    img.addEventListener('load', () => fig.classList.remove('off'));
    const cap = document.createElement('figcaption'); cap.textContent = name;
    fig.append(img, cap);
    fig.addEventListener('click', () => { const z = !fig.classList.contains('zoom'); grid.querySelectorAll('.zoom').forEach((x) => x.classList.remove('zoom')); fig.classList.toggle('zoom', z); if (z) fig.scrollIntoView({ block: 'nearest' }); });
    return fig;
  }));
}
$('camTabs').addEventListener('click', (e) => { const b = e.target.closest('button[data-t]'); if (!b) return; camTab = b.dataset.t; camOpen = true; saveUi(); renderCams(); });
$('camToggle').addEventListener('click', () => { camOpen = !camOpen; saveUi(); renderCams(); });

// ── 모델 추가: sunnylink.wiki 카테고리별 목록에서 내려받기 ──
let cat = null, catPick = null, catTimer = null;
async function pollCatalog() {
  try {
    const r = await (await fetch('/api/sim/catalog', { cache: 'no-store' })).json();
    if (r.data) cat = r.data;
  } catch (_) { /* 다음 주기에 다시 */ }
  renderCatalog();
}
function renderCatalog() {
  const nav = $('catNav'), list = $('catList');
  if (!cat) { nav.replaceChildren(); list.innerHTML = '<p class="play-empty">카탈로그를 받는 중입니다… (sp_sim 노드가 실행 중이어야 합니다)</p>'; return; }
  const cats = cat.categories || [];
  if (!cats.some((c) => c.id === catPick)) catPick = (cats.find((c) => c.available) || cats[0] || {}).id || null;
  $('catMeta').textContent = cat.wiki_total ? `sunnylink.wiki ${cat.wiki_total}개 · 기준 ${cat.wiki_updated} · 받은 시각 ${cat.updated}` : '';
  $('catNote').textContent = cat.note || (cat.busy ? '목록 갱신 중…' : cat.queue ? `대기 중인 설치 ${cat.queue}건` : '');
  nav.innerHTML = cats.map((c) => `<button type="button" data-c="${esc(c.id)}" class="${c.id === catPick ? 'on' : ''}" title="받을 수 있는 모델 ${c.available}개 / 전체 ${c.models.length}개">
    <span>${esc(c.icon || '')}</span><span>${esc(c.name)}</span><small>${c.available}/${c.models.length}</small></button>`).join('');
  const c = cats.find((x) => x.id === catPick);
  if (!c) { list.innerHTML = '<p class="play-empty">카테고리가 없습니다. 목록 새로고침을 눌러 주세요.</p>'; return; }
  $('catDesc').textContent = c.description || '';
  const only = $('catOnly').checked, inst = cat.installing || {};
  const rows = c.models.filter((m) => !only || m.source || m.installed);
  list.innerHTML = rows.length ? rows.map((m) => {
    const st = m.id ? inst[m.id] : null;
    let act;
    if (st && st.stage) act = `<span class="busy">${esc(st.stage)}</span>`;
    else if (m.installed) act = `<span class="ok">설치됨${(cat.enabled || []).includes(m.id) ? ' · 실행 중' : ''}</span><button type="button" data-rm="${esc(m.id)}">삭제</button>`;
    else if (m.source) act = `<button type="button" class="go" data-in="${esc(m.id)}">내려받기</button>`;
    else act = `<span class="why">받을 수 없음<br>${esc(m.why || '')}</span>`;
    if (st && st.error) act += `<span class="err">${esc(st.error)}</span>`;
    const meta = [m.date, m.best_for, (m.tags || []).join(' · '), m.score != null ? `커뮤니티 ${m.score}점${m.votes ? ` (${m.votes}표)` : ''}` : ''].filter(Boolean).join(' · ');
    return `<div class="cat-item${m.source || m.installed ? '' : ' na'}"><b>${esc(m.name)}${m.badge ? `<em>${esc(m.badge)}</em>` : ''}</b>
      <div class="act">${act}</div><small title="${esc(m.note || '')}">${esc(meta)}${m.note ? `<br>${esc(m.note)}` : ''}</small></div>`;
  }).join('') : '<p class="play-empty">이 카테고리에는 받을 수 있는 모델이 없습니다.</p>';
}
$('catNav').addEventListener('click', (e) => { const b = e.target.closest('button[data-c]'); if (b) { catPick = b.dataset.c; renderCatalog(); $('catList').scrollTop = 0; } });
$('catOnly').addEventListener('change', renderCatalog);
$('catList').addEventListener('click', async (e) => {
  const add = e.target.closest('button[data-in]'), rm = e.target.closest('button[data-rm]');
  if (add) { add.disabled = true; await control({ install: [add.dataset.in] }); setTimeout(pollCatalog, 600); }
  if (rm && window.confirm('이 모델의 파일을 삭제할까요? (다시 받으려면 변환에 수 분이 걸립니다)')) { await control({ remove: rm.dataset.rm }); setTimeout(pollCatalog, 600); }
});
$('catRefresh').addEventListener('click', async () => { await control({ refresh_catalog: true }); setTimeout(pollCatalog, 600); });
$('catOpen').addEventListener('click', () => { $('catDlg').showModal(); pollCatalog(); clearInterval(catTimer); catTimer = setInterval(pollCatalog, 2000); });
$('catClose').addEventListener('click', () => $('catDlg').close());
$('catDlg').addEventListener('close', () => { clearInterval(catTimer); catTimer = null; });

// ── 테마 (다크 / 라이트) — 다른 화면과 같은 저장값 ──
function applyTheme(theme) {
  const t = theme === 'light' ? 'light' : 'dark';
  document.documentElement.dataset.theme = t;
  try { localStorage.setItem('rosWebTheme', t); } catch (_) { /* 저장 불가 */ }
  const btn = $('themeToggle');
  btn.textContent = t === 'light' ? '☀' : '☾';
  btn.title = t === 'light' ? '라이트 모드 (누르면 다크 모드)' : '다크 모드 (누르면 라이트 모드)';
}
applyTheme(document.documentElement.dataset.theme);
$('themeToggle').addEventListener('click', () => applyTheme(document.documentElement.dataset.theme === 'light' ? 'dark' : 'light'));
window.addEventListener('storage', (e) => { if (e.key === 'rosWebTheme' && e.newValue) applyTheme(e.newValue); });

poll(); pollPreview(); pollPlay(); pollViz(); renderCams();
setInterval(pollPlay, 1000);
setInterval(pollViz, 500);
setInterval(poll, 250);
setInterval(pollPreview, 400);
