const state = {
  topics: [],
  groups: [],
  cameras: [],
  visualization: { range_m: 70, point_layers: [], boxes: [], markers: [] },
  system: {},
  gps: null,
  bag: { recording: false },
  vehicle: null,
  // ── bag 재생(Play) 관련: 항상 실시간 데이터와 분리된 슬롯 ──
  bagPlay: { playing: false, starting: false, file: null },
  gpsPlay: null,
  vehiclePlay: null,
  playNamespace: '/bagplay',
};

const defaults = ['/object_3d/front/result_image', '/object_3d/left/result_image', '/object_3d/rear/result_image', '/object_3d/right/result_image'];
const LAYOUT_NAMES = { balanced: 'Balanced', 'wide-viz': 'Wide RViz', 'camera-focus': 'Camera Focus', 'topic-focus': 'Topic Focus', custom: 'Custom' };

// 상단 헤더 항목 기본 구성 (이름/순서/표시 커스터마이즈 대상)
const HEADER_DEFAULTS = [
  { id: 'title',        label: 'ROS Web Monitor', visible: true },
  { id: 'layout',       label: 'Edit Layout',     visible: true },
  { id: 'viewmode',     label: '뷰 전환',          visible: true },
  { id: 'bag',          label: 'Bag',             visible: true },
  { id: 'bagplay',      label: 'Play',            visible: true },
  { id: 'reboot',       label: '⟳ 재부팅',         visible: true },
  { id: 'shutdown',     label: '⏻ 종료',           visible: true },
  { id: 'stat-topics',  label: 'topics',          visible: true },
  { id: 'stat-active',  label: 'active',          visible: true },
  { id: 'stat-cameras', label: 'cameras',         visible: true },
];
function loadHeaderConfig() {
  const saved = JSON.parse(localStorage.getItem('rosWebHeaderConfig') || 'null');
  if (!Array.isArray(saved)) return HEADER_DEFAULTS.map((x) => ({ ...x }));
  const byId = new Map(saved.map((s) => [s.id, s]));
  // 저장된 순서 유지 + 신규 기본 항목은 뒤에 보강
  const ordered = saved
    .filter((s) => HEADER_DEFAULTS.some((d) => d.id === s.id))
    .map((s) => ({ ...HEADER_DEFAULTS.find((d) => d.id === s.id), ...s }));
  HEADER_DEFAULTS.forEach((d) => { if (!byId.has(d.id)) ordered.push({ ...d }); });
  // title은 항상 맨 앞 고정
  ordered.sort((a, b) => (a.id === 'title' ? -1 : b.id === 'title' ? 1 : 0));
  return ordered;
}
const view = {
  zoom: 1,
  yaw: 0,
  pitch: 1.08,
  panX: 0,
  panY: 0,
  dragging: false,
  dragMode: 'rotate',
  lastX: 0,
  lastY: 0,
  layout: localStorage.getItem('rosWebLayout') || 'balanced',
  customLayout: JSON.parse(localStorage.getItem('rosWebCustomLayout') || 'null'),
  cameraLayout: (() => {
    const saved = JSON.parse(localStorage.getItem('rosWebCameraLayout') || 'null');
    if (saved && saved.slots) return saved;
    // 구버전 rosWebCameraSlots 마이그레이션
    const oldSlots = JSON.parse(localStorage.getItem('rosWebCameraSlots') || 'null') || defaults.slice();
    return { cols: 2, slots: oldSlots.map((t, i) => ({ id: 'c' + i, topic: t || '' })) };
  })(),
  cameraEditMode: false,
  cameraOptionSig: '',
  vizSettings: (() => {
    const s = JSON.parse(localStorage.getItem('rosWebVizSettings') || 'null') || {};
    return { maxPoints: s.maxPoints ?? 900, range: s.range ?? 120, defaultZoom: s.defaultZoom ?? 1 };
  })(),
  vizLayers: JSON.parse(localStorage.getItem('rosWebVizLayers') || 'null') || [
    { id: 'l1', name: 'LiDAR',     color: '#64c8ff', topic: '', visible: true },
    { id: 'l2', name: 'Radar',     color: '#f1bf58', topic: '', visible: true },
    { id: 'l3', name: '3D Object', color: '#f16e78', topic: '', visible: true },
    { id: 'l4', name: 'Track',     color: '#56d38b', topic: '', visible: true },
    { id: 'l5', name: 'Marker',    color: '#c791ff', topic: '', visible: true },
  ],
  bagConfig: JSON.parse(localStorage.getItem('rosWebBagConfig') || 'null') || { all: true, topics: [] },
  headerConfig: loadHeaderConfig(),
  sysSubTab: localStorage.getItem('rosWebSysSubTab') || 'all',
  sysTabConfig: JSON.parse(localStorage.getItem('rosWebSysTabConfig') || 'null') || [
    { id: 'all', label: 'All' }, { id: 'gpu', label: 'GPU' }, { id: 'cpu', label: 'CPU' },
    { id: 'mem', label: 'MEM' }, { id: 'eng', label: 'ENG' }, { id: 'ctrl', label: 'CTRL' },
    { id: 'info', label: 'INFO' },
  ],
  leftTab: localStorage.getItem('rosWebLeftTab') || 'topics',
  leftTabConfig: JSON.parse(localStorage.getItem('rosWebLeftTabConfig') || 'null') || [
    { id: 'topics', label: '토픽' },
    { id: 'groups', label: '센서' },
    { id: 'system', label: '시스템' },
  ],
  mode: localStorage.getItem('rosWebViewMode') || 'live',   // 'live' | 'play'
  editLayout: false,
  layoutBeforeEdit: null,
  panelOrderBeforeEdit: null,
  colWidthBeforeEdit: '',
  leftRowsBeforeEdit: '',
  rightRowsBeforeEdit: '',
};

const $ = (id) => document.querySelector(id);
const dashboard = $('#dashboard');
const topicCount = $('#topicCount');
const activeCount = $('#activeCount');
const cameraCount = $('#cameraCount');
const clock = $('#clock');
const shownCount = $('#shownCount');
const groupGrid = $('#groupGrid');
const systemGrid = $('#systemGrid');
const systemSubtitle = $('#systemSubtitle');
const cameraGrid = $('#cameraGrid');
const cameraStatus = $('#cameraStatus');
const topicRows = $('#topicRows');
const searchInput = $('#searchInput');
const groupFilter = $('#groupFilter');
const activeOnly = $('#activeOnly');
const vizCanvas = $('#vizCanvas');
const vizStats = $('#vizStats');
const vizCtx = vizCanvas.getContext('2d');
const zoomRange = $('#zoomRange');
const layoutMode = $('#layoutMode');
const layoutName = $('#layoutName');
const bagToggle = $('#bagToggle');
const bagStatus = $('#bagStatus');
const vehicleBody = $('#vehicleBody');
const bagPlayToggle = $('#bagPlayToggle');
const bagPlayStatus = $('#bagPlayStatus');
const playBanner = $('#playBanner');
const playBannerText = $('#playBannerText');

let bagBaseLabel = 'Bag';  // 헤더 설정에서 변경 가능한 Bag 버튼 기본 라벨
let bagPlayBaseLabel = 'Play';  // 헤더 설정에서 변경 가능한 Play 버튼 기본 라벨

/* ── 실시간 뷰 / Play 뷰 모드 헬퍼 ──
   bag 재생 시 모든 토픽은 항상 state.playNamespace(기본 '/bagplay') 하위로 리맵되어
   들어오므로, 프런트는 이 접두어만으로 실시간 데이터와 재생 데이터를 완전히 분리해서
   같은 뷰어(토픽표/RViz/카메라/센서그룹/차량/GPS)를 재사용해 그린다. */
function isPlayTopic(name) {
  const ns = state.playNamespace || '/bagplay';
  return typeof name === 'string' && name.startsWith(ns + '/');
}
function effectiveTopic(topic) {
  if (!topic) return topic;
  return view.mode === 'play' ? (state.playNamespace || '/bagplay') + topic : topic;
}
function currentModeTopics() {
  const inPlay = view.mode === 'play';
  return state.topics.filter((t) => isPlayTopic(t.name) === inPlay);
}
function currentModeCameras() {
  const inPlay = view.mode === 'play';
  return state.cameras.filter((c) => isPlayTopic(c.name) === inPlay);
}

// ── 우측 패널별 크기(너비%·높이%) — 편집 모드에서 그립으로 독립 조절 ──
const PANEL_SIZE_DEFAULTS = {
  rviz:    { w: 62, h: 56 },
  vehicle: { w: 35, h: 56 },
  camera:  { w: 62, h: 40 },
  gps:     { w: 35, h: 40 },
  objects: { w: 35, h: 34 },   // 감지 객체 목록
  events:  { w: 62, h: 34 },   // 이벤트 로그
};
function loadPanelSizes() {
  const saved = JSON.parse(localStorage.getItem('rosWebPanelSizes') || 'null') || {};
  const out = {};
  for (const k of Object.keys(PANEL_SIZE_DEFAULTS)) {
    const s = saved[k] || {};
    out[k] = {
      w: typeof s.w === 'number' ? s.w : PANEL_SIZE_DEFAULTS[k].w,
      h: typeof s.h === 'number' ? s.h : PANEL_SIZE_DEFAULTS[k].h,
    };
  }
  return out;
}
let panelSizes = loadPanelSizes();
function savePanelSizes() { localStorage.setItem('rosWebPanelSizes', JSON.stringify(panelSizes)); }
function applyPanelSizes() {
  const rs = document.querySelector('.right-stage');
  if (!rs) return;
  for (const name of Object.keys(PANEL_SIZE_DEFAULTS)) {
    const p = rs.querySelector(`.panel[data-panel="${name}"]`);
    if (!p) continue;
    const s = panelSizes[name];
    p.style.width = `calc(${s.w}% - 6px)`;
    p.style.height = `${s.h}%`;
  }
}

applyStoredLayout();
setLayout(view.layout, false);
setupPanelDragging();
applyPanelSizes();    // 저장된 패널 크기 적용
applyHeaderConfig();  // 저장된 헤더 구성(이름/순서/표시) 적용
view.zoom = view.vizSettings.defaultZoom || 1;  // 저장된 기본 확대값 적용
zoomRange.value = view.zoom.toFixed(2);

function groupForTopic(topic) {
  const ns = ((state.playNamespace || '/bagplay') + '/').toLowerCase();
  let name = topic.name.toLowerCase();
  // Play 네임스페이스 접두어는 분류 시엔 제거 (실시간과 동일하게 분류되도록)
  if (name.startsWith(ns)) name = name.slice(ns.length - 1);
  if (name.includes('camera') || name.includes('image')) return 'cameras';
  if (name.includes('ouster') || name.includes('lidar') || name.includes('point') || name.includes('scan')) return 'lidar';
  if (name.includes('radar')) return 'radar';
  if (name.includes('gps') || name.includes('geodetic') || name.includes('cov') || name.includes('euler')) return 'localization';
  if (name.includes('can') || name.startsWith('/acu')) return 'vehicle_can';
  if (name.includes('object') || name.includes('cluster') || name.includes('detection') || name.includes('track')) return 'perception';
  return name.replace(/^\//, '').split('/')[0] || 'root';
}

let lastHeavyRender = 0;       // 무거운 DOM 렌더(토픽표/시스템/그룹) 쓰로틀
let vizRafPending = false;     // 시각화는 requestAnimationFrame으로 코얼레싱

function scheduleViz() {
  if (vizRafPending) return;
  vizRafPending = true;
  requestAnimationFrame(() => { vizRafPending = false; renderVisualization(); });
}

function render(payload) {
  state.topics = payload.topics || [];
  state.groups = payload.groups || [];
  state.cameras = payload.cameras || [];
  state.visualization = payload.visualization || { range_m: 70, point_layers: [], boxes: [], markers: [] };
  state.system = payload.system || {};
  state.gps = payload.gps || null;
  state.gpsTrack = payload.gps_track || null;
  state.streamPort = payload.stream_port || null;
  state.streamPorts = payload.stream_ports || (payload.stream_port ? [payload.stream_port] : []);
  state.vehicle = payload.vehicle || null;
  state.gpsPlay = payload.gps_play || null;
  state.vehiclePlay = payload.vehicle_play || null;
  state.playNamespace = payload.play_namespace || '/bagplay';
  if (payload.bag) state.bag = payload.bag;
  state.bagPlay = payload.bag_play || { playing: false, starting: false, file: null };
  // 통계는 백엔드 전체 합계가 아니라 현재 뷰 모드(실시간/Play)에 해당하는 것만 집계
  // (bag 재생 중에도 라이브 카운트가 부풀지 않도록)
  const modeTopics = currentModeTopics();
  topicCount.textContent = modeTopics.length;
  activeCount.textContent = modeTopics.filter((t) => t.active).length;
  cameraCount.textContent = currentModeCameras().length;
  clock.textContent = payload.time || '';
  state.time = payload.time || '';

  // 빠른 갱신(매 메시지): 차량·GPS·카메라age·Bag — 가벼움
  renderVehicle();
  renderGps();
  renderGpsTrackBtn();
  updateGpsLiveLine();
  renderCameras();
  updateBagUI();
  updateBagPlayUI();
  renderProximityAlert();   // 근접 장애물 경고
  collectEvents();          // 이벤트 로그 수집(상태 전이)
  renderEvents();
  renderObjects();          // 감지 객체 목록
  scheduleViz();   // 시각화는 다음 애니메이션 프레임에 1회만

  // 시스템 스파크라인 히스토리는 탭과 무관하게 항상 축적
  const cpu = state.system.cpu || {}, mem = state.system.memory || {}, gpu = state.system.gpu || {};
  pushHistory(sysHistory.cpu, cpu.usage_percent || 0);
  pushHistory(sysHistory.gpu, gpu.usage_percent ?? 0);
  pushHistory(sysHistory.ram, mem.usage_percent || 0);

  // 무거운 갱신(토픽표/시스템/그룹)은 ~0.7s 마다 + 현재 보이는 탭만 → 버벅임 방지
  const now = (typeof performance !== 'undefined' ? performance.now() : Date.now());
  if (now - lastHeavyRender > 700) {
    lastHeavyRender = now;
    const tab = view.editLayout ? null : view.leftTab;   // 편집 중엔 전부
    if (!tab || tab === 'groups') renderGroups();
    if (!tab || tab === 'system') renderSystem();
    if (!tab || tab === 'topics') renderTopics();
  }
}

function updateBagUI() {
  const rec = !!(state.bag && state.bag.recording);
  bagToggle.textContent = (rec ? '■ ' : '● ') + (rec ? `${bagBaseLabel} Save` : bagBaseLabel);
  bagToggle.classList.toggle('recording', rec);
  bagStatus.hidden = !rec;
}

/* ════════════════════════════════════════════════════════════
   ROS2 bag 재생(Play) — ROS1/ROS2 bag 모두 지원.
   재생된 토픽은 항상 /bagplay/* 네임스페이스로 격리되어 실시간과 안 섞인다.
   ════════════════════════════════════════════════════════════ */
function updateBagPlayUI() {
  if (!bagPlayToggle) return;
  const bp = state.bagPlay || {};
  const active = !!(bp.playing || bp.starting);
  bagPlayToggle.textContent = bp.starting ? '⏳ 준비중' : (bp.playing ? `■ ${bagPlayBaseLabel} 중지` : `▶ ${bagPlayBaseLabel}`);
  bagPlayToggle.classList.toggle('playing', !!bp.playing);
  bagPlayToggle.classList.toggle('starting', !!bp.starting);
  if (bagPlayStatus) {
    bagPlayStatus.hidden = !active;
    bagPlayStatus.textContent = bp.file || '';
  }
  document.querySelectorAll('.viewmode-btn[data-mode="play"]').forEach((b) => b.classList.toggle('has-data', !!bp.playing));
  updatePlayBanner();
}

function fmtClock(sec) {
  sec = Math.max(0, Math.floor(sec || 0));
  const m = Math.floor(sec / 60), s = sec % 60;
  return `${m}:${String(s).padStart(2, '0')}`;
}

let playSeekDragging = false;  // 시크바 드래그 중엔 SSE 갱신이 값을 덮어쓰지 않게 함

function updatePlayBanner() {
  if (!playBanner) return;
  const bp = state.bagPlay || {};
  const active = !!(bp.playing || bp.starting);
  playBanner.hidden = !active;
  document.body.classList.toggle('has-play-banner', active);
  // 재생/일시정지 여부와 무관하게 파일명은 항상 노출, 상태만 접두어로 구분
  if (playBannerText) {
    playBannerText.textContent = bp.starting
      ? `bag 재생 준비 중… (${bp.file || ''})`
      : `${bp.paused ? '⏸ 일시정지' : '▶ 재생 중'}: ${bp.file || ''}`;
  }
  const hasTiming = bp.playing && typeof bp.duration_sec === 'number' && bp.duration_sec > 0 && bp.ctrl_available;
  const pauseBtn = document.getElementById('playPauseBtn');
  const nextBtn = document.getElementById('playNextBtn');
  const seekBar = document.getElementById('playSeekBar');
  const curEl = document.getElementById('playTimeCur');
  const durEl = document.getElementById('playTimeDur');
  const rateSel = document.getElementById('playRateSel');
  if (pauseBtn) {
    pauseBtn.hidden = !hasTiming;
    pauseBtn.textContent = bp.paused ? '▶' : '⏸';
  }
  if (nextBtn) nextBtn.hidden = !hasTiming || !bp.paused;
  if (seekBar) seekBar.hidden = !hasTiming;
  if (curEl) curEl.hidden = !hasTiming;
  if (durEl) durEl.hidden = !hasTiming;
  if (rateSel) rateSel.hidden = !hasTiming;
  if (!hasTiming) return;
  if (durEl) durEl.textContent = fmtClock(bp.duration_sec);
  if (!playSeekDragging) {
    if (curEl) curEl.textContent = fmtClock(bp.position_sec);
    if (seekBar) {
      seekBar.max = String(bp.duration_sec);
      seekBar.value = String(bp.position_sec || 0);
    }
  }
  if (rateSel && document.activeElement !== rateSel) rateSel.value = String(bp.rate || 1);
}

async function togglePlayPause() {
  const bp = state.bagPlay || {};
  const url = bp.paused ? '/api/bag/play/resume' : '/api/bag/play/pause';
  try {
    const res = await fetch(url, { method: 'POST' });
    const data = await res.json();
    if (data.ok === false) vdNotice('재생 제어 실패: ' + (data.error || '알 수 없는 오류'));
  } catch (e) {
    vdNotice('재생 제어 요청 실패: ' + e.message);
  }
}
async function playNextFrame() {
  try {
    const res = await fetch('/api/bag/play/next', { method: 'POST' });
    const data = await res.json();
    if (data.ok === false) vdNotice('다음 프레임 이동 실패: ' + (data.error || '알 수 없는 오류'));
  } catch (e) {
    vdNotice('다음 프레임 요청 실패: ' + e.message);
  }
}
async function seekBagPlay(sec) {
  try {
    const res = await fetch('/api/bag/play/seek', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ sec }),
    });
    const data = await res.json();
    if (!data.ok) vdNotice('이동 실패: ' + (data.error || '알 수 없는 오류'));
  } catch (e) {
    vdNotice('이동 요청 실패: ' + e.message);
  }
}
async function setPlayRate(rate) {
  try {
    const res = await fetch('/api/bag/play/rate', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ rate }),
    });
    const data = await res.json();
    if (!data.ok) vdNotice('배속 변경 실패: ' + (data.error || '알 수 없는 오류'));
  } catch (e) {
    vdNotice('배속 변경 요청 실패: ' + e.message);
  }
}

// 실시간 뷰 ⇄ Play 뷰 전환: 같은 패널(RViz/카메라/토픽표/센서그룹/차량/GPS)을 그대로 두고
// 데이터 소스만 /bagplay 네임스페이스 유무로 바꿔서 다시 그린다.
function setViewMode(mode) {
  view.mode = mode === 'play' ? 'play' : 'live';
  localStorage.setItem('rosWebViewMode', view.mode);
  document.querySelectorAll('.viewmode-btn').forEach((b) => b.classList.toggle('active', b.dataset.mode === view.mode));
  view.cameraOptionSig = '';  // 모드 전환 시 카메라 스트림 URL을 강제로 다시 매칭
  const modeTopics = currentModeTopics();
  topicCount.textContent = modeTopics.length;
  activeCount.textContent = modeTopics.filter((t) => t.active).length;
  cameraCount.textContent = currentModeCameras().length;
  renderVehicle();
  renderGps();
  renderGroups();
  renderTopics();
  renderCameras();
  renderVizLegend();
  renderVisualization();
  renderObjects();
  renderProximityAlert();
}

let playFilesCache = [];

async function openPlayModal() {
  document.getElementById('playOverlay').hidden = false;
  document.getElementById('playModal').hidden = false;
  updatePlayModalFoot();
  await refreshPlayFileList();
}
function closePlayModal() {
  document.getElementById('playOverlay').hidden = true;
  document.getElementById('playModal').hidden = true;
}
async function refreshPlayFileList() {
  const listEl = document.getElementById('playFileList');
  listEl.innerHTML = '<div class="bag-empty">불러오는 중…</div>';
  try {
    const res = await fetch('/api/bag/play/list', { cache: 'no-store' });
    const data = await res.json();
    if (!data.ok) { listEl.innerHTML = `<div class="bag-empty">${escapeHtml(data.error || '목록 조회 실패')}</div>`; return; }
    playFilesCache = data.files || [];
    renderPlayFileList();
  } catch (e) {
    listEl.innerHTML = `<div class="bag-empty">목록 조회 실패: ${escapeHtml(e.message)}</div>`;
  }
}
function renderPlayFileList() {
  const listEl = document.getElementById('playFileList');
  if (!playFilesCache.length) {
    listEl.innerHTML = '<div class="bag-empty">재생 가능한 bag이 없습니다 — 상단 Bag 버튼으로 먼저 녹화하거나 서버의 bag 폴더에 ROS1(.bag)/ROS2 bag을 넣어주세요</div>';
    return;
  }
  const bp = state.bagPlay || {};
  listEl.innerHTML = playFilesCache.map((f) => {
    const isPlaying = bp.playing && bp.file === f.name;
    return `<button type="button" class="play-file-item${isPlaying ? ' selected' : ''}" data-name="${escapeHtml(f.name)}">
      <span class="bag-grp play-file-type" style="--g:${f.type === 'ros1' ? '#f1bf58' : '#64c8ff'}">${f.type === 'ros1' ? 'ROS1' : 'ROS2'}</span>
      <span class="bag-tname play-file-name">${escapeHtml(f.name)}${isPlaying ? ' · ● 재생중' : ''}</span>
      <span class="play-file-meta">${f.size_kb ? f.size_kb + ' KB · ' : ''}${escapeHtml(f.modified || '')}</span>
    </button>`;
  }).join('');
  listEl.querySelectorAll('.play-file-item').forEach((btn) => {
    btn.addEventListener('click', () => startBagPlay(btn.dataset.name));
  });
}
async function startBagPlay(name) {
  const loopChk = document.getElementById('playLoopChk');
  const loop = !!(loopChk && loopChk.checked);
  try {
    const res = await fetch('/api/bag/play/start', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, loop }),
    });
    const data = await res.json();
    if (!data.ok) { vdNotice('재생 실패: ' + (data.error || '알 수 없는 오류')); return; }
    state.bagPlay = { playing: true, starting: false, file: data.file };
    updateBagPlayUI();
    setViewMode('play');
    closePlayModal();
  } catch (e) {
    vdNotice('재생 요청 실패: ' + e.message);
  }
}
async function stopBagPlay() {
  try {
    const res = await fetch('/api/bag/play/stop', { method: 'POST' });
    const data = await res.json();
    if (data.ok === false) vdNotice('중지 실패: ' + (data.error || '알 수 없는 오류'));
  } catch (e) {
    vdNotice('중지 요청 실패: ' + e.message);
  } finally {
    state.bagPlay = { playing: false, starting: false, file: null };
    updateBagPlayUI();
    updatePlayModalFoot();
    renderPlayFileList();
  }
}
function updatePlayModalFoot() {
  const stopBtn = document.getElementById('playStopBtn');
  const label = document.getElementById('playStatusLabel');
  const bp = state.bagPlay || {};
  if (stopBtn) stopBtn.hidden = !bp.playing;
  if (label) label.textContent = bp.playing ? `재생 중: ${bp.file || ''}` : (bp.starting ? '준비 중…' : '');
}

// ── 상단 헤더 구성 적용 (순서/표시/이름) ──
function applyHeaderConfig() {
  const cfg = view.headerConfig || HEADER_DEFAULTS;
  cfg.forEach((item, idx) => {
    const el = document.querySelector(`[data-hitem="${item.id}"]`);
    if (!el) return;
    if (item.id !== 'title') el.style.order = String(idx);  // title은 항상 왼쪽 고정
    el.style.display = item.visible ? '' : 'none';
    applyHeaderLabel(item.id, item.label);
  });
}

function applyHeaderLabel(id, label) {
  if (id === 'title') {
    const t = document.getElementById('appTitle'); if (t) t.textContent = label;
  } else if (id === 'layout') {
    const b = document.getElementById('editLayout'); if (b) b.textContent = label;
  } else if (id === 'bag') {
    bagBaseLabel = label || 'Bag'; updateBagUI();
  } else if (id === 'bagplay') {
    bagPlayBaseLabel = label || 'Play'; updateBagPlayUI();
  } else if (id === 'reboot') {
    const b = document.getElementById('btnReboot'); if (b) b.textContent = label;
  } else if (id === 'shutdown') {
    const b = document.getElementById('btnShutdown'); if (b) b.textContent = label;
  } else if (id.startsWith('stat-')) {
    const el = document.querySelector(`[data-hitem="${id}"] span`); if (el) el.textContent = label;
  }
}

async function toggleBag() {
  const rec = !!(state.bag && state.bag.recording);
  bagToggle.disabled = true;
  try {
    let res;
    if (rec) {
      res = await fetch('/api/bag/stop', { method: 'POST' });
    } else {
      const cfg = view.bagConfig || { all: true, topics: [] };
      res = await fetch('/api/bag/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ all: !!cfg.all, topics: cfg.all ? [] : (cfg.topics || []) }),
      });
    }
    const data = await res.json();
    if (data.ok === false) {
      alert('Bag 오류: ' + (data.error || '알 수 없음'));
    } else if (rec) {
      state.bag = { recording: false, last_saved: data.path };
      const cnt = data.mcap_count;
      if (data.path && cnt === 0) {
        alert('⚠ Bag 저장됨 — 그러나 .mcap 파일이 없습니다 (수신된 메시지 없음 또는 토픽 미선택):\n' + data.path);
      } else if (data.path) {
        alert(`Bag 저장 완료 (mcap ${cnt}개):\n` + data.path);
      }
    } else {
      state.bag = { recording: true, start_time: data.start_time };
    }
    updateBagUI();
  } catch (e) {
    alert('Bag 요청 실패: ' + e.message);
  } finally {
    bagToggle.disabled = false;
  }
}

let bagEdit = null;
let headerEdit = null;
let hiDragId = null;

const BAG_GROUP_META = {
  cameras:      { label: 'CAM',   color: '#f16e78' },
  lidar:        { label: 'LiDAR', color: '#64c8ff' },
  radar:        { label: 'RADAR', color: '#f1bf58' },
  localization: { label: 'LOC',   color: '#56d38b' },
  vehicle_can:  { label: 'CAN',   color: '#c791ff' },
  perception:   { label: 'PERC',  color: '#ff9f6e' },
};
function bagGroupMeta(name) {
  const g = groupForTopic({ name });
  return BAG_GROUP_META[g] || { label: 'ETC', color: '#6b7a88' };
}

function openSettingsModal() {
  // Bag 탭 상태
  const recording = !!(state.bag && state.bag.recording);
  const bagBusy = recording || !!(state.bagPlay && (state.bagPlay.playing || state.bagPlay.starting));
  bagEdit = { all: !!view.bagConfig.all, topics: new Set(view.bagConfig.topics || []) };
  document.getElementById('bagAll').checked = bagEdit.all;
  document.getElementById('bagAll').disabled = recording;
  document.getElementById('bagTopicSearch').value = '';
  document.getElementById('bagPicker').classList.toggle('disabled', bagEdit.all || recording);
  renderBagTopicList('');
  const dirInput = document.getElementById('bagDirInput');
  const dirHint = document.getElementById('bagDirHint');
  if (dirInput) {
    dirInput.value = (state.bag && state.bag.dir) || '';
    dirInput.disabled = bagBusy;
    dirInput.classList.remove('invalid');
  }
  const dirBrowseBtnEl = document.getElementById('bagDirBrowseBtn');
  if (dirBrowseBtnEl) dirBrowseBtnEl.disabled = bagBusy;
  if (dirHint) {
    dirHint.classList.remove('bag-dir-error', 'bag-dir-ok');
    dirHint.textContent = bagBusy
      ? '녹화·재생 중에는 저장 경로를 변경할 수 없습니다.'
      : '녹화 파일과 재생 목록이 저장되는 폴더입니다. 절대 경로로 입력하세요.';
  }
  // 화면 구성 탭 상태
  headerEdit = view.headerConfig.map((x) => ({ ...x }));
  renderHeaderItemList();
  // RViz 탭 상태 (통합)
  vsmEdit = {
    vizSettings: { ...view.vizSettings },
    vizLayers: view.vizLayers.map((l) => ({ ...l })),
  };
  document.getElementById('vizMaxPoints').value = String(view.vizSettings.maxPoints);
  document.getElementById('vizRange').value = String(view.vizSettings.range);
  document.getElementById('vizDefaultZoom').value = String(view.vizSettings.defaultZoom);
  renderVsmLayerConfig();
  openDriveSettingsEdit();   // 주행 뷰 섹션
  // 백업 탭 상태
  renderBackupInfo();
  switchSettingsTab('header');
  document.getElementById('bagOverlay').hidden = false;
  document.getElementById('bagModal').hidden = false;
}

function switchSettingsTab(tab) {
  document.querySelectorAll('.set-tab').forEach((b) => b.classList.toggle('active', b.dataset.settab === tab));
  document.getElementById('setPanelHeader').hidden = tab !== 'header';
  document.getElementById('setPanelBag').hidden = tab !== 'bag';
  document.getElementById('setPanelRviz').hidden = tab !== 'rviz';
  document.getElementById('setPanelBackup').hidden = tab !== 'backup';
  document.getElementById('bagSelCount').style.visibility = tab === 'bag' ? '' : 'hidden';
}

// ── 화면 구성: 헤더 항목 편집 리스트 ──
function renderHeaderItemList() {
  const list = document.getElementById('headerItemList');
  list.innerHTML = headerEdit.map((it) => {
    const fixed = it.id === 'title';
    return `<div class="hi-row${fixed ? ' hi-row-fixed' : ''}" data-id="${it.id}">
      <span class="hi-drag${fixed ? ' hi-fixed' : ''}" title="${fixed ? '위치 고정' : '드래그로 순서 변경'}">${fixed ? '•' : '⠿'}</span>
      <label class="hi-vis-wrap" title="표시/숨김"><input type="checkbox" class="hi-vis"${it.visible ? ' checked' : ''}><span></span></label>
      <input type="text" class="hi-label" value="${escapeHtml(it.label)}" maxlength="24" placeholder="이름">
      <span class="hi-id">${it.id}</span>
    </div>`;
  }).join('');
  list.querySelectorAll('.hi-row').forEach((row) => {
    const id = row.dataset.id;
    const it = headerEdit.find((x) => x.id === id);
    row.querySelector('.hi-vis').addEventListener('change', (e) => { it.visible = e.target.checked; });
    row.querySelector('.hi-label').addEventListener('input', (e) => { it.label = e.target.value; });
    const handle = row.querySelector('.hi-drag:not(.hi-fixed)');
    if (handle) bindHeaderItemDrag(handle, row, id);
  });
}

function bindHeaderItemDrag(handle, row, id) {
  handle.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    hiDragId = id;
    row.classList.add('hi-dragging');
    handle.setPointerCapture(e.pointerId);
    let dropId = null;
    const onMove = (ev) => {
      const el = document.elementFromPoint(ev.clientX, ev.clientY);
      const over = el && el.closest('.hi-row');
      document.querySelectorAll('.hi-row').forEach((r) => r.classList.remove('hi-over'));
      if (over && over.dataset.id !== id && over.dataset.id !== 'title') {
        over.classList.add('hi-over'); dropId = over.dataset.id;
      } else dropId = null;
    };
    const onUp = () => {
      handle.releasePointerCapture(e.pointerId);
      handle.removeEventListener('pointermove', onMove);
      handle.removeEventListener('pointerup', onUp);
      if (dropId) {
        const fi = headerEdit.findIndex((x) => x.id === id);
        const ti = headerEdit.findIndex((x) => x.id === dropId);
        if (fi >= 0 && ti >= 0) { const [m] = headerEdit.splice(fi, 1); headerEdit.splice(ti, 0, m); }
      }
      hiDragId = null;
      renderHeaderItemList();
    };
    handle.addEventListener('pointermove', onMove);
    handle.addEventListener('pointerup', onUp);
  });
}

function renderBagTopicList(query) {
  const list = document.getElementById('bagTopicList');
  const q = (query || '').trim().toLowerCase();
  // 재생(bag play) 중인 /bagplay/* 토픽은 녹화 대상에서 제외 (실시간 데이터만 녹화)
  const names = state.topics.map((t) => t.name).filter((n) => !isPlayTopic(n) && (!q || n.toLowerCase().includes(q))).sort();
  if (!names.length) {
    list.innerHTML = '<div class="bag-empty">표시할 토픽이 없습니다</div>';
    updateBagSelCount();
    return;
  }
  list.innerHTML = names.map((n) => {
    const m = bagGroupMeta(n);
    const sel = bagEdit.topics.has(n);
    return `<button type="button" class="bag-topic-item${sel ? ' selected' : ''}" data-topic="${escapeHtml(n)}"${bagEdit.all ? ' disabled' : ''}>
      <span class="bag-check"></span>
      <span class="bag-grp" style="--g:${m.color}">${m.label}</span>
      <span class="bag-tname">${escapeHtml(n)}</span>
    </button>`;
  }).join('');
  list.querySelectorAll('.bag-topic-item').forEach((item) => {
    item.addEventListener('click', () => {
      if (bagEdit.all) return;
      const n = item.dataset.topic;
      if (bagEdit.topics.has(n)) { bagEdit.topics.delete(n); item.classList.remove('selected'); }
      else { bagEdit.topics.add(n); item.classList.add('selected'); }
      updateBagSelCount();
    });
  });
  updateBagSelCount();
}

function updateBagSelCount() {
  const el = document.getElementById('bagSelCount');
  const total = state.topics.length;
  el.textContent = bagEdit.all
    ? `전체 토픽 녹화 (-a · ${total}개)`
    : `${bagEdit.topics.size} / ${total}개 선택됨`;
}

// Bag 저장 경로 변경 요청 (녹화/재생 중 아닐 때만 유효). 실패 시 알림 표시.
async function saveBagDir() {
  const input = document.getElementById('bagDirInput');
  if (!input || input.disabled) return;
  const newDir = input.value.trim();
  const curDir = (state.bag && state.bag.dir) || '';
  if (!newDir || newDir === curDir) return;
  try {
    const res = await fetch('/api/bag/config', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ dir: newDir }),
    });
    const data = await res.json();
    if (!data.ok) {
      vdNotice('Bag 저장 경로 변경 실패:\n' + (data.error || '알 수 없는 오류'));
      return;
    }
    state.bag = { ...(state.bag || {}), dir: data.dir };
  } catch (e) {
    vdNotice('Bag 저장 경로 변경 요청 실패: ' + e.message);
  }
}

/* ── Bag 저장 경로 "찾아보기" — 서버 파일시스템 폴더 선택기 ── */
let dirBrowsePath = '';

function openDirBrowser() {
  document.getElementById('dirBrowseOverlay').hidden = false;
  document.getElementById('dirBrowseModal').hidden = false;
  const dirInput = document.getElementById('bagDirInput');
  const start = (dirInput && dirInput.value.trim()) || (state.bag && state.bag.dir) || '/home/nvidia';
  loadDirBrowse(start);
}
function closeDirBrowser() {
  document.getElementById('dirBrowseOverlay').hidden = true;
  document.getElementById('dirBrowseModal').hidden = true;
}
async function loadDirBrowse(path) {
  const listEl = document.getElementById('dirBrowseList');
  const pathEl = document.getElementById('dirBrowsePath');
  const selectBtn = document.getElementById('dirBrowseSelectBtn');
  const warnEl = document.getElementById('dirBrowseWritable');
  listEl.innerHTML = '<div class="bag-empty">불러오는 중…</div>';
  try {
    const res = await fetch('/api/fs/list?path=' + encodeURIComponent(path), { cache: 'no-store' });
    const data = await res.json();
    if (!data.ok) {
      listEl.innerHTML = `<div class="bag-empty">${escapeHtml(data.error || '조회 실패')}</div>`;
      return;
    }
    dirBrowsePath = data.path;
    pathEl.textContent = data.path;
    pathEl.title = data.path;
    const upRow = data.parent
      ? `<button type="button" class="dir-browse-item dir-browse-up" data-path="${escapeHtml(data.parent)}">⤴ 상위 폴더</button>`
      : '';
    const rows = (data.dirs || []).map((name) => {
      const full = data.path.endsWith('/') ? data.path + name : data.path + '/' + name;
      return `<button type="button" class="dir-browse-item" data-path="${escapeHtml(full)}"><span class="dir-ico">📁</span>${escapeHtml(name)}</button>`;
    }).join('');
    listEl.innerHTML = (upRow + rows) || '<div class="bag-empty">하위 폴더가 없습니다</div>';
    listEl.querySelectorAll('.dir-browse-item').forEach((btn) => {
      btn.addEventListener('click', () => loadDirBrowse(btn.dataset.path));
    });
    if (selectBtn) selectBtn.disabled = !data.writable;
    if (warnEl) warnEl.textContent = data.writable ? '' : '⚠ 이 폴더는 쓰기 권한이 없습니다';
  } catch (e) {
    listEl.innerHTML = `<div class="bag-empty">조회 실패: ${escapeHtml(e.message)}</div>`;
  }
}
async function createDirBrowseFolder() {
  const input = document.getElementById('dirBrowseNewName');
  const name = (input.value || '').trim();
  if (!name) return;
  try {
    const res = await fetch('/api/fs/mkdir', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ path: dirBrowsePath, name }),
    });
    const data = await res.json();
    if (!data.ok) { vdNotice('폴더 생성 실패: ' + (data.error || '알 수 없는 오류')); return; }
    input.value = '';
    dirBrowsePath = data.path;
    await loadDirBrowse(data.path);
  } catch (e) {
    vdNotice('폴더 생성 요청 실패: ' + e.message);
  }
}

function closeSettingsModal(save) {
  document.getElementById('bagOverlay').hidden = true;
  document.getElementById('bagModal').hidden = true;
  if (save) {
    if (bagEdit) {
      view.bagConfig = { all: bagEdit.all, topics: [...bagEdit.topics] };
      localStorage.setItem('rosWebBagConfig', JSON.stringify(view.bagConfig));
      saveBagDir();
    }
    if (headerEdit) {
      view.headerConfig = headerEdit.map((x) => ({ ...x }));
      localStorage.setItem('rosWebHeaderConfig', JSON.stringify(view.headerConfig));
      applyHeaderConfig();
    }
    if (vsmEdit) {
      view.vizSettings.maxPoints = parseInt(document.getElementById('vizMaxPoints').value, 10) || 900;
      view.vizSettings.range = parseInt(document.getElementById('vizRange').value, 10) || 120;
      const dz = parseFloat(document.getElementById('vizDefaultZoom').value);
      view.vizSettings.defaultZoom = (isFinite(dz) && dz > 0) ? Math.max(0.45, Math.min(30.0, dz)) : 1;
      view.vizLayers = vsmEdit.vizLayers.map((l) => ({ ...l }));
      localStorage.setItem('rosWebVizSettings', JSON.stringify(view.vizSettings));
      localStorage.setItem('rosWebVizLayers', JSON.stringify(view.vizLayers));
      state.visualization.range_m = view.vizSettings.range;
      postVizConfig(view.vizSettings);
      renderVizLegend();
      renderVisualization();
    }
  }
  commitDriveSettingsEdit(save);
  bagEdit = null;
  headerEdit = null;
  vsmEdit = null;
}

function renderVisualization() {
  if (view.vizTab && view.vizTab !== 'rviz') { renderDriveView(); return; }
  const viz = state.visualization;
  const rect = vizCanvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(320, Math.floor(rect.width));
  const height = Math.max(320, Math.floor(rect.height));
  if (vizCanvas.width !== Math.floor(width * dpr) || vizCanvas.height !== Math.floor(height * dpr)) {
    vizCanvas.width = Math.floor(width * dpr);
    vizCanvas.height = Math.floor(height * dpr);
  }
  vizCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  vizCtx.clearRect(0, 0, width, height);
  drawGrid3D(width, height);

  const topicToLayer = {};
  for (const lyr of view.vizLayers) {
    // Play 뷰에서는 /bagplay/* 로 리맵된 토픽만 매칭되어 실시간 데이터와 자동으로 분리됨
    if (lyr.topic && lyr.visible) topicToLayer[effectiveTopic(lyr.topic)] = lyr;
  }

  // 1) 포인트: 깊이 정렬 없이 레이어별로 한 번에 그림(점은 z-order 무의미) — 가장 큰 최적화
  let pointCount = 0;
  for (const layer of viz.point_layers || []) {
    const lyr = topicToLayer[layer.topic];
    if (!lyr) continue;
    vizCtx.fillStyle = lyr.color;
    for (const p of layer.points || []) {
      const pr = project3D(p[0], p[1], p[2] || 0, width, height);
      if (!pr.visible) continue;
      const size = Math.max(1, Math.min(3, pr.scale * 0.018));
      vizCtx.fillRect(pr.x, pr.y, size, size);
      pointCount += 1;
    }
  }

  // 2) 박스/마커만 깊이 정렬(소수) 후 그림
  const items = [];
  for (const marker of viz.markers || []) {
    const lyr = topicToLayer[marker.topic];
    if (lyr) items.push({ depth: 0, type: 'marker', marker, color: lyr.color });
  }
  for (const box of viz.boxes || []) {
    const lyr = topicToLayer[box.topic];
    if (!lyr) continue;
    const center = project3D(box.x || 0, box.y || 0, box.z || 0, width, height);
    if (center.visible) items.push({ depth: center.depth, type: 'box', box, color: lyr.color });
  }
  items.sort((a, b) => b.depth - a.depth);
  let boxes = 0;
  let markers = 0;
  for (const item of items) {
    if (item.type === 'marker') { markers += draw3DMarker(item.marker, item.color, width, height); }
    else if (item.type === 'box') { boxes += draw3DBox(item.box, item.color, width, height); }
  }
  drawEgo3D(width, height);
  vizStats.textContent = `${pointCount} points, ${boxes} boxes, ${markers} markers, yaw ${(view.yaw * 180 / Math.PI).toFixed(0)} deg`;
}

function project3D(x, y, z, width, height) {
  const cy = Math.cos(view.yaw), sy = Math.sin(view.yaw);
  const forward = cy * x - sy * y;
  const side = sy * x + cy * y;
  const sp = Math.sin(view.pitch), cp = Math.cos(view.pitch);
  const screenYWorld = forward * sp + z * cp;
  const depthWorld = forward * cp - z * sp;
  const distance = 170 / view.zoom;
  const depth = distance + depthWorld;
  const scale = Math.min(width, height) * 1.05 / Math.max(20, depth);
  return { x: width / 2 - side * scale + view.panX, y: height * 0.55 - screenYWorld * scale + view.panY, scale, depth, visible: depth > 4 };
}

function drawGrid3D(width, height) {
  vizCtx.fillStyle = '#07090b';
  vizCtx.fillRect(0, 0, width, height);
  const range = Math.max(30, Math.min(200, state.visualization.range_m || 80));
  const step = range <= 50 ? 5 : range <= 120 ? 10 : 20;
  vizCtx.lineWidth = 1;
  for (let v = -range; v <= range; v += step) {
    const major = v === 0;
    vizCtx.strokeStyle = major ? '#42505c' : '#1e2830';
    drawWorldLine(-range, v, 0, range, v, 0, width, height);
    drawWorldLine(v, -range, 0, v, range, 0, width, height);
  }
  vizCtx.fillStyle = '#81909e';
  vizCtx.font = '12px system-ui, sans-serif';
  drawAxis(width, height);
  vizCtx.fillText('드래그: 회전 · 휠클릭+드래그: 이동 · 휠: 확대', 12, height - 14);
}

function drawAxis(width, height) {
  const axes = [
    [[0, 0, 0], [12, 0, 0], '#f16e78', 'X'],
    [[0, 0, 0], [0, 12, 0], '#56d38b', 'Y'],
    [[0, 0, 0], [0, 0, 5], '#64c8ff', 'Z'],
  ];
  for (const [a, b, color, label] of axes) {
    const pa = project3D(a[0], a[1], a[2], width, height);
    const pb = project3D(b[0], b[1], b[2], width, height);
    if (!pa.visible || !pb.visible) continue;
    vizCtx.strokeStyle = color;
    vizCtx.lineWidth = 2;
    vizCtx.beginPath();
    vizCtx.moveTo(pa.x, pa.y);
    vizCtx.lineTo(pb.x, pb.y);
    vizCtx.stroke();
    vizCtx.fillStyle = color;
    vizCtx.fillText(label, pb.x + 4, pb.y + 4);
  }
}

function drawWorldLine(x1, y1, z1, x2, y2, z2, width, height) {
  const a = project3D(x1, y1, z1, width, height);
  const b = project3D(x2, y2, z2, width, height);
  if (!a.visible || !b.visible) return;
  vizCtx.beginPath();
  vizCtx.moveTo(a.x, a.y);
  vizCtx.lineTo(b.x, b.y);
  vizCtx.stroke();
}

function draw3DPoint(item) {
  vizCtx.fillStyle = item.color;
  const size = Math.max(1, Math.min(3, item.p.scale * 0.018));
  vizCtx.fillRect(item.p.x, item.p.y, size, size);
}

function draw3DMarker(marker, color, width, height) {
  const points = marker.points || [];
  if (!points.length) return 0;
  vizCtx.strokeStyle = color;
  vizCtx.fillStyle = color;
  vizCtx.lineWidth = 1.6;
  if ([4, 5].includes(marker.type)) {
    vizCtx.beginPath();
    let started = false;
    points.forEach((point, index) => {
      const p = project3D(point[0], point[1], point[2] || 0, width, height);
      if (!p.visible) return;
      if (!started || marker.type === 5 && index % 2 === 0) { vizCtx.moveTo(p.x, p.y); started = true; }
      else vizCtx.lineTo(p.x, p.y);
    });
    vizCtx.stroke();
    return 1;
  }
  let count = 0;
  // radar 객체 포인트는 점이 너무 많지 않을 때 각 점에 거리 표시
  const showDist = marker.kind === 'radar' && points.length <= 80;
  for (const point of points) {
    const p = project3D(point[0], point[1], point[2] || 0, width, height);
    if (!p.visible) continue;
    vizCtx.beginPath();
    vizCtx.arc(p.x, p.y, Math.max(2, Math.min(5, p.scale * 0.03)), 0, Math.PI * 2);
    vizCtx.fill();
    if (showDist) {
      const d = Math.hypot(point[0], point[1]);
      vizCtx.font = '9px system-ui, sans-serif';
      const text = `${d.toFixed(0)}m`;
      vizCtx.lineWidth = 2.5;
      vizCtx.strokeStyle = 'rgba(0,0,0,0.75)';
      vizCtx.strokeText(text, p.x + 4, p.y - 3);
      vizCtx.fillStyle = color;
      vizCtx.fillText(text, p.x + 4, p.y - 3);
    }
    count += 1;
  }
  return count;
}

function draw3DBox(box, color, width, height) {
  const sx = Math.max(box.sx || 1, 0.4) / 2;
  const sy = Math.max(box.sy || 1, 0.4) / 2;
  const sz = Math.max(box.sz || 1, 0.4) / 2;
  const yaw = box.yaw || 0;
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const corners = [];
  for (const dx of [-sx, sx]) for (const dy of [-sy, sy]) for (const dz of [-sz, sz]) {
    const rx = c * dx - s * dy;
    const ry = s * dx + c * dy;
    corners.push(project3D((box.x || 0) + rx, (box.y || 0) + ry, (box.z || 0) + dz, width, height));
  }
  const edges = [[0,1],[0,2],[0,4],[3,1],[3,2],[3,7],[5,1],[5,4],[5,7],[6,2],[6,4],[6,7]];
  vizCtx.strokeStyle = color;
  vizCtx.lineWidth = box.kind === 'radar' ? 1.7 : 2.2;
  for (const [a, b] of edges) {
    if (!corners[a].visible || !corners[b].visible) continue;
    vizCtx.beginPath();
    vizCtx.moveTo(corners[a].x, corners[a].y);
    vizCtx.lineTo(corners[b].x, corners[b].y);
    vizCtx.stroke();
  }
  // 라벨 + 거리 표시 (거리는 항상 표시 — cluster/tracker/radar 박스)
  let top = null;
  for (const cnr of corners) { if (cnr.visible && (!top || cnr.y < top.y)) top = cnr; }
  if (top) {
    const dist = Math.hypot(box.x || 0, box.y || 0);
    const lbl = box.label ? String(box.label).slice(0, 16) + ' · ' : '';
    vizCtx.font = 'bold 11px system-ui, sans-serif';
    const text = `${lbl}${dist.toFixed(1)}m`;
    // 가독성을 위해 어두운 외곽선 후 색상 텍스트
    vizCtx.lineWidth = 3;
    vizCtx.strokeStyle = 'rgba(0,0,0,0.75)';
    vizCtx.strokeText(text, top.x + 4, top.y - 5);
    vizCtx.fillStyle = color;
    vizCtx.fillText(text, top.x + 4, top.y - 5);
  }
  return 1;
}

function drawEgo3D(width, height) {
  const pts = [[-3,0,0.3],[2,1.2,0.2],[1,0,1.1],[2,-1.2,0.2]].map((p) => project3D(p[0], p[1], p[2], width, height));
  if (pts.some((p) => !p.visible)) return;
  vizCtx.fillStyle = '#f3f6f8';
  vizCtx.strokeStyle = '#080a0c';
  vizCtx.lineWidth = 2;
  vizCtx.beginPath();
  vizCtx.moveTo(pts[0].x, pts[0].y);
  for (const p of pts.slice(1)) vizCtx.lineTo(p.x, p.y);
  vizCtx.closePath();
  vizCtx.fill();
  vizCtx.stroke();
}

async function postVizConfig(settings) {
  try {
    await fetch('/api/viz-config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ maxPoints: settings.maxPoints, range: settings.range }),
    });
  } catch (_) {}
}

const VIZ_TOPIC_TYPES = new Set([
  'sensor_msgs/msg/PointCloud2',
  'visualization_msgs/msg/MarkerArray',
  'visualization_msgs/msg/Marker',
  'vision_msgs/msg/Detection3DArray',
]);

let vsmEdit = null;

// RViz 설정은 상단 통합 설정 모달의 'RViz' 탭으로 통합됨
function openVizSettingsModal() {
  openSettingsModal();
  switchSettingsTab('rviz');
}

function collectVizTopics() {
  // 레이어 설정은 항상 논리(실시간) 토픽 이름 기준 — Play 뷰에서는 effectiveTopic()으로 자동 매칭됨
  const all = new Set();
  for (const t of state.topics) { if (!isPlayTopic(t.name) && VIZ_TOPIC_TYPES.has(t.type)) all.add(t.name); }
  for (const l of (state.visualization.point_layers || [])) { if (!isPlayTopic(l.topic)) all.add(l.topic); }
  for (const b of (state.visualization.boxes || [])) { if (!isPlayTopic(b.topic)) all.add(b.topic); }
  for (const m of (state.visualization.markers || [])) { if (!isPlayTopic(m.topic)) all.add(m.topic); }
  return [...all].sort();
}

function renderVsmLayerConfig() {
  if (!vsmEdit) return;
  const container = document.getElementById('vsmLayerConfig');
  if (!container) return;
  const allTopics = collectVizTopics();
  // include topics already assigned in layers even if not in state yet
  for (const lyr of vsmEdit.vizLayers) { if (lyr.topic && !allTopics.includes(lyr.topic)) allTopics.push(lyr.topic); }

  container.innerHTML = vsmEdit.vizLayers.map((lyr) => {
    const opts = allTopics.map((t) => `<option value="${escapeHtml(t)}"${lyr.topic === t ? ' selected' : ''}>${escapeHtml(t)}</option>`).join('');
    return `<div class="vsm-layer-row" data-id="${escapeHtml(lyr.id)}">
      <input type="color" class="vsm-lyr-color" value="${escapeHtml(lyr.color)}" title="색상">
      <input type="text" class="vsm-lyr-name" value="${escapeHtml(lyr.name)}" placeholder="레이어 이름" maxlength="24">
      <select class="vsm-lyr-topic"><option value="">— topic 없음 —</option>${opts}</select>
      <label class="vsm-vis-toggle" title="표시/숨김">
        <input type="checkbox" class="vsm-lyr-vis"${lyr.visible ? ' checked' : ''}>
        <span class="vsm-vis-label">${lyr.visible ? '표시' : '숨김'}</span>
      </label>
      <button class="vsm-lyr-del" type="button" title="삭제">×</button>
    </div>`;
  }).join('') + `<div class="vsm-add-layer-row"><button id="vsmAddLayer" type="button">＋ 레이어 추가</button></div>`;

  container.querySelectorAll('.vsm-layer-row').forEach((row) => {
    const lyr = vsmEdit.vizLayers.find((l) => l.id === row.dataset.id);
    if (!lyr) return;
    row.querySelector('.vsm-lyr-color').addEventListener('input', (e) => { lyr.color = e.target.value; });
    row.querySelector('.vsm-lyr-name').addEventListener('input', (e) => { lyr.name = e.target.value; });
    row.querySelector('.vsm-lyr-topic').addEventListener('change', (e) => { lyr.topic = e.target.value; });
    const visCb = row.querySelector('.vsm-lyr-vis');
    visCb.addEventListener('change', () => {
      lyr.visible = visCb.checked;
      const label = row.querySelector('.vsm-vis-label');
      if (label) label.textContent = visCb.checked ? '표시' : '숨김';
    });
    row.querySelector('.vsm-lyr-del').addEventListener('click', () => {
      vsmEdit.vizLayers = vsmEdit.vizLayers.filter((l) => l.id !== lyr.id);
      renderVsmLayerConfig();
    });
  });
  const addBtn = document.getElementById('vsmAddLayer');
  if (addBtn) {
    addBtn.addEventListener('click', () => {
      vsmEdit.vizLayers.push({ id: 'l' + Date.now(), name: '새 레이어', color: '#aaaaaa', topic: '', visible: true });
      renderVsmLayerConfig();
    });
  }
}

function renderVizLegend() {
  const legend = document.querySelector('.viz-legend');
  if (!legend) return;
  const withTopic = view.vizLayers.filter((l) => l.topic);
  if (!withTopic.length) {
    legend.innerHTML = '<span style="color:var(--muted)">레이어 미설정 — ⚙ 눌러서 토픽 지정</span>';
    return;
  }
  legend.innerHTML = withTopic.map((l) =>
    `<button type="button" class="viz-legend-item${l.visible ? '' : ' off'}" data-layer="${escapeHtml(l.id)}" title="클릭하여 표시 on/off">
       <i class="viz-dot" style="background:${l.color}"></i>${escapeHtml(l.name)}
     </button>`).join('');
  legend.querySelectorAll('.viz-legend-item').forEach((btn) => {
    btn.addEventListener('click', () => {
      const lyr = view.vizLayers.find((x) => x.id === btn.dataset.layer);
      if (!lyr) return;
      lyr.visible = !lyr.visible;
      localStorage.setItem('rosWebVizLayers', JSON.stringify(view.vizLayers));
      renderVizLegend();
      renderVisualization();
    });
  });
}

let gpsMap = null;
let gpsMarker = null;
let gpsFollow = true;  // true면 현재 위치 자동 추적, 사용자가 지도를 끌면 false

function initGpsMap(lat, lng) {
  if (gpsMap) return;
  const mapEl = document.getElementById('gpsMap');
  if (!mapEl || typeof L === 'undefined') return;
  gpsMap = L.map(mapEl, { zoomControl: true, attributionControl: false }).setView([lat, lng], 16);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(gpsMap);
  gpsMarker = L.circleMarker([lat, lng], {
    radius: 9, fillColor: '#64c8ff', color: '#ffffff', weight: 2, opacity: 1, fillOpacity: 0.9,
  }).addTo(gpsMap);
  // 사용자가 지도를 드래그하면 자동 추적 해제 ('현재 위치' 버튼으로 복귀)
  gpsMap.on('dragstart', () => { gpsFollow = false; });
}

function refreshGpsMap() { if (gpsMap) gpsMap.invalidateSize(); }

function recenterGps() {
  if (!gpsMap || !state.gps) return;
  gpsFollow = true;
  gpsMap.invalidateSize();
  gpsMap.setView([state.gps.latitude, state.gps.longitude], Math.max(gpsMap.getZoom(), 16), { animate: true });
}

// ── GPS 이동경로 기록/불러오기 ──
let gpsTrackLine = null;    // 불러온 저장 경로 (빨강)
let gpsLiveLine = null;     // 기록 중 실시간 경로 (노랑)
let gpsLivePts = [];

async function toggleGpsTrackRec() {
  const rec = state.gpsTrack && state.gpsTrack.recording;
  try {
    const res = await fetch(rec ? '/api/gps/track/stop' : '/api/gps/track/start', { method: 'POST' });
    const d = await res.json();
    if (!d.ok) { alert(d.error || '실패'); return; }
    // 다음 SSE 틱을 기다리지 않고 버튼 상태 즉시 반영
    state.gpsTrack = { recording: !rec, points: 0 };
    renderGpsTrackBtn();
    if (rec) {
      alert(`이동 경로 저장 완료\n${d.file} (${d.points} pts)`);
      if (gpsLiveLine && gpsMap) { gpsMap.removeLayer(gpsLiveLine); gpsLiveLine = null; }
      gpsLivePts = [];
    } else {
      gpsLivePts = [];
    }
  } catch (e) { alert('요청 실패: ' + e); }
}

function renderGpsTrackBtn() {
  const btn = document.getElementById('gpsTrackRec');
  if (!btn) return;
  const t = state.gpsTrack;
  const rec = !!(t && t.recording);
  btn.textContent = rec ? `■ 기록 저장 (${t.points})` : '● 경로 기록';
  btn.classList.toggle('recording', rec);
}

function updateGpsLiveLine() {
  if (!(state.gpsTrack && state.gpsTrack.recording) || !state.gps || !gpsMap) return;
  const pos = [state.gps.latitude, state.gps.longitude];
  const last = gpsLivePts[gpsLivePts.length - 1];
  if (!last || last[0] !== pos[0] || last[1] !== pos[1]) {
    gpsLivePts.push(pos);
    if (!gpsLiveLine) {
      gpsLiveLine = L.polyline(gpsLivePts, { color: '#f1bf58', weight: 3, opacity: 0.9 }).addTo(gpsMap);
    } else {
      gpsLiveLine.addLatLng(pos);
    }
  }
}

function drawGpsTrack(points, name) {
  if (!gpsMap) { alert('지도가 아직 준비되지 않았습니다'); return; }
  clearGpsTrack();
  const latlngs = points.map((p) => [p[1], p[2]]);
  if (!latlngs.length) { alert('경로에 포인트가 없습니다'); return; }
  gpsTrackLine = L.polyline(latlngs, { color: '#f16e78', weight: 3, opacity: 0.85 }).addTo(gpsMap);
  gpsTrackLine.bindTooltip(name || '이동 경로');
  gpsFollow = false;
  gpsMap.fitBounds(gpsTrackLine.getBounds(), { padding: [24, 24] });
}

function clearGpsTrack() {
  if (gpsTrackLine && gpsMap) gpsMap.removeLayer(gpsTrackLine);
  gpsTrackLine = null;
}

async function showGpsTrackMenu() {
  const menu = document.getElementById('gpsTrackMenu');
  if (!menu) return;
  if (!menu.hidden) { menu.hidden = true; return; }
  let d;
  try {
    d = await (await fetch('/api/gps/track/list', { cache: 'no-store' })).json();
  } catch (e) { alert('목록 조회 실패: ' + e); return; }
  if (!d.ok) { alert(d.error || '목록 조회 실패'); return; }
  menu.innerHTML = '';
  if (gpsTrackLine) {
    const clear = document.createElement('button');
    clear.type = 'button';
    clear.className = 'track-item track-clear';
    clear.textContent = '✕ 표시된 경로 지우기';
    clear.addEventListener('click', () => { clearGpsTrack(); menu.hidden = true; });
    menu.appendChild(clear);
  }
  if (!d.files.length) {
    const empty = document.createElement('div');
    empty.className = 'track-empty';
    empty.textContent = '저장된 이동 경로가 없습니다';
    menu.appendChild(empty);
  }
  for (const f of d.files) {
    const row = document.createElement('div');
    row.className = 'track-row';
    const item = document.createElement('button');
    item.type = 'button';
    item.className = 'track-item';
    item.innerHTML = `<span>${escapeHtml(f.name)}</span><span class="track-size">${f.size_kb} KB</span>`;
    item.addEventListener('click', async () => {
      menu.hidden = true;
      try {
        const t = await (await fetch('/api/gps/track/load?file=' + encodeURIComponent(f.name))).json();
        if (!t.ok) { alert(t.error || '불러오기 실패'); return; }
        drawGpsTrack(t.points, f.name);
      } catch (e) { alert('불러오기 실패: ' + e); }
    });
    const del = document.createElement('button');
    del.type = 'button';
    del.className = 'track-del';
    del.textContent = '🗑';
    del.title = '이 경로 삭제';
    del.addEventListener('click', async (e) => {
      e.stopPropagation();
      if (!confirm(`${f.name}\n이 이동 경로를 삭제할까요?`)) return;
      try {
        const r = await (await fetch('/api/gps/track/delete?file=' + encodeURIComponent(f.name), { method: 'POST' })).json();
        if (!r.ok) { alert(r.error || '삭제 실패'); return; }
        menu.hidden = true;
        showGpsTrackMenu();  // 목록 새로고침
      } catch (err) { alert('삭제 실패: ' + err); }
    });
    row.appendChild(item);
    row.appendChild(del);
    menu.appendChild(row);
  }
  menu.hidden = false;
}

function renderGps() {
  const gps = view.mode === 'play' ? state.gpsPlay : state.gps;
  const subtitle = document.getElementById('gpsSubtitle');
  const latEl = document.getElementById('gpsLat');
  const lngEl = document.getElementById('gpsLng');
  const altEl = document.getElementById('gpsAlt');
  const linkEl = document.getElementById('gpsOpenMap');
  if (!gps) {
    if (subtitle) subtitle.textContent = 'waiting for /gpsfix';
    if (linkEl) linkEl.classList.remove('active');
    return;
  }
  if (subtitle) {
    let extra = '';
    if (gps.satellites != null) extra += ` · sats ${gps.satellites}`;
    if (gps.status != null) extra += ` · st ${gps.status}`;
    subtitle.textContent = `${gps.topic} · ${gps.age_sec}s ago${extra}`;
  }
  if (latEl) latEl.textContent = `${gps.latitude.toFixed(7)}°`;
  if (lngEl) lngEl.textContent = `${gps.longitude.toFixed(7)}°`;
  if (altEl) altEl.textContent = `${gps.altitude.toFixed(1)} m`;
  if (linkEl) {
    linkEl.href = `https://maps.google.com/maps?q=${gps.latitude},${gps.longitude}`;
    linkEl.classList.add('active');
  }
  const recBtn = document.getElementById('gpsRecenter');
  if (recBtn) recBtn.disabled = false;
  initGpsMap(gps.latitude, gps.longitude);
  if (gpsMap && gpsMarker) {
    const pos = [gps.latitude, gps.longitude];
    gpsMarker.setLatLng(pos);
    if (gpsFollow) gpsMap.panTo(pos);  // 자동 추적 중일 때만 이동
  }
}

const VEH_GEAR = { 0: 'P', 5: 'D', 6: 'N', 7: 'R' };
const VEH_RPM_MAX = 8000;   // motor_rpm 게이지 최대(추정)

// Vehicle Monitor 서브카드 구성(순서/너비span/높이) — 편집 모드에서 조정
// drive = 기어 + 브레이크/엑셀 통합 카드
const VEH_CARD_DEFAULTS = [
  { id: 'drive', span: 2 }, { id: 'steer', span: 1 }, { id: 'wheels', span: 2 },
  { id: 'batt', span: 1 }, { id: 'torque', span: 1 },
];
function loadVehCards() {
  const saved = JSON.parse(localStorage.getItem('rosWebVehCards') || 'null');
  if (!Array.isArray(saved)) return VEH_CARD_DEFAULTS.map((x) => ({ span: 1, ...x }));
  const ordered = saved
    .filter((s) => VEH_CARD_DEFAULTS.some((d) => d.id === s.id))
    .map((s) => ({ span: 1, ...s }));
  VEH_CARD_DEFAULTS.forEach((d) => { if (!ordered.some((o) => o.id === d.id)) ordered.push({ span: 1, ...d }); });
  return ordered;
}
let vehCards = loadVehCards();
function saveVehCards() { localStorage.setItem('rosWebVehCards', JSON.stringify(vehCards)); }
let vehEditRendered = false;

function vehTire(spin) {
  return `<svg class="veh-tire${spin ? ' spin' : ''}" viewBox="0 0 40 40">
    <circle class="tread" cx="20" cy="20" r="17"/>
    <circle class="rim" cx="20" cy="20" r="7"/>
    <line class="spoke" x1="20" y1="4" x2="20" y2="36"/>
    <line class="spoke" x1="4" y1="20" x2="36" y2="20"/>
  </svg>`;
}

// ── SVG 아크 게이지 헬퍼 (0° = 12시, 시계방향 +) ──
function vehPolar(cx, cy, r, deg) {
  const a = (deg - 90) * Math.PI / 180;
  return [cx + r * Math.cos(a), cy + r * Math.sin(a)];
}
function vehArc(cx, cy, r, a0, a1) {
  if (Math.abs(a1 - a0) < 0.01) return '';
  const [sx, sy] = vehPolar(cx, cy, r, a0);
  const [ex, ey] = vehPolar(cx, cy, r, a1);
  const large = Math.abs(a1 - a0) > 180 ? 1 : 0;
  const sweep = a1 > a0 ? 1 : 0;
  return `M ${sx.toFixed(2)} ${sy.toFixed(2)} A ${r} ${r} 0 ${large} ${sweep} ${ex.toFixed(2)} ${ey.toFixed(2)}`;
}
// 세그먼트 이퀄라이저 막대 (이미지 스타일)
function vehEq(pct, cls) {
  const N = 16, filled = Math.round(Math.max(0, Math.min(100, pct)) / 100 * N);
  let bars = '';
  for (let i = 0; i < N; i++) {
    const h = 32 + (i / (N - 1)) * 68;
    bars += `<i class="${i < filled ? 'on' : ''}" style="height:${h}%"></i>`;
  }
  return `<div class="veh-eq veh-eq-${cls}">${bars}</div>`;
}

function renderVehicle() {
  if (!vehicleBody) return;
  // 편집 중에는 한 번만 그려 드래그/리사이즈가 끊기지 않게 함
  if (view.editLayout && vehEditRendered) return;
  const v = view.mode === 'play' ? state.vehiclePlay : state.vehicle;
  const topicLabel = effectiveTopic('/acu_ck2_ver2');
  const sub = document.getElementById('vehicleSubtitle');
  if (!v) {
    vehicleBody.innerHTML = `<div class="veh-nodata">${escapeHtml(topicLabel)} 데이터 없음</div>`;
    if (sub) sub.textContent = `${topicLabel} — 대기중`;
    return;
  }
  if (sub) sub.textContent = topicLabel;

  const num = (x) => (typeof x === 'number' ? x : 0);
  // 차량 규약: steer_ang_val +=좌 / -=우. 화면 게이지·스케일(-468좌 ~ +468우)과
  // 핸들 회전 방향을 실제와 맞추기 위해 부호 반전(우회전=+).
  const steer = -num(v.steer_angle), steerSpd = num(v.steer_speed);
  const brake = num(v.brake), rpm = num(v.motor_rpm), soc = num(v.soc), tq = num(v.motor_tq);
  const gearCur = VEH_GEAR[Math.round(num(v.gear))] ?? '—';
  const brakePct = Math.max(0, Math.min(100, (brake / 50) * 100));
  const rpmPct = Math.max(0, Math.min(100, (Math.abs(rpm) / VEH_RPM_MAX) * 100));
  const socColor = soc >= 50 ? 'var(--green)' : soc >= 20 ? 'var(--yellow)' : 'var(--red)';

  const gearName = { P: 'PARK', R: 'REVERSE', N: 'NEUTRAL', D: 'DRIVE' };
  const gearHtml = ['P', 'R', 'N', 'D'].map((g) =>
    `<span class="veh-gear-seg${g === gearCur ? ' active' : ''}">${g}</span>`).join('');
  const wcCell = (pos, label, val) => {
    const w = num(val);
    const live = Math.abs(w) > 0.5 ? ' live' : '';
    return `<div class="wc-cell wc-${pos}${live}"><span class="wc-lbl">${label}</span><span class="wc-val">${w.toFixed(0)}</span><span class="wc-u">km/h</span></div>`;
  };
  const carMoving = [v.wheel_fl, v.wheel_fr, v.wheel_rl, v.wheel_rr].some((x) => Math.abs(num(x)) > 0.5);
  const carSvg = `<div class="wc-car"><svg viewBox="0 0 64 116" class="wc-svg${carMoving ? ' moving' : ''}">
      <ellipse cx="32" cy="58" rx="30" ry="54" class="car-aura"/>
      <rect class="car-wheel" x="6" y="24" width="9" height="17" rx="2.5"/>
      <rect class="car-wheel" x="49" y="24" width="9" height="17" rx="2.5"/>
      <rect class="car-wheel" x="6" y="78" width="9" height="17" rx="2.5"/>
      <rect class="car-wheel" x="49" y="78" width="9" height="17" rx="2.5"/>
      <path d="M14 20 Q14 7 32 7 Q50 7 50 20 L50 96 Q50 110 32 110 Q14 110 14 96 Z" class="car-body"/>
      <path d="M19 31 Q32 25 45 31 L43 45 Q32 41 21 45 Z" class="car-glass"/>
      <rect x="20" y="47" width="24" height="32" rx="6" class="car-roof"/>
      <path d="M21 81 Q32 86 43 81 L45 92 Q32 98 19 92 Z" class="car-glass"/>
    </svg></div>`;

  // ── 조향 아크 게이지 ──
  const steerF = Math.max(-1, Math.min(1, steer / 468));
  const steerTrack = vehArc(50, 50, 40, -135, 135);
  const steerAct = steerF >= 0 ? vehArc(50, 50, 40, 0, steerF * 135) : vehArc(50, 50, 40, steerF * 135, 0);

  // ── 배터리 링 게이지 ──
  const socF = Math.max(0, Math.min(1, soc / 100));
  const battTrack = vehArc(50, 50, 40, -135, 135);
  const battAct = vehArc(50, 50, 40, -135, -135 + socF * 270);
  const battStroke = soc >= 50 ? '#35d07f' : soc >= 20 ? '#f2c14e' : '#ff5a64';

  // ── 모터 토크 반원 게이지 ──
  const tqF = Math.max(-1, Math.min(1, tq / 300));
  const tqTrack = vehArc(50, 58, 42, -90, 90);
  const tqAct = tqF >= 0 ? vehArc(50, 58, 42, 0, tqF * 90) : vehArc(50, 58, 42, tqF * 90, 0);
  const [tnx, tny] = vehPolar(50, 58, 40, tqF * 90);

  // 카드 id → 내부 HTML
  const inner = {
    drive: `<div class="veh-drive">
        <div class="veh-dc-sec">
          <div class="veh-card-lbl">GEAR <span class="veh-gear-name">${gearName[gearCur] || '—'}</span></div>
          <div class="veh-gear-box">${gearHtml}</div>
        </div>
        <div class="veh-dc-div"></div>
        <div class="veh-dc-sec">
          <div class="veh-card-lbl">BRAKE / ACCEL</div>
          <div class="veh-pedal2">
            <div class="veh-pedcol">
              <div class="veh-ped-head veh-ped-brake">◉ BRAKE <b>${brakePct.toFixed(0)}<i>%</i></b></div>
              ${vehEq(brakePct, 'brake')}
            </div>
            <div class="veh-pedcol">
              <div class="veh-ped-head veh-ped-accel">⊿ ACCEL <b>${rpmPct.toFixed(0)}<i>%</i></b></div>
              ${vehEq(rpmPct, 'accel')}
            </div>
          </div>
        </div>
      </div>`,
    steer: `<div class="veh-card-lbl">STEERING</div>
        <div class="veh-steer-wrap">
          <svg class="veh-gauge" viewBox="0 0 100 100">
            <path d="${steerTrack}" class="g-track"/>
            ${steerAct ? `<path d="${steerAct}" class="g-steer"/>` : ''}
            <g transform="translate(50 50) rotate(${steer})" class="sw">
              <circle r="15" class="sw-rim"/><circle r="4.2" class="sw-hub"/>
              <line x1="0" y1="0" x2="0" y2="-15" class="sw-spoke"/>
              <line x1="0" y1="0" x2="-13" y2="7.5" class="sw-spoke"/>
              <line x1="0" y1="0" x2="13" y2="7.5" class="sw-spoke"/>
            </g>
          </svg>
          <div class="veh-steer-info">
            <div class="veh-steer-big">${steer > 0 ? '+' : ''}${steer.toFixed(0)}<span>°</span></div>
            <div class="veh-card-sub">STEERING ANGLE</div>
          </div>
        </div>
        <div class="veh-scale"><span>-468°</span><span>+468°</span></div>`,
    wheels: `<div class="veh-card-lbl">WHEEL SPEED</div>
        <div class="veh-wheelcar">
          ${wcCell('fl', 'FL', v.wheel_fl)}
          ${carSvg}
          ${wcCell('fr', 'FR', v.wheel_fr)}
          ${wcCell('rl', 'RL', v.wheel_rl)}
          ${wcCell('rr', 'RR', v.wheel_rr)}
        </div>`,
    batt: `<div class="veh-card-lbl">BATTERY <span class="veh-card-sub2">STATE OF CHARGE</span></div>
        <div class="veh-ring-wrap">
          <svg class="veh-gauge" viewBox="0 0 100 100">
            <path d="${battTrack}" class="g-track"/>
            ${battAct ? `<path d="${battAct}" class="g-batt" style="stroke:${battStroke}"/>` : ''}
          </svg>
          <div class="veh-ring-center">
            <svg class="veh-batt-ico" viewBox="0 0 24 24" style="color:${battStroke}"><rect x="3" y="7" width="15" height="10" rx="2"/><rect x="18.5" y="10" width="2.5" height="4" rx="1"/><path d="M11 8.5l-2.5 4h2l-0.5 3 3-4h-2z" class="bolt"/></svg>
            <div class="veh-ring-pct" style="color:${battStroke}">${soc.toFixed(0)}<span>%</span></div>
          </div>
        </div>`,
    torque: `<div class="veh-card-lbl">MOTOR TORQUE</div>
        <div class="veh-torque-wrap">
          <svg class="veh-gauge-half" viewBox="0 0 100 66">
            <path d="${tqTrack}" class="g-track"/>
            ${tqAct ? `<path d="${tqAct}" class="g-torque"/>` : ''}
            <line x1="50" y1="58" x2="${tnx.toFixed(1)}" y2="${tny.toFixed(1)}" class="g-needle"/>
            <circle cx="50" cy="58" r="3" class="g-needle-hub"/>
            <text x="50" y="44" class="g-tq-val">${tq.toFixed(0)}</text>
            <text x="50" y="55" class="g-tq-unit">N·m</text>
          </svg>
          <div class="veh-scale veh-scale-3"><span>-300</span><span>0</span><span>300</span></div>
        </div>`,
  };

  const cardsHtml = vehCards.map((c) => {
    if (!inner[c.id]) return '';
    const extra = c.id === 'steer' ? ' veh-steer' : '';
    const styleH = c.h ? ` style="min-height:${c.h}px"` : '';
    return `<div class="veh-card${extra}" data-card="${c.id}" data-span="${c.span || 1}"${styleH}>
      <span class="veh-card-drag" title="드래그로 위치 이동">⠿</span>
      ${inner[c.id]}
      <span class="veh-card-grip" title="드래그로 크기 조절"></span>
    </div>`;
  }).join('');

  vehicleBody.innerHTML =
    `<div class="veh-edit-hint">⠿ 드래그로 위치 · ↘ 모서리로 크기(가로 폭·세로 높이) 조절</div>
     <div class="veh-grid">${cardsHtml}</div>`;

  if (view.editLayout) { bindVehCardEdit(); vehEditRendered = true; }
}

// 편집 모드: 서브카드 드래그 재배치 + 모서리 리사이즈
function bindVehCardEdit() {
  const grid = vehicleBody.querySelector('.veh-grid');
  if (!grid) return;
  grid.querySelectorAll('.veh-card').forEach((card) => {
    const id = card.dataset.card;
    const grip = card.querySelector('.veh-card-grip');
    if (grip) grip.addEventListener('pointerdown', (e) => {
      e.preventDefault(); e.stopPropagation();
      const cfg = vehCards.find((c) => c.id === id); if (!cfg) return;
      grip.setPointerCapture(e.pointerId);
      const startX = e.clientX, startY = e.clientY;
      const startH = card.getBoundingClientRect().height;
      const startSpan = cfg.span || 1;
      const onMove = (ev) => {
        const dx = ev.clientX - startX, dy = ev.clientY - startY;
        const h = Math.max(64, Math.round(startH + dy));
        cfg.h = h; card.style.minHeight = h + 'px';
        const span = (startSpan === 1 && dx > 60) ? 2 : (startSpan === 2 && dx < -60) ? 1 : startSpan;
        cfg.span = span; card.dataset.span = String(span);
      };
      const onUp = () => {
        grip.releasePointerCapture(e.pointerId);
        grip.removeEventListener('pointermove', onMove);
        grip.removeEventListener('pointerup', onUp);
        saveVehCards();
      };
      grip.addEventListener('pointermove', onMove);
      grip.addEventListener('pointerup', onUp);
    });
    card.addEventListener('pointerdown', (e) => {
      if (e.target.closest('.veh-card-grip')) return;
      e.preventDefault();
      card.classList.add('veh-dragging');
      card.setPointerCapture(e.pointerId);
      let dropId = null;
      const onMove = (ev) => {
        const el = document.elementFromPoint(ev.clientX, ev.clientY);
        const over = el && el.closest('.veh-card');
        grid.querySelectorAll('.veh-card').forEach((c) => c.classList.remove('veh-over'));
        if (over && over.dataset.card !== id) { over.classList.add('veh-over'); dropId = over.dataset.card; }
        else dropId = null;
      };
      const onUp = () => {
        card.releasePointerCapture(e.pointerId);
        card.removeEventListener('pointermove', onMove);
        card.removeEventListener('pointerup', onUp);
        card.classList.remove('veh-dragging');
        grid.querySelectorAll('.veh-card').forEach((c) => c.classList.remove('veh-over'));
        if (dropId) {
          const fi = vehCards.findIndex((c) => c.id === id);
          const ti = vehCards.findIndex((c) => c.id === dropId);
          if (fi >= 0 && ti >= 0) { const [m] = vehCards.splice(fi, 1); vehCards.splice(ti, 0, m); }
        }
        saveVehCards();
        vehEditRendered = false; renderVehicle();
      };
      card.addEventListener('pointermove', onMove);
      card.addEventListener('pointerup', onUp);
    });
  });
}


/* ════════════════════════════════════════════════════════════
   자율주행 모니터링 신기능: 감지 객체 목록 / 이벤트 로그 / 근접 경고
   ════════════════════════════════════════════════════════════ */

// ── 감지 객체 목록 (RViz 박스/레이더 마커 → 거리순 테이블) ──
function objPosLabel(x, y) {
  const ang = Math.atan2(y, x) * 180 / Math.PI;   // x=전방, y=좌측
  if (ang >= -35 && ang < 35) return '전방';
  if (ang >= 35 && ang < 100) return '전방좌';
  if (ang >= -100 && ang < -35) return '전방우';
  if (ang >= 100 || ang < -145) return (ang >= 100 && ang < 145) ? '후방좌' : '후방';
  return '후방우';
}
function vizTopicColor() {
  const m = {};
  for (const l of view.vizLayers) { if (l.topic) m[effectiveTopic(l.topic)] = l.color; }
  return m;
}
function renderObjects() {
  const rowsEl = document.getElementById('objRows');
  if (!rowsEl) return;
  const colorOf = vizTopicColor();
  const inPlay = view.mode === 'play';
  const objs = [];
  for (const b of state.visualization.boxes || []) {
    if (isPlayTopic(b.topic) !== inPlay) continue;
    objs.push({ x: b.x || 0, y: b.y || 0, label: b.label || b.kind || '객체',
      kind: b.kind || '', size: `${(b.sx || 0).toFixed(1)}×${(b.sy || 0).toFixed(1)}`,
      color: colorOf[b.topic] || '#9fb2c4' });
  }
  for (const m of state.visualization.markers || []) {
    if (isPlayTopic(m.topic) !== inPlay) continue;
    if (m.kind !== 'radar' || [4, 5].includes(m.type)) continue;
    for (const p of (m.points || []).slice(0, 40)) {
      objs.push({ x: p[0] || 0, y: p[1] || 0, label: 'radar', kind: 'radar', size: '—',
        color: colorOf[m.topic] || '#f1bf58' });
    }
  }
  objs.forEach((o) => { o.dist = Math.hypot(o.x, o.y); });
  objs.sort((a, b) => a.dist - b.dist);
  const top = objs.slice(0, 30);
  const sub = document.getElementById('objSubtitle');
  if (sub) sub.textContent = `${objs.length}개 감지`;
  if (!top.length) { rowsEl.innerHTML = '<div class="obj-empty">감지된 객체 없음</div>'; return; }
  rowsEl.innerHTML = top.map((o, i) => `<div class="obj-row">
      <span class="obj-idx" style="background:${o.color}">${i + 1}</span>
      <span class="obj-name">${escapeHtml(String(o.label).slice(0, 16))}</span>
      <span class="obj-pos">${objPosLabel(o.x, o.y)}</span>
      <span class="obj-dist">${o.dist.toFixed(1)}m</span>
      <span class="obj-size">${o.size}</span>
    </div>`).join('');
}

// ── 이벤트 로그 (토픽 상태 전이·Bag·고온·저배터리, 클라이언트 파생) ──
const eventLog = [];
const evtTopicPrev = {};
let evtSeeded = false;
let evtBagPrev = false;
let evtTempHot = false;
let evtSocLow = false;
let eventsDirty = true;     // 첫 렌더에서 빈 목록 안내를 그리도록 true 로 시작
let evtConnLost = false;
const EVT_GROUPS = new Set(['cameras', 'lidar', 'radar', 'localization', 'vehicle_can', 'perception']);

function pushEvent(level, desc, mod) {
  const t = state.time || new Date().toTimeString().slice(0, 8);
  eventLog.unshift({ time: t.length > 8 ? t.slice(-8) : t, level, desc, mod });
  if (eventLog.length > 80) eventLog.pop();
  eventsDirty = true;
}
function collectEvents() {
  const rec = !!(state.bag && state.bag.recording);
  if (rec !== evtBagPrev) { pushEvent('정보', rec ? 'Bag 녹화 시작' : 'Bag 녹화 종료', 'Recorder'); evtBagPrev = rec; }

  // 온도: 최고 온도 80°C 이상 진입/해제 (엣지 트리거)
  const maxT = Math.max(0, ...((state.system.thermal || []).map((t) => t.c || 0)));
  if (maxT >= 80 && !evtTempHot) { pushEvent('경고', `고온 감지 ${maxT.toFixed(0)}°C`, 'System'); evtTempHot = true; }
  else if (maxT < 75 && evtTempHot) { evtTempHot = false; }

  // 배터리 SOC 20% 미만 진입 (엣지 트리거)
  const soc = state.vehicle && typeof state.vehicle.soc === 'number' ? state.vehicle.soc : null;
  if (soc != null) {
    if (soc < 20 && !evtSocLow) { pushEvent('경고', `배터리 부족 ${soc.toFixed(0)}%`, 'Vehicle'); evtSocLow = true; }
    else if (soc >= 25 && evtSocLow) { evtSocLow = false; }
  }

  for (const t of state.topics) {
    const g = groupForTopic(t);
    if (!EVT_GROUPS.has(g)) continue;
    const subFail = t.error && (t.error.includes('Failed to create') || t.error.includes('invalid allocator'));
    const cur = (t.error && !subFail) ? 'error' : t.active ? 'active' : 'idle';
    const prev = evtTopicPrev[t.name];
    evtTopicPrev[t.name] = cur;
    if (!evtSeeded) {
      // 시작 시점에 이미 오류인 센서 토픽은 바로 기록
      if (cur === 'error') pushEvent('오류', `${t.name.split('/').filter(Boolean).slice(-2).join('/')} 오류`, g === 'localization' ? 'GNSS' : g === 'vehicle_can' ? 'CAN' : g);
      continue;
    }
    if (prev === undefined || prev === cur) continue;
    const short = t.name.split('/').filter(Boolean).slice(-2).join('/');
    const mod = g === 'localization' ? 'GNSS' : g === 'vehicle_can' ? 'CAN'
      : g === 'perception' ? 'Percept' : g.charAt(0).toUpperCase() + g.slice(1);
    if (cur === 'error') pushEvent('오류', `${short} 오류`, mod);
    else if (cur === 'idle' && prev === 'active') pushEvent('경고', `${short} 신호 끊김`, mod);
    else if (cur === 'active' && prev !== 'active') pushEvent('정보', `${short} 수신 정상`, mod);
  }
  if (!evtSeeded && state.topics.length) {
    const act = state.topics.filter((t) => t.active).length;
    pushEvent('정보', `모니터링 시작 · 활성 토픽 ${act}/${state.topics.length}`, 'Monitor');
    // 토픽은 있는데 하나도 수신되지 않는 센서 그룹
    for (const g of state.groups || []) {
      if (EVT_GROUPS.has(g.name) && g.total > 0 && !g.active) pushEvent('경고', `${g.name} 수신 없음 (${g.total}개 토픽)`, g.name);
    }
  }
  if (state.topics.length) evtSeeded = true;
}
function renderEvents() {
  if (!eventsDirty) return;
  const el = document.getElementById('evtRows');
  if (!el) { eventsDirty = false; return; }
  const cnt = document.getElementById('evtSubtitle');
  if (cnt) cnt.textContent = eventLog.length ? `${eventLog.length}건` : '';
  if (!eventLog.length) { el.innerHTML = '<div class="obj-empty">이벤트 없음 — 상태 변화 시 기록됩니다</div>'; }
  else {
    el.innerHTML = eventLog.map((e) => {
      const cls = e.level === '오류' ? 'evt-bad' : e.level === '경고' ? 'evt-warn' : 'evt-ok';
      return `<div class="evt-row">
        <span class="evt-time">${escapeHtml(e.time)}</span>
        <span class="evt-lv ${cls}">${e.level}</span>
        <span class="evt-desc">${escapeHtml(e.desc)}</span>
        <span class="evt-mod">${escapeHtml(e.mod)}</span>
      </div>`;
    }).join('');
  }
  eventsDirty = false;
}

// ── 근접 장애물 경고 (최근접 박스 거리 기반, RViz 위 배너) ──
const PROX_WARN_M = 7.0;    // 주황 경고
const PROX_DANGER_M = 3.5;  // 빨강 위험
function renderProximityAlert() {
  const el = document.getElementById('vizAlert');
  if (!el) return;
  let best = null;
  const inPlay = view.mode === 'play';
  for (const b of state.visualization.boxes || []) {
    if (isPlayTopic(b.topic) !== inPlay) continue;
    const d = Math.hypot(b.x || 0, b.y || 0);
    if (d > 0.3 && (!best || d < best.d)) best = { d, x: b.x || 0, y: b.y || 0, label: b.label || '' };
  }
  if (!best || best.d > PROX_WARN_M) { el.hidden = true; return; }
  const danger = best.d <= PROX_DANGER_M;
  el.hidden = false;
  el.className = 'viz-alert ' + (danger ? 'viz-alert-danger' : 'viz-alert-warn');
  el.textContent = `⚠ ${objPosLabel(best.x, best.y)} ${best.d.toFixed(1)}m 장애물${danger ? ' — 위험' : ''}`;
}

function renderGroups() {
  // 백엔드 groups는 실시간+재생 토픽을 합산하므로, 현재 뷰 모드 토픽만으로 클라이언트에서 재집계
  const agg = {};
  for (const t of currentModeTopics()) {
    const g = groupForTopic(t);
    const a = agg[g] || (agg[g] = { name: g, total: 0, active: 0, rate_hz: 0 });
    a.total += 1;
    if (t.active) a.active += 1;
    a.rate_hz += t.rate_hz;
  }
  const groups = Object.keys(agg).sort().map((k) => agg[k]);
  if (!groups.length) { groupGrid.innerHTML = '<div class="empty">아직 ROS 토픽을 찾지 못했습니다.</div>'; return; }
  groupGrid.innerHTML = groups.map((group) => {
    const percent = group.total ? Math.round((group.active / group.total) * 100) : 0;
    return `<article class="group-card"><b>${escapeHtml(group.name.replace(/_/g, ' '))}</b><div class="numbers"><span>${group.active}/${group.total}</span><span>${group.rate_hz.toFixed(1)} Hz</span></div><div class="bar"><i style="width:${percent}%"></i></div></article>`;
  }).join('');
}


function usageColor(pct) {
  if (pct >= 80) return 'high';
  if (pct >= 60) return 'mid';
  return 'low';
}

const sysHistory = { cpu: [], gpu: [], ram: [], power: [] };
const SYS_HIST_MAX = 80;

function pushHistory(arr, val) {
  arr.push(val);
  if (arr.length > SYS_HIST_MAX) arr.shift();
}

function drawSparkline(canvasId, data, strokeColor, fillColor) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;
  const W = canvas.width, H = canvas.height;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, W, H);
  if (data.length < 2) return;
  const pts = data.map((v, i) => [
    (i / (SYS_HIST_MAX - 1)) * W,
    H - (Math.max(0, Math.min(100, v)) / 100) * H * 0.92,
  ]);
  ctx.beginPath();
  pts.forEach(([x, y], i) => i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y));
  ctx.strokeStyle = strokeColor;
  ctx.lineWidth = 1.5;
  ctx.lineJoin = 'round';
  ctx.stroke();
  ctx.lineTo(W, H); ctx.lineTo(0, H); ctx.closePath();
  ctx.fillStyle = fillColor;
  ctx.fill();
}

function renderSystem() {
  const sys = state.system || {};
  const cpu = sys.cpu || {};
  const memory = sys.memory || {};
  const gpu = sys.gpu || {};
  systemSubtitle.textContent = sys.model ? sys.model.replace('NVIDIA ', '') : 'Jetson status';
  // (스파크라인 히스토리는 render() 빠른 경로에서 항상 축적됨)

  // 현재 활성 탭이 설정에 없으면(이름변경 무관, id기준) 첫 탭으로
  const tabs = view.sysTabConfig;
  if (!tabs.some((t) => t.id === view.sysSubTab)) view.sysSubTab = tabs[0].id;
  const tab = view.sysSubTab;
  const editing = view.editLayout;
  const tabBar = `<nav class="sysm-tabs${editing ? ' sysm-tabs-edit' : ''}">${tabs.map((t) =>
    `<button class="sysm-tab${t.id === tab ? ' active' : ''}" data-systab="${t.id}" type="button">${
      editing ? `<span class="sysm-tab-drag" data-systab="${t.id}">⠿</span>` : ''
    }<span class="sysm-tab-label">${escapeHtml(t.label)}</span></button>`
  ).join('')}</nav>${editing ? '<div class="sysm-edit-hint">드래그로 순서 변경 · 더블클릭으로 이름 변경</div>' : ''}`;

  let body = '';
  if (tab === 'all') body = renderSysAll(sys);
  else if (tab === 'gpu') body = renderSysGpu(sys);
  else if (tab === 'cpu') body = renderSysCpu(sys);
  else if (tab === 'mem') body = renderSysMem(sys);
  else if (tab === 'eng') body = renderSysEng(sys);
  else if (tab === 'ctrl') body = renderSysCtrl(sys);
  else if (tab === 'info') body = renderSysInfo(sys);

  systemGrid.innerHTML = tabBar + `<div class="sysm-body">${body}</div>`;

  // 탭 클릭 이벤트
  systemGrid.querySelectorAll('.sysm-tab').forEach((btn) => {
    btn.addEventListener('click', (e) => {
      if (editing) return;  // 편집 모드에선 탭 전환 안 함 (드래그/이름변경 전용)
      view.sysSubTab = btn.dataset.systab;
      localStorage.setItem('rosWebSysSubTab', view.sysSubTab);
      renderSystem();
    });
    if (editing) {
      btn.addEventListener('dblclick', () => {
        const t = view.sysTabConfig.find((x) => x.id === btn.dataset.systab);
        if (!t) return;
        const nv = prompt('탭 이름 변경:', t.label);
        if (nv && nv.trim()) {
          t.label = nv.trim().slice(0, 12);
          localStorage.setItem('rosWebSysTabConfig', JSON.stringify(view.sysTabConfig));
          renderSystem();
        }
      });
    }
  });

  // 편집 모드: 드래그로 순서 변경
  if (editing) {
    systemGrid.querySelectorAll('.sysm-tab-drag').forEach((handle) => {
      handle.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        const btn = handle.closest('.sysm-tab');
        const fromId = handle.dataset.systab;
        btn.classList.add('tab-dragging');
        handle.setPointerCapture(e.pointerId);
        let dropTarget = null;
        const onMove = (ev) => {
          const el = document.elementFromPoint(ev.clientX, ev.clientY);
          const overBtn = el && el.closest('.sysm-tab');
          systemGrid.querySelectorAll('.sysm-tab').forEach((b) => b.classList.remove('tab-drop-over'));
          if (overBtn && overBtn !== btn) { overBtn.classList.add('tab-drop-over'); dropTarget = overBtn; }
          else dropTarget = null;
        };
        const onUp = () => {
          handle.releasePointerCapture(e.pointerId);
          handle.removeEventListener('pointermove', onMove);
          handle.removeEventListener('pointerup', onUp);
          if (dropTarget) {
            const toId = dropTarget.dataset.systab;
            const cfg = view.sysTabConfig;
            const fi = cfg.findIndex((t) => t.id === fromId);
            const ti = cfg.findIndex((t) => t.id === toId);
            if (fi >= 0 && ti >= 0) { const [m] = cfg.splice(fi, 1); cfg.splice(ti, 0, m); }
            localStorage.setItem('rosWebSysTabConfig', JSON.stringify(cfg));
          }
          renderSystem();
        };
        handle.addEventListener('pointermove', onMove);
        handle.addEventListener('pointerup', onUp);
      });
    });
  }

  // 캔버스 스파크라인 (해당 탭에 캔버스가 있을 때만)
  drawSparkline('sysChartCpu', sysHistory.cpu, '#64c8ff', 'rgba(100,200,255,0.12)');
  drawSparkline('sysChartGpu', sysHistory.gpu, '#56d38b', 'rgba(86,211,139,0.12)');
  drawSparkline('sysChartRam', sysHistory.ram, '#f1bf58', 'rgba(241,191,88,0.12)');
}

function sysMetricCard(label, pct, sub, canvasId, lv) {
  return `
    <article class="sysm-card">
      <div class="sysm-top">
        <div>
          <div class="sysm-label">${label}</div>
          <div class="sysm-big usage-${lv}">${pct.toFixed(1)}<span class="sysm-unit">%</span></div>
          <div class="sysm-subs">${sub}</div>
        </div>
        ${canvasId ? `<canvas id="${canvasId}" class="sysm-chart" width="120" height="52"></canvas>` : ''}
      </div>
      <div class="sysm-bar"><i class="bar-${lv}" style="width:${Math.min(100, pct)}%"></i></div>
    </article>`;
}

// ── All: CPU/GPU/RAM 요약 + 온도 ─────────────────────────────────────────
function renderSysAll(sys) {
  const cpu = sys.cpu || {}, memory = sys.memory || {}, gpu = sys.gpu || {};
  const thermal = sys.thermal || [];
  const cpuPct = cpu.usage_percent || 0, ramPct = memory.usage_percent || 0, gpuPct = gpu.usage_percent ?? 0;
  const loadAvg = (cpu.loadavg || []).slice(0, 3);
  const ramUsedGB = (memory.used_mb / 1024).toFixed(1), ramTotGB = (memory.total_mb / 1024).toFixed(0);
  const powerTot = gpu.power_tot_mw;
  const thermalHtml = thermal.length
    ? thermal.slice(0, 10).map((t) => {
        // 온도에 따라 그린(<65) / 주황(65~79) / 빨강(>=80)
        const lv = t.c >= 80 ? 'hot' : t.c >= 65 ? 'warm' : 'cool';
        const w = Math.max(2, Math.min(100, t.c));  // 0~100°C 스케일
        return `<div class="temp-row">
          <span class="temp-name">${escapeHtml(t.name)}</span>
          <div class="temp-track"><i class="temp-bar t-${lv}" style="width:${w}%"></i></div>
          <span class="temp-val t-${lv}">${t.c}°C</span>
        </div>`;
      }).join('')
    : '<span class="sys-muted">n/a</span>';

  return sysMetricCard('CPU', cpuPct,
      `${cpu.temp_c != null ? `<span class="sysm-sub">${cpu.temp_c}°C</span>` : ''}${loadAvg.length ? `<span class="sysm-sub">load ${loadAvg.join(' / ')}</span>` : ''}`,
      'sysChartCpu', usageColor(cpuPct))
    + sysMetricCard('GPU', gpuPct,
      `${gpu.temp_c != null ? `<span class="sysm-sub">${gpu.temp_c}°C</span>` : ''}${gpu.freq_mhz ? `<span class="sysm-sub">${gpu.freq_mhz} MHz</span>` : ''}${powerTot != null ? `<span class="sysm-sub">${(powerTot/1000).toFixed(1)} W</span>` : ''}`,
      'sysChartGpu', usageColor(gpuPct))
    + sysMetricCard('RAM', ramPct,
      `<span class="sysm-sub">${ramUsedGB} / ${ramTotGB} GB</span>${memory.swap_total_mb > 0 ? `<span class="sysm-sub">SWAP ${memory.swap_used_mb}/${memory.swap_total_mb} MB</span>` : ''}`,
      'sysChartRam', usageColor(ramPct))
    + `<article class="sysm-card sysm-card-info">
        <div class="sysm-info-row">
          <div class="sysm-stat"><span class="sysm-k">모델</span><span class="sysm-v">${escapeHtml((sys.model||'').replace('NVIDIA ',''))}</span></div>
          <div class="sysm-stat"><span class="sysm-k">JetPack</span><span class="sysm-v">${escapeHtml(sys.jetpack||'—')}</span></div>
          <div class="sysm-stat"><span class="sysm-k">가동</span><span class="sysm-v usage-low">${formatUptime(sys.uptime_sec||0)}</span></div>
          ${powerTot != null ? `<div class="sysm-stat"><span class="sysm-k">전력</span><span class="sysm-v usage-mid">${(powerTot/1000).toFixed(1)} W</span></div>` : ''}
        </div>
      </article>
      <article class="sysm-card"><div class="sysm-label">온도</div><div class="temp-list">${thermalHtml}</div></article>`;
}

// ── GPU 상세 ─────────────────────────────────────────────────────────────
function renderSysGpu(sys) {
  const gpu = sys.gpu || {};
  const gpuPct = gpu.usage_percent ?? 0;
  const rows = [
    ['사용률', `${gpuPct.toFixed(1)} %`],
    ['온도', gpu.temp_c != null ? `${gpu.temp_c} °C` : '—'],
    ['주파수', gpu.freq_mhz ? `${gpu.freq_mhz} MHz` : '—'],
    ['GPU 전력', gpu.power_mw != null ? `${(gpu.power_mw/1000).toFixed(2)} W` : '—'],
    ['전체 전력', gpu.power_tot_mw != null ? `${(gpu.power_tot_mw/1000).toFixed(2)} W` : '—'],
    ['평균 전력', gpu.power_avg_mw != null ? `${(gpu.power_avg_mw/1000).toFixed(2)} W` : '—'],
  ];
  return sysMetricCard('GPU', gpuPct,
      `${gpu.temp_c != null ? `<span class="sysm-sub">${gpu.temp_c}°C</span>` : ''}${gpu.freq_mhz ? `<span class="sysm-sub">${gpu.freq_mhz} MHz</span>` : ''}`,
      'sysChartGpu', usageColor(gpuPct))
    + `<article class="sysm-card sysm-card-info"><div class="sysm-kv-list">${
        rows.map(([k, v]) => `<div class="sysm-kv"><span class="sysm-k">${k}</span><span class="sysm-v">${escapeHtml(v)}</span></div>`).join('')
      }</div></article>`;
}

// ── CPU: 코어별 점유율 ───────────────────────────────────────────────────
function renderSysCpu(sys) {
  const cpu = sys.cpu || {};
  const cpuPct = cpu.usage_percent || 0;
  const loadAvg = (cpu.loadavg || []).slice(0, 3);
  const detail = cpu.cores_detail && cpu.cores_detail.length
    ? cpu.cores_detail
    : (cpu.cores || []).map((u, i) => ({ idx: i, online: true, usage: u, freq_mhz: null, gov: '' }));

  const coreRows = detail.map((c) => {
    const lv = c.online ? usageColor(c.usage) : 'off';
    const w = Math.max(2, Math.min(100, c.usage));
    return `<div class="cpu-core-row">
      <span class="cpu-core-id">CPU${c.idx}</span>
      <div class="cpu-core-track"><i class="bar-${lv}" style="width:${w}%"></i></div>
      <span class="cpu-core-pct">${c.online ? c.usage.toFixed(0) + '%' : 'off'}</span>
      <span class="cpu-core-freq">${c.freq_mhz ? c.freq_mhz + ' MHz' : ''}</span>
    </div>`;
  }).join('');

  return sysMetricCard('CPU 전체', cpuPct,
      `${cpu.temp_c != null ? `<span class="sysm-sub">${cpu.temp_c}°C</span>` : ''}${loadAvg.length ? `<span class="sysm-sub">load ${loadAvg.join(' / ')}</span>` : ''}<span class="sysm-sub">${detail.length} cores</span>`,
      'sysChartCpu', usageColor(cpuPct))
    + `<article class="sysm-card"><div class="sysm-label">코어별 점유율</div><div class="cpu-core-list">${coreRows}</div></article>`;
}

// ── MEM: RAM / SWAP / Disk ───────────────────────────────────────────────
function renderSysMem(sys) {
  const m = sys.memory || {};
  const ramPct = m.usage_percent || 0;
  const ramUsedGB = (m.used_mb / 1024).toFixed(2), ramTotGB = (m.total_mb / 1024).toFixed(1);
  const swapPct = m.swap_total_mb > 0 ? (m.swap_used_mb / m.swap_total_mb * 100) : 0;
  const disk = m.disk || {};
  const diskPct = disk.total ? (disk.used / disk.total * 100) : 0;

  let html = sysMetricCard('RAM', ramPct,
    `<span class="sysm-sub">${ramUsedGB} / ${ramTotGB} GB</span><span class="sysm-sub">여유 ${((m.available_mb||0)/1024).toFixed(2)} GB</span>`,
    null, usageColor(ramPct));
  if (m.swap_total_mb > 0) {
    html += sysMetricCard('SWAP', swapPct,
      `<span class="sysm-sub">${m.swap_used_mb} / ${m.swap_total_mb} MB</span>`,
      null, usageColor(swapPct));
  }
  if (disk.total) {
    html += sysMetricCard('DISK', diskPct,
      `<span class="sysm-sub">${disk.used} / ${disk.total} ${disk.unit}B</span><span class="sysm-sub">여유 ${disk.available} ${disk.unit}B</span>`,
      null, usageColor(diskPct));
  }
  return html;
}

// ── ENG: 하드웨어 엔진 + 전력 rail ───────────────────────────────────────
function renderSysEng(sys) {
  const engines = sys.engines || [];
  const rails = sys.power_rails || [];
  const engHtml = engines.length
    ? engines.map((e) =>
        `<div class="eng-chip ${e.online ? 'eng-on' : 'eng-off'}">
          <span class="eng-name">${escapeHtml(e.name)}</span>
          <span class="eng-meta">${e.online ? (e.freq_mhz ? e.freq_mhz + ' MHz' : 'on') : 'off'}</span>
        </div>`).join('')
    : '<span class="sys-muted">엔진 정보 없음</span>';
  const railHtml = rails.length
    ? rails.map((r) =>
        `<div class="sysm-kv"><span class="sysm-k">${escapeHtml(r.name)}</span><span class="sysm-v">${
          r.power_mw != null ? (r.power_mw/1000).toFixed(2) + ' W' : '—'
        }${r.avg_mw != null ? ` <small>(avg ${(r.avg_mw/1000).toFixed(2)}W)</small>` : ''}</span></div>`).join('')
    : '<span class="sys-muted">전력 rail 정보 없음</span>';
  return `<article class="sysm-card"><div class="sysm-label">하드웨어 엔진</div><div class="eng-grid">${engHtml}</div></article>
    <article class="sysm-card sysm-card-info"><div class="sysm-label">전력 Rail</div><div class="sysm-kv-list">${railHtml}</div></article>`;
}

// ── CTRL: Fan / NVP model / jetson_clocks ────────────────────────────────
function renderSysCtrl(sys) {
  const c = sys.control || {};
  const fan = c.fan || {};
  const fanPct = fan.speed_pct || 0;
  let html = '';
  if (fan.name) {
    html += sysMetricCard(`FAN (${escapeHtml(fan.name)})`, fanPct,
      `${fan.rpm != null ? `<span class="sysm-sub">${fan.rpm} RPM</span>` : ''}${fan.profile ? `<span class="sysm-sub">${escapeHtml(fan.profile)}</span>` : ''}`,
      null, fanPct >= 80 ? 'high' : fanPct >= 50 ? 'mid' : 'low');
  }
  const rows = [
    ['NV Power Model', c.nvpmodel || '—'],
    ['jetson_clocks', c.jetson_clocks === true ? 'ON' : c.jetson_clocks === false ? 'OFF' : '—'],
  ];
  html += `<article class="sysm-card sysm-card-info"><div class="sysm-kv-list">${
    rows.map(([k, v]) => {
      const cls = v === 'ON' ? 'usage-low' : v === 'OFF' ? 'usage-mid' : '';
      return `<div class="sysm-kv"><span class="sysm-k">${k}</span><span class="sysm-v ${cls}">${escapeHtml(v)}</span></div>`;
    }).join('')
  }</div></article>`;
  return html;
}

// ── INFO: 보드 / 플랫폼 / 라이브러리 ─────────────────────────────────────
function renderSysInfo(sys) {
  const board = sys.board || {};
  const section = (title, obj) => {
    const keys = Object.keys(obj || {});
    if (!keys.length) return '';
    return `<article class="sysm-card sysm-card-info"><div class="sysm-label">${title}</div><div class="sysm-kv-list sysm-kv-list--left">${
      keys.map((k) => `<div class="sysm-kv"><span class="sysm-k">${escapeHtml(k)}</span><span class="sysm-v">${escapeHtml(String(obj[k]))}</span></div>`).join('')
    }</div></article>`;
  };
  const out = section('하드웨어', board.hardware) + section('플랫폼', board.platform) + section('라이브러리', board.libraries);
  return out || `<article class="sysm-card sysm-card-info"><div class="sysm-info-row">
    <div class="sysm-stat"><span class="sysm-k">모델</span><span class="sysm-v">${escapeHtml((sys.model||'').replace('NVIDIA ',''))}</span></div>
    <div class="sysm-stat"><span class="sysm-k">JetPack</span><span class="sysm-v">${escapeHtml(sys.jetpack||'—')}</span></div>
  </div></article>`;
}

function formatUptime(seconds) {
  const h = Math.floor(seconds / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  return `${h}h ${m}m`;
}

// 카메라 슬롯 드롭다운은 항상 '논리(실시간) 토픽' 카탈로그를 보여준다 — 실시간/Play
// 두 모드에서 같은 슬롯 설정을 그대로 재사용하기 위함. 실제로 표시되는 스트림은
// activeCameraFor()가 현재 뷰 모드에 맞춰(effectiveTopic) 골라준다.
function cameraOptions() {
  const byName = new Map();
  for (const camera of state.cameras) {
    if (isPlayTopic(camera.name)) continue;
    byName.set(camera.name, camera);
  }
  for (const topic of state.topics) {
    if (isPlayTopic(topic.name)) continue;
    if (topic.type === 'sensor_msgs/msg/CompressedImage' || topic.name.includes('/object_3d/') && topic.name.includes('result_image')) {
      byName.set(topic.name, byName.get(topic.name) || { name: topic.name, age_sec: null, stream_url: `/stream?topic=${topic.name}` });
    }
  }
  return Array.from(byName.values()).sort((a, b) => a.name.localeCompare(b.name));
}

// 슬롯에 설정된 논리 토픽에 대해, 현재 뷰 모드(실시간/Play)에서 실제로 살아있는 카메라 항목을 찾는다.
function activeCameraFor(slot) {
  if (!slot || !slot.topic) return null;
  const want = effectiveTopic(slot.topic);
  return state.cameras.find((c) => c.name === want) || null;
}

function saveCameraLayout() {
  localStorage.setItem('rosWebCameraLayout', JSON.stringify(view.cameraLayout));
}

function applyCamGrid() {
  const cols = view.cameraLayout.cols || 2;
  const totalItems = view.cameraLayout.slots.length + (view.cameraEditMode ? 1 : 0);
  const rows = Math.max(1, Math.ceil(totalItems / cols));
  cameraGrid.style.setProperty('--cam-cols', cols);
  cameraGrid.style.setProperty('--cam-rows', rows);
  document.querySelectorAll('.cam-col-btn').forEach((btn) => {
    btn.classList.toggle('active', Number(btn.dataset.cols) === cols);
  });
}

let camDragId = null;

function buildCameraCard(slot, options) {
  const card = document.createElement('article');
  card.className = 'camera-card';
  card.dataset.id = slot.id;

  const optHtml = options.length
    ? options.map((o) => `<option value="${escapeHtml(o.name)}"${o.name === slot.topic ? ' selected' : ''}>${escapeHtml(o.name)}</option>`).join('')
    : '<option value="">카메라 없음</option>';

  card.innerHTML = `
    <div class="cam-card-head">
      <div class="cam-drag-handle" title="드래그로 순서 변경">⠿</div>
      <select>${optHtml}</select>
      <button class="cam-remove-btn" type="button" title="삭제">×</button>
    </div>
    <img alt="">
    <div class="camera-meta"><b></b><span></span></div>`;

  const select = card.querySelector('select');
  select.addEventListener('change', () => {
    slot.topic = select.value;
    saveCameraLayout();
    refreshCameraStream(card, slot, options);
  });

  // 삭제 버튼 (편집 모드에서만 표시)
  card.querySelector('.cam-remove-btn').addEventListener('click', () => {
    view.cameraLayout.slots = view.cameraLayout.slots.filter((s) => s.id !== slot.id);
    saveCameraLayout();
    rebuildCameraGrid(options);
  });

  // 드래그 핸들 (편집 모드 순서 변경)
  const dragHandle = card.querySelector('.cam-drag-handle');
  dragHandle.addEventListener('pointerdown', (e) => {
    if (!view.cameraEditMode) return;
    e.preventDefault();
    camDragId = slot.id;
    card.classList.add('cam-dragging');
    dragHandle.setPointerCapture(e.pointerId);
    let target = null;
    const onMove = (ev) => {
      const el = document.elementFromPoint(ev.clientX, ev.clientY);
      const overCard = el && el.closest('.camera-card');
      document.querySelectorAll('.camera-card').forEach((c) => c.classList.remove('cam-drop-over'));
      if (overCard && overCard !== card) { overCard.classList.add('cam-drop-over'); target = overCard; }
      else target = null;
    };
    const onUp = () => {
      dragHandle.releasePointerCapture(e.pointerId);
      dragHandle.removeEventListener('pointermove', onMove);
      dragHandle.removeEventListener('pointerup', onUp);
      card.classList.remove('cam-dragging');
      document.querySelectorAll('.camera-card').forEach((c) => c.classList.remove('cam-drop-over'));
      if (target && target.dataset.id !== slot.id) {
        const slots = view.cameraLayout.slots;
        const fromIdx = slots.findIndex((s) => s.id === slot.id);
        const toIdx = slots.findIndex((s) => s.id === target.dataset.id);
        if (fromIdx >= 0 && toIdx >= 0) {
          const [moved] = slots.splice(fromIdx, 1);
          slots.splice(toIdx, 0, moved);
          saveCameraLayout();
          rebuildCameraGrid(options);
        }
      }
      camDragId = null;
    };
    dragHandle.addEventListener('pointermove', onMove);
    dragHandle.addEventListener('pointerup', onUp);
  });

  refreshCameraStream(card, slot, options);
  return card;
}

function refreshCameraStream(card, slot, options) {
  const camera = activeCameraFor(slot);
  const img = card.querySelector('img');
  const title = card.querySelector('b');
  const age = card.querySelector('span');
  if (!camera || !slot.topic) {
    if (img.dataset.topic) { img.removeAttribute('src'); img.dataset.topic = ''; }
    title.textContent = slot.topic ? (slot.topic.split('/').filter(Boolean)[0] || slot.topic) : '—';
    age.textContent = slot.topic ? '수신 없음' : '';
    applyCameraSignalState(card, null, slot);
    return;
  }
  // 카메라 스트림은 보조 포트로 분리하고, 슬롯 순서대로 여러 포트에 나눠 접속
  // (브라우저의 호스트:포트당 동시연결 한도 6 → 7번째 슬롯부터 검은 화면 방지)
  const ports = state.streamPorts || [];
  const slotIdx = Math.max(0, view.cameraLayout.slots.findIndex((sl) => sl.id === slot.id));
  const port = ports.length ? ports[slotIdx % ports.length] : null;
  const base = port ? `${location.protocol}//${location.hostname}:${port}` : '';
  const streamUrl = `${base}${camera.stream_url}${camera.stream_url.includes('?') ? '&' : '?'}id=${slot.id}`;
  if (img.dataset.topic !== camera.name) {
    img.dataset.topic = camera.name;
    img.dataset.baseurl = streamUrl;
    startCameraStream(img, streamUrl);
  }
  title.textContent = camera.name.split('/').filter(Boolean).slice(-1)[0] || camera.name;
  age.textContent = camera.age_sec == null ? 'waiting' : `${camera.age_sec}s ago`;
  applyCameraSignalState(card, camera, slot);
}

// 카드 신호 상태 (CSS: .cam-empty / .cam-waiting / .cam-stale)
//  - 토픽 미지정 → empty
//  - 토픽 지정했지만 발행 안 됨, 또는 마지막 프레임이 오래됨 → stale('신호 없음')
const CAMERA_STALE_SEC = 3;
function applyCameraSignalState(card, camera, slot) {
  const empty = !slot.topic;
  const waiting = !!camera && camera.age_sec == null;
  const stale = !empty && (!camera || (!waiting && camera.age_sec > CAMERA_STALE_SEC));
  card.classList.toggle('cam-empty', empty);
  card.classList.toggle('cam-waiting', waiting);
  card.classList.toggle('cam-stale', stale);
}

// MJPEG 스트림 시작 + 실패(브라우저 연결한도/일시 끊김) 시 자동 재시도.
// 여러 카메라 동시 스트리밍 시 브라우저의 호스트당 동시연결 한도(~6)로
// 특정 카메라(예: /avm_cam/image_raw)가 안 뜨는 문제를 완화한다.
function startCameraStream(img, baseUrl) {
  if (img._camRetry) { clearTimeout(img._camRetry); img._camRetry = null; }
  const load = () => {
    if (img.dataset.baseurl !== baseUrl) return;  // 슬롯이 다른 토픽으로 바뀜
    img.src = ''; void img.offsetWidth;
    img.src = `${baseUrl}${baseUrl.includes('?') ? '&' : '?'}_t=${Date.now()}`;
  };
  img.onerror = () => {
    if (img.dataset.baseurl !== baseUrl) return;
    img._camRetry = setTimeout(load, 1500);  // 1.5초 후 재시도
  };
  load();
}

function rebuildCameraGrid(options) {
  cameraGrid.innerHTML = '';
  applyCamGrid();
  for (const slot of view.cameraLayout.slots) {
    cameraGrid.appendChild(buildCameraCard(slot, options));
  }
  // 편집 모드: 빈 슬롯 추가 버튼 (그리드 마지막)
  if (view.cameraEditMode) {
    const addCard = document.createElement('button');
    addCard.className = 'cam-add-card-btn';
    addCard.type = 'button';
    addCard.title = '카메라 추가';
    addCard.textContent = '＋';
    addCard.addEventListener('click', () => {
      view.cameraLayout.slots.push({ id: 'c' + Date.now(), topic: '' });
      saveCameraLayout();
      rebuildCameraGrid(options);
    });
    cameraGrid.appendChild(addCard);
  }
  cameraGrid.classList.toggle('cam-edit', view.cameraEditMode);
  document.getElementById('cameraEditBar').hidden = !view.cameraEditMode;
}

function renderCameras() {
  const options = cameraOptions();
  // view.mode를 시그니처에 포함시켜 실시간⇄Play 전환 시 스트림이 다시 매칭되게 함
  const sig = options.map((o) => o.name).join('|') + '|' + view.cameraLayout.slots.length + '|' + view.cameraEditMode + '|' + view.mode;
  if (sig !== view.cameraOptionSig) {
    view.cameraOptionSig = sig;
    rebuildCameraGrid(options);
  } else {
    // 스트림 age만 갱신
    for (const slot of view.cameraLayout.slots) {
      const card = cameraGrid.querySelector(`.camera-card[data-id="${CSS.escape(slot.id)}"]`);
      if (!card) continue;
      const camera = activeCameraFor(slot);
      const age = card.querySelector('span');
      if (age && camera) age.textContent = camera.age_sec == null ? 'waiting' : `${camera.age_sec}s ago`;
      applyCameraSignalState(card, camera, slot);
    }
  }
  cameraStatus.textContent = `${view.cameraLayout.slots.length}개 슬롯 · ${options.length}개 토픽`;
}

function renderTopics() {
  const query = searchInput.value.trim().toLowerCase();
  const group = groupFilter.value;
  const rows = currentModeTopics().filter((topic) => {
    if (activeOnly.checked && !topic.active) return false;
    if (group !== 'all' && groupForTopic(topic) !== group) return false;
    if (!query) return true;
    return topic.name.toLowerCase().includes(query) || topic.type.toLowerCase().includes(query);
  });
  shownCount.textContent = `${rows.length} shown`;
  if (!rows.length) { topicRows.innerHTML = '<tr><td class="empty" colspan="4">조건에 맞는 토픽이 없습니다.</td></tr>'; return; }
  const expanded = new Set([...topicRows.querySelectorAll('.topic-detail-row:not([hidden])')].map((r) => r.dataset.topic));
  topicRows.innerHTML = rows.map((topic) => {
    const subFail = topic.error && (topic.error.includes('Failed to create') || topic.error.includes('invalid allocator'));
    const status = (topic.error && !subFail) ? 'error' : topic.active ? 'active' : 'idle';
    const age = topic.age_sec === null ? '-' : `${topic.age_sec.toFixed(1)}s`;
    const tid = 'td-' + CSS.escape(topic.name.replace(/\//g, '_'));
    const isExpanded = expanded.has(topic.name);
    const typeShort = topic.type.split('/').pop();
    const firstSeenHtml = topic.first_seen_ago != null
      ? `<span class="td-label">First seen</span><span>${formatDuration(topic.first_seen_ago)} ago</span>` : '';
    const errorHtml = topic.error ? `<span class="td-label">Error</span><span class="td-error">${escapeHtml(topic.error)}</span>` : '';
    const lastValue = topic.last_value || {};
    const entries = Object.entries(lastValue);
    const paramHtml = entries.length
      ? `<div class="td-params">
          <div class="td-params-hd"><span>파라미터 이름</span><span>값 (실시간)</span></div>
          ${entries.map(([k, v]) =>
            `<div class="td-param-row"><span class="td-pk">${escapeHtml(k)}</span><span class="td-pv">${v === null ? '<i class="td-null">—</i>' : escapeHtml(String(v))}</span></div>`
          ).join('')}
        </div>`
      : `<div class="td-params"><div class="td-no-val">실시간 값 데이터 없음</div></div>`;
    return `<tr class="topic-row${isExpanded ? ' expanded' : ''}" data-topic="${escapeHtml(topic.name)}">
      <td><span class="pill ${status}">${status}</span></td>
      <td class="topic">${escapeHtml(topic.name)}<span class="type-line">${escapeHtml(topic.type)}</span></td>
      <td>${topic.rate_hz.toFixed(1)}</td><td>${age}</td>
    </tr>
    <tr class="topic-detail-row" data-topic="${escapeHtml(topic.name)}"${isExpanded ? '' : ' hidden'}>
      <td colspan="4" class="topic-detail">
        <div class="td-grid">
          <span class="td-label">Type</span><span class="td-mono">${escapeHtml(typeShort)}</span>
          <span class="td-label">Msgs</span><span>${topic.total.toLocaleString()}</span>
          ${firstSeenHtml}
          <span class="td-label">Rate</span><span>${topic.rate_hz.toFixed(2)} Hz</span>
          ${errorHtml}
        </div>
        ${paramHtml}
      </td>
    </tr>`;
  }).join('');
}

// 토픽 행 클릭은 이벤트 위임(1회 바인딩) — 매 렌더마다 수백 개 리스너 재바인딩 제거
topicRows.addEventListener('click', (e) => {
  const row = e.target.closest('.topic-row');
  if (!row) return;
  const detail = topicRows.querySelector(`.topic-detail-row[data-topic="${CSS.escape(row.dataset.topic)}"]`);
  if (detail) { detail.hidden = !detail.hidden; row.classList.toggle('expanded', !detail.hidden); }
});

function formatDuration(sec) {
  const s = Math.floor(sec);
  if (s < 60) return `${s}s`;
  if (s < 3600) return `${Math.floor(s / 60)}m ${s % 60}s`;
  return `${Math.floor(s / 3600)}h ${Math.floor((s % 3600) / 60)}m`;
}

function setZoom(value) {
  view.zoom = Math.max(0.45, Math.min(30.0, Number(value)));
  zoomRange.value = view.zoom.toFixed(2);
  renderVisualization();
}

function setViewPreset(name) {
  const dz = view.vizSettings.defaultZoom || 1;
  const presets = {
    top: { yaw: 0, pitch: Math.PI / 2, zoom: dz * 1.05 },
    front: { yaw: 0, pitch: 0.02, zoom: dz },
    left: { yaw: Math.PI / 2, pitch: 0.02, zoom: dz },
    right: { yaw: -Math.PI / 2, pitch: 0.02, zoom: dz },
    reset: { yaw: 0, pitch: 1.08, zoom: dz },
  };
  Object.assign(view, presets[name] || presets.reset);
  view.panX = 0; view.panY = 0;  // 이동(pan) 초기화
  setZoom(view.zoom);
}

function setLayout(value, save = true) {
  view.layout = value;
  dashboard.className = `dashboard layout-${value}`;
  if (layoutMode) layoutMode.value = value;
  if (layoutName) layoutName.textContent = LAYOUT_NAMES[value] || value;
  if (save) localStorage.setItem('rosWebLayout', value);
  applyPanelSizes();
  setTimeout(renderVisualization, 0);
  setTimeout(refreshGpsMap, 60);
}

function setupPanelDragging() {
  document.querySelectorAll('.panel[draggable="true"]').forEach((panel) => {
    panel.addEventListener('dragstart', (event) => {
      if (!view.editLayout) { event.preventDefault(); return; }
      event.dataTransfer.setData('text/plain', panel.dataset.panel);
      panel.classList.add('dragging');
    });
    panel.addEventListener('dragend', () => panel.classList.remove('dragging'));
  });
  document.querySelectorAll('.panel-zone').forEach((zone) => {
    zone.addEventListener('dragover', (event) => {
      if (!view.editLayout) return;
      event.preventDefault();
      zone.classList.add('drag-over');
      const dragged = document.querySelector('.panel.dragging');
      if (!dragged) return;
      const after = panelAfterPointer(zone, event.clientY);
      if (after) zone.insertBefore(dragged, after);
      else zone.appendChild(dragged);
    });
    zone.addEventListener('dragleave', () => zone.classList.remove('drag-over'));
    zone.addEventListener('drop', () => {
      if (!view.editLayout) return;
      zone.classList.remove('drag-over');
      saveCustomLayout();
    });
  });
}

function panelAfterPointer(zone, y) {
  const panels = [...zone.querySelectorAll('.panel:not(.dragging)')];
  return panels.reduce((closest, child) => {
    const rect = child.getBoundingClientRect();
    const offset = y - rect.top - rect.height / 2;
    if (offset < 0 && offset > closest.offset) return { offset, element: child };
    return closest;
  }, { offset: Number.NEGATIVE_INFINITY, element: null }).element;
}

function saveCustomLayout() {
  const layout = {};
  document.querySelectorAll('.panel-zone').forEach((zone) => {
    layout[zone.dataset.zone] = [...zone.querySelectorAll('.panel')].map((panel) => panel.dataset.panel);
  });
  view.customLayout = layout;
  setLayout('custom', false);
}

// ── 탭 드래그 상태
let tabDragId = null;

function renderLeftTabBar() {
  const nav = document.getElementById('leftTabs');
  if (!nav) return;
  nav.innerHTML = view.leftTabConfig.map((t) => `
    <button class="left-tab${t.id === view.leftTab ? ' active' : ''}" data-tab="${t.id}" type="button">
      <span class="left-tab-drag" data-tab="${t.id}">⠿</span>
      <span>${t.label}</span>
    </button>`).join('');
  nav.querySelectorAll('.left-tab').forEach((btn) => {
    btn.addEventListener('click', (e) => {
      if (e.target.closest('.left-tab-drag')) return;
      switchLeftTab(btn.dataset.tab);
    });
  });
  nav.querySelectorAll('.left-tab-drag').forEach((handle) => {
    handle.addEventListener('pointerdown', (e) => {
      if (!view.editLayout) return;
      e.preventDefault();
      const btn = handle.closest('.left-tab');
      tabDragId = btn.dataset.tab;
      btn.classList.add('tab-dragging');
      handle.setPointerCapture(e.pointerId);
      let dropTarget = null;
      const onMove = (ev) => {
        const el = document.elementFromPoint(ev.clientX, ev.clientY);
        const overBtn = el && el.closest('.left-tab');
        nav.querySelectorAll('.left-tab').forEach((b) => b.classList.remove('tab-drop-over'));
        if (overBtn && overBtn !== btn) { overBtn.classList.add('tab-drop-over'); dropTarget = overBtn; }
        else dropTarget = null;
      };
      const onUp = () => {
        handle.releasePointerCapture(e.pointerId);
        handle.removeEventListener('pointermove', onMove);
        handle.removeEventListener('pointerup', onUp);
        btn.classList.remove('tab-dragging');
        nav.querySelectorAll('.left-tab').forEach((b) => b.classList.remove('tab-drop-over'));
        if (dropTarget) {
          const fromId = tabDragId, toId = dropTarget.dataset.tab;
          const cfg = view.leftTabConfig;
          const fi = cfg.findIndex((t) => t.id === fromId);
          const ti = cfg.findIndex((t) => t.id === toId);
          if (fi >= 0 && ti >= 0) { const [m] = cfg.splice(fi, 1); cfg.splice(ti, 0, m); }
          localStorage.setItem('rosWebLeftTabConfig', JSON.stringify(cfg));
          renderLeftTabBar();
        }
        tabDragId = null;
      };
      handle.addEventListener('pointermove', onMove);
      handle.addEventListener('pointerup', onUp);
    });
  });
}

function switchLeftTab(tab) {
  view.leftTab = tab;
  localStorage.setItem('rosWebLeftTab', tab);
  renderLeftTabBar();
  document.querySelectorAll('.left-rail .panel').forEach((panel) =>
    panel.classList.toggle('tab-inactive', !view.editLayout && panel.dataset.panel !== tab));
  const leftRail = document.querySelector('.left-rail');
  if (leftRail && !view.editLayout) leftRail.style.gridTemplateRows = 'auto minmax(0, 1fr)';
  // 탭 전환 즉시 해당 패널 렌더 (heavy 렌더는 보이는 탭만 돌므로 stale 방지)
  if (tab === 'topics') renderTopics();
  else if (tab === 'groups') renderGroups();
  else if (tab === 'system') renderSystem();
}

function enterEditMode() {
  view.editLayout = true;
  view.cameraEditMode = true;
  view.cameraOptionSig = '';
  // 편집 모드: 모든 패널 표시, grid-row 복원
  document.querySelectorAll('.left-rail .panel').forEach((p) => p.classList.remove('tab-inactive'));
  view.layoutBeforeEdit = view.layout;
  view.customLayoutBeforeEdit = view.customLayout ? JSON.stringify(view.customLayout) : null;
  view.panelOrderBeforeEdit = {};
  document.querySelectorAll('.panel-zone').forEach((zone) => {
    view.panelOrderBeforeEdit[zone.dataset.zone] = [...zone.querySelectorAll('.panel')].map((p) => p.dataset.panel);
  });
  view.colWidthBeforeEdit = dashboard.style.gridTemplateColumns;
  const leftRailEl = document.querySelector('.left-rail');
  const rightStageEl = document.querySelector('.right-stage');
  view.leftRowsBeforeEdit = leftRailEl ? leftRailEl.style.gridTemplateRows : '';
  view.rightRowsBeforeEdit = rightStageEl ? rightStageEl.style.gridTemplateRows : '';
  view.rightColsBeforeEdit = rightStageEl ? rightStageEl.style.gridTemplateColumns : '';
  if (leftRailEl) leftRailEl.style.gridTemplateRows = '';
  document.body.classList.add('layout-edit');
  applyPanelSizes();
  addPanelGrips();   // 패널마다 독립 크기조절 그립
  updateColResizeHandle();
  vehEditRendered = false; renderVehicle();  // Vehicle 서브카드 편집 UI 표시
  renderSystem();  // 시스템 서브탭 편집 UI(드래그/이름변경) 표시
}

function exitEditMode(save) {
  view.editLayout = false;
  view.cameraEditMode = false;
  view.cameraOptionSig = '';
  document.body.classList.remove('layout-edit');
  vehEditRendered = false;     // Vehicle 일반 렌더로 복귀
  switchLeftTab(view.leftTab); // 탭 상태 복원
  document.querySelectorAll('.row-resize-handle').forEach((h) => h.remove());
  removePanelGrips();
  if (save) {
    // 패널을 영역 간 이동(custom)했을 때만 custom으로 고정, 아니면 현재 레이아웃 유지
    if (view.layout === 'custom') captureResizeSizes();
    localStorage.setItem('rosWebLayout', view.layout);
    if (view.customLayout) localStorage.setItem('rosWebCustomLayout', JSON.stringify(view.customLayout));
    const leftRailEl = document.querySelector('.left-rail');
    const rightStageEl = document.querySelector('.right-stage');
    if (dashboard.style.gridTemplateColumns) localStorage.setItem('rosWebColWidth', dashboard.style.gridTemplateColumns);
    if (leftRailEl && leftRailEl.style.gridTemplateRows) localStorage.setItem('rosWebLeftRows', leftRailEl.style.gridTemplateRows);
    if (rightStageEl && rightStageEl.style.gridTemplateRows) localStorage.setItem('rosWebRightRows', rightStageEl.style.gridTemplateRows);
    if (rightStageEl && rightStageEl.style.gridTemplateColumns) localStorage.setItem('rosWebRightCols', rightStageEl.style.gridTemplateColumns);
  } else {
    dashboard.style.gridTemplateColumns = view.colWidthBeforeEdit;
    const leftRailEl = document.querySelector('.left-rail');
    const rightStageEl = document.querySelector('.right-stage');
    if (leftRailEl) leftRailEl.style.gridTemplateRows = view.leftRowsBeforeEdit;
    if (rightStageEl) rightStageEl.style.gridTemplateRows = view.rightRowsBeforeEdit;
    if (rightStageEl) rightStageEl.style.gridTemplateColumns = view.rightColsBeforeEdit;
    view.customLayout = view.customLayoutBeforeEdit ? JSON.parse(view.customLayoutBeforeEdit) : null;
    setLayout(view.layoutBeforeEdit, false);
    if (view.panelOrderBeforeEdit) {
      const panels = new Map([...document.querySelectorAll('.panel')].map((p) => [p.dataset.panel, p]));
      for (const [zoneName, names] of Object.entries(view.panelOrderBeforeEdit)) {
        const zone = document.querySelector(`[data-zone="${zoneName}"]`);
        if (!zone) continue;
        for (const name of names) { const p = panels.get(name); if (p) zone.appendChild(p); }
      }
    }
  }
  renderSystem();  // 시스템 서브탭 편집 UI 해제
  renderVehicle(); // Vehicle 일반 표시 즉시 복귀
}

function applyStoredLayout() {
  const savedCol = localStorage.getItem('rosWebColWidth');
  const savedLeft = localStorage.getItem('rosWebLeftRows');
  const savedRight = localStorage.getItem('rosWebRightRows');
  const savedRightCols = localStorage.getItem('rosWebRightCols');
  if (savedCol) dashboard.style.gridTemplateColumns = savedCol;
  const leftRailEl = document.querySelector('.left-rail');
  const rightStageEl = document.querySelector('.right-stage');
  if (savedLeft && leftRailEl) leftRailEl.style.gridTemplateRows = savedLeft;
  if (savedRight && rightStageEl) rightStageEl.style.gridTemplateRows = savedRight;
  if (savedRightCols && rightStageEl) rightStageEl.style.gridTemplateColumns = savedRightCols;
  if (!view.customLayout) return;
  const panels = new Map([...document.querySelectorAll('.panel')].map((panel) => [panel.dataset.panel, panel]));
  for (const [zoneName, names] of Object.entries(view.customLayout)) {
    if (!Array.isArray(names)) continue;
    const zone = document.querySelector(`[data-zone="${zoneName}"]`);
    if (!zone) continue;
    for (const name of names) {
      const panel = panels.get(name);
      if (panel) zone.appendChild(panel);
    }
  }
}

function captureResizeSizes() {
  const layout = view.customLayout || {};
  document.querySelectorAll('.panel-zone').forEach((zone) => {
    layout[zone.dataset.zone] = [...zone.querySelectorAll('.panel')].map((p) => p.dataset.panel);
  });
  view.customLayout = layout;
  setLayout('custom', false);
}

function escapeHtml(value) {
  return String(value).replaceAll('&', '&amp;').replaceAll('<', '&lt;').replaceAll('>', '&gt;').replaceAll('"', '&quot;').replaceAll("'", '&#039;');
}

/* ── Column resize ──────────────────────────────────── */
let colResizing = false;
let colResizeStartX = 0;
let colResizeInitWidth = 0;

const colHandle = document.getElementById('colResizeHandle');

function updateColResizeHandle() {
  const leftRailEl = document.querySelector('.left-rail');
  if (!leftRailEl || !colHandle) return;
  const dashRect = dashboard.getBoundingClientRect();
  const railRect = leftRailEl.getBoundingClientRect();
  colHandle.style.left = `${railRect.right - dashRect.left}px`;
}

colHandle.addEventListener('pointerdown', (e) => {
  if (!view.editLayout) return;
  colResizing = true;
  colResizeStartX = e.clientX;
  colResizeInitWidth = document.querySelector('.left-rail').getBoundingClientRect().width;
  colHandle.setPointerCapture(e.pointerId);
  colHandle.classList.add('active');
  e.preventDefault();
});
colHandle.addEventListener('pointermove', (e) => {
  if (!colResizing) return;
  const dashW = dashboard.getBoundingClientRect().width;
  const newW = Math.max(240, Math.min(dashW * 0.65, colResizeInitWidth + (e.clientX - colResizeStartX)));
  dashboard.style.gridTemplateColumns = `${Math.round(newW)}px minmax(0, 1fr)`;
  updateColResizeHandle();
  refreshGpsMap();
});
colHandle.addEventListener('pointerup', () => {
  colResizing = false;
  colHandle.classList.remove('active');
});

/* ── Row resize ─────────────────────────────────────── */
function addRowResizeHandles() {
  document.querySelectorAll('.row-resize-handle').forEach((h) => h.remove());
  document.querySelectorAll('.panel-zone').forEach((zone) => {
    const panels = [...zone.querySelectorAll('.panel')];
    panels.forEach((panel, i) => {
      if (i === panels.length - 1) return;
      const handle = document.createElement('div');
      handle.className = 'row-resize-handle';
      panel.appendChild(handle);
      let resizing = false;
      let startY = 0;
      let aboveH = 0;
      let belowH = 0;
      const belowPanel = panels[i + 1];
      handle.addEventListener('dragstart', (e) => e.preventDefault());
      handle.addEventListener('pointerdown', (e) => {
        if (!view.editLayout) return;
        resizing = true;
        startY = e.clientY;
        aboveH = panel.getBoundingClientRect().height;
        belowH = belowPanel.getBoundingClientRect().height;
        handle.setPointerCapture(e.pointerId);
        handle.classList.add('active');
        e.preventDefault();
        e.stopPropagation();
      });
      handle.addEventListener('pointermove', (e) => {
        if (!resizing) return;
        const dy = e.clientY - startY;
        const newAbove = Math.max(80, aboveH + dy);
        const newBelow = Math.max(80, belowH - dy);
        const allPanels = [...zone.querySelectorAll('.panel')];
        const heights = allPanels.map((p, idx) => {
          if (p === panel) return newAbove;
          if (p === belowPanel) return newBelow;
          return p.getBoundingClientRect().height;
        });
        zone.style.gridTemplateRows = heights.map((h) => `${Math.round(h)}px`).join(' ');
      });
      handle.addEventListener('pointerup', () => { resizing = false; handle.classList.remove('active'); });
    });
  });
}

/* ── 우측 패널별 리사이즈 그립(편집 모드: 패널마다 독립 크기 조절) ── */
function removePanelGrips() {
  document.querySelectorAll('.panel-grip').forEach((g) => g.remove());
}
function addPanelGrips() {
  removePanelGrips();
  const rs = document.querySelector('.right-stage');
  if (!rs) return;
  rs.querySelectorAll('.panel').forEach((p) => {
    const name = p.dataset.panel;
    if (!PANEL_SIZE_DEFAULTS[name]) return;
    const grip = document.createElement('div');
    grip.className = 'panel-grip';
    grip.title = '드래그로 이 패널 크기 조절';
    p.appendChild(grip);
    bindPanelGrip(grip, p, name);
  });
}
function bindPanelGrip(grip, panel, name) {
  grip.addEventListener('dragstart', (e) => e.preventDefault());
  grip.addEventListener('pointerdown', (e) => {
    e.preventDefault(); e.stopPropagation();
    const rs = document.querySelector('.right-stage');
    if (!rs) return;
    grip.setPointerCapture(e.pointerId);
    const onMove = (ev) => {
      const rsRect = rs.getBoundingClientRect();
      const pr = panel.getBoundingClientRect();
      const wPct = Math.max(20, Math.min(100, (ev.clientX - pr.left) / rsRect.width * 100));
      const hPct = Math.max(14, Math.min(100, (ev.clientY - pr.top) / rsRect.height * 100));
      panelSizes[name] = { w: Math.round(wPct), h: Math.round(hPct) };
      panel.style.width = `calc(${Math.round(wPct)}% - 6px)`;
      panel.style.height = `${Math.round(hPct)}%`;
      if (name === 'rviz') renderVisualization();
      if (name === 'gps') refreshGpsMap();
    };
    const onUp = () => {
      grip.releasePointerCapture(e.pointerId);
      grip.removeEventListener('pointermove', onMove);
      grip.removeEventListener('pointerup', onUp);
      savePanelSizes();
      renderVisualization();
      refreshGpsMap();
    };
    grip.addEventListener('pointermove', onMove);
    grip.addEventListener('pointerup', onUp);
  });
}

window.addEventListener('resize', updateColResizeHandle);

// 왼쪽 탭 초기 렌더 + 활성화
renderLeftTabBar();
switchLeftTab(view.leftTab);

let searchDebounce = null;   // 타이핑마다 242행 재구성 방지
searchInput.addEventListener('input', () => {
  clearTimeout(searchDebounce);
  searchDebounce = setTimeout(renderTopics, 150);
});
groupFilter.addEventListener('change', renderTopics);
activeOnly.addEventListener('change', renderTopics);
window.addEventListener('resize', renderVisualization);
window.addEventListener('resize', refreshGpsMap);
document.querySelectorAll('.cam-col-btn').forEach((btn) => {
  btn.addEventListener('click', () => {
    view.cameraLayout.cols = Number(btn.dataset.cols);
    saveCameraLayout();
    applyCamGrid();
    view.cameraOptionSig = '';
    renderCameras();
  });
});
$('#camAddSlot').addEventListener('click', () => {
  view.cameraLayout.slots.push({ id: 'c' + Date.now(), topic: '' });
  saveCameraLayout();
  view.cameraOptionSig = '';
  renderCameras();
});
$('#doneCameraLayout').addEventListener('click', () => exitEditMode(true));
$('#editLayout').addEventListener('click', enterEditMode);
$('#saveLayout').addEventListener('click', () => exitEditMode(true));
$('#cancelLayout').addEventListener('click', () => exitEditMode(false));
layoutMode.addEventListener('change', () => { if (view.editLayout) { setLayout(layoutMode.value, false); applyPanelSizes(); addPanelGrips(); } });
zoomRange.addEventListener('input', () => setZoom(zoomRange.value));
$('#zoomIn').addEventListener('click', () => setZoom(view.zoom + 0.15));
$('#zoomOut').addEventListener('click', () => setZoom(view.zoom - 0.15));
$('#viewReset').addEventListener('click', () => setViewPreset('reset'));
document.querySelectorAll('.view-preset').forEach((button) => button.addEventListener('click', () => setViewPreset(button.dataset.view)));
vizCanvas.addEventListener('pointerdown', (event) => {
  view.dragging = true;
  // 가운데 휠 버튼(1) = 이동(pan), 그 외(왼쪽 0 등) = 회전(rotate)
  view.dragMode = (event.button === 1) ? 'pan' : 'rotate';
  if (event.button === 1) event.preventDefault();  // 가운데 버튼 자동스크롤 방지
  view.lastX = event.clientX; view.lastY = event.clientY;
  vizCanvas.setPointerCapture(event.pointerId);
});
vizCanvas.addEventListener('pointermove', (event) => {
  if (!view.dragging) return;
  const dx = event.clientX - view.lastX;
  const dy = event.clientY - view.lastY;
  view.lastX = event.clientX;
  view.lastY = event.clientY;
  if (view.dragMode === 'pan') {
    view.panX += dx;   // 좌/우 이동
    view.panY += dy;   // 상/하 이동
  } else {
    view.yaw += dx * 0.008;
    view.pitch = Math.max(0.02, Math.min(Math.PI / 2, view.pitch - dy * 0.006));
  }
  renderVisualization();
});
vizCanvas.addEventListener('pointerup', () => { view.dragging = false; });
// 가운데 버튼 클릭 시 브라우저 기본 자동스크롤 차단
vizCanvas.addEventListener('auxclick', (event) => { if (event.button === 1) event.preventDefault(); });
vizCanvas.addEventListener('wheel', (event) => { event.preventDefault(); setZoom(view.zoom + (event.deltaY < 0 ? 0.14 : -0.14)); }, { passive: false });


// 네이티브 confirm/alert이 차단된(키오스크/웹뷰) 환경에서도 동작하는 자체 모달
function vdConfirm(message) {
  return new Promise((resolve) => {
    const ov = document.createElement('div');
    ov.className = 'vd-dialog-overlay';
    ov.innerHTML = `<div class="vd-dialog">
      <div class="vd-dialog-msg">${escapeHtml(message).replace(/\n/g, '<br>')}</div>
      <div class="vd-dialog-btns">
        <button type="button" class="vd-dialog-ok">확인</button>
        <button type="button" class="vd-dialog-cancel">취소</button>
      </div></div>`;
    document.body.appendChild(ov);
    const done = (v) => { ov.remove(); resolve(v); };
    ov.querySelector('.vd-dialog-ok').addEventListener('click', () => done(true));
    ov.querySelector('.vd-dialog-cancel').addEventListener('click', () => done(false));
    ov.addEventListener('click', (e) => { if (e.target === ov) done(false); });
  });
}
function vdNotice(message) {
  const ov = document.createElement('div');
  ov.className = 'vd-dialog-overlay';
  ov.innerHTML = `<div class="vd-dialog">
    <div class="vd-dialog-msg">${escapeHtml(String(message)).replace(/\n/g, '<br>')}</div>
    <div class="vd-dialog-btns"><button type="button" class="vd-dialog-ok">확인</button></div></div>`;
  document.body.appendChild(ov);
  const close = () => ov.remove();
  ov.querySelector('.vd-dialog-ok').addEventListener('click', close);
  ov.addEventListener('click', (e) => { if (e.target === ov) close(); });
}

// 카메라 MJPEG 스트림 + SSE 가 브라우저의 호스트당 동시연결(~6)을 점유하면
// POST 요청이 슬롯을 못 잡아 간헐적으로 막힌다. 곧 종료/재부팅하므로 안전하게 연결을 비운다.
function freeHttpConnections() {
  try {
    document.querySelectorAll('.camera-card img').forEach((img) => {
      if (img._camRetry) { clearTimeout(img._camRetry); img._camRetry = null; }
      img.onerror = null; img.removeAttribute('src'); img.dataset.topic = '';
    });
    if (typeof events !== 'undefined' && events && events.close) events.close();
  } catch (_) {}
}

async function systemPower(action) {
  const label = action === 'shutdown' ? '종료' : '재부팅';
  const cmd = action === 'shutdown' ? 'shutdown -h now' : 'reboot';
  if (!(await vdConfirm(`정말 시스템을 ${label}하시겠습니까?\n\n명령: ${cmd}`))) return;
  clock.textContent = `${label} 요청 중…`;
  freeHttpConnections();            // 연결 슬롯 확보
  await new Promise((r) => setTimeout(r, 150));  // 연결 정리 대기
  // 타임아웃 + 재시도로 확실히 전달
  for (let attempt = 1; attempt <= 4; attempt++) {
    try {
      const ctrl = new AbortController();
      const timer = setTimeout(() => ctrl.abort(), 4000);
      const res = await fetch(`/api/system/${action}`, { method: 'POST', signal: ctrl.signal, cache: 'no-store' });
      clearTimeout(timer);
      const data = await res.json();
      if (data && data.ok === false) {
        clock.textContent = '';
        vdNotice(`${label} 실패:\n${data.error || '권한 부족 — passwordless sudo 설정이 필요할 수 있습니다.'}`);
      } else {
        clock.textContent = `${label} 중…`;
      }
      return;
    } catch (e) {
      // abort/네트워크 끊김: 종료가 시작돼 응답 전에 끊겼을 수도, 슬롯을 못 잡았을 수도 있음
      if (attempt >= 4) {
        // 마지막까지 실패 → 종료가 진행 중일 가능성이 높으므로 진행 표시
        clock.textContent = `${label} 중…`;
        return;
      }
      await new Promise((r) => setTimeout(r, 400));  // 잠시 후 재시도
    }
  }
}
$('#btnShutdown').addEventListener('click', () => systemPower('shutdown'));
$('#btnReboot').addEventListener('click', () => systemPower('reboot'));
$('#gpsRecenter').addEventListener('click', recenterGps);
$('#gpsTrackRec').addEventListener('click', toggleGpsTrackRec);
$('#gpsTrackLoad').addEventListener('click', showGpsTrackMenu);
document.addEventListener('click', (e) => {
  const menu = document.getElementById('gpsTrackMenu');
  if (menu && !menu.hidden && !menu.contains(e.target) && e.target.id !== 'gpsTrackLoad') menu.hidden = true;
});
bagToggle.addEventListener('click', toggleBag);
document.querySelectorAll('.viewmode-btn').forEach((btn) => btn.addEventListener('click', () => setViewMode(btn.dataset.mode)));
setViewMode(view.mode);  // 저장된 모드로 탭 활성 표시 초기화
if (bagPlayToggle) bagPlayToggle.addEventListener('click', openPlayModal);
const playModalCloseBtn = $('#playModalClose');
const playOverlayEl = $('#playOverlay');
const playModalCancelBtn = $('#playModalCancelBtn');
const playStopBtnEl = $('#playStopBtn');
const playBannerStopBtn = $('#playBannerStop');
if (playModalCloseBtn) playModalCloseBtn.addEventListener('click', closePlayModal);
if (playOverlayEl) playOverlayEl.addEventListener('click', closePlayModal);
if (playModalCancelBtn) playModalCancelBtn.addEventListener('click', closePlayModal);
if (playStopBtnEl) playStopBtnEl.addEventListener('click', async () => { await stopBagPlay(); updatePlayModalFoot(); renderPlayFileList(); });
if (playBannerStopBtn) playBannerStopBtn.addEventListener('click', stopBagPlay);
const bagDirBrowseBtn = $('#bagDirBrowseBtn');
if (bagDirBrowseBtn) bagDirBrowseBtn.addEventListener('click', openDirBrowser);
const dirBrowseCloseBtn = $('#dirBrowseClose');
const dirBrowseOverlayEl = $('#dirBrowseOverlay');
const dirBrowseCancelBtn = $('#dirBrowseCancelBtn');
const dirBrowseSelectBtnEl = $('#dirBrowseSelectBtn');
const dirBrowseNewBtn = $('#dirBrowseNewBtn');
if (dirBrowseCloseBtn) dirBrowseCloseBtn.addEventListener('click', closeDirBrowser);
if (dirBrowseOverlayEl) dirBrowseOverlayEl.addEventListener('click', closeDirBrowser);
if (dirBrowseCancelBtn) dirBrowseCancelBtn.addEventListener('click', closeDirBrowser);
if (dirBrowseNewBtn) dirBrowseNewBtn.addEventListener('click', createDirBrowseFolder);
if (dirBrowseSelectBtnEl) dirBrowseSelectBtnEl.addEventListener('click', () => {
  const dirInput = document.getElementById('bagDirInput');
  if (dirInput) dirInput.value = dirBrowsePath;
  closeDirBrowser();
});
// ── 재생(Play) 배너: 일시정지/다음프레임/시크바/배속 ──
const playPauseBtnEl = $('#playPauseBtn');
const playNextBtnEl = $('#playNextBtn');
const playSeekBarEl = $('#playSeekBar');
const playRateSelEl = $('#playRateSel');
if (playPauseBtnEl) playPauseBtnEl.addEventListener('click', togglePlayPause);
if (playNextBtnEl) playNextBtnEl.addEventListener('click', playNextFrame);
if (playRateSelEl) playRateSelEl.addEventListener('change', () => setPlayRate(playRateSelEl.value));
if (playSeekBarEl) {
  playSeekBarEl.addEventListener('pointerdown', () => { playSeekDragging = true; });
  playSeekBarEl.addEventListener('input', () => {
    const curEl = document.getElementById('playTimeCur');
    if (curEl) curEl.textContent = fmtClock(Number(playSeekBarEl.value));
  });
  playSeekBarEl.addEventListener('change', async () => {
    await seekBagPlay(Number(playSeekBarEl.value));
    playSeekDragging = false;
  });
}
$('#headerSettings').addEventListener('click', openSettingsModal);
$('#bagModalClose').addEventListener('click', () => closeSettingsModal(false));
$('#bagOverlay').addEventListener('click', () => closeSettingsModal(false));
$('#bagModalCancel').addEventListener('click', () => closeSettingsModal(false));
$('#bagModalSave').addEventListener('click', () => closeSettingsModal(true));
document.querySelectorAll('.set-tab').forEach((b) => b.addEventListener('click', () => switchSettingsTab(b.dataset.settab)));
$('#headerReset').addEventListener('click', () => {
  headerEdit = HEADER_DEFAULTS.map((x) => ({ ...x }));
  renderHeaderItemList();
});
$('#bagAll').addEventListener('change', (e) => {
  bagEdit.all = e.target.checked;
  document.getElementById('bagPicker').classList.toggle('disabled', bagEdit.all);
  renderBagTopicList(document.getElementById('bagTopicSearch').value);
});
$('#bagTopicSearch').addEventListener('input', (e) => renderBagTopicList(e.target.value));
$('#bagSelectAll').addEventListener('click', () => {
  if (bagEdit.all) return;
  state.topics.forEach((t) => { if (!isPlayTopic(t.name)) bagEdit.topics.add(t.name); });
  renderBagTopicList(document.getElementById('bagTopicSearch').value);
});
$('#bagSelectNone').addEventListener('click', () => {
  if (bagEdit.all) return;
  bagEdit.topics.clear();
  renderBagTopicList(document.getElementById('bagTopicSearch').value);
});
// RViz 패널의 ⚙ 버튼은 제거됨 — RViz 설정은 상단 ⚙ 설정 → RViz 탭에서 변경

// ── 설정 백업(export/import) ──
const DEFAULT_SNAPSHOT_KEY = 'rosWebDefaultSnapshot';

function renderBackupInfo() {
  const el = document.getElementById('backupInfo');
  if (!el) return;
  let n = 0;
  for (let i = 0; i < localStorage.length; i++) {
    const k = localStorage.key(i);
    if (k && k.startsWith('rosWeb') && k !== DEFAULT_SNAPSHOT_KEY) n += 1;
  }
  let dline = '저장된 기본값: 없음';
  const snap = localStorage.getItem(DEFAULT_SNAPSHOT_KEY);
  if (snap) {
    try {
      const o = JSON.parse(snap);
      const cnt = Object.keys(o.settings || {}).length;
      const when = o.ts ? o.ts.replace('T', ' ').slice(0, 19) : '있음';
      dline = `저장된 기본값: ${when} (${cnt}개 항목)`;
    } catch (_) { dline = '저장된 기본값: 있음'; }
  }
  el.textContent = `현재 설정 항목: ${n}개\n${dline}`;
}

// 현재 설정 전체를 "기본값" 스냅샷으로 저장
function saveCurrentAsDefault() {
  const data = {};
  for (let i = 0; i < localStorage.length; i++) {
    const k = localStorage.key(i);
    if (k && k.startsWith('rosWeb') && k !== DEFAULT_SNAPSHOT_KEY) data[k] = localStorage.getItem(k);
  }
  localStorage.setItem(DEFAULT_SNAPSHOT_KEY, JSON.stringify({ ts: new Date().toISOString(), settings: data }));
  renderBackupInfo();
  vdNotice(`현재 설정을 기본값으로 저장했습니다. (${Object.keys(data).length}개 항목)`);
}

// 저장해둔 기본값으로 되돌리기
async function restoreDefault() {
  const raw = localStorage.getItem(DEFAULT_SNAPSHOT_KEY);
  if (!raw) { vdNotice('저장된 기본값이 없습니다.\n먼저 "현재 설정을 기본값으로 저장"을 누르세요.'); return; }
  if (!(await vdConfirm('저장해둔 기본값으로 되돌립니다.\n현재 설정은 모두 덮어써집니다. 계속할까요?'))) return;
  try {
    const obj = JSON.parse(raw);
    const settings = obj.settings || {};
    // 기존 rosWeb* 제거(스냅샷 키 제외) 후 기본값 적용
    const toRemove = [];
    for (let i = 0; i < localStorage.length; i++) {
      const k = localStorage.key(i);
      if (k && k.startsWith('rosWeb') && k !== DEFAULT_SNAPSHOT_KEY) toRemove.push(k);
    }
    toRemove.forEach((k) => localStorage.removeItem(k));
    Object.keys(settings).forEach((k) => {
      if (typeof settings[k] === 'string') localStorage.setItem(k, settings[k]);
    });
    location.reload();
  } catch (e) {
    vdNotice('기본값 복원 실패: ' + e.message);
  }
}
function exportSettings() {
  const data = {};
  for (let i = 0; i < localStorage.length; i++) {
    const k = localStorage.key(i);
    if (k && k.startsWith('rosWeb')) data[k] = localStorage.getItem(k);
  }
  const payload = { _meta: { app: 'ros_web_monitor', version: 1, ts: new Date().toISOString() }, settings: data };
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  const stamp = new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-');
  a.href = url; a.download = `ros_web_monitor_settings_${stamp}.json`;
  document.body.appendChild(a); a.click(); a.remove();
  URL.revokeObjectURL(url);
}
function importSettings(file) {
  const reader = new FileReader();
  reader.onload = () => {
    try {
      const obj = JSON.parse(reader.result);
      const settings = obj && obj.settings && typeof obj.settings === 'object' ? obj.settings : obj;
      if (!settings || typeof settings !== 'object') throw new Error('형식이 올바르지 않습니다');
      let n = 0;
      Object.keys(settings).forEach((k) => {
        if (k.startsWith('rosWeb') && typeof settings[k] === 'string') { localStorage.setItem(k, settings[k]); n += 1; }
      });
      if (!n) throw new Error('복원할 rosWeb 설정 항목이 없습니다');
      alert(`설정 ${n}개를 복원했습니다. 적용을 위해 새로고침합니다.`);
      location.reload();
    } catch (e) {
      alert('백업 불러오기 실패: ' + e.message);
    }
  };
  reader.readAsText(file);
}
$('#backupExport').addEventListener('click', exportSettings);
$('#backupImportBtn').addEventListener('click', () => $('#backupImportFile').click());
$('#backupImportFile').addEventListener('change', (e) => {
  const f = e.target.files && e.target.files[0];
  if (f) importSettings(f);
  e.target.value = '';
});
$('#backupSaveDefault').addEventListener('click', saveCurrentAsDefault);
$('#backupRestoreDefault').addEventListener('click', restoreDefault);

// 서버의 UI/ 폴더 설정 파일(가장 최신)로 전체 설정 교체
async function loadUiFolderSettings() {
  if (!(await vdConfirm('UI 폴더의 설정 파일로 전체 설정을 바꿉니다.\n현재 설정은 덮어써집니다. 계속할까요?'))) return;
  try {
    const res = await fetch('/api/ui/default-settings', { cache: 'no-store' });
    if (!res.ok) throw new Error('UI 폴더에 설정 파일이 없습니다');
    const obj = await res.json();
    const settings = obj && obj.settings && typeof obj.settings === 'object' ? obj.settings : obj;
    const keys = Object.keys(settings || {}).filter((k) => k.startsWith('rosWeb') && typeof settings[k] === 'string');
    if (!keys.length) throw new Error('복원할 rosWeb 설정 항목이 없습니다');
    const toRemove = [];
    for (let i = 0; i < localStorage.length; i++) {
      const k = localStorage.key(i);
      if (k && k.startsWith('rosWeb') && k !== DEFAULT_SNAPSHOT_KEY) toRemove.push(k);
    }
    toRemove.forEach((k) => localStorage.removeItem(k));
    keys.forEach((k) => localStorage.setItem(k, settings[k]));
    location.reload();
  } catch (e) {
    vdNotice('UI 폴더 설정 불러오기 실패: ' + e.message);
  }
}
$('#backupLoadUiFile').addEventListener('click', loadUiFolderSettings);

postVizConfig(view.vizSettings);
renderVizLegend();

// ── 주행 뷰 (RViz 패널 탭: 3D 주행 / 2D 레이더) ──────────────────────────
// 좌표계는 RViz 패널과 동일: x=전방, y=좌측, z=위 (base_link 기준).
// 데이터: 트랙(/vehicle_tracks), 클러스터(Detection3D), 레이더 POINTS 마커, LiDAR 포인트,
//         차량(/acu_ck2_ver2) 속도·조향·브레이크. 차선 토픽(토픽명에 lane)이 없으면
//         조향각 기반 예상 경로를 차로로 그린다.
// camMode: off | auto(전방 1대) | all(전체) | select(cameras 목록), camLayout: strip | surround
const DRIVE_DEFAULTS = {
  tracks: true, clusters: true, radar: true, lidar: false, path: true,
  camMode: 'auto', cameras: [], camLayout: 'strip',
  range: 80, laneWidth: 3.5, steerRatio: 14.5, wheelbase: 2.7,
};
const DRIVE_PRESETS = {
  chase:   { yaw: 0,     pitch: 0.36, dist: 30, tx: 16 },
  oblique: { yaw: -0.62, pitch: 0.62, dist: 46, tx: 16 },
  top:     { yaw: 0,     pitch: 1.50, dist: 95, tx: 28 },
};
const DRIVE_FOV = 55 * Math.PI / 180;
const DRIVE_NEAR = 0.5;
const DRIVE_COLORS = {
  track: '#c4b5fd', lead: '#3ddc97', cluster: '#ff5d6c', radar: '#c084fc',
  lidar: 'rgba(56,189,248,0.55)', ego: '#f0765a', egoBody: '#1c2a3a', car: '#22303f',
};
const driveSettings = (() => {
  let saved = {};
  try { saved = JSON.parse(localStorage.getItem('rosWebDriveSettings') || 'null') || {}; } catch (_) { /* 기본값 사용 */ }
  // 구버전(camera: bool, cameraTopic: 단일 토픽) 이전
  if (saved.camMode === undefined && 'camera' in saved) {
    saved.camMode = !saved.camera ? 'off' : saved.cameraTopic ? 'select' : 'auto';
    saved.cameras = saved.cameraTopic ? [saved.cameraTopic] : [];
  }
  delete saved.camera; delete saved.cameraTopic;
  return { ...DRIVE_DEFAULTS, ...saved, cameras: Array.isArray(saved.cameras) ? saved.cameras : [] };
})();
function saveDriveSettings() { localStorage.setItem('rosWebDriveSettings', JSON.stringify(driveSettings)); }

// ?vtab=drive3d|radar2d 로 탭을 바로 열 수 있음 (링크 공유용)
view.vizTab = new URLSearchParams(location.search).get('vtab') || localStorage.getItem('rosWebVizTab') || 'rviz';
const driveCam = (() => {
  const preset = localStorage.getItem('rosWebDrivePreset') || 'chase';
  return { preset, ...(DRIVE_PRESETS[preset] || DRIVE_PRESETS.chase), dragging: false, lastX: 0, lastY: 0 };
})();
let driveRange2d = driveSettings.range;
// 2D 뷰 이동량(m): 화면 중심을 전방 panF, 좌측 panL 만큼 옮김. radar2dMap = 마지막 렌더의 좌표 변환값
const view2d = { panF: 0, panL: 0, dragging: false, lastX: 0, lastY: 0 };
let radar2dMap = null;
function resetView2d() { driveRange2d = driveSettings.range; view2d.panF = 0; view2d.panL = 0; }
// 표시 폭에 맞는 1·2·5 단위 눈금 간격 (화면에 약 8~12칸)
function niceGridStep(span) {
  const raw = span / 10;
  const p = Math.pow(10, Math.floor(Math.log10(raw)));
  const m = raw / p;
  return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p;
}

const vizPanelEl = document.querySelector('.viz-panel');
const driveViewEl = $('#driveView');
const driveCanvas = $('#driveCanvas');
const driveCtx = driveCanvas.getContext('2d');

function setVizTab(tab) {
  view.vizTab = ['rviz', 'drive3d', 'radar2d'].includes(tab) ? tab : 'rviz';
  localStorage.setItem('rosWebVizTab', view.vizTab);
  vizPanelEl.dataset.vtab = view.vizTab;
  driveViewEl.hidden = view.vizTab === 'rviz';
  document.querySelectorAll('#vizTabs [data-vtab]').forEach((b) => b.classList.toggle('active', b.dataset.vtab === view.vizTab));
  $('#driveKicker').textContent = view.vizTab === 'radar2d' ? 'RADAR VIEW' : 'DRIVING VIEW';
  $('#driveHint').textContent = view.vizTab === 'radar2d' ? '휠 확대 · 드래그 이동 · 더블클릭 원위치' : '드래그 회전 · 휠 확대';
  renderVisualization();
}

function setDrivePreset(name) {
  const p = DRIVE_PRESETS[name] || DRIVE_PRESETS.chase;
  Object.assign(driveCam, p, { preset: name });
  localStorage.setItem('rosWebDrivePreset', name);
  document.querySelectorAll('#drivePresets [data-dpreset]').forEach((b) => b.classList.toggle('active', b.dataset.dpreset === name));
  renderVisualization();
}

// ── 장면 데이터 구성 ──
function driveVehicleState() {
  const v = view.mode === 'play' ? state.vehiclePlay : state.vehicle;
  if (!v) return null;
  const n = (x) => (typeof x === 'number' ? x : 0);
  const wheels = [v.wheel_fl, v.wheel_fr, v.wheel_rl, v.wheel_rr].filter((x) => typeof x === 'number');
  const speed = wheels.length ? wheels.reduce((a, b) => a + b, 0) / wheels.length : 0;
  const steer = n(v.steer_angle);   // 차량 규약: + = 좌
  const roadWheel = steer / Math.max(1, driveSettings.steerRatio) * Math.PI / 180;
  const curvature = Math.tan(roadWheel) / Math.max(0.5, driveSettings.wheelbase);
  return { speed, steer, curvature, brake: n(v.brake), torque: n(v.motor_tq), gear: VEH_GEAR[Math.round(n(v.gear))] ?? '—' };
}

// 예상 경로 위 호 길이 s, 횡 오프셋 d 지점 (x, y)
function pathPoint(k, s, d = 0) {
  const th = k * s;
  const x = Math.abs(k) < 1e-5 ? s : Math.sin(th) / k;
  const y = Math.abs(k) < 1e-5 ? 0 : (1 - Math.cos(th)) / k;
  return [x - d * Math.sin(th), y + d * Math.cos(th)];
}
// 점 (x, y) 의 예상 경로 기준 횡 오프셋
function pathLateral(k, x, y) {
  if (Math.abs(k) < 1e-5) return y;
  const r = 1 / k;
  return Math.sign(r) * (Math.abs(r) - Math.hypot(x, y - r));
}

function driveScene() {
  const inPlay = view.mode === 'play';
  const viz = state.visualization || {};
  const ds = driveSettings;
  const veh = driveVehicleState();
  const k = veh ? veh.curvature : 0;
  const tracks = [];
  const clusters = [];
  for (const b of viz.boxes || []) {
    if (isPlayTopic(b.topic) !== inPlay) continue;
    if (b.ns === 'vehicle_rejected') continue;
    const o = { ...b, x: b.x || 0, y: b.y || 0, z: b.z || 0, dist: Math.hypot(b.x || 0, b.y || 0) };
    if (b.kind === 'track') tracks.push(o);
    else if (b.kind !== 'radar') clusters.push(o);
  }
  // 트랙과 겹치는 클러스터(트랙의 원본)는 중복 표시하지 않음
  const clustersOnly = clusters.filter((c) => !tracks.some((t) => Math.hypot(t.x - c.x, t.y - c.y) < 1.6));
  const radar = [];
  const lanes = [];
  for (const m of viz.markers || []) {
    if (isPlayTopic(m.topic) !== inPlay) continue;
    if (/lane/i.test(m.topic) && [4, 5].includes(m.type)) { lanes.push(m); continue; }
    if (m.kind !== 'radar' || [4, 5].includes(m.type)) continue;
    for (const p of m.points || []) radar.push({ x: p[0] || 0, y: p[1] || 0, z: p[2] || 0, dist: Math.hypot(p[0] || 0, p[1] || 0) });
  }
  const lidarTopic = effectiveTopic((view.vizLayers.find((l) => l.id === 'l1') || {}).topic || '');
  const lidar = [];
  for (const layer of viz.point_layers || []) {
    if (isPlayTopic(layer.topic) !== inPlay) continue;
    if (lidarTopic ? layer.topic !== lidarTopic : /radar/i.test(layer.topic)) continue;
    lidar.push(layer.points || []);
  }
  // 선행 차량: 예상 경로 차로 안에서 가장 가까운 전방 객체 (트랙 우선, 없으면 클러스터)
  const inLane = (o) => o.x > 1.5 && o.x < ds.range && Math.abs(pathLateral(k, o.x, o.y)) < ds.laneWidth / 2 + 0.4;
  const pick = (arr) => arr.filter(inLane).sort((a, b) => a.x - b.x)[0] || null;
  const lead = (ds.tracks && pick(tracks)) || (ds.clusters && pick(clustersOnly)) || null;
  const leadRadar = lead ? radar.find((r) => Math.hypot(r.x - lead.x, r.y - lead.y) < 2.8) || null : null;
  return {
    veh, k, lead, leadRadar, lanes,
    tracks: ds.tracks ? tracks : [],
    clusters: ds.clusters ? clustersOnly : [],
    radar: ds.radar ? radar : [],
    lidar: ds.lidar ? lidar : [],
  };
}

// ── 3D 카메라 ──
function driveCamera(width, height) {
  const cp = Math.cos(driveCam.pitch), sp = Math.sin(driveCam.pitch);
  const cy = Math.cos(driveCam.yaw), sy = Math.sin(driveCam.yaw);
  const target = [driveCam.tx, 0, 0];
  const eye = [target[0] - driveCam.dist * cp * cy, target[1] - driveCam.dist * cp * sy, driveCam.dist * sp];
  let f = [target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]];
  const fl = Math.hypot(...f); f = f.map((v) => v / fl);
  let r = [f[1], -f[0], 0];   // f × (0,0,1)
  const rl = Math.hypot(...r) || 1; r = r.map((v) => v / rl);
  const u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]];
  const focal = (height / 2) / Math.tan(DRIVE_FOV / 2);
  return { eye, f, r, u, focal, width, height, cx: width / 2, cy: height * 0.46 };
}
function camSpace(cam, p) {
  const d = [p[0] - cam.eye[0], p[1] - cam.eye[1], (p[2] || 0) - cam.eye[2]];
  return [d[0] * cam.r[0] + d[1] * cam.r[1] + d[2] * cam.r[2],
          d[0] * cam.u[0] + d[1] * cam.u[1] + d[2] * cam.u[2],
          d[0] * cam.f[0] + d[1] * cam.f[1] + d[2] * cam.f[2]];
}
function camToScreen(cam, c) {
  return { x: cam.cx + c[0] * cam.focal / c[2], y: cam.cy - c[1] * cam.focal / c[2], z: c[2] };
}
function project(cam, p) {
  const c = camSpace(cam, p);
  return c[2] > DRIVE_NEAR ? camToScreen(cam, c) : null;
}
// 근평면 클리핑 후 화면 좌표 다각형
function clipPoly(cam, pts) {
  const cs = pts.map((p) => camSpace(cam, p));
  const out = [];
  for (let i = 0; i < cs.length; i++) {
    const a = cs[i], b = cs[(i + 1) % cs.length];
    const ain = a[2] > DRIVE_NEAR, bin = b[2] > DRIVE_NEAR;
    if (ain) out.push(a);
    if (ain !== bin) {
      const t = (DRIVE_NEAR - a[2]) / (b[2] - a[2]);
      out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, DRIVE_NEAR + 1e-3]);
    }
  }
  return out.map((c) => camToScreen(cam, c));
}
function strokePoly3(cam, pts, closed = false) {
  // 선분 단위 클리핑 (polyline)
  let drawing = false;
  driveCtx.beginPath();
  const n = closed ? pts.length : pts.length - 1;
  for (let i = 0; i < n; i++) {
    let a = camSpace(cam, pts[i]), b = camSpace(cam, pts[(i + 1) % pts.length]);
    if (a[2] <= DRIVE_NEAR && b[2] <= DRIVE_NEAR) { drawing = false; continue; }
    const clipTo = (p, q) => { const t = (DRIVE_NEAR - p[2]) / (q[2] - p[2]); return [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t, DRIVE_NEAR + 1e-3]; };
    if (a[2] <= DRIVE_NEAR) { a = clipTo(a, b); drawing = false; }
    let bClipped = false;
    if (b[2] <= DRIVE_NEAR) { b = clipTo(b, a); bClipped = true; }
    const sa = camToScreen(cam, a), sb = camToScreen(cam, b);
    if (!drawing) driveCtx.moveTo(sa.x, sa.y);
    driveCtx.lineTo(sb.x, sb.y);
    drawing = !bClipped;
  }
  driveCtx.stroke();
}
function fillPoly3(cam, pts) {
  const s = clipPoly(cam, pts);
  if (s.length < 3) return false;
  driveCtx.beginPath();
  driveCtx.moveTo(s[0].x, s[0].y);
  for (const p of s.slice(1)) driveCtx.lineTo(p.x, p.y);
  driveCtx.closePath();
  return true;
}

// 직육면체 (바닥 z0, 길이 L·폭 W·높이 H, yaw) — 보이는 면만 칠함
function drawSolid(cam, cx, cy, z0, L, W, H, yaw, style) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const P = (dx, dy, dz) => [cx + c * dx - s * dy, cy + s * dx + c * dy, z0 + dz];
  const hl = L / 2, hw = W / 2;
  const v = [P(-hl, -hw, 0), P(hl, -hw, 0), P(hl, hw, 0), P(-hl, hw, 0), P(-hl, -hw, H), P(hl, -hw, H), P(hl, hw, H), P(-hl, hw, H)];
  const faces = [
    { i: [4, 5, 6, 7], n: [0, 0, 1], top: true },
    { i: [0, 3, 2, 1], n: [0, 0, -1] },
    { i: [1, 2, 6, 5], n: [c, s, 0], front: true },
    { i: [0, 4, 7, 3], n: [-c, -s, 0], rear: true },
    { i: [0, 1, 5, 4], n: [s, -c, 0] },
    { i: [3, 7, 6, 2], n: [-s, c, 0] },
  ];
  const centerOf = (f) => f.i.reduce((a, k) => [a[0] + v[k][0] / 4, a[1] + v[k][1] / 4, a[2] + v[k][2] / 4], [0, 0, 0]);
  const vis = [];
  for (const f of faces) {
    const fc = centerOf(f);
    const toEye = [cam.eye[0] - fc[0], cam.eye[1] - fc[1], cam.eye[2] - fc[2]];
    if (f.n[0] * toEye[0] + f.n[1] * toEye[1] + f.n[2] * toEye[2] <= 0) continue;
    vis.push({ f, depth: camSpace(cam, fc)[2] });
  }
  vis.sort((a, b) => b.depth - a.depth);
  for (const { f } of vis) {
    if (!fillPoly3(cam, f.i.map((k) => v[k]))) continue;
    driveCtx.fillStyle = f.top ? (style.top || style.fill) : f.front || f.rear ? (style.side2 || style.fill) : style.fill;
    driveCtx.fill();
    if (style.stroke) { driveCtx.strokeStyle = style.stroke; driveCtx.lineWidth = style.lineWidth || 1.2; driveCtx.stroke(); }
    if (f.rear && style.tail) {
      // 후미등
      driveCtx.strokeStyle = style.tail; driveCtx.lineWidth = 3;
      strokePoly3(cam, [P(-hl - 0.01, -hw * 0.85, H * 0.72), P(-hl - 0.01, -hw * 0.35, H * 0.72)]);
      strokePoly3(cam, [P(-hl - 0.01, hw * 0.35, H * 0.72), P(-hl - 0.01, hw * 0.85, H * 0.72)]);
    }
  }
}

// 차량 모형: 바퀴 + 차체 + 캐빈(지붕 강조색)
function drawCar(cam, o, accent, opts = {}) {
  const L = Math.max(2.6, Math.min(12, Math.max(o.sx || 4.4, o.sy || 1.8)));
  const W = Math.max(1.4, Math.min(3.0, Math.min(o.sx || 4.4, o.sy || 1.8)));
  const H = Math.max(1.2, Math.min(3.4, o.sz || 1.5));
  const yaw = (o.sy || 0) > (o.sx || 0) ? (o.yaw || 0) + Math.PI / 2 : (o.yaw || 0);
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const at = (dx, dy) => [o.x + c * dx - s * dy, o.y + s * dx + c * dy];
  const stroke = opts.stroke || 'rgba(190,210,235,0.75)';
  // 그림자
  driveCtx.fillStyle = 'rgba(0,0,0,0.35)';
  const sh = [at(-L / 2 - 0.3, -W / 2 - 0.3), at(L / 2 + 0.3, -W / 2 - 0.3), at(L / 2 + 0.3, W / 2 + 0.3), at(-L / 2 - 0.3, W / 2 + 0.3)].map((p) => [p[0], p[1], 0.01]);
  if (fillPoly3(cam, sh)) driveCtx.fill();
  if (opts.plate) {
    driveCtx.fillStyle = opts.plate;
    const pl = [at(-L / 2 - 0.8, -W / 2 - 0.6), at(L / 2 + 0.8, -W / 2 - 0.6), at(L / 2 + 0.8, W / 2 + 0.6), at(-L / 2 - 0.8, W / 2 + 0.6)].map((p) => [p[0], p[1], 0.02]);
    if (fillPoly3(cam, pl)) { driveCtx.fill(); driveCtx.strokeStyle = opts.plateStroke || opts.plate; driveCtx.lineWidth = 1; driveCtx.stroke(); }
  }
  for (const [dx, dy] of [[L * 0.31, W * 0.44], [L * 0.31, -W * 0.44], [-L * 0.31, W * 0.44], [-L * 0.31, -W * 0.44]]) {
    const p = at(dx, dy);
    drawSolid(cam, p[0], p[1], 0, 0.72, 0.26, 0.62, yaw, { fill: '#0b1016', stroke: 'rgba(120,140,160,0.5)', lineWidth: 1 });
  }
  const body = opts.body || DRIVE_COLORS.car;
  drawSolid(cam, o.x, o.y, 0.28, L, W, H * 0.42, yaw, { fill: body, side2: body, top: body, stroke, tail: opts.tail });
  const cab = at(-L * 0.06, 0);
  drawSolid(cam, cab[0], cab[1], 0.28 + H * 0.42, L * 0.56, W * 0.84, H * 0.5, yaw,
    { fill: 'rgba(30,44,60,0.95)', side2: 'rgba(40,60,80,0.95)', top: accent, stroke });
}

// placed: 이미 그린 라벨 사각형 목록 — 겹치면 건너뜀 (가까운 객체부터 호출)
function drawLabel(x, y, parts, border, placed) {
  driveCtx.font = '600 12px "Pretendard Variable", Pretendard, Inter, system-ui, sans-serif';
  const gap = 8;
  const widths = parts.map((p) => driveCtx.measureText(p.text).width);
  const w = widths.reduce((a, b) => a + b, 0) + gap * (parts.length - 1) + 18;
  const h = 24;
  const lx = Math.round(x - w / 2), ly = Math.round(y - h);
  if (placed) {
    if (placed.some((r) => lx < r[2] + 4 && lx + w > r[0] - 4 && ly < r[3] + 2 && ly + h > r[1] - 2)) return;
    placed.push([lx, ly, lx + w, ly + h]);
  }
  driveCtx.fillStyle = 'rgba(10,16,24,0.86)';
  driveCtx.strokeStyle = border;
  driveCtx.lineWidth = 1;
  driveCtx.beginPath();
  if (driveCtx.roundRect) driveCtx.roundRect(lx, ly, w, h, 6); else driveCtx.rect(lx, ly, w, h);
  driveCtx.fill(); driveCtx.stroke();
  let tx = lx + 9;
  parts.forEach((p, i) => {
    driveCtx.fillStyle = p.color;
    driveCtx.fillText(p.text, tx, ly + 16);
    tx += widths[i] + gap;
  });
}

// Camera Preview 그리드에서 프레임이 있는 <img> (토픽별 1개) — 추가 연결 없이 재사용
function liveCameraImgs() {
  const out = new Map();
  for (const im of document.querySelectorAll('#cameraGrid img')) {
    if (im.dataset.topic && im.naturalWidth > 0 && !out.has(im.dataset.topic)) out.set(im.dataset.topic, im);
  }
  return out;
}
function driveCameraTopics() {
  const ds = driveSettings;
  const live = liveCameraImgs();
  if (ds.camMode === 'off') return [];
  if (ds.camMode === 'all') return [...live.keys()];
  if (ds.camMode === 'select') return ds.cameras.slice();
  const keys = [...live.keys()];
  const front = keys.find((t) => /front/i.test(t) && !/left|right/i.test(t)) || keys.find((t) => /front/i.test(t)) || keys[0];
  return front ? [front] : [];
}
// 토픽 이름으로 차량 기준 방향 추정 (surround 배치용)
function cameraSlotOf(topic) {
  const t = topic.toLowerCase();
  const left = /left/.test(t), right = /right/.test(t);
  if (/avm|around|bird|surround/.test(t)) return 'bc';
  if (/rear|back/.test(t)) return left ? 'bl' : right ? 'br' : 'bc';
  if (left) return 'tl';
  if (right) return 'tr';
  return 'tc';
}
let driveTileHits = [];   // [{x,y,w,h,topic}] — 클릭 시 확대 보기
function drawCameraTile(img, topic, x, y, w, h) {
  driveCtx.save();
  driveCtx.beginPath();
  if (driveCtx.roundRect) driveCtx.roundRect(x, y, w, h, 7); else driveCtx.rect(x, y, w, h);
  driveCtx.clip();
  driveCtx.fillStyle = '#05080c';
  driveCtx.fillRect(x, y, w, h);
  if (img) { try { driveCtx.drawImage(img, x, y, w, h); } catch (_) { /* 디코딩 중 프레임 */ } }
  else {
    driveCtx.fillStyle = '#7f90a3';
    driveCtx.font = '500 11px "Pretendard Variable", Pretendard, system-ui, sans-serif';
    driveCtx.fillText('미리보기에 없음', x + 8, y + h / 2);
  }
  driveCtx.fillStyle = 'rgba(5,8,12,0.65)';
  driveCtx.fillRect(x, y + h - 18, w, 18);
  driveCtx.fillStyle = '#e8eef5';
  driveCtx.font = '600 10.5px "JetBrains Mono", ui-monospace, monospace';
  const name = topic.split('/').filter(Boolean)[0] || topic;
  driveCtx.fillText(name.length * 6.4 > w - 12 ? name.slice(0, Math.max(3, Math.floor((w - 12) / 6.4))) : name, x + 6, y + h - 5);
  driveCtx.restore();
  driveCtx.strokeStyle = 'rgba(120,170,220,0.38)';
  driveCtx.lineWidth = 1;
  driveCtx.beginPath();
  if (driveCtx.roundRect) driveCtx.roundRect(x, y, w, h, 7); else driveCtx.rect(x, y, w, h);
  driveCtx.stroke();
  driveTileHits.push({ x, y, w, h, topic });
}
function drawCameraTiles(width, height) {
  driveTileHits = [];
  const topics = driveCameraTopics();
  if (!topics.length) return;
  const live = liveCameraImgs();
  const aspectOf = (im) => (im ? im.naturalHeight / im.naturalWidth : 0.5625);
  const gap = 8, top = 48, bottomPad = 34;
  if (driveSettings.camLayout === 'surround' && topics.length > 1) {
    const slots = { tl: [], tc: [], tr: [], bl: [], bc: [], br: [] };
    for (const t of topics) slots[cameraSlotOf(t)].push(t);
    const maxPerRow = Math.max(slots.tl.length + slots.tc.length + slots.tr.length, slots.bl.length + slots.bc.length + slots.br.length, 3);
    // 위·아래 줄이 겹치지 않고 가운데 장면이 40% 이상 보이도록 높이 제한
    const hMax = Math.max(36, (height - top - bottomPad - height * 0.4) / 2);
    const w = Math.max(64, Math.min(210, (width - 28 - gap * (maxPerRow + 1)) / maxPerRow, hMax / 0.5625));
    const h = w * 0.5625;
    const place = (list, align, y) => {
      const total = list.length * w + Math.max(0, list.length - 1) * gap;
      let x = align === 'l' ? 14 : align === 'r' ? width - 14 - total : (width - total) / 2;
      for (const t of list) { drawCameraTile(live.get(t), t, x, y, w, h); x += w + gap; }
    };
    place(slots.tl, 'l', top); place(slots.tc, 'c', top); place(slots.tr, 'r', top);
    const yb = height - bottomPad - h;
    place(slots.bl, 'l', yb); place(slots.bc, 'c', yb); place(slots.br, 'r', yb);
    return;
  }
  // strip: 상단 오른쪽부터 한 줄(넘치면 여러 줄), 1대면 큰 PIP
  const n = topics.length;
  if (n === 1) {
    const w = Math.min(320, Math.max(160, width * 0.26));
    const im = live.get(topics[0]);
    drawCameraTile(im, topics[0], width - w - 14, top + 8, w, w * aspectOf(im));
    return;
  }
  if (view.vizTab === 'radar2d') {
    // 2D는 좌우가 비어 있으므로 오른쪽 세로 열(넘치면 왼쪽으로 열 추가)
    const avail = height - top - 8 - bottomPad;
    let w = Math.min(200, width * 0.2);
    let perCol = Math.max(1, Math.floor((avail + gap) / (w * 0.5625 + gap)));
    if (Math.ceil(n / perCol) > 2) {
      // 최대 2열: 열당 개수에 맞춰 타일을 줄임
      perCol = Math.ceil(n / 2);
      w = Math.min(width * 0.2, Math.max(36, (avail + gap) / perCol - gap) / 0.5625);
    }
    const h = w * 0.5625;
    topics.forEach((t, i) => {
      const c = Math.floor(i / perCol), r = i % perCol;
      drawCameraTile(live.get(t), t, width - 14 - (c + 1) * w - c * gap, top + 8 + r * (h + gap), w, h);
    });
    return;
  }
  const cols = Math.min(n, Math.max(1, Math.floor((width - 28 + gap) / (110 + gap))));
  const rows = Math.ceil(n / cols);
  let w = Math.min(220, (width - 28 - gap * (cols - 1)) / cols);
  if (rows * (w * 0.5625 + gap) > height * 0.42) w = (height * 0.42 / rows - gap) / 0.5625;
  const h = w * 0.5625;
  topics.forEach((t, i) => {
    const r = Math.floor(i / cols), c = i % cols;
    const inRow = Math.min(cols, n - r * cols);
    const x0 = width - 14 - inRow * w - (inRow - 1) * gap;
    drawCameraTile(live.get(t), t, x0 + c * (w + gap), top + 8 + r * (h + gap), w, h);
  });
}

function sizeDriveCanvas() {
  const rect = driveCanvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(240, Math.floor(rect.width));
  const height = Math.max(200, Math.floor(rect.height));
  if (driveCanvas.width !== Math.floor(width * dpr) || driveCanvas.height !== Math.floor(height * dpr)) {
    driveCanvas.width = Math.floor(width * dpr);
    driveCanvas.height = Math.floor(height * dpr);
  }
  driveCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { width, height };
}

function renderDrive3D(sc) {
  const { width, height } = sizeDriveCanvas();
  const cam = driveCamera(width, height);
  const ds = driveSettings;
  const range = ds.range;
  const lw = ds.laneWidth;
  const k = sc.k;

  // 배경: 상단은 안개, 하단은 지면
  const bg = driveCtx.createLinearGradient(0, 0, 0, height);
  bg.addColorStop(0, '#0d1a2b'); bg.addColorStop(0.45, '#0b1320'); bg.addColorStop(1, '#070b10');
  driveCtx.fillStyle = bg;
  driveCtx.fillRect(0, 0, width, height);
  const vg = driveCtx.createRadialGradient(width / 2, height * 0.1, 10, width / 2, height * 0.1, width * 0.7);
  vg.addColorStop(0, 'rgba(56,120,200,0.16)'); vg.addColorStop(1, 'rgba(56,120,200,0)');
  driveCtx.fillStyle = vg;
  driveCtx.fillRect(0, 0, width, height);

  // 지면 그리드
  const gx0 = -30, gx1 = range + 20, gy = 40, step = 5;
  driveCtx.lineWidth = 1;
  for (let x = Math.ceil(gx0 / step) * step; x <= gx1; x += step) {
    driveCtx.strokeStyle = x % 20 === 0 ? 'rgba(120,170,220,0.16)' : 'rgba(120,170,220,0.07)';
    strokePoly3(cam, [[x, -gy, 0], [x, gy, 0]]);
  }
  for (let y = -gy; y <= gy; y += step) {
    driveCtx.strokeStyle = 'rgba(120,170,220,0.07)';
    strokePoly3(cam, [[gx0, y, 0], [gx1, y, 0]]);
  }

  // 차로 (예상 경로 기반) 또는 차선 마커
  const N = 48;
  const along = (d, s0 = 0, s1 = range) => Array.from({ length: N + 1 }, (_, i) => { const p = pathPoint(k, s0 + (s1 - s0) * i / N, d); return [p[0], p[1], 0.02]; });
  if (ds.path) {
    const left = along(lw / 2), right = along(-lw / 2).reverse();
    driveCtx.fillStyle = 'rgba(80,120,160,0.22)';
    if (fillPoly3(cam, [...left, ...right])) driveCtx.fill();
    driveCtx.strokeStyle = 'rgba(210,225,240,0.75)'; driveCtx.lineWidth = 2;
    strokePoly3(cam, along(lw / 2)); strokePoly3(cam, along(-lw / 2));
    driveCtx.setLineDash([10, 12]); driveCtx.strokeStyle = 'rgba(150,170,190,0.35)'; driveCtx.lineWidth = 1.4;
    strokePoly3(cam, along(lw * 1.5)); strokePoly3(cam, along(-lw * 1.5));
    driveCtx.setLineDash([]);
    // 거리 눈금
    driveCtx.font = '500 12px "Pretendard Variable", Pretendard, system-ui, sans-serif';
    for (let d = 20; d <= range; d += 20) {
      const p = pathPoint(k, d, -lw * 1.5 - 1.2);
      const sp = project(cam, [p[0], p[1], 0]);
      if (!sp || sp.y < 0 || sp.y > height) continue;
      driveCtx.fillStyle = 'rgba(160,180,200,0.75)';
      driveCtx.fillText(`${d} m`, sp.x + 4, sp.y);
      driveCtx.fillRect(sp.x - 8, sp.y - 4, 8, 1);
    }
    // 선행 차량까지 경로 빔
    const leadS = sc.lead ? Math.max(3, sc.lead.x - 2.5) : 14;
    const beamL = along(0.45, 2.6, leadS), beamR = along(-0.45, 2.6, leadS).reverse();
    if (fillPoly3(cam, [...beamL, ...beamR])) {
      const n0 = project(cam, [2.6, 0, 0]), n1 = project(cam, pathPoint(k, leadS).concat(0));
      const g = n0 && n1 ? driveCtx.createLinearGradient(n0.x, n0.y, n1.x, n1.y) : null;
      if (g) { g.addColorStop(0, 'rgba(56,189,248,0.75)'); g.addColorStop(1, 'rgba(56,189,248,0.08)'); }
      driveCtx.fillStyle = g || 'rgba(56,189,248,0.35)';
      driveCtx.fill();
    }
  }
  for (const m of sc.lanes) {
    driveCtx.strokeStyle = m.color || '#e8eef5'; driveCtx.lineWidth = 2;
    const pts = m.points.map((p) => [p[0], p[1], 0.03]);
    if (m.type === 5) for (let i = 0; i + 1 < pts.length; i += 2) strokePoly3(cam, [pts[i], pts[i + 1]]);
    else strokePoly3(cam, pts);
  }

  // LiDAR 점
  driveCtx.fillStyle = DRIVE_COLORS.lidar;
  for (const pts of sc.lidar) {
    for (const p of pts) {
      const s = project(cam, p);
      if (!s) continue;
      const r = Math.max(0.8, Math.min(2.2, cam.focal / s.z * 0.06));
      driveCtx.fillRect(s.x, s.y, r, r);
    }
  }

  // 객체·자차를 깊이 순으로
  const items = [];
  const depthOf = (x, y) => camSpace(cam, [x, y, 0.8])[2];
  items.push({ depth: depthOf(0, 0), draw: () => drawCar(cam, { x: 0, y: 0, sx: 4.6, sy: 1.9, sz: 1.55, yaw: 0 }, DRIVE_COLORS.ego,
    { body: DRIVE_COLORS.egoBody, tail: '#ff5d6c', stroke: 'rgba(200,220,240,0.85)' }) });
  for (const t of sc.tracks) {
    const isLead = t === sc.lead;
    items.push({ depth: depthOf(t.x, t.y), obj: t, draw: () => drawCar(cam, t, isLead ? DRIVE_COLORS.lead : '#e8eef5',
      isLead ? { plate: 'rgba(61,220,151,0.16)', plateStroke: 'rgba(61,220,151,0.55)', stroke: 'rgba(160,240,200,0.85)' } : {}) });
  }
  for (const c of sc.clusters) {
    const isLead = c === sc.lead;
    items.push({ depth: depthOf(c.x, c.y), obj: c, draw: () => drawSolid(cam, c.x, c.y, Math.max(0, (c.z || 0) - (c.sz || 1) / 2),
      Math.max(0.4, c.sx || 1), Math.max(0.4, c.sy || 1), Math.max(0.4, c.sz || 1), c.yaw || 0,
      { fill: isLead ? 'rgba(61,220,151,0.16)' : 'rgba(255,93,108,0.14)', top: isLead ? 'rgba(61,220,151,0.28)' : 'rgba(255,93,108,0.26)',
        stroke: isLead ? DRIVE_COLORS.lead : DRIVE_COLORS.cluster, lineWidth: 1.4 }) });
  }
  for (const r of sc.radar) {
    items.push({ depth: depthOf(r.x, r.y), draw: () => {
      const s = project(cam, [r.x, r.y, 0.6]);
      if (!s) return;
      const rad = Math.max(2.5, Math.min(7, cam.focal / s.z * 0.28));
      driveCtx.fillStyle = 'rgba(192,132,252,0.22)';
      driveCtx.beginPath(); driveCtx.arc(s.x, s.y, rad * 2, 0, Math.PI * 2); driveCtx.fill();
      driveCtx.fillStyle = DRIVE_COLORS.radar;
      driveCtx.beginPath(); driveCtx.arc(s.x, s.y, rad, 0, Math.PI * 2); driveCtx.fill();
      if (r === sc.leadRadar) {
        driveCtx.strokeStyle = '#ffc04d'; driveCtx.lineWidth = 3;
        driveCtx.beginPath(); driveCtx.arc(s.x, s.y, rad * 2.2, 0, Math.PI * 2); driveCtx.stroke();
      }
    } });
  }
  items.sort((a, b) => b.depth - a.depth);
  for (const it of items) it.draw();

  // 라벨 (가까운 객체 우선, 최대 14개)
  const labeled = [...sc.tracks, ...sc.clusters].filter((o) => o.x > -10 && o.dist < range + 10)
    .sort((a, b) => (b === sc.lead) - (a === sc.lead) || a.dist - b.dist).slice(0, 14);
  const placed = [];
  for (const o of labeled) {
    const s = project(cam, [o.x, o.y, Math.max(1.6, (o.z || 0) + (o.sz || 1) / 2) + 0.9]);
    if (!s || s.x < -40 || s.x > width + 40 || s.y < 10 || s.y > height) continue;
    if (o === sc.lead) drawLabel(s.x, s.y, [{ text: 'LEAD', color: DRIVE_COLORS.lead }, { text: `${o.x.toFixed(1)} m`, color: '#e8eef5' }], 'rgba(61,220,151,0.6)', placed);
    else {
      const tag = o.kind === 'track' ? `#${o.id ?? '?'}` : '장애물';
      const col = o.kind === 'track' ? DRIVE_COLORS.track : DRIVE_COLORS.cluster;
      drawLabel(s.x, s.y, [{ text: tag, color: col }, { text: `${o.dist.toFixed(1)} m`, color: '#e8eef5' }], 'rgba(120,170,220,0.3)', placed);
    }
  }
  drawCameraTiles(width, height);
}

function renderRadar2D(sc) {
  const { width, height } = sizeDriveCanvas();
  const ds = driveSettings;
  const lw = ds.laneWidth;
  const k = sc.k;
  driveCtx.fillStyle = '#080c12';
  driveCtx.fillRect(0, 0, width, height);
  const R = driveRange2d;          // 화면 중심(자차+이동량) 기준 전방 표시 거리
  const Rr = R * 0.2;              // 후방 표시 거리
  const padL = 58, padR = 16, padT = 52, padB = 38;   // 위·아래 여백은 HUD 글자 자리
  const plotH = height - padT - padB;
  const scale = plotH / (R + Rr);
  const cx = padL + (width - padL - padR) / 2;
  const y0 = padT + R * scale;
  const { panF, panL } = view2d;
  const toS = (x, y) => [cx - (y - panL) * scale, y0 - (x - panF) * scale];
  const latMax = (width - padL - padR) / 2 / scale;
  // 눈금 간격이 화면에서 18px 이상 되도록 (패널이 낮을 때 거리 글자 겹침 방지)
  const step = niceGridStep((R + Rr) * Math.max(1, 180 / plotH));
  radar2dMap = { cx, y0, scale, padT, plotH, step };
  const fmt = (v) => (step < 1 ? v.toFixed(1) : Math.round(v)) + ' m';

  // 그리드 + 거리 눈금
  driveCtx.lineWidth = 1;
  driveCtx.font = '500 12px "Pretendard Variable", Pretendard, system-ui, sans-serif';
  for (let x = Math.ceil((panF - Rr) / step) * step; x <= panF + R + 1e-6; x += step) {
    const [, sy] = toS(x, 0);
    driveCtx.strokeStyle = Math.abs(x) < 1e-6 ? 'rgba(120,170,220,0.28)' : 'rgba(120,170,220,0.12)';
    driveCtx.beginPath(); driveCtx.moveTo(padL, sy); driveCtx.lineTo(width - padR, sy); driveCtx.stroke();
    driveCtx.fillStyle = 'rgba(160,180,200,0.75)';
    driveCtx.fillText(fmt(Math.abs(x) < 1e-6 ? 0 : x), 10, sy + 4);
  }
  for (let y = Math.ceil((panL - latMax) / step) * step; y <= panL + latMax; y += step) {
    const [sx] = toS(0, y);
    driveCtx.strokeStyle = Math.abs(y) < 1e-6 ? 'rgba(120,170,220,0.2)' : 'rgba(120,170,220,0.10)';
    driveCtx.beginPath(); driveCtx.moveTo(sx, padT - 10); driveCtx.lineTo(sx, height - padB); driveCtx.stroke();
  }

  // 그리드 영역 밖(표시 거리 초과)은 그리지 않음
  driveCtx.save();
  driveCtx.beginPath();
  driveCtx.rect(padL, padT - 8, width - padL - padR, height - padT - padB + 16);
  driveCtx.clip();

  // 차선 (예상 경로) 점선
  if (ds.path) {
    const N = 60;
    const lane = (d) => {
      driveCtx.beginPath();
      for (let i = 0; i <= N; i++) {
        const s0 = Math.min(-Rr * 0.3, panF - Rr), s1 = Math.max(R, panF + R);
        const p = pathPoint(k, s0 + (s1 - s0) * i / N, d);
        const [sx, sy] = toS(p[0], p[1]);
        if (i === 0) driveCtx.moveTo(sx, sy); else driveCtx.lineTo(sx, sy);
      }
      driveCtx.stroke();
    };
    driveCtx.setLineDash([7, 8]);
    driveCtx.strokeStyle = 'rgba(200,215,230,0.55)'; driveCtx.lineWidth = 1.4;
    lane(lw / 2); lane(-lw / 2);
    driveCtx.strokeStyle = 'rgba(160,175,190,0.3)';
    lane(lw * 1.5); lane(-lw * 1.5);
    driveCtx.setLineDash([]);
  }
  for (const m of sc.lanes) {
    driveCtx.strokeStyle = m.color || '#e8eef5'; driveCtx.lineWidth = 1.6;
    driveCtx.beginPath();
    m.points.forEach((p, i) => {
      const [sx, sy] = toS(p[0], p[1]);
      if (i === 0 || (m.type === 5 && i % 2 === 0)) driveCtx.moveTo(sx, sy); else driveCtx.lineTo(sx, sy);
    });
    driveCtx.stroke();
  }

  // LiDAR
  driveCtx.fillStyle = 'rgba(56,189,248,0.45)';
  for (const pts of sc.lidar) for (const p of pts) { const [sx, sy] = toS(p[0], p[1]); driveCtx.fillRect(sx, sy, 1.4, 1.4); }

  const rect = (o, stroke, fill) => {
    const L = Math.max(o.sx || 1, 0.4), W = Math.max(o.sy || 1, 0.4), yaw = o.yaw || 0;
    const c = Math.cos(yaw), s = Math.sin(yaw);
    const minPx = 7 / scale;
    const hl = Math.max(L, minPx) / 2, hw = Math.max(W, minPx) / 2;
    const pts = [[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]].map(([dx, dy]) => toS(o.x + c * dx - s * dy, o.y + s * dx + c * dy));
    driveCtx.beginPath();
    pts.forEach(([sx, sy], i) => (i ? driveCtx.lineTo(sx, sy) : driveCtx.moveTo(sx, sy)));
    driveCtx.closePath();
    driveCtx.fillStyle = fill; driveCtx.fill();
    driveCtx.strokeStyle = stroke; driveCtx.lineWidth = 2; driveCtx.stroke();
    return toS(o.x, o.y);
  };
  const placed = [];
  const text = (str, sx, sy, color) => {
    driveCtx.font = '500 12px "Pretendard Variable", Pretendard, system-ui, sans-serif';
    const w = driveCtx.measureText(str).width;
    if (placed.some((r) => sx < r[2] && sx + w > r[0] && sy - 12 < r[3] && sy + 2 > r[1])) return;
    placed.push([sx, sy - 12, sx + w, sy + 2]);
    driveCtx.lineWidth = 3; driveCtx.strokeStyle = 'rgba(8,12,18,0.85)';
    driveCtx.strokeText(str, sx, sy);
    driveCtx.fillStyle = color; driveCtx.fillText(str, sx, sy);
  };

  for (const c of sc.clusters) {
    const lead = c === sc.lead;
    const [sx, sy] = rect(c, lead ? DRIVE_COLORS.lead : 'rgba(255,93,108,0.85)', lead ? 'rgba(61,220,151,0.14)' : 'rgba(255,93,108,0.12)');
    if (lead) text(`LEAD ${c.x.toFixed(1)} m`, sx + 10, sy - 6, DRIVE_COLORS.lead);
  }
  for (const t of sc.tracks) {
    const lead = t === sc.lead;
    const [sx, sy] = rect(t, lead ? DRIVE_COLORS.lead : '#7ee0b0', lead ? 'rgba(61,220,151,0.22)' : 'rgba(126,224,176,0.10)');
    text(lead ? `LEAD #${t.id ?? '?'} ${t.x.toFixed(1)} m` : `#${t.id ?? '?'}`, sx + 10, sy - 6, lead ? DRIVE_COLORS.lead : DRIVE_COLORS.track);
  }
  const labelRadar = sc.radar.length <= 40;
  for (const r of sc.radar) {
    const [sx, sy] = toS(r.x, r.y);
    driveCtx.fillStyle = DRIVE_COLORS.radar;
    driveCtx.beginPath(); driveCtx.arc(sx, sy, 5, 0, Math.PI * 2); driveCtx.fill();
    if (r === sc.leadRadar) { driveCtx.strokeStyle = '#ffc04d'; driveCtx.lineWidth = 2; driveCtx.beginPath(); driveCtx.arc(sx, sy, 9, 0, Math.PI * 2); driveCtx.stroke(); }
    if (labelRadar) text(`${r.dist.toFixed(0)}m`, sx + 9, sy + 14, DRIVE_COLORS.radar);
  }

  // 자차
  const [ex, ey] = toS(0, 0);
  const ew = Math.max(12, 1.9 * scale), eh = Math.max(22, 4.6 * scale);
  driveCtx.fillStyle = '#e8eef5';
  driveCtx.beginPath();
  if (driveCtx.roundRect) driveCtx.roundRect(ex - ew / 2, ey - eh / 2, ew, eh, 3); else driveCtx.rect(ex - ew / 2, ey - eh / 2, ew, eh);
  driveCtx.fill();
  driveCtx.fillStyle = DRIVE_COLORS.ego;
  driveCtx.fillRect(ex - ew / 2 + 2, ey - eh / 2 + 2, ew - 4, Math.max(3, eh * 0.14));
  driveCtx.restore();
  drawCameraTiles(width, height);
}

function updateDriveHud(sc) {
  const v = sc.veh;
  const inPlay = view.mode === 'play';
  $('#driveSource').textContent = inPlay ? 'Play' : '실시간';
  $('#driveSource').classList.toggle('play', inPlay);
  const badge = $('#driveLaneBadge');
  if (sc.lanes.length) badge.textContent = `차선 마커 ${sc.lanes.length}개`;
  else if (driveSettings.path) badge.textContent = v ? '예상 주행 경로 · 조향 기반' : '예상 경로 · 차량 데이터 없음';
  else badge.textContent = '';
  badge.hidden = !badge.textContent;
  const nObj = sc.tracks.length + sc.clusters.length;
  const grid = view.vizTab === 'radar2d' && radar2dMap ? ` · 눈금 ${radar2dMap.step < 1 ? radar2dMap.step.toFixed(1) : radar2dMap.step} m` : '';
  $('#driveCount').textContent = `주변 객체 ${nObj}개 · 레이더 ${sc.radar.length}${grid}`;
  vizStats.textContent = `트랙 ${sc.tracks.length} · 클러스터 ${sc.clusters.length} · 레이더 ${sc.radar.length}${sc.lidar.length ? ` · LiDAR ${sc.lidar.reduce((a, p) => a + p.length, 0)}` : ''}`;
  $('#dsSpeed').innerHTML = v ? `${Math.abs(v.speed).toFixed(1)} <small>km/h</small> <i class="ds-gear">${escapeHtml(v.gear)}</i>` : '—';
  // 차량 규약 +=좌. 부호 대신 좌/우로 표기 (Vehicle Monitor 게이지는 우=+ 로 반전 표시)
  $('#dsSteer').textContent = !v ? '—' : Math.abs(v.steer) < 0.5 ? '직진 0.0°' : `${v.steer > 0 ? '좌' : '우'} ${Math.abs(v.steer).toFixed(1)}°`;
  const pedal = !v ? '—' : v.brake > 2 ? '브레이크 밟음' : v.torque > 5 ? '가속 중' : '페달 없음';
  const pedalEl = $('#dsPedal');
  pedalEl.textContent = pedal;
  pedalEl.className = v && v.brake > 2 ? 'ds-brake' : v && v.torque > 5 ? 'ds-accel' : '';
  const leadEl = $('#dsLead');
  leadEl.textContent = sc.lead ? `${sc.lead.x.toFixed(1)} m${sc.lead.kind === 'track' ? ` · #${sc.lead.id ?? '?'}` : ''}` : '없음';
  leadEl.className = sc.lead && sc.lead.x < PROX_WARN_M ? 'ds-brake' : sc.lead ? 'ds-lead' : '';
}

function renderDriveView() {
  const sc = driveScene();
  if (view.vizTab === 'radar2d') renderRadar2D(sc); else renderDrive3D(sc);
  updateDriveHud(sc);
}

// ── 설정 (통합 설정 창 → RViz 탭 '주행 뷰' 섹션) ──
let driveEdit = null;
function cameraTopicChoices() {
  const set = new Set([...currentModeCameras().map((c) => c.name), ...view.cameraLayout.slots.map((sl) => sl.topic).filter(Boolean)]);
  if (driveEdit) driveEdit.cameras.forEach((t) => set.add(t));
  return [...set].sort();
}
function renderDriveCamList() {
  const el = $('#dsetCamList');
  if (!el || !driveEdit) return;
  const inGrid = new Set(view.cameraLayout.slots.map((sl) => sl.topic).filter(Boolean));
  const topics = cameraTopicChoices();
  const selectable = driveEdit.camMode === 'select';
  el.classList.toggle('disabled', !selectable);
  el.innerHTML = topics.length ? topics.map((t) => `<label class="dset-cam${inGrid.has(t) ? '' : ' off-grid'}" title="${inGrid.has(t) ? '' : 'Camera Preview 슬롯에 없는 카메라는 표시되지 않습니다'}">
      <input type="checkbox" value="${escapeHtml(t)}"${driveEdit.cameras.includes(t) ? ' checked' : ''}${selectable ? '' : ' disabled'}>
      <span>${escapeHtml(t)}</span>${inGrid.has(t) ? '' : '<em>미리보기에 없음</em>'}</label>`).join('')
    : '<div class="obj-empty">카메라 토픽 없음</div>';
  el.querySelectorAll('input').forEach((cb) => cb.addEventListener('change', () => {
    driveEdit.cameras = cb.checked ? [...driveEdit.cameras.filter((x) => x !== cb.value), cb.value] : driveEdit.cameras.filter((x) => x !== cb.value);
  }));
}
function openDriveSettingsEdit() {
  driveEdit = { ...driveSettings, cameras: driveSettings.cameras.slice() };
  document.querySelectorAll('#setPanelRviz [data-dset]').forEach((el) => {
    const v = driveEdit[el.dataset.dset];
    if (el.type === 'checkbox') el.checked = !!v; else el.value = String(v);
  });
  $('#dsetRangeVal').textContent = `${driveEdit.range} m`;
  renderDriveCamList();
}
function commitDriveSettingsEdit(save) {
  if (save && driveEdit) {
    Object.assign(driveSettings, driveEdit);
    resetView2d();
    saveDriveSettings();
    renderVisualization();
  }
  driveEdit = null;
}
document.querySelectorAll('#setPanelRviz [data-dset]').forEach((el) => {
  el.addEventListener(el.type === 'range' ? 'input' : 'change', () => {
    if (!driveEdit) return;
    const key = el.dataset.dset;
    if (el.type === 'checkbox') driveEdit[key] = el.checked;
    else if (el.type === 'number' || el.type === 'range') {
      const n = Number(el.value);
      if (Number.isFinite(n) && n > 0) driveEdit[key] = n;
    } else driveEdit[key] = el.value;
    if (key === 'range') $('#dsetRangeVal').textContent = `${driveEdit.range} m`;
    if (key === 'camMode') {
      // 직접 선택으로 바꿀 때 비어 있으면 현재 보이는 카메라로 채움
      if (driveEdit.camMode === 'select' && !driveEdit.cameras.length) driveEdit.cameras = [...liveCameraImgs().keys()];
      renderDriveCamList();
    }
  });
});
function openDriveSettings() {
  openVizSettingsModal();
  const sec = view.vizTab === 'rviz' ? null : $('#vsmDriveSection');
  if (sec) setTimeout(() => sec.scrollIntoView({ block: 'start', behavior: 'smooth' }), 30);
}
$('#driveSettingsBtn').addEventListener('click', openDriveSettings);
$('#vizSettingsBtn').addEventListener('click', openDriveSettings);
$('#driveReset').addEventListener('click', () => {
  resetView2d();
  setDrivePreset(driveCam.preset);
});
document.querySelectorAll('#vizTabs [data-vtab]').forEach((b) => b.addEventListener('click', () => setVizTab(b.dataset.vtab)));
document.querySelectorAll('#drivePresets [data-dpreset]').forEach((b) => b.addEventListener('click', () => setDrivePreset(b.dataset.dpreset)));

// ── 마우스 조작 ──
let driveDown = null;
function driveTileAt(e) {
  const r = driveCanvas.getBoundingClientRect();
  const x = e.clientX - r.left, y = e.clientY - r.top;
  return driveTileHits.find((t) => x >= t.x && x <= t.x + t.w && y >= t.y && y <= t.y + t.h) || null;
}
driveCanvas.addEventListener('pointerdown', (e) => {
  driveDown = { x: e.clientX, y: e.clientY };
  if (driveTileAt(e)) return;
  if (view.vizTab === 'radar2d') {
    view2d.dragging = true; view2d.lastX = e.clientX; view2d.lastY = e.clientY;
    driveCanvas.setPointerCapture(e.pointerId);
    return;
  }
  if (view.vizTab !== 'drive3d') return;
  driveCam.dragging = true; driveCam.lastX = e.clientX; driveCam.lastY = e.clientY;
  driveCanvas.setPointerCapture(e.pointerId);
});
driveCanvas.addEventListener('pointermove', (e) => {
  if (view2d.dragging && radar2dMap) {
    // 드래그한 방향으로 지도가 따라오도록 이동
    view2d.panL += (e.clientX - view2d.lastX) / radar2dMap.scale;
    view2d.panF += (e.clientY - view2d.lastY) / radar2dMap.scale;
    view2d.lastX = e.clientX; view2d.lastY = e.clientY;
    renderVisualization();
    return;
  }
  if (!driveCam.dragging) return;
  driveCam.yaw -= (e.clientX - driveCam.lastX) * 0.006;
  driveCam.pitch = Math.max(0.08, Math.min(1.52, driveCam.pitch + (e.clientY - driveCam.lastY) * 0.005));
  driveCam.lastX = e.clientX; driveCam.lastY = e.clientY;
  renderVisualization();
});
driveCanvas.addEventListener('pointerup', (e) => {
  driveCam.dragging = false;
  view2d.dragging = false;
  const moved = driveDown ? Math.hypot(e.clientX - driveDown.x, e.clientY - driveDown.y) : 99;
  driveDown = null;
  const tile = moved < 5 ? driveTileAt(e) : null;
  if (tile) openCamZoom(tile.topic);
});
driveCanvas.addEventListener('pointermove', (e) => {
  if (!driveCam.dragging && !view2d.dragging) driveCanvas.style.cursor = driveTileAt(e) ? 'zoom-in' : view.vizTab === 'radar2d' ? 'grab' : '';
});
driveCanvas.addEventListener('dblclick', () => {
  if (view.vizTab !== 'radar2d') return;
  resetView2d();
  renderVisualization();
});
driveCanvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const f = e.deltaY < 0 ? 1 / 1.2 : 1.2;
  if (view.vizTab === 'radar2d' && radar2dMap) {
    // 마우스가 가리키는 지점을 고정한 채 확대/축소
    const r = driveCanvas.getBoundingClientRect();
    const sx = e.clientX - r.left, sy = e.clientY - r.top;
    const m = radar2dMap;
    const wx = view2d.panF + (m.y0 - sy) / m.scale;
    const wy = view2d.panL + (m.cx - sx) / m.scale;
    driveRange2d = Math.max(3, Math.min(250, driveRange2d * f));
    const scale = m.plotH / (driveRange2d * 1.2);
    const y0 = m.padT + driveRange2d * scale;
    view2d.panF = wx - (y0 - sy) / scale;
    view2d.panL = wy - (m.cx - sx) / scale;
  } else driveCam.dist = Math.max(8, Math.min(260, driveCam.dist * f));
  renderVisualization();
}, { passive: false });

// 카메라 PIP 갱신용 (데이터 수신과 별개로 ~8fps)
setInterval(() => {
  if (view.vizTab !== 'rviz' && driveSettings.camMode !== 'off' && !document.hidden) scheduleViz();
}, 120);

// ── 카메라 확대 보기 ──
// Camera Preview 카드의 <img>(MJPEG) 프레임을 캔버스에 다시 그림 → 브라우저 연결 추가 없음
const camZoomEl = $('#camZoom');
const camZoomCanvas = $('#camZoomCanvas');
const camZoomCtx = camZoomCanvas.getContext('2d');
let camZoomTopic = null;
let camZoomRaf = 0;
function camZoomTopics() { return [...liveCameraImgs().keys()]; }
function openCamZoom(topic) {
  if (!topic) return;
  camZoomTopic = topic;
  camZoomEl.hidden = false;
  if (!camZoomRaf) camZoomRaf = requestAnimationFrame(drawCamZoom);
}
function closeCamZoom() {
  camZoomEl.hidden = true;
  camZoomTopic = null;
  if (camZoomRaf) cancelAnimationFrame(camZoomRaf);
  camZoomRaf = 0;
}
function stepCamZoom(dir) {
  const list = camZoomTopics();
  if (!list.length) return;
  const i = list.indexOf(camZoomTopic);
  camZoomTopic = list[(i + dir + list.length) % list.length];
}
function drawCamZoom() {
  camZoomRaf = 0;
  if (!camZoomTopic) return;
  const rect = camZoomCanvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const W = Math.max(1, Math.floor(rect.width)), H = Math.max(1, Math.floor(rect.height));
  if (camZoomCanvas.width !== W * dpr || camZoomCanvas.height !== H * dpr) { camZoomCanvas.width = W * dpr; camZoomCanvas.height = H * dpr; }
  camZoomCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  camZoomCtx.fillStyle = '#000';
  camZoomCtx.fillRect(0, 0, W, H);
  const img = liveCameraImgs().get(camZoomTopic);
  const cam = state.cameras.find((c) => c.name === camZoomTopic);
  if (img) {
    const sc = Math.min(W / img.naturalWidth, H / img.naturalHeight);
    const w = img.naturalWidth * sc, h = img.naturalHeight * sc;
    try { camZoomCtx.drawImage(img, (W - w) / 2, (H - h) / 2, w, h); } catch (_) { /* 디코딩 중 프레임 */ }
    $('#camZoomMeta').textContent = `${img.naturalWidth}×${img.naturalHeight}${cam && cam.age_sec != null ? ` · ${cam.age_sec}s ago` : ''}`;
  } else {
    camZoomCtx.fillStyle = '#7f90a3';
    camZoomCtx.font = '500 15px "Pretendard Variable", Pretendard, system-ui, sans-serif';
    camZoomCtx.textAlign = 'center';
    camZoomCtx.fillText('프레임 대기 중 — Camera Preview에 열려 있는 카메라만 확대할 수 있습니다', W / 2, H / 2);
    camZoomCtx.textAlign = 'start';
    $('#camZoomMeta').textContent = '';
  }
  const list = camZoomTopics();
  $('#camZoomTitle').textContent = camZoomTopic;
  $('#camZoomIdx').textContent = list.length > 1 ? `${list.indexOf(camZoomTopic) + 1} / ${list.length}` : '';
  camZoomRaf = requestAnimationFrame(drawCamZoom);
}
$('#camZoomClose').addEventListener('click', closeCamZoom);
$('#camZoomPrev').addEventListener('click', () => stepCamZoom(-1));
$('#camZoomNext').addEventListener('click', () => stepCamZoom(1));
camZoomEl.addEventListener('click', (e) => { if (e.target === camZoomEl) closeCamZoom(); });
document.addEventListener('keydown', (e) => {
  if (camZoomEl.hidden) return;
  if (e.key === 'Escape') closeCamZoom();
  else if (e.key === 'ArrowLeft') stepCamZoom(-1);
  else if (e.key === 'ArrowRight') stepCamZoom(1);
});
// Camera Preview 카드 클릭 → 확대 (편집 모드 제외). 카드는 다시 만들어지므로 위임으로 처리
cameraGrid.addEventListener('click', (e) => {
  if (view.cameraEditMode) return;
  const img = e.target.closest('.camera-card img');
  if (img && img.dataset.topic) openCamZoom(img.dataset.topic);
});

setDrivePreset(driveCam.preset);
setVizTab(view.vizTab);


const events = new EventSource('/events');
events.onmessage = (event) => {
  render(JSON.parse(event.data));
  if (evtConnLost) { evtConnLost = false; pushEvent('정보', '모니터 서버 연결 복구', 'Monitor'); renderEvents(); }
};
events.onerror = async () => {
  if (!evtConnLost) { evtConnLost = true; pushEvent('경고', '모니터 서버 연결 끊김 — 재시도 중', 'Monitor'); renderEvents(); }
  try {
    const response = await fetch('/api/state', { cache: 'no-store' });
    render(await response.json());
  } catch (error) {
    clock.textContent = '연결 재시도 중';
  }
};
