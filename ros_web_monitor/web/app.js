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
  // Camera Preview 탭: cam = 카메라(원본), perc = 인지 카메라(object_3d 결과). 탭마다 슬롯·열 수를 따로 저장
  cameraLayouts: {
    cam: (() => {
      const saved = JSON.parse(localStorage.getItem('rosWebCameraLayout') || 'null');
      if (saved && saved.slots) return saved;
      // 구버전 rosWebCameraSlots 마이그레이션
      const oldSlots = JSON.parse(localStorage.getItem('rosWebCameraSlots') || 'null') || defaults.slice();
      return { cols: 2, slots: oldSlots.map((t, i) => ({ id: 'c' + i, topic: t || '' })) };
    })(),
    perc: (() => {
      const saved = JSON.parse(localStorage.getItem('rosWebPercCameraLayout') || 'null');
      if (saved && saved.slots) return saved;
      const cams = ['Front_120_Camera', 'Left_Front_Camera', 'Right_Front_Camera', 'Left_Rear_Camera', 'Rear_Camera', 'Right_Rear_Camera'];
      return { cols: 3, slots: cams.map((c, i) => ({ id: 'p' + i, topic: `/object_3d/${c}/result_image/compressed` })) };
    })(),
    // 도로 인지: road_perception 결과 (차선·노면표시·신호등·표지판)
    road: (() => {
      const saved = JSON.parse(localStorage.getItem('rosWebRoadCameraLayout') || 'null');
      if (saved && saved.slots) return saved;
      const cams = ['Front_120_Camera', 'Front_60_Camera', 'Front_30_Camera', 'Rear_Camera',
                    'Left_Front_Camera', 'Right_Front_Camera', 'Left_Rear_Camera', 'Right_Rear_Camera'];
      return { cols: 4, slots: cams.map((c, i) => ({ id: 'r' + i, topic: `/road_perception/${c}/result_image/compressed` })) };
    })(),
  },
  camTab: ['perc', 'road'].includes(localStorage.getItem('rosWebCamTab')) ? localStorage.getItem('rosWebCamTab') : 'cam',
  get cameraLayout() { return this.cameraLayouts[this.camTab]; },
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

// ── 차량 기하 (데이터 좌표계 = multi_lidar_merge output_frame) ──
// 이 차량은 전방이 +y, 좌측이 -x (Front_Top Ouster y≈+3.6~4.9, Rear_Top y=-2.3).
// 기본 차체 = remove_vehicle CropBox (x -1.3~1.7, y -2.5~5.7). 설정 → RViz → '차량 위치'에서 변경.
const VEHICLE_DEFAULTS = { forward: '+y', cx: 0.2, cy: 1.6, length: 8.2, width: 3.0, height: 2.6 };
const vehicleGeom = (() => {
  try { return { ...VEHICLE_DEFAULTS, ...(JSON.parse(localStorage.getItem('rosWebVehicleGeom') || 'null') || {}) }; }
  catch (_) { return { ...VEHICLE_DEFAULTS }; }
})();
function vehHeading() { return { '+x': 0, '+y': Math.PI / 2, '-x': Math.PI, '-y': -Math.PI / 2 }[vehicleGeom.forward] ?? Math.PI / 2; }
// 데이터 좌표 (x, y) → 차량 중심 기준 [전방, 좌측]
function toVeh(x, y) {
  const h = vehHeading(), c = Math.cos(h), sn = Math.sin(h);
  const dx = x - vehicleGeom.cx, dy = y - vehicleGeom.cy;
  return [c * dx + sn * dy, -sn * dx + c * dy];
}
// 차체 모서리까지의 거리 (차체 안이면 0)
function vehEdgeDist(x, y) {
  const [f, l] = toVeh(x, y);
  return Math.hypot(Math.max(0, Math.abs(f) - vehicleGeom.length / 2), Math.max(0, Math.abs(l) - vehicleGeom.width / 2));
}
function insideVehicle(x, y) {
  const [f, l] = toVeh(x, y);
  return Math.abs(f) <= vehicleGeom.length / 2 && Math.abs(l) <= vehicleGeom.width / 2;
}
view.yaw = -vehHeading();   // 차량 전방이 화면 위쪽

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
const CAM_GRIDS = { cam: cameraGrid, perc: $('#cameraGridPerc'), road: $('#cameraGridRoad') };
const CAM_TABS = ['cam', 'perc', 'road'];
const CAM_SLOT_PREFIX = { cam: 'c', perc: 'p', road: 'r' };
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
// /bagplay/<토픽> → <토픽> (설정에는 실시간 이름으로 저장하고, 쓸 때 effectiveTopic 으로 변환)
function logicalTopic(name) {
  const ns = state.playNamespace || '/bagplay';
  return isPlayTopic(name) ? name.slice(ns.length) : name;
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

// ── 좌측 레일 디자인: 그룹 메타(이름·색·아이콘) ──
const UI_ICONS = {
  topics: '<path d="M4 6h16M4 12h16M4 18h10"/><circle cx="19" cy="18" r="1.6" fill="currentColor" stroke="none"/>',
  groups: '<circle cx="12" cy="12" r="2"/><path d="M7.8 7.8a6 6 0 0 0 0 8.4M16.2 16.2a6 6 0 0 0 0-8.4M4.9 4.9a10 10 0 0 0 0 14.2M19.1 19.1a10 10 0 0 0 0-14.2"/>',
  system: '<rect x="6" y="6" width="12" height="12" rx="2"/><rect x="9.5" y="9.5" width="5" height="5" rx="1"/><path d="M9 3v3M15 3v3M9 18v3M15 18v3M3 9h3M3 15h3M18 9h3M18 15h3"/>',
  cameras: '<path d="M4 8h3l2-2.5h6L17 8h3v11H4z"/><circle cx="12" cy="13" r="3.4"/>',
  lidar: '<circle cx="12" cy="12" r="2.4"/><circle cx="12" cy="12" r="6"/><path d="M12 3v2M12 19v2M3 12h2M19 12h2"/>',
  radar: '<path d="M4 18a11 11 0 0 1 16 0"/><path d="M7.5 15a6.5 6.5 0 0 1 9 0"/><circle cx="12" cy="18" r="1.6" fill="currentColor" stroke="none"/>',
  localization: '<path d="M12 21s-6.5-6.2-6.5-11a6.5 6.5 0 0 1 13 0c0 4.8-6.5 11-6.5 11z"/><circle cx="12" cy="10" r="2.3"/>',
  vehicle_can: '<path d="M5 15l1.6-5.2A2 2 0 0 1 8.5 8.4h7a2 2 0 0 1 1.9 1.4L19 15v3h-2.5v-1.6h-9V18H5z"/><circle cx="8" cy="14.4" r=".9" fill="currentColor"/><circle cx="16" cy="14.4" r=".9" fill="currentColor"/>',
  perception: '<path d="M2.5 12S6 5.5 12 5.5 21.5 12 21.5 12 18 18.5 12 18.5 2.5 12 2.5 12z"/><circle cx="12" cy="12" r="3"/>',
  other: '<circle cx="6" cy="12" r="1.5"/><circle cx="12" cy="12" r="1.5"/><circle cx="18" cy="12" r="1.5"/>',
};
function uiIcon(name, size = 16) {
  return `<svg class="ui-ico" viewBox="0 0 24 24" width="${size}" height="${size}" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${UI_ICONS[name] || UI_ICONS.other}</svg>`;
}
const GROUP_META = {
  cameras:      { label: '카메라',   color: '#57c7ff' },
  lidar:        { label: '라이다',   color: '#a78bfa' },
  radar:        { label: '레이더',   color: '#ffc65b' },
  localization: { label: '측위',     color: '#4fdd97' },
  vehicle_can:  { label: '차량 CAN', color: '#ff9f5a' },
  perception:   { label: '인지',     color: '#ff7aa8' },
};
const GROUP_ORDER = Object.keys(GROUP_META);
function groupMeta(g) { return GROUP_META[g] || { label: g.replace(/_/g, ' '), color: '#8996a6', other: true }; }
function topicStatusOf(topic) {
  const subFail = topic.error && (topic.error.includes('Failed to create') || topic.error.includes('invalid allocator'));
  return (topic.error && !subFail) ? 'error' : topic.active ? 'active' : 'idle';
}

function groupForTopic(topic) {
  const ns = ((state.playNamespace || '/bagplay') + '/').toLowerCase();
  let name = topic.name.toLowerCase();
  // Play 네임스페이스 접두어는 분류 시엔 제거 (실시간과 동일하게 분류되도록)
  if (name.startsWith(ns)) name = name.slice(ns.length - 1);
  if (name.includes('camera') || name.includes('image')) return 'cameras';
  if (name.includes('ouster') || name.includes('velodyne') || name.includes('lidar') || name.includes('point') || name.includes('scan')) return 'lidar';
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
  updateLeftTabBadges();
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
        body: JSON.stringify({ all: !!cfg.all, topics: cfg.all ? [] : (cfg.topics || []), light: cfg.light !== false }),
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
  bagEdit = { all: !!view.bagConfig.all, topics: new Set(view.bagConfig.topics || []), light: view.bagConfig.light !== false };
  document.getElementById('bagLight').checked = bagEdit.light;
  document.getElementById('bagLight').disabled = recording;
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
  document.getElementById('setPanelRoad').hidden = tab !== 'road';
  document.getElementById('setPanelLens').hidden = tab !== 'lens';
  document.getElementById('bagModal').classList.toggle('road-wide', tab === 'road' || tab === 'lens');
  roadIpmActivate(tab === 'road');
  lensActivate(tab === 'lens');
  document.getElementById('bagSelCount').style.visibility = tab === 'bag' ? '' : 'hidden';
}

// ── 보정값 적용 체크박스 (설정 → 카메라 보정). 파일에 저장되어 인지 노드도 1초 안에 따라옴 ──
let calApply = { camera: true, perception: true, road: true };
let calApplyCams = {};
function calRectParam(name) {
  return calApply.camera && name.endsWith('/image_raw/compressed') ? '&rect=1' : '';
}
const CAL_NAMES = { Front_120_Camera: '전방 120', Front_60_Camera: '전방 60', Front_30_Camera: '전방 30', Rear_Camera: '후방',
  Left_Front_Camera: '좌전방', Right_Front_Camera: '우전방', Left_Rear_Camera: '좌후방', Right_Rear_Camera: '우후방' };
function calApplyRender(res) {
  const msg = document.getElementById('lensApplyMsg');
  msg.classList.toggle('err', !res.ok);
  if (!res.ok) { msg.textContent = res.error || '보정값 적용 설정을 읽지 못했습니다'; return; }
  const changed = calApply.camera !== res.settings.camera;
  calApply = res.settings;
  calApplyCams = res.cameras || {};
  document.querySelectorAll('[data-calapply]').forEach((el) => { el.checked = !!calApply[el.dataset.calapply]; });
  const missing = Object.keys(calApplyCams).filter((c) => !calApplyCams[c]).map((c) => CAL_NAMES[c] || c);
  msg.textContent = missing.length ? `보정값이 없어 원본으로 표시: ${missing.join(', ')}` : '';
  if (changed) { view.cameraOptionSig = ''; renderCameras(); }
}
async function calApplyLoad() {
  try {
    const r = await fetch('/api/calibration_apply');
    calApplyRender(r.ok ? await r.json() : { ok: false, error: '모니터를 다시 시작해야 이 기능을 쓸 수 있습니다' });
  } catch (e) {
    calApplyRender({ ok: false, error: '모니터 서버에 연결하지 못했습니다' });
  }
}
document.querySelectorAll('[data-calapply]').forEach((el) => {
  el.addEventListener('change', async () => {
    try {
      const r = await fetch('/api/calibration_apply', { method: 'POST', body: JSON.stringify({ [el.dataset.calapply]: el.checked }) });
      calApplyRender(r.ok ? await r.json() : { ok: false, error: '모니터를 다시 시작해야 이 기능을 쓸 수 있습니다' });
    } catch (e) {
      calApplyRender({ ok: false, error: '모니터 서버에 연결하지 못했습니다' });
    }
  });
});
calApplyLoad();

// ── 카메라 보정: 탭을 여는 동안만 보정 도구를 켜고 그 페이지를 불러옴 ──
let lensToken = 0;
let lensOn = false;
function lensShowStatus(text, isError) {
  const el = document.getElementById('lensStatus');
  el.textContent = text;
  el.classList.toggle('err', !!isError);
}
async function lensActivate(on) {
  if (on === lensOn) return;
  lensOn = on;
  const token = ++lensToken;
  const frame = document.getElementById('lensFrame');
  const open = document.getElementById('lensOpen');
  if (!on) {
    frame.hidden = true;
    frame.src = 'about:blank';
    open.hidden = true;
    fetch('/api/lens_calibrator/stop', { method: 'POST' }).catch(() => {});
    return;
  }
  calApplyLoad();
  lensShowStatus('보정 도구를 켜는 중…');
  try {
    const started = await fetch('/api/lens_calibrator/start', { method: 'POST' });
    if (!started.ok) throw new Error('모니터를 다시 시작해야 이 기능을 쓸 수 있습니다');
    let state = await started.json();
    for (let i = 0; i < 40 && state.ok && !state.running && token === lensToken; i += 1) {
      await new Promise((resolve) => setTimeout(resolve, 500));
      state = await (await fetch('/api/lens_calibrator')).json();
    }
    if (token !== lensToken) return;
    if (!state.ok) throw new Error(state.error || '보정 도구를 켜지 못했습니다');
    if (!state.running) throw new Error('보정 도구가 응답하지 않습니다');
    const url = `${location.protocol}//${location.hostname}:${state.port}/`;
    frame.src = `${url}?theme=dark`;
    frame.hidden = false;
    open.href = url;
    open.hidden = false;
    const local = ['localhost', '127.0.0.1'].includes(location.hostname);
    const phone = local ? `휴대폰은 같은 네트워크에서 보드 IP의 ${state.port} 포트로 접속합니다.` : `휴대폰은 ${url} 로 접속합니다.`;
    lensShowStatus(`${phone} 샘플은 자동 저장되고, 계산 결과는 페이지 안의 [저장]을 눌러야 남습니다.`);
  } catch (e) {
    if (token === lensToken) lensShowStatus(e.message || '모니터 서버에 연결하지 못했습니다', true);
  }
}
window.addEventListener('pagehide', () => {
  if (lensOn) navigator.sendBeacon('/api/lens_calibrator/stop');
});

// ── 도로 인지: 차선 인식 범위 편집 ──
// 범위 사각형 = 영상 위 4점(0~1, 가까운 왼쪽·가까운 오른쪽·먼 오른쪽·먼 왼쪽) + 실제 크기(거리·폭)
const ROAD_MARGIN = 0.15;   // 영상 밖으로 조금 나가는 꼭짓점도 잡을 수 있게 둘레 여백
const ROAD_LANE_WIDTH = 3.5;  // 국내 일반 차로 폭 (안내선 간격)
const ROAD_VP = 4;             // 소실점 핸들 번호 (0~3 = 꼭짓점)
const roadIpm = { active: false, cams: {}, saved: {}, cam: null, drag: -1, raf: 0, z: 1, cx: 0.5, cy: 0.5, needFit: true, pan: null, frozen: null, bevT: 0 };

// 단위 정사각형 (u: 왼→오, v: 가까운→먼) → 사각형 (가까운 왼쪽, 가까운 오른쪽, 먼 오른쪽, 먼 왼쪽) 투영 변환
function roadQuadMap(p) {
  const [[x0, y0], [x1, y1], [x2, y2], [x3, y3]] = p;
  const dx1 = x1 - x2, dx2 = x3 - x2, sx = x0 - x1 + x2 - x3;
  const dy1 = y1 - y2, dy2 = y3 - y2, sy = y0 - y1 + y2 - y3;
  const den = dx1 * dy2 - dx2 * dy1;
  const g = (sx * dy2 - dx2 * sy) / den, h = (dx1 * sy - sx * dy1) / den;
  const a = x1 - x0 + g * x1, b = x3 - x0 + h * x3, d = y1 - y0 + g * y1, e = y3 - y0 + h * y3;
  return (u, v) => {
    const w = g * u + h * v + 1;
    return [(a * u + b * v + x0) / w, (d * u + e * v + y0) / w];
  };
}
// 두 직선 (a-b), (c-d) 의 교점. 거의 평행이면 null
function roadCross(a, b, c, d) {
  const r = [b[0] - a[0], b[1] - a[1]], q = [d[0] - c[0], d[1] - c[1]];
  const den = r[0] * q[1] - r[1] * q[0];
  if (Math.abs(den) < 1e-7) return null;
  const t = ((c[0] - a[0]) * q[1] - (c[1] - a[1]) * q[0]) / den;
  return [a[0] + r[0] * t, a[1] + r[1] * t];
}
function roadVanish(p) { return roadCross(p[0], p[3], p[1], p[2]); }
// 소실점 vp 와 먼 변 위 한 점 q 로 먼 쪽 꼭짓점 다시 계산 (먼 변은 가까운 변과 나란하게)
function roadFarFrom(p, vp, q) {
  const q2 = [q[0] + p[1][0] - p[0][0], q[1] + p[1][1] - p[0][1]];
  const r = roadCross(p[1], vp, q, q2), l = roadCross(p[0], vp, q, q2);
  return r && l ? [p[0], p[1], r, l] : null;
}
function roadFrameSrc() {
  return roadIpm.frozen || roadEl('roadRaw');
}

function roadEl(id) { return document.getElementById(id); }
function roadStream(img, topic, tag, extra = '') {
  stopCameraStream(img);
  const ports = state.streamPorts || [];
  const base = ports.length ? `${location.protocol}//${location.hostname}:${ports[ports.length - 1]}` : '';
  const url = `${base}/stream?topic=${topic}&id=${tag}${extra}`;
  img.dataset.topic = topic;
  img.dataset.baseurl = url;
  startCameraStream(img, url);
}
function roadStatus(text, kind) {
  const el = roadEl('roadStatus');
  el.textContent = text || '';
  el.className = 'road-status' + (kind ? ' ' + kind : '');
}
function roadApplyState(cameras) {
  roadIpm.saved = cameras;
  roadIpm.cams = JSON.parse(JSON.stringify(cameras));
}
async function roadIpmActivate(on) {
  if (on === roadIpm.active) return;
  roadIpm.active = on;
  // 편집 중에는 창에 가려진 인지 탭 스트림을 멈춤 — 포트당 연결 한도(6)가 차면 결과 영상이 연결되지 못함
  view.cameraOptionSig = '';
  renderCameras();
  if (!on) {
    stopCameraStream(roadEl('roadRaw'));
    stopCameraStream(roadEl('roadResult'));
    if (roadIpm.raf) cancelAnimationFrame(roadIpm.raf);
    roadIpm.raf = 0;
    roadUnfreeze();
    return;
  }
  roadStatus('불러오는 중…');
  let res = null;
  try {
    const r = await fetch('/api/road_perception/ipm');
    res = r.ok ? await r.json() : { ok: false, error: '모니터를 다시 시작해야 이 기능을 쓸 수 있습니다 (bringup 재시작)' };
  } catch (e) {
    res = { ok: false, error: '모니터 서버에 연결하지 못했습니다' };
  }
  if (!roadIpm.active) return;
  if (!res.ok) { roadStatus(res.error, 'err'); return; }
  roadApplyState(res.cameras);
  const names = Object.keys(res.cameras);
  roadEl('roadCam').innerHTML = names.map((n) => `<option value="${escapeHtml(n)}">${escapeHtml(n)}</option>`).join('');
  roadSelectCam(names.includes(roadIpm.cam) ? roadIpm.cam : names[0]);
  roadStatus('');
  if (!roadIpm.raf) roadDraw();
}
function roadSelectCam(name) {
  roadIpm.cam = name;
  if (!name) return;
  roadEl('roadCam').value = name;
  roadFillFields();
  roadIpm.needFit = true;
  roadUnfreeze();
  // 확대해도 선명하도록 원본 해상도. 렌즈 보정이 켜져 있으면 편 영상 위에서 범위를 맞춘다
  roadStream(roadEl('roadRaw'), `/${name}/image_raw/compressed`, 'roadraw', `&full=1${roadIpm.cams[name] && roadIpm.cams[name].rect ? '&rect=1' : ''}`);
  roadStream(roadEl('roadResult'), `/road_perception/${name}/result_image/compressed`, 'roadres');
}
function roadFillFields() {
  const c = roadIpm.cams[roadIpm.cam];
  if (!c) return;
  roadEl('roadNear').value = c.near;
  roadEl('roadFar').value = c.far;
  roadEl('roadWidth').value = c.width;
  roadEl('roadCalibrated').checked = !!c.calibrated;
  roadBadge();
}
function roadDirty() {
  const a = roadIpm.cams[roadIpm.cam], b = roadIpm.saved[roadIpm.cam];
  return !!a && JSON.stringify(a) !== JSON.stringify(b);
}
function roadBadge() {
  const c = roadIpm.cams[roadIpm.cam];
  const el = roadEl('roadBadge');
  if (!c) { el.textContent = ''; return; }
  const parts = [c.custom ? '직접 설정한 범위' : '기본 범위(근사)'];
  if (roadDirty()) parts.push('저장 안 됨');
  el.textContent = parts.join(' · ');
  el.classList.toggle('dirty', roadDirty());
}
function roadConvex(p) {
  let sign = 0;
  for (let i = 0; i < 4; i++) {
    const a = p[i], b = p[(i + 1) % 4], c = p[(i + 2) % 4];
    const cr = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
    if (Math.abs(cr) < 1e-4) return false;
    if (sign && Math.sign(cr) !== sign) return false;
    sign = Math.sign(cr);
  }
  return true;
}
// 정규 좌표(0~1) ↔ 캔버스 CSS 픽셀. z=1 이면 영상이 캔버스에 꽉 참, (cx, cy) = 화면 가운데의 정규 좌표
function roadImgSize() {
  if (roadIpm.frozen) return [roadIpm.frozen.width, roadIpm.frozen.height];
  const img = roadEl('roadRaw');
  return img.naturalWidth > 0 ? [img.naturalWidth, img.naturalHeight] : [1920, 1536];
}
function roadUnfreeze() {
  roadIpm.frozen = null;
  const b = roadEl('roadFreeze');
  b.textContent = '화면 정지';
  b.setAttribute('aria-pressed', 'false');
}
// 위에서 본 미리보기: 바닥 줄(가로줄)마다 영상의 해당 선분을 잘라 그림 (아핀 띠)
function roadDrawBev() {
  const cv = roadEl('roadBev');
  const c = roadIpm.cams[roadIpm.cam];
  const W = cv.clientWidth, H = cv.clientHeight;
  if (!c || !W || !H) return;
  const dpr = window.devicePixelRatio || 1;
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const g = cv.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.fillStyle = '#080c11';
  g.fillRect(0, 0, W, H);
  const span = Math.max(0.5, c.far - c.near);
  // 실제 비율 유지: 폭 c.width, 길이 span
  const k = Math.min(W / c.width, H / span);
  const bw = c.width * k, bh = span * k, bx = (W - bw) / 2, by = (H - bh) / 2;
  const src = roadFrameSrc();
  const [aw, ah] = roadImgSize();
  const ready = roadIpm.frozen || src.naturalWidth > 0;
  const map = roadQuadMap(c.corners);
  if (ready) {
    const rows = Math.min(160, Math.max(30, Math.ceil(bh / 2)));
    for (let i = 0; i < rows; i++) {
      const v1 = 1 - i / rows, v2 = 1 - (i + 1) / rows;           // 위쪽 줄 = 먼 곳
      const y1 = by + bh * i / rows, y2 = by + bh * (i + 1) / rows;
      const [ax, ay] = map(0, v1), [bx1, by1] = map(1, v1), [cx, cy] = map(0, v2);
      const sx0 = ax * aw, sy0 = ay * ah, ux = bx1 * aw - sx0, uy = by1 * ah - sy0, vx = cx * aw - sx0, vy = cy * ah - sy0;
      const det = ux * vy - uy * vx;
      if (Math.abs(det) < 1e-6) continue;
      // 영상 픽셀 → 미리보기: S0→(bx,y1), S0+U→(bx+bw,y1), S0+V→(bx,y2)
      const a = (bw * vy) / det, c2 = (-bw * vx) / det;
      const b = (-(y2 - y1) * uy) / det, d = ((y2 - y1) * ux) / det;
      g.save();
      g.beginPath();
      g.rect(bx, y1, bw, y2 - y1 + 0.7);        // 줄 사이 틈 방지
      g.clip();
      g.setTransform(dpr * a, dpr * b, dpr * c2, dpr * d, dpr * (bx - a * sx0 - c2 * sy0), dpr * (y1 - b * sx0 - d * sy0));
      g.drawImage(src, 0, 0, aw, ah);
      g.restore();
    }
  }
  // 눈금: 세로선 3.5 m(차로 폭), 가로선 5 m
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.lineWidth = 1;
  g.strokeStyle = 'rgba(87,199,255,.75)';
  g.setLineDash([5, 4]);
  g.beginPath();
  const n = Math.floor(c.width / ROAD_LANE_WIDTH);
  for (let i = 0; i <= n; i++) {
    const x = bx + bw / 2 + (i - n / 2) * ROAD_LANE_WIDTH * k;
    g.moveTo(x, by); g.lineTo(x, by + bh);
  }
  g.stroke();
  g.strokeStyle = 'rgba(255,255,255,.4)';
  g.beginPath();
  for (let d = Math.ceil(c.near / 5) * 5; d <= c.far; d += 5) {
    const y = by + bh - (d - c.near) * k;
    g.moveTo(bx, y); g.lineTo(bx + bw, y);
  }
  g.stroke();
  g.setLineDash([]);
  g.font = '600 10px sans-serif';
  g.fillStyle = '#d7e4ef';
  for (let d = Math.ceil(c.near / 5) * 5; d <= c.far; d += 5) g.fillText(`${d} m`, bx + 3, by + bh - (d - c.near) * k - 3);
}
function roadView() {
  const cv = roadEl('roadCanvas');
  const W = cv.clientWidth, H = cv.clientHeight;
  const [aw, ah] = roadImgSize();
  const s = Math.min(W / aw, H / ah) * roadIpm.z;
  const iw = aw * s, ih = ah * s;
  return { W, H, iw, ih, ox: W / 2 - roadIpm.cx * iw, oy: H / 2 - roadIpm.cy * ih };
}
// 영상과 사각형이 모두 보이게 맞춤
function roadFit() {
  const c = roadIpm.cams[roadIpm.cam];
  const cv = roadEl('roadCanvas');
  const W = cv.clientWidth, H = cv.clientHeight;
  if (!W || !H) return;
  const xs = [0, 1, ...(c ? c.corners.map((p) => p[0]) : [])], ys = [0, 1, ...(c ? c.corners.map((p) => p[1]) : [])];
  const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
  const [aw, ah] = roadImgSize();
  const base = Math.min(W / aw, H / ah);
  roadIpm.z = Math.min(W / ((x1 - x0) * aw), H / ((y1 - y0) * ah)) / base * 0.96;
  roadIpm.cx = (x0 + x1) / 2;
  roadIpm.cy = (y0 + y1) / 2;
  roadIpm.needFit = false;
  roadEl('roadZoomLabel').textContent = `${Math.round(roadIpm.z * 100)}%`;
}
// 화면 위 (mx, my) 지점을 고정한 채 확대/축소
function roadZoomAt(f, mx, my) {
  const v = roadView();
  const u = (mx - v.ox) / v.iw, w = (my - v.oy) / v.ih;
  roadIpm.z = Math.max(0.5, Math.min(10, roadIpm.z * f));
  const m = roadView();
  roadIpm.cx -= (mx - (m.ox + u * m.iw)) / m.iw;
  roadIpm.cy -= (my - (m.oy + w * m.ih)) / m.ih;
  roadEl('roadZoomLabel').textContent = `${Math.round(roadIpm.z * 100)}%`;
}
function roadDraw() {
  roadIpm.raf = 0;
  if (!roadIpm.active) return;
  const cv = roadEl('roadCanvas');
  if (roadIpm.needFit) roadFit();
  const v = roadView();
  const dpr = window.devicePixelRatio || 1;
  if (v.W > 0 && (cv.width !== Math.round(v.W * dpr) || cv.height !== Math.round(v.H * dpr))) {
    cv.width = Math.round(v.W * dpr);
    cv.height = Math.round(v.H * dpr);
  }
  const g = cv.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.fillStyle = '#080c11';
  g.fillRect(0, 0, v.W, v.H);
  const img = roadEl('roadRaw');
  if (roadIpm.frozen) g.drawImage(roadIpm.frozen, v.ox, v.oy, v.iw, v.ih);
  else if (img.naturalWidth > 0) g.drawImage(img, v.ox, v.oy, v.iw, v.ih);
  else {
    g.fillStyle = '#7f8c9a';
    g.font = '12px sans-serif';
    g.textAlign = 'center';
    g.fillText('카메라 영상 기다리는 중', v.W / 2, v.H / 2);
    g.textAlign = 'left';
  }
  const c = roadIpm.cams[roadIpm.cam];
  if (c) {
    const pts = c.corners.map(([u, w]) => [v.ox + u * v.iw, v.oy + w * v.ih]);
    g.beginPath();
    pts.forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y)));
    g.closePath();
    g.fillStyle = 'rgba(87,199,255,.16)';
    g.fill();
    g.strokeStyle = '#57c7ff';
    g.lineWidth = 2;
    g.stroke();
    // 가까운 변 강조 + 거리 표시
    g.beginPath(); g.moveTo(...pts[0]); g.lineTo(...pts[1]);
    g.strokeStyle = '#ffc65b'; g.lineWidth = 3; g.stroke();
    // 차로 폭 안내선: 입력한 폭 기준 3.5 m 간격 (실제 차선과 겹치면 폭이 맞음)
    if (c.width > ROAD_LANE_WIDTH) {
      const qm = roadQuadMap(pts);
      g.save();
      g.setLineDash([7, 6]); g.lineWidth = 1.5; g.strokeStyle = 'rgba(255,255,255,.85)';
      for (let d = ROAD_LANE_WIDTH; d < c.width - 0.2; d += ROAD_LANE_WIDTH) {
        const t = d / c.width;
        g.beginPath();
        g.moveTo(...qm(t, 0));
        g.lineTo(...qm(t, 1));
        g.stroke();
      }
      g.restore();
    }
    // 소실점 + 연장선: 실제 차선에 겹치면 맞음 (사각형 안쪽뿐 아니라 화면 끝까지 보임)
    const vp = roadVanish(c.corners);
    if (vp) {
      const vs = [v.ox + vp[0] * v.iw, v.oy + vp[1] * v.ih];
      g.save();
      g.setLineDash([10, 6]); g.lineWidth = 1.5; g.strokeStyle = 'rgba(87,199,255,.9)';
      for (const q of [pts[0], pts[1]]) {
        const dx = q[0] - vs[0], dy = q[1] - vs[1], L = Math.hypot(dx, dy) || 1;
        const reach = v.W + v.H;
        g.beginPath(); g.moveTo(...vs); g.lineTo(vs[0] + dx / L * reach, vs[1] + dy / L * reach); g.stroke();
      }
      g.restore();
      g.beginPath(); g.arc(vs[0], vs[1], roadIpm.drag === ROAD_VP ? 9 : 7, 0, Math.PI * 2);
      g.fillStyle = roadIpm.drag === ROAD_VP ? '#ffffff' : '#ff7aa8';
      g.fill();
      g.lineWidth = 2; g.strokeStyle = '#06101a'; g.stroke();
      g.font = '600 11px sans-serif';
      g.fillStyle = '#ffb3cc';
      g.fillText('소실점', vs[0] + 11, vs[1] - 9);
    }
    g.font = '600 11px sans-serif';
    const tag = (text, x, y, col) => {
      const w = g.measureText(text).width + 10;
      const tx = Math.max(2, Math.min(v.W - w - 2, x - w / 2)), ty = Math.max(2, Math.min(v.H - 18, y - 9));
      g.fillStyle = 'rgba(8,12,17,.82)'; g.fillRect(tx, ty, w, 18);
      g.fillStyle = col; g.fillText(text, tx + 5, ty + 13);
    };
    // 글자가 꼭짓점을 가리지 않도록 변 바깥쪽에 표시
    tag(`가까운 변 ${c.near} m · 폭 ${c.width} m`, (pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2 + 22, '#ffc65b');
    tag(`먼 변 ${c.far} m`, (pts[2][0] + pts[3][0]) / 2, (pts[2][1] + pts[3][1]) / 2 - 22, '#9fdcff');
    pts.forEach(([x, y], i) => {
      g.beginPath(); g.arc(x, y, roadIpm.drag === i ? 8 : 6, 0, Math.PI * 2);
      g.fillStyle = roadIpm.drag === i ? '#ffffff' : i < 2 ? '#ffc65b' : '#57c7ff';
      g.fill();
      g.lineWidth = 2; g.strokeStyle = '#06101a'; g.stroke();
    });
  }
  const nowT = performance.now();
  if (roadIpm.drag >= 0 || roadIpm.frozen ? nowT - roadIpm.bevT > 60 : nowT - roadIpm.bevT > 150) {
    roadIpm.bevT = nowT;
    roadDrawBev();
  }
  roadIpm.raf = requestAnimationFrame(roadDraw);
}
function roadPointer(e) {
  const r = roadEl('roadCanvas').getBoundingClientRect();
  return [e.clientX - r.left, e.clientY - r.top];
}
roadEl('roadCanvas').addEventListener('pointerdown', (e) => {
  const c = roadIpm.cams[roadIpm.cam];
  if (!c) return;
  const v = roadView();
  const [mx, my] = roadPointer(e);
  let best = -1, bd = 22;
  const vp = roadVanish(c.corners);
  [...c.corners, ...(vp ? [vp] : [])].forEach(([u, w], i) => {
    const d = Math.hypot(v.ox + u * v.iw - mx, v.oy + w * v.ih - my);
    if (d < bd) { bd = d; best = i; }
  });
  e.currentTarget.setPointerCapture(e.pointerId);
  e.preventDefault();
  if (best < 0) {            // 빈 곳: 화면 이동
    roadIpm.pan = { x: mx, y: my, cx: roadIpm.cx, cy: roadIpm.cy };
    e.currentTarget.style.cursor = 'grabbing';
    return;
  }
  roadIpm.drag = best;
});
roadEl('roadCanvas').addEventListener('pointermove', (e) => {
  const c = roadIpm.cams[roadIpm.cam];
  const v = roadView();
  const [mx, my] = roadPointer(e);
  if (roadIpm.pan) {
    roadIpm.cx = roadIpm.pan.cx - (mx - roadIpm.pan.x) / v.iw;
    roadIpm.cy = roadIpm.pan.cy - (my - roadIpm.pan.y) / v.ih;
    return;
  }
  if (roadIpm.drag < 0) {
    const vpH = c && roadVanish(c.corners);
    const near = c && [...c.corners, ...(vpH ? [vpH] : [])].some(([u, w]) => Math.hypot(v.ox + u * v.iw - mx, v.oy + w * v.ih - my) < 22);
    e.currentTarget.style.cursor = near ? 'grab' : 'move';
    return;
  }
  const clampN = (q) => +Math.max(-ROAD_MARGIN, Math.min(1 + ROAD_MARGIN, q)).toFixed(4);
  const q = [(mx - v.ox) / v.iw, (my - v.oy) / v.ih];
  const cur = c.corners.map((p) => p.slice());
  const lock = roadEl('roadVpLock').checked;
  const curVp = roadVanish(cur);
  let next = null;
  if (roadIpm.drag === ROAD_VP) {
    // 소실점 이동: 가까운 변은 그대로, 먼 쪽 꼭짓점이 따라옴 (먼 변 높이 유지)
    next = roadFarFrom(cur, q, [(cur[2][0] + cur[3][0]) / 2, (cur[2][1] + cur[3][1]) / 2]);
  } else if (lock && curVp && roadIpm.drag < 2) {
    // 가까운 꼭짓점 이동: 소실점 유지, 같은 쪽 먼 꼭짓점이 연장선을 따라 움직임
    const moved = cur.map((p) => p.slice());
    moved[roadIpm.drag] = [clampN(q[0]), clampN(q[1])];
    next = roadFarFrom(moved, curVp, [(cur[2][0] + cur[3][0]) / 2, (cur[2][1] + cur[3][1]) / 2]);
  } else if (lock && curVp) {
    // 먼 꼭짓점 이동: 소실점 유지, 먼 변 전체가 앞뒤로 움직임
    next = roadFarFrom(cur, curVp, q);
  } else {
    next = cur;
    next[roadIpm.drag] = [clampN(q[0]), clampN(q[1])];
  }
  if (!next) return;
  next = next.map((p) => p.map((n) => +n.toFixed(4)));
  // 꼬이거나 저장 가능한 범위를 벗어나는 위치로는 움직이지 않음
  if (!roadConvex(next) || next.flat().some((n) => n < -ROAD_MARGIN || n > 1 + ROAD_MARGIN)) return;
  c.corners = next;
  roadBadge();
  roadStatus('');
});
const roadDragEnd = () => { roadIpm.drag = -1; roadIpm.pan = null; };
roadEl('roadCanvas').addEventListener('wheel', (e) => {
  e.preventDefault();
  const [mx, my] = roadPointer(e);
  roadZoomAt(e.deltaY < 0 ? 1.2 : 1 / 1.2, mx, my);
}, { passive: false });
roadEl('roadCanvas').addEventListener('dblclick', roadFit);
const roadZoomCenter = (f) => { const v = roadView(); roadZoomAt(f, v.W / 2, v.H / 2); };
roadEl('roadZoomIn').addEventListener('click', () => roadZoomCenter(1.3));
roadEl('roadZoomOut').addEventListener('click', () => roadZoomCenter(1 / 1.3));
roadEl('roadZoomFit').addEventListener('click', roadFit);
roadEl('roadFreeze').addEventListener('click', (e) => {
  const raw = roadEl('roadRaw');
  if (roadIpm.frozen) {
    roadUnfreeze();
    return;
  }
  if (!(raw.naturalWidth > 0)) { roadStatus('아직 카메라 영상이 없습니다.', 'err'); return; }
  const f = document.createElement('canvas');
  f.width = raw.naturalWidth;
  f.height = raw.naturalHeight;
  f.getContext('2d').drawImage(raw, 0, 0);
  roadIpm.frozen = f;
  e.currentTarget.textContent = '정지 해제';
  e.currentTarget.setAttribute('aria-pressed', 'true');
});
document.querySelectorAll('.road-lanes [data-lanes]').forEach((b) => b.addEventListener('click', () => {
  const c = roadIpm.cams[roadIpm.cam];
  if (!c) return;
  c.width = Number(b.dataset.lanes) * ROAD_LANE_WIDTH;
  roadFillFields();
  roadStatus('');
}));
roadEl('roadCanvas').addEventListener('pointerup', roadDragEnd);
roadEl('roadCanvas').addEventListener('pointercancel', roadDragEnd);
roadEl('roadCam').addEventListener('change', (e) => { roadSelectCam(e.target.value); roadStatus(''); });
[['roadNear', 'near'], ['roadFar', 'far'], ['roadWidth', 'width']].forEach(([id, key]) => {
  roadEl(id).addEventListener('input', (e) => {
    const c = roadIpm.cams[roadIpm.cam];
    const val = parseFloat(e.target.value);
    if (c && Number.isFinite(val)) { c[key] = val; roadBadge(); roadStatus(''); }
  });
});
roadEl('roadCalibrated').addEventListener('change', (e) => {
  const c = roadIpm.cams[roadIpm.cam];
  if (c) { c.calibrated = e.target.checked; roadBadge(); }
});
async function roadPost(path, body, okText) {
  roadStatus('저장 중…');
  let res;
  try {
    const r = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    res = r.ok ? await r.json() : { ok: false, error: `서버 오류 (${r.status}) — bringup 을 다시 시작했는지 확인하세요` };
  } catch (e) {
    res = { ok: false, error: '모니터 서버에 연결하지 못했습니다' };
  }
  if (!res.ok) { roadStatus(res.error, 'err'); return; }
  const others = roadIpm.cams;           // 다른 카메라의 저장 안 한 편집은 유지
  const cur = roadIpm.cam;
  const edits = Object.keys(others).filter((n) => n !== cur && JSON.stringify(others[n]) !== JSON.stringify(roadIpm.saved[n]));
  const kept = Object.fromEntries(edits.map((n) => [n, others[n]]));
  roadApplyState(res.cameras);
  Object.assign(roadIpm.cams, kept);
  roadFillFields();
  roadStatus(okText, 'ok');
}
roadEl('roadSave').addEventListener('click', () => {
  const c = roadIpm.cams[roadIpm.cam];
  if (!c) return;
  roadPost('/api/road_perception/ipm', { camera: roadIpm.cam, corners: c.corners, near: c.near, far: c.far, width: c.width, offset: c.offset, calibrated: c.calibrated, rect: !!c.rect },
    `${roadIpm.cam} 범위를 저장했습니다. 1~2초 뒤 결과에 반영됩니다.`);
});
roadEl('roadRevert').addEventListener('click', () => {
  if (!roadIpm.saved[roadIpm.cam]) return;
  roadIpm.cams[roadIpm.cam] = JSON.parse(JSON.stringify(roadIpm.saved[roadIpm.cam]));
  roadFillFields();
  roadStatus('');
});
roadEl('roadReset').addEventListener('click', () => {
  if (!roadIpm.cam || !window.confirm(`${roadIpm.cam} 의 직접 설정한 범위를 지우고 기본값으로 되돌릴까요?`)) return;
  roadPost('/api/road_perception/ipm/reset', { camera: roadIpm.cam }, `${roadIpm.cam} 을 기본 범위로 되돌렸습니다.`);
});

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

// 창 아래 공통 '저장': 편집했지만 저장 안 한 차선 범위도 함께 저장 (실패하면 이벤트 로그에 남김)
async function roadSaveAllDirty() {
  for (const name of Object.keys(roadIpm.cams)) {
    const c = roadIpm.cams[name];
    if (JSON.stringify(c) === JSON.stringify(roadIpm.saved[name])) continue;
    try {
      const r = await fetch('/api/road_perception/ipm', { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ camera: name, corners: c.corners, near: c.near, far: c.far, width: c.width, offset: c.offset, calibrated: c.calibrated, rect: !!c.rect }) });
      const res = r.ok ? await r.json() : { ok: false, error: `서버 오류 ${r.status}` };
      if (!res.ok) pushEvent('경고', `${name} 차선 범위 저장 실패: ${res.error}`, 'Road');
    } catch (e) {
      pushEvent('경고', `${name} 차선 범위 저장 실패: 서버 연결 안 됨`, 'Road');
    }
  }
  renderEvents();
}
function closeSettingsModal(save) {
  if (save) roadSaveAllDirty();
  roadIpmActivate(false);
  lensActivate(false);
  document.getElementById('bagOverlay').hidden = true;
  document.getElementById('bagModal').hidden = true;
  if (save) {
    if (bagEdit) {
      view.bagConfig = { all: bagEdit.all, topics: [...bagEdit.topics], light: document.getElementById('bagLight').checked };
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

// ── RViz 탭 렌더러 ──
// 좌표: x=전방, y=좌측, z=위. project3D() 는 view.yaw/pitch/zoom/pan 기반 원근 투영.
// 점 색상 모드(view.vizColorMode): layer(레이어 색) | height(높이 색상맵) | range(거리 색상맵)
const VIZ_COLORMAP = ['#3b4cc0', '#2c7bd6', '#17b2e0', '#2fd3b4', '#7fe36a', '#d6e04a', '#fbb13c', '#f36b3b', '#d93a5b'];
const VIZ_BINS = 24;
const VIZ_KIND_COLOR = { track: '#3ddc97', object: '#ff5d6c', detection: '#ff5d6c', radar: '#ffc04d', marker: '#c084fc' };
view.vizColorMode = localStorage.getItem('rosWebVizColorMode') || 'height';
view.vizLabels = localStorage.getItem('rosWebVizLabels') !== 'off';
let vizHits = [];          // 툴팁용: [{x, y, box, color}]
let vizMouse = null;       // 캔버스 내 마우스 좌표
let vizFrameTimes = [];    // 초당 갱신 횟수 계산

const vizBinColors = (() => {
  const hex = (h) => [1, 3, 5].map((i) => parseInt(h.slice(i, i + 2), 16));
  const stops = VIZ_COLORMAP.map(hex);
  return Array.from({ length: VIZ_BINS }, (_, i) => {
    const t = i / (VIZ_BINS - 1) * (stops.length - 1);
    const a = stops[Math.floor(t)], b = stops[Math.min(stops.length - 1, Math.floor(t) + 1)], f = t - Math.floor(t);
    return `rgb(${a.map((v, k) => Math.round(v + (b[k] - v) * f)).join(',')})`;
  });
})();

// 표시할 포인트 레이어 목록. 병합 클라우드(merge_output) 레이어에 데이터가 없으면
// (예: 녹화가 밀려 bag 에 병합 클라우드가 거의 없는 경우) 백엔드가 병합 좌표계로 변환한
// 개별 라이다(merge_part)를 대신 그 레이어 색으로 표시한다.
function vizPointLayers(viz, topicToLayer) {
  const out = [];
  for (const layer of viz.point_layers || []) {
    const lyr = topicToLayer[layer.topic];
    if (lyr) out.push({ ...layer, lyr });
  }
  const merged = viz.merge_output ? topicToLayer[effectiveTopic(viz.merge_output)] : null;
  if (merged && !out.some((l) => l.lyr === merged)) {
    const inPlay = view.mode === 'play';
    for (const layer of viz.point_layers || []) {
      if (layer.merge_part && isPlayTopic(layer.topic) === inPlay && !topicToLayer[layer.topic]) out.push({ ...layer, lyr: merged, fallback: true });
    }
  }
  return out;
}

function sizeVizCanvas() {
  const rect = vizCanvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(320, Math.floor(rect.width));
  const height = Math.max(240, Math.floor(rect.height));
  if (vizCanvas.width !== Math.floor(width * dpr) || vizCanvas.height !== Math.floor(height * dpr)) {
    vizCanvas.width = Math.floor(width * dpr);
    vizCanvas.height = Math.floor(height * dpr);
  }
  vizCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { width, height };
}

function renderVisualization() {
  if (view.vizTab && view.vizTab !== 'rviz') { renderDriveView(); return; }
  const viz = state.visualization;
  const { width, height } = sizeVizCanvas();
  drawVizBackground(width, height);
  drawGrid3D(width, height);

  const topicToLayer = {};
  for (const lyr of view.vizLayers) {
    // Play 뷰에서는 /bagplay/* 로 리맵된 토픽만 매칭되어 실시간 데이터와 자동으로 분리됨
    if (lyr.topic && lyr.visible) topicToLayer[effectiveTopic(lyr.topic)] = lyr;
  }

  // 1) 포인트: 색상 구간별로 모아 한 번에 칠함 (fillStyle 변경 최소화), 'lighter' 합성으로 은은한 발광
  let pointCount = 0;
  const range = Math.max(10, state.visualization.range_m || 80);
  vizCtx.globalCompositeOperation = 'lighter';
  for (const layer of vizPointLayers(viz, topicToLayer)) {
    const lyr = layer.lyr;
    const mode = view.vizColorMode;
    const bins = mode === 'layer' ? null : Array.from({ length: VIZ_BINS }, () => []);
    if (!bins) vizCtx.fillStyle = lyr.color;
    vizCtx.globalAlpha = 0.85;
    for (const p of layer.points || []) {
      const pr = project3D(p[0], p[1], p[2] || 0, width, height);
      if (!pr.visible) continue;
      const size = Math.max(1.2, Math.min(3.2, pr.scale * 0.02));
      pointCount += 1;
      if (!bins) { vizCtx.fillRect(pr.x - size / 2, pr.y - size / 2, size, size); continue; }
      const t = mode === 'height' ? ((p[2] || 0) + 2) / 6 : Math.hypot(p[0], p[1]) / Math.min(range, 60);
      bins[Math.max(0, Math.min(VIZ_BINS - 1, Math.floor(t * VIZ_BINS)))].push(pr.x - size / 2, pr.y - size / 2, size);
    }
    if (bins) {
      bins.forEach((arr, i) => {
        if (!arr.length) return;
        vizCtx.fillStyle = vizBinColors[i];
        for (let k = 0; k < arr.length; k += 3) vizCtx.fillRect(arr[k], arr[k + 1], arr[k + 2], arr[k + 2]);
      });
    }
  }
  vizCtx.globalAlpha = 1;
  vizCtx.globalCompositeOperation = 'source-over';

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
  vizHits = [];
  const labels = [];
  for (const item of items) {
    if (item.type === 'marker') { markers += draw3DMarker(item.marker, item.color, width, height); }
    else if (item.type === 'box') { boxes += draw3DBox(item.box, item.color, width, height, labels); }
  }
  drawEgo3D(width, height);
  if (view.vizLabels) drawVizLabels(labels);
  drawVizGizmo(width, height);
  drawVizScaleBar(width, height);
  updateVizTooltip();

  const now = performance.now();
  vizFrameTimes.push(now);
  while (vizFrameTimes.length && now - vizFrameTimes[0] > 1000) vizFrameTimes.shift();
  vizStats.textContent = `점 ${pointCount.toLocaleString()} · 박스 ${boxes} · 마커 ${markers} · 갱신 ${vizFrameTimes.length}/s`;
  const src = document.getElementById('vizSrc');
  if (src) {
    const veh = driveVehicleState();
    src.textContent = (view.mode === 'play' ? 'Play' : '실시간') + (veh ? ` · ${Math.abs(veh.speed).toFixed(1)} km/h${veh.source === 'gps' ? ' (GPS)' : ''}` : '');
    src.classList.toggle('play', view.mode === 'play');
  }
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

function drawVizBackground(width, height) {
  const bg = vizCtx.createRadialGradient(width / 2, height * 0.55, 20, width / 2, height * 0.55, Math.max(width, height) * 0.75);
  bg.addColorStop(0, '#0b1724');
  bg.addColorStop(0.6, '#070d15');
  bg.addColorStop(1, '#04070b');
  vizCtx.fillStyle = bg;
  vizCtx.fillRect(0, 0, width, height);
}

function drawGrid3D(width, height) {
  const range = Math.max(30, Math.min(200, state.visualization.range_m || 80));
  const step = range <= 50 ? 5 : range <= 120 ? 10 : 20;
  vizCtx.lineWidth = 1;
  // 격자: 멀수록 옅게
  for (let v = -range; v <= range; v += step) {
    const fade = 1 - Math.abs(v) / range;
    const major = v === 0;
    vizCtx.strokeStyle = major ? 'rgba(120,170,220,0.32)' : `rgba(120,170,220,${(0.05 + fade * 0.09).toFixed(3)})`;
    drawWorldLine(-range, v, 0, range, v, 0, width, height);
    drawWorldLine(v, -range, 0, v, range, 0, width, height);
  }
  // 거리 링 + 라벨
  const rings = [5, 10, 20, 30, 50, 70, 100, 150, 200].filter((r) => r <= range && (range <= 60 || r >= 10));
  vizCtx.font = '500 11px "Pretendard Variable", Pretendard, system-ui, sans-serif';
  for (const r of rings) {
    vizCtx.strokeStyle = r === rings[rings.length - 1] ? 'rgba(56,189,248,0.30)' : 'rgba(56,189,248,0.16)';
    vizCtx.setLineDash(r <= 10 ? [] : [4, 6]);
    vizCtx.beginPath();
    let started = false;
    for (let i = 0; i <= 96; i++) {
      const a = i / 96 * Math.PI * 2;
      const p = project3D(vehicleGeom.cx + Math.cos(a) * r, vehicleGeom.cy + Math.sin(a) * r, 0, width, height);
      if (!p.visible) { started = false; continue; }
      if (!started) { vizCtx.moveTo(p.x, p.y); started = true; } else vizCtx.lineTo(p.x, p.y);
    }
    vizCtx.stroke();
    const la = vehHeading() - 0.35;
    const lp = project3D(vehicleGeom.cx + Math.cos(la) * r, vehicleGeom.cy + Math.sin(la) * r, 0, width, height);
    if (lp.visible && lp.x > 0 && lp.x < width && lp.y > 0 && lp.y < height) {
      vizCtx.fillStyle = 'rgba(125,196,255,0.75)';
      vizCtx.fillText(`${r} m`, lp.x + 3, lp.y - 3);
    }
  }
  vizCtx.setLineDash([]);
}

// 화면 오른쪽 아래 방향 표시 (X=전방 빨강, Y=좌측 초록, Z=위 파랑)
function drawVizGizmo(width, height) {
  const cx = width - 46, cy = height - 54, L = 24;
  const cyw = Math.cos(view.yaw), syw = Math.sin(view.yaw), sp = Math.sin(view.pitch), cp = Math.cos(view.pitch);
  const dir = (x, y, z) => {
    const forward = cyw * x - syw * y, side = syw * x + cyw * y;
    return [-side, -(forward * sp + z * cp)];
  };
  vizCtx.fillStyle = 'rgba(8,12,18,0.7)';
  vizCtx.strokeStyle = 'rgba(120,170,220,0.25)';
  vizCtx.beginPath(); vizCtx.arc(cx, cy, L + 12, 0, Math.PI * 2); vizCtx.fill(); vizCtx.stroke();
  const axes = [[[1, 0, 0], '#ff5d6c', 'X'], [[0, 1, 0], '#3ddc97', 'Y'], [[0, 0, 1], '#38bdf8', 'Z']];
  vizCtx.font = '700 10px "Pretendard Variable", Pretendard, system-ui, sans-serif';
  vizCtx.lineWidth = 2.2;
  for (const [v, color, label] of axes) {
    const [dx, dy] = dir(...v);
    vizCtx.strokeStyle = color;
    vizCtx.beginPath(); vizCtx.moveTo(cx, cy); vizCtx.lineTo(cx + dx * L, cy + dy * L); vizCtx.stroke();
    vizCtx.fillStyle = color;
    vizCtx.fillText(label, cx + dx * (L + 6) - 3, cy + dy * (L + 6) + 3);
  }
}

// 축척 막대: 자차 위치의 화면 배율로 1·2·5 단위 길이 표시
function drawVizScaleBar(width, height) {
  const p = project3D(0, 0, 0, width, height);
  if (!p.visible) return;
  const target = 90;
  const raw = target / p.scale;
  const pow = Math.pow(10, Math.floor(Math.log10(raw)));
  const m = raw / pow;
  const len = (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * pow;
  const px = len * p.scale;
  const x1 = width - 100 - px, y = height - 18;
  vizCtx.strokeStyle = 'rgba(232,238,245,0.7)';
  vizCtx.lineWidth = 1.5;
  vizCtx.beginPath();
  vizCtx.moveTo(x1, y - 5); vizCtx.lineTo(x1, y); vizCtx.lineTo(x1 + px, y); vizCtx.lineTo(x1 + px, y - 5);
  vizCtx.stroke();
  vizCtx.fillStyle = 'rgba(232,238,245,0.8)';
  vizCtx.font = '500 11px "Pretendard Variable", Pretendard, system-ui, sans-serif';
  vizCtx.fillText(`${len >= 1 ? len : len.toFixed(1)} m`, x1 + px / 2 - 12, y - 8);
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

function draw3DMarker(marker, color, width, height) {
  const points = marker.points || [];
  if (!points.length) return 0;
  vizCtx.strokeStyle = color;
  vizCtx.fillStyle = color;
  if ([4, 5].includes(marker.type)) {
    // 선: 굵은 반투명 선 위에 가는 선을 겹쳐 발광 효과
    for (const [w, a] of [[4, 0.18], [1.6, 0.95]]) {
      vizCtx.lineWidth = w;
      vizCtx.globalAlpha = a;
      vizCtx.beginPath();
      let started = false;
      points.forEach((point, index) => {
        const p = project3D(point[0], point[1], point[2] || 0, width, height);
        if (!p.visible) return;
        if (!started || marker.type === 5 && index % 2 === 0) { vizCtx.moveTo(p.x, p.y); started = true; }
        else vizCtx.lineTo(p.x, p.y);
      });
      vizCtx.stroke();
    }
    vizCtx.globalAlpha = 1;
    return 1;
  }
  let count = 0;
  // radar 객체 포인트는 점이 너무 많지 않을 때 각 점에 거리 표시
  const showDist = marker.kind === 'radar' && points.length <= 80 && view.vizLabels;
  for (const point of points) {
    const p = project3D(point[0], point[1], point[2] || 0, width, height);
    if (!p.visible) continue;
    const r = Math.max(2.2, Math.min(5, p.scale * 0.03));
    vizCtx.globalAlpha = 0.25;
    vizCtx.beginPath(); vizCtx.arc(p.x, p.y, r * 2.2, 0, Math.PI * 2); vizCtx.fill();
    vizCtx.globalAlpha = 1;
    vizCtx.beginPath(); vizCtx.arc(p.x, p.y, r, 0, Math.PI * 2); vizCtx.fill();
    if (showDist) {
      const d = Math.hypot(point[0], point[1]);
      vizCtx.font = '600 10px "Pretendard Variable", Pretendard, system-ui, sans-serif';
      const text = `${d.toFixed(0)}m`;
      vizCtx.lineWidth = 3;
      vizCtx.strokeStyle = 'rgba(4,7,11,0.85)';
      vizCtx.strokeText(text, p.x + 6, p.y - 4);
      vizCtx.fillText(text, p.x + 6, p.y - 4);
    }
    count += 1;
  }
  return count;
}

function hexToRgba(hex, a) {
  const h = /^#([0-9a-f]{6})$/i.test(hex || '') ? hex : '#9fb2c4';
  return `rgba(${parseInt(h.slice(1, 3), 16)},${parseInt(h.slice(3, 5), 16)},${parseInt(h.slice(5, 7), 16)},${a})`;
}

// 박스: 반투명 면(바닥·윗면) + 모서리 + 진행 방향 표시. 라벨은 labels 에 모아 마지막에 겹침 없이 그림
function draw3DBox(box, color, width, height, labels) {
  const sx = Math.max(box.sx || 1, 0.4) / 2;
  const sy = Math.max(box.sy || 1, 0.4) / 2;
  const sz = Math.max(box.sz || 1, 0.4) / 2;
  const yaw = box.yaw || 0;
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const bx = box.x || 0, by = box.y || 0, bz = box.z || 0;
  const P = (dx, dy, dz) => project3D(bx + c * dx - s * dy, by + s * dx + c * dy, bz + dz, width, height);
  const corners = [];
  for (const dx of [-sx, sx]) for (const dy of [-sy, sy]) for (const dz of [-sz, sz]) corners.push(P(dx, dy, dz));
  if (corners.some((p) => !p.visible)) return 0;
  const dist = vehEdgeDist(bx, by);
  const near = dist < PROX_WARN_M && !insideVehicle(bx, by);
  const fillFace = (idx, alpha) => {
    vizCtx.beginPath();
    idx.forEach((k, i) => (i ? vizCtx.lineTo(corners[k].x, corners[k].y) : vizCtx.moveTo(corners[k].x, corners[k].y)));
    vizCtx.closePath();
    vizCtx.fillStyle = hexToRgba(color, alpha);
    vizCtx.fill();
  };
  fillFace([0, 2, 6, 4], near ? 0.22 : 0.12);   // 바닥
  fillFace([1, 3, 7, 5], near ? 0.32 : 0.2);    // 윗면
  const edges = [[0,1],[0,2],[0,4],[3,1],[3,2],[3,7],[5,1],[5,4],[5,7],[6,2],[6,4],[6,7]];
  vizCtx.strokeStyle = color;
  vizCtx.lineWidth = near ? 2.4 : 1.6;
  vizCtx.beginPath();
  for (const [a, b] of edges) { vizCtx.moveTo(corners[a].x, corners[a].y); vizCtx.lineTo(corners[b].x, corners[b].y); }
  vizCtx.stroke();
  // 진행 방향: 윗면 중심 → 앞면 중심
  const top = P(0, 0, sz), front = P(sx, 0, sz);
  vizCtx.lineWidth = 2;
  vizCtx.beginPath(); vizCtx.moveTo(top.x, top.y); vizCtx.lineTo(front.x, front.y); vizCtx.stroke();

  let peak = corners[0];
  for (const cnr of corners) if (cnr.y < peak.y) peak = cnr;
  const tag = box.kind === 'track' && box.id != null ? `#${box.id}` : String(box.label || box.kind || '').slice(0, 14);
  labels.push({ x: peak.x, y: peak.y - 6, tag, dist, color, near });
  vizHits.push({ x: top.x, y: top.y, box, color });
  return 1;
}

function drawVizLabels(labels) {
  labels.sort((a, b) => (b.near - a.near) || a.dist - b.dist);
  const placed = [];
  vizCtx.font = '600 11px "Pretendard Variable", Pretendard, system-ui, sans-serif';
  for (const l of labels.slice(0, 40)) {
    const d = `${l.dist.toFixed(1)} m`;
    const wt = l.tag ? vizCtx.measureText(l.tag).width : 0;
    const wd = vizCtx.measureText(d).width;
    const w = wt + wd + (l.tag ? 22 : 14), h = 20;
    const x = Math.round(l.x - w / 2), y = Math.round(l.y - h);
    if (placed.some((r) => x < r[2] + 3 && x + w > r[0] - 3 && y < r[3] + 2 && y + h > r[1] - 2)) continue;
    placed.push([x, y, x + w, y + h]);
    vizCtx.fillStyle = 'rgba(8,12,18,0.85)';
    vizCtx.strokeStyle = hexToRgba(l.color, l.near ? 0.9 : 0.45);
    vizCtx.lineWidth = 1;
    vizCtx.beginPath();
    if (vizCtx.roundRect) vizCtx.roundRect(x, y, w, h, 6); else vizCtx.rect(x, y, w, h);
    vizCtx.fill(); vizCtx.stroke();
    let tx = x + 7;
    if (l.tag) { vizCtx.fillStyle = l.color; vizCtx.fillText(l.tag, tx, y + 14); tx += wt + 8; }
    vizCtx.fillStyle = l.near ? '#ffb4bb' : '#e8eef5';
    vizCtx.fillText(d, tx, y + 14);
  }
}

// 자차: 바닥 그림자 + 차체(불투명 직육면체, 면 깊이 정렬) + 진행 방향 화살표
// 위치·방향·크기는 vehicleGeom (P 의 x=차량 전방, y=차량 좌측 로컬 좌표)
function drawEgo3D(width, height) {
  const L = vehicleGeom.length, W = vehicleGeom.width, H = vehicleGeom.height;
  const h = vehHeading(), c = Math.cos(h), sn = Math.sin(h);
  const P = (x, y, z) => project3D(vehicleGeom.cx + c * x - sn * y, vehicleGeom.cy + sn * x + c * y, z, width, height);
  const v = [];
  for (const dx of [-L / 2, L / 2]) for (const dy of [-W / 2, W / 2]) for (const dz of [0, H]) v.push(P(dx, dy, dz));
  if (v.some((p) => !p.visible)) return;
  // 바닥 발광
  const g0 = P(0, 0, 0);
  const glow = vizCtx.createRadialGradient(g0.x, g0.y, 2, g0.x, g0.y, Math.max(20, g0.scale * 4.5));
  glow.addColorStop(0, 'rgba(56,189,248,0.35)'); glow.addColorStop(1, 'rgba(56,189,248,0)');
  vizCtx.fillStyle = glow;
  vizCtx.beginPath(); vizCtx.arc(g0.x, g0.y, Math.max(20, g0.scale * 4.5), 0, Math.PI * 2); vizCtx.fill();
  const faces = [
    { i: [0, 2, 6, 4], fill: '#0b131c' }, { i: [1, 3, 7, 5], fill: '#f0765a', top: true },
    { i: [4, 5, 7, 6], fill: '#1d2b3a' }, { i: [0, 1, 3, 2], fill: '#16222f', rear: true },
    { i: [0, 1, 5, 4], fill: '#1a2734' }, { i: [2, 3, 7, 6], fill: '#1a2734' },
  ];
  for (const f of faces) f.depth = f.i.reduce((a, k) => a + v[k].depth, 0) / 4;
  faces.sort((a, b) => b.depth - a.depth);
  for (const f of faces) {
    vizCtx.beginPath();
    f.i.forEach((k, n) => (n ? vizCtx.lineTo(v[k].x, v[k].y) : vizCtx.moveTo(v[k].x, v[k].y)));
    vizCtx.closePath();
    vizCtx.fillStyle = f.fill;
    vizCtx.fill();
    vizCtx.strokeStyle = 'rgba(210,225,240,0.85)';
    vizCtx.lineWidth = 1.2;
    vizCtx.stroke();
  }
  // 진행 방향 화살표 (차 앞 지면)
  const a = P(L / 2 + 0.6, 0, 0.05), b = P(L / 2 + 3.6, 0, 0.05), l = P(L / 2 + 2.5, 0.8, 0.05), r = P(L / 2 + 2.5, -0.8, 0.05);
  vizCtx.strokeStyle = '#38bdf8';
  vizCtx.lineWidth = 2.4;
  vizCtx.beginPath(); vizCtx.moveTo(a.x, a.y); vizCtx.lineTo(b.x, b.y); vizCtx.moveTo(l.x, l.y); vizCtx.lineTo(b.x, b.y); vizCtx.lineTo(r.x, r.y); vizCtx.stroke();
}

// 마우스를 올린 박스 정보
function updateVizTooltip() {
  const tip = document.getElementById('vizTip');
  if (!tip) return;
  if (!vizMouse || view.dragging) { tip.hidden = true; return; }
  let best = null, bd = 22;
  for (const h of vizHits) {
    const d = Math.hypot(h.x - vizMouse.x, h.y - vizMouse.y);
    if (d < bd) { bd = d; best = h; }
  }
  if (!best) { tip.hidden = true; vizCanvas.style.cursor = ''; return; }
  const b = best.box;
  const dist = vehEdgeDist(b.x || 0, b.y || 0);
  const kindName = { track: '차량 트랙', object: '객체', detection: '클러스터', radar: '레이더', marker: '마커' }[b.kind] || b.kind || '객체';
  tip.innerHTML = `<b style="color:${best.color}">${escapeHtml(b.kind === 'track' && b.id != null ? `#${b.id}` : String(b.label || kindName))}</b>
    <span>${escapeHtml(kindName)} · ${objPosLabel(b.x || 0, b.y || 0)}</span>
    <dl><dt>거리</dt><dd>${insideVehicle(b.x || 0, b.y || 0) ? '차체 안' : `${dist.toFixed(1)} m (차체 기준)`}</dd>
    <dt>위치</dt><dd>x ${(b.x || 0).toFixed(1)}, y ${(b.y || 0).toFixed(1)}</dd>
    <dt>크기</dt><dd>${(b.sx || 0).toFixed(1)} × ${(b.sy || 0).toFixed(1)} × ${(b.sz || 0).toFixed(1)} m</dd>
    <dt>방향</dt><dd>${((b.yaw || 0) * 180 / Math.PI).toFixed(0)}°</dd></dl>
    <em>${escapeHtml(b.topic || '')}</em>`;
  tip.hidden = false;
  const shell = vizCanvas.getBoundingClientRect();
  const tw = tip.offsetWidth, th = tip.offsetHeight;
  tip.style.left = `${Math.min(shell.width - tw - 8, best.x + 14)}px`;
  tip.style.top = `${Math.max(8, Math.min(shell.height - th - 8, best.y - th / 2))}px`;
  vizCanvas.style.cursor = 'pointer';
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
  updatePlayTrack();
  const rec = state.gpsTrack && (state.gpsTrack.recording || state.gpsTrack.bag_recording);
  if (!rec && gpsLiveLine && gpsMap) { gpsMap.removeLayer(gpsLiveLine); gpsLiveLine = null; gpsLivePts = []; }
  if (!rec || !state.gps || !gpsMap || view.mode === 'play') return;
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

// Play 중인 bag 의 GPS 경로 (보라). 백엔드가 gps_track.json 을 읽거나 bag 에서 추출할 때까지 2초 간격 재확인
let playTrackLine = null, playTrackFile = null, playTrackPending = false, playTrackNext = 0;
function clearPlayTrack() {
  if (playTrackLine && gpsMap) gpsMap.removeLayer(playTrackLine);
  playTrackLine = null; playTrackFile = null;
}
async function updatePlayTrack() {
  const bp = state.bagPlay || {};
  const want = view.mode === 'play' && bp.playing ? bp.file : null;
  if (!want) { if (playTrackFile) clearPlayTrack(); return; }
  if (!gpsMap || playTrackPending || (playTrackFile === want && playTrackLine) || performance.now() < playTrackNext) return;
  playTrackPending = true;
  try {
    const t = await (await fetch('/api/bag/play/track', { cache: 'no-store' })).json();
    if (t.file !== want) return;
    if (t.status === 'loading') { playTrackNext = performance.now() + 2000; return; }
    clearPlayTrack();
    playTrackFile = want;
    if (t.status === 'ready' && t.points && t.points.length) {
      playTrackLine = L.polyline(t.points.map((p) => [p[1], p[2]]), { color: '#c084fc', weight: 4, opacity: 0.85 }).addTo(gpsMap);
      playTrackLine.bindTooltip(`Bag 경로 · ${want} (${t.points.length}점)`);
      gpsMap.fitBounds(playTrackLine.getBounds(), { padding: [24, 24] });
    }
  } catch (_) {
    playTrackNext = performance.now() + 2000;
  } finally {
    playTrackPending = false;
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
  const [f, l] = toVeh(x, y);
  const ang = Math.atan2(l, f) * 180 / Math.PI;   // 차량 기준 0°=전방, +=좌측
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
  for (let i = objs.length - 1; i >= 0; i--) if (insideVehicle(objs[i].x, objs[i].y)) objs.splice(i, 1);
  objs.forEach((o) => { o.dist = vehEdgeDist(o.x, o.y); });
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
    if (insideVehicle(b.x || 0, b.y || 0)) continue;   // 차체 안 = 자차 클러스터
    const d = vehEdgeDist(b.x || 0, b.y || 0);
    if (!best || d < best.d) best = { d, x: b.x || 0, y: b.y || 0, label: b.label || '' };
  }
  if (!best || best.d > PROX_WARN_M) { el.hidden = true; return; }
  const danger = best.d <= PROX_DANGER_M;
  el.hidden = false;
  el.className = 'viz-alert ' + (danger ? 'viz-alert-danger' : 'viz-alert-warn');
  el.textContent = `⚠ ${objPosLabel(best.x, best.y)} ${best.d.toFixed(1)}m 장애물${danger ? ' — 위험' : ''}`;
}

// 센서 탭: 위쪽 전체 건강도(링) + 주요 센서 그룹 카드(링·상태·중단 토픽) + 기타 그룹 칩
function groupStatus(active, total) {
  if (!total) return { cls: 'na', text: '—' };
  const r = active / total;
  return r >= 0.9 ? { cls: 'ok', text: '정상' } : r > 0 ? { cls: 'warn', text: '일부 중단' } : { cls: 'bad', text: '중단' };
}
function renderGroups() {
  // 백엔드 groups는 실시간+재생 토픽을 합산하므로, 현재 뷰 모드 토픽만으로 클라이언트에서 재집계
  const agg = {};
  for (const t of currentModeTopics()) {
    const g = groupForTopic(t);
    const a = agg[g] || (agg[g] = { name: g, total: 0, active: 0, rate_hz: 0, down: [] });
    a.total += 1;
    if (t.active) a.active += 1; else a.down.push(t.name);
    a.rate_hz += t.rate_hz;
  }
  const names = Object.keys(agg);
  if (!names.length) { groupGrid.innerHTML = '<div class="empty">아직 ROS 토픽을 찾지 못했습니다.</div>'; return; }
  const main = GROUP_ORDER.filter((g) => agg[g]).map((g) => agg[g]);
  const others = names.filter((g) => !GROUP_META[g]).sort().map((g) => agg[g]);
  const mt = main.reduce((s, g) => s + g.total, 0), ma = main.reduce((s, g) => s + g.active, 0);
  const health = mt ? Math.round(ma / mt * 100) : 0;
  const hst = groupStatus(ma, mt);
  const totalHz = main.reduce((s, g) => s + g.rate_hz, 0);
  const warnGroups = main.filter((g) => groupStatus(g.active, g.total).cls !== 'ok');
  const shortName = (n) => n.split('/').filter(Boolean).slice(-2).join('/');

  const hero = `<section class="sg-hero st-${hst.cls}">
      <div class="sg-ring" style="--p:${health}"><div><b>${health}<small>%</small></b><span>센서 상태</span></div></div>
      <div class="sg-hero-body">
        <div class="sg-hero-title"><span class="sg-badge st-${hst.cls}">${hst.text}</span><span>주요 센서 ${main.length}개 그룹</span></div>
        <div class="sg-hero-stats">
          <div><b>${ma}</b><span>활성 토픽</span></div>
          <div><b>${mt - ma}</b><span>중단·대기</span></div>
          <div><b>${totalHz >= 1000 ? (totalHz / 1000).toFixed(1) + 'k' : totalHz.toFixed(0)}</b><span>Hz 합계</span></div>
        </div>
        <div class="sg-hero-warn">${warnGroups.length ? warnGroups.map((g) => `<span style="--gc:${groupMeta(g.name).color}">${groupMeta(g.name).label} ${g.active}/${g.total}</span>`).join('') : '<em>모든 주요 센서가 정상 수신 중</em>'}</div>
      </div>
    </section>`;

  const cards = main.map((g) => {
    const m = groupMeta(g.name);
    const pct = g.total ? Math.round(g.active / g.total * 100) : 0;
    const st = groupStatus(g.active, g.total);
    const down = g.down.slice(0, 2).map((n) => `<li title="${escapeHtml(n)}">${escapeHtml(shortName(n))}</li>`).join('')
      + (g.down.length > 2 ? `<li class="more">외 ${g.down.length - 2}개</li>` : '');
    return `<article class="group-card sg-card st-${st.cls}" data-group="${g.name}" style="--gc:${m.color}" title="클릭: 토픽 탭에서 이 그룹만 보기">
      <header><span class="sg-ico">${uiIcon(g.name, 16)}</span><b>${escapeHtml(m.label)}</b><span class="sg-badge st-${st.cls}">${st.text}</span></header>
      <div class="sg-main">
        <div class="sg-mini-ring" style="--p:${pct}"><span>${pct}%</span></div>
        <div class="sg-nums"><div><b>${g.active}</b><small>/ ${g.total} 활성</small></div><div class="sg-hz">${g.rate_hz.toFixed(1)} <small>Hz</small></div></div>
      </div>
      ${g.down.length ? `<ul class="sg-down">${down}</ul>` : '<div class="sg-allok">전체 수신 중</div>'}
    </article>`;
  }).join('');

  const otherHtml = others.length ? `<section class="sg-others">
      <div class="sg-sub">기타 토픽 그룹 <small>${others.length}</small></div>
      <div class="sg-chips">${others.map((g) => `<button type="button" class="sg-chip${g.active ? ' on' : ''}" data-group="other" title="${escapeHtml(g.name)} · ${g.rate_hz.toFixed(1)} Hz"><i></i>${escapeHtml(g.name.replace(/_/g, ' '))}<b>${g.active}/${g.total}</b></button>`).join('')}</div>
    </section>` : '';

  groupGrid.innerHTML = hero + `<div class="sg-grid">${cards}</div>` + otherHtml;
}
groupGrid.addEventListener('click', (e) => {
  const c = e.target.closest('[data-group]');
  if (!c) return;
  setTopicGroup(c.dataset.group);
  switchLeftTab('topics');
});


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
  ctx.lineTo(pts[pts.length - 1][0], H); ctx.lineTo(pts[0][0], H); ctx.closePath();
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

  const ring = (label, pct, lv, sub) => `<div class="sys-ring lv-${lv}" style="--p:${Math.min(100, pct).toFixed(1)}">
      <div class="ring-dial"><b>${pct.toFixed(0)}<small>%</small></b><span>${label}</span></div>
      <div class="ring-sub">${sub}</div></div>`;
  const trend = (label, id, pct, lv) => `<div class="trend-row lv-${lv}"><span class="trend-k">${label}</span><canvas id="${id}" class="trend-canvas" width="320" height="30"></canvas><b>${pct.toFixed(0)}%</b></div>`;
  return `<article class="sysm-card sys-hero">
      ${ring('CPU', cpuPct, usageColor(cpuPct), `${cpu.temp_c != null ? cpu.temp_c + '°C' : ''}${loadAvg.length ? ` · load ${loadAvg[0]}` : ''}`)}
      ${ring('GPU', gpuPct, usageColor(gpuPct), `${gpu.temp_c != null ? gpu.temp_c + '°C' : ''}${gpu.freq_mhz ? ` · ${gpu.freq_mhz} MHz` : ''}`)}
      ${ring('RAM', ramPct, usageColor(ramPct), `${ramUsedGB} / ${ramTotGB} GB`)}
    </article>
    <article class="sysm-card sys-trend"><div class="sysm-label">최근 추이 <small>${loadAvg.length ? 'load ' + loadAvg.join(' / ') : ''}</small></div>
      ${trend('CPU', 'sysChartCpu', cpuPct, usageColor(cpuPct))}${trend('GPU', 'sysChartGpu', gpuPct, usageColor(gpuPct))}${trend('RAM', 'sysChartRam', ramPct, usageColor(ramPct))}
    </article>
    <article class="sysm-card sysm-card-info">
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
// 인지 카메라(object_3d 등 인식 결과 영상) 여부
function isPerceptionCamera(name) {
  return /\/object_3d[^/]*\//.test(name) || name.includes('result_image') || name.includes('/perception/');
}
// 토픽이 속한 Camera Preview 탭: road(도로 인지) / perc(객체 인지) / cam(원본)
function cameraTabOf(name) {
  if (name.includes('/road_perception/')) return 'road';
  return isPerceptionCamera(name) ? 'perc' : 'cam';
}
function cameraOptions(tab) {
  let all = allCameraOptions();
  // 같은 영상의 raw/compressed 가 둘 다 있으면 compressed 만 보여줌 (activeCameraFor 가 raw 지정도 compressed 로 대체)
  const names = new Set(all.map((o) => o.name));
  all = all.filter((o) => !names.has(o.name + '/compressed'));
  return tab ? all.filter((o) => cameraTabOf(o.name) === tab) : all;
}
function allCameraOptions() {
  const byName = new Map();
  for (const camera of state.cameras) {
    if (isPlayTopic(camera.name)) continue;
    byName.set(camera.name, camera);
  }
  for (const topic of state.topics) {
    if (isPlayTopic(topic.name)) continue;
    if (topic.name.endsWith('/preview/compressed')) continue;   // 하드웨어 미리보기는 원본 슬롯에 자동 대체
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
  // raw 토픽이 없으면 같은 이름의 /compressed (예: object_3d 결과 영상)로 대체
  return state.cameras.find((c) => c.name === want) || state.cameras.find((c) => c.name === want + '/compressed') || null;
}

function saveCameraLayout() {
  localStorage.setItem('rosWebCameraLayout', JSON.stringify(view.cameraLayouts.cam));
  localStorage.setItem('rosWebPercCameraLayout', JSON.stringify(view.cameraLayouts.perc));
  localStorage.setItem('rosWebRoadCameraLayout', JSON.stringify(view.cameraLayouts.road));
}
function allCameraSlots() { return CAM_TABS.flatMap((t) => view.cameraLayouts[t].slots); }
function camEditing(tab) { return view.cameraEditMode && tab === view.camTab; }

function applyCamGrid(tab = view.camTab) {
  const layout = view.cameraLayouts[tab];
  const grid = CAM_GRIDS[tab];
  const cols = layout.cols || 2;
  const totalItems = layout.slots.length + (camEditing(tab) ? 1 : 0);
  const rows = Math.max(1, Math.ceil(totalItems / cols));
  grid.style.setProperty('--cam-cols', cols);
  grid.style.setProperty('--cam-rows', rows);
  if (tab === view.camTab) {
    document.querySelectorAll('.cam-col-btn').forEach((btn) => {
      btn.classList.toggle('active', Number(btn.dataset.cols) === cols);
    });
  }
}

// 탭 전환: 카메라 탭 그리드는 주행 뷰(2D/3D)가 영상을 재사용하므로 숨김 상태에서도 스트림 유지,
// 인지 카메라 탭은 보일 때만 스트림을 연다
function setCamTab(tab) {
  view.camTab = CAM_TABS.includes(tab) ? tab : 'cam';
  try { localStorage.setItem('rosWebCamTab', view.camTab); } catch (e) { /* ignore */ }
  document.querySelectorAll('#camTabs button').forEach((b) => b.classList.toggle('active', b.dataset.camtab === view.camTab));
  for (const t of CAM_TABS) CAM_GRIDS[t].classList.toggle('cam-grid-bg', t !== view.camTab);
  view.cameraOptionSig = '';
  renderCameras();
}

let camDragId = null;

function buildCameraCard(slot, options, tab) {
  const layout = view.cameraLayouts[tab];
  if (slot.topic && !options.some((o) => o.name === slot.topic)) {
    options = [{ name: slot.topic }, ...options];   // 아직 발행 안 된 토픽도 선택 상태 유지
  }
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
    refreshCameraStream(card, slot, tab);
  });

  // 삭제 버튼 (편집 모드에서만 표시)
  card.querySelector('.cam-remove-btn').addEventListener('click', () => {
    layout.slots = layout.slots.filter((s) => s.id !== slot.id);
    saveCameraLayout();
    rebuildCameraGrid(tab);
  });

  // 드래그 핸들 (편집 모드 순서 변경)
  const dragHandle = card.querySelector('.cam-drag-handle');
  dragHandle.addEventListener('pointerdown', (e) => {
    if (!camEditing(tab)) return;
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
        const slots = layout.slots;
        const fromIdx = slots.findIndex((s) => s.id === slot.id);
        const toIdx = slots.findIndex((s) => s.id === target.dataset.id);
        if (fromIdx >= 0 && toIdx >= 0) {
          const [moved] = slots.splice(fromIdx, 1);
          slots.splice(toIdx, 0, moved);
          saveCameraLayout();
          rebuildCameraGrid(tab);
        }
      }
      camDragId = null;
    };
    dragHandle.addEventListener('pointermove', onMove);
    dragHandle.addEventListener('pointerup', onUp);
  });

  refreshCameraStream(card, slot, tab);
  return card;
}

function refreshCameraStream(card, slot, tab) {
  const camera = activeCameraFor(slot);
  const img = card.querySelector('img');
  const title = card.querySelector('b');
  const age = card.querySelector('span');
  if (!camera || !slot.topic) {
    if (img.dataset.topic) stopCameraStream(img);
    title.textContent = slot.topic ? cameraLabel(slot.topic) : '—';
    age.textContent = slot.topic ? '수신 없음' : '';
    applyCameraSignalState(card, null, slot);
    return;
  }
  // 카메라 스트림은 보조 포트로 분리하고, 슬롯 순서대로 여러 포트에 나눠 접속
  // (브라우저의 호스트:포트당 동시연결 한도 6 → 7번째 슬롯부터 검은 화면 방지)
  const ports = state.streamPorts || [];
  // 인지 카메라 탭은 카메라 탭 슬롯 뒤 번호로 이어서 포트를 나눔 (두 탭이 같은 포트에 몰리지 않게)
  const slotIdx = Math.max(0, view.cameraLayouts[tab].slots.findIndex((sl) => sl.id === slot.id))
    + CAM_TABS.slice(0, CAM_TABS.indexOf(tab)).reduce((n, t) => n + view.cameraLayouts[t].slots.length, 0);
  const port = ports.length ? ports[slotIdx % ports.length] : null;
  const base = port ? `${location.protocol}//${location.hostname}:${port}` : '';
  const streamUrl = `${base}${camera.stream_url}${camera.stream_url.includes('?') ? '&' : '?'}id=${slot.id}${calRectParam(camera.name)}`;
  if (img.dataset.topic !== camera.name || img.dataset.baseurl !== streamUrl) {
    img.dataset.topic = camera.name;
    img.dataset.baseurl = streamUrl;
    startCameraStream(img, streamUrl);
  }
  title.textContent = cameraLabel(camera.name);
  age.textContent = camera.age_sec == null ? 'waiting' : `${camera.age_sec}s ago`;
  applyCameraSignalState(card, camera, slot);
}

// 카드 신호 상태 (CSS: .cam-empty / .cam-waiting / .cam-stale)
//  - 토픽 미지정 → empty
//  - 토픽 지정했지만 발행 안 됨, 또는 마지막 프레임이 오래됨 → stale('신호 없음')
const CAMERA_STALE_SEC = 3;
// 카드 제목: '/object_3d/Front_120_Camera/result_image/compressed' → 'Front_120_Camera'
const CAMERA_GENERIC_SEG = new Set(['object_3d', 'road_perception', 'bagplay', 'image_raw', 'compressed', 'result_image', 'preview', 'image', 'perception']);
function cameraLabel(name) {
  const segs = name.split('/').filter(Boolean);
  return segs.find((x) => !CAMERA_GENERIC_SEG.has(x)) || segs.slice(-1)[0] || name;
}
function applyCameraSignalState(card, camera, slot) {
  const empty = !slot.topic;
  const waiting = !!camera && camera.age_sec == null;
  // Play 는 녹화 공백(1~4초)이 그대로 재생되므로 끊김 판정을 늦춤.
  // 원래 느린 토픽(예: 인식 결과 영상 수 초 간격)은 평소 프레임 간격의 3배까지 기다림 (최대 90초)
  // 인지 결과 영상은 추론 부하에 따라 간격이 흔들리므로 기본 6초
  const base = (view.mode === 'play' ? 2 : 1) * (camera && isPerceptionCamera(camera.name) ? 6 : CAMERA_STALE_SEC);
  const staleSec = Math.max(base, Math.min(90, ((camera && camera.interval_sec) || 0) * 3));
  const stale = !empty && (!camera || (!waiting && camera.age_sec > staleSec));
  card.classList.toggle('cam-empty', empty);
  card.classList.toggle('cam-waiting', waiting);
  card.classList.toggle('cam-stale', stale);
}

// MJPEG 스트림 시작 + 실패(브라우저 연결한도/일시 끊김) 시 자동 재시도.
// 여러 카메라 동시 스트리밍 시 브라우저의 호스트당 동시연결 한도(~6)로
// 특정 카메라(예: /avm_cam/image_raw)가 안 뜨는 문제를 완화한다.
// MJPEG 연결을 즉시 끊음. <img> 를 DOM 에서 지우거나 src 속성만 제거하면 Chrome 이 연결을 한동안
// 붙잡고 있어 포트당 연결 한도(6)를 잡아먹으므로, 빈 이미지로 바꿔 요청을 중단시킨다.
const CAMERA_BLANK = 'data:image/gif;base64,R0lGODlhAQABAAAAACw=';
function stopCameraStream(img) {
  if (img._camRetry) { clearTimeout(img._camRetry); img._camRetry = null; }
  img.onerror = null;
  if (img.getAttribute('src')) img.src = CAMERA_BLANK;
  img.removeAttribute('src');
  img.dataset.topic = '';
  img.dataset.baseurl = '';
}

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

function rebuildCameraGrid(tab = view.camTab) {
  const grid = CAM_GRIDS[tab];
  const layout = view.cameraLayouts[tab];
  const options = cameraOptions(tab);
  grid.querySelectorAll('img').forEach(stopCameraStream);   // 기존 스트림 연결 정리 후 다시 만듦
  grid.innerHTML = '';
  applyCamGrid(tab);
  for (const slot of layout.slots) {
    grid.appendChild(buildCameraCard(slot, options, tab));
  }
  // 편집 모드: 빈 슬롯 추가 버튼 (그리드 마지막)
  if (camEditing(tab)) {
    const addCard = document.createElement('button');
    addCard.className = 'cam-add-card-btn';
    addCard.type = 'button';
    addCard.title = '카메라 추가';
    addCard.textContent = '＋';
    addCard.addEventListener('click', () => {
      layout.slots.push({ id: CAM_SLOT_PREFIX[tab] + Date.now(), topic: '' });
      saveCameraLayout();
      rebuildCameraGrid(tab);
    });
    grid.appendChild(addCard);
  }
  grid.classList.toggle('cam-edit', camEditing(tab));
  document.getElementById('cameraEditBar').hidden = !view.cameraEditMode;
}

const camTabSigs = {};
function renderCameras() {
  if (!view.cameraOptionSig) { CAM_TABS.forEach((t) => { camTabSigs[t] = ''; }); view.cameraOptionSig = 'set'; }
  for (const tab of CAM_TABS) {
    const grid = CAM_GRIDS[tab];
    const layout = view.cameraLayouts[tab];
    const active = tab === view.camTab;
    if (tab !== 'cam' && (!active || roadIpm.active)) {
      // 숨겨진 인지 탭(객체·도로)은 연결을 닫아 둠 (차선 범위 편집 중에는 보이는 탭도)
      if (camTabSigs[tab] !== 'off') { grid.querySelectorAll('img').forEach(stopCameraStream); grid.innerHTML = ''; camTabSigs[tab] = 'off'; }
      continue;
    }
    const options = cameraOptions(tab);
    // view.mode를 시그니처에 포함시켜 실시간⇄Play 전환 시 스트림이 다시 매칭되게 함
    const sig = options.map((o) => o.name).join('|') + '|' + layout.slots.length + '|' + camEditing(tab) + '|' + view.mode;
    if (sig !== camTabSigs[tab]) {
      camTabSigs[tab] = sig;
      rebuildCameraGrid(tab);
    } else {
      // 스트림 연결 상태·age 갱신. 나중에 나타난 카메라(예: Play 시작 직후의 /bagplay 토픽)는
      // 여기서 스트림을 시작한다 (이전에는 새로고침해야 연결됐음)
      for (const slot of layout.slots) {
        const card = grid.querySelector(`.camera-card[data-id="${CSS.escape(slot.id)}"]`);
        if (card) refreshCameraStream(card, slot, tab);
      }
    }
  }
  const counts = {};
  for (const tab of CAM_TABS) {
    const n = view.cameraLayouts[tab].slots.filter((sl) => activeCameraFor(sl)).length;
    counts[tab] = n;
    const badge = document.querySelector(`#camTabs button[data-camtab="${tab}"] em`);
    if (badge) badge.textContent = `${n}/${view.cameraLayouts[tab].slots.length}`;
  }
  cameraStatus.textContent = `${view.cameraLayout.slots.length}개 슬롯 · ${cameraOptions(view.camTab).length}개 토픽`;
}

// 토픽 탭 필터: 상태(전체/활성/대기/오류) · 그룹 칩. 그룹 select 는 숨기고 칩과 동기화
view.topicStatus = localStorage.getItem('rosWebTopicStatus') || 'all';
view.topicGroup = localStorage.getItem('rosWebTopicGroup') || 'all';
function topicMatchesGroup(topic, g) {
  if (g === 'all') return true;
  const tg = groupForTopic(topic);
  return g === 'other' ? !GROUP_META[tg] : tg === g;
}
function renderTopicFilters(all) {
  const sumEl = document.getElementById('topicSummary');
  const chipEl = document.getElementById('topicGroupChips');
  if (!sumEl || !chipEl) return;
  const cnt = { all: all.length, active: 0, idle: 0, error: 0 };
  for (const t of all) cnt[topicStatusOf(t)] += 1;
  const st = [['all', '전체'], ['active', '활성'], ['idle', '대기'], ['error', '오류']];
  const sumHtml = st.map(([k, l]) => `<button type="button" class="ts-chip ts-${k}${view.topicStatus === k ? ' on' : ''}" data-status="${k}">${k === 'all' ? '' : '<i></i>'}<span>${l}</span><b>${cnt[k]}</b></button>`).join('');
  const gcount = { all: all.length, other: 0 };
  for (const t of all) { const g = groupForTopic(t); if (GROUP_META[g]) gcount[g] = (gcount[g] || 0) + 1; else gcount.other += 1; }
  const groups = ['all', ...GROUP_ORDER.filter((g) => gcount[g]), ...(gcount.other ? ['other'] : [])];
  const chipHtml = groups.map((g) => {
    const m = g === 'all' ? { label: '모든 그룹', color: '#c9d6e2' } : g === 'other' ? { label: '기타', color: '#8996a6' } : groupMeta(g);
    return `<button type="button" class="tg-chip${view.topicGroup === g ? ' on' : ''}" data-group="${g}" style="--gc:${m.color}">${g === 'all' ? '' : uiIcon(g, 13)}<span>${m.label}</span><b>${gcount[g] || 0}</b></button>`;
  }).join('');
  if (sumEl.innerHTML !== sumHtml) sumEl.innerHTML = sumHtml;
  if (chipEl.innerHTML !== chipHtml) chipEl.innerHTML = chipHtml;
}
function hzPct(hz) { return Math.max(0, Math.min(100, Math.log10(1 + hz) / Math.log10(101) * 100)); }

function renderTopics() {
  const query = searchInput.value.trim().toLowerCase();
  const group = view.topicGroup;
  const all = currentModeTopics();
  renderTopicFilters(all);
  const rows = all.filter((topic) => {
    if (activeOnly.checked && !topic.active) return false;
    if (view.topicStatus !== 'all' && topicStatusOf(topic) !== view.topicStatus) return false;
    if (!topicMatchesGroup(topic, group)) return false;
    if (!query) return true;
    return topic.name.toLowerCase().includes(query) || topic.type.toLowerCase().includes(query);
  });
  shownCount.textContent = `${rows.length} / ${all.length} 표시`;
  if (!rows.length) { topicRows.innerHTML = '<tr><td class="empty" colspan="4">조건에 맞는 토픽이 없습니다.</td></tr>'; return; }
  const expanded = new Set([...topicRows.querySelectorAll('.topic-detail-row:not([hidden])')].map((r) => r.dataset.topic));
  topicRows.innerHTML = rows.map((topic) => {
    const status = topicStatusOf(topic);
    const age = topic.age_sec === null ? '-' : topic.age_sec >= 100 ? `${Math.round(topic.age_sec)}s` : `${topic.age_sec.toFixed(1)}s`;
    const ageLv = topic.age_sec === null ? 'na' : topic.age_sec < 1.5 ? 'ok' : topic.age_sec < 5 ? 'warn' : 'bad';
    const g = groupForTopic(topic);
    const gm = groupMeta(g);
    const cut = topic.name.lastIndexOf('/');
    const ns = cut > 0 ? topic.name.slice(0, cut + 1) : '/';
    const leaf = topic.name.slice(cut + 1);
    const pkg = topic.type.split('/')[0] || '';
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
    return `<tr class="topic-row st-${status}${isExpanded ? ' expanded' : ''}" data-topic="${escapeHtml(topic.name)}" style="--gc:${gm.color}">
      <td class="t-state" title="${status}"><i class="sdot ${status}"></i></td>
      <td class="topic" title="${escapeHtml(topic.name)}"><span class="t-name"><span class="t-ns">${escapeHtml(ns)}</span><b>${escapeHtml(leaf)}</b></span><span class="type-line"><em class="t-type">${escapeHtml(typeShort)}</em><span class="t-pkg">${escapeHtml(pkg)}</span></span></td>
      <td class="t-hz"><span>${topic.rate_hz.toFixed(1)}</span><i class="hzbar"><b style="width:${hzPct(topic.rate_hz).toFixed(0)}%"></b></i></td>
      <td class="t-age age-${ageLv}">${age}</td>
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
document.getElementById('topicSummary').addEventListener('click', (e) => {
  const b = e.target.closest('[data-status]');
  if (!b) return;
  view.topicStatus = b.dataset.status;
  try { localStorage.setItem('rosWebTopicStatus', view.topicStatus); } catch (err) { /* ignore */ }
  renderTopics();
});
document.getElementById('topicGroupChips').addEventListener('click', (e) => {
  const b = e.target.closest('[data-group]');
  if (!b) return;
  setTopicGroup(b.dataset.group);
});
function setTopicGroup(g) {
  view.topicGroup = g;
  try { localStorage.setItem('rosWebTopicGroup', g); } catch (err) { /* ignore */ }
  if ([...groupFilter.options].some((o) => o.value === g)) groupFilter.value = g;
  renderTopics();
}
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
  const y0 = -vehHeading();   // 차량 전방이 화면 위쪽이 되는 yaw
  const presets = {
    top: { yaw: y0, pitch: Math.PI / 2, zoom: dz * 1.05 },
    front: { yaw: y0, pitch: 0.02, zoom: dz },
    left: { yaw: y0 + Math.PI / 2, pitch: 0.02, zoom: dz },
    right: { yaw: y0 - Math.PI / 2, pitch: 0.02, zoom: dz },
    reset: { yaw: y0, pitch: 1.08, zoom: dz },
  };
  // 현재 시점 → 목표 시점으로 0.35초 동안 부드럽게 이동 (yaw 는 짧은 쪽으로 회전)
  const to = { ...(presets[name] || presets.reset), panX: 0, panY: 0 };
  const from = { yaw: view.yaw, pitch: view.pitch, zoom: view.zoom, panX: view.panX, panY: view.panY };
  let dyaw = ((to.yaw - from.yaw) % (Math.PI * 2) + Math.PI * 3) % (Math.PI * 2) - Math.PI;
  const t0 = performance.now(), dur = 350;
  const step = (now) => {
    const t = Math.min(1, (now - t0) / dur), e = 1 - Math.pow(1 - t, 3);
    view.yaw = from.yaw + dyaw * e;
    view.pitch = from.pitch + (to.pitch - from.pitch) * e;
    view.panX = from.panX + (to.panX - from.panX) * e;
    view.panY = from.panY + (to.panY - from.panY) * e;
    setZoom(from.zoom + (to.zoom - from.zoom) * e);
    if (t < 1) requestAnimationFrame(step);
  };
  requestAnimationFrame(step);
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

// 탭 배지: 토픽 활성/전체 · 센서 건강도 · CPU 사용률 (매 갱신, 가벼움)
function updateLeftTabBadges() {
  const nav = document.getElementById('leftTabs');
  if (!nav) return;
  const topics = currentModeTopics();
  const active = topics.filter((t) => t.active).length;
  const errors = topics.filter((t) => topicStatusOf(t) === 'error').length;
  const set = (id, text, lv) => {
    const el = nav.querySelector(`[data-badge="${id}"]`);
    if (!el) return;
    if (el.textContent !== text) el.textContent = text;
    el.className = 'lt-badge' + (lv ? ' lv-' + lv : '');
  };
  set('topics', topics.length ? `${active}/${topics.length}` : '', errors ? 'high' : '');
  const main = topics.filter((t) => GROUP_META[groupForTopic(t)]);
  const health = main.length ? Math.round(main.filter((t) => t.active).length / main.length * 100) : null;
  set('groups', health == null ? '' : `${health}%`, health == null ? '' : health >= 90 ? 'low' : health >= 60 ? 'mid' : 'high');
  const cpu = state.system && state.system.cpu ? state.system.cpu.usage_percent : null;
  set('system', cpu == null ? '' : `CPU ${Math.round(cpu)}%`, cpu == null ? '' : usageColor(cpu));
}

function renderLeftTabBar() {
  const nav = document.getElementById('leftTabs');
  if (!nav) return;
  nav.innerHTML = view.leftTabConfig.map((t) => `
    <button class="left-tab${t.id === view.leftTab ? ' active' : ''}" data-tab="${t.id}" type="button">
      <span class="left-tab-drag" data-tab="${t.id}">⠿</span>
      ${uiIcon(t.id, 15)}<span class="left-tab-label">${t.label}</span><em class="lt-badge" data-badge="${t.id}"></em>
    </button>`).join('');
  updateLeftTabBadges();
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
groupFilter.addEventListener('change', () => setTopicGroup(groupFilter.value));
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
  view.cameraLayout.slots.push({ id: CAM_SLOT_PREFIX[view.camTab] + Date.now(), topic: '' });
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
vizCanvas.addEventListener('pointermove', (event) => {
  const r = vizCanvas.getBoundingClientRect();
  vizMouse = { x: event.clientX - r.left, y: event.clientY - r.top };
  if (!view.dragging) updateVizTooltip();
});
vizCanvas.addEventListener('pointerleave', () => { vizMouse = null; updateVizTooltip(); });
document.querySelectorAll('#vizColorMode [data-cmode]').forEach((b) => {
  b.classList.toggle('active', b.dataset.cmode === view.vizColorMode);
  b.addEventListener('click', () => {
    view.vizColorMode = b.dataset.cmode;
    localStorage.setItem('rosWebVizColorMode', view.vizColorMode);
    document.querySelectorAll('#vizColorMode [data-cmode]').forEach((x) => x.classList.toggle('active', x === b));
    renderVisualization();
  });
});
const vizLabelBtn = document.getElementById('vizLabelToggle');
vizLabelBtn.classList.toggle('active', view.vizLabels);
vizLabelBtn.addEventListener('click', () => {
  view.vizLabels = !view.vizLabels;
  localStorage.setItem('rosWebVizLabels', view.vizLabels ? 'on' : 'off');
  vizLabelBtn.classList.toggle('active', view.vizLabels);
  renderVisualization();
});
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
    document.querySelectorAll('.camera-card img').forEach(stopCameraStream);
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
  range: 80, range2d: 20, laneWidth: 3.5, steerRatio: 14.5, wheelbase: 2.7,
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
  if (saved.range2d === 30) saved.range2d = 20;   // 이전 기본값(30 m) → 새 기본값
  const cameras = (Array.isArray(saved.cameras) ? saved.cameras : []).map((t) => t.replace(/^\/bagplay(?=\/)/, ''));
  return { ...DRIVE_DEFAULTS, ...saved, cameras };
})();
function saveDriveSettings() { localStorage.setItem('rosWebDriveSettings', JSON.stringify(driveSettings)); }

// ?vtab=drive3d|radar2d 로 탭을 바로 열 수 있음 (링크 공유용)
view.vizTab = new URLSearchParams(location.search).get('vtab') || localStorage.getItem('rosWebVizTab') || 'rviz';
const driveCam = (() => {
  const preset = localStorage.getItem('rosWebDrivePreset') || 'chase';
  return { preset, ...(DRIVE_PRESETS[preset] || DRIVE_PRESETS.chase), dragging: false, lastX: 0, lastY: 0 };
})();
let driveRange2d = driveSettings.range2d;
// 2D 뷰 이동량(m): 화면 중심을 전방 panF, 좌측 panL 만큼 옮김. radar2dMap = 마지막 렌더의 좌표 변환값
const view2d = { panF: 0, panL: 0, dragging: false, lastX: 0, lastY: 0 };
let radar2dMap = null;
function resetView2d() { driveRange2d = driveSettings.range2d; view2d.panF = 0; view2d.panL = 0; }
// 2D 후방 표시 거리: 전방의 20%, 단 자차 뒤끝 + 2 m 이상
function radar2dRear(R) { return Math.max(R * 0.2, vehicleGeom.length / 2 + 2); }
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
// GPS 진행방향(track, 북 기준 시계방향) 변화율 → 요레이트(deg/s, +=좌회전). 새 GPS 샘플마다 갱신
const gpsMotion = { key: '', track: null, t: 0, rate: 0 };
function gpsMotionUpdate(g) {
  const key = `${g.latitude},${g.longitude},${g.track}`;
  if (key === gpsMotion.key) return;
  const now = performance.now() / 1000;
  if (gpsMotion.track != null && now - gpsMotion.t > 0.2 && now - gpsMotion.t < 5) {
    const d = ((g.track - gpsMotion.track + 540) % 360) - 180;
    gpsMotion.rate = gpsMotion.rate * 0.5 + (-d / (now - gpsMotion.t)) * 0.5;
  }
  Object.assign(gpsMotion, { key, track: g.track, t: now });
}

function driveVehicleState() {
  const v = view.mode === 'play' ? state.vehiclePlay : state.vehicle;
  if (!v) {
    // CAN(/acu_ck2_ver2) 이 없으면 GPSFix 속도·진행방향으로 대신 (bag 재생 시 차량 움직임 표시용)
    const g = view.mode === 'play' ? state.gpsPlay : state.gps;
    if (!g || typeof g.speed !== 'number' || g.age_sec > 3) return null;
    if (typeof g.track === 'number') gpsMotionUpdate(g);
    const mps = Math.max(0, g.speed);
    const yawRate = mps > 1 ? gpsMotion.rate : 0;
    return { source: 'gps', speed: mps * 3.6, steer: null, yawRate, curvature: mps > 1 ? (yawRate * Math.PI / 180) / mps : 0,
      brake: 0, torque: 0, gear: '—' };
  }
  const n = (x) => (typeof x === 'number' ? x : 0);
  const wheels = [v.wheel_fl, v.wheel_fr, v.wheel_rl, v.wheel_rr].filter((x) => typeof x === 'number');
  const speed = wheels.length ? wheels.reduce((a, b) => a + b, 0) / wheels.length : 0;
  const steer = n(v.steer_angle);   // 차량 규약: + = 좌
  const roadWheel = steer / Math.max(1, driveSettings.steerRatio) * Math.PI / 180;
  const curvature = Math.tan(roadWheel) / Math.max(0.5, driveSettings.wheelbase);
  return { source: 'can', speed, steer, curvature, brake: n(v.brake), torque: n(v.motor_tq), gear: VEH_GEAR[Math.round(n(v.gear))] ?? '—' };
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
    if (insideVehicle(b.x || 0, b.y || 0)) continue;   // 차체 안 = 자차 클러스터
    const [f, l] = toVeh(b.x || 0, b.y || 0);
    const o = { ...b, x: f, y: l, z: b.z || 0, yaw: (b.yaw || 0) - vehHeading(), dist: vehEdgeDist(b.x || 0, b.y || 0) };
    if (b.kind === 'track') tracks.push(o);
    else if (b.kind !== 'radar') clusters.push(o);
  }
  // 트랙과 겹치는 클러스터(트랙의 원본)는 중복 표시하지 않음
  const clustersOnly = clusters.filter((c) => !tracks.some((t) => Math.hypot(t.x - c.x, t.y - c.y) < 1.6));
  const radar = [];
  const lanes = [];
  for (const m of viz.markers || []) {
    if (isPlayTopic(m.topic) !== inPlay) continue;
    if (/lane/i.test(m.topic) && [4, 5].includes(m.type)) {
      lanes.push({ ...m, points: (m.points || []).map((p) => [...toVeh(p[0], p[1]), p[2] || 0]) });
      continue;
    }
    if (m.kind !== 'radar' || [4, 5].includes(m.type)) continue;
    for (const p of m.points || []) {
      const [f, l] = toVeh(p[0] || 0, p[1] || 0);
      radar.push({ x: f, y: l, z: p[2] || 0, dist: vehEdgeDist(p[0] || 0, p[1] || 0) });
    }
  }
  const lidarTopic = effectiveTopic((view.vizLayers.find((l) => l.id === 'l1') || {}).topic || '');
  const lidar = [];
  const hasLidarTopic = (viz.point_layers || []).some((l) => l.topic === lidarTopic);
  for (const layer of viz.point_layers || []) {
    if (isPlayTopic(layer.topic) !== inPlay) continue;
    // LiDAR 레이어 토픽이 없으면 병합 좌표계로 변환된 개별 라이다로 대체
    const use = lidarTopic && hasLidarTopic ? layer.topic === lidarTopic : (lidarTopic ? layer.merge_part : !/radar/i.test(layer.topic));
    if (!use) continue;
    lidar.push(ds.lidar ? (layer.points || []).map((p) => [...toVeh(p[0], p[1]), p[2] || 0]) : []);
  }
  // 선행 차량: 예상 경로 차로 안에서 가장 가까운 전방 객체 (트랙 우선, 없으면 클러스터)
  const front = vehicleGeom.length / 2;   // 차량 중심 → 앞 범퍼
  const inLane = (o) => o.x > front + 0.3 && o.x < front + ds.range && Math.abs(pathLateral(k, o.x, o.y)) < ds.laneWidth / 2 + 0.4;
  const pick = (arr) => arr.filter(inLane).sort((a, b) => a.x - b.x)[0] || null;
  const lead = (ds.tracks && pick(tracks)) || (ds.clusters && pick(clustersOnly)) || null;
  const leadRadar = lead ? radar.find((r) => Math.hypot(r.x - lead.x, r.y - lead.y) < 2.8) || null : null;
  // 앞 범퍼 ~ 선행 객체 뒷면 간격 (객체의 전방축 방향 반폭 = |cos|·sx/2 + |sin|·sy/2)
  if (lead) lead.gap = lead.x - front - (Math.abs(Math.cos(lead.yaw || 0)) * (lead.sx || 0) + Math.abs(Math.sin(lead.yaw || 0)) * (lead.sy || 0)) / 2;
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
  const W = Math.max(1.4, Math.min(3.2, Math.min(o.sx || 4.4, o.sy || 1.8)));
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
function liveCameraImgs(sel = '.camera-grid img') {
  const out = new Map();
  for (const im of document.querySelectorAll(sel)) {
    if (im.dataset.topic && im.naturalWidth > 0 && !out.has(im.dataset.topic)) out.set(im.dataset.topic, im);
  }
  return out;
}
function driveCameraTopics() {
  const ds = driveSettings;
  const live = liveCameraImgs();
  if (ds.camMode === 'off') return [];
  if (ds.camMode === 'all') return [...live.keys()];
  if (ds.camMode === 'select') return ds.cameras.map((t) => effectiveTopic(logicalTopic(t)));   // Play 에서는 /bagplay/... 로
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
  const parts = topic.split('/').filter(Boolean);
  const name = (isPlayTopic(topic) ? parts[1] : parts[0]) || topic;   // /bagplay/<카메라>/... 는 카메라 이름 표시
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
  const odo = driveOdo % 20;
  for (let x = Math.ceil(gx0 / step) * step; x <= gx1 + 20; x += step) {
    const xs = x - odo;   // 주행거리만큼 뒤로 이동
    if (xs < gx0 || xs > gx1) continue;
    driveCtx.strokeStyle = x % 20 === 0 ? 'rgba(120,170,220,0.16)' : 'rgba(120,170,220,0.07)';
    strokePoly3(cam, [[xs, -gy, 0], [xs, gy, 0]]);
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
    // 바깥 차선: 지면 좌표 기준 점선(3 m 칠, 6 m 빔) — 주행거리만큼 흘러감
    driveCtx.strokeStyle = 'rgba(170,190,210,0.5)'; driveCtx.lineWidth = 1.6;
    for (let s0 = -(driveOdo % 9); s0 < range; s0 += 9) {
      for (const d of [lw * 1.5, -lw * 1.5]) {
        const a = pathPoint(k, Math.max(0, s0), d), b = pathPoint(k, Math.max(0, s0 + 3), d);
        if (s0 + 3 > 0) strokePoly3(cam, [[a[0], a[1], 0.02], [b[0], b[1], 0.02]]);
      }
    }
    // 거리 눈금
    driveCtx.font = '500 12px "Pretendard Variable", Pretendard, system-ui, sans-serif';
    for (let d = 20; d <= range; d += 20) {
      const p = pathPoint(k, vehicleGeom.length / 2 + d, -lw * 1.5 - 1.2);   // 앞 범퍼 기준 거리
      const sp = project(cam, [p[0], p[1], 0]);
      if (!sp || sp.y < 0 || sp.y > height) continue;
      driveCtx.fillStyle = 'rgba(160,180,200,0.75)';
      driveCtx.fillText(`${d} m`, sp.x + 4, sp.y);
      driveCtx.fillRect(sp.x - 8, sp.y - 4, 8, 1);
    }
    // 선행 차량까지 경로 빔
    const bumper = vehicleGeom.length / 2 + 0.1;
    const leadS = sc.lead ? Math.max(bumper + 0.5, sc.lead.x - 2.5) : bumper + 12;
    const beamL = along(0.45, bumper, leadS), beamR = along(-0.45, bumper, leadS).reverse();
    if (fillPoly3(cam, [...beamL, ...beamR])) {
      const n0 = project(cam, [bumper, 0, 0]), n1 = project(cam, pathPoint(k, leadS).concat(0));
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
  items.push({ depth: depthOf(0, 0), draw: () => drawCar(cam, { x: 0, y: 0, sx: vehicleGeom.length, sy: vehicleGeom.width, sz: vehicleGeom.height, yaw: 0 }, DRIVE_COLORS.ego,
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
    if (o === sc.lead) drawLabel(s.x, s.y, [{ text: 'LEAD', color: DRIVE_COLORS.lead }, { text: `${Math.max(0, o.gap).toFixed(1)} m`, color: '#e8eef5' }], 'rgba(61,220,151,0.6)', placed);
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
  const Rr = radar2dRear(R);       // 후방 표시 거리
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
    driveCtx.strokeStyle = 'rgba(200,215,230,0.55)'; driveCtx.lineWidth = 1.4;
    lane(lw / 2); lane(-lw / 2);
    // 바깥 차선: 지면 좌표 점선(3 m / 9 m 주기) — 주행거리만큼 흘러 움직임 표시
    driveCtx.strokeStyle = 'rgba(170,185,200,0.45)';
    const s0 = Math.min(-Rr * 0.3, panF - Rr), s1 = Math.max(R, panF + R);
    for (let s = s0 - ((driveOdo + s0) % 9 + 9) % 9; s < s1; s += 9) {
      for (const d of [lw * 1.5, -lw * 1.5]) {
        const a = toS(...pathPoint(k, s, d)), b = toS(...pathPoint(k, s + 3, d));
        driveCtx.beginPath(); driveCtx.moveTo(a[0], a[1]); driveCtx.lineTo(b[0], b[1]); driveCtx.stroke();
      }
    }
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
    if (lead) text(`LEAD ${Math.max(0, c.gap).toFixed(1)} m`, sx + 10, sy - 6, DRIVE_COLORS.lead);
  }
  for (const t of sc.tracks) {
    const lead = t === sc.lead;
    const [sx, sy] = rect(t, lead ? DRIVE_COLORS.lead : '#7ee0b0', lead ? 'rgba(61,220,151,0.22)' : 'rgba(126,224,176,0.10)');
    text(lead ? `LEAD #${t.id ?? '?'} ${Math.max(0, t.gap).toFixed(1)} m` : `#${t.id ?? '?'}`, sx + 10, sy - 6, lead ? DRIVE_COLORS.lead : DRIVE_COLORS.track);
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
  const ew = Math.max(12, vehicleGeom.width * scale), eh = Math.max(22, vehicleGeom.length * scale);
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
  else if (driveSettings.path) badge.textContent = !v ? '예상 경로 · 차량 데이터 없음' : v.source === 'gps' ? '예상 주행 경로 · GPS 방향 기반' : '예상 주행 경로 · 조향 기반';
  else badge.textContent = '';
  badge.hidden = !badge.textContent;
  const nObj = sc.tracks.length + sc.clusters.length;
  const grid = view.vizTab === 'radar2d' && radar2dMap ? ` · 눈금 ${radar2dMap.step < 1 ? radar2dMap.step.toFixed(1) : radar2dMap.step} m` : '';
  $('#driveCount').textContent = `주변 객체 ${nObj}개 · 레이더 ${sc.radar.length}${grid}`;
  vizStats.textContent = `트랙 ${sc.tracks.length} · 클러스터 ${sc.clusters.length} · 레이더 ${sc.radar.length}${sc.lidar.length ? ` · LiDAR ${sc.lidar.reduce((a, p) => a + p.length, 0)}` : ''}`;
  $('#dsSpeed').innerHTML = !v ? '—' : `${Math.abs(v.speed).toFixed(1)} <small>km/h</small> <i class="ds-gear">${escapeHtml(v.source === 'gps' ? 'GPS' : v.gear)}</i>`;
  // 차량 규약 +=좌. 부호 대신 좌/우로 표기 (Vehicle Monitor 게이지는 우=+ 로 반전 표시)
  $('#dsSteer').textContent = !v ? '—'
    : v.source === 'gps' ? (Math.abs(v.yawRate) < 0.5 ? '직진 (GPS)' : `${v.yawRate > 0 ? '좌' : '우'}회전 ${Math.abs(v.yawRate).toFixed(1)}°/s`)
    : Math.abs(v.steer) < 0.5 ? '직진 0.0°' : `${v.steer > 0 ? '좌' : '우'} ${Math.abs(v.steer).toFixed(1)}°`;
  const pedal = !v ? '—' : v.source === 'gps' ? '— (CAN 없음)' : v.brake > 2 ? '브레이크 밟음' : v.torque > 5 ? '가속 중' : '페달 없음';
  const pedalEl = $('#dsPedal');
  pedalEl.textContent = pedal;
  pedalEl.className = v && v.brake > 2 ? 'ds-brake' : v && v.torque > 5 ? 'ds-accel' : '';
  const leadEl = $('#dsLead');
  leadEl.textContent = sc.lead ? `${Math.max(0, sc.lead.gap).toFixed(1)} m${sc.lead.kind === 'track' ? ` · #${sc.lead.id ?? '?'}` : ''}` : '없음';
  leadEl.className = sc.lead && sc.lead.gap < PROX_WARN_M ? 'ds-brake' : sc.lead ? 'ds-lead' : '';
}

// 누적 주행거리(m): 지면 격자·차선 점선을 차량 속도만큼 뒤로 흘려 움직임을 표시
let driveOdo = 0, driveOdoT = 0;
function renderDriveView() {
  const sc = driveScene();
  const now = performance.now() / 1000;
  if (sc.veh && driveOdoT) driveOdo += (sc.veh.speed / 3.6) * Math.min(0.5, now - driveOdoT);
  driveOdoT = now;
  if (view.vizTab === 'radar2d') renderRadar2D(sc); else renderDrive3D(sc);
  updateDriveHud(sc);
}

// ── 설정 (통합 설정 창 → RViz 탭 '주행 뷰' 섹션) ──
let driveEdit = null;
function cameraTopicChoices() {
  const set = new Set([...currentModeCameras().map((c) => logicalTopic(c.name)), ...allCameraSlots().map((sl) => logicalTopic(sl.topic)).filter(Boolean)]);
  if (driveEdit) driveEdit.cameras.forEach((t) => set.add(logicalTopic(t)));
  return [...set].sort();
}
function renderDriveCamList() {
  const el = $('#dsetCamList');
  if (!el || !driveEdit) return;
  const inGrid = new Set(view.cameraLayouts.cam.slots.map((sl) => logicalTopic(sl.topic)).filter(Boolean));
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
  document.querySelectorAll('#setPanelRviz [data-vgeom]').forEach((el) => { el.value = String(vehicleGeom[el.dataset.vgeom]); });
  driveEdit = { ...driveSettings, cameras: driveSettings.cameras.slice() };
  document.querySelectorAll('#setPanelRviz [data-dset]').forEach((el) => {
    const v = driveEdit[el.dataset.dset];
    if (el.type === 'checkbox') el.checked = !!v; else el.value = String(v);
  });
  $('#dsetRangeVal').textContent = `${driveEdit.range} m`;
  renderDriveCamList();
}
function commitDriveSettingsEdit(save) {
  if (save) {
    const before = JSON.stringify(vehicleGeom);
    document.querySelectorAll('#setPanelRviz [data-vgeom]').forEach((el) => {
      const key = el.dataset.vgeom;
      if (key === 'forward') { vehicleGeom.forward = el.value; return; }
      const n = Number(el.value);
      if (Number.isFinite(n) && (n > 0 || key === 'cx' || key === 'cy')) vehicleGeom[key] = n;
    });
    if (JSON.stringify(vehicleGeom) !== before) {
      localStorage.setItem('rosWebVehicleGeom', JSON.stringify(vehicleGeom));
      setViewPreset('reset');   // 전방 방향이 바뀌었을 수 있으므로 시점 재정렬
    }
  }
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
      if (driveEdit.camMode === 'select' && !driveEdit.cameras.length) driveEdit.cameras = [...liveCameraImgs().keys()].map(logicalTopic);
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
    const scale = m.plotH / (driveRange2d + radar2dRear(driveRange2d));
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
// 확대 창 좌우 이동은 지금 보이는 탭의 카메라 안에서만
function camZoomTopics() { return [...liveCameraImgs(`#${CAM_GRIDS[view.camTab].id} img`).keys()]; }
// 확대 창은 원본 해상도 스트림(?full=1)을 따로 하나 열어서 그림 — 미리보기 그리드는 960px 축소본이라서.
// 원본이 뜨기 전까지는 그리드 영상을 그대로 사용
let camZoomFull = null;
function setCamZoomFull(topic) {
  if (camZoomFull) { stopCameraStream(camZoomFull); camZoomFull.remove(); camZoomFull = null; }
  const cam = topic && state.cameras.find((c) => c.name === topic);
  if (!cam) return;
  const ports = state.streamPorts || [];
  const port = ports.length ? ports[ports.length - 1] : null;
  const base = port ? `${location.protocol}//${location.hostname}:${port}` : '';
  const img = new Image();
  img.dataset.topic = topic;
  img.dataset.baseurl = `${base}${cam.stream_url}&full=1&id=zoom${calRectParam(topic)}`;
  // 화면에 붙어 있어야 MJPEG 프레임이 계속 갱신됨 (1px, 보이지 않게)
  img.style.cssText = 'position:absolute;width:1px;height:1px;opacity:0;pointer-events:none;left:0;top:0';
  camZoomEl.appendChild(img);
  startCameraStream(img, img.dataset.baseurl);
  camZoomFull = img;
}
function openCamZoom(topic) {
  if (!topic) return;
  camZoomTopic = topic;
  setCamZoomFull(topic);
  camZoomEl.hidden = false;
  if (!camZoomRaf) camZoomRaf = requestAnimationFrame(drawCamZoom);
}
function closeCamZoom() {
  camZoomEl.hidden = true;
  camZoomTopic = null;
  setCamZoomFull(null);
  if (camZoomRaf) cancelAnimationFrame(camZoomRaf);
  camZoomRaf = 0;
}
function stepCamZoom(dir) {
  const list = camZoomTopics();
  if (!list.length) return;
  const i = list.indexOf(camZoomTopic);
  camZoomTopic = list[(i + dir + list.length) % list.length];
  setCamZoomFull(camZoomTopic);
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
  const full = camZoomFull && camZoomFull.dataset.topic === camZoomTopic && camZoomFull.naturalWidth > 0 ? camZoomFull : null;
  const img = full || liveCameraImgs().get(camZoomTopic);
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
for (const grid of Object.values(CAM_GRIDS)) {
  grid.addEventListener('click', (e) => {
    if (view.cameraEditMode) return;
    const img = e.target.closest('.camera-card img');
    if (img && img.dataset.topic) openCamZoom(img.dataset.topic);
  });
}
document.querySelectorAll('#camTabs button').forEach((b) => b.addEventListener('click', () => setCamTab(b.dataset.camtab)));
setCamTab(view.camTab);

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
