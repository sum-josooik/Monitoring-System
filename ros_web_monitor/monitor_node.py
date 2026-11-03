import glob
import io
import json
import math
import mimetypes
import os
import re
import shutil
import signal
import subprocess
import tempfile
import threading
import time
from collections import defaultdict, deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import sys as _sys
for _jtop_venv in [
    '/home/nvidia/.local/share/jtop/lib/python3.12/site-packages',
    '/root/.local/share/jtop/lib/python3.12/site-packages',
]:
    if _jtop_venv not in _sys.path:
        _sys.path.insert(0, _jtop_venv)
try:
    from jtop import jtop as JTop
    JTOP_AVAILABLE = True
except Exception:
    JTOP_AVAILABLE = False
    JTop = None

import cv2
import numpy as np
import yaml
from PIL import Image as PilImage

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
try:
    from sensor_msgs_py import point_cloud2
except Exception:
    point_cloud2 = None
try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None
try:
    # ros2 bag play가 띄우는 '/rosbag2_player/...' 서비스로 일시정지/재개/이동(seek)/배속을 제어.
    from rosbag2_interfaces.srv import Pause as BagPause
    from rosbag2_interfaces.srv import PlayNext as BagPlayNext
    from rosbag2_interfaces.srv import Resume as BagResume
    from rosbag2_interfaces.srv import Seek as BagSeek
    from rosbag2_interfaces.srv import SetRate as BagSetRate
    BAG_CTRL_AVAILABLE = True
except Exception:
    BAG_CTRL_AVAILABLE = False


def static_dir():
    if get_package_share_directory:
        try:
            share_dir = Path(get_package_share_directory('ros_web_monitor')) / 'web'
            if share_dir.exists():
                return share_dir
        except Exception:
            pass
    return Path(__file__).resolve().parent / 'web'


STATIC_DIR = static_dir()


def ui_settings_dir():
    # 설치본(share/ros_web_monitor/UI) 우선, 없으면 소스 트리의 UI/
    for d in (STATIC_DIR.parent / 'UI', Path(__file__).resolve().parent.parent / 'UI'):
        if d.is_dir():
            return d
    return None


def default_ui_settings_file():
    # UI/ 폴더의 설정 백업 JSON 중 가장 최신(파일명 타임스탬프 기준)
    d = ui_settings_dir()
    if d is None:
        return None
    files = sorted(d.glob('*.json'))
    return files[-1] if files else None

# ── ROS1 bag → ROS2 bag 변환용 격리 venv ──
# 시스템 파이썬(externally-managed-environment)을 건드리지 않도록 사용자 홈 아래
# 전용 venv에 'rosbags' 패키지를 최초 1회만 설치해서 사용한다 (jtop venv와 동일한 패턴).
ROSBAGS_VENV = Path.home() / '.local' / 'share' / 'ros_web_monitor' / 'rosbags_venv'


def ensure_rosbags_convert():
    """rosbags-convert 실행 파일 경로를 반환. 없으면 격리 venv에 설치를 시도한다."""
    exe = ROSBAGS_VENV / 'bin' / 'rosbags-convert'
    if exe.exists():
        return str(exe)
    try:
        ROSBAGS_VENV.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run([_sys.executable, '-m', 'venv', str(ROSBAGS_VENV)],
                        check=True, timeout=60, capture_output=True)
        pip = ROSBAGS_VENV / 'bin' / 'pip'
        subprocess.run([str(pip), 'install', '-q', 'rosbags'],
                        check=True, timeout=180, capture_output=True)
    except Exception:
        return None
    return str(exe) if exe.exists() else None


def _now():
    return time.monotonic()


def json_safe(obj):
    """NaN/Infinity를 None으로 치환 — 표준 JSON 호환 보장."""
    if isinstance(obj, float):
        return None if (math.isnan(obj) or math.isinf(obj)) else obj
    if isinstance(obj, dict):
        return {k: json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [json_safe(v) for v in obj]
    return obj


def extract_msg_values(msg, type_name=''):
    """ROS 메시지에서 표시용 주요 필드 추출."""
    vals = {}
    try:
        # GPS / NavSatFix / GPSFix
        if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
            vals['lat'] = round(float(msg.latitude), 7)
            vals['lng'] = round(float(msg.longitude), 7)
            vals['alt'] = round(float(msg.altitude), 2)
            if hasattr(msg, 'status') and hasattr(msg.status, 'status'):
                vals['gps_status'] = int(msg.status.status)

        # IMU
        elif hasattr(msg, 'linear_acceleration') and hasattr(msg, 'angular_velocity'):
            la, av = msg.linear_acceleration, msg.angular_velocity
            vals['accel_xyz'] = f'{la.x:.2f}, {la.y:.2f}, {la.z:.2f}'
            vals['gyro_xyz'] = f'{av.x:.3f}, {av.y:.3f}, {av.z:.3f}'
            if hasattr(msg, 'orientation'):
                o = msg.orientation
                vals['quat_xyzw'] = f'{o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f}'

        # Odometry
        elif (hasattr(msg, 'pose') and hasattr(msg, 'twist')
              and hasattr(getattr(msg.pose, 'pose', None), 'position')):
            p = msg.pose.pose.position
            v = msg.twist.twist.linear
            vals['pos_xyz'] = f'{p.x:.3f}, {p.y:.3f}, {p.z:.3f}'
            vals['vel_xyz'] = f'{v.x:.3f}, {v.y:.3f}, {v.z:.3f}'

        # TwistStamped
        elif hasattr(msg, 'twist') and hasattr(getattr(msg, 'twist', None), 'linear'):
            lv, av = msg.twist.linear, msg.twist.angular
            vals['linear_xyz'] = f'{lv.x:.3f}, {lv.y:.3f}, {lv.z:.3f}'
            vals['angular_xyz'] = f'{av.x:.3f}, {av.y:.3f}, {av.z:.3f}'

        # Twist (no stamp)
        elif (hasattr(msg, 'linear') and hasattr(msg, 'angular')
              and hasattr(getattr(msg, 'linear', None), 'x')):
            vals['linear_xyz'] = f'{msg.linear.x:.3f}, {msg.linear.y:.3f}, {msg.linear.z:.3f}'
            vals['angular_xyz'] = f'{msg.angular.x:.3f}, {msg.angular.y:.3f}, {msg.angular.z:.3f}'

        # PoseStamped
        elif (hasattr(msg, 'pose') and hasattr(getattr(msg, 'pose', None), 'position')
              and not hasattr(msg.pose, 'pose')):
            p = msg.pose.position
            vals['pos_xyz'] = f'{p.x:.3f}, {p.y:.3f}, {p.z:.3f}'

        # CameraInfo
        elif (hasattr(msg, 'width') and hasattr(msg, 'height')
              and hasattr(msg, 'distortion_model') and hasattr(msg, 'k')):
            vals['width'] = int(msg.width)
            vals['height'] = int(msg.height)
            vals['model'] = str(msg.distortion_model)
            k = list(msg.k)
            if len(k) >= 5:
                vals['fx'] = round(float(k[0]), 1)
                vals['fy'] = round(float(k[4]), 1)

        # CompressedImage
        elif hasattr(msg, 'format') and hasattr(msg, 'data') and isinstance(msg.data, (bytes, bytearray, list)):
            vals['format'] = str(msg.format)
            vals['size_kb'] = round(len(bytes(msg.data)) / 1024, 1)

        # Image (raw)
        elif (hasattr(msg, 'width') and hasattr(msg, 'height')
              and hasattr(msg, 'encoding') and hasattr(msg, 'step')):
            vals['width'] = int(msg.width)
            vals['height'] = int(msg.height)
            vals['encoding'] = str(msg.encoding)
            vals['step'] = int(msg.step)

        # PointCloud2
        elif hasattr(msg, 'width') and hasattr(msg, 'height') and hasattr(msg, 'fields'):
            vals['points'] = msg.width * msg.height
            if hasattr(msg, 'header'):
                vals['frame'] = str(msg.header.frame_id)
            vals['fields'] = ', '.join(f.name for f in msg.fields[:6])

        # BatteryState
        elif hasattr(msg, 'voltage') and hasattr(msg, 'percentage'):
            vals['voltage_V'] = round(float(msg.voltage), 2)
            pct = float(msg.percentage)
            vals['charge_pct'] = f'{pct * 100:.1f}%' if pct <= 1.0 else f'{pct:.1f}%'
            if hasattr(msg, 'current'):
                vals['current_A'] = round(float(msg.current), 3)

        # Float64MultiArray / Float32MultiArray
        elif hasattr(msg, 'data') and hasattr(msg, 'layout'):
            arr = list(msg.data)[:6]
            vals['data'] = '[' + ', '.join(f'{v:.3f}' for v in arr) + (', …' if len(list(msg.data)) > 6 else '') + ']'
            vals['length'] = len(list(msg.data))

        # Simple scalar (Float32, Float64, Int32, Bool, String)
        elif hasattr(msg, 'data') and not hasattr(msg, 'header'):
            d = msg.data
            if isinstance(d, bool):
                vals['data'] = str(d)
            elif isinstance(d, (int, float)):
                vals['data'] = round(float(d), 5)
            elif isinstance(d, str):
                vals['data'] = d[:120]
            elif isinstance(d, (bytes, bytearray)):
                vals['data'] = f'{len(d)} bytes'

        # DiagnosticArray
        elif hasattr(msg, 'status') and hasattr(msg.status, '__iter__'):
            statuses = list(msg.status)[:4]
            vals['count'] = len(statuses)
            lvl_map = {0: 'OK', 1: 'WARN', 2: 'ERR', 3: 'STALE'}
            for i, s in enumerate(statuses[:3]):
                vals[f'status_{i}'] = f'{s.name}: {lvl_map.get(s.level, str(s.level))}'

        # Generic fallback — works for any custom message type
        else:
            # ROS2 Jazzy: __slots__ uses '_field' prefix; actual access uses 'field'
            try:
                field_names = list(type(msg).get_fields_and_field_types().keys())
            except Exception:
                slots = getattr(msg, '__slots__', [])
                field_names = [f.lstrip('_') for f in slots]
            skip = {'header', 'stamp'}
            for fname in field_names:
                if fname in skip:
                    continue
                val = getattr(msg, fname, None)
                if isinstance(val, bool):
                    vals[fname] = str(val)
                elif isinstance(val, (int, float)):
                    fv = float(val)
                    if math.isnan(fv) or math.isinf(fv):
                        vals[fname] = None  # NaN/Inf는 JSON 호환되도록 null
                    else:
                        vals[fname] = round(fv, 4)
                elif isinstance(val, str) and val:
                    vals[fname] = val[:80]
                elif isinstance(val, (list, tuple, bytes, bytearray)):
                    arr = list(val) if not isinstance(val, (bytes, bytearray)) else list(val)
                    nums = [x for x in arr[:6] if isinstance(x, (int, float))]
                    if nums:
                        vals[fname] = '[' + ', '.join(
                            ('nan' if (isinstance(v, float) and (math.isnan(v) or math.isinf(v))) else f'{v:.3f}')
                            for v in nums
                        ) + (', …' if len(arr) > 6 else '') + ']'
    except Exception:
        pass
    return vals


class TopicStats:
    def __init__(self, name, type_name):
        self.name = name
        self.type_name = type_name
        self.first_seen = _now()
        self.last_seen = None
        self.total = 0
        self.timestamps = deque(maxlen=2000)
        self.last_error = ''
        self.last_value = {}

    def record(self):
        stamp = _now()
        self.last_seen = stamp
        self.total += 1
        self.timestamps.append(stamp)

    def snapshot(self, rate_window_sec):
        now = _now()
        while self.timestamps and now - self.timestamps[0] > rate_window_sec:
            self.timestamps.popleft()
        rate = 0.0
        if len(self.timestamps) > 1:
            elapsed = max(self.timestamps[-1] - self.timestamps[0], 0.001)
            rate = (len(self.timestamps) - 1) / elapsed
        age = None if self.last_seen is None else now - self.last_seen
        return {
            'name': self.name,
            'type': self.type_name,
            'total': self.total,
            'rate_hz': round(rate, 2),
            'age_sec': None if age is None else round(age, 2),
            'active': age is not None and age < max(rate_window_sec * 2.0, 3.0),
            'error': self.last_error,
            'first_seen_ago': round(now - self.first_seen, 0),
            'last_value': dict(self.last_value),
        }


class JtopMonitor:
    """jtop을 백그라운드 스레드로 구동하며 최신 Jetson 시스템 스냅샷을 제공."""

    def __init__(self):
        self._snapshot = {}
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name='jtop-monitor')
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                with JTop(interval=0.5) as jetson:
                    while self._running and jetson.ok():
                        snap = self._extract(jetson)
                        with self._lock:
                            self._snapshot = snap
                        time.sleep(0.5)
            except Exception:
                time.sleep(2.0)

    def _extract(self, jetson):
        stats = jetson.stats

        # ── CPU ──────────────────────────────────────────────────────────────
        cpu_keys = sorted(k for k in stats if re.match(r'^CPU\d+$', k))
        cores = [int(stats[k]) for k in cpu_keys]
        # per-core 상세 (governor / freq / usage)
        cores_detail = []
        try:
            for i, c in enumerate(jetson.cpu.get('cpu', [])):
                if not c.get('online', True):
                    cores_detail.append({'idx': i, 'online': False, 'usage': 0,
                                         'freq_mhz': None, 'gov': 'offline'})
                    continue
                usage = round(100.0 - float(c.get('idle', 0)), 1)
                fcur = c.get('freq', {}).get('cur', None)
                cores_detail.append({
                    'idx': i, 'online': True, 'usage': usage,
                    'freq_mhz': round(fcur / 1000) if fcur else None,
                    'gov': c.get('governor', ''),
                })
        except Exception:
            cores_detail = []
        # 상세가 있으면 그것을 기준으로 cores/total 재계산
        if cores_detail:
            cores = [c['usage'] for c in cores_detail]
        cpu_total = round(sum(cores) / len(cores), 1) if cores else 0.0
        load = os.getloadavg() if hasattr(os, 'getloadavg') else (0.0, 0.0, 0.0)
        try:
            cpu_temp = jetson.temperature.get('cpu', {}).get('temp', None)
        except Exception:
            cpu_temp = None

        # ── GPU ──────────────────────────────────────────────────────────────
        try:
            gpu_cluster = next(iter(jetson.gpu.values()), {})
            gpu_usage = float(gpu_cluster.get('status', {}).get('load', 0) or 0)
            gpu_freq_khz = gpu_cluster.get('freq', {}).get('cur', None)
            gpu_freq_mhz = round(gpu_freq_khz / 1000) if gpu_freq_khz else None
        except Exception:
            gpu_usage = float(stats.get('GPU', 0) or 0)
            gpu_freq_mhz = None
        try:
            gpu_temp = jetson.temperature.get('gpu', {}).get('temp', None)
        except Exception:
            gpu_temp = None

        # ── RAM / SWAP ───────────────────────────────────────────────────────
        try:
            ram_data = jetson.memory.get('RAM', {})
            ram_tot  = ram_data.get('tot',  0)
            ram_used = ram_data.get('used', 0)
            ram_tot_mb  = round(ram_tot  / 1024)   # kB → MB
            ram_used_mb = round(ram_used / 1024)
            ram_pct = round(ram_used / ram_tot * 100, 1) if ram_tot else 0.0
            swap_data    = jetson.memory.get('SWAP', {})
            swap_tot_mb  = round(swap_data.get('tot',  0) / 1024)
            swap_used_mb = round(swap_data.get('used', 0) / 1024)
        except Exception:
            mem = read_memory_snapshot()
            ram_tot_mb, ram_used_mb, ram_pct = mem['total_mb'], mem['used_mb'], mem['usage_percent']
            swap_tot_mb = swap_used_mb = 0

        # ── Power ────────────────────────────────────────────────────────────
        try:
            power_tot    = jetson.power.get('tot', {})
            power_cur_mw = power_tot.get('power', None)
            power_avg_mw = power_tot.get('avg',   None)
            gpu_power_mw = jetson.power.get('rail', {}).get('VDD_GPU', {}).get('power', None)
        except Exception:
            power_cur_mw = power_avg_mw = gpu_power_mw = None

        # ── Thermal ──────────────────────────────────────────────────────────
        try:
            thermal = []
            for name, tdata in jetson.temperature.items():
                if isinstance(tdata, dict):
                    val = tdata.get('temp', None)
                    if val is not None and -50 < float(val) < 200:
                        thermal.append({'name': str(name), 'c': round(float(val), 1)})
            thermal.sort(key=lambda t: t['c'], reverse=True)
        except Exception:
            thermal = read_thermal_snapshot()

        # ── Uptime ───────────────────────────────────────────────────────────
        try:
            uptime = stats.get('uptime', None)
            uptime_sec = int(uptime.total_seconds()) if uptime else read_uptime()
        except Exception:
            uptime_sec = read_uptime()

        # ── Board info ───────────────────────────────────────────────────────
        board_info = {}
        try:
            hw    = jetson.board.get('hardware', {})
            model = hw.get('Module', '') or hw.get('Model', '')
            jp_ver = hw.get('Jetpack', '')
            l4t    = hw.get('L4T', '')
            jetpack_str = (f'JetPack {jp_ver}' if jp_ver else '') + (f' [L4T {l4t}]' if l4t else '')
            board_info = {
                'platform':  {str(k): str(v) for k, v in jetson.board.get('platform', {}).items()},
                'hardware':  {str(k): str(v) for k, v in hw.items()},
                'libraries': {str(k): str(v) for k, v in jetson.board.get('libraries', {}).items()},
            }
        except Exception:
            model = read_first_existing(['/proc/device-tree/model'], strip_null=True)
            jetpack_str = ''

        # ── Engines (NVENC/NVDEC/NVJPG/VIC/SE/PVA/APE …) ── ENG 탭 ─────────────
        engines = []
        try:
            for group, items in (jetson.engine or {}).items():
                if not isinstance(items, dict):
                    continue
                for ename, edata in items.items():
                    if not isinstance(edata, dict):
                        continue
                    fcur = edata.get('cur', None)
                    engines.append({
                        'group':   str(group),
                        'name':    str(ename),
                        'online':  bool(edata.get('online', False)),
                        'freq_mhz': round(fcur / 1000) if fcur else None,
                    })
        except Exception:
            engines = []

        # ── Power rails ── ENG 탭 ─────────────────────────────────────────────
        power_rails = []
        try:
            for rname, rdata in jetson.power.get('rail', {}).items():
                if not isinstance(rdata, dict):
                    continue
                power_rails.append({
                    'name':     str(rname),
                    'power_mw': rdata.get('power', None),
                    'avg_mw':   rdata.get('avg', None),
                    'volt_mv':  rdata.get('volt', None),
                    'curr_ma':  rdata.get('curr', None),
                })
        except Exception:
            power_rails = []

        # ── Fan / NVP model / jetson_clocks ── CTRL 탭 ────────────────────────
        fan = {}
        try:
            for fname, fdata in (jetson.fan or {}).items():
                spd = fdata.get('speed', [])
                rpm = fdata.get('rpm', [])
                fan = {
                    'name':      str(fname),
                    'speed_pct': round(float(spd[0]), 1) if spd else 0.0,
                    'rpm':       int(rpm[0]) if rpm else None,
                    'profile':   str(fdata.get('profile', '')),
                }
                break
        except Exception:
            fan = {}
        try:
            nvp_model = str(jetson.nvpmodel)
        except Exception:
            nvp_model = ''
        try:
            jc_raw = str(jetson.jetson_clocks)
            jetson_clocks = jc_raw in ('True', 'true', 'on', 'ON')
        except Exception:
            jetson_clocks = None

        # ── Disk ── MEM 탭 ────────────────────────────────────────────────────
        disk = {}
        try:
            d = jetson.disk or {}
            unit = d.get('unit', 'G')
            disk = {
                'total':     round(float(d.get('total', 0)), 1),
                'used':      round(float(d.get('used', 0)), 1),
                'available': round(float(d.get('available', 0)), 1),
                'unit':      str(unit),
            }
        except Exception:
            disk = {}

        return {
            'model':   model,
            'jetpack': jetpack_str.strip() or read_jetpack_version(),
            'cpu': {
                'usage_percent': cpu_total,
                'cores':   cores,
                'cores_detail': cores_detail,
                'loadavg': [round(v, 2) for v in load],
                'temp_c':  round(float(cpu_temp), 1) if cpu_temp is not None else None,
            },
            'memory': {
                'total_mb':     ram_tot_mb,
                'used_mb':      ram_used_mb,
                'available_mb': ram_tot_mb - ram_used_mb,
                'usage_percent': ram_pct,
                'swap_used_mb':  swap_used_mb,
                'swap_total_mb': swap_tot_mb,
                'disk':          disk,
            },
            'gpu': {
                'usage_percent': round(gpu_usage, 1),
                'temp_c':        round(float(gpu_temp), 1) if gpu_temp is not None else None,
                'freq_mhz':      gpu_freq_mhz,
                'power_mw':      gpu_power_mw,
                'power_tot_mw':  power_cur_mw,
                'power_avg_mw':  power_avg_mw,
            },
            'engines':     engines,
            'power_rails': power_rails,
            'control': {
                'fan':           fan,
                'nvpmodel':      nvp_model,
                'jetson_clocks': jetson_clocks,
            },
            'board':      board_info,
            'thermal':    thermal[:12],
            'uptime_sec': uptime_sec,
        }

    def snapshot(self):
        with self._lock:
            return dict(self._snapshot)

    def stop(self):
        self._running = False


class MonitorNode(Node):
    def __init__(self):
        super().__init__('ros_web_monitor')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('stream_port', 8081)
        self.declare_parameter('scan_period_sec', 2.0)
        self.declare_parameter('rate_window_sec', 5.0)
        self.declare_parameter('subscribe_all', True)
        self.declare_parameter('visual_max_points_per_topic', 900)
        self.declare_parameter('prune_after_sec', 10.0)
        self.declare_parameter('bag_dir', '/home/nvidia/robot_ws/bag')

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.stream_port = int(self.get_parameter('stream_port').value)
        self.scan_period_sec = float(self.get_parameter('scan_period_sec').value)
        self.rate_window_sec = float(self.get_parameter('rate_window_sec').value)
        self.subscribe_all = bool(self.get_parameter('subscribe_all').value)
        self.visual_max_points_per_topic = int(self.get_parameter('visual_max_points_per_topic').value)
        self.prune_after_sec = float(self.get_parameter('prune_after_sec').value)

        self.lock = threading.RLock()
        self.stats = {}
        self.subscriptions_by_topic = {}
        self.camera_frames = {}
        self.camera_conditions = defaultdict(threading.Condition)
        self.topic_missing_since = {}
        self.visual_points = {}
        self.visual_boxes = {}
        self.visual_markers = {}
        self.visual_last_processed = {}
        self.system_cache = {}
        self.system_cache_stamp = 0.0
        # 스냅샷 JSON 캐시: 다중 접속(SSE 여러 클라이언트 + /api/state) 시
        # snapshot()+json.dumps 를 클라이언트 수만큼 반복하지 않도록 0.25s 공유
        self._snap_json = None
        self._snap_json_stamp = 0.0
        self._snap_json_lock = threading.Lock()
        self.gps_data = {}
        # ── GPS 이동경로 기록 ──
        self.gps_track_dir = Path('/home/nvidia/captin5_ws/src/ros_web_monitor/log/gps')
        self.gps_track_recording = False
        self.gps_track_points = []
        self.gps_track_started = None
        self.visual_range_m = 120.0
        self.jtop_monitor = JtopMonitor() if JTOP_AVAILABLE else None
        self.server = None
        self.server_thread = None

        # ── ROS2 bag 녹화 상태 ──
        # 초기값은 launch 파라미터(bag_dir)에서 가져오고, 실행 중엔 설정 화면에서 변경 가능(set_bag_dir)
        self.bag_dir = os.path.expanduser(str(self.get_parameter('bag_dir').value))
        self.bag_lock = threading.Lock()
        self.bag_process = None
        self.bag_tmp = None
        self.bag_start_time = None
        self.bag_last_saved = None
        self.bag_topics = []   # 빈 리스트 = 전체(-a)
        self._bag_log = None

        # ── ROS2 bag 재생(Play) 상태 ──
        # 재생된 토픽은 항상 play_namespace 하위로 리맵되어 실시간 토픽과 절대 섞이지 않는다.
        # (예: /acu_ck2_ver2 → /bagplay/acu_ck2_ver2) 프런트는 실시간뷰/Play뷰 탭으로
        # 이 네임스페이스 유무만 보고 같은 뷰어(RViz/카메라/토픽표)를 재사용해서 그린다.
        self.play_namespace = '/bagplay'
        self.play_lock = threading.Lock()
        self.play_process = None
        self.play_starting = False
        self.play_file = None
        self.play_start_time = None
        self.play_tmp_dir = None   # ROS1 bag 변환 시 생긴 임시 디렉토리(정지 시 정리)
        self._play_log = None
        self.gps_data_play = {}    # Play 네임스페이스 GPS (실시간 gps_data와 분리)
        # ── bag 재생 제어(일시정지/이동/배속): ros2 bag play가 노출하는
        # /rosbag2_player/{pause,resume,seek,set_rate,play_next} 서비스를 호출한다.
        # 재생 위치를 직접 조회하는 서비스는 없으므로, pause/resume/seek/rate 시점마다
        # 기준점(bag_ns, wall, rate)을 갱신해 두고 그 사이를 시간 경과로 추정한다.
        self.play_loop = False
        self.play_start_ns = None       # bag 시작 시각(unix epoch, ns) — metadata.yaml 기준
        self.play_duration_ns = None    # bag 전체 길이(ns)
        self.play_pos_ref = {}          # {'bag_ns','wall','rate','paused'}
        self.play_ctrl_clients = {}     # 서비스 이름 → rclpy Client (재생 세션당 재사용)

        self.scan_timer = self.create_timer(self.scan_period_sec, self.scan_topics)
        self.http_timer = self.create_timer(1.0, self.log_http_once)
        self._logged_http = False
        self.start_http_server()
        self.scan_topics()

    def start_http_server(self):
        handler = make_handler(self)
        # 카메라 MJPEG 스트림 + SSE 가 연결을 오래 점유하므로 accept 대기열을 키워
        # 종료/재부팅 등 새 요청이 간헐적으로 거부되지 않게 한다 (기본 5 → 128).
        ThreadingHTTPServer.request_queue_size = 128
        self.server = ThreadingHTTPServer((self.host, self.port), handler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        # 카메라 MJPEG 전용 보조 서버: 브라우저의 호스트:포트당 동시연결 한도(~6) 때문에
        # SSE + 카메라 5개가 메인 포트를 다 차지하면 API fetch가 실패하므로 포트를 분리한다.
        self.stream_server = None
        if self.stream_port and self.stream_port != self.port:
            try:
                self.stream_server = ThreadingHTTPServer((self.host, self.stream_port), handler)
                threading.Thread(target=self.stream_server.serve_forever, daemon=True).start()
            except OSError as exc:
                self.get_logger().warn(f'스트림 포트 {self.stream_port} 사용 불가 ({exc}) — 메인 포트로 스트리밍')
                self.stream_server = None

    def log_http_once(self):
        if not self._logged_http:
            self.get_logger().info(f'ROS web monitor listening on http://{self.host}:{self.port}')
            self._logged_http = True

    def destroy_node(self):
        # 녹화 중이면 안전하게 종료/저장
        if getattr(self, 'bag_process', None) is not None and self.bag_process.poll() is None:
            try:
                self.stop_bag()
            except Exception:
                pass
        # 재생 중이면 프로세스 정리(임시 변환 디렉토리 포함)
        if getattr(self, 'play_process', None) is not None and self.play_process.poll() is None:
            try:
                self.stop_play()
            except Exception:
                pass
        if getattr(self, 'jtop_monitor', None):
            self.jtop_monitor.stop()
        if self.server:
            self.server.shutdown()
            self.server.server_close()
        if getattr(self, 'stream_server', None):
            self.stream_server.shutdown()
            self.stream_server.server_close()
        super().destroy_node()

    def scan_topics(self):
        topics = self.get_topic_names_and_types()
        seen = set()
        for name, type_names in topics:
            if not type_names:
                continue
            type_name = type_names[0]
            seen.add(name)
            with self.lock:
                self.stats.setdefault(name, TopicStats(name, type_name))
                self.stats[name].type_name = type_name
            if self.subscribe_all and name not in self.subscriptions_by_topic:
                self.subscribe_topic(name, type_name)

        now = _now()
        with self.lock:
            for name in list(self.stats.keys()):
                # 우리가 구독 중이면 발행자가 죽어도 그래프에 토픽이 남으므로
                # 그래프 존재 여부가 아니라 발행자 수로 생존을 판단한다
                alive = name in seen and self.count_publishers(name) > 0
                if alive:
                    self.topic_missing_since.pop(name, None)
                    continue
                self.stats[name].last_error = 'no active publishers'
                since = self.topic_missing_since.setdefault(name, now)
                # prune_after_sec 이상 발행자가 없는 토픽은 목록에서 제거
                if now - since >= self.prune_after_sec:
                    self.remove_topic(name)

    def remove_topic(self, name):
        """그래프에서 사라진 토픽의 구독/상태/프레임을 정리한다. self.lock 보유 상태에서 호출."""
        self.stats.pop(name, None)
        self.topic_missing_since.pop(name, None)
        sub = self.subscriptions_by_topic.pop(name, None)
        if sub is not None:
            try:
                self.destroy_subscription(sub)
            except Exception:
                pass
        self.camera_frames.pop(name, None)
        self.visual_points.pop(name, None)
        self.visual_boxes.pop(name, None)
        self.visual_markers.pop(name, None)
        self.visual_last_processed.pop(name, None)
        # 스트림 대기 중인 클라이언트가 있으면 깨워서 종료 판단하게 함
        condition = self.camera_conditions.pop(name, None)
        if condition is not None:
            with condition:
                condition.notify_all()

    def subscribe_topic(self, name, type_name):
        try:
            msg_type = get_message(type_name)
            qos = QoSProfile(
                depth=5,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            if type_name == 'sensor_msgs/msg/CompressedImage':
                callback = lambda msg, topic=name: self.handle_compressed_image(topic, msg)
            elif type_name == 'sensor_msgs/msg/Image' and is_streamable_raw_image_topic(name):
                callback = lambda msg, topic=name: self.handle_raw_image(topic, msg)
            elif type_name == 'sensor_msgs/msg/PointCloud2':
                callback = lambda msg, topic=name: self.handle_pointcloud(topic, msg)
            elif type_name == 'visualization_msgs/msg/MarkerArray':
                callback = lambda msg, topic=name: self.handle_marker_array(topic, msg)
            elif type_name == 'visualization_msgs/msg/Marker':
                callback = lambda msg, topic=name: self.handle_marker(topic, msg)
            elif type_name == 'vision_msgs/msg/Detection3DArray':
                callback = lambda msg, topic=name: self.handle_detection_array(topic, msg)
            elif type_name in ('sensor_msgs/msg/NavSatFix', 'gps_msgs/msg/GPSFix') or \
                    'NavSatFix' in type_name or 'GPSFix' in type_name:
                callback = lambda msg, topic=name: self.handle_gps(topic, msg)
            else:
                callback = lambda msg, topic=name, tn=type_name: self.handle_generic(topic, msg, tn)
            sub = self.create_subscription(msg_type, name, callback, qos)
            self.subscriptions_by_topic[name] = sub
            with self.lock:
                self.stats[name].last_error = ''
        except Exception as exc:
            with self.lock:
                self.stats.setdefault(name, TopicStats(name, type_name)).last_error = str(exc)
            self.get_logger().warn(f'Could not subscribe to {name} ({type_name}): {exc}')

    def handle_message(self, topic):
        with self.lock:
            stat = self.stats.get(topic)
            if stat:
                stat.record()

    def handle_generic(self, topic, msg, type_name=''):
        """모든 타입의 fallback 핸들러 — 타이밍 기록 + 값 추출."""
        with self.lock:
            stat = self.stats.get(topic)
            if stat:
                stat.record()
                vals = extract_msg_values(msg, type_name or stat.type_name)
                if vals:
                    stat.last_value = vals

    def handle_compressed_image(self, topic, msg):
        self.handle_message(topic)
        frame = bytes(msg.data)
        with self.lock:
            stat = self.stats.get(topic)
            if stat:
                stat.last_value = {
                    'format': str(msg.format),
                    'size_kb': round(len(frame) / 1024, 1),
                }
            self.camera_frames[topic] = {
                'stamp': _now(),
                'format': msg.format or 'jpeg',
                'data': frame,
            }
        condition = self.camera_conditions[topic]
        with condition:
            condition.notify_all()

    def handle_raw_image(self, topic, msg):
        self.handle_message(topic)
        with self.lock:
            stat = self.stats.get(topic)
            if stat and not stat.last_value:
                stat.last_value = {
                    'width': int(msg.width),
                    'height': int(msg.height),
                    'encoding': str(msg.encoding),
                }
        if not self.should_process_visual(topic, min_interval=0.08):
            return
        try:
            frame = encode_raw_image_as_jpeg(msg)
        except Exception as exc:
            with self.lock:
                stat = self.stats.get(topic)
                if stat:
                    stat.last_error = f'image encode: {exc}'
            return
        with self.lock:
            self.camera_frames[topic] = {
                'stamp': _now(),
                'format': 'jpeg',
                'data': frame,
            }
        condition = self.camera_conditions[topic]
        with condition:
            condition.notify_all()

    def should_process_visual(self, topic, min_interval=0.18):
        now = _now()
        last = self.visual_last_processed.get(topic, 0.0)
        if now - last < min_interval:
            return False
        self.visual_last_processed[topic] = now
        return True

    def handle_pointcloud(self, topic, msg):
        with self.lock:
            stat = self.stats.get(topic)
            if stat:
                stat.record()
                stat.last_value = {
                    'points': msg.width * msg.height,
                    'frame': str(msg.header.frame_id) if hasattr(msg, 'header') else '',
                    'fields': ', '.join(f.name for f in msg.fields[:6]),
                }
        if point_cloud2 is None or not self.should_process_visual(topic):
            return
        lowered = topic.lower()
        if not any(token in lowered for token in ('point', 'lidar', 'ouster', 'scan', 'radar')):
            return
        try:
            point_count = max(1, msg.width * msg.height)
            step = max(1, point_count // self.visual_max_points_per_topic)
            points = []
            for index, point in enumerate(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)):
                if index % step != 0:
                    continue
                x, y, z = float(point[0]), float(point[1]), float(point[2])
                rng = self.visual_range_m
                if abs(x) <= rng and abs(y) <= rng and -6.0 <= z <= 8.0:
                    points.append([round(x, 2), round(y, 2), round(z, 2)])
                if len(points) >= self.visual_max_points_per_topic:
                    break
            with self.lock:
                self.visual_points[topic] = {'stamp': _now(), 'topic': topic, 'points': points}
        except Exception as exc:
            with self.lock:
                stat = self.stats.get(topic)
                if stat:
                    stat.last_error = f'visual pointcloud parse: {exc}'

    def handle_marker_array(self, topic, msg):
        with self.lock:
            stat = self.stats.get(topic)
            if stat:
                stat.record()
                stat.last_value = {'markers': len(msg.markers)}
        if not self.should_process_visual(topic):
            return
        boxes = []
        markers = []
        for marker in msg.markers[:350]:
            parsed_boxes, parsed_markers = marker_to_visual(topic, marker)
            boxes.extend(parsed_boxes)
            markers.extend(parsed_markers)
        with self.lock:
            self.visual_boxes[topic] = {'stamp': _now(), 'items': boxes[:220]}
            self.visual_markers[topic] = {'stamp': _now(), 'items': markers[:260]}

    def handle_marker(self, topic, msg):
        self.handle_message(topic)
        if not self.should_process_visual(topic):
            return
        boxes, markers = marker_to_visual(topic, msg)
        with self.lock:
            self.visual_boxes[topic] = {'stamp': _now(), 'items': boxes[:40]}
            self.visual_markers[topic] = {'stamp': _now(), 'items': markers[:40]}

    def handle_detection_array(self, topic, msg):
        with self.lock:
            stat = self.stats.get(topic)
            if stat:
                stat.record()
                stat.last_value = {'detections': len(msg.detections)}
        if not self.should_process_visual(topic):
            return
        boxes = []
        for index, detection in enumerate(msg.detections[:180]):
            bbox = detection.bbox
            label = detection_label(detection, index)
            boxes.append({
                'topic': topic,
                'x': round(float(bbox.center.position.x), 2),
                'y': round(float(bbox.center.position.y), 2),
                'z': round(float(bbox.center.position.z), 2),
                'sx': round(max(float(bbox.size.x), 0.2), 2),
                'sy': round(max(float(bbox.size.y), 0.2), 2),
                'sz': round(max(float(bbox.size.z), 0.2), 2),
                'yaw': round(yaw_from_quaternion(bbox.center.orientation), 3),
                'label': label,
                'kind': 'detection',
            })
        with self.lock:
            self.visual_boxes[topic] = {'stamp': _now(), 'items': boxes}

    def handle_gps(self, topic, msg):
        self.handle_message(topic)
        try:
            lat = float(msg.latitude)
            lng = float(msg.longitude)
            alt = float(msg.altitude)
            if -90.0 <= lat <= 90.0 and -180.0 <= lng <= 180.0:
                gps_vals = {
                    'lat': round(lat, 7),
                    'lng': round(lng, 7),
                    'alt': round(alt, 2),
                }
                if hasattr(msg, 'status') and hasattr(msg.status, 'status'):
                    gps_vals['gps_status'] = int(msg.status.status)
                sats = None
                if hasattr(msg, 'status') and hasattr(msg.status, 'satellites_used'):
                    sats = int(msg.status.satellites_used)
                with self.lock:
                    stat = self.stats.get(topic)
                    if stat:
                        stat.last_value = gps_vals
                    record = {
                        'topic': topic,
                        'latitude': lat,
                        'longitude': lng,
                        'altitude': round(alt, 3),
                        'status': gps_vals.get('gps_status'),
                        'satellites': sats,
                        'stamp': _now(),
                    }
                    # bag 재생 네임스페이스는 실시간 GPS와 분리된 슬롯에 저장
                    # (재생 중에도 실제 차량 GPS를 덮어쓰지 않도록)
                    if topic.startswith(self.play_namespace + '/'):
                        self.gps_data_play = record
                    else:
                        self.gps_data = record
                        if self.gps_track_recording:
                            # 저장 파일에는 절대 시각이 유용하므로 epoch 초 사용 (_now는 monotonic)
                            self.gps_track_points.append(
                                [round(time.time(), 2), round(lat, 7), round(lng, 7), round(alt, 2)])
        except AttributeError:
            pass

    def gps_snapshot(self, play=False):
        with self.lock:
            src = self.gps_data_play if play else self.gps_data
            if not src:
                return None
            data = dict(src)
        data['age_sec'] = round(_now() - data.pop('stamp'), 1)
        return data

    # ── GPS 이동경로 기록/저장/불러오기 ──
    def start_gps_track(self):
        with self.lock:
            if self.gps_track_recording:
                return {'ok': False, 'error': '이미 기록 중입니다'}
            self.gps_track_recording = True
            self.gps_track_points = []
            self.gps_track_started = time.strftime('%Y-%m-%d %H:%M:%S')
        return {'ok': True}

    def stop_gps_track(self):
        with self.lock:
            if not self.gps_track_recording:
                return {'ok': False, 'error': '기록 중이 아닙니다'}
            self.gps_track_recording = False
            points = self.gps_track_points
            started = self.gps_track_started
            self.gps_track_points = []
            self.gps_track_started = None
        if not points:
            return {'ok': False, 'error': '기록된 GPS 포인트가 없습니다'}
        try:
            self.gps_track_dir.mkdir(parents=True, exist_ok=True)
            name = 'track_' + time.strftime('%Y%m%d_%H%M%S') + '.json'
            payload = {
                'started': started,
                'ended': time.strftime('%Y-%m-%d %H:%M:%S'),
                'point_count': len(points),
                # points: [unix_stamp, lat, lng, alt]
                'points': points,
            }
            (self.gps_track_dir / name).write_text(json.dumps(payload), encoding='utf-8')
            return {'ok': True, 'file': name, 'points': len(points)}
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}

    def list_gps_tracks(self):
        try:
            if not self.gps_track_dir.exists():
                return {'ok': True, 'files': []}
            files = sorted(self.gps_track_dir.glob('track_*.json'), reverse=True)
            out = []
            for f in files[:100]:
                out.append({'name': f.name, 'size_kb': round(f.stat().st_size / 1024, 1)})
            return {'ok': True, 'files': out}
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}

    def delete_gps_track(self, name):
        # 파일명만 허용 (경로 조작 방지)
        if not name or '/' in name or '\\' in name or '..' in name or not name.endswith('.json'):
            return {'ok': False, 'error': '잘못된 파일명'}
        path = self.gps_track_dir / name
        if not path.exists():
            return {'ok': False, 'error': '파일이 없습니다'}
        try:
            path.unlink()
            return {'ok': True, 'file': name}
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}

    def load_gps_track(self, name):
        # 파일명만 허용 (경로 조작 방지)
        if not name or '/' in name or '\\' in name or '..' in name or not name.endswith('.json'):
            return {'ok': False, 'error': '잘못된 파일명'}
        path = self.gps_track_dir / name
        if not path.exists():
            return {'ok': False, 'error': '파일이 없습니다'}
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
            return {'ok': True, 'file': name,
                    'started': data.get('started'), 'ended': data.get('ended'),
                    'points': data.get('points', [])}
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}

    # ── ROS2 bag 재생(Play) ──────────────────────────────────────────────
    def _bag_dir_path(self, name):
        """이름 검증(경로 조작 방지) 후 bag_dir 하위 절대경로 반환. 없거나 잘못되면 None."""
        if not name or '/' in name or '\\' in name or '..' in name:
            return None
        p = Path(self.bag_dir) / name
        return p if p.exists() else None

    def list_play_bags(self):
        """bag_dir 아래 재생 가능한 항목 나열: ROS2 bag 폴더(metadata.yaml) + ROS1 .bag 파일."""
        try:
            base = Path(self.bag_dir)
            if not base.exists():
                return {'ok': True, 'files': []}
            items = []
            for p in base.iterdir():
                try:
                    if p.is_dir() and (p / 'metadata.yaml').exists():
                        items.append({'name': p.name, 'type': 'ros2', 'mtime': p.stat().st_mtime})
                    elif p.is_file() and p.suffix == '.bag':
                        items.append({'name': p.name, 'type': 'ros1',
                                      'size_kb': round(p.stat().st_size / 1024, 1),
                                      'mtime': p.stat().st_mtime})
                except Exception:
                    continue
            items.sort(key=lambda x: x['mtime'], reverse=True)
            for it in items:
                it['modified'] = time.strftime('%Y-%m-%d %H:%M', time.localtime(it.pop('mtime')))
            return {'ok': True, 'files': items}
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}

    def read_bag_topics(self, bag_dir):
        """ROS2 bag의 metadata.yaml에서 토픽 이름 목록을 읽는다(리맵 인자 구성용)."""
        return self.read_bag_info(bag_dir)['topics']

    def read_bag_info(self, bag_dir):
        """ROS2 bag의 metadata.yaml에서 토픽 목록 + 시작시각/길이(seek용)를 읽는다."""
        out = {'topics': [], 'start_ns': None, 'duration_ns': None}
        try:
            meta = yaml.safe_load((Path(bag_dir) / 'metadata.yaml').read_text())
            info = (meta or {}).get('rosbag2_bagfile_information', {}) or {}
            for t in info.get('topics_with_message_count', []) or []:
                n = ((t or {}).get('topic_metadata', {}) or {}).get('name')
                if n:
                    out['topics'].append(n)
            start = (info.get('starting_time') or {}).get('nanoseconds_since_epoch')
            dur = (info.get('duration') or {}).get('nanoseconds')
            if isinstance(start, int):
                out['start_ns'] = start
            if isinstance(dur, int):
                out['duration_ns'] = dur
        except Exception:
            pass
        return out

    def convert_ros1_bag(self, path):
        """ROS1 .bag → ROS2 bag(sqlite3) 임시 변환. 성공 시 (변환된_ros2_bag_dir, None)."""
        exe = ensure_rosbags_convert()
        if not exe:
            return None, ('rosbags 변환 도구를 준비하지 못했습니다 (최초 1회 인터넷 연결 필요). '
                           f'수동 설치:\npython3 -m venv {ROSBAGS_VENV} && '
                           f'{ROSBAGS_VENV}/bin/pip install rosbags')
        tmp_dir = Path(tempfile.mkdtemp(prefix='ros_web_monitor_play_'))
        dst = tmp_dir / 'converted'
        try:
            proc = subprocess.run([exe, '--src', str(path), '--dst', str(dst)],
                                   capture_output=True, text=True, timeout=180)
            if proc.returncode != 0 or not (dst / 'metadata.yaml').exists():
                shutil.rmtree(tmp_dir, ignore_errors=True)
                err = (proc.stderr or proc.stdout or '').strip()[-500:]
                return None, f'ROS1 bag 변환 실패: {err or "알 수 없는 오류"}'
            return dst, None
        except Exception as exc:
            shutil.rmtree(tmp_dir, ignore_errors=True)
            return None, f'ROS1 bag 변환 오류: {exc}'

    def play_status(self):
        with self.play_lock:
            playing = self.play_process is not None and self.play_process.poll() is None
            info = {
                'playing': playing,
                'starting': self.play_starting,
                'file': self.play_file if (playing or self.play_starting) else None,
                'start_time': self.play_start_time if playing else None,
                'namespace': self.play_namespace,
            }
        if playing and self.play_start_ns is not None and self.play_duration_ns:
            dur_sec = self.play_duration_ns / 1e9
            pos_ns = self._play_estimated_ns()
            info['duration_sec'] = round(dur_sec, 2)
            info['position_sec'] = round(max(0.0, min(dur_sec, (pos_ns - self.play_start_ns) / 1e9)), 2)
            info['paused'] = bool(self.play_pos_ref.get('paused'))
            info['rate'] = self.play_pos_ref.get('rate', 1.0)
            info['loop'] = self.play_loop
            info['ctrl_available'] = BAG_CTRL_AVAILABLE
        return info

    # ── 재생 위치 추정 (position-query 서비스가 없어 기준점+경과시간으로 근사) ──
    def _play_estimated_ns(self):
        ref = self.play_pos_ref
        if not ref or self.play_start_ns is None:
            return self.play_start_ns or 0
        if ref.get('paused'):
            return ref.get('bag_ns', self.play_start_ns)
        elapsed_sec = (time.monotonic() - ref.get('wall', time.monotonic())) * ref.get('rate', 1.0)
        ns = ref.get('bag_ns', self.play_start_ns) + int(elapsed_sec * 1e9)
        end_ns = self.play_start_ns + (self.play_duration_ns or 0)
        if self.play_loop and self.play_duration_ns:
            ns = self.play_start_ns + ((ns - self.play_start_ns) % self.play_duration_ns)
        else:
            ns = min(ns, end_ns)
        return max(self.play_start_ns, ns)

    def _play_sync_position(self):
        """현재 추정 위치를 새 기준점으로 고정 (배속 변경 직전 등에 호출)."""
        self.play_pos_ref['bag_ns'] = self._play_estimated_ns()
        self.play_pos_ref['wall'] = time.monotonic()

    # ── 재생 제어 서비스 호출 (/rosbag2_player/pause 등) ──
    def _play_client(self, name, srv_type):
        client = self.play_ctrl_clients.get(name)
        if client is None:
            client = self.create_client(srv_type, f'/rosbag2_player/{name}')
            self.play_ctrl_clients[name] = client
        return client

    def _play_call(self, name, srv_type, request, timeout_sec=2.5):
        with self.play_lock:
            proc = self.play_process
        if proc is None or proc.poll() is not None:
            return None, '재생 중이 아닙니다'
        if not BAG_CTRL_AVAILABLE:
            return None, '이 ROS 배포판에는 rosbag2_interfaces가 없어 재생 제어를 지원하지 않습니다'
        try:
            client = self._play_client(name, srv_type)
            if not client.wait_for_service(timeout_sec=timeout_sec):
                return None, '재생 제어 서비스에 연결할 수 없습니다 (준비 중일 수 있습니다)'
            future = client.call_async(request)
            deadline = time.monotonic() + timeout_sec
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not future.done():
                return None, '재생 제어 응답 시간 초과'
            exc = future.exception()
            if exc:
                return None, str(exc)
            return future.result(), None
        except Exception as exc:
            return None, str(exc)

    def play_pause(self):
        if not BAG_CTRL_AVAILABLE:
            return {'ok': False, 'error': '이 ROS 배포판에는 rosbag2_interfaces가 없어 재생 제어를 지원하지 않습니다'}
        result, err = self._play_call('pause', BagPause, BagPause.Request())
        if err:
            return {'ok': False, 'error': err}
        self.play_pos_ref['bag_ns'] = self._play_estimated_ns()
        self.play_pos_ref['paused'] = True
        return {'ok': True, 'paused': True}

    def play_resume(self):
        if not BAG_CTRL_AVAILABLE:
            return {'ok': False, 'error': '이 ROS 배포판에는 rosbag2_interfaces가 없어 재생 제어를 지원하지 않습니다'}
        result, err = self._play_call('resume', BagResume, BagResume.Request())
        if err:
            return {'ok': False, 'error': err}
        self.play_pos_ref['wall'] = time.monotonic()
        self.play_pos_ref['paused'] = False
        return {'ok': True, 'paused': False}

    def play_next_frame(self):
        if not BAG_CTRL_AVAILABLE:
            return {'ok': False, 'error': '이 ROS 배포판에는 rosbag2_interfaces가 없어 재생 제어를 지원하지 않습니다'}
        result, err = self._play_call('play_next', BagPlayNext, BagPlayNext.Request())
        if err:
            return {'ok': False, 'error': err}
        if not getattr(result, 'success', False):
            return {'ok': False, 'error': '일시정지 상태에서만 다음 프레임으로 이동할 수 있습니다'}
        return {'ok': True}

    def play_set_rate(self, rate):
        if not BAG_CTRL_AVAILABLE:
            return {'ok': False, 'error': '이 ROS 배포판에는 rosbag2_interfaces가 없어 재생 제어를 지원하지 않습니다'}
        try:
            rate = float(rate)
        except (TypeError, ValueError):
            return {'ok': False, 'error': '올바르지 않은 배속입니다'}
        if rate <= 0:
            return {'ok': False, 'error': '배속은 0보다 커야 합니다'}
        req = BagSetRate.Request()
        req.rate = rate
        result, err = self._play_call('set_rate', BagSetRate, req)
        if err:
            return {'ok': False, 'error': err}
        if not getattr(result, 'success', False):
            return {'ok': False, 'error': '배속 적용 실패'}
        self._play_sync_position()
        self.play_pos_ref['rate'] = rate
        return {'ok': True, 'rate': rate}

    def play_seek(self, offset_sec):
        if not BAG_CTRL_AVAILABLE:
            return {'ok': False, 'error': '이 ROS 배포판에는 rosbag2_interfaces가 없어 재생 제어를 지원하지 않습니다'}
        if self.play_start_ns is None or not self.play_duration_ns:
            return {'ok': False, 'error': '재생 길이 정보를 알 수 없어 이동할 수 없습니다'}
        try:
            offset_sec = float(offset_sec)
        except (TypeError, ValueError):
            return {'ok': False, 'error': '올바르지 않은 위치입니다'}
        dur_sec = self.play_duration_ns / 1e9
        offset_sec = max(0.0, min(dur_sec, offset_sec))
        target_ns = self.play_start_ns + int(round(offset_sec * 1e9))
        req = BagSeek.Request()
        req.time.sec = target_ns // 1_000_000_000
        req.time.nanosec = target_ns % 1_000_000_000
        result, err = self._play_call('seek', BagSeek, req)
        if err:
            return {'ok': False, 'error': err}
        if not getattr(result, 'success', False):
            return {'ok': False, 'error': '유효하지 않은 위치입니다'}
        self.play_pos_ref['bag_ns'] = target_ns
        self.play_pos_ref['wall'] = time.monotonic()
        return {'ok': True, 'position_sec': offset_sec}

    def start_play(self, name, loop=False):
        # 변환(최대 180초)처럼 오래 걸리는 작업 중에는 play_lock을 놓아 두어
        # 다른 클라이언트의 상태 조회(snapshot)가 멈추지 않게 한다.
        with self.play_lock:
            already = self.play_process is not None and self.play_process.poll() is None
            if self.play_starting or already:
                return {'ok': False, 'error': '이미 재생 중이거나 준비 중입니다'}
            self.play_starting = True
            self.play_file = name  # 준비 중에도 파일명을 먼저 노출

        def fail(msg):
            with self.play_lock:
                self.play_starting = False
                self.play_file = None
            return {'ok': False, 'error': msg}

        path = self._bag_dir_path(name)
        if not path:
            return fail('유효하지 않은 bag 파일입니다')

        tmp_dir = None
        if path.is_file() and path.suffix == '.bag':
            bag_dir, err = self.convert_ros1_bag(path)
            if err:
                return fail(err)
            tmp_dir = bag_dir.parent
        elif path.is_dir() and (path / 'metadata.yaml').exists():
            bag_dir = path
        else:
            return fail('지원하지 않는 bag 형식입니다 (.bag 또는 ROS2 bag 폴더만 가능)')

        bag_info = self.read_bag_info(bag_dir)
        topics = bag_info['topics']
        # 실시간 토픽과 절대 안 섞이도록 모든 토픽을 play_namespace 하위로 리맵
        remaps = [f'{t}:={self.play_namespace}{t}' for t in topics]
        cmd = ['ros2', 'bag', 'play', str(bag_dir), '--disable-keyboard-controls']
        if loop:
            cmd.append('-l')
        if remaps:
            cmd += ['--remap'] + remaps

        try:
            log = open(os.path.join(self.bag_dir, '.play_log.txt'), 'w')
            proc = subprocess.Popen(cmd, cwd=self.bag_dir, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        except Exception as exc:
            if tmp_dir:
                shutil.rmtree(tmp_dir, ignore_errors=True)
            return fail(str(exc))

        with self.play_lock:
            self.play_process = proc
            self._play_log = log
            self.play_tmp_dir = tmp_dir
            self.play_start_time = time.strftime('%Y-%m-%d %H:%M:%S')
            self.play_starting = False
            self.play_loop = loop
            self.play_start_ns = bag_info['start_ns']
            self.play_duration_ns = bag_info['duration_ns']
            self.play_pos_ref = {
                'bag_ns': self.play_start_ns or 0,
                'wall': time.monotonic(),
                'rate': 1.0,
                'paused': False,
            }
            self.play_ctrl_clients = {}
        self.get_logger().info(f'bag 재생 시작: {cmd}')
        return {'ok': True, 'playing': True, 'file': name, 'start_time': self.play_start_time,
                'topics': topics, 'namespace': self.play_namespace,
                'duration_sec': round(self.play_duration_ns / 1e9, 2) if self.play_duration_ns else None}

    def stop_play(self):
        with self.play_lock:
            proc = self.play_process
            if proc is None or proc.poll() is not None:
                self.play_process = None
                self.play_starting = False
                return {'ok': False, 'error': '재생 중이 아닙니다'}
            self.play_process = None  # 중복 stop 방지 위해 즉시 비움
            tmp_dir = self.play_tmp_dir
            log = self._play_log
            played = self.play_file
            self.play_tmp_dir = None
            self._play_log = None
            self.play_file = None
            self.play_start_time = None
            self.play_start_ns = None
            self.play_duration_ns = None
            self.play_pos_ref = {}
            ctrl_clients = self.play_ctrl_clients
            self.play_ctrl_clients = {}

        for client in ctrl_clients.values():
            try:
                self.destroy_client(client)
            except Exception:
                pass

        try:
            pgid = os.getpgid(proc.pid)
        except Exception:
            pgid = None
        try:
            if pgid is not None:
                os.killpg(pgid, signal.SIGINT)
            else:
                proc.send_signal(signal.SIGINT)
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                if pgid is not None:
                    os.killpg(pgid, signal.SIGTERM)
                else:
                    proc.terminate()
                proc.wait(timeout=5)
            except Exception:
                try:
                    if pgid is not None:
                        os.killpg(pgid, signal.SIGKILL)
                    else:
                        proc.kill()
                except Exception:
                    pass
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        if log:
            try:
                log.close()
            except Exception:
                pass
        if tmp_dir:
            shutil.rmtree(tmp_dir, ignore_errors=True)
        self.get_logger().info(f'bag 재생 종료: {played}')
        return {'ok': True, 'playing': False, 'file': played}

    def visualization_snapshot(self):
        now = _now()
        with self.lock:
            point_layers = [
                {**layer, 'age_sec': round(now - layer['stamp'], 2)}
                for layer in self.visual_points.values()
                if now - layer['stamp'] < 5.0
            ]
            box_items = []
            marker_items = []
            for layer in self.visual_boxes.values():
                if now - layer['stamp'] < 5.0:
                    box_items.extend(layer['items'])
            for layer in self.visual_markers.values():
                if now - layer['stamp'] < 5.0:
                    marker_items.extend(layer['items'])
        point_layers.sort(key=lambda item: item['topic'])
        return {
            'range_m': int(self.visual_range_m),
            'point_layers': point_layers[:12],
            'boxes': box_items[:420],
            'markers': marker_items[:420],
        }

    def snapshot(self):
        with self.lock:
            topics = [stat.snapshot(self.rate_window_sec) for stat in self.stats.values()]
            cameras = []
            for name, frame in self.camera_frames.items():
                cameras.append({
                    'name': name,
                    'age_sec': round(_now() - frame['stamp'], 2),
                    'stream_url': '/stream?topic=' + name,
                })
        topics.sort(key=lambda item: item['name'])
        cameras.sort(key=lambda item: item['name'])
        groups = group_topics(topics)
        active = sum(1 for item in topics if item['active'])
        return {
            'time': time.strftime('%Y-%m-%d %H:%M:%S'),
            'stream_port': self.stream_port if getattr(self, 'stream_server', None) else None,
            'topic_count': len(topics),
            'active_count': active,
            'camera_count': len(cameras),
            'topics': topics,
            'groups': groups,
            'cameras': cameras,
            'visualization': self.visualization_snapshot(),
            'system': self.system_snapshot(),
            'gps': self.gps_snapshot(),
            'gps_play': self.gps_snapshot(play=True),
            'gps_track': {
                'recording': self.gps_track_recording,
                'points': len(self.gps_track_points),
            },
            'bag': self.bag_status(),
            'bag_play': self.play_status(),
            'play_namespace': self.play_namespace,
            'vehicle': self.vehicle_snapshot('/acu_ck2_ver2'),
            'vehicle_play': self.vehicle_snapshot(self.play_namespace + '/acu_ck2_ver2'),
        }

    def snapshot_json(self):
        """직렬화된 스냅샷 문자열 (0.25s 캐시 — 다중 클라이언트가 공유)."""
        now = _now()
        with self._snap_json_lock:
            if self._snap_json is not None and now - self._snap_json_stamp < 0.25:
                return self._snap_json
            data = json.dumps(json_safe(self.snapshot()), allow_nan=False)
            self._snap_json = data
            self._snap_json_stamp = now
            return data

    def vehicle_snapshot(self, topic='/acu_ck2_ver2'):
        """지정 토픽(기본 /acu_ck2_ver2, 재생 시 /bagplay/acu_ck2_ver2)의 last_value 에서 차량 상태 추출."""
        with self.lock:
            stat = self.stats.get(topic)
            lv = dict(stat.last_value) if stat and stat.last_value else None
        if not lv:
            return None

        def num(key):
            x = lv.get(key)
            return round(float(x), 2) if isinstance(x, (int, float)) else None

        return {
            'gear':        num('gear_pos'),
            'steer_angle': num('steer_ang_val'),
            'steer_speed': num('steer_spd_val'),
            'brake':       num('brake_est_dpt_val'),
            'wheel_fr':    num('whl_speed_front_right'),
            'wheel_fl':    num('whl_speed_front_left'),
            'wheel_rr':    num('whl_speed_rear_right'),
            'wheel_rl':    num('whl_speed_rear_left'),
            'soc':         num('x_ev_soc_val'),
            'motor_rpm':   num('motor_rpm'),
            'motor_tq':    num('motor_feedback_tq_val'),
        }

    # ── ROS2 bag 녹화 ────────────────────────────────────────────────────
    def bag_status(self):
        with self.bag_lock:
            recording = self.bag_process is not None and self.bag_process.poll() is None
            return {
                'recording': recording,
                'start_time': self.bag_start_time if recording else None,
                'topics': list(self.bag_topics) if recording else [],
                'last_saved': self.bag_last_saved,
                'dir': self.bag_dir,
            }

    def set_bag_dir(self, path):
        """Bag 녹화/재생 저장 경로 변경. 녹화·재생 중에는 변경 불가."""
        with self.bag_lock:
            recording = self.bag_process is not None and self.bag_process.poll() is None
        with self.play_lock:
            playing = self.play_process is not None and self.play_process.poll() is None
        if recording:
            return {'ok': False, 'error': '녹화 중에는 저장 경로를 변경할 수 없습니다'}
        if playing:
            return {'ok': False, 'error': '재생 중에는 저장 경로를 변경할 수 없습니다'}
        if not path or not isinstance(path, str) or not path.strip():
            return {'ok': False, 'error': '경로를 입력하세요'}
        path = os.path.expanduser(path.strip())
        if not path.startswith('/'):
            return {'ok': False, 'error': '절대 경로를 입력하세요 (예: /home/nvidia/robot_ws/bag)'}
        try:
            os.makedirs(path, exist_ok=True)
            probe = os.path.join(path, '.write_test')
            with open(probe, 'w') as f:
                f.write('')
            os.remove(probe)
        except Exception as exc:
            return {'ok': False, 'error': f'경로를 사용할 수 없습니다: {exc}'}
        self.bag_dir = path
        self.get_logger().info(f'bag 저장 경로 변경: {path}')
        return {'ok': True, 'dir': path}

    # ── 저장 경로 선택용 서버측 폴더 탐색기 (Bag 저장 경로 "찾아보기") ──
    def list_dir(self, path):
        """지정 경로의 하위 디렉토리 목록을 반환. 존재하지 않으면 가장 가까운 상위 폴더로 대체."""
        try:
            base = os.path.expanduser((path or '').strip()) or self.bag_dir
            if not base.startswith('/'):
                base = self.bag_dir
            base = os.path.realpath(base)
            while base != '/' and not os.path.isdir(base):
                base = os.path.dirname(base)
            if not os.path.isdir(base):
                base = '/'
            try:
                names = []
                with os.scandir(base) as it:
                    for entry in it:
                        try:
                            if entry.name.startswith('.'):
                                continue
                            if entry.is_dir(follow_symlinks=False):
                                names.append(entry.name)
                        except OSError:
                            continue
            except PermissionError:
                return {'ok': False, 'error': '이 폴더에 접근 권한이 없습니다', 'path': base}
            names.sort(key=str.lower)
            parent = os.path.dirname(base) if base != '/' else None
            return {'ok': True, 'path': base, 'parent': parent, 'dirs': names,
                     'writable': os.access(base, os.W_OK)}
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}

    def make_dir(self, path, name):
        """폴더 탐색기에서 새 하위 폴더 생성."""
        if not name or not isinstance(name, str) or '/' in name or '\\' in name or name in ('.', '..'):
            return {'ok': False, 'error': '올바르지 않은 폴더 이름입니다'}
        base = os.path.realpath(os.path.expanduser((path or '').strip()))
        if not os.path.isdir(base):
            return {'ok': False, 'error': '상위 폴더가 존재하지 않습니다'}
        new_path = os.path.join(base, name.strip())
        try:
            os.makedirs(new_path, exist_ok=True)
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}
        return self.list_dir(new_path)

    def start_bag(self, topics=None):
        with self.bag_lock:
            if self.bag_process is not None and self.bag_process.poll() is None:
                return {'ok': False, 'error': '이미 녹화 중입니다'}
            try:
                os.makedirs(self.bag_dir, exist_ok=True)
                # 처음부터 최종 타임스탬프 폴더명으로 기록
                #  → mcap 파일명이 '<폴더명>_0.mcap'이 되어 숨김파일/이름 문제 없음,
                #    metadata.yaml 참조 정합성도 보장 (rename 불필요)
                name = time.strftime('%Y_%m_%d_%H_%M_%S')
                out = os.path.join(self.bag_dir, name)
                if os.path.exists(out):
                    out = f'{out}_{int(time.time())}'
                    name = os.path.basename(out)
                # 토픽 선택: 비어있으면 전체(-a), 아니면 지정 토픽만
                sel = [t for t in (topics or []) if isinstance(t, str) and t.strip()]
                cmd = ['ros2', 'bag', 'record', '-o', out]
                cmd += sel if sel else ['-a']
                # 디버깅용 로그 캡처
                self._bag_log = open(os.path.join(self.bag_dir, '.record_log.txt'), 'w')
                # start_new_session=True → 별도 프로세스 그룹 생성(종료 시 그룹 전체에 SIGINT)
                self.bag_process = subprocess.Popen(
                    cmd,
                    cwd=self.bag_dir,
                    stdout=self._bag_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                self.bag_tmp = out
                self.bag_topics = sel
                self.bag_start_time = name
                self.bag_last_saved = None
                self.get_logger().info(f'bag 녹화 시작: {cmd}')
                return {'ok': True, 'recording': True, 'start_time': self.bag_start_time,
                        'topics': sel}
            except Exception as exc:
                self.bag_process = None
                self.bag_tmp = None
                return {'ok': False, 'error': str(exc)}

    def stop_bag(self):
        with self.bag_lock:
            if self.bag_process is None or self.bag_process.poll() is not None:
                self.bag_process = None
                return {'ok': False, 'error': '녹화 중이 아닙니다'}
            proc = self.bag_process
            # 프로세스 그룹 전체에 SIGINT → recorder가 mcap을 flush/close하고
            # metadata.yaml까지 정상 마무리 (이게 빠지면 .mcap이 안 닫힘)
            try:
                pgid = os.getpgid(proc.pid)
            except Exception:
                pgid = None
            try:
                if pgid is not None:
                    os.killpg(pgid, signal.SIGINT)
                else:
                    proc.send_signal(signal.SIGINT)
                proc.wait(timeout=25)
            except subprocess.TimeoutExpired:
                try:
                    if pgid is not None:
                        os.killpg(pgid, signal.SIGTERM)
                    else:
                        proc.terminate()
                    proc.wait(timeout=5)
                except Exception:
                    try:
                        if pgid is not None:
                            os.killpg(pgid, signal.SIGKILL)
                        else:
                            proc.kill()
                    except Exception:
                        pass
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
            self.bag_process = None
            if self._bag_log:
                try:
                    self._bag_log.close()
                except Exception:
                    pass
                self._bag_log = None
            # 시작 시점에 이미 최종 폴더명으로 기록했으므로 이동 불필요
            saved = None
            mcap_count = 0
            if self.bag_tmp and os.path.isdir(self.bag_tmp):
                saved = self.bag_tmp
                # 숨김(.) mcap까지 모두 카운트
                mcap_count = len(glob.glob(os.path.join(saved, '*.mcap'))) \
                    + len(glob.glob(os.path.join(saved, '.*.mcap')))
            self.bag_tmp = None
            self.bag_start_time = None
            self.bag_last_saved = saved
            self.get_logger().info(f'bag 녹화 종료, 저장: {saved} (mcap {mcap_count}개)')
            return {'ok': True, 'recording': False, 'path': saved, 'mcap_count': mcap_count}

    # ── 시스템 전원 제어 ─────────────────────────────────────────────────
    def system_power(self, action):
        if action == 'shutdown':
            cmd = ['/usr/sbin/shutdown', '-h', 'now']
        elif action == 'reboot':
            cmd = ['/usr/sbin/reboot']
        else:
            return {'ok': False, 'error': '알 수 없는 동작'}
        # 녹화 중이면 먼저 안전하게 저장
        try:
            if self.bag_process is not None and self.bag_process.poll() is None:
                self.stop_bag()
        except Exception:
            pass
        self.get_logger().warn(f'시스템 {action} 요청 → {" ".join(cmd)}')
        # sudo -n(비번 없이)로 실행 후 결과를 실제로 확인해서 실패를 보고한다.
        try:
            proc = subprocess.run(['sudo', '-n'] + cmd, capture_output=True, text=True, timeout=15)
            if proc.returncode == 0:
                return {'ok': True, 'action': action}
            err = (proc.stderr or proc.stdout or '').strip()
            if 'password' in err.lower():
                user = os.environ.get('USER', 'nvidia')
                err = ('passwordless sudo 미설정 — 1회 설정 필요:\n'
                       f'echo "{user} ALL=(root) NOPASSWD: /usr/sbin/shutdown, /usr/sbin/reboot" '
                       '| sudo tee /etc/sudoers.d/ros_web_monitor_power '
                       '&& sudo chmod 440 /etc/sudoers.d/ros_web_monitor_power')
            return {'ok': False, 'error': err or 'sudo 실패'}
        except subprocess.TimeoutExpired:
            # 종료/재부팅이 진행되어 응답 전에 끊긴 정상 케이스
            return {'ok': True, 'action': action}
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}

    def system_snapshot(self):
        now = _now()
        if now - self.system_cache_stamp < 1.0 and self.system_cache:
            return self.system_cache
        if self.jtop_monitor is not None:
            snap = self.jtop_monitor.snapshot()
            if snap:
                self.system_cache = snap
                self.system_cache_stamp = now
                return snap
        snapshot = collect_system_snapshot()
        self.system_cache = snapshot
        self.system_cache_stamp = now
        return snapshot

    def wait_for_frame(self, topic, previous_stamp=None, timeout=2.0):
        condition = self.camera_conditions[topic]
        deadline = _now() + timeout
        while _now() < deadline:
            with self.lock:
                frame = self.camera_frames.get(topic)
                if frame and frame['stamp'] != previous_stamp:
                    return frame
            with condition:
                condition.wait(timeout=0.25)
        with self.lock:
            return self.camera_frames.get(topic)





def collect_system_snapshot():
    return {
        'model': read_first_existing(['/proc/device-tree/model'], strip_null=True),
        'jetpack': read_jetpack_version(),
        'cpu': read_cpu_snapshot(),
        'memory': read_memory_snapshot(),
        'gpu': read_gpu_snapshot(),
        'thermal': read_thermal_snapshot(),
        'uptime_sec': read_uptime(),
    }


def read_first_existing(paths, strip_null=False):
    for path in paths:
        try:
            data = Path(path).read_text(errors='ignore').strip()
            if strip_null:
                data = data.replace('\x00', '').strip()
            if data:
                return data
        except Exception:
            pass
    return ''


def read_jetpack_version():
    release = read_first_existing(['/etc/nv_tegra_release'])
    try:
        proc = subprocess.run(['dpkg-query', '-W', '-f=${Version}', 'nvidia-l4t-core'], capture_output=True, text=True, timeout=0.5)
        l4t = proc.stdout.strip() if proc.returncode == 0 else ''
    except Exception:
        l4t = ''
    if release and l4t:
        return f'{release} / L4T {l4t}'
    return release or (f'L4T {l4t}' if l4t else 'unknown')


def read_cpu_snapshot():
    per_core = []
    try:
        lines = Path('/proc/stat').read_text().splitlines()
        for line in lines:
            if not re.match(r'^cpu\d+\s', line):
                continue
            parts = [int(v) for v in line.split()[1:]]
            idle = parts[3] + (parts[4] if len(parts) > 4 else 0)
            total = sum(parts)
            used = 0.0 if total <= 0 else (total - idle) / total * 100.0
            per_core.append(round(used, 1))
    except Exception:
        pass
    load = os.getloadavg() if hasattr(os, 'getloadavg') else (0.0, 0.0, 0.0)
    overall = round(sum(per_core) / len(per_core), 1) if per_core else 0.0
    return {'usage_percent': overall, 'cores': per_core, 'loadavg': [round(v, 2) for v in load]}


def read_memory_snapshot():
    values = {}
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            key, value = line.split(':', 1)
            values[key] = int(value.strip().split()[0])
    except Exception:
        pass
    total = values.get('MemTotal', 0)
    available = values.get('MemAvailable', 0)
    used = max(total - available, 0)
    percent = round(used / total * 100.0, 1) if total else 0.0
    return {'total_mb': round(total / 1024), 'used_mb': round(used / 1024), 'available_mb': round(available / 1024), 'usage_percent': percent}


def read_gpu_snapshot():
    result = {'usage_percent': None, 'raw': ''}
    try:
        proc = subprocess.run(['timeout', '1', 'tegrastats', '--interval', '100'], capture_output=True, text=True, timeout=1.5)
        lines = [line for line in proc.stdout.splitlines() if line.strip()]
        raw = lines[-1].strip() if lines else (proc.stderr or '').strip()
        result['raw'] = raw
        match = re.search(r'GR3D_FREQ\s+(\d+)%', raw)
        if match:
            result['usage_percent'] = int(match.group(1))
        temp = re.search(r'gpu@(\d+(?:\.\d+)?)C', raw)
        if temp:
            result['temp_c'] = round(float(temp.group(1)), 1)
        power = re.search(r'VDD_GPU\s+(\d+)mW/(\d+)mW', raw)
        if power:
            result['power_mw'] = int(power.group(1))
            result['avg_power_mw'] = int(power.group(2))
        mem = re.search(r'RAM\s+(\d+)/(\d+)MB', raw)
        if mem:
            result['tegrastats_ram_mb'] = {'used': int(mem.group(1)), 'total': int(mem.group(2))}
    except Exception:
        pass
    return result


def read_thermal_snapshot():
    temps = []
    for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        try:
            temp = int(path.read_text().strip()) / 1000.0
            kind = path.parent.joinpath('type').read_text(errors='ignore').strip()
            temps.append({'name': kind, 'c': round(temp, 1)})
        except Exception:
            pass
    temps.sort(key=lambda item: item['c'], reverse=True)
    return temps[:8]


def read_uptime():
    try:
        return round(float(Path('/proc/uptime').read_text().split()[0]))
    except Exception:
        return 0

def is_streamable_raw_image_topic(topic):
    lowered = topic.lower()
    return 'result_image' in lowered or lowered.endswith('/avm_cam/image_raw')


def encode_raw_image_as_jpeg(msg):
    encoding = msg.encoding.lower()
    channels = 1
    if encoding in ('rgb8', 'bgr8'):
        channels = 3
    elif encoding in ('rgba8', 'bgra8'):
        channels = 4
    elif encoding in ('mono8', '8uc1'):
        channels = 1
    else:
        raise ValueError(f'unsupported encoding {msg.encoding}')

    row = np.frombuffer(msg.data, dtype=np.uint8)
    expected_row_bytes = msg.width * channels
    if msg.step and msg.step != expected_row_bytes:
        row = row.reshape((msg.height, msg.step))[:, :expected_row_bytes]
        image = row.reshape((msg.height, msg.width, channels)) if channels > 1 else row.reshape((msg.height, msg.width))
    else:
        image = row.reshape((msg.height, msg.width, channels)) if channels > 1 else row.reshape((msg.height, msg.width))

    if encoding == 'bgr8':
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    elif encoding == 'bgra8':
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
    elif encoding == 'rgba8':
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)

    pil_image = PilImage.fromarray(image)
    output = io.BytesIO()
    pil_image.save(output, format='JPEG', quality=82)
    return output.getvalue()

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def color_to_hex(color, fallback='#64c8ff'):
    if getattr(color, 'a', 0.0) == 0.0 and color.r == 0.0 and color.g == 0.0 and color.b == 0.0:
        return fallback
    r = max(0, min(255, int(color.r * 255)))
    g = max(0, min(255, int(color.g * 255)))
    b = max(0, min(255, int(color.b * 255)))
    return f'#{r:02x}{g:02x}{b:02x}'


def marker_to_visual(topic, marker):
    boxes = []
    markers = []
    color = color_to_hex(marker.color, marker_color(topic))
    kind = marker_kind(topic)
    if marker.type in (1, 2):
        boxes.append({
            'topic': topic,
            'x': round(float(marker.pose.position.x), 2),
            'y': round(float(marker.pose.position.y), 2),
            'z': round(float(marker.pose.position.z), 2),
            'sx': round(max(float(marker.scale.x), 0.2), 2),
            'sy': round(max(float(marker.scale.y), 0.2), 2),
            'sz': round(max(float(marker.scale.z), 0.2), 2),
            'yaw': round(yaw_from_quaternion(marker.pose.orientation), 3),
            'label': marker.text or marker.ns or topic.rsplit('/', 1)[-1],
            'kind': kind,
            'color': color,
        })
    if marker.points:
        points = [[round(float(p.x), 2), round(float(p.y), 2), round(float(p.z), 2)] for p in marker.points[:500]]
        markers.append({
            'topic': topic,
            'type': int(marker.type),
            'points': points,
            'color': color,
            'scale': round(max(float(marker.scale.x), float(marker.scale.y), 0.08), 2),
            'kind': kind,
        })
    return boxes, markers


def marker_color(topic):
    lowered = topic.lower()
    if 'radar' in lowered:
        return '#f2c15b'
    if 'cluster' in lowered or 'object' in lowered or 'detection' in lowered:
        return '#f46d75'
    if 'track' in lowered:
        return '#55d188'
    return '#64c8ff'


def marker_kind(topic):
    lowered = topic.lower()
    if 'radar' in lowered:
        return 'radar'
    if 'track' in lowered:
        return 'track'
    if 'cluster' in lowered or 'object' in lowered or 'detection' in lowered:
        return 'object'
    return 'marker'


def detection_label(detection, index):
    if detection.results:
        result = detection.results[0]
        hypothesis = getattr(result, 'hypothesis', None)
        if hypothesis and getattr(hypothesis, 'class_id', ''):
            return hypothesis.class_id
    return f'object {index + 1}'

def group_topics(topics):
    groups = defaultdict(lambda: {'total': 0, 'active': 0, 'rate_hz': 0.0})
    for item in topics:
        name = item['name'].strip('/')
        key = name.split('/')[0] if name else 'root'
        lowered = item['name'].lower()
        if 'camera' in lowered or 'image' in lowered:
            key = 'cameras'
        elif 'ouster' in lowered or 'lidar' in lowered or 'point' in lowered or 'scan' in lowered:
            key = 'lidar'
        elif 'radar' in lowered:
            key = 'radar'
        elif 'gps' in lowered or 'geodetic' in lowered or 'cov' in lowered or 'euler' in lowered:
            key = 'localization'
        elif 'can' in lowered or lowered.startswith('/acu'):
            key = 'vehicle_can'
        elif 'object' in lowered or 'cluster' in lowered or 'detection' in lowered or 'track' in lowered:
            key = 'perception'
        groups[key]['total'] += 1
        groups[key]['active'] += 1 if item['active'] else 0
        groups[key]['rate_hz'] += item['rate_hz']
    return [
        {'name': key, 'total': value['total'], 'active': value['active'], 'rate_hz': round(value['rate_hz'], 2)}
        for key, value in sorted(groups.items())
    ]


def make_handler(node):
    class MonitorRequestHandler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == '/':
                self.serve_file(STATIC_DIR / 'index.html')
            elif parsed.path == '/app.css':
                self.serve_file(STATIC_DIR / 'app.css')
            elif parsed.path == '/app.js':
                self.serve_file(STATIC_DIR / 'app.js')
            elif parsed.path == '/api/ui/default-settings':
                path = default_ui_settings_file()
                if path is None:
                    self.send_error(HTTPStatus.NOT_FOUND, 'No UI settings file')
                else:
                    self.serve_file(path)
            elif parsed.path == '/api/state':
                self.serve_json_str(node.snapshot_json())
            elif parsed.path == '/events':
                self.serve_events()
            elif parsed.path == '/stream':
                topic = parse_qs(parsed.query).get('topic', [''])[0]
                self.serve_mjpeg(topic)
            elif parsed.path == '/api/gps/track/list':
                self.serve_json(node.list_gps_tracks())
            elif parsed.path == '/api/gps/track/load':
                name = parse_qs(parsed.query).get('file', [''])[0]
                self.serve_json(node.load_gps_track(name))
            elif parsed.path == '/api/bag/play/list':
                self.serve_json(node.list_play_bags())
            elif parsed.path == '/api/fs/list':
                path = parse_qs(parsed.query).get('path', [''])[0]
                self.serve_json(node.list_dir(path))
            else:
                self.send_error(HTTPStatus.NOT_FOUND, 'Not found')

        def serve_file(self, path):
            if not path.exists():
                self.send_error(HTTPStatus.NOT_FOUND, 'File not found')
                return
            data = path.read_bytes()
            content_type = mimetypes.guess_type(str(path))[0] or 'application/octet-stream'
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            # 정적 파일 캐시 방지 (브라우저가 옛 버전 쓰는 문제 해결)
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def serve_json(self, payload):
            data = json.dumps(json_safe(payload), allow_nan=False).encode('utf-8')
            try:
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def serve_json_str(self, payload_str):
            """이미 직렬화된 JSON 문자열 전송 (스냅샷 캐시용)."""
            data = payload_str.encode('utf-8')
            try:
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass  # HTTP 접근 로그 억제

        def serve_events(self):
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            try:
                while True:
                    # 캐시된 직렬화 결과 사용 → 다중 클라이언트여도 스냅샷 1회 생성
                    payload = node.snapshot_json()
                    self.wfile.write(f'data: {payload}\n\n'.encode('utf-8'))
                    self.wfile.flush()
                    # 0.5s(2Hz): 1Hz보다 반응성↑ + monitor_node CPU 부담 완화(카메라 등
                    # 다른 노드에 코어 양보). 프론트는 무거운 DOM 0.7s 쓰로틀 + 시각화 rAF.
                    time.sleep(0.5)
            except (BrokenPipeError, ConnectionResetError):
                return

        def serve_mjpeg(self, topic):
            if not topic:
                self.send_error(HTTPStatus.BAD_REQUEST, 'Missing topic')
                return
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            last_stamp = None
            try:
                while True:
                    frame = node.wait_for_frame(topic, last_stamp)
                    if not frame:
                        time.sleep(0.25)
                        continue
                    last_stamp = frame['stamp']
                    data = frame['data']
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(data)}\r\n\r\n'.encode('ascii'))
                    self.wfile.write(data)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path == '/api/viz-config':
                length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(length)
                try:
                    cfg = json.loads(body)
                    max_pts = int(cfg.get('maxPoints', node.visual_max_points_per_topic))
                    range_m = float(cfg.get('range', node.visual_range_m))
                    node.visual_max_points_per_topic = max(100, min(40000, max_pts))
                    node.visual_range_m = max(10.0, min(500.0, range_m))
                    self.serve_json({'ok': True,
                                     'maxPoints': node.visual_max_points_per_topic,
                                     'range': node.visual_range_m})
                except Exception as exc:
                    self.send_error(HTTPStatus.BAD_REQUEST, str(exc))
            elif parsed.path == '/api/bag/start':
                topics = None
                try:
                    length = int(self.headers.get('Content-Length', 0))
                    if length > 0:
                        body = json.loads(self.rfile.read(length))
                        if not body.get('all', False):
                            topics = body.get('topics', [])
                except Exception:
                    topics = None
                self.serve_json(node.start_bag(topics))
            elif parsed.path == '/api/bag/stop':
                self.serve_json(node.stop_bag())
            elif parsed.path == '/api/bag/config':
                path = ''
                try:
                    length = int(self.headers.get('Content-Length', 0))
                    if length > 0:
                        body = json.loads(self.rfile.read(length))
                        path = body.get('dir', '')
                except Exception:
                    pass
                self.serve_json(node.set_bag_dir(path))
            elif parsed.path == '/api/fs/mkdir':
                path, name = '', ''
                try:
                    length = int(self.headers.get('Content-Length', 0))
                    if length > 0:
                        body = json.loads(self.rfile.read(length))
                        path = body.get('path', '')
                        name = body.get('name', '')
                except Exception:
                    pass
                self.serve_json(node.make_dir(path, name))
            elif parsed.path == '/api/bag/play/start':
                name, loop = '', False
                try:
                    length = int(self.headers.get('Content-Length', 0))
                    if length > 0:
                        body = json.loads(self.rfile.read(length))
                        name = body.get('name', '')
                        loop = bool(body.get('loop', False))
                except Exception:
                    pass
                self.serve_json(node.start_play(name, loop))
            elif parsed.path == '/api/bag/play/stop':
                self.serve_json(node.stop_play())
            elif parsed.path == '/api/bag/play/pause':
                self.serve_json(node.play_pause())
            elif parsed.path == '/api/bag/play/resume':
                self.serve_json(node.play_resume())
            elif parsed.path == '/api/bag/play/next':
                self.serve_json(node.play_next_frame())
            elif parsed.path == '/api/bag/play/seek':
                sec = None
                try:
                    length = int(self.headers.get('Content-Length', 0))
                    if length > 0:
                        body = json.loads(self.rfile.read(length))
                        sec = body.get('sec')
                except Exception:
                    pass
                self.serve_json(node.play_seek(sec))
            elif parsed.path == '/api/bag/play/rate':
                rate = None
                try:
                    length = int(self.headers.get('Content-Length', 0))
                    if length > 0:
                        body = json.loads(self.rfile.read(length))
                        rate = body.get('rate')
                except Exception:
                    pass
                self.serve_json(node.play_set_rate(rate))
            elif parsed.path == '/api/gps/track/start':
                self.serve_json(node.start_gps_track())
            elif parsed.path == '/api/gps/track/stop':
                self.serve_json(node.stop_gps_track())
            elif parsed.path == '/api/gps/track/delete':
                name = parse_qs(parsed.query).get('file', [''])[0]
                self.serve_json(node.delete_gps_track(name))
            elif parsed.path == '/api/system/shutdown':
                self.serve_json(node.system_power('shutdown'))
            elif parsed.path == '/api/system/reboot':
                self.serve_json(node.system_power('reboot'))
            else:
                self.send_error(HTTPStatus.NOT_FOUND, 'Not found')

    return MonitorRequestHandler


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    _last_err_log = 0.0
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except RuntimeError as exc:
                # 일부 커스텀 메시지 타입은 Python 변환 실패 → 해당 메시지만 스킵
                now = time.monotonic()
                if now - _last_err_log > 10.0:
                    node.get_logger().warn(f'메시지 역직렬화 실패 (무시): {exc}')
                    _last_err_log = now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
