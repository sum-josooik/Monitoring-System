import glob
import io
import json
import math
import mimetypes
import os
import re
import shutil
import signal
import socket
import struct
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

try:
    from road_perception import rectify as lens_rectify     # 렌즈 보정 (카메라 탭, 차선 범위 편집)
    from road_perception import lens_calib
except ImportError:
    lens_rectify = None
    lens_calib = None

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
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


def default_ui_settings_file(preferred_dir=None):
    # UI/ 폴더의 설정 백업 JSON 중 가장 최신(파일명 타임스탬프 기준). 백업 저장 폴더를 먼저 본다
    for d in (Path(preferred_dir) if preferred_dir else None, ui_settings_dir()):
        files = sorted(d.glob('*.json')) if d is not None and d.is_dir() else []
        if files:
            return files[-1]
    return None

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
        self.declare_parameter('stream_port_count', 3)
        self.declare_parameter('scan_period_sec', 2.0)
        self.declare_parameter('rate_window_sec', 5.0)
        self.declare_parameter('subscribe_all', True)
        self.declare_parameter('visual_max_points_per_topic', 900)
        self.declare_parameter('prune_after_sec', 10.0)
        self.declare_parameter('bag_dir', '/home/nvidia/robot_ws/bag')
        # 브라우저용 카메라 미리보기: 원본(1920x1536, ~1MB) 대신 줄인 JPEG 를 보냄
        # (GPU nvJPEG 로 디코딩·축소·인코딩, 실패 시 CPU). 0 = 끔(원본 그대로)
        # road_perception 차선 인식 범위 (설정 창 '도로 인지' 탭에서 편집, 노드가 파일 변경을 감지해 바로 적용)
        self.declare_parameter('lens_calibrator_port', 8090)
        self.declare_parameter('calibration_apply_file', lens_rectify.SETTINGS_PATH if lens_rectify else '')
        self.declare_parameter('intrinsics_dir', lens_rectify.INTRINSICS_DIR if lens_rectify else '')
        self.lens_port = int(self.get_parameter('lens_calibrator_port').value)
        self.lens_proc = None
        self.lens_lock = threading.Lock()
        self.declare_parameter('road_ipm_dir', '~/captin5_ws/src/road_perception/config/ipm')
        self.declare_parameter('ui_settings_dir', '~/captin5_ws/src/ros_web_monitor/UI')   # 설정 백업을 저장·불러오는 폴더
        self.declare_parameter('road_cameras_config', '')   # 비우면 road_perception 패키지의 config/cameras.yaml
        self.declare_parameter('preview_width', 960)
        self.declare_parameter('preview_fps', 12.0)
        self.declare_parameter('preview_quality', 75)

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.stream_port = int(self.get_parameter('stream_port').value)
        self.stream_port_count = max(1, int(self.get_parameter('stream_port_count').value))
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
        # raw Image 토픽 중 같은 이름의 /compressed 가 있는 것 → 구독하지 않고 compressed 수신률로 표시
        # (raw 를 구독하면 카메라 노드가 프레임마다 6MB 를 이 노드에도 보내야 함)
        self.mirrored_raw = {}
        self.preview = PreviewTranscoder(
            self, int(self.get_parameter('preview_width').value),
            float(self.get_parameter('preview_fps').value),
            int(self.get_parameter('preview_quality').value))
        # 왜곡을 편 카메라 영상 (?rect=1). 보고 있는 토픽만, 켠 뒤에만 변환한다. 확대·범위 편집용은 원본 해상도.
        self.lens_store = None
        self.rect_preview = self.rect_full = None
        if lens_rectify is not None:
            self.calibration_apply_file = os.path.expanduser(str(self.get_parameter('calibration_apply_file').value))
            self.lens_store = lens_rectify.LensStore(
                'camera', self.calibration_apply_file, os.path.expanduser(str(self.get_parameter('intrinsics_dir').value)),
                on_error=lambda text: self.get_logger().warn(text))
            self.rect_preview = PreviewTranscoder(
                self, int(self.get_parameter('preview_width').value), float(self.get_parameter('preview_fps').value),
                int(self.get_parameter('preview_quality').value), suffix=RECT_SUFFIX, lens_store=self.lens_store)
            self.rect_full = PreviewTranscoder(self, 1920, 6.0, 85, suffix=RECT_FULL_SUFFIX, lens_store=self.lens_store)
        self.topic_missing_since = {}
        self.visual_points = {}
        self.visual_boxes = {}
        self.visual_markers = {}
        self.visual_radar = {}   # RadarMsg 토픽 → 레이더 원값(dx 전방, dy 좌측) — 웹이 차량 설정의 장착 위치로 변환
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
        self.bag_gps_points = None    # Bag 녹화 중이면 list (GPS 경로 동시 기록)
        self.play_track = {'file': None, 'status': 'none', 'points': []}   # Play 중인 bag 의 GPS 경로
        self.visual_range_m = 120.0
        # multi_lidar_merge 입력 라이다별 변환 (topic → (R, t)), 자차 CropBox
        self.merge_poses, self.merge_crop, self.merge_output = load_merge_poses(self.get_logger())
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
        # 카메라 MJPEG 전용 보조 서버들: 브라우저는 호스트:포트당 동시연결을 6개로 제한하므로
        # 메인 포트와 분리하고, 포트를 여러 개(stream_port ~ stream_port+count-1) 열어
        # 브라우저가 슬롯별로 나눠 접속하게 한다 (포트 3개 → 카메라 최대 18대).
        self.stream_servers = []
        self.stream_ports = []
        if self.stream_port:
            for port in range(self.stream_port, self.stream_port + self.stream_port_count):
                if port == self.port:
                    continue
                try:
                    srv = ThreadingHTTPServer((self.host, port), handler)
                except OSError as exc:
                    self.get_logger().warn(f'스트림 포트 {port} 사용 불가 ({exc})')
                    continue
                threading.Thread(target=srv.serve_forever, daemon=True).start()
                self.stream_servers.append(srv)
                self.stream_ports.append(port)
        if not self.stream_ports:
            self.get_logger().warn('스트림 포트를 하나도 열지 못함 — 메인 포트로 스트리밍')

    def log_http_once(self):
        if not self._logged_http:
            self.get_logger().info(f'ROS web monitor listening on http://{self.host}:{self.port}')
            self._logged_http = True

    def destroy_node(self):
        self.lens_tool_stop()
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
        for srv in getattr(self, 'stream_servers', []):
            srv.shutdown()
            srv.server_close()
        super().destroy_node()

    def scan_topics(self):
        topics = self.get_topic_names_and_types()
        seen = set()
        all_names = {name for name, _ in topics}
        for name, type_names in topics:
            if not type_names:
                continue
            type_name = type_names[0]
            seen.add(name)
            with self.lock:
                self.stats.setdefault(name, TopicStats(name, type_name))
                self.stats[name].type_name = type_name
            if (type_name == 'sensor_msgs/msg/Image' and not is_streamable_raw_image_topic(name)
                    and name + '/compressed' in all_names):
                if name not in self.mirrored_raw:
                    self.mirrored_raw[name] = name + '/compressed'
                    sub = self.subscriptions_by_topic.pop(name, None)
                    if sub is not None:
                        try:
                            self.destroy_subscription(sub)
                        except Exception:
                            pass
                continue
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
        for suffix in (PREVIEW_SUFFIX, RECT_SUFFIX, RECT_FULL_SUFFIX):
            self.camera_frames.pop(name + suffix, None)
        self.mirrored_raw.pop(name, None)
        self.visual_points.pop(name, None)
        self.visual_boxes.pop(name, None)
        self.visual_markers.pop(name, None)
        self.visual_radar.pop(name, None)
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
            elif type_name in HEAVY_RAW_TYPES:
                # 화면에 쓰지 않는 대용량 메시지(raw 카메라 영상 ~6MB, 라이다 패킷)는 Python 객체로
                # 역직렬화하지 않고 바이트 그대로 받아 Hz/크기만 집계 (CPU·지연 대폭 감소)
                callback = lambda data, topic=name, tn=type_name: self.handle_serialized(topic, data, tn)
            elif type_name == 'sensor_msgs/msg/PointCloud2':
                callback = lambda msg, topic=name: self.handle_pointcloud(topic, msg)
            elif type_name == 'visualization_msgs/msg/MarkerArray':
                callback = lambda msg, topic=name: self.handle_marker_array(topic, msg)
            elif type_name == 'visualization_msgs/msg/Marker':
                callback = lambda msg, topic=name: self.handle_marker(topic, msg)
            elif type_name == RADAR_MSG_TYPE:
                callback = lambda msg, topic=name: self.handle_radar(topic, msg)
            elif type_name == 'vision_msgs/msg/Detection3DArray':
                callback = lambda msg, topic=name: self.handle_detection_array(topic, msg)
            elif type_name in ('sensor_msgs/msg/NavSatFix', 'gps_msgs/msg/GPSFix') or \
                    'NavSatFix' in type_name or 'GPSFix' in type_name:
                callback = lambda msg, topic=name: self.handle_gps(topic, msg)
            else:
                callback = lambda msg, topic=name, tn=type_name: self.handle_generic(topic, msg, tn)
            raw = type_name in HEAVY_RAW_TYPES and not (
                type_name == 'sensor_msgs/msg/Image' and is_streamable_raw_image_topic(name))
            sub = self.create_subscription(msg_type, name, callback, qos, raw=raw)
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

    def handle_serialized(self, topic, data, type_name):
        with self.lock:
            stat = self.stats.get(topic)
            if not stat:
                return
            stat.record()
            if type_name == 'sensor_msgs/msg/Image':
                stat.last_value = {**parse_image_header(data), 'bytes': len(data)}
            else:
                stat.last_value = {'bytes': len(data)}

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
                'interval': self._frame_interval(topic),
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
                'interval': self._frame_interval(topic),
            }
        condition = self.camera_conditions[topic]
        with condition:
            condition.notify_all()

    def _frame_interval(self, topic):
        """카메라 프레임 간격 지수평균(초). self.lock 안에서 호출. 느린 인식 결과 영상의 끊김 판정용."""
        prev = self.camera_frames.get(topic)
        if not prev:
            return None
        dt = _now() - prev['stamp']
        old = prev.get('interval')
        return dt if old is None else old * 0.7 + dt * 0.3

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
            # numpy 벡터 연산으로 다운샘플 (Python 루프 대비 수십 배 빠름)
            arr = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32) if len(arr) else np.zeros((0, 3), np.float32)
            step = max(1, len(xyz) // max(1, self.visual_max_points_per_topic))
            xyz = xyz[::step]
            # 병합 노드 입력 라이다면 같은 변환을 적용해 병합 좌표계로 옮김 (병합 클라우드가 없을 때 대체 표시)
            base = topic[len(self.play_namespace):] if topic.startswith(self.play_namespace + '/') else topic
            merge_part = base in self.merge_poses
            if merge_part:
                rot, trans = self.merge_poses[base]
                xyz = xyz @ rot.T + trans
                lo, hi = self.merge_crop
                if lo is not None:
                    inside = np.all((xyz >= lo) & (xyz <= hi), axis=1)
                    xyz = xyz[~inside]        # 자차 영역 제거 (병합 노드 remove_vehicle 과 동일)
            rng = self.visual_range_m
            keep = (np.abs(xyz[:, 0]) <= rng) & (np.abs(xyz[:, 1]) <= rng) & (xyz[:, 2] >= -6.0) & (xyz[:, 2] <= 8.0)
            points = np.round(xyz[keep][:self.visual_max_points_per_topic], 2).tolist()
            with self.lock:
                self.visual_points[topic] = {'stamp': _now(), 'topic': topic, 'points': points,
                                             'merge_part': merge_part}
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

    def handle_radar(self, topic, msg):
        """can_parser RadarMsg(Bosch 레이더 40개 객체) → 유효 객체 [dx, dy, vx, vy, moving, id]."""
        with self.lock:
            stat = self.stats.get(topic)
            if stat:
                stat.record()
        side = radar_side(topic)
        if side is None or not self.should_process_visual(topic):
            return
        objects = []
        for a, b in zip(msg.data_a, msg.data_b):
            dx, dy = float(a.dx), float(a.dy)
            if not (math.isfinite(dx) and math.isfinite(dy)) or dx >= 200.0 or abs(dy) >= 200.0:
                continue   # 센티넬(빈 슬롯)
            if abs(dx) < 1e-3 and abs(dy) < 1e-3:
                continue
            objects.append([round(dx, 2), round(dy, 2), round(float(a.vx), 2), round(float(a.vy), 2),
                            1 if b.moving else 0, int(a.id)])
        with self.lock:
            if stat:
                stat.last_value = {'objects': len(objects)}
            self.visual_radar[topic] = {'topic': topic, 'side': side, 'stamp': _now(), 'objects': objects}

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
                'id': str(getattr(detection, 'id', '') or index),
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
                    # GPSFix 는 속도(m/s)·진행방향(북 기준 시계방향 deg)을 함께 제공
                    for key in ('speed', 'track'):
                        val = getattr(msg, key, None)
                        if isinstance(val, (int, float)) and math.isfinite(val):
                            record[key] = round(float(val), 3)
                    # bag 재생 네임스페이스는 실시간 GPS와 분리된 슬롯에 저장
                    # (재생 중에도 실제 차량 GPS를 덮어쓰지 않도록)
                    if topic.startswith(self.play_namespace + '/'):
                        self.gps_data_play = record
                    else:
                        self.gps_data = record
                        point = [round(time.time(), 2), round(lat, 7), round(lng, 7), round(alt, 2)]
                        if self.gps_track_recording:
                            # 저장 파일에는 절대 시각이 유용하므로 epoch 초 사용 (_now는 monotonic)
                            self.gps_track_points.append(point)
                        if self.bag_gps_points is not None:
                            self.bag_gps_points.append(point)   # Bag 녹화와 함께 기록
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
        with self.play_lock:
            self.play_track = {'file': name, 'status': 'loading', 'points': []}
        threading.Thread(target=self.load_play_track, args=(name, bag_dir), daemon=True).start()
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
            self.play_track = {'file': None, 'status': 'none', 'points': []}
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
            radar = [
                {'topic': r['topic'], 'side': r['side'], 'objects': r['objects'], 'age_sec': round(now - r['stamp'], 2)}
                for r in self.visual_radar.values()
                if now - r['stamp'] < 2.0
            ]
        point_layers.sort(key=lambda item: item['topic'])
        return {
            'range_m': int(self.visual_range_m),
            'point_layers': point_layers[:24],
            'merge_output': self.merge_output,
            'boxes': box_items[:420],
            'markers': marker_items[:420],
            'radar': radar,
        }

    def snapshot(self):
        with self.lock:
            topics = []
            for stat in self.stats.values():
                item = stat.snapshot(self.rate_window_sec)
                src = self.mirrored_raw.get(stat.name)
                if src and src in self.stats:
                    # 구독하지 않은 raw 토픽: 같은 카메라 /compressed 의 수신률·상태로 표시
                    m = self.stats[src].snapshot(self.rate_window_sec)
                    item.update({k: m[k] for k in ('rate_hz', 'age_sec', 'active', 'total')})
                    item['error'] = ''
                    item['last_value'] = {'note': '미구독(대용량) — /compressed 수신 기준'}
                topics.append(item)
            cameras = []
            for name, frame in self.camera_frames.items():
                if is_preview_topic(name) or name.endswith((PREVIEW_SUFFIX, RECT_SUFFIX, RECT_FULL_SUFFIX)):
                    continue   # 미리보기는 원본 카메라 스트림에 자동 대체되므로 목록에서 제외
                cameras.append({
                    'name': name,
                    'age_sec': round(_now() - frame['stamp'], 2),
                    'interval_sec': round(frame.get('interval') or 0.0, 2),
                    'stream_url': '/stream?topic=' + name,
                })
        topics.sort(key=lambda item: item['name'])
        cameras.sort(key=lambda item: item['name'])
        groups = group_topics(topics)
        active = sum(1 for item in topics if item['active'])
        return {
            'time': time.strftime('%Y-%m-%d %H:%M:%S'),
            'stream_port': self.stream_ports[0] if getattr(self, 'stream_ports', None) else None,
            'stream_ports': list(getattr(self, 'stream_ports', [])),
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
                'bag_recording': self.bag_gps_points is not None,
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

    # ── road_perception 차선 인식 범위 ──
    def _road_config(self):
        path = str(self.get_parameter('road_cameras_config').value or '')
        if not path:
            path = os.path.join(get_package_share_directory('road_perception'), 'config', 'cameras.yaml')
        with open(os.path.expanduser(path)) as f:
            cc = yaml.safe_load(f) or {}
        cams = {n: c for n, c in (cc.get('cameras') or {}).items()
                if (c or {}).get('enabled', True) and 'lanes' in (c or {}).get('tasks', [])}
        return cc, cams

    def _road_ipm_path(self, camera):
        return os.path.join(os.path.expanduser(str(self.get_parameter('road_ipm_dir').value)), f'{camera}.yaml')

    def road_ipm_state(self):
        """카메라별 현재 차선 인식 범위. corners = 범위 사각형의 영상 위 위치(0~1) — 가까운 왼쪽, 가까운 오른쪽, 먼 오른쪽, 먼 왼쪽."""
        try:
            cc, cams = self._road_config()
        except Exception as exc:
            return {'ok': False, 'error': f'road_perception 설정을 읽지 못함: {exc}'}
        out = {}
        for name, c in cams.items():
            cfg = {**(cc.get('ipm_defaults') or {}), **(c.get('ipm') or {})}
            path = self._road_ipm_path(name)
            custom = os.path.isfile(path)
            if custom:
                try:
                    with open(path) as f:
                        cfg.update(yaml.safe_load(f) or {})
                except Exception as exc:
                    self.get_logger().warn(f'{path} 읽기 실패: {exc}')
                    custom = False
            x0, x1 = [float(v) for v in cfg.get('x_range', [2.0, 20.0])]
            y0, y1 = [float(v) for v in cfg.get('y_range', [-6.0, 6.0])]
            lens = self._road_lens(name)
            # 파일은 원본 기준, 편집 화면은 편 영상 기준
            src = np.float32(lens_rectify.rect_quad(cfg['src'], cfg['dst'], lens)) if lens is not None else np.float32(cfg['src'])
            H = cv2.getPerspectiveTransform(np.float32(cfg['dst']), src)
            corners = []
            # w 부호는 행렬마다 다르므로 기준점(dst 첫 점)과 비교. 부호가 다르면 지평선 너머 → 사다리꼴 점으로 대체
            ref = np.sign((H @ np.array([*cfg['dst'][0], 1.0]))[2]) or 1.0
            for i, (gx, gy) in enumerate(((x0, y1), (x0, y0), (x1, y0), (x1, y1))):
                u, v, w = H @ np.array([gx, gy, 1.0])
                pt = [u / w, v / w] if w * ref > 1e-6 else [float(src[i][0]), float(src[i][1])]
                corners.append([round(float(min(1.15, max(-0.15, q))), 4) for q in pt])   # 편집 화면 여백까지만
            out[name] = {'corners': corners, 'near': round(x0, 2), 'far': round(x1, 2), 'width': round(y1 - y0, 2),
                         'offset': round((y0 + y1) / 2, 2), 'calibrated': bool(cfg.get('calibrated', False)), 'custom': custom,
                         'rect': lens is not None}
        return {'ok': True, 'cameras': out}

    def _road_lens(self, camera):
        """도로 인지 탭에 보정이 켜져 있고 렌즈 값이 있으면 Lens. 인지 노드와 같은 판단."""
        if self.lens_store is None or not lens_rectify.read_settings(self.calibration_apply_file).get('road', True):
            return None
        return self.lens_store.load(camera)

    def calibration_apply_state(self):
        if self.lens_store is None:
            return {'ok': False, 'error': 'road_perception 패키지를 찾지 못했습니다'}
        settings = lens_rectify.read_settings(self.calibration_apply_file)
        return {'ok': True, 'settings': settings,
                'cameras': {c: self.lens_store.load(c) is not None for c in lens_rectify.CAMERAS}}

    def calibration_apply_save(self, body):
        if self.lens_store is None:
            return {'ok': False, 'error': 'road_perception 패키지를 찾지 못했습니다'}
        if not isinstance(body, dict) or not all(isinstance(body[k], bool) for k in body):
            return {'ok': False, 'error': '잘못된 값'}
        try:
            lens_rectify.write_settings(body, self.calibration_apply_file)
        except OSError as exc:
            return {'ok': False, 'error': f'저장 실패: {exc}'}
        return self.calibration_apply_state()

    def road_ipm_save(self, body):
        try:
            _, cams = self._road_config()
            camera = str(body.get('camera', ''))
            if camera not in cams:
                return {'ok': False, 'error': f'차선 인식 카메라가 아님: {camera}'}
            src = [[float(a), float(b)] for a, b in body['corners']]
            near, far, width = float(body['near']), float(body['far']), float(body['width'])
            offset = float(body.get('offset', 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            return {'ok': False, 'error': f'잘못된 값: {exc}'}
        if len(src) != 4 or not all(math.isfinite(q) and -0.5 <= q <= 1.5 for pt in src for q in pt):
            return {'ok': False, 'error': '꼭짓점 4개가 영상 범위를 벗어남'}
        if not (0.0 <= near < far <= 120.0 and 0.5 <= width <= 60.0 and abs(offset) <= 30.0):
            return {'ok': False, 'error': '거리 또는 폭이 허용 범위를 벗어남 (0 ≤ 가까운 거리 < 먼 거리 ≤ 120 m, 폭 0.5~60 m)'}
        cross = [(src[(i + 1) % 4][0] - src[i][0]) * (src[(i + 2) % 4][1] - src[(i + 1) % 4][1])
                 - (src[(i + 1) % 4][1] - src[i][1]) * (src[(i + 2) % 4][0] - src[(i + 1) % 4][0]) for i in range(4)]
        if not (all(c > 1e-5 for c in cross) or all(c < -1e-5 for c in cross)):
            return {'ok': False, 'error': '꼭짓점이 꼬였거나 일직선임 — 볼록한 사각형이어야 함'}
        if body.get('rect'):    # 편집 화면은 편 영상 기준 → 파일에는 항상 원본 기준으로 저장
            lens = self.lens_store.load(camera) if self.lens_store is not None else None
            if lens is None:
                return {'ok': False, 'error': f'{camera} 보정값이 없어 저장할 수 없습니다. 도로 인지 탭을 다시 여세요'}
            W, H_ = lens.size
            src = (lens.rect_to_raw(np.array(src) * [W, H_]) / [W, H_]).tolist()
        hw = width / 2
        cfg = {
            'calibrated': bool(body.get('calibrated', False)),
            'src_normalized': True,
            'src': src,
            'dst': [[near, offset + hw], [near, offset - hw], [far, offset - hw], [far, offset + hw]],
            'x_range': [near, far],
            'y_range': [offset - hw, offset + hw],
            'resolution': round(max(0.05, max(far - near, width) / 800.0), 4),   # BEV 크기 제한
            'note': time.strftime('%Y-%m-%d %H:%M') + ' 웹 설정',
        }
        path = self._road_ipm_path(camera)
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            if os.path.isfile(path):                  # 어안 보정값은 유지
                with open(path) as f:
                    old = yaml.safe_load(f) or {}
                for k in ('camera_matrix', 'dist_coeffs', 'fisheye'):
                    if k in old:
                        cfg[k] = old[k]
            fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), suffix='.tmp')
            with os.fdopen(fd, 'w') as f:
                yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)
            os.replace(tmp, path)
        except OSError as exc:
            return {'ok': False, 'error': f'저장 실패: {exc}'}
        return self.road_ipm_state()

    def road_ipm_reset(self, camera):
        try:
            _, cams = self._road_config()
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}
        if camera not in cams:
            return {'ok': False, 'error': f'차선 인식 카메라가 아님: {camera}'}
        path = self._road_ipm_path(camera)
        try:
            if os.path.isfile(path):
                os.remove(path)
        except OSError as exc:
            return {'ok': False, 'error': f'삭제 실패: {exc}'}
        return self.road_ipm_state()

    # ── 설정 백업: 브라우저가 아니라 서버 파일에 저장되는 값 ──
    @staticmethod
    def _read_text(path):
        try:
            with open(path, encoding='utf-8') as f:
                return f.read()
        except OSError:
            return None

    @staticmethod
    def _write_text(path, text):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), suffix='.tmp')
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(text)
        os.chmod(tmp, 0o644)
        os.replace(tmp, path)

    def backup_folder(self):
        return os.path.expanduser(str(self.get_parameter('ui_settings_dir').value))

    def ui_backup_save(self, body):
        if not isinstance(body, dict) or not isinstance(body.get('settings'), dict):
            return {'ok': False, 'error': '잘못된 형식'}
        folder = self.backup_folder()
        name = time.strftime('ros_web_monitor_settings_%Y-%m-%d-%H-%M-%S.json')
        path = os.path.join(folder, name)
        if os.path.exists(path):
            return {'ok': False, 'error': '같은 이름의 파일이 있습니다. 잠시 후 다시 시도하세요'}
        try:
            self._write_text(path, json.dumps(body, indent=2, ensure_ascii=False))
        except OSError as exc:
            return {'ok': False, 'error': f'저장 실패: {exc}'}
        return {'ok': True, 'name': name, 'path': path}

    def ui_backup_list(self):
        folder = self.backup_folder()
        items = []
        for path in sorted(glob.glob(os.path.join(folder, '*.json')), reverse=True):
            try:
                with open(path, encoding='utf-8') as f:
                    doc = json.load(f)
                stat = os.stat(path)
            except (OSError, ValueError):
                continue
            settings = doc.get('settings', doc) if isinstance(doc, dict) else None
            if not isinstance(settings, dict):
                continue
            items.append({'name': os.path.basename(path), 'size_kb': round(stat.st_size / 1024, 1),
                          'modified': time.strftime('%Y-%m-%d %H:%M', time.localtime(stat.st_mtime)),
                          'items': len(settings), 'server': isinstance(doc.get('server'), dict)})
        return {'ok': True, 'folder': folder, 'files': items}

    def ui_backup_path(self, name):
        """이름 검증 후 백업 폴더 안의 파일 경로. 없거나 잘못되면 None."""
        if not re.fullmatch(r'[\w][\w.-]*\.json', name or ''):
            return None
        path = os.path.join(self.backup_folder(), name)
        return path if os.path.isfile(path) else None

    def settings_backup(self):
        out = {'ok': True, 'bag_dir': self.bag_dir}
        if self.lens_store is None or lens_calib is None:
            return out
        out['calibration_apply'] = lens_rectify.read_settings(self.calibration_apply_file)
        directory = self.lens_store.dir
        out['intrinsics'] = {c: self._read_text(lens_calib.active_path(directory, c)) for c in lens_rectify.CAMERAS}
        library = {}
        for camera in lens_rectify.CAMERAS:
            paths = sorted(glob.glob(os.path.join(directory, 'library', camera, '*.yaml')))
            items = {os.path.basename(p)[:-5]: self._read_text(p) for p in paths}
            if items:
                library[camera] = items
        out['intrinsics_library'] = library
        try:
            _, cams = self._road_config()
            out['road_ipm'] = {c: self._read_text(self._road_ipm_path(c)) for c in cams}
        except Exception as exc:
            self.get_logger().warn(f'차선 범위를 백업하지 못함: {exc}')
        return out

    @staticmethod
    def _parse_yaml_doc(text, required, label):
        try:
            doc = yaml.safe_load(text)
        except yaml.YAMLError as exc:
            raise ValueError(f'{label}: YAML 오류') from exc
        missing = [k for k in required if not isinstance(doc, dict) or k not in doc]
        if missing:
            raise ValueError(f'{label}: 항목 없음 ({", ".join(missing)})')

    def settings_restore(self, body):
        """백업의 server 부분을 복원. 전부 검사한 뒤에 쓴다. 보관 폴더(library)는 없는 이름만 추가."""
        if not isinstance(body, dict):
            return {'ok': False, 'error': '잘못된 형식'}
        result = {'ok': True, 'skipped': [], 'warnings': [], 'auto_backups': [],
                  'road_ipm': 0, 'intrinsics': 0, 'library_added': 0, 'calibration_apply': False}
        writes, removes, library, active_cameras = [], [], [], []
        has_files = any(k in body for k in ('calibration_apply', 'intrinsics', 'intrinsics_library', 'road_ipm'))
        if has_files and (self.lens_store is None or lens_calib is None):
            return {'ok': False, 'error': 'road_perception 패키지를 찾지 못했습니다'}

        def changes(path, text):
            return self._read_text(path) != text

        try:
            apply_values = body.get('calibration_apply')
            if apply_values is not None and not (isinstance(apply_values, dict)
                                                 and all(isinstance(v, bool) for v in apply_values.values())):
                raise ValueError('탭별 보정 적용 값이 잘못됨')
            for section in ('intrinsics', 'road_ipm', 'intrinsics_library'):
                if not isinstance(body.get(section, {}), dict):
                    raise ValueError(f'{section} 형식이 잘못됨')

            for camera, text in body.get('intrinsics', {}).items():
                if camera not in lens_rectify.CAMERAS:
                    result['skipped'].append(camera)
                    continue
                path = lens_calib.active_path(self.lens_store.dir, camera)
                if text is None:
                    if os.path.isfile(path):
                        active_cameras.append(camera)
                        removes.append(path)
                        result['intrinsics'] += 1
                    continue
                if not isinstance(text, str):
                    raise ValueError(f'{camera}: 보정값 형식이 잘못됨')
                self._parse_yaml_doc(text, lens_calib.LIBRARY_KEYS, camera)
                if changes(path, text):
                    active_cameras.append(camera)
                    writes.append((path, text))
                    result['intrinsics'] += 1

            for camera, items in body.get('intrinsics_library', {}).items():
                if camera not in lens_rectify.CAMERAS:
                    result['skipped'].append(camera)
                    continue
                if not isinstance(items, dict):
                    raise ValueError(f'{camera}: 보관 폴더 형식이 잘못됨')
                for name, text in items.items():
                    if not lens_calib.NAME_RE.match(str(name)) or not isinstance(text, str):
                        raise ValueError(f'{camera}: 보관 이름이 잘못됨')
                    self._parse_yaml_doc(text, lens_calib.LIBRARY_KEYS, f'{camera}/{name}')
                    path = lens_calib.library_path(self.lens_store.dir, camera, name)
                    if not os.path.exists(path):
                        library.append((path, text))

            if 'road_ipm' in body:
                _, cams = self._road_config()
                for camera, text in body['road_ipm'].items():
                    if camera not in cams:
                        result['skipped'].append(camera)
                        continue
                    path = self._road_ipm_path(camera)
                    if text is None:
                        if os.path.isfile(path):
                            removes.append(path)
                            result['road_ipm'] += 1
                        continue
                    if not isinstance(text, str):
                        raise ValueError(f'{camera}: 차선 범위 형식이 잘못됨')
                    self._parse_yaml_doc(text, ('src', 'dst', 'x_range', 'y_range'), f'{camera} 차선 범위')
                    if changes(path, text):
                        writes.append((path, text))
                        result['road_ipm'] += 1
        except Exception as exc:
            return {'ok': False, 'error': f'백업 내용 오류, 아무것도 바꾸지 않았습니다. {exc}'}

        try:
            for camera in active_cameras:   # 덮어쓰기 전에 현재 보정값을 보관 폴더에 남긴다
                saved = lens_calib.backup_active(self.lens_store.dir, camera)
                if saved:
                    result['auto_backups'].append(f'{camera}/{saved}')
            for path, text in writes + library:
                self._write_text(path, text)
            for path in removes:
                os.remove(path)
            result['library_added'] = len(library)
            if apply_values is not None:
                lens_rectify.write_settings(apply_values, self.calibration_apply_file)
                result['calibration_apply'] = True
        except (OSError, ValueError, yaml.YAMLError) as exc:
            return {'ok': False, 'error': f'복원 중 실패, 일부만 복원됐을 수 있습니다. {exc}'}

        if isinstance(body.get('bag_dir'), str) and body['bag_dir'] != self.bag_dir:
            bag = self.set_bag_dir(body['bag_dir'])
            if bag.get('ok'):
                result['bag_dir'] = bag['dir']
            else:
                result['warnings'].append(f"Bag 저장 경로: {bag.get('error')}")
        return result

    # ── 렌즈 보정 도구 (설정 탭을 여는 동안만 별도 프로세스로 실행) ──
    def _lens_port_open(self):
        try:
            with socket.create_connection(('127.0.0.1', self.lens_port), timeout=0.3):
                return True
        except OSError:
            return False

    def _lens_log_path(self):
        return os.path.expanduser('~/.ros/lens_calibrator.log')

    def lens_tool_state(self):
        proc = self.lens_proc
        out = {'ok': True, 'running': self._lens_port_open(), 'port': self.lens_port,
               'owned': proc is not None and proc.poll() is None}
        if proc is not None and proc.poll() is not None:
            try:
                with open(self._lens_log_path(), errors='replace') as f:
                    tail = f.read().strip().splitlines()[-1:]
            except OSError:
                tail = []
            out['ok'] = False
            out['error'] = '보정 도구가 시작 직후 종료됐습니다' + (f': {tail[0][:200]}' if tail else '')
        return out

    def lens_tool_start(self):
        with self.lens_lock:
            if (self.lens_proc is None or self.lens_proc.poll() is not None) and not self._lens_port_open():
                try:
                    os.makedirs(os.path.dirname(self._lens_log_path()), exist_ok=True)
                    with open(self._lens_log_path(), 'w') as log:
                        self.lens_proc = subprocess.Popen(
                            [_sys.executable, '-m', 'road_perception.lens_calibrator', '--port', str(self.lens_port)],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                except OSError as exc:
                    return {'ok': False, 'error': f'보정 도구 실행 실패: {exc}'}
        return self.lens_tool_state()

    def lens_tool_stop(self):
        """이 모니터가 켠 프로세스만 끈다. 터미널에서 따로 켠 도구는 건드리지 않는다."""
        with self.lens_lock:
            proc, self.lens_proc = self.lens_proc, None
            if proc is None or proc.poll() is not None:
                return self.lens_tool_state()
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=2)
            except ProcessLookupError:
                pass
        return self.lens_tool_state()

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

    # 전체 녹화 시 기본 제외: raw 카메라(compressed 있음, AVM 은 raw 뿐이라 유지), 라이다 원본 패킷,
    # Ouster 2D 이미지, 클러스터 디버그 점, 재생 중인 /bagplay 토픽.
    # 2026-11-04 bag(34.9GB/225s)은 raw 영상 때문에 녹화가 밀려 카메라 1~4초 공백, 병합 라이다 7개만 기록됨.
    BAG_EXCLUDE_ALWAYS = r'^/bagplay/.*'
    BAG_EXCLUDE_HEAVY = (r'^/(?!avm_cam/)[^/]+/image_raw$|^.*/velodyne_packets$|'
                         r'^.*Ouster/(nearir|range|reflec|signal)_image$|^/cluster_debug_points$')

    def start_bag(self, topics=None, light=True):
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
                if sel:
                    cmd += sel
                else:
                    exclude = self.BAG_EXCLUDE_ALWAYS + ('|' + self.BAG_EXCLUDE_HEAVY if light else '')
                    cmd += ['-a', '--exclude-regex', exclude]
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
                with self.lock:
                    self.bag_gps_points = []
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
            with self.lock:
                gps_points, self.bag_gps_points = self.bag_gps_points or [], None
            if saved and gps_points:
                self.save_bag_gps_track(saved, gps_points)
            self.bag_tmp = None
            self.bag_start_time = None
            self.bag_last_saved = saved
            self.get_logger().info(f'bag 녹화 종료, 저장: {saved} (mcap {mcap_count}개)')
            return {'ok': True, 'recording': False, 'path': saved, 'mcap_count': mcap_count}

    def save_bag_gps_track(self, bag_dir, points, source='live'):
        """bag 폴더에 gps_track.json 저장 + GPS 경로 목록(log/gps)에도 복사."""
        payload = {
            'bag': os.path.basename(bag_dir),
            'source': source,
            'started': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(points[0][0])),
            'ended': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(points[-1][0])),
            'point_count': len(points),
            'points': points,   # [unix_stamp, lat, lng, alt]
        }
        text = json.dumps(payload)
        try:
            (Path(bag_dir) / 'gps_track.json').write_text(text, encoding='utf-8')
        except Exception as exc:
            self.get_logger().warn(f'bag GPS 경로 저장 실패: {exc}')
        try:
            self.gps_track_dir.mkdir(parents=True, exist_ok=True)
            (self.gps_track_dir / f'track_bag_{os.path.basename(bag_dir)}.json').write_text(text, encoding='utf-8')
        except Exception:
            pass

    def load_play_track(self, name, bag_dir):
        """재생 bag 의 GPS 경로: gps_track.json 이 있으면 사용, 없으면 bag 의 GPS 토픽에서 추출 후 캐시."""
        def done(status, points, source=''):
            with self.play_lock:
                if self.play_track.get('file') == name:
                    self.play_track = {'file': name, 'status': status, 'points': points, 'source': source}

        cache = Path(bag_dir) / 'gps_track.json'
        try:
            if cache.exists():
                data = json.loads(cache.read_text(encoding='utf-8'))
                done('ready', data.get('points', []), data.get('source', 'file'))
                return
        except Exception:
            pass
        try:
            import rosbag2_py
            meta = yaml.safe_load((Path(bag_dir) / 'metadata.yaml').read_text())
            info = (meta or {}).get('rosbag2_bagfile_information', {}) or {}
            storage = info.get('storage_identifier') or 'mcap'
            gps_topics = {}
            for t in info.get('topics_with_message_count', []) or []:
                tm = (t or {}).get('topic_metadata', {}) or {}
                if ('GPSFix' in tm.get('type', '') or 'NavSatFix' in tm.get('type', '')) and t.get('message_count', 0):
                    gps_topics[tm['name']] = tm['type']
            if not gps_topics:
                done('none', [])
                return
            topic = '/gpsfix' if '/gpsfix' in gps_topics else sorted(gps_topics)[0]
            msg_type = get_message(gps_topics[topic])
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage),
                        rosbag2_py.ConverterOptions('cdr', 'cdr'))
            reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
            points = []
            while reader.has_next():
                _, data, stamp = reader.read_next()
                msg = deserialize_message(data, msg_type)
                lat, lng = float(msg.latitude), float(msg.longitude)
                if math.isfinite(lat) and math.isfinite(lng) and abs(lat) <= 90 and abs(lng) <= 180 and (lat or lng):
                    points.append([round(stamp / 1e9, 2), round(lat, 7), round(lng, 7), round(float(msg.altitude), 2)])
            done('ready' if points else 'none', points, f'bag:{topic}')
            if points:
                self.save_bag_gps_track(str(bag_dir), points, source=f'bag:{topic}')
        except Exception as exc:
            self.get_logger().warn(f'bag GPS 경로 추출 실패: {exc}')
            done('error', [])

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

def load_merge_poses(logger=None):
    """multi_lidar_merge.yaml 에서 입력 토픽별 장착 변환(R=Rz·Ry·Rx, t)과 remove_vehicle 박스를 읽는다."""
    poses, crop, output = {}, (None, None), ''
    try:
        share = Path(get_package_share_directory('multi_lidar_merge')) if get_package_share_directory else None
        path = share / 'config' / 'multi_lidar_merge.yaml' if share else None
        if not path or not path.exists():
            return poses, crop, output
        params = (yaml.safe_load(path.read_text()) or {}).get('multi_lidar_merge', {}).get('ros__parameters', {})
        scale = math.pi / 180.0 if params.get('angles_in_degrees') else 1.0
        output = str(params.get('output_topic', '') or '')
        lidars = params.get('lidars', {}) or {}
        for topic, name in zip(params.get('input_topics', []) or [], params.get('lidar_names', []) or []):
            pose = lidars.get(name) or {}
            if not pose.get('enabled', True):
                continue
            r, p, y = (float(pose.get(k, 0.0)) * scale for k in ('roll', 'pitch', 'yaw'))
            cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
            rot = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                            [-sp, cp * sr, cp * cr]], np.float32)
            trans = np.array([float(pose.get(k, 0.0)) for k in ('x', 'y', 'z')], np.float32)
            poses[topic] = (rot, trans)
        rv = params.get('remove_vehicle', {}) or {}
        if rv.get('enabled', True) and rv.get('min') and rv.get('max'):
            crop = (np.array(rv['min'], np.float32), np.array(rv['max'], np.float32))
    except Exception as exc:
        if logger:
            logger.warn(f'multi_lidar_merge 설정 읽기 실패: {exc}')
    return poses, crop, output


# 역직렬화 없이(raw=True) 구독하는 대용량 타입
HEAVY_RAW_TYPES = {
    'sensor_msgs/msg/Image',
    'velodyne_msgs/msg/VelodyneScan',
    'ouster_sensor_msgs/msg/PacketMsg',
}


def parse_image_header(data):
    """직렬화된(CDR) sensor_msgs/Image 앞부분에서 width/height/encoding 만 읽는다."""
    try:
        le = data[1] == 1                      # 캡슐화 헤더: 0x0001 = little endian
        order = '<' if le else '>'
        off = 4 + 8                            # 캡슐화(4) + stamp(sec, nanosec)
        (n,) = struct.unpack_from(order + 'I', data, off)
        off += 4 + n                           # frame_id 문자열(널 포함)
        off = 4 + ((off - 4 + 3) & ~3)         # 4바이트 정렬 (캡슐화 이후 기준)
        height, width = struct.unpack_from(order + 'II', data, off)
        off += 8
        (n,) = struct.unpack_from(order + 'I', data, off)
        enc = bytes(data[off + 4:off + 4 + max(0, n - 1)]).decode('ascii', 'replace')
        return {'width': int(width), 'height': int(height), 'encoding': enc}
    except Exception:
        return {}


PREVIEW_SUFFIX = '@preview'
SERVER_RESTORE_MAX_BYTES = 16 * 1024 * 1024
RECT_SUFFIX = '@rect'
RECT_FULL_SUFFIX = '@rectfull'


class PreviewTranscoder:
    """
    브라우저로 보내는 카메라 MJPEG 를 줄인 JPEG 로 바꿔 둔다 (camera_frames[topic + '@preview']).
    원본 1920x1536 JPEG(~1MB)를 카메라 8대 x 10Hz 로 보내면 브라우저가 디코딩에만 CPU 2코어 이상을 씀.
    스트림 클라이언트가 보고 있는 토픽만, preview_fps 이하로, 여러 토픽을 한 번에(배치) 처리.
    GPU(torchvision nvJPEG: 디코딩 → 축소 → 인코딩)가 기본이고 안 되면 CPU(축소 디코딩)로.
    """

    def __init__(self, node, width, fps, quality, suffix=PREVIEW_SUFFIX, lens_store=None):
        self.node = node
        self.suffix = suffix
        self.lens_store = lens_store    # 있으면 왜곡을 펴서 낸다. 그 카메라의 보정값이 없으면 그대로
        self.width = max(0, int(width))
        self.period = 1.0 / max(1.0, float(fps))
        self.quality = max(30, min(95, int(quality)))
        self.demand = {}        # topic -> 마지막 요청 시각
        self.last_src = {}      # topic -> 마지막으로 변환한 원본 stamp
        self.last_out = {}      # topic -> 마지막 출력 시각
        self.backend = None
        self.event = threading.Event()
        if self.width:
            threading.Thread(target=self.run, daemon=True, name='preview-transcoder').start()

    def handles(self, topic):
        return bool(self.width) and topic.endswith('/image_raw/compressed')

    def _lens(self, topic):
        if self.lens_store is None:
            return None
        parts = [p for p in topic.split('/') if p]
        return self.lens_store.load(parts[0]) if parts else None

    def want(self, topic):
        self.demand[topic] = _now()
        self.event.set()

    def _init_backend(self):
        try:
            import torch
            import torch.nn.functional as F
            from torchvision.io import decode_jpeg, encode_jpeg
            if not torch.cuda.is_available():
                raise RuntimeError('CUDA 없음')
            self._torch, self._F, self._dec, self._enc = torch, F, decode_jpeg, encode_jpeg
            self.backend = 'gpu'
        except Exception as exc:
            self.node.get_logger().warn(f'미리보기 GPU 변환 사용 불가({exc}) → CPU')
            self.backend = 'cpu'
        self.node.get_logger().info(f'카메라 미리보기 변환: {self.width}px, {1.0 / self.period:.0f}Hz, {self.backend}')

    def _gpu(self, topics, datas):
        torch, F = self._torch, self._F
        with torch.inference_mode():
            imgs = self._dec([torch.frombuffer(bytearray(d), dtype=torch.uint8) for d in datas], device='cuda')
            outs = []
            for topic, im in zip(topics, imgs):
                h, w = int(im.shape[1]), int(im.shape[2])
                lens = self._lens(topic)
                if lens is not None and lens.matches(w, h):
                    # 먼저 원본 해상도에서 편 뒤 아래에서 축소 (앨리어싱 방지)
                    im = lens_rectify.warp_tensor(im, lens.grid_tensor(w, h, im.device))
                if w > self.width:
                    nh = max(2, round(h * self.width / w))
                    im = F.interpolate(im.unsqueeze(0).float(), size=(nh, self.width), mode='bilinear',
                                       antialias=True, align_corners=False).squeeze(0).round_().clamp_(0, 255).to(torch.uint8)
                outs.append(im.contiguous())
            return [e.cpu().numpy().tobytes() for e in self._enc(outs, quality=self.quality)]

    def _cpu(self, topic, data):
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_REDUCED_COLOR_2)
        if img is None:
            return None
        lens = self._lens(topic)
        if lens is not None and abs(img.shape[1] / img.shape[0] - lens.size[0] / lens.size[1]) < 0.01:
            img = lens.remap(img)
        if img.shape[1] > self.width:
            img = cv2.resize(img, (self.width, max(2, round(img.shape[0] * self.width / img.shape[1]))),
                             interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        return buf.tobytes() if ok else None

    def run(self):
        node = self.node
        while rclpy.ok():
            self.event.wait(timeout=1.0)
            self.event.clear()
            now = _now()
            due = []
            with node.lock:
                for topic, t in list(self.demand.items()):
                    if now - t > 3.0:
                        self.demand.pop(topic, None)
                        node.camera_frames.pop(topic + self.suffix, None)
                        continue
                    f = node.camera_frames.get(topic)
                    if not f or f['stamp'] == self.last_src.get(topic) or now - self.last_out.get(topic, 0.0) < self.period:
                        continue
                    due.append((topic, f['stamp'], f['data']))
            if not due:
                time.sleep(0.01)
                if self.demand:
                    self.event.set()
                continue
            if self.backend is None:
                self._init_backend()
            outs = None
            if self.backend == 'gpu':
                try:
                    outs = self._gpu([t for t, _, _ in due], [d for _, _, d in due])
                except Exception as exc:
                    node.get_logger().warn(f'미리보기 GPU 변환 실패: {exc}', throttle_duration_sec=30.0)
            if outs is None:
                outs = [self._cpu(t, d) for t, _, d in due]
            stamp = _now()
            for (topic, src_stamp, _), out in zip(due, outs):
                self.last_src[topic] = src_stamp
                self.last_out[topic] = stamp
                if not out:
                    continue
                key = topic + self.suffix
                with node.lock:
                    node.camera_frames[key] = {'stamp': stamp, 'format': 'jpeg', 'data': out, 'interval': None}
                cond = node.camera_conditions[key]
                with cond:
                    cond.notify_all()
            self.event.set()


def preview_topic_for(topic):
    """'/<ns>/image_raw/compressed' → '/<ns>/preview/compressed' (gst_camera_node 의 하드웨어 미리보기)."""
    suffix = '/image_raw/compressed'
    return topic[:-len(suffix)] + '/preview/compressed' if topic.endswith(suffix) else None


def is_preview_topic(topic):
    return topic.endswith('/preview/compressed')


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
            'id': int(marker.id),
            'ns': marker.ns,
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


RADAR_MSG_TYPE = 'can_parser_msgs/msg/RadarMsg'


def radar_side(topic):
    """레이더 객체 토픽의 장착 위치 (front/rear). 진단 토픽(/Radar_Info 등)은 None."""
    lowered = topic.lower()
    if 'rear' in lowered:
        return 'rear'
    if 'front' in lowered:
        return 'front'
    return None


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
            elif parsed.path == '/bus_model.js':
                self.serve_file(STATIC_DIR / 'bus_model.js')
            elif parsed.path == '/cockpit':
                self.serve_file(STATIC_DIR / 'cockpit.html')
            elif parsed.path == '/cockpit.js':
                self.serve_file(STATIC_DIR / 'cockpit.js')
            elif parsed.path == '/cockpit.css':
                self.serve_file(STATIC_DIR / 'cockpit.css')
            elif parsed.path == '/api/ui/default-settings':
                path = default_ui_settings_file(node.backup_folder())
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
            elif parsed.path == '/api/road_perception/ipm':
                self.serve_json(node.road_ipm_state())
            elif parsed.path == '/api/lens_calibrator':
                self.serve_json(node.lens_tool_state())
            elif parsed.path == '/api/settings/server-backup':
                self.serve_json(node.settings_backup())
            elif parsed.path == '/api/settings/ui-list':
                self.serve_json(node.ui_backup_list())
            elif parsed.path == '/api/settings/ui-file':
                path = node.ui_backup_path(parse_qs(parsed.query).get('name', [''])[0])
                if path is None:
                    self.send_error(HTTPStatus.NOT_FOUND, 'No such backup file')
                else:
                    self.serve_file(Path(path))
            elif parsed.path == '/api/calibration_apply':
                self.serve_json(node.calibration_apply_state())
            elif parsed.path == '/api/gps/track/list':
                self.serve_json(node.list_gps_tracks())
            elif parsed.path == '/api/gps/track/load':
                name = parse_qs(parsed.query).get('file', [''])[0]
                self.serve_json(node.load_gps_track(name))
            elif parsed.path == '/api/bag/play/track':
                with node.play_lock:
                    track = dict(node.play_track)
                self.serve_json({'ok': True, **track})
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
            # 카메라 노드가 하드웨어로 만든 저해상도 미리보기(<ns>/preview/compressed)가 살아 있으면 그것을 보냄
            # (브라우저 JPEG 디코딩·전송량 약 1/4). ?full=1 이면 항상 원본.
            full = parse_qs(urlparse(self.path).query).get('full', ['0'])[0] == '1'
            rect = parse_qs(urlparse(self.path).query).get('rect', ['0'])[0] == '1'
            rect_tc = None
            if rect and node.rect_preview is not None and node.rect_preview.handles(topic):
                rect_tc = node.rect_full if full else node.rect_preview
            started = _now()
            preview = None if (full or rect_tc) else preview_topic_for(topic)
            # 축소 미리보기(모니터가 GPU 로 만듦) — 원본 카메라 JPEG 토픽만
            transcode = not full and node.preview.handles(topic)
            try:
                while True:
                    src = topic
                    if rect_tc is not None:
                        rect_tc.want(topic)
                        with node.lock:
                            pf = node.camera_frames.get(topic + rect_tc.suffix)
                        if pf and _now() - pf['stamp'] < 2.0:
                            src = topic + rect_tc.suffix
                        elif _now() - started < 8.0:
                            # 범위 편집은 편 영상 기준이므로 준비될 때까지 원본을 섞지 않는다
                            time.sleep(0.05)
                            continue
                    if preview:
                        with node.lock:
                            pf = node.camera_frames.get(preview)
                        if pf and _now() - pf['stamp'] < 2.0:
                            src = preview
                    if src == topic and transcode:
                        node.preview.want(topic)
                        with node.lock:
                            pf = node.camera_frames.get(topic + PREVIEW_SUFFIX)
                        # 미리보기가 아직 없거나 멈췄으면 원본으로 (변환기 실패 대비)
                        if pf and _now() - pf['stamp'] < 2.0:
                            src = topic + PREVIEW_SUFFIX
                    frame = node.wait_for_frame(src, last_stamp)
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
                light = True
                try:
                    length = int(self.headers.get('Content-Length', 0))
                    if length > 0:
                        body = json.loads(self.rfile.read(length))
                        if not body.get('all', False):
                            topics = body.get('topics', [])
                        light = bool(body.get('light', True))
                except Exception:
                    topics = None
                self.serve_json(node.start_bag(topics, light=light))
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
            elif parsed.path == '/api/calibration_apply':
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
                except Exception:
                    self.serve_json({'ok': False, 'error': '요청 형식 오류'})
                    return
                self.serve_json(node.calibration_apply_save(body))
            elif parsed.path == '/api/settings/ui-save':
                try:
                    length = int(self.headers.get('Content-Length', 0))
                    if length > SERVER_RESTORE_MAX_BYTES:
                        self.serve_json({'ok': False, 'error': '설정이 너무 큽니다'})
                        return
                    body = json.loads(self.rfile.read(length) or b'{}')
                except Exception:
                    self.serve_json({'ok': False, 'error': '요청 형식 오류'})
                    return
                self.serve_json(node.ui_backup_save(body))
            elif parsed.path == '/api/settings/server-restore':
                try:
                    length = int(self.headers.get('Content-Length', 0))
                    if length > SERVER_RESTORE_MAX_BYTES:
                        self.serve_json({'ok': False, 'error': '백업 파일이 너무 큽니다'})
                        return
                    body = json.loads(self.rfile.read(length) or b'{}')
                except Exception:
                    self.serve_json({'ok': False, 'error': '요청 형식 오류'})
                    return
                self.serve_json(node.settings_restore(body))
            elif parsed.path == '/api/lens_calibrator/start':
                self.serve_json(node.lens_tool_start())
            elif parsed.path == '/api/lens_calibrator/stop':
                self.serve_json(node.lens_tool_stop())
            elif parsed.path in ('/api/road_perception/ipm', '/api/road_perception/ipm/reset'):
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
                except Exception:
                    self.serve_json({'ok': False, 'error': '요청 형식 오류'})
                    return
                if parsed.path.endswith('/reset'):
                    self.serve_json(node.road_ipm_reset(str(body.get('camera', ''))))
                else:
                    self.serve_json(node.road_ipm_save(body))
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
