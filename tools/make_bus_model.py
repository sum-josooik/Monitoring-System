#!/usr/bin/env python3
"""STL 차체 모델 → 웹 모니터용 메쉬 (ros_web_monitor/web/bus_model.js 의 데이터 부분).

  python3 tools/make_bus_model.py [STL 경로]

좌표: f = 차량 전방, l = 좌측, z = 위 (cm 정수). 원점 = 차체 길이·폭 중앙(미러 제외), 바닥.
STL(Hyundai_County.stl)은 cm 단위, 차량 앞이 -Y, 좌측이 +X.
"""
import json
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
STL = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / 'bringup' / '3D_model' / 'Hyundai_County.stl'
OUT = ROOT / 'ros_web_monitor' / 'web' / 'bus_model.js'
MARK_A, MARK_B = '// @bus-data-begin', '// @bus-data-end'

d = STL.read_bytes()
n = struct.unpack('<I', d[80:84])[0]
a = np.frombuffer(d[84:], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]), count=n)
T = a['v'].reshape(-1, 3, 3).astype(np.float64)
N = a['n'].astype(np.float64)

# STL → 차량 좌표 (f, l, z)
V = np.stack([-T[:, :, 1], T[:, :, 0], T[:, :, 2]], axis=2)
Nv = np.stack([-N[:, 1], N[:, 0], N[:, 2]], axis=1)
# 감는 방향을 저장된 면 법선과 맞춤 (바깥쪽이 반시계)
cr = np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0])
flip = np.einsum('ij,ij->i', cr, Nv) < 0
V[flip] = V[flip][:, ::-1]
cr = np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0])
keep = np.linalg.norm(cr, axis=1) > 1e-6
V = V[keep]

pts = V.reshape(-1, 3)
# 차체 폭 중앙: 미러(차체 앞쪽 위)를 뺀 몸통 기준
body = pts[(pts[:, 0] < pts[:, 0].max() - 80) & (pts[:, 2] > 60)]
lc = (body[:, 1].min() + body[:, 1].max()) / 2
fc = (pts[:, 0].min() + pts[:, 0].max()) / 2
V[:, :, 0] -= fc
V[:, :, 1] -= lc
V[:, :, 2] -= pts[:, 2].min()
pts = V.reshape(-1, 3)
length = float(pts[:, 0].max() - pts[:, 0].min())
half_w = float(np.abs(pts[:, 1]).max())
body_half_w = float((body[:, 1].max() - body[:, 1].min()) / 2)
height = float(pts[:, 2].max())

# 바퀴: 낮은 곳에서 축(차폭 방향) 둘레에 모인 삼각형
low = pts[(pts[:, 2] < 90) & (np.abs(pts[:, 1]) > body_half_w * 0.55)]
axles = []
for sign in (1, -1):
    s = low[np.sign(low[:, 0]) == sign]
    hist, edges = np.histogram(s[:, 0], bins=60)
    k = int(np.argmax(hist))
    near = s[np.abs(s[:, 0] - (edges[k] + edges[k + 1]) / 2) < 60]
    axles.append((float(np.median(near[:, 0])), float(np.percentile(near[:, 2], 50))))
kind = np.zeros(len(V), dtype=np.int8)
for af, az in axles:
    r = np.hypot(V[:, :, 0] - af, V[:, :, 2] - az).max(axis=1)
    kind[(r < 52) & (np.abs(V[:, :, 1]).min(axis=1) > body_half_w * 0.5)] = 1

q = np.round(V).astype(np.int32)
uniq, inv = np.unique(q.reshape(-1, 3), axis=0, return_inverse=True)
tri = inv.reshape(-1, 3)
ok = (tri[:, 0] != tri[:, 1]) & (tri[:, 1] != tri[:, 2]) & (tri[:, 0] != tri[:, 2])
tri, kind = tri[ok], kind[ok]

data = {
    'length': round(length, 1), 'halfWidth': round(half_w, 1), 'bodyHalfWidth': round(body_half_w, 1), 'height': round(height, 1),
    'axles': [[round(f, 1), round(z, 1)] for f, z in axles],
    'v': uniq.reshape(-1).tolist(), 't': tri.reshape(-1).tolist(), 'k': kind.tolist(),
}
block = f"{MARK_A} (tools/make_bus_model.py 가 생성 — 직접 고치지 말 것)\nconst BUS_DATA = {json.dumps(data, separators=(',', ':'))};\n{MARK_B}"
src = OUT.read_text() if OUT.exists() else f"'use strict';\n{MARK_A}\n{MARK_B}\n"
i, j = src.index(MARK_A), src.index(MARK_B) + len(MARK_B)
OUT.write_text(src[:i] + block + src[j:])
print(f'{OUT.name}: verts {len(uniq)}, tris {len(tri)} (wheel {int(kind.sum())}), length {length:.0f} cm, '
      f'half width {half_w:.0f} (body {body_half_w:.0f}), height {height:.0f}, axles {data["axles"]}, {OUT.stat().st_size // 1024} KiB')
