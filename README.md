# ROS Web Monitor

ROS 2 차량의 센서, 인식 결과, 시스템 상태를 브라우저 한 화면에서 보는 경량 웹 대시보드입니다.
ROS 2 노드 하나(`monitor_node`)가 모든 토픽을 구독하고, 내장 HTTP 서버로 대시보드를 제공합니다.
별도 웹 서버나 Node.js는 필요 없습니다.

![ROS Web Monitor](docs/overview_20261104_1800.jpg)

*실차 화면 (2026-11-04 18:00): 토픽 117개(활성 86), 카메라 9개 슬롯 모두 표시(스트림 포트 3개로 분산), RViz 탭(높이 색상 LiDAR·거리 링·차량 모양 자차·근접 경고), GNSS 위치(위성 33개)*

## 주요 기능

| 패널 | 내용 |
|---|---|
| **Topic Health** | 전체 토픽의 상태(active / idle / error), 주기(Hz), 마지막 수신 후 경과 시간. 검색, 그룹 필터, 행을 펼쳐 최근 메시지 값 확인 |
| **센서 그룹** | Cameras · LiDAR · Radar · Localization · Vehicle/CAN · Perception별 활성 토픽 수 요약 |
| **시스템 (Jetson)** | jtop 기반 CPU 코어별 사용률, GPU, 메모리, 온도, HW 엔진, 전원 모드 |
| **Camera Preview** | `Image` / `CompressedImage` 토픽 MJPEG 미리보기. 슬롯 추가·삭제·순서 변경·열 수 조절. 배지 `LIVE` / `WAIT` / `NO SIGNAL`. 클릭하면 [크게 보기](#카메라). 스트림 포트 여러 개로 나눠 연결해 6대를 넘어도 모두 표시 |
| **RViz Monitor** | 탭 3개: [**RViz**](#rviz-monitor-rviz-탭) / [**3D 주행** · **2D 레이더**](#주행-뷰-3d-주행--2d-레이더) |
| **GPS Map** | `NavSatFix` / `GPSFix` 위치를 Leaflet 지도에 표시. 이동 경로 기록·저장·불러오기. Bag 녹화와 함께 경로 기록, Play 시 bag 경로 표시([Bag](#bag-녹화--재생)) |
| **Vehicle Monitor** | `/acu_ck2_ver2` 기반 기어, 조향, 페달, 바퀴 속도, 배터리, 모터 토크 게이지 |
| **감지 객체** | 인식된 객체 목록(유형, 방위, 차체 기준 거리, 크기), 가까운 순. 차체 안의 박스(자차 클러스터)는 제외 |
| **이벤트 로그** | 모니터링 시작(활성 토픽 수), 시작 시 이미 오류인 센서·수신 없는 센서 그룹, 토픽 끊김 / 복구 / 오류, Bag 녹화 시작·종료, 고온(80°C), 배터리 부족(20%), 모니터 서버 연결 끊김·복구 |
| **Bag 녹화 / 재생** | 전체 또는 선택 토픽 녹화(기본: 대용량 토픽 제외). ROS 1 `.bag`, ROS 2 bag 재생. 재생 토픽은 `/bagplay/...`로 분리되어 **Play 뷰**에서 확인 |
| **전원** | 상단 버튼으로 보드 재부팅 / 종료 (sudoers 설정 필요, 아래 참고) |

화면 배치는 **Edit Layout**에서 패널을 드래그하고 크기를 바꿔 조정합니다. 상단 ⚙(또는 RViz 패널의 ⚙)에서 헤더 항목, Bag 녹화, RViz·차량 위치·주행 뷰, 백업을 설정합니다.

## RViz Monitor (RViz 탭)

![RViz 탭](docs/rviz_20261104_1800.jpg)

*차량 트랙 #1 에 마우스를 올린 모습. 차체 기준 거리·위치·크기·방향·토픽이 표시됩니다*

- **포인트 색상**: 오른쪽 위에서 **레이어 색 / 높이 / 거리** 선택(기본 높이). 파랑→빨강 색상표, 겹칠수록 밝아지는 발광 표현
- **바닥**: 멀수록 옅어지는 격자, 차량 중심 기준 거리 링(5·10·20·30·50 m …)과 거리 라벨
- **박스**: 바닥·윗면 반투명 채움 + 모서리 + 진행 방향 선. 차체에서 7 m 안이면 강조. 라벨 `#트랙ID 거리`는 겹치지 않게 가까운 것부터 최대 40개(**라벨** 버튼으로 끄기)
- **마우스 툴팁**: 박스에 올리면 ID, 종류, 방위, 차체 기준 거리, 위치, 크기, 방향
- **자차**: [차량 위치](#차량-위치-좌표계) 설정의 실제 차체(기본 8.2 × 3.0 m, 전방 +y)와 진행 방향 화살표
- **시점**: Top / Front / Left / Right / Reset 은 차량 전방 기준이며 0.35초 동안 부드럽게 전환. 드래그 회전, 휠클릭 드래그 이동, 휠 확대
- **보조 표시**: XYZ 방향 표시(오른쪽 아래), 축척 막대(1·2·5 단위), `실시간/Play · 속도` 배지, `점 · 박스 · 마커 · 초당 갱신` 통계, 근접 장애물 경고 배너
- **병합 클라우드 대체 표시**: LiDAR 레이어 토픽(`/Multi_Lidar_pointcloud`)에 데이터가 없으면, 개별 라이다를 `multi_lidar_merge.yaml`과 같은 변환으로 병합 좌표계에 옮겨(자차 영역 제거 포함) 대신 그립니다

## 차량 위치 (좌표계)

데이터 좌표계는 `multi_lidar_merge`의 `output_frame`입니다. 이 차량은 **전방 +y, 좌측 −x**입니다. 앞쪽 상단 Ouster가 y = +3.6~4.9 m, 뒤쪽 상단 Ouster가 y = −2.3 m에 있습니다. 차체 기본값은 병합 노드의 `remove_vehicle` 영역(x −1.3~1.7, y −2.5~5.7)입니다.

자차 표시, 방위(전방/후방/좌/우), 차체 기준 거리, 근접 경고, 감지 객체 목록, 주행 뷰가 모두 이 차체를 기준으로 계산됩니다. ⚙ → **RViz** → **차량 위치**에서 전방 방향(+x / +y / −x / −y), 중심 x·y, 길이·폭·높이를 바꿀 수 있습니다.

<img src="docs/settings_rviz_20261104_1800.jpg" alt="RViz 설정: 차량 위치 · 주행 뷰 · 주행 뷰 카메라" width="440">

## 주행 뷰 (3D 주행 / 2D 레이더)

RViz 패널의 **3D 주행**, **2D 레이더** 탭입니다. 트랙, 클러스터, 레이더, LiDAR, 차량 정보, 카메라를 차량 기준(차체 중심, 전방이 위)으로 모아 보여 줍니다.

| 3D 주행 (카메라 전체 9대, 차량 둘레 배치, LiDAR 켬) | 2D 레이더 (기본 20 m, 눈금 2 m) |
|---|---|
| ![3D 주행](docs/drive3d_20261104_1800.jpg) | ![2D 레이더](docs/radar2d_20261104_1800.jpg) |

- **3D 주행**
  - 자차와 차량 트랙은 차량 모형, 클러스터는 빨간 박스, 레이더는 보라색 점으로 그립니다. 라벨은 `#트랙ID 거리`, `장애물 거리`입니다.
  - 내 차로 안에서 가장 가까운 앞 객체를 **LEAD**(초록)로 표시하고, 앞 범퍼에서 LEAD까지 파란 빔을 그립니다. 거리 = 앞 범퍼 ~ 그 차 뒷면입니다.
  - 시점은 **주행 / 입체 / 위에서**입니다. 드래그로 회전, 휠로 확대, ↺로 초기화합니다.
- **2D 레이더**
  - 위에서 본 평면에 거리 눈금, 차로 점선, 트랙 ID, 레이더 거리, 장애물, LEAD를 표시합니다.
  - 기본 표시는 **전방 20 m**이고, 후방은 차체 뒤끝 + 2 m 이상 보입니다.
  - **휠로 마우스 위치 기준 확대**(약 3 m~250 m), **드래그로 이동**, **더블클릭**이나 ↺로 원위치합니다. 눈금(0.5 / 1 / 2 / 5 m …)은 확대에 맞춰 바뀌고, 글자가 겹치지 않게 간격을 유지합니다.
- **움직임 표시**: 지면 격자와 차선 점선이 차량 속도만큼 뒤로 흐릅니다. 속도·조향은 CAN(`/acu_ck2_ver2`)에서 가져오고, 없으면 **`GPSFix` 속도·진행방향**을 씁니다. 이때 `GPS` 표시가 붙고, 예상 경로는 GPS 회전율로 휩니다.
- **하단 정보 줄**: 차량 속도(기어 또는 GPS), 스티어링(좌/우 각도, GPS면 회전율 °/s), 페달 상태, 선행 차량 거리
- **카메라**: 표시 안 함 / 전방 1대(자동) / 전체 / 직접 선택. 배치는 **상단 한 줄**(2D는 오른쪽 세로 열) 또는 **차량 둘레**(토픽 이름의 Front·Left·Right·Rear·avm으로 위치 결정)입니다. 타일을 클릭하면 크게 봅니다. 선택한 카메라는 실시간 이름으로 저장되어 Play에서는 자동으로 `/bagplay/...` 영상을 씁니다.
- 주소 뒤에 `?vtab=drive3d` 또는 `?vtab=radar2d`를 붙이면 해당 탭이 바로 열립니다.

| 표시 | 출처 |
|---|---|
| 차량 트랙 | `kind=track` 박스 (예: `/vehicle_tracks` MarkerArray CUBE, `ns=vehicle_tracks`, ID = `marker.id`) |
| 클러스터 장애물 | Detection3DArray / 객체 MarkerArray 박스. 트랙과 1.6 m 안으로 겹치거나 차체 안의 것은 생략 |
| 레이더 점 | 토픽 이름에 `radar`가 들어간 POINTS 마커 (`/radar_markers/front`, `/radar_markers/rear`) |
| LiDAR 점 | RViz **LiDAR** 레이어 토픽. 없으면 병합 좌표계로 변환한 개별 라이다 |
| 차선 | 토픽 이름에 `lane`이 들어간 LINE_STRIP / LINE_LIST 마커. **없으면 예상 주행 경로를 차로로 표시** |
| 속도·조향·페달 | `/acu_ck2_ver2`, 없으면 `/gpsfix`(속도·진행방향). Play 뷰에서는 `/bagplay/...` |

**설정**은 ⚙ → **RViz** → **주행 뷰** / **주행 뷰 카메라**에서 하며, 저장하면 백업 파일에도 들어갑니다.

| 항목 | 기본값 | 설명 |
|---|---|---|
| 표시 레이어 | 트랙·클러스터·레이더·예상 경로 켬, LiDAR 끔 | 레이어별 표시 여부 |
| 전방 표시 거리 | 80 m | 3D 차로 길이 |
| 2D 기본 표시 거리 | 20 m | 2D 처음 확대 범위 (↺·더블클릭 시 복귀) |
| 차로 폭 / 조향비 / 축거 | 3.5 m / 14.5 / 2.7 m | 예상 경로 계산용. **조향비·축거는 추정치이므로 실제 차량 값으로 바꿔야 정확합니다** |
| 카메라 표시 / 배치 | 전방 1대(자동) / 상단 한 줄 | 위 설명 참고 |

## 카메라

![카메라 확대 보기](docs/camera_zoom_20261104_1800.jpg)

- **확대 보기**: Camera Preview 영상이나 주행 뷰 카메라 타일을 클릭하면 크게 봅니다. **‹ ›** 또는 **←/→** 키로 카메라를 바꾸고, **Esc**나 바깥 클릭으로 닫습니다. 제목줄에 토픽, 해상도, 마지막 프레임 경과 시간, 순번이 나옵니다.
- **스트림 포트 분산**: 브라우저는 호스트:포트당 동시 연결이 6개입니다. 그래서 카메라 슬롯을 스트림 포트 3개(8081~8083)에 순서대로 나눠 연결합니다. 포트 3개로 최대 18대까지 볼 수 있습니다.
- **연결 관리**: 슬롯을 다시 만들거나 토픽이 바뀔 때 이전 MJPEG 연결을 즉시 끊습니다. 뒤늦게 나타난 카메라(예: Play 시작 직후의 `/bagplay` 토픽)는 새로고침 없이 자동으로 연결합니다.
- **재사용**: 확대 보기와 주행 뷰 카메라는 Camera Preview에 열린 영상을 다시 그리므로 브라우저 연결을 더 쓰지 않습니다. 대신 Camera Preview 슬롯에 없는 카메라는 표시할 수 없습니다.

## Bag 녹화 / 재생

![Play 뷰](docs/play_20261104_1800.jpg)

*Play 뷰: 재생 바, RViz 탭(bag 에 병합 클라우드가 거의 없어 개별 라이다 8대를 변환해 표시, GPS 속도 표시), GPS 지도의 bag 전체 경로(보라)*

<img src="docs/settings_bag_20261104_1800.jpg" alt="Bag 녹화 설정" width="440">

- **대용량 토픽 제외 (기본 켬)**: 전체 녹화(`-a`) 시 raw 카메라 영상(`*/image_raw`, compressed는 기록, AVM은 raw뿐이라 유지), 라이다 원본 패킷(`velodyne_packets`), Ouster 2D 이미지(`nearir/range/reflec/signal_image`), `/cluster_debug_points`를 뺍니다. 재생 중인 `/bagplay/...` 토픽은 항상 뺍니다. 설정 → **Bag 녹화**에서 끌 수 있습니다.
- **GPS 경로 동시 기록**: Bag 녹화 중에는 GPS 경로가 함께 기록되어 지도에 노란 선으로 보입니다. 녹화를 멈추면 bag 폴더의 `gps_track.json`과 `log/gps/track_bag_<bag>.json`에 저장됩니다.
- **Play 시 경로 표시**: bag을 재생하면 그 bag의 전체 경로를 지도에 **보라색**으로 그립니다. `gps_track.json`이 없는 기존 bag은 bag 안의 `/gpsfix`(없으면 NavSatFix 토픽)에서 경로를 뽑아(35 GB bag 기준 약 5초) `gps_track.json`으로 저장해 둡니다.
- **Play 화면 보완**:
  - 병합 클라우드가 없으면 개별 라이다로 대체 표시합니다.
  - CAN이 없으면 GPS 속도·진행방향으로 움직임을 표시합니다.
  - 녹화 공백이 재생되는 것을 감안해, Play에서는 `NO SIGNAL` 판정을 6초로 늘립니다(실시간 3초).

## 요구 사항

- ROS 2 Jazzy (Ubuntu 24.04, Jetson AGX Thor에서 검증)
- Python 패키지: `rclpy`, `sensor_msgs`, `sensor_msgs_py`, `rosidl_runtime_py`, `rosbag2_py`, `python3-opencv`, `python3-numpy`, `python3-pil`, `python3-yaml`
- 선택:
  - [jetson-stats](https://github.com/rbonghi/jetson_stats) (`jtop`): 시스템 탭 데이터용
  - `rosbags`: ROS 1 `.bag` 재생용. 처음 쓸 때 `~/.local/share/ros_web_monitor/rosbags_venv`에 자동 설치됩니다.
  - `multi_lidar_merge` 패키지 설치(`share/multi_lidar_merge/config/multi_lidar_merge.yaml`): 라이다 변환·자차 영역 대체 표시용
- 브라우저에서 CDN에 접속할 수 있어야 합니다(Leaflet 지도, 웹 폰트). 오프라인이면 지도가 표시되지 않고, 글꼴은 시스템 기본 폰트로 대체됩니다.

## 빌드 및 실행

```bash
cd ~/captin5_ws
colcon build --packages-select ros_web_monitor
source install/setup.bash

ros2 launch ros_web_monitor monitor.launch.py
# 또는 전체 시스템과 함께: ros2 launch bringup run.launch.xml
```

브라우저에서 `http://<host>:8080`에 접속합니다. 기본값은 `http://192.168.1.100:8080`입니다.
웹 파일(`web/`)만 바꿨다면 빌드 후 브라우저 새로고침(Ctrl+Shift+R)으로 적용되고, `monitor_node.py`를 바꿨다면 노드를 다시 실행해야 합니다.

```bash
ros2 launch ros_web_monitor monitor.launch.py host:=0.0.0.0 port:=8080
```

## 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `host` | `0.0.0.0` (launch: `192.168.1.100`) | HTTP 서버 바인드 주소 |
| `port` | `8080` | 대시보드, API 포트 |
| `stream_port` | `8081` | 카메라 MJPEG 스트림 전용 첫 포트 |
| `stream_port_count` | `3` | 스트림 포트 개수(`stream_port` ~ `stream_port+count-1`). 포트 3개 → 카메라 최대 18대 |
| `scan_period_sec` | `2.0` | 토픽 목록 갱신 주기 |
| `rate_window_sec` | `5.0` | Hz 계산 구간 |
| `subscribe_all` | `true` | 모든 토픽 자동 구독 |
| `visual_max_points_per_topic` | `900` | RViz 패널에 보내는 토픽당 최대 포인트 수 (화면에서도 변경 가능) |
| `prune_after_sec` | `10.0` | 사라진 토픽을 목록에서 정리하기까지의 시간 |
| `bag_dir` | `/home/nvidia/robot_ws/bag` | Bag 녹화, 재생 기본 폴더 (화면에서 변경 가능) |

**성능 관련 동작**
- 화면에 쓰지 않는 대용량 메시지(raw `Image`, `VelodyneScan`, Ouster `PacketMsg`)는 역직렬화하지 않고 바이트로만 받아 Hz와 크기를 셉니다. raw 영상의 해상도와 인코딩은 메시지 앞부분만 읽어 표시합니다.
- PointCloud2 다운샘플은 numpy로 처리합니다.

## 화면 설정 저장과 기본 설정 파일

레이아웃, 카메라 슬롯, RViz 레이어·색상 모드, 차량 위치, 주행 뷰 같은 화면 설정은 **브라우저 localStorage**에 저장됩니다.
따라서 다른 브라우저나 다른 주소(`localhost` ↔ IP)로 접속하거나 브라우저 데이터를 지우면 설정이 비어 있습니다.

이를 보완하기 위해 저장소의 [`UI/`](UI/) 폴더에 기본 설정 파일을 둡니다.

- 브라우저에 저장된 설정이 없으면 페이지를 열 때 `UI/*.json` 중 **가장 최신 파일**(파일명 순)을 자동으로 적용합니다.
- ⚙ 설정 → **백업** 탭
  - **⬇ 설정 백업 다운로드**: 현재 설정을 JSON으로 저장
  - **⬆ 백업 불러오기**: JSON 파일로 복원
  - **📄 UI 폴더 설정 파일 불러오기**: `UI/`의 최신 파일로 강제 교체

팀 공통 기본 화면을 바꾸려면 백업 탭에서 설정을 다운로드해 `UI/`에 넣고 다시 빌드합니다(`colcon build --packages-select ros_web_monitor`).

## 재부팅 / 종료 버튼 설정

상단 **⟳ 재부팅**, **⏻ 종료** 버튼은 `sudo -n`으로 실행됩니다. 처음 한 번 비밀번호 없는 sudo를 허용해야 합니다.

```bash
echo "$USER ALL=(root) NOPASSWD: /usr/sbin/shutdown, /usr/sbin/reboot" \
  | sudo tee /etc/sudoers.d/ros_web_monitor_power && sudo chmod 440 /etc/sudoers.d/ros_web_monitor_power
```

## HTTP API

| 메서드 | 경로 | 설명 |
|---|---|---|
| GET | `/`, `/app.js`, `/app.css` | 대시보드 (`?vtab=rviz\|drive3d\|radar2d`로 RViz 패널 탭 지정) |
| GET | `/api/state` | 전체 상태 스냅샷 (토픽, 카메라, 시스템, GPS·속도·진행방향, 차량, 스트림 포트 등) |
| GET | `/events` | 상태 실시간 푸시 (Server-Sent Events) |
| GET | `/stream?topic=<topic>` | 카메라 MJPEG 스트림 (각 스트림 포트에서도 제공) |
| GET | `/api/ui/default-settings` | `UI/` 폴더의 최신 화면 설정 JSON |
| GET | `/api/bag/play/list` | 재생 가능한 bag 목록 |
| GET | `/api/bag/play/track` | 재생 중인 bag 의 GPS 경로 (`status`: loading / ready / none) |
| GET | `/api/gps/track/list`, `/api/gps/track/load?file=` | 저장된 GPS 이동 경로 |
| GET | `/api/fs/list?path=` | Bag 저장 폴더 탐색 |
| POST | `/api/viz-config` | RViz 패널 설정 반영 |
| POST | `/api/bag/config`, `/api/bag/start`, `/api/bag/stop` | Bag 녹화 (`start` body: `all`, `topics`, `light`) |
| POST | `/api/bag/play/{start,stop,pause,resume,next,seek,rate}` | Bag 재생 제어 |
| POST | `/api/gps/track/{start,stop,delete}` | GPS 이동 경로 기록 |
| POST | `/api/fs/mkdir` | Bag 저장 폴더 생성 |
| POST | `/api/system/reboot`, `/api/system/shutdown` | 보드 전원 제어 |

## 디렉터리 구조

```
ros_web_monitor/
├── ros_web_monitor/
│   ├── monitor_node.py      # ROS 2 노드 + HTTP 서버 + API
│   └── web/                 # 프런트엔드 (index.html, app.js, app.css)
├── launch/monitor.launch.py
├── UI/                      # 기본 화면 설정 JSON (자동 적용)
├── docs/                    # README 스크린샷
├── log/gps/                 # GPS 이동 경로 저장 (git 제외)
├── package.xml
└── setup.py
```

## 문제 해결

- **페이지가 뜨지 않음**: `host` 파라미터의 주소로 접속했는지 확인하세요. launch 기본값은 `192.168.1.100`이라 `localhost`로는 접속할 수 없습니다.
- **화면 설정이 초기화됨**: 접속 주소나 브라우저가 바뀌었기 때문입니다. `UI/` 기본 설정이 자동 적용되며, 필요하면 백업 탭에서 다시 불러오세요.
- **시스템 탭이 비어 있음**: `jtop`이 설치되어 있고 `jtop.service`가 실행 중인지 확인하세요.
- **재부팅 / 종료 버튼 실패**: 위 sudoers 설정을 확인하세요.
- **일부 카메라 슬롯만 검은 화면 (`LIVE`인데 영상 없음)**: 스트림 포트(기본 8081~8083)가 모두 열렸는지 확인하세요(`ss -ltn | grep 808`). 시작 로그에 `스트림 포트 … 사용 불가` 경고가 있는지 보고, 방화벽이 있다면 이 포트들을 허용하세요.
- **자차 위치·방향이 점구름의 빈 영역과 안 맞음**: ⚙ → RViz → **차량 위치**에서 전방 방향과 차체 중심·크기를 실제 차량에 맞추세요.
- **주행 뷰 하단이 `—`, 배지가 "차량 데이터 없음"**: `/acu_ck2_ver2`와 `/gpsfix` 속도가 모두 없는 상태입니다. 객체 표시는 영향이 없고, 예상 경로는 직진으로 그립니다.
- **주행 뷰에 카메라가 안 나옴**: 선택한 카메라가 Camera Preview 슬롯에 열려 있는지 확인하세요. 설정의 카메라 목록에 "미리보기에 없음"으로 표시됩니다.
- **GPS 패널이 비어 있음**: 수신기가 위치를 계산하지 못한 상태입니다(`/pvtgeodetic`의 `nr_sv: 255`, `mode: 0`). 실내이거나 안테나 연결 문제일 수 있으니, 하늘이 보이는 곳에서 확인하세요.

## 알려진 문제

- **녹화가 밀린 bag은 재생에서도 끊김**: CPU가 포화된 상태에서 raw 영상까지 전체 녹화하면 녹화기가 따라가지 못합니다. 그러면 메시지가 1~4초씩 몰려 저장되고, 재생할 때도 그대로 재현됩니다. 2026-11-04 bag(34.9 GB / 225초)은 카메라 공백이 최대 4.7초였고, 병합 LiDAR는 7개만 기록됐습니다. 녹화할 때는 **대용량 토픽 제외**를 켜 두세요.
- **보드 CPU 포화**: 카메라 노드(`v4l2_camera`) 8개가 CPU 코어 약 8개 분량을 씁니다(load average 약 44 / 14코어). 녹화 공백과 화면 지연의 근본 원인이므로, 하드웨어 JPEG 인코더(NVJPG) 사용이나 raw 영상 발행 주기 조정을 검토 중입니다.
- **차선은 실제 인식 결과가 아님**: 차선 인식 토픽이 없으면 주행 뷰의 차로는 조향각(또는 GPS 회전율) 기반 예상 경로입니다. 선행 차량(LEAD) 판단도 이 예상 차로를 기준으로 합니다.
- **레이더 좌표계 미확인**: 레이더 마커는 `can` 프레임으로 들어오며, 병합 좌표계와 일치하는지는 레이더를 연결한 상태에서 아직 확인하지 않았습니다.
