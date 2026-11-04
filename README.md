# ROS Web Monitor

ROS 2 차량의 센서, 인식 결과, 시스템 상태를 브라우저 한 화면에서 보는 경량 웹 대시보드입니다.
ROS 2 노드 하나(`monitor_node`)가 모든 토픽을 구독하고, 내장 HTTP 서버로 대시보드를 제공합니다.
별도 웹 서버나 Node.js는 필요 없습니다.

![ROS Web Monitor](docs/overview_20261104.jpg)

*실차 화면 (2026-11-04): 토픽 117개(활성 86), GMSL 카메라 8대 + AVM, RViz 패널의 3D 주행 탭(트랙·클러스터·선행 차량·카메라 PIP), GNSS 위치(위성 31개). 번호판은 가림 처리*

## 주요 기능

| 패널 | 내용 |
|---|---|
| **Topic Health** | 전체 토픽의 상태(active / idle / error), 주기(Hz), 마지막 수신 후 경과 시간. 검색, 그룹 필터, 행을 펼쳐 최근 메시지 값 확인 |
| **센서 그룹** | Cameras · LiDAR · Radar · Localization · Vehicle/CAN · Perception별 활성 토픽 수 요약 |
| **시스템 (Jetson)** | jtop 기반 CPU 코어별 사용률, GPU, 메모리, 온도, HW 엔진, 전원 모드 |
| **Camera Preview** | `Image` / `CompressedImage` 토픽을 MJPEG로 미리보기. 슬롯 추가, 삭제, 순서 변경, 열 수 조절. 상태 배지: `LIVE` / `WAIT` / `NO SIGNAL`(3초 이상 프레임 없음). 영상을 클릭하면 크게 보기(←/→ 로 카메라 전환, Esc 닫기) |
| **RViz Monitor** | 탭 3개: **RViz**(PointCloud2, MarkerArray, 레이더, 클러스터, 트랙 3D 표시. Top/Front/Left/Right 시점, 레이어 토글, 근접 장애물 경고 배너) / **3D 주행** / **2D 레이더**. 자세한 내용은 [주행 뷰](#주행-뷰-3d-주행--2d-레이더) |
| **GPS Map** | `NavSatFix` / `GPSFix` 위치를 Leaflet 지도에 표시. 이동 경로 기록, 저장, 불러오기 |
| **Vehicle Monitor** | `/acu_ck2_ver2` 기반 기어, 조향, 페달, 바퀴 속도, 배터리, 모터 토크 게이지 |
| **감지 객체** | 인식된 객체 목록(유형, 방위, 거리, 크기), 가까운 순 |
| **이벤트 로그** | 모니터링 시작(활성 토픽 수), 시작 시점에 이미 오류인 센서·수신 없는 센서 그룹, 토픽 끊김 / 복구 / 오류, Bag 녹화 시작·종료, 고온(80°C), 배터리 부족(20%), 모니터 서버 연결 끊김·복구 |
| **Bag 녹화 / 재생** | 전체 또는 선택 토픽 녹화(`ros2 bag record`). ROS 1 `.bag`, ROS 2 bag 재생. 재생 토픽은 `/bagplay/...`로 분리되며 **Play 뷰**에서 확인 |
| **전원** | 상단 버튼으로 보드 재부팅 / 종료 (sudoers 설정 필요, 아래 참고) |

화면 배치는 **Edit Layout**에서 패널을 드래그하고 크기를 바꿔 조정합니다. 상단 ⚙에서 헤더 항목, Bag 녹화, RViz 레이어·주행 뷰, 백업을 설정합니다.

## 주행 뷰 (3D 주행 / 2D 레이더)

RViz Monitor 패널 상단의 **3D 주행**, **2D 레이더** 탭입니다. 트랙, 클러스터, 레이더, LiDAR, 차량 정보, 카메라를 한 화면에 모아 운전자 시점으로 보여 줍니다.

| 3D 주행 (카메라: 전체, 배치: 차량 둘레) | 2D 레이더 (약 30 m로 확대) |
|---|---|
| ![3D 주행](docs/drive3d_20261104.jpg) | ![2D 레이더](docs/radar2d_20261104.jpg) |

**화면 구성**

- **3D 주행**
  - 자차와 차량 트랙을 차량 모형으로, 클러스터를 빨간 박스로, 레이더를 보라색 점으로 그립니다. 라벨은 `#트랙ID 거리`, `장애물 거리` 형식입니다.
  - 내 차로 안에서 가장 가까운 앞 객체를 **LEAD**(초록)로 표시하고, 자차에서 LEAD까지 파란 빔을 그립니다. LEAD와 겹치는 레이더 점에는 주황 링이 붙습니다.
  - 시점: **주행 / 입체 / 위에서**. 드래그로 회전, 휠로 확대, ↺로 초기화합니다.
- **2D 레이더**
  - 위에서 본 평면에 거리 눈금, 차로 점선, 트랙 ID, 레이더 거리, 장애물, LEAD를 표시합니다.
  - **휠로 마우스 위치 기준 확대**(약 3 m~250 m), **드래그로 이동**, **더블클릭**이나 ↺로 원위치합니다. 눈금 간격(0.5 / 1 / 2 / 5 / 10 / 20 m …)은 확대 정도에 맞춰 바뀌고, 왼쪽 아래에 표시됩니다.
- **하단 정보 줄**: 차량 속도(기어), 스티어링(좌/우 각도), 페달 상태(브레이크 밟음 / 가속 중 / 페달 없음), 선행 차량 거리.
- **카메라**: 화면 위에 카메라 영상을 겹쳐 보여 주며, 영상 위를 클릭하면 [크게 보기](#카메라-확대-보기)가 열립니다.
- 주소 뒤에 `?vtab=drive3d` 또는 `?vtab=radar2d`를 붙이면 해당 탭이 바로 열립니다.

**사용하는 데이터**

| 표시 | 출처 |
|---|---|
| 차량 트랙 | `kind=track` 박스 (예: `/vehicle_tracks` MarkerArray CUBE, `ns=vehicle_tracks`, ID = `marker.id`) |
| 클러스터 장애물 | Detection3DArray / 객체 MarkerArray 박스. 트랙과 1.6 m 안으로 겹치는 클러스터는 생략 |
| 레이더 점 | 토픽 이름에 `radar`가 들어간 POINTS 마커 (`/radar_markers/front`, `/radar_markers/rear`) |
| LiDAR 점 | RViz 레이어 설정의 **LiDAR** 레이어 토픽 (기본 꺼짐) |
| 차선 | 토픽 이름에 `lane`이 들어간 LINE_STRIP / LINE_LIST 마커. **없으면 조향각으로 계산한 예상 주행 경로를 차로로 표시** |
| 속도·조향·페달 | `/acu_ck2_ver2` (Play 뷰에서는 `/bagplay/acu_ck2_ver2`) |

**설정**: 패널의 ⚙ 또는 상단 ⚙ → **RViz** 탭 아래쪽 **주행 뷰**, **주행 뷰 카메라** 섹션에서 바꾸고 **저장**을 누릅니다. 값은 다른 화면 설정과 함께 백업 파일에 들어갑니다.

<img src="docs/settings_drive_20261104.jpg" alt="주행 뷰 설정" width="440">

| 항목 | 기본값 | 설명 |
|---|---|---|
| 표시 레이어 | 트랙·클러스터·레이더·예상 경로 켜짐, LiDAR 꺼짐 | 레이어별 표시 여부 |
| 전방 표시 거리 | 80 m | 3D 차로 길이, 2D 기본 확대 범위 |
| 차로 폭 / 조향비 / 축거 | 3.5 m / 14.5 / 2.7 m | 예상 경로 계산용. **조향비·축거는 추정치이므로 실제 차량 값으로 바꿔야 경로가 정확합니다** |
| 카메라 표시 | 전방 1대 (자동) | 표시 안 함 / 전방 1대 / 전체 카메라 / 직접 선택(체크 순서대로) |
| 배치 | 상단 한 줄 | **상단 한 줄**(2D에서는 오른쪽 세로 열) / **차량 둘레**(토픽 이름의 Front·Left·Right·Rear·avm으로 위치 결정) |

## 카메라 확대 보기

Camera Preview의 영상이나 주행 뷰의 카메라 타일을 클릭하면 크게 볼 수 있습니다. **‹ ›** 버튼이나 **←/→** 키로 카메라를 바꾸고, **Esc**나 바깥 클릭으로 닫습니다. 제목줄에 토픽, 해상도, 마지막 프레임 경과 시간이 나옵니다.

![카메라 확대 보기](docs/camera_zoom_20261104.jpg)

확대 보기와 주행 뷰 카메라는 Camera Preview에 이미 열려 있는 영상을 다시 그리는 방식이라 **브라우저 연결을 추가로 쓰지 않습니다.** 그래서 Camera Preview 슬롯에 없는 카메라는 표시할 수 없습니다.

## 요구 사항

- ROS 2 Jazzy (Ubuntu 24.04, Jetson AGX Thor에서 검증)
- Python 패키지: `rclpy`, `sensor_msgs`, `sensor_msgs_py`, `rosidl_runtime_py`, `python3-opencv`, `python3-numpy`, `python3-pil`, `python3-yaml`
- 선택:
  - [jetson-stats](https://github.com/rbonghi/jetson_stats) (`jtop`): 시스템 탭 데이터용
  - `rosbags`: ROS 1 `.bag` 재생용. 처음 쓸 때 `~/.local/share/ros_web_monitor/rosbags_venv`에 자동 설치됩니다.
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

launch 인자 예시:

```bash
ros2 launch ros_web_monitor monitor.launch.py host:=0.0.0.0 port:=8080
```

## 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `host` | `0.0.0.0` (launch: `192.168.1.100`) | HTTP 서버 바인드 주소 |
| `port` | `8080` | 대시보드, API 포트 |
| `stream_port` | `8081` | 카메라 MJPEG 스트림 전용 포트. 메인 포트의 브라우저 동시 연결 한도를 보호하기 위해 분리 |
| `scan_period_sec` | `2.0` | 토픽 목록 갱신 주기 |
| `rate_window_sec` | `5.0` | Hz 계산 구간 |
| `subscribe_all` | `true` | 모든 토픽 자동 구독 |
| `visual_max_points_per_topic` | `900` | RViz 패널에 보내는 토픽당 최대 포인트 수 (화면에서도 변경 가능) |
| `prune_after_sec` | `10.0` | 사라진 토픽을 목록에서 정리하기까지의 시간 |
| `bag_dir` | `/home/nvidia/robot_ws/bag` | Bag 녹화, 재생 기본 폴더 (화면에서 변경 가능) |

## 화면 설정 저장과 기본 설정 파일

레이아웃, 카메라 슬롯, RViz 레이어 같은 화면 설정은 **브라우저 localStorage**에 저장됩니다.
따라서 다른 브라우저나 다른 주소(`localhost` ↔ IP)로 접속하거나 브라우저 데이터를 지우면 설정이 비어 있습니다.

이를 보완하기 위해 저장소의 [`UI/`](UI/) 폴더에 기본 설정 파일을 둡니다.

- 브라우저에 저장된 설정이 없으면 페이지를 열 때 `UI/*.json` 중 **가장 최신 파일**(파일명 순)을 자동으로 적용합니다.
- ⚙ 설정 → **백업** 탭
  - **⬇ 설정 백업 다운로드**: 현재 설정을 JSON으로 저장
  - **⬆ 백업 불러오기**: JSON 파일로 복원
  - **📄 UI 폴더 설정 파일 불러오기**: `UI/`의 최신 파일로 강제 교체

팀 공통 기본 화면을 바꾸려면 다음 순서로 진행합니다.

1. 백업 탭에서 설정을 다운로드합니다.
2. 받은 파일을 `UI/`에 넣습니다.
3. 다시 빌드합니다(`colcon build --packages-select ros_web_monitor`).

## 재부팅 / 종료 버튼 설정

상단 **⟳ 재부팅**, **⏻ 종료** 버튼은 `sudo -n`으로 실행됩니다. 처음 한 번 비밀번호 없는 sudo를 허용해야 합니다.

```bash
echo "$USER ALL=(root) NOPASSWD: /usr/sbin/shutdown, /usr/sbin/reboot" \
  | sudo tee /etc/sudoers.d/ros_web_monitor_power && sudo chmod 440 /etc/sudoers.d/ros_web_monitor_power
```

## HTTP API

| 메서드 | 경로 | 설명 |
|---|---|---|
| GET | `/`, `/app.js`, `/app.css` | 대시보드 |
| GET | `/api/state` | 전체 상태 스냅샷 (토픽, 카메라, 시스템, GPS, 차량 등) |
| GET | `/events` | 상태 실시간 푸시 (Server-Sent Events) |
| GET | `/stream?topic=<topic>` | 카메라 MJPEG 스트림 (`stream_port`에서도 제공) |
| GET | `/api/ui/default-settings` | `UI/` 폴더의 최신 화면 설정 JSON |
| GET | `/api/bag/play/list` | 재생 가능한 bag 목록 |
| GET | `/api/gps/track/list`, `/api/gps/track/load?file=` | 저장된 GPS 이동 경로 |
| GET | `/api/fs/list?path=` | Bag 저장 폴더 탐색 |
| POST | `/api/viz-config` | RViz 패널 설정 반영 |
| POST | `/api/bag/config`, `/api/bag/start`, `/api/bag/stop` | Bag 녹화 |
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
- **주행 뷰 하단이 `—`, 배지가 "차량 데이터 없음"**: `/acu_ck2_ver2`가 들어오지 않는 상태입니다(CAN 미연결 등). 객체·레이더 표시는 영향이 없고, 예상 경로는 직진으로 그립니다.
- **주행 뷰에 카메라가 안 나옴**: 선택한 카메라가 Camera Preview 슬롯에 열려 있는지 확인하세요. 설정의 카메라 목록에 "미리보기에 없음"으로 표시됩니다.
- **GPS 패널이 비어 있음**: 수신기가 위치를 계산하지 못한 상태입니다(`/pvtgeodetic`의 `nr_sv: 255`, `mode: 0`). 실내이거나 안테나 연결 문제일 수 있으니, 하늘이 보이는 곳에서 확인하세요.

## 알려진 문제

- **카메라 슬롯이 6개를 넘으면 일부 슬롯이 검은 화면으로 나옴**: 브라우저는 한 서버 주소(호스트:포트)에 동시에 열 수 있는 HTTP 연결이 6개로 제한됩니다. 모든 MJPEG 스트림이 `stream_port` 하나를 쓰기 때문에 7번째 슬롯부터는 연결을 열지 못합니다. 토픽 자체는 정상으로 수신되므로 배지는 `LIVE`로 표시됩니다. 검은 슬롯의 카메라는 확대 보기와 주행 뷰에도 나오지 않습니다. 지금은 카메라 슬롯을 6개 이하로 두는 방법으로 피할 수 있습니다.
- **차선은 실제 인식 결과가 아님**: 차선 인식 토픽이 없으면 주행 뷰의 차로는 조향각 기반 예상 경로입니다. 선행 차량(LEAD) 판단도 이 예상 차로를 기준으로 합니다.
