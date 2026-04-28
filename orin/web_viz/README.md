# orin/web_viz — ADCM 브라우저 시각화

HTML5 Canvas + WebSocket 로 실시간 궤적을 보여준다. `selfdrive/modeld/udp_bridge.py` 가 이미 발행하는 바이너리 UDP 스트림 (COMA 36B @ 5006, ADCM 1243B @ 5007) 을 그대로 소비한다.

**차량 pose 는 항상 ADCM ego 직송** — sim 에선 sender (`adcm_trajectory_sender.py`, `tools/sim/test_udp_sender.py`) 가 `gpsLocationExternal` + `livePose.orientationNED` 로 실 ADCM 의 자체 localization 을 모방해 보내주므로 viz 측 분기/적분 불필요. 실차/sim 코드 경로 동일.

## 의존성

```bash
pip install --user websockets
```

(`.venv/bin/python` 이 깨져 있어 `--user` 로 설치. `cereal` 의존성 없음 — 이 viz 서버는 순수 socket/asyncio/websockets 만 사용.)

## 실행

```bash
cd /home/gnu/workspace/openpilot-steer-limit-gwlee

# viz 서버 기동 (sim / 실차 동일 명령)
python3 orin/web_viz/server.py

# 브라우저로 접속
# → http://localhost:8080/

# (별 터미널) 시뮬은 이미 떠있다고 가정 — sender 만
./tools/sim/launch_sender.sh --speed 8 --curve -15
```

브라우저에는 `udp_bridge.py` 가 받은 ADCM 패킷의 planned 경로(청록) + planned ego(빨강) + ADCM ego 기반 actual 차량(초록) 이 실시간 표시됨.

원격 PC 에서 브라우저 열고 싶으면 방화벽 8080/8765 개방 후 `http://<server-ip>:8080/`.

## 포트 표

| 포트 | 방향 | 설명 |
|---|---|---|
| UDP 5006 | `server.py` 수신 | COMA 36B 바이너리 (`<Idddd`, magic `COMA`, livePose 파생 dynamics 표시 전용) |
| UDP 5007 | `server.py` 수신 | ADCM 1243B 바이너리 미러 (50 × (x,y,yaw) + ego + meta) |
| WS 8765 | 브라우저 ↔ 서버 | JSON 메시지 브로드캐스트 |
| HTTP 8080 | 브라우저 → 서버 | `index.html` 정적 서빙 |

`--coma-port`, `--adcm-port`, `--ws-port`, `--http-port` 로 오버라이드 가능. `--http-port 0` 은 자동 할당.

## WebSocket JSON 스키마

### `type="vehicle_planned"` (ADCM 패킷마다)
```json
{"type":"vehicle_planned",
 "x":..., "y":..., "yaw":...,
 "target_accel":..., "drive_mode":bool, "turn_signal":0|1|2|3,
 "emergency":..., "n_valid":..., "frame":int}
```

### `type="trajectory_world"` (ADCM 패킷마다, 50점)
```json
{"type":"trajectory_world", "seq":int, "num_points":int,
 "points":[{"x":..., "y":..., "yaw":...}, ...]}
```

### `type="vehicle"` (ADCM 또는 COMA 패킷마다)
```json
{"type":"vehicle",
 "x":..., "y":..., "heading":yaw_enu,
 "speed":..., "accel":a_fwd,
 "v_fwd":..., "v_right":..., "yaw_rate":...}
```
position/heading 은 ADCM 패킷 수신 시 갱신, dynamics 는 COMA 패킷 수신 시 갱신.

## 좌표계 / Pose source

- ADCM ego/trajectory 는 송신측 좌표계의 절대좌표 (실차: UTM Zone 52N — x=easting, y=northing, yaw=0→east, CCW+ / sim: MetaDrive xy 환원).
- 서버는 **첫 ADCM 패킷의 ego 위치** 를 anchor 로 저장, 이후 모든 x/y 에서 anchor 를 빼서 브라우저에 보냄 (큰 절대값의 float 정밀도 손실 회피).
- 첫 ADCM 전까지 COMA 패킷은 드롭 (서버 stderr 에 rate-limited warn).

### Pose 소스 — 모드 무관 단일 경로

| | Real-car / Sim 공통 |
|---|---|
| position / heading | **ADCM ego 직송** (실차: Orin 자체 localization / sim: sender 가 gpsLocationExternal + livePose.orientationNED 로 모방) |
| dynamics (속도/가속/yaw_rate) | livePose (COMA 패킷 파생) |
| drift | 0 |

sim 에서 sender 가 ADCM 을 모방하는 방식: `tools/sim/test_udp_sender.py` 와 `orin/adcm_trajectory_sender.py` 가 `gpsLocationExternal.{latitude,longitude}` 를 m 단위 (`(lat-BASE_LAT)*100000`) 로 환원해 ego_x/y 채우고, `livePose.orientationNED.z` 에서 `π/2 +` 변환으로 ENU yaw 를 ego_yaw 채움. JSON 리플레이 sender 는 첫 GPS 시점 캡처 후 평행이동+회전 변환을 모든 frame 에 적용 (start-anchor).

## 유지보수 노트

- `parse_adcm` / `parse_coma` 바이너리 파서가 `orin/web_viz/server.py` 에 있다. `selfdrive/modeld/udp_bridge.py` 의 1243B/36B 레이아웃을 바꾸면 **같이 수정** 필요.
- GPS lat/lon → m 환원 상수 (`GPS_BASE_LAT`, `GPS_BASE_LON`, `GPS_DEG_TO_METERS`) 는 sender 두 곳 (`tools/sim/test_udp_sender.py`, `orin/adcm_trajectory_sender.py`) 과 `tools/sim/lib/common.py` `GPSState.from_xy` 세 곳에 같은 값으로 박혀있다. 한 곳 바꾸면 다른 두 곳도 같이.
