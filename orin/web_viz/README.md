# orin/web_viz — ADCM 브라우저 시각화

HTML5 Canvas + WebSocket 로 실시간 궤적을 보여준다. `selfdrive/modeld/udp_bridge.py` 가 이미 발행하는 바이너리 UDP 스트림 (COMA 56B @ 5006, ADCM 1243B @ 5007) 을 그대로 소비한다. 즉 **bridge / sender 쪽은 수정 없음**.

## 의존성

```bash
pip install --user websockets
```

(`.venv/bin/python` 이 깨져 있어 `--user` 로 설치. `cereal` 의존성 없음 — 이 viz 서버는 순수 socket/asyncio/websockets 만 사용.)

## 실행

```bash
cd /home/gnu/workspace/openpilot-steer-limit-gwlee

# viz 서버 기동
python3 orin/web_viz/server.py

# 브라우저로 접속
# → http://localhost:8080/

# (별 터미널) 시뮬은 이미 떠있다고 가정 — sender 만
./tools/sim/launch_sender.sh --speed 8 --curve -15
```

브라우저에는 `udp_bridge.py` 가 받은 ADCM 패킷의 planned 경로(청록) + planned ego(빨강) + livePose 기반 actual 차량(초록) 이 실시간 표시됨. 실차에서도 동일 명령.

원격 PC 에서 브라우저 열고 싶으면 방화벽 8080/8765 개방 후 `http://<server-ip>:8080/`.

## 포트 표

| 포트 | 방향 | 설명 |
|---|---|---|
| UDP 5006 | `server.py` 수신 | COMA 56B 바이너리 (`<IIdddddd`, magic `COMA`, livePose 파생) |
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

### `type="vehicle"` (COMA 패킷마다, 20Hz)
```json
{"type":"vehicle",
 "x":..., "y":..., "heading":yaw_enu,
 "speed":..., "accel":a_fwd,
 "v_fwd":..., "v_right":..., "yaw_rate":...,
 "ts":..., "frame":seq}
```

## 좌표계 / Pose source

- ADCM sender 는 UTM Zone 52N 절대좌표 (x=easting, y=northing, yaw=0→east, CCW+) 사용.
- 서버는 **첫 ADCM 패킷의 ego 위치** 를 anchor 로 저장, 이후 모든 x/y 에서 anchor 를 빼서 브라우저에 보냄 (UTM 330km easting 의 float 정밀도 손실 회피).
- 첫 ADCM 전까지 COMA 패킷은 드롭 (서버 stderr 에 rate-limited warn).
- **Actual pose 소스 = livePose** (COMA 패킷).
  - yaw: `orientationNED.z` 직접 사용 (PoseKalman 의 cameraOdometry + IMU fused 결과). 적분 없음.
  - position: `velocityDevice.x` 를 fresh yaw 축으로 적분. yaw 와 동일 필터 state 에서 나오므로 좌표계 일관성 보장.
- `heading` 은 ENU yaw. 현재 테스트 환경 (MetaDrive) 부호 관례 때문에 `yaw_enu = π/2 + yaw_ned` 로 받음 (일반 NED→ENU 는 `π/2 − yaw_ned`). 차량 화살표가 "0→동쪽, CCW+" 로 뜸.

## 유지보수 노트

- `parse_adcm` / `parse_coma` 바이너리 파서가 `orin/web_viz/server.py` 에 있다. `selfdrive/modeld/udp_bridge.py` 의 1243B/56B 레이아웃을 바꾸면 **같이 수정** 필요. 추후 `orin/adcm_protocol.py` 로 추출 가능.
