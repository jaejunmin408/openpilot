# ADCM (Orin) to openpilot 경로 브릿지 - 전체 흐름

## 개요

Orin의 ADCM Planning이 생성한 driving trajectory를 openpilot의 controlsd로 전달하여 차량을 제어하는 구조.
modeld(신경망)를 udp_bridge.py로 대체하고, 기존 plannerd/controlsd 파이프라인을 그대로 활용한다.

---

## 1. 테스트 경로 생성

**실행:**
```bash
python3 generate_test_trajectory.py
```

**출력:** `test_trajectory_left_turn.json`

| 항목 | 값 |
|------|-----|
| 프레임 수 | 134개 (20Hz, 6.7초) |
| 시나리오 | 10m/s 직진 2초 → 좌회전(R=30m) → 직진 |
| 프레임 내용 | ego 위치 + 50개 궤적 포인트 (글로벌 좌표) + velocity |

**프레임 구조 (JSON):**
```json
{
  "time": 2.0,
  "ego_position": { "x": 20.0, "y": 0.0, "yaw": 0.0 },
  "trajectory": [
    { "x": 21.0, "y": 0.0, "yaw": 0.033 },
    ...
  ],
  "target_velocity_per_point": [10.0, 10.0, ...],
  "target_speed": 0.0,
  "drive_mode": true,
  "turn_signal": 1,
  "sizeof_trajectory": 50
}
```

---

## 2. Sender (Orin → openpilot)

**파일:** `adcm_trajectory_sender.py`

**실행:**
```bash
python3 adcm_trajectory_sender.py --ip COMMA_IP --port 5005
```

**역할:**
- JSON 프레임을 순차적으로 읽음
- 각 프레임을 1659 bytes UDP 패킷으로 pack
- 20Hz로 openpilot 디바이스에 전송

**UDP 패킷 구조 (1659 bytes):**

| 영역 | 크기 | 내용 | 형식 |
|------|------|------|------|
| Header | 16B | magic(0x41444301) + seq + timestamp | uint32 + uint32 + float64 |
| Points | 1600B | 50 x (x, y, yaw, velocity) | float64 x 4 x 50 |
| Ego | 24B | ego_x, ego_y, ego_yaw | float64 x 3 |
| Meta | 18B | target_accel, drive_mode, emergency, turn_signal | float64 + bool + float64 + uint8 |
| Footer | 1B | sizeof_trajectory | uint8 |

**핵심:** 글로벌 좌표 + 거리 기반 그대로 전송 (변환은 수신 측에서)

---

## 3. udp_bridge.py (openpilot 내부, modeld 대체)

**파일:** `selfdrive/modeld/udp_bridge.py`

**역할:** ADCM 원본 궤적을 수신하여 openpilot modelV2 형식으로 변환 후 publish

### 변환 파이프라인 (6단계)

#### Step 1: 패킷 파싱
```
UDP 1659 bytes → 50개 포인트 (x, y, yaw, velocity) + ego + metadata
```

#### Step 2: 좌표 변환 (글로벌 → 차량 기준)
```
dx = px - ego_x
dy = py - ego_y
rel_x = dx * cos(-ego_yaw) - dy * sin(-ego_yaw)    # 전방 거리
rel_y = dx * sin(-ego_yaw) + dy * cos(-ego_yaw)    # 좌측 거리
rel_yaw = pyaw - ego_yaw                            # 상대 heading
```

#### Step 3: 시간축 생성 (거리 + 속도 → 시간)
```
ds = 두 점 사이 거리 (~1m)
v_avg = (vel[i] + vel[i+1]) / 2
dt = ds / v_avg
cum_time = [0, 0.1, 0.2, ... , ~5.0]    # 50개 포인트의 누적 시간
```

> velocity가 있어야 이 변환이 가능. 없으면 시간축 생성 불가.

#### Step 4: T_IDXS 33개로 보간
```
T_IDXS = [0, 0.01, 0.04, 0.09, 0.16, ... , 10.0]    # 이차 간격, 33개

position_x   = np.interp(T_IDXS, cum_time, rel_x)
position_y   = np.interp(T_IDXS, cum_time, rel_y)
velocity     = np.interp(T_IDXS, cum_time, vel)
yaw          = np.interp(T_IDXS, cum_time, rel_yaw)
```

#### Step 5: 미분값 계산
```
velocity_x     = velocity * cos(yaw)       # 전방 속도 성분
velocity_y     = velocity * sin(yaw)       # 횡방향 속도 성분
acceleration_x = gradient(velocity_x, T_IDXS)
acceleration_y = gradient(velocity_y, T_IDXS)
yaw_rate       = gradient(yaw, T_IDXS)
```

#### Step 6: action 계산 + modelV2 publish
```
desiredCurvature     = yaw 기반 curvature 계산
desiredAcceleration  = ADCM의 target_accel 값 사용
shouldStop           = drive_mode==false or n_valid==0
```

### 발행 메시지

| 메시지 | 용도 |
|--------|------|
| `modelV2` | plannerd가 소비 (position, velocity, acceleration, orientation, action) |
| `drivingModelData` | 경로 다항식 계수 (UI 등) |
| `cameraOdometry` | calibrationd/locationd 유지용 dummy |

---

## 4. plannerd (기존 openpilot 코드, 수정 없음)

**파일:** `selfdrive/controls/lib/longitudinal_planner.py`

**입력:** `modelV2` 메시지

**처리:**
1. `modelV2.position.x / velocity.x / acceleration.x` 읽음
2. MPC 솔버 실행 → 최적 속도/가속도 궤적 계산
3. `aTarget` (목표 가속도) 계산
4. `shouldStop` 판단
5. `desiredCurvature` = `modelV2.action.desiredCurvature` 전달

**출력:** `longitudinalPlan` 메시지

| 필드 | 타입 | 설명 |
|------|------|------|
| `aTarget` | float | 목표 가속도 (m/s^2) |
| `shouldStop` | bool | 정지 여부 |
| `desiredCurvature` | float | 목표 곡률 (rad/m) |
| `speeds` | float[17] | 속도 궤적 |
| `accels` | float[17] | 가속도 궤적 |
| `jerks` | float[17] | 저크 궤적 |
| `allowBrake` | bool | 브레이크 허용 |
| `allowThrottle` | bool | 가속 허용 |

---

## 5. controlsd (기존 openpilot 코드, 수정 없음)

**파일:** `selfdrive/controls/controlsd.py`

**입력:** `longitudinalPlan` 메시지

**처리:**
- **종방향:** `aTarget` → LongControl PID → `actuators.accel`
- **횡방향:** `desiredCurvature` → `clip_curvature()` → LatControl → `actuators.steeringAngle`

**출력:** `carControl` → pandad → CAN bus → 차량 (조향 + 가감속)

---

## 전체 데이터 흐름도

```
generate_test_trajectory.py
        |
        v
  JSON (134 프레임, 50pts x 글로벌좌표 + velocity)
        |
        v
adcm_trajectory_sender.py  --UDP 1659B, 20Hz-->  udp_bridge.py
 (Orin)                                           (openpilot, modeld 대체)
                                                        |
                                                  [1] 패킷 파싱
                                                  [2] 좌표 변환 (글로벌 -> 차량기준)
                                                  [3] 시간축 생성 (거리/속도 -> 시간)
                                                  [4] T_IDXS 33개로 보간
                                                  [5] velocity/accel/yaw_rate 계산
                                                  [6] action (curvature, accel) 계산
                                                        |
                                                        v
                                                  modelV2 publish
                                                        |
                                                        v
                                                  plannerd (MPC)
                                                   - aTarget 계산
                                                   - desiredCurvature 전달
                                                        |
                                                        v
                                                  controlsd
                                                   - 종방향: PID(aTarget)
                                                   - 횡방향: LatControl(curvature)
                                                        |
                                                        v
                                                  pandad -> CAN -> 차량
```

---

## 파일 목록

| 파일 | 위치 | 역할 |
|------|------|------|
| `generate_test_trajectory.py` | Orin `/home/a/orin/work/` | 테스트 경로 JSON 생성 |
| `test_trajectory_left_turn.json` | Orin `/home/a/orin/work/` | 생성된 테스트 경로 데이터 |
| `adcm_trajectory_sender.py` | Orin `/home/a/orin/work/` | JSON → UDP 패킷 전송 |
| `udp_bridge.py` | openpilot `selfdrive/modeld/` | ADCM UDP 수신 → modelV2 변환/발행 |
| `longitudinal_planner.py` | openpilot `selfdrive/controls/lib/` | modelV2 → MPC → longitudinalPlan (수정 없음) |
| `controlsd.py` | openpilot `selfdrive/controls/` | longitudinalPlan → 차량 제어 (수정 없음) |
| `process_config.py` | openpilot `system/manager/` | udp_bridge 활성화 설정 |

---

## 핵심 포인트

1. **ADCM은 원본 그대로 전송** — 좌표 변환, 시간축 생성은 openpilot 측에서 수행
2. **velocity 필수** — 시간축 생성에 포인트별 속도가 꼭 필요 (실차 적용 시 ADCM 패킷 확장 필요)
3. **plannerd/controlsd 수정 없음** — 기존 openpilot MPC + PID 그대로 활용
4. **modeld만 대체** — udp_bridge.py가 modeld 역할을 대신함

---

## 실차 적용 시 추가 작업

| 작업 | 위치 | 내용 |
|------|------|------|
| UDP 활성화 | Orin `adcm_global_config.json` | `"UDP": true` |
| velocity 추가 | Orin `Control_autosar.cpp` | `point.target_velocity`를 패킷에 포함 |
| comma IP 추가 | Orin `driving_trajectory_provider.cpp` | UDP 전송 타겟에 comma 디바이스 추가 |
| 네트워크 연결 | 물리 | Orin과 comma를 같은 서브넷 (192.168.1.x) 연결 |
| 좌표계 검증 | 실차 | ADCM 글로벌 좌표계 방향 확인 (toRelative 함수의 heading+PI 처리) |
