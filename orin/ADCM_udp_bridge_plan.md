# ADCM UDP Bridge - modeld 대체 계획

## Context

Orin의 ADCM Planning이 생성한 driving trajectory를 openpilot으로 전달하여 차량을 제어하려 한다.
modeld(신경망)를 `udp_bridge.py`로 대체하고, 기존 plannerd/controlsd 파이프라인을 수정 없이 그대로 활용한다.
Orin에서 UDP로 전송한 ADCM 궤적 패킷을 수신 -> 좌표 변환 -> modelV2 메시지로 변환/발행하는 bridge를 구현한다.

---

## 수정/생성 파일 목록

| 파일 | 작업 | 설명 |
|------|------|------|
| `selfdrive/modeld/udp_bridge.py` | **신규 생성** | ADCM UDP 수신 -> modelV2 변환/발행 (핵심) |
| `system/manager/process_config.py` | **수정** | modeld -> udp_bridge로 직접 교체 |

---

## Step 1: `udp_bridge.py` 생성

### 1-1. UDP 수신부

- `socket.socket(AF_INET, SOCK_DGRAM)` 바인딩 (port 5005)
- non-blocking 또는 timeout 설정 (50ms) -> 패킷 없으면 마지막 패킷 재사용
- 1659 bytes 패킷 파싱 (struct.unpack):

```
Header (16B): magic(uint32) + seq(uint32) + timestamp(float64)
Points (1600B): 50 x (x, y, yaw, velocity) as float64 x 4
Ego (24B): ego_x, ego_y, ego_yaw as float64 x 3
Meta (18B): target_accel(float64) + drive_mode(bool) + emergency(float64) + turn_signal(uint8)
Footer (1B): sizeof_trajectory (uint8)
```

### 1-2. 좌표 변환 (글로벌 -> 차량 기준)

```python
dx = px - ego_x
dy = py - ego_y
rel_x = dx * cos(-ego_yaw) - dy * sin(-ego_yaw)    # 전방 거리
rel_y = dx * sin(-ego_yaw) + dy * cos(-ego_yaw)    # 좌측 거리
rel_yaw = pyaw - ego_yaw                            # 상대 heading
```

### 1-3. 시간축 생성 (거리 + 속도 -> 시간)

- 연속 두 점 사이 거리 `ds = sqrt(dx^2 + dy^2)`
- 평균 속도 `v_avg = (vel[i] + vel[i+1]) / 2`
- `dt = ds / max(v_avg, 0.1)`
- 누적 시간 배열 생성

### 1-4. T_IDXS 33개로 보간

- `ModelConstants.T_IDXS` (0 ~ 10초, 이차 간격, 33개) 사용
- `np.interp(T_IDXS, cum_time, rel_x/rel_y/rel_yaw/velocity)`

### 1-5. 미분값 계산

```python
velocity_x = velocity * cos(yaw)
velocity_y = velocity * sin(yaw)
acceleration_x = np.gradient(velocity_x, T_IDXS)
acceleration_y = np.gradient(velocity_y, T_IDXS)
yaw_rate = np.gradient(yaw, T_IDXS)
```

### 1-6. action 계산

- 기존 `get_accel_from_plan()`, `get_curvature_from_plan()` 재사용
- ADCM의 target_accel, drive_mode 활용

### 1-7. modelV2 메시지 발행

PubMaster로 3개 메시지 발행:

#### modelV2 필수 필드:
| 필드 | 값 |
|------|-----|
| `position.x/y/z` | 보간된 rel_x, rel_y, 0 |
| `velocity.x/y/z` | velocity_x, velocity_y, 0 |
| `acceleration.x/y/z` | acceleration_x, acceleration_y, 0 |
| `orientation.x/y/z` | 0, 0, yaw |
| `orientationRate.x/y/z` | 0, 0, yaw_rate |
| `action.desiredCurvature` | get_curvature_from_plan() |
| `action.desiredAcceleration` | get_accel_from_plan() |
| `action.shouldStop` | drive_mode==false or n_valid==0 |
| `laneLines/roadEdges/leadsV3` | dummy (prob=0) |
| `meta.*` | 안전 기본값 |
| `confidence` | green |

#### drivingModelData:
- path 다항식 계수 (fill_xyz_poly)

#### cameraOdometry:
- dummy 값 (calibrationd/locationd 유지)

### 재사용 기존 코드:
- `ModelConstants.T_IDXS` -- `selfdrive/modeld/constants.py:9`
- `get_accel_from_plan()` -- `selfdrive/controls/lib/drive_helpers.py:42`
- `get_curvature_from_plan()` -- `selfdrive/controls/lib/drive_helpers.py:62`
- `smooth_value()` -- `selfdrive/controls/lib/drive_helpers.py:21`
- `fill_xyzt()` -- `selfdrive/modeld/fill_model_msg.py:18`
- `fill_xyz_poly()` -- `selfdrive/modeld/fill_model_msg.py:45`

---

## Step 2: `process_config.py` 수정

modeld 프로세스를 udp_bridge로 직접 교체:

```python
# 기존: PythonProcess("modeld", "selfdrive.modeld.modeld", only_onroad),
# 변경:
PythonProcess("modeld", "selfdrive.modeld.udp_bridge", only_onroad),
```

---

## 주의사항

1. plannerd 필수 필드: position.x, velocity.x, acceleration.x, action.desiredAcceleration, action.shouldStop
2. controlsd 필수 필드: action.desiredCurvature, meta.laneChangeState
3. 20Hz 발행 필수 -- plannerd가 modelV2를 polling
4. cameraOdometry dummy 필수 -- calibrationd/locationd 의존

---

## 검증 방법

1. openpilot 실행 후 Orin에서 `adcm_trajectory_sender.py` 실행
2. `selfdrive/debug/dump.py --modelV2` 로 메시지 수신 확인
3. plannerd -> longitudinalPlan, controlsd -> carControl 발행 확인
4. 실차 테스트
