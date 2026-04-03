# openpilot External AI Control Fork

외부 AI(AlphaMayo)가 생성한 주행 경로를 실제 차량(Hyundai Ioniq 5)에서 추종하도록 openpilot의 제어 모듈만을 활용하는 프로젝트입니다.

openpilot의 인지/판단 기능을 제거하고, 외부 PC에서 수신한 경로 명령을 CAN 통신을 통해 차량의 횡/종방향을 직접 제어하는 구조로 재설계하였습니다.

## 프로젝트 배경

AI 기반 모델(AlphaMayo)이 외부 PC에서 주행 경로를 생성하면, 차량 내 comma 디바이스가 이를 UDP로 수신하여 조향 및 가감속 명령을 계산하고, CAN 통신을 통해 실제 차량의 횡/종방향을 제어하는 구조입니다.

openpilot은 본래 자체 카메라 기반 인지(modeld)와 경로 판단을 내장하고 있지만, 본 프로젝트에서는 이를 의도적으로 분리하여 **인지/판단은 외부 AI(AlphaMayo)가 담당**하고, **openpilot은 순수 제어 실행기**로만 동작하도록 재설계했습니다. 이를 통해 외부에서 개발된 자율주행 알고리즘을 실차 환경에서 빠르게 검증할 수 있는 플랫폼을 구축했습니다.

### 곡선 구간 경로 추종 문제 분석 및 개선

개발 초기, 직선 구간에서는 경로를 안정적으로 추종했으나 **곡선 구간에서 차량이 목표 경로를 이탈하는 문제**가 발생했습니다. 처음에는 단순한 제어기 튜닝 문제로 판단했지만, CAN 데이터와 PlotJuggler를 활용해 원인을 추적한 결과 다음 두 가지 근본 원인을 확인했습니다:

1. **횡가속도 한계 초과**: 곡선 구간에서 차량의 횡가속도가 안정 한계인 **3 m/s²을 초과**하여 타이어 그립 한계에 도달, 제어 명령 대비 실제 차량 거동이 괴리
2. **경로 생성 모델의 한계**: AlphaMayo가 사용하는 Bicycle Model 기반 경로 생성이 실제 차량의 속도별 조향 한계와 최대 조향 토크를 충분히 반영하지 못함

이를 해결하기 위해 다음과 같은 **선제적 감속 로직**을 설계하였습니다:

- CAN 분석 도구(cabana)를 활용하여 **속도별 최대 조향 토크 및 횡가속도 한계를 사전 파악**
- 수신된 미래 경로의 곡률(curvature)을 사전 스캔하여, 현재 속도 기준으로 **횡가속도가 3 m/s²을 초과할 것으로 예측되는 구간을 검출**
- 해당 코너 진입 **이전에 선제적으로 감속**하여 곡률 추종이 가능한 속도까지 미리 낮추는 로직 적용
- openpilot의 MPC(Model Predictive Control) 기반 종방향 제어와 연계하여 급감속 없이 부드러운 속도 프로파일 생성

이 개선을 통해 단순히 외부 경로를 "그대로 따라가는" 수준을 넘어, **차량 동역학 한계를 고려한 안전한 경로 추종**을 실현하였으며, 실차 시험에서 곡선 구간의 경로 이탈 문제를 해결했습니다.

## 시스템 아키텍처

```
┌─────────────────────┐          UDP:5005          ┌──────────────────────────────────┐
│  외부 PC (AlphaMayo) │  ──────────────────────→   │     comma 디바이스 (openpilot)    │
│                     │   50개 궤적점              │                                  │
│  · AI 경로 생성      │   (x, y, yaw, velocity)    │  ┌──────────┐   ┌──────────┐     │
│  · 글로벌 좌표 출력   │   + 차량 상태              │  │  modeld   │   │udp_bridge│     │
│                     │   + 메타데이터              │  │(카메라전용)│   │(경로변환) │     │
└─────────────────────┘                            │  └─────┬────┘   └─────┬────┘     │
                                                   │        │              │           │
                                                   │  cameraOdometry   modelV2 (20Hz) │
                                                   │        │        drivingModelData  │
                                                   │        └──────┬───────┘           │
                                                   │               ↓                   │
                                                   │  ┌──────────────────────┐         │
                                                   │  │  controlsd / plannerd │         │
                                                   │  │  (횡/종방향 제어 실행)  │         │
                                                   │  └──────────┬───────────┘         │
                                                   │             ↓                     │
                                                   │     panda (CAN 인터페이스)         │
                                                   │             ↓                     │
                                                   │   Hyundai Ioniq 5 (조향 + 가감속)  │
                                                   └──────────────────────────────────┘
```

### 핵심 설계 원칙

- **인지/판단 분리**: openpilot의 인지(modeld) 기능을 카메라 오도메트리 전용으로 축소하고, 판단은 외부 AI(AlphaMayo)에 위임
- **모듈 분리**: modeld는 cameraOdometry만 발행, udp_bridge가 외부 경로를 modelV2/drivingModelData로 변환 발행
- **인터페이스 호환**: controlsd, plannerd는 수정 없이 그대로 사용 (openpilot 표준 메시지 포맷 준수)
- **실시간성 보장**: 20Hz 루프 주기 유지, non-blocking UDP 수신
- **차량 동역학 반영**: 속도별 횡가속도 한계를 고려한 선제적 감속 로직

## 주요 구현 내용

### 1. UDP 통신 프로토콜 (`selfdrive/modeld/udp_bridge.py`)

외부 PC(AlphaMayo)로부터 1659바이트 UDP 패킷을 수신하여 파싱합니다.

| 영역 | 크기 | 내용 |
|------|------|------|
| Header | 16B | Magic(0x41444301) + Sequence + Timestamp |
| Path Points | 1600B | 50개 궤적점 x 4 float64 (x, y, yaw, velocity) |
| Ego State | 24B | 차량 현재 위치 (x, y, yaw) |
| Metadata | 18B | 목표 가속도, 주행 모드, 비상 상태, 방향 지시등 |
| Footer | 1B | 유효 포인트 수 |

### 2. 좌표 변환

AlphaMayo가 글로벌 좌표계로 출력한 궤적을 차량 기준 상대 좌표로 변환합니다.

```python
# 회전 행렬을 사용한 글로벌 → 차량 좌표 변환
dx = points[:, 0] - ego['x']
dy = points[:, 1] - ego['y']
c = np.cos(-ego['yaw'])
s = np.sin(-ego['yaw'])
rel_x = dx * c - dy * s
rel_y = dx * s + dy * c
```

### 3. 시간축 보간

수신된 50개 raw point를 openpilot 표준 33개 timestep(`T_IDXS`)으로 보간합니다.

- 거리/속도 기반 누적 시간 배열 생성
- `np.interp`를 사용하여 T_IDXS 기준으로 x, y, yaw, velocity 보간
- 보간된 데이터로부터 velocity, acceleration, yaw_rate 미분값 계산

### 4. 제어 명령 계산

보간된 경로로부터 종방향/횡방향 제어 명령을 산출합니다.

- **종방향**: 속도 프로파일로부터 `desiredAcceleration` 계산, 스무딩 적용 (τ = 0.3s)
- **횡방향**: yaw/yaw_rate로부터 `desiredCurvature` 계산
- **선제적 감속**: 미래 경로 곡률 스캔 → 횡가속도 3 m/s² 초과 예측 시 코너 진입 전 감속
- **안전 로직**: drive_mode OFF 또는 유효 포인트 부재 시 `shouldStop = True`

### 5. 메시지 발행

openpilot 표준 메시지 포맷으로 변환하여 20Hz로 발행합니다.

- **modelV2**: position, velocity, acceleration, orientation, action, lane lines(dummy), leads(dummy)
- **drivingModelData**: 경로 다항식 계수 (3차), action, lane line meta

## 프로세스 구성

| 프로세스 | 역할 | 발행 메시지 |
|----------|------|------------|
| `modeld` | 카메라 inference (camera-only mode) | cameraOdometry |
| `udp_bridge` | 외부 경로 수신, 변환, 곡률 기반 감속 판단 | modelV2, drivingModelData |
| `controlsd` | 횡/종방향 제어 실행 (미수정) | carControl |
| `plannerd` | MPC 기반 종방향 계획 (미수정) | longitudinalPlan |

프로세스 등록: `system/manager/process_config.py`

```python
PythonProcess("modeld", "selfdrive.modeld.modeld", only_onroad),
PythonProcess("udp_bridge", "selfdrive.modeld.udp_bridge", only_onroad),
```

## 개발 이력 및 문제 해결

### 곡선 구간 경로 추종 실패 → 선제적 감속 로직 도입
- **문제**: 곡선 구간에서 횡가속도가 3 m/s²을 초과하여 차량이 경로를 이탈
- **원인 분석**: CAN 데이터 분석(cabana)으로 속도별 최대 조향 토크와 횡가속도 한계를 파악, AlphaMayo 경로가 차량 동역학 한계를 미반영
- **해결**: 미래 경로 곡률을 사전 스캔하여 한계 초과 구간 진입 전 MPC 연계 감속 로직 적용

### 발행 주기 동기화 문제
- **문제**: UDP 수신 대기(blocking)로 인해 modelV2 발행 주기가 불안정하여 controlsd/plannerd가 기대하는 20Hz를 충족하지 못함
- **해결**: `sock.setblocking(False)` + 독립 타이밍 루프로 전환하여 패킷 수신 여부와 무관하게 20Hz 발행 유지

### modeld/udp_bridge 역할 분리
- **문제**: 초기에 modeld를 완전히 대체하려 했으나 cameraOdometry가 누락되어 locationd에서 오류 발생
- **해결**: modeld는 카메라 기반 cameraOdometry만 발행, udp_bridge는 경로 메시지만 발행하도록 분리

### Lane State Enum 오류
- **문제**: lane line 상태값에 잘못된 enum을 사용하여 controlsd에서 예외 발생
- **해결**: openpilot 내부 enum 정의(`log.capnp`)를 확인하여 올바른 상태값으로 수정

## 시험 및 데이터 분석

PlotJuggler를 활용하여 실시간 데이터 모니터링 및 제어 성능 분석을 수행합니다.

### 주요 모니터링 항목

| 항목 | 메시지 | 용도 |
|------|--------|------|
| 실제 가속도 | `carState.aEgo` | 종방향 추종 성능 확인 |
| 요청 가속도 | `carControl.actuatorsOutput.accel` | 명령값 대비 실제값 비교 |
| 횡가속도 | `carState.aEgo` (횡방향 성분) | 곡선 구간 안정성 확인 (3 m/s² 한계) |
| 목표 곡률 | `modelV2.action.desiredCurvature` | 횡방향 제어 입력 확인 |
| 차량 속도 | `carState.vEgo` | 속도 프로파일 추종 및 감속 로직 검증 |
| 조향각 | `carState.steeringAngleDeg` | 횡방향 추종 성능 확인 |

## 설정 파라미터

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `UDP_PORT` | 5005 | 외부 PC 경로 수신 포트 |
| `PACKET_SIZE` | 1659 | UDP 패킷 크기 |
| `MAGIC` | 0x41444301 | 패킷 검증 매직 넘버 |
| `MAX_POINTS` | 50 | 최대 궤적 포인트 수 |
| `LONG_SMOOTH_SECONDS` | 0.3 | 종방향 가속도 스무딩 시정수 |
| `LAT_SMOOTH_SECONDS` | 0.0 | 횡방향 곡률 스무딩 시정수 |
| `MODEL_RUN_FREQ` | 20Hz | 메시지 발행 주기 |

## 기술 스택

- **차량**: Hyundai Ioniq 5
- **플랫폼**: comma 3X + openpilot (Python/C++)
- **외부 AI**: AlphaMayo (경로 생성)
- **통신**: UDP 소켓 (비동기 수신)
- **데이터 처리**: NumPy (좌표 변환, 보간, 미분)
- **메시지 직렬화**: Cap'n Proto (cereal)
- **CAN 분석**: cabana, PlotJuggler
- **차량 인터페이스**: panda (CAN)

## 디렉토리 구조 (주요 수정 파일)

```
openpilot/
├── selfdrive/
│   ├── modeld/
│   │   ├── modeld.py           # 카메라 전용 모드로 수정 (cameraOdometry만 발행)
│   │   └── udp_bridge.py       # [신규] 외부 경로 수신, 변환, 곡률 기반 감속 판단
│   └── controls/
│       ├── controlsd.py        # 미수정 (modelV2 소비, 횡/종방향 제어 실행)
│       └── plannerd.py         # 미수정 (modelV2 소비, MPC 종방향 계획)
└── system/
    └── manager/
        └── process_config.py   # udp_bridge 프로세스 등록
```

## 기반 프로젝트

이 프로젝트는 [comma.ai의 openpilot](https://github.com/commaai/openpilot)을 기반으로 합니다.
