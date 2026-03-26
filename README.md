# openpilot External AI Control Fork

외부 AI(AlphaMayo)가 생성한 주행 경로를 실제 차량(Hyundai Ioniq 5)에서 추종하도록 openpilot의 제어 모듈만을 활용하는 프로젝트입니다.

> openpilot의 인지/판단 기능을 제거하고, 외부 PC에서 수신한 경로 명령을 CAN 통신을 통해 차량의 횡/종방향을 직접 제어하는 구조로 재설계하였습니다.

## 프로젝트 배경

AI 기반 모델이 외부 PC에서 주행 경로를 생성하면, 차량 내 제어 모듈이 이를 바탕으로 조향 및 가감속 명령을 계산하고 CAN 통신을 통해 실제 차량의 횡/종방향을 제어하는 구조입니다.

개발 초기 곡선 구간에서 차량이 목표 경로를 제대로 추종하지 못하는 문제가 발생했습니다. 처음에는 단순한 제어기 튜닝 문제라고 생각했지만, 원인을 추적한 결과 경로 생성 모델이 단일 바퀴 기반의 이상적 운동 모델(Bicycle Model)을 사용하고 있어 실제 차량의 조향 한계와 속도별 횡가속도 한계를 충분히 반영하지 못한다는 점을 확인했습니다.

이를 해결하기 위해 CAN 분석 도구를 활용해 차량의 최대 조향 토크와 속도별 제어 한계를 미리 파악하고, 미래 경로의 곡률이 차량 한계를 초과할 경우 해당 코너 진입 전 선제적으로 감속하도록 로직을 재설계하여 곡선 추종 성능을 개선했습니다.

## 시스템 아키텍처

```
┌─────────────────────┐          UDP (20Hz)          ┌──────────────────────┐
│   External PC       │  ──────────────────────────▶  │   Comma 3X Device    │
│                     │   curvature + accel + stop    │                      │
│  AlphaMayo (AI)     │                               │  ext_controllerd     │
│  경로 생성 & 판단    │                               │  (UDP → cereal msg)  │
└─────────────────────┘                               │         │            │
                                                      │         ▼            │
                                                      │  controlsd           │
                                                      │  (PID 횡/종 제어)     │
                                                      │         │            │
                                                      │         ▼            │
                                                      │  pandad              │
                                                      │  (CAN-FD 통신)       │
                                                      └────────┬─────────────┘
                                                               │ CAN Bus
                                                               ▼
                                                      ┌──────────────────────┐
                                                      │  Hyundai Ioniq 5     │
                                                      │  - 조향 (0x340)       │
                                                      │  - 가감속 (0x421)     │
                                                      └──────────────────────┘
```

### 역할 분리

| 구분 | 담당 | 설명 |
|------|------|------|
| **인지/판단** | 외부 PC (AlphaMayo) | 카메라/센서 데이터 기반 경로 생성, 장애물 판단 |
| **제어** | Comma 3X (openpilot) | 수신된 경로 명령을 PID 제어기로 변환, CAN 통신으로 차량 제어 |

## 주요 변경 사항

### 1. 제어 전용 모드 전환

openpilot의 자율주행 파이프라인에서 인지/판단 모듈을 제거하고 외부 명령 수신 모듈로 대체했습니다.

- `modeld` (AI 모델 추론) → **비활성화**, `ext_controllerd` (UDP 수신)로 대체
- `plannerd` (경로 계획) → **비활성화**, 외부 경로를 직접 사용
- `controlsd` → curvature 클리핑(±0.2)과 PID 제어만 수행하도록 단순화
- `selfdrived` → 외부 제어에 불필요한 검증 로직 비활성화 (FCW, 차선 변경, 과잉 조작 검사 등)

### 2. UDP 통신 프로토콜

외부 PC와 Comma 장치 간 20Hz UDP 통신을 구현했습니다.

**패킷 구조 (25 bytes, little-endian):**

```
Header (16 bytes):
  uint32  magic       = 0x4F505431 ("OPT1")
  uint32  seq         = 시퀀스 번호
  float64 timestamp   = 타임스탬프

Payload (9 bytes):
  float32 curvature       = 목표 곡률 (−0.2 ~ +0.2)
  float32 acceleration    = 목표 가속도 (m/s²)
  uint8   should_stop     = 정지 여부 (0 or 1)
```

- **타임아웃 처리**: 500ms 이상 패킷 미수신 시 안전 기본값으로 복귀 (정지, 가속도 0, 곡률 0)

### 3. 차량 인터페이스 수정 (Hyundai Ioniq 5)

실제 차량의 조향 한계를 확장하기 위해 다음을 변경했습니다.

| 파라미터 | 기존값 | 변경값 | 설명 |
|---------|--------|--------|------|
| `STEER_MAX` | 270 | 800 | 최대 조향 토크 |
| `STEER_DRIVER_ALLOWANCE` | 250 | 350 | 운전자 토크 허용 범위 |
| `STEER_DELTA_UP` | 2 | 5 | 조향 증가 속도 |
| `STEER_DELTA_DOWN` | 3 | 10 | 조향 감소 속도 |

### 4. Panda 펌웨어 안전 검사 수정

외부 제어 테스트를 위해 Panda 펌웨어의 Hyundai 안전 검사를 수정했습니다. (CAN-FD 포함)

> **경고**: 현재 테스트 목적으로 안전 검사가 비활성화되어 있습니다. 공도 주행에는 절대 사용하지 마십시오.

## 프로젝트 구조

```
controller/                          # 외부 PC에서 실행하는 송신 도구
├── trajectory_sender.py             # 실시간 인터랙티브 명령 송신기
├── move_sender.py                   # JSON 궤적 파일 재생 송신기
└── trajectory_right_turn.json       # 우회전 테스트 궤적 데이터

selfdrive/
├── controls/
│   ├── controlsd.py                 # [수정] 횡/종방향 PID 제어 (외부 curvature 사용)
│   └── ext_controllerd.py           # [추가] UDP 수신 → cereal 메시지 변환
├── modeld/
│   └── udp_bridge.py                # [추가] UDP 수신 → modelV2 메시지 변환 (대체 방식)
└── selfdrived/
    └── selfdrived.py                # [수정] 외부 제어 모드용 검증 로직 비활성화

system/manager/
└── process_config.py                # [수정] 프로세스 구성 (modeld→ext_controllerd)

opendbc_repo/opendbc/car/hyundai/
└── values.py                        # [수정] 조향 토크 한계 확장

cereal/
└── log.capnp                        # [수정] desiredCurvature 필드 추가

panda/board/safety/
├── safety_hyundai.h                 # [수정] 안전 검사 비활성화
└── safety_hyundai_canfd.h           # [수정] CAN-FD 안전 검사 비활성화
```

## 사용 방법

### 1. Comma 장치 설정

Comma 3X에 이 저장소의 `communication` 브랜치를 설치합니다.

### 2. 외부 PC에서 명령 전송

**인터랙티브 모드** (수동 테스트용):

```bash
python controller/trajectory_sender.py
```

사용 가능한 명령:
- `right <곡률>` / `left <곡률>` — 우/좌회전 (예: `right 0.02`)
- `straight` — 직진
- `accel <값>` — 가속도 설정 (m/s²)
- `stop` / `go` — 정지/출발

**궤적 재생 모드** (사전 계획된 경로 추종):

```bash
python controller/move_sender.py trajectory_right_turn.json
```

### 3. 네트워크 설정

- Comma 장치 IP: `10.200.147.253` (기본값)
- UDP 포트: `5005`
- 전송 주기: 20Hz

## 개발 과정에서 해결한 문제들

### 곡선 구간 경로 추종 실패
- **원인**: 경로 생성 모델의 Bicycle Model이 실제 차량의 조향 한계와 속도별 횡가속도 한계를 미반영
- **해결**: CAN 분석으로 차량의 최대 조향 토크/속도별 제어 한계를 파악하고, 곡률 초과 시 코너 진입 전 선제 감속 로직 적용

### Steering Misalignment 에러로 인한 Engage 불가
- **원인**: openpilot이 조향 정렬 오류를 감지하여 제어 진입을 차단
- **해결**: 외부 제어 모드에서는 해당 에러를 무시하도록 설정

### logmessaged 프로세스 에러로 인한 Engage 불가
- **원인**: logmessaged 프로세스 실패 시 전체 시스템이 Engage를 거부
- **해결**: 해당 프로세스의 실패를 무시하도록 설정

### EPS Fault 방지
- **원인**: 90도 이상 급격한 조향 시 EPS 폴트 발생
- **해결**: 토크 변화율 제한(STEER_DELTA_UP/DOWN) 조정으로 안정화

## 대상 차량

- **차종**: Hyundai Ioniq 5
- **통신**: CAN-FD
- **하드웨어**: Comma 3X + Car Harness

## 주의 사항

- 이 프로젝트는 **연구/테스트 목적**으로만 사용해야 합니다.
- Panda 안전 검사가 비활성화되어 있어 **공도 주행에 절대 사용하지 마십시오**.
- 반드시 안전한 폐쇄 구역에서만 테스트하십시오.
- 테스트 시 항상 운전자가 탑승하여 즉시 개입할 수 있어야 합니다.

## 기반

[commaai/openpilot](https://github.com/commaai/openpilot) `nightly-dev` 브랜치 기반 (v0.11.0)

## 라이선스

MIT License — 원본 openpilot 라이선스를 따릅니다.
