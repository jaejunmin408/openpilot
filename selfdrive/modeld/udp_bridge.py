#!/usr/bin/env python3
"""
Alpamayo UDP Bridge — reference path slice 추종 모드

외부 publisher가 ~10 Hz로 차량 앞 ~20 m 분량의 reference path slice를
ego-frame JSON으로 보내준다.

  packet: {"pred_xyz": [[x, y, z], ...]}   # (N,3) list
  수신 좌표계: x=forward(m), y=right(m)  ← publisher 측 규약
  내부 좌표계: x=forward(m), y=left(m)   ← openpilot body frame
  parse 시점에 y 부호를 뒤집어 내부적으로는 openpilot 규약으로 통일한다.

매 패킷이 그 시점 차량 위치 기준으로 잘려 들어오므로 anchor/world 변환 없이
받은 path를 그대로 ego-frame path로 사용한다.

횡제어(LAT_CONTROLLER):
  "mpc"          — alpasim LinearMPC 이식판 (기본값).
                   controls/lib/alpasim_mpc/ 참고. 전륜 조향각 δ 를 내고
                   κ = curvature_factor(v)·δ 로 desiredCurvature 로 환산한다.
  "pure_pursuit" — 기존 방식. 속도 비례 look-ahead L_d = clip(k·v, 10, 20).
                   MPC 해가 계속 실패할 때의 폴백으로도 쓰인다.

종방향은 TARGET_SPEED_MPS 유지 P 제어 (MPC 모드에서도 동일 —
MPC 의 accel_cmd 는 진단용으로만 로깅한다. 자세한 이유는 MPC_USE_ACCEL 참고).
"""
import datetime
import csv
import json
import math
import os
import socket
import time
import numpy as np

import cereal.messaging as messaging
from cereal import car, log
from cereal.messaging import PubMaster, SubMaster
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.alpasim_mpc import (
    EgoState,
    MPCGains,
    MPCPathTracker,
    SolverFailurePolicy,
    VehicleParameters,
)
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value
from openpilot.selfdrive.controls.lib.local_world import LocalWorld
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS

# ── 설정 ──────────────────────────────────────────────
UDP_PORT = 5005
LOCAL_PATH_VIZ_PORT = 5007   # 수신한 ref path slice를 ego-frame viz로 미러
VEHICLE_VIZ_PORT = 5006      # LocalWorld 현재 pose + 6초 trail (viz only)
WORLD_PATH_VIZ_PORT = 5008   # 수신 path 를 LocalWorld 기준 world frame 으로 변환해 송신 (viz only)
RECV_BUF_SIZE = 65535

T_IDXS = np.array(ModelConstants.T_IDXS, dtype=np.float64)
X_IDXS = np.array(ModelConstants.X_IDXS, dtype=np.float64)
IDX_N = ModelConstants.IDX_N   # 33

# ── 종방향 ────────────────────────────────────────────
TARGET_SPEED_MPS = 10.0 / 3.6
LON_KP = 0.3
ACCEL_MIN = -3.5
ACCEL_MAX = 2.0

MIN_LAT_CONTROL_SPEED = 0.0001          # 이 속도 이하에서는 직전 curvature 유지

CURV_DEADZONE = 0.00                  # |curvature| 이 이하면 0 으로 (직진 데드존)

# ── 곡률 상한 (제어기 공용 안전 한계) ────────────────
# 제어기별 튜닝값이 아니라 차량 한계다. MPC 와 pure pursuit 이 같은 값을 쓴다
# — pure pursuit 이 MPC 의 폴백이므로, 폴백으로 넘어가는 순간 더 급한 조향이
# 허용되면 안 된다.
#
# 이 fork 는 openpilot 의 ISO 제한(clip_curvature)이 bypass 되어 있어서
# (drive_helpers.py 참고) 이게 desiredCurvature 의 유일한 상한이다.
#
# 횡가속 = κ · v² 이므로 허용 위험도는 목표속도에 제곱으로 딸려간다:
#             κ=0.05(R=20m)   κ=0.10(R=10m)   κ=0.20(R=5m)
#   10 km/h     0.39 m/s²       0.77 m/s²       1.54 m/s²   ← 현재 설정
#   15 km/h     0.87 m/s²       1.74 m/s²       3.47 m/s²
#   30 km/h     3.47 m/s²       6.94 m/s²      13.9  m/s²   ← 속도 올릴 때 반드시 재검토
# TARGET_SPEED_MPS 를 올릴 때 이 값을 같이 내리지 않으면 위험해진다.
MAX_CURVATURE = 0.2

# ── pure pursuit 파라미터 ────────────────────────────
# 속도 비례 look-ahead: L_d = clip(PP_LOOKAHEAD_K_S * v_ego, MIN, MAX)
#   대중적 pure pursuit 방식(L_d ∝ v). 최소 10m, 최대 20m 로 clamp.
PP_LOOKAHEAD_MIN_M = 10.0
PP_LOOKAHEAD_MAX_M = 20.0
PP_LOOKAHEAD_K_S = 3.0               # look-ahead time gain (s) — L_d = k · v

# goal 점 선택 방식:
#   PP_INDEX_FRAC 이 None 이면 → 거리 기반(속도 비례 L_d 만큼 떨어진 점)
#   PP_INDEX_FRAC 이 [0.0, 1.0] 값이면 → 들어온 path 인덱스의 그 비율 지점을 goal 로
#   예) 0.5 → path 중간 인덱스, 1.0 → path 끝점, 0.0 → 첫 점
PP_INDEX_FRAC = None                 # None → 속도 비례 거리 기반 look-ahead 사용


def lookahead_for_speed(v_ego):
    """속도 비례 look-ahead 거리 L_d = clip(k · v, MIN, MAX)."""
    return float(np.clip(PP_LOOKAHEAD_K_S * max(v_ego, 0.0),
                         PP_LOOKAHEAD_MIN_M, PP_LOOKAHEAD_MAX_M))


# ── 횡제어기 선택 ────────────────────────────────────
LAT_CONTROLLER = os.environ.get("UDP_BRIDGE_LAT_CONTROLLER", "mpc")   # "mpc" | "pure_pursuit"

# ── alpasim MPC 파라미터 ─────────────────────────────
# horizon 이 내다보는 거리 = MPC_N_HORIZON · MPC_DT_MPC · v_ego.
#   alpasim 기본값 20 × 0.1 = 2.0 s 기준
#       10 km/h(2.78 m/s) → 5.6 m     ← 현재 설정
#       15 km/h(4.17 m/s) → 8.3 m
#   기존 pure pursuit 의 L_d(10~20 m)보다 짧다. 그래도 폐루프는 잘 붙는다
#   (원호 정상상태 횡오차 10 km/h: κ0.05 +0.056 m, κ0.10 +0.115 m, κ0.20 +0.249 m,
#    solver 실패 0 건, 정상상태 지령 κ 가 경로 κ 와 일치).
#
# 2.0 s 는 줄이면 안 된다. 폐루프 원호 정상상태 횡오차 실측
# (15 km/h, alpasim 기본 gain, -는 안쪽으로 파고듦 / +는 바깥쪽으로 벌어짐):
#       nh=20 dt=0.05 (1.0s) → κ0.02 +0.130  κ0.05 +0.339  κ0.10 +0.761  ← 나쁨
#       nh=20 dt=0.10 (2.0s) → κ0.02 -0.044  κ0.05 -0.103  κ0.10 -0.144  ← 기본값
#       nh=40 dt=0.10 (4.0s) → κ0.02 -0.005  κ0.05 -0.034  κ0.10 -0.175
#       nh=20 dt=0.20 (4.0s) → κ0.02 -0.024  κ0.05 -0.127  κ0.10 -0.478
# 늘려도 이득이 크지 않고(nh=40 이 약간 낫지만 solve 비용 2배) 큰 곡률에서는
# 오히려 나빠진다 — A 행렬을 현재 yaw 에 고정해 선형화하므로 예측 구간에서
# yaw 가 많이 변하면 모델이 틀리기 때문. 기본값 유지를 권한다.
MPC_N_HORIZON = 20
MPC_DT_MPC = 0.1

# alpasim configs/controller/default.yaml 과 동일한 gain.
#
#   idx_start_penalty=10 → 앞 10 스텝(=1.0 s)은 추종 코스트 미적용.
#     상태제약과 입력 코스트는 전 구간 유지. 지연/초기 오프셋 보상용이며
#     pure pursuit 의 look-ahead 를 코스트 마스크로 표현한 것에 가깝다.
#     곡선에서 살짝 안쪽으로 파고드는(corner cutting) 원인이고, 효과는
#     단조롭지만 크지 않다. 원호 정상상태 횡오차 실측:
#         isp= 1 (0.1s) → κ0.02 -0.029  κ0.05 -0.072  κ0.10 -0.116
#         isp= 5 (0.5s) → κ0.02 -0.030  κ0.05 -0.076  κ0.10 -0.121
#         isp=10 (1.0s) → κ0.02 -0.044  κ0.05 -0.103  κ0.10 -0.144  ← 기본값
#         isp=15 (1.5s) → κ0.02 -0.065  κ0.05 -0.139  κ0.10 -0.129
#     10 km/h 에서는 reach 가 짧아 부호가 뒤집혀 바깥쪽으로 벌어진다:
#         isp= 1 → κ0.05 +0.041  κ0.10 +0.085
#         isp= 5 → κ0.05 +0.042  κ0.10 +0.088
#         isp=10 → κ0.05 +0.056  κ0.10 +0.115  ← 기본값
#     직선 1.5 m 오프셋 수렴은 isp 와 무관하게 전부 0 에 붙는다
#     (수렴시간만 isp=3 → 1.95 s, isp=10 → 2.40 s 로 조금 차이).
#     어느 속도에서든 isp 를 내리면 오차가 줄지만 개선폭은 ~0.03 m 수준이다.
#
#   long_position_weight: 종제어를 외부 P 제어기에 맡긴 상태에서 종방향
#     reference 갭이 조향에 섞이는 게 싫으면 0.0 으로 두면 완전히 분리된다.
MPC_GAINS = MPCGains(
    long_position_weight=2.0,
    lat_position_weight=1.0,
    heading_weight=1.0,
    acceleration_weight=0.1,
    rel_front_steering_angle_weight=5.0,
    rel_acceleration_weight=1.0,
    idx_start_penalty=10,
)

# MPC 가 낸 accel_cmd 를 종방향에 쓸지. 기본 False.
#
# MPC 자체는 종·횡 결합 문제다 (상태 8개 중 x/vx/accel 이 종방향, 입력 2개 중
# 하나가 accel_cmd, 코스트에 long_position_weight=2.0). 그런데 이 구성에서는
# accel_cmd 를 버려도 조향이 전혀 달라지지 않는다. 이유 두 가지:
#
#  1) ego frame 이라 x0 의 yaw 가 항상 0 이고, yaw=0 에서 선형화하면
#     A[x,yaw] = -vx·sin(0) = 0, A[y,vx] = sin(0) = 0 이 되어 종/횡이 정확히
#     블록대각으로 분리된다. 즉 QP 가 두 개의 독립 문제로 쪼개지고,
#     long_position_weight 는 조향 출력에 비트단위로 영향이 없다.
#     (tests/test_linear_mpc.py TestLongitudinalLateralDecoupling 참고)
#  2) 수신 path 에는 시간/속도 정보가 없어서 reference 의 종방향 목표를
#     아래 P 제어기로 만들어 넣는다. 그러니 MPC 의 accel_cmd 는 그걸 되풀이한
#     값이다 — 실측 상관계수 1.00, 값도 거의 같다 (저속 출발 시 +0.210 vs +0.242).
#
# True 로 켜도 동작은 한다(실측: 목표 4.17 m/s 에 4.10 m/s 로 수렴 — 약간 처짐). 다만
# acceleration_weight 가 가속 상태를 0 으로 당겨서 정상상태 속도가 살짝 처진다.
MPC_USE_ACCEL = False

# MPC 해가 연속 실패할 때의 동작.
# alpasim 원본은 실패 시 u=0 을 낸다(=조향 0). 선회 중에 조향이 0 으로
# 스냅하면 실차에서는 위험하므로, 직전 값을 유지하다가 이 횟수를 넘기면
# pure pursuit 으로 폴백한다.
MPC_MAX_CONSECUTIVE_FAILS = 10        # 20 Hz 기준 0.5 s
MPC_FALLBACK_TO_PURE_PURSUIT = True

# 조향 액추에이터 시정수 [s]. alpasim 기본 0.1.
# liveDelay.lateralDelay 를 쓰고 싶으면 여기 대신 아래 tracker 생성부 참고.
MPC_STEERING_TIME_CONSTANT = 0.1

# CarParams 대기 한도 [s]. 넘으면 alpasim 기본 차량 파라미터(Ford Fusion)로 진행.
CARPARAMS_WAIT_S = 5.0

# ── 디버그 로그 ──────────────────────────────────────
DEBUG_LOG_DIR = os.path.join(BASEDIR, "logs")
DIAG_LOG_DIR = os.environ.get("UDP_BRIDGE_DIAG_LOG_DIR", DEBUG_LOG_DIR)
DIAG_LD_VALUES_M = tuple(
    sorted(
        {
            PP_LOOKAHEAD_MIN_M,
            PP_LOOKAHEAD_MAX_M,
            *(
                float(token)
                for token in os.environ.get("UDP_BRIDGE_DIAG_LDS_M", "").split(",")
                if token.strip()
            ),
        }
    )
)
DIAG_CONTROL_EVERY_N = max(1, int(os.environ.get("UDP_BRIDGE_DIAG_CONTROL_EVERY_N", "1")))


def format_xy_points(x, y, prec=2):
    """numpy array (x, y) → '[(x0,y0), (x1,y1), ...]' 사람이 읽는 문자열."""
    fmt = f"%.{prec}f"
    return "[" + ", ".join(f"({fmt % xi},{fmt % yi})" for xi, yi in zip(x, y)) + "]"


def _as_int_or_none(value):
    try:
        if value is None:
            return None
        return int(value)
    except (TypeError, ValueError):
        return None


def _as_float_or_none(value):
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _packet_meta(d):
    header = d.get("packet_header") if isinstance(d.get("packet_header"), dict) else {}
    direct_header = d.get("header") if isinstance(d.get("header"), dict) else {}
    if direct_header:
        header = {**direct_header, **header}

    t0_utc_ns = _as_int_or_none(d.get("t0_utc_ns"))
    source_t0_us = _as_int_or_none(d.get("t0_us"))
    if source_t0_us is None and t0_utc_ns is not None:
        source_t0_us = t0_utc_ns // 1000
    if source_t0_us is None:
        source_t0_us = _as_int_or_none(header.get("source_t0_us"))

    return {
        "udp_mode": d.get("udp_mode"),
        "label": d.get("label"),
        "clip_id": d.get("clip_id"),
        "sample_id": _as_int_or_none(d.get("sample_id", header.get("sample_id"))),
        "plan_seq": _as_int_or_none(d.get("front_frame_id", header.get("plan_seq"))),
        "tx_seq": _as_int_or_none(header.get("tx_seq")),
        "source_t0_us": source_t0_us,
        "tx_time_us": _as_int_or_none(header.get("tx_time_us")),
        "payload_actual_offset_s": _as_float_or_none(d.get("actual_offset_s", d.get("target_offset_s"))),
        "inference_time_s": _as_float_or_none(d.get("inference_time_s")),
        "coord_note": d.get("coordinate_note"),
        "packet_header": header,
    }


# ── JSON 패킷 파싱 ───────────────────────────────────
def parse_path_packet(data: bytes):
    """{"pred_xyz": [[x,y,z], ...]} 형태 ego-frame slice 수신.
    실패 시 None. 수신은 x=forward, y=right 규약이지만 내부적으로는
    openpilot body frame(y=left)로 통일해서 반환한다.
    """
    try:
        d = json.loads(data.decode())
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
        cloudlog.warning(f"udp_bridge: invalid JSON ({e})")
        return None
    if not isinstance(d, dict):
        cloudlog.warning(f"udp_bridge: invalid JSON root type {type(d).__name__}")
        return None

    try:
        pred_xyz = np.asarray(d['pred_xyz'], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as e:
        cloudlog.warning(f"udp_bridge: malformed packet ({e})")
        return None

    if pred_xyz.ndim != 2 or pred_xyz.shape[1] < 2 or pred_xyz.shape[0] < 2:
        cloudlog.warning(f"udp_bridge: bad pred_xyz shape {pred_xyz.shape}")
        return None

    x = pred_xyz[:, 0]
    raw_y = pred_xyz[:, 1]
    y = -raw_y   # 수신 y=right(+) → 내부 y=left(+) (openpilot 규약)

    return {
        'x': x,
        'y': y,
        'raw_y': raw_y,
        'N': int(x.shape[0]),
        'meta': _packet_meta(d),
    }


# ── pure pursuit (lateral) ───────────────────────────
def pure_pursuit_curvature(path_ego, v_ego, lookahead_m=None, index_frac=None):
    """ego-frame path(x=fwd, y=left)에서 pure pursuit 으로 desired curvature 산출.

    goal 점 선택 방식 2가지:
      A) 인덱스 비율 기반 (index_frac 이 지정되면 이 방식 우선)
         - 들어온 path 인덱스의 index_frac(0~1) 지점을 goal 로.
         - 예) 0.5 → path 중간 인덱스, 1.0 → 끝점, 0.0 → 첫 점
      B) 거리 기반 (index_frac 이 None 일 때)
         - ego(0,0) 에서 L_d 만큼 떨어진 path 위 점을 goal 로.
         - L_d 까지 닿는 점이 없으면 path 끝점(전방 마지막 점)을 goal 로.

    산출: κ = 2·Δy / L_d_eff²  (y left(+) → κ CCW(+), openpilot desiredCurvature 규약)
          L_d_eff 는 실제 goal 점까지의 직선거리.
    """
    x = np.asarray(path_ego['x'], dtype=np.float64)
    y = np.asarray(path_ego['y'], dtype=np.float64)
    n = x.shape[0]

    d = np.hypot(x, y)

    if index_frac is not None and n > 0:
        # A) 인덱스 비율 기반
        f = float(np.clip(index_frac, 0.0, 1.0))
        goal_idx = int(round(f * (n - 1)))
    else:
        # B) 거리 기반 (lookahead_m 미지정 시 속도 비례 L_d)
        L_d = float(lookahead_for_speed(v_ego) if lookahead_m is None else lookahead_m)
        fwd_mask = x > 0.0
        candidates = np.where(fwd_mask & (d >= L_d))[0]
        if candidates.size > 0:
            goal_idx = int(candidates[0])
        elif fwd_mask.any():
            goal_idx = int(np.where(fwd_mask)[0][-1])
        else:
            goal_idx = int(np.argmin(d))

    L_d_eff = max(float(d[goal_idx]), 1e-3)
    y_goal = float(y[goal_idx])

    kappa = 2.0 * y_goal / (L_d_eff * L_d_eff)
    kappa = float(np.clip(kappa, -MAX_CURVATURE, MAX_CURVATURE))
    return kappa, goal_idx, L_d_eff


# ── alpasim MPC 셋업 ─────────────────────────────────
def load_car_params(timeout_s=CARPARAMS_WAIT_S):
    """CarParams 를 non-blocking 으로 기다린다. 못 받으면 None.

    controlsd 처럼 block=True 로 무한 대기하면 CarParams 가 없는 흐름
    (오프로드 replay 등)에서 udp_bridge 가 멈춘다. MPC 는 alpasim 기본
    차량 파라미터만으로도 돌아가므로 타임아웃 후 진행한다.
    """
    params = Params()
    deadline = time.monotonic() + timeout_s
    while True:
        raw = params.get("CarParams")
        if raw is not None:
            try:
                return messaging.log_from_bytes(raw, car.CarParams)
            except Exception as e:
                cloudlog.warning(f"udp_bridge: CarParams parse 실패 ({e})")
                return None
        if time.monotonic() >= deadline:
            return None
        time.sleep(0.1)


def build_lat_mpc(CP):
    """alpasim MPC path tracker 생성. CP 가 None 이면 alpasim 기본 파라미터.

    Returns:
        (tracker, VM) — VM 은 δ→κ 환산용 opendbc VehicleModel (없으면 None)
    """
    if CP is not None:
        veh = VehicleParameters.from_car_params(
            CP, steering_time_constant=MPC_STEERING_TIME_CONSTANT)
        try:
            vm = VehicleModel(CP)
        except Exception as e:
            cloudlog.warning(f"udp_bridge: VehicleModel 생성 실패 ({e}), kinematic 1/L 사용")
            vm = None
    else:
        veh = VehicleParameters(steering_time_constant=MPC_STEERING_TIME_CONSTANT)
        vm = None

    tracker = MPCPathTracker(
        vehicle_params=veh,
        gains=MPC_GAINS,
        n_horizon=MPC_N_HORIZON,
        dt_mpc=MPC_DT_MPC,
        target_speed=TARGET_SPEED_MPS,
        lon_kp=LON_KP,
        accel_min=ACCEL_MIN,
        accel_max=ACCEL_MAX,
        curv_limit=MAX_CURVATURE,
        vm=vm,
    )
    cloudlog.warning(
        f"udp_bridge MPC: horizon={MPC_N_HORIZON}×{MPC_DT_MPC}s={tracker.horizon_seconds:.1f}s "
        + f"(≈{tracker.reach_at_speed(TARGET_SPEED_MPS):.1f}m @ {TARGET_SPEED_MPS*3.6:.0f}km/h), "
        + f"mass={veh.mass:.0f}kg L={veh.wheelbase:.2f}m l_r={veh.l_rig_to_cg:.2f}m "
        + f"Caf={veh.front_cornering_stiffness:.0f} Car={veh.rear_cornering_stiffness:.0f} "
        + f"tau_s={veh.steering_time_constant:.2f}s VM={'opendbc' if vm else 'kinematic'}"
    )
    return tracker, vm


def ego_state_for_mpc(sm, steer_ratio, angle_offset_deg):
    """SubMaster → EgoState. livePose 가 죽어 있으면 횡속도/yaw rate 0."""
    live_pose = sm["livePose"] if sm.alive["livePose"] else None
    return EgoState.from_messages(
        sm["carState"],
        live_pose=live_pose,
        steer_ratio=steer_ratio,
        angle_offset_deg=angle_offset_deg,
    )


def _safe_float(value):
    if value is None:
        return ""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return ""
    if not math.isfinite(value):
        return ""
    return value


def _packet_age_s_from_us(now_wall_us, timestamp_us):
    timestamp_us = _as_int_or_none(timestamp_us)
    if timestamp_us is None:
        return None
    return (int(now_wall_us) - timestamp_us) / 1_000_000.0


def _path_shape_metrics(path_ego):
    y = np.asarray(path_ego["y"], dtype=np.float64)
    raw_y = np.asarray(path_ego.get("raw_y", y), dtype=np.float64)
    x = np.asarray(path_ego["x"], dtype=np.float64)
    dy = np.diff(y)
    d2y = np.diff(y, n=2)
    return {
        "first_x_m": float(x[0]) if x.size else None,
        "first_y_m": float(y[0]) if y.size else None,
        "first_raw_y_m": float(raw_y[0]) if raw_y.size else None,
        "last_x_m": float(x[-1]) if x.size else None,
        "last_y_m": float(y[-1]) if y.size else None,
        "mean_y_m": float(np.mean(y)) if y.size else None,
        "std_y_m": float(np.std(y)) if y.size else None,
        "max_abs_y_m": float(np.max(np.abs(y))) if y.size else None,
        "dy_step_rms_m": float(np.sqrt(np.mean(dy * dy))) if dy.size else None,
        "d2y_step_rms_m": float(np.sqrt(np.mean(d2y * d2y))) if d2y.size else None,
    }


MPC_DIAG_FIELDS = [
    "lat_controller",
    "mpc_status",
    "mpc_ok",
    "mpc_solve_ms",
    "mpc_iters",
    "mpc_delta_rad",
    "mpc_kappa",
    "mpc_curv_factor",
    "mpc_accel_cmd",
    "mpc_lat_err_m",
    "mpc_s_proj_m",
    "mpc_ref_reach_m",
    "mpc_ref_extrap_m",
    "mpc_fail_streak",
    "mpc_v_lat_mps",
    "mpc_yaw_rate_rps",
    "mpc_steer_state_rad",
]


def mpc_diag_columns(mpc_res, ego, fail_streak, controller_name):
    """diag CSV 의 MPC 컬럼. mpc_res 가 None 이면 빈 값."""
    cols = {"lat_controller": controller_name, "mpc_fail_streak": int(fail_streak)}
    if ego is not None:
        cols["mpc_v_lat_mps"] = _safe_float(ego.v_lat)
        cols["mpc_yaw_rate_rps"] = _safe_float(ego.yaw_rate)
        cols["mpc_steer_state_rad"] = _safe_float(ego.steering_angle)
    if mpc_res is not None:
        cols.update({
            "mpc_status": mpc_res.status,
            "mpc_ok": int(bool(mpc_res.ok)),
            "mpc_solve_ms": _safe_float(mpc_res.solve_time_ms),
            "mpc_iters": int(mpc_res.iters),
            "mpc_delta_rad": _safe_float(mpc_res.steering_cmd),
            "mpc_kappa": _safe_float(mpc_res.curvature),
            "mpc_curv_factor": _safe_float(mpc_res.curvature_factor),
            "mpc_accel_cmd": _safe_float(mpc_res.accel_cmd),
            "mpc_lat_err_m": _safe_float(mpc_res.lat_error_m),
            "mpc_s_proj_m": _safe_float(mpc_res.s_proj_m),
            "mpc_ref_reach_m": _safe_float(mpc_res.ref_reach_m),
            "mpc_ref_extrap_m": _safe_float(mpc_res.ref_extrapolated_m),
        })
    return cols


def build_path_diag_row(path_ego, *, event, recv_count, frame_id, v_ego, kappa_smoothed,
                        prev_goal_y_by_ld, now_wall_us, now_mono_s, slice_s=None,
                        mpc_res=None, ego=None, fail_streak=0,
                        controller_name=LAT_CONTROLLER):
    meta = dict(path_ego.get("meta") or {})
    row = {
        "event": event,
        "monotonic_s": now_mono_s,
        "wall_unix_s": now_wall_us / 1_000_000.0,
        "frame_id": int(frame_id),
        "recv_count": int(recv_count),
        "N": int(path_ego.get("N", len(path_ego.get("x", [])))),
        "v_ego_mps": _safe_float(v_ego),
        "kappa_smoothed": _safe_float(kappa_smoothed),
        "slice_s_m": _safe_float(slice_s),
        "udp_mode": meta.get("udp_mode") or "",
        "label": meta.get("label") or "",
        "clip_id": meta.get("clip_id") or "",
        "sample_id": "" if meta.get("sample_id") is None else int(meta.get("sample_id")),
        "plan_seq": "" if meta.get("plan_seq") is None else int(meta.get("plan_seq")),
        "tx_seq": "" if meta.get("tx_seq") is None else int(meta.get("tx_seq")),
        "source_t0_us": "" if meta.get("source_t0_us") is None else int(meta.get("source_t0_us")),
        "tx_time_us": "" if meta.get("tx_time_us") is None else int(meta.get("tx_time_us")),
        "source_age_s": _safe_float(_packet_age_s_from_us(now_wall_us, meta.get("source_t0_us"))),
        "tx_age_s": _safe_float(_packet_age_s_from_us(now_wall_us, meta.get("tx_time_us"))),
        "payload_actual_offset_s": _safe_float(meta.get("payload_actual_offset_s")),
        "inference_time_s": _safe_float(meta.get("inference_time_s")),
        "coord_note": meta.get("coord_note") or "",
    }
    row.update({key: _safe_float(value) for key, value in _path_shape_metrics(path_ego).items()})
    row.update(mpc_diag_columns(mpc_res, ego, fail_streak, controller_name))

    for ld_m in DIAG_LD_VALUES_M:
        kappa_pp, goal_idx, L_d_eff = pure_pursuit_curvature(path_ego, v_ego, lookahead_m=ld_m)
        goal_x = float(path_ego["x"][goal_idx])
        goal_y = float(path_ego["y"][goal_idx])
        prev_goal_y = prev_goal_y_by_ld.get(float(ld_m))
        tag = f"ld{ld_m:g}"
        row[f"{tag}_goal_idx"] = int(goal_idx)
        row[f"{tag}_goal_x_m"] = goal_x
        row[f"{tag}_goal_y_m"] = goal_y
        row[f"{tag}_raw_goal_y_m"] = float(path_ego.get("raw_y", path_ego["y"])[goal_idx])
        row[f"{tag}_L_eff_m"] = float(L_d_eff)
        row[f"{tag}_kappa_raw"] = float(kappa_pp)
        row[f"{tag}_goal_y_delta_m"] = "" if prev_goal_y is None else goal_y - float(prev_goal_y)
    return row


def diag_fieldnames():
    fields = [
        "event",
        "monotonic_s",
        "wall_unix_s",
        "frame_id",
        "recv_count",
        "N",
        "v_ego_mps",
        "kappa_smoothed",
        "slice_s_m",
        "udp_mode",
        "label",
        "clip_id",
        "sample_id",
        "plan_seq",
        "tx_seq",
        "source_t0_us",
        "tx_time_us",
        "source_age_s",
        "tx_age_s",
        "payload_actual_offset_s",
        "inference_time_s",
        "coord_note",
        "first_x_m",
        "first_y_m",
        "first_raw_y_m",
        "last_x_m",
        "last_y_m",
        "mean_y_m",
        "std_y_m",
        "max_abs_y_m",
        "dy_step_rms_m",
        "d2y_step_rms_m",
        *MPC_DIAG_FIELDS,
    ]
    for ld_m in DIAG_LD_VALUES_M:
        tag = f"ld{ld_m:g}"
        fields.extend(
            [
                f"{tag}_goal_idx",
                f"{tag}_goal_x_m",
                f"{tag}_goal_y_m",
                f"{tag}_raw_goal_y_m",
                f"{tag}_L_eff_m",
                f"{tag}_kappa_raw",
                f"{tag}_goal_y_delta_m",
            ]
        )
    return fields


def open_diag_csv():
    os.makedirs(DIAG_LOG_DIR, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(DIAG_LOG_DIR, f"udp_bridge_path_diag_{ts}.csv")
    fh = open(path, "w", newline="", buffering=1)
    writer = csv.DictWriter(fh, fieldnames=diag_fieldnames(), extrasaction="ignore")
    writer.writeheader()
    cloudlog.warning(f"udp_bridge path diagnostics CSV: {path}")
    return path, fh, writer


# ── 종방향 ───────────────────────────────────────────
def longitudinal_accel(v_ego):
    """TARGET_SPEED 유지 P 제어."""
    a_cmd = LON_KP * (TARGET_SPEED_MPS - max(v_ego, 0.0))
    return float(np.clip(a_cmd, ACCEL_MIN, ACCEL_MAX))


# ── viz용 리샘플 (path → T_IDXS 33점) ────────────────
def resample_for_viz(path_ego):
    """ego-frame path(x,y)를 T_IDXS 33점 시계열로 리샘플 (viz only).
    arc-length를 TARGET_SPEED 기준 시간으로 환산해 보간. path 너머는 마지막 값 hold.
    """
    x = path_ego['x']; y = path_ego['y']
    dx = np.diff(x); dy = np.diff(y)
    ds = np.hypot(dx, dy)
    s_path = np.concatenate([[0.0], np.cumsum(ds)])
    s_target = T_IDXS * TARGET_SPEED_MPS

    x_viz = np.interp(s_target, s_path, x).astype(np.float32)
    y_viz = np.interp(s_target, s_path, y).astype(np.float32)
    z_viz = np.zeros(IDX_N, dtype=np.float32)
    dx_viz = np.gradient(x_viz)
    dy_viz = np.gradient(y_viz)
    yaw = np.arctan2(dy_viz, np.maximum(dx_viz, 1e-3)).astype(np.float32)
    v = np.full(IDX_N, TARGET_SPEED_MPS, dtype=np.float32)
    vx = (v * np.cos(yaw)).astype(np.float32)
    vy = (v * np.sin(yaw)).astype(np.float32)
    return {
        'x': x_viz, 'y': y_viz, 'z': z_viz,
        'yaw': yaw, 'v': v, 'vx': vx, 'vy': vy,
    }


def default_resampled():
    zeros = np.zeros(IDX_N, dtype=np.float32)
    return {
        'x': zeros.copy(), 'y': zeros.copy(), 'z': zeros.copy(),
        'yaw': zeros.copy(), 'v': zeros.copy(),
        'vx': zeros.copy(), 'vy': zeros.copy(),
    }


def idle_action():
    return log.ModelDataV2.Action(
        desiredCurvature=0.0,
        desiredAcceleration=0.0,
        shouldStop=True,
    )


# ── 메시지 발행 ──────────────────────────────────────
def fill_xyzt(builder, t, x, y, z, x_std=None, y_std=None, z_std=None):
    builder.t = list(t) if not isinstance(t, list) else t
    builder.x = x.tolist() if hasattr(x, 'tolist') else list(x)
    builder.y = y.tolist() if hasattr(y, 'tolist') else list(y)
    builder.z = z.tolist() if hasattr(z, 'tolist') else list(z)
    if x_std is not None:
        builder.xStd = x_std.tolist()
    if y_std is not None:
        builder.yStd = y_std.tolist()
    if z_std is not None:
        builder.zStd = z_std.tolist()


def publish_messages(pm, rs, action, frame_id, v_ego):
    """modelV2 + drivingModelData + longitudinalPlan + driverAssistance 발행."""
    now_ns = int(time.monotonic() * 1e9)
    t_list = ModelConstants.T_IDXS

    zeros_33 = np.zeros(IDX_N, dtype=np.float32)
    low_std = np.full(IDX_N, 0.1, dtype=np.float32)

    # ── modelV2 ──
    modelv2_send = messaging.new_message('modelV2')
    modelv2_send.valid = True
    mv2 = modelv2_send.modelV2

    mv2.frameId = frame_id
    mv2.frameIdExtra = frame_id
    mv2.frameAge = 0
    mv2.frameDropPerc = 0.0
    mv2.timestampEof = now_ns
    mv2.modelExecutionTime = 0.0

    fill_xyzt(mv2.position, t_list, rs['x'], rs['y'], rs['z'],
              x_std=low_std, y_std=low_std, z_std=low_std)
    fill_xyzt(mv2.velocity, t_list, rs['vx'], rs['vy'], zeros_33)
    fill_xyzt(mv2.acceleration, t_list, zeros_33, zeros_33, zeros_33)
    fill_xyzt(mv2.orientation, t_list, zeros_33, zeros_33, rs['yaw'])
    fill_xyzt(mv2.orientationRate, t_list, zeros_33, zeros_33, zeros_33)

    mv2.action = action

    mv2.init('laneLines', 4)
    default_lane_y = [1.8, 1.8, -1.8, -1.8]
    for i in range(4):
        ll = mv2.laneLines[i]
        lane_y = np.full(IDX_N, default_lane_y[i], dtype=np.float32)
        fill_xyzt(ll, [], X_IDXS.astype(np.float32), lane_y, zeros_33)
    mv2.laneLineStds = [0.0, 0.0, 0.0, 0.0]
    mv2.laneLineProbs = [0.0, 0.0, 0.0, 0.0]

    mv2.init('roadEdges', 2)
    default_edge_y = [3.0, -3.0]
    for i in range(2):
        re = mv2.roadEdges[i]
        edge_y = np.full(IDX_N, default_edge_y[i], dtype=np.float32)
        fill_xyzt(re, [], X_IDXS.astype(np.float32), edge_y, zeros_33)
    mv2.roadEdgeStds = [0.0, 0.0]

    mv2.init('leadsV3', 3)
    for i in range(3):
        lead = mv2.leadsV3[i]
        lead_n = len(ModelConstants.LEAD_T_IDXS)
        lead.t = ModelConstants.LEAD_T_IDXS
        lead.x = [200.0] * lead_n
        lead.y = [0.0] * lead_n
        lead.v = [0.0] * lead_n
        lead.a = [0.0] * lead_n
        lead.xStd = [100.0] * lead_n
        lead.yStd = [100.0] * lead_n
        lead.vStd = [100.0] * lead_n
        lead.aStd = [100.0] * lead_n
        lead.prob = 0.0
        lead.probTime = ModelConstants.LEAD_T_OFFSETS[i]

    meta = mv2.meta
    meta.desireState = [0.0] * ModelConstants.DESIRE_LEN
    meta.desirePrediction = [0.0] * (ModelConstants.DESIRE_PRED_LEN * ModelConstants.DESIRE_PRED_WIDTH)
    meta.engagedProb = 1.0

    meta.init('disengagePredictions')
    dp = meta.disengagePredictions
    dp.t = ModelConstants.META_T_IDXS
    n_meta = len(ModelConstants.META_T_IDXS)
    dp.brakeDisengageProbs = [0.0] * n_meta
    dp.gasDisengageProbs = [0.0] * n_meta
    dp.steerOverrideProbs = [0.0] * n_meta
    dp.brake3MetersPerSecondSquaredProbs = [0.0] * n_meta
    dp.brake4MetersPerSecondSquaredProbs = [0.0] * n_meta
    dp.brake5MetersPerSecondSquaredProbs = [0.0] * n_meta
    dp.gasPressProbs = [1.0] * n_meta
    dp.brakePressProbs = [0.0] * n_meta

    meta.laneChangeState = log.LaneChangeState.off
    meta.laneChangeDirection = log.LaneChangeDirection.none
    meta.hardBrakePredicted = False

    mv2.confidence = log.ModelDataV2.ConfidenceClass.green

    # ── drivingModelData ──
    dmd_send = messaging.new_message('drivingModelData')
    dmd_send.valid = True
    dmd = dmd_send.drivingModelData

    dmd.frameId = frame_id
    dmd.frameIdExtra = frame_id
    dmd.frameDropPerc = 0.0
    dmd.modelExecutionTime = 0.0
    dmd.action = action

    xyz = np.stack([rs['x'], rs['y'], rs['z']], axis=1)
    coeffs = np.polynomial.polynomial.polyfit(T_IDXS, xyz, deg=ModelConstants.POLY_PATH_DEGREE)
    dmd.path.xCoefficients = coeffs[:, 0].tolist()
    dmd.path.yCoefficients = coeffs[:, 1].tolist()
    dmd.path.zCoefficients = coeffs[:, 2].tolist()

    dmd.laneLineMeta.leftY = -1.8
    dmd.laneLineMeta.leftProb = 0.0
    dmd.laneLineMeta.rightY = 1.8
    dmd.laneLineMeta.rightProb = 0.0

    dmd.meta.laneChangeState = log.LaneChangeState.off
    dmd.meta.laneChangeDirection = log.LaneChangeDirection.none

    # ── longitudinalPlan ──
    plan_send = messaging.new_message('longitudinalPlan')
    plan_send.valid = True
    lp = plan_send.longitudinalPlan
    lp.aTarget = float(action.desiredAcceleration)
    lp.shouldStop = bool(action.shouldStop)
    lp.allowBrake = True
    lp.allowThrottle = True
    lp.hasLead = False
    lp.speeds = [float(v_ego)]

    # ── driverAssistance ──
    assist_send = messaging.new_message('driverAssistance')
    assist_send.valid = True

    pm.send('modelV2', modelv2_send)
    pm.send('drivingModelData', dmd_send)
    pm.send('longitudinalPlan', plan_send)
    pm.send('driverAssistance', assist_send)


# ── viz 송신 (수신 path 를 world frame 으로 변환) ─────
def ego_to_world(x_ego, y_ego, viz_anchor):
    """ego-frame (x=fwd, y=LEFT) → world frame (NED). viz_anchor=(x0,y0,yaw0)."""
    x0, y0, yaw0 = viz_anchor
    c0, s0 = math.cos(yaw0), math.sin(yaw0)
    wx = x0 + c0 * x_ego + s0 * y_ego
    wy = y0 + s0 * x_ego - c0 * y_ego
    return wx, wy


def build_world_path(path_ego, viz_anchor):
    """수신 ego-frame path → world frame 점 list."""
    px = path_ego['x']
    py = path_ego['y']
    wx, wy = ego_to_world(px, py, viz_anchor)
    return [{"x": float(wx[i]), "y": float(wy[i])} for i in range(len(px))]


def send_world_path_viz(viz_sock, path_ego, viz_anchor, seq,
                         goal_ego_xy, i_goal, L_d_eff, kappa_raw):
    """확장된 trajectory_world 송신: world path + ego path + goal point + kappa.

    points       : world frame (server 가 display_anchor 빼서 전달)
    ego_points   : 원본 ego frame 좌표 (debug panel 용)
    goal_world   : world frame goal (server 가 anchor 빼서 전달)
    goal_ego     : ego frame goal + 부가 정보
    kappa_raw    : pure pursuit raw 결과 (smooth 전)
    """
    world_pts = build_world_path(path_ego, viz_anchor)
    gx, gy = ego_to_world(np.float64(goal_ego_xy[0]), np.float64(goal_ego_xy[1]), viz_anchor)
    ego_pts = [
        {"x": float(path_ego['x'][i]), "y": float(path_ego['y'][i])}
        for i in range(len(path_ego['x']))
    ]
    msg = {
        "type": "trajectory_world",
        "seq": int(seq),
        "num_points": len(world_pts),
        "dt_s": 0.0,
        "points": world_pts,
        "ego_points": ego_pts,
        "goal_world": {"x": float(gx), "y": float(gy)},
        "goal_ego": {
            "x": float(goal_ego_xy[0]),
            "y": float(goal_ego_xy[1]),
            "i": int(i_goal),
            "L_d_eff": float(L_d_eff),
        },
        "kappa_raw": float(kappa_raw),
    }
    try:
        viz_sock.sendto(json.dumps(msg).encode(), ("127.0.0.1", WORLD_PATH_VIZ_PORT))
    except OSError:
        pass


# ── viz 송신 (vehicle trail) ─────────────────────────
def send_vehicle_viz(viz_sock, world, lp, frame_id):
    """LocalWorld 현재 pose + 6초 history를 viz(5006) 로 송신 (viz only)."""
    cur = world.current()
    if cur is None:
        return
    _, x, y, yaw = cur
    speed = float(np.hypot(lp.velocityDevice.x, lp.velocityDevice.y))
    accel = float(lp.accelerationDevice.x)

    vehicle_msg = {
        "type": "vehicle",
        "x": float(x), "y": float(y),
        "heading": float(yaw),
        "speed": speed,
        "accel": accel,
        "curvature": 0.0,
        "should_stop": False,
        "frame": int(frame_id),
    }
    try:
        viz_sock.sendto(json.dumps(vehicle_msg).encode(), ("127.0.0.1", VEHICLE_VIZ_PORT))
    except OSError:
        pass

    hist = world.history()
    if len(hist) >= 2:
        pts = [
            {"x": float(hx), "y": float(hy), "yaw": float(hyaw), "vel": speed, "curvature": 0.0}
            for (_, hx, hy, hyaw) in hist
        ]
        traj_msg = {
            "type": "trajectory",
            "seq": int(frame_id),
            "plan_seq": int(frame_id),
            "coord_mode": 1,
            "num_points": len(pts),
            "dt_s": 0.05,
            "points": pts,
            "packet_count": int(frame_id),
        }
        try:
            viz_sock.sendto(json.dumps(traj_msg).encode(), ("127.0.0.1", VEHICLE_VIZ_PORT))
        except OSError:
            pass


# ── main ──────────────────────────────────────────────
def main():
    cloudlog.warning("udp_bridge init (ref-path slice mode, target=15km/h, "
                     + f"lat={LAT_CONTROLLER})")

    pm = PubMaster(["modelV2", "drivingModelData", "longitudinalPlan", "driverAssistance"])
    sm = SubMaster(["carState", "livePose", "selfdriveState", "liveParameters"])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.setblocking(False)

    viz_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    viz_sock.setblocking(False)

    cloudlog.warning(f"udp_bridge listening on port {UDP_PORT} (ref path slice JSON @ ~10Hz)")
    cloudlog.warning(f"udp_bridge viz: vehicle/trail→{VEHICLE_VIZ_PORT}, raw mirror→{LOCAL_PATH_VIZ_PORT}, "
                     f"world path→{WORLD_PATH_VIZ_PORT}")
    cloudlog.warning(f"udp_bridge debug log: engage 시 {DEBUG_LOG_DIR}/udp_bridge_debug_*.log 생성")

    # alpasim MPC 셋업. CarParams 를 못 받아도 기본 파라미터로 진행한다.
    use_mpc = LAT_CONTROLLER == "mpc"
    tracker = None
    CP = None
    if use_mpc:
        CP = load_car_params()
        if CP is None:
            cloudlog.warning(
                f"udp_bridge: CarParams 를 {CARPARAMS_WAIT_S:.0f}s 내에 못 받음 → "
                + "alpasim 기본 차량 파라미터(Ford Fusion)로 MPC 진행")
        tracker, VM = build_lat_mpc(CP)
    elif LAT_CONTROLLER != "pure_pursuit":
        raise ValueError(f"unknown LAT_CONTROLLER {LAT_CONTROLLER!r} "
                         + "(expected 'mpc' or 'pure_pursuit')")

    default_steer_ratio = float(CP.steerRatio) if CP is not None and CP.steerRatio > 0.1 else None
    if use_mpc and default_steer_ratio is None:
        # steerRatio 가 없으면 steeringAngleDeg → 전륜각 환산을 못 해서 MPC 의
        # 조향 상태가 항상 0 이 된다. 액추에이터 지연 모델이 무력화되므로
        # 과도 응답이 나빠진다(발산은 안 함 — 폐루프가 매 50 ms 다시 푼다).
        cloudlog.error("udp_bridge: steerRatio 없음 → MPC 조향 상태를 0 으로 둔다 (과도 응답 저하)")

    world = LocalWorld()        # viz only (vehicle trail)
    frame_id = 0
    path = None
    recv_count = 0
    log_counter = 0
    prev_curvature = 0.0

    # MPC 상태
    mpc_res = None
    fail_policy = SolverFailurePolicy(
        max_consecutive_fails=MPC_MAX_CONSECUTIVE_FAILS,
        use_fallback=MPC_FALLBACK_TO_PURE_PURSUIT,
    )

    # diag CSV 도 debug log 와 동일하게 engage rising/falling edge 마다 열고 닫음
    diag_fh = None
    diag_writer = None
    prev_goal_y_by_ld = {float(ld_m): None for ld_m in DIAG_LD_VALUES_M}

    # engage rising/falling edge 마다 debug log 파일을 새로 열고 닫음
    debug_log = None
    prev_engaged = False

    loop_period = 1.0 / ModelConstants.MODEL_RUN_FREQ  # 50ms = 20Hz

    while True:
        loop_start = time.monotonic()

        # 1. UDP 패킷 수신 — 큐에 쌓인 것 중 최신만 사용
        try:
            while True:
                data, _ = sock.recvfrom(RECV_BUF_SIZE)
                pkt = parse_path_packet(data)
                if pkt is not None:
                    recv_count += 1
                    path = pkt
                    # 디버그 로그: 수신 path (내부 좌표계, y=left). 한 줄=한 path. engage 중에만.
                    if debug_log is not None:
                        debug_log.write(
                            f"[t={time.monotonic():.3f}] PATH recv #{recv_count} N={pkt['N']} "
                            f"pts={format_xy_points(pkt['x'], pkt['y'])}\n"
                        )
                    # raw mirror → viz (ego-frame 원본)
                    try:
                        viz_sock.sendto(data, ('127.0.0.1', LOCAL_PATH_VIZ_PORT))
                    except OSError:
                        pass
                    # 패킷 도착 시점 pure pursuit 1회 → goal + raw kappa snapshot
                    v_ego_now = max(sm["carState"].vEgo, 0.0) if sm.alive["carState"] else 0.0
                    kappa_pp_pkt, i_goal_pkt, L_d_eff_pkt = pure_pursuit_curvature(pkt, v_ego_now, index_frac=PP_INDEX_FRAC)
                    if diag_writer is not None:   # engage 중에만 기록
                        now_wall_us = time.time_ns() // 1000
                        now_mono_s = time.monotonic()
                        diag_row = build_path_diag_row(
                            pkt,
                            event="recv",
                            recv_count=recv_count,
                            frame_id=frame_id,
                            v_ego=v_ego_now,
                            kappa_smoothed=None,
                            prev_goal_y_by_ld=prev_goal_y_by_ld,
                            now_wall_us=now_wall_us,
                            now_mono_s=now_mono_s,
                            slice_s=0.0,
                            mpc_res=mpc_res,
                            fail_streak=fail_policy.fail_streak,
                        )
                        diag_writer.writerow(diag_row)
                        for ld_m in DIAG_LD_VALUES_M:
                            tag = f"ld{ld_m:g}"
                            prev_goal_y_by_ld[float(ld_m)] = diag_row.get(f"{tag}_goal_y_m")
                    goal_xy = (float(pkt['x'][i_goal_pkt]), float(pkt['y'][i_goal_pkt]))
                    # world frame 변환 → viz(5008) (LocalWorld init 되어 있을 때만)
                    # MPC 모드에서는 goal/κ 를 직전 제어루프 MPC 결과로 대체한다
                    # (여기서 MPC 를 또 돌리지 않기 위해. 최대 50 ms 지연).
                    viz_goal_xy, viz_i_goal = goal_xy, i_goal_pkt
                    viz_L_d, viz_kappa = L_d_eff_pkt, kappa_pp_pkt
                    if use_mpc and mpc_res is not None and mpc_res.x_ref is not None:
                        ref_end = mpc_res.x_ref[-1]
                        viz_goal_xy = (float(ref_end[0]), float(ref_end[1]))
                        viz_i_goal = -1                     # MPC 는 path 인덱스 개념이 없음
                        viz_L_d = float(mpc_res.ref_reach_m)
                        viz_kappa = float(mpc_res.curvature)
                    if world.is_initialized():
                        _, x0, y0, yaw0 = world.current()
                        send_world_path_viz(viz_sock, pkt, (x0, y0, yaw0), recv_count,
                                            viz_goal_xy, viz_i_goal, viz_L_d, viz_kappa)
        except BlockingIOError:
            pass

        # 2. SubMaster 업데이트 (LocalWorld 적분은 viz용 trail 에만 사용)
        sm.update(0)
        if sm.updated["livePose"]:
            world.update(sm["livePose"], sm.logMonoTime["livePose"])

        v_ego = max(sm["carState"].vEgo, 0.0)

        # 2.5. engage edge 감지 → debug log + diag CSV 파일 open/close
        engaged = bool(sm["selfdriveState"].enabled) if sm.alive["selfdriveState"] else False
        if engaged and not prev_engaged:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            os.makedirs(DEBUG_LOG_DIR, exist_ok=True)
            debug_log_path = f"{DEBUG_LOG_DIR}/udp_bridge_debug_{ts}.log"
            debug_log = open(debug_log_path, "w", buffering=1)   # line-buffered
            cloudlog.warning(f"udp_bridge debug log OPEN (engage): {debug_log_path}")
            # diag CSV 도 새로 열고, goal_y delta 누적값 리셋
            diag_path, diag_fh, diag_writer = open_diag_csv()
            prev_goal_y_by_ld = {float(ld_m): None for ld_m in DIAG_LD_VALUES_M}
            # MPC warm start / 폴백 상태 초기화 (engage 시점은 불연속)
            if tracker is not None:
                tracker.reset()
                fail_policy.reset()
        elif (not engaged) and prev_engaged:
            if debug_log is not None:
                cloudlog.warning(f"udp_bridge debug log CLOSE (disengage): {debug_log.name}")
                debug_log.close()
                debug_log = None
            if diag_fh is not None:
                cloudlog.warning(f"udp_bridge diag CSV CLOSE (disengage): {diag_fh.name}")
                diag_fh.close()
                diag_fh = None
                diag_writer = None
        prev_engaged = engaged

        # 3. tracker — 받은 path 를 그대로(원점 재정렬 없이) 추종
        if path is not None:
            # pure pursuit 는 항상 계산한다: MPC 모드에서도 진단 비교용 + 폴백용.
            kappa_pp, i_goal, L_d_eff = pure_pursuit_curvature(path, v_ego, index_frac=PP_INDEX_FRAC)

            ego = None
            mpc_res = None
            if use_mpc:
                lp = sm["liveParameters"]
                if sm.alive["liveParameters"]:
                    steer_ratio = float(lp.steerRatio) if lp.steerRatio > 0.1 else default_steer_ratio
                    angle_offset = float(lp.angleOffsetDeg)
                else:
                    steer_ratio = default_steer_ratio
                    angle_offset = 0.0

                ego = ego_state_for_mpc(sm, steer_ratio, angle_offset)
                mpc_res = tracker.update(path['x'], path['y'], ego)

                kappa_raw, fail_event = fail_policy.select(
                    mpc_res.ok, mpc_res.curvature,
                    prev_curvature=prev_curvature,
                    fallback_curvature=kappa_pp,
                )
                if fail_event == "recovered":
                    cloudlog.warning(f"udp_bridge MPC 복구 (연속 실패 후, status={mpc_res.status})")
                elif fail_event == "fallback_entered":
                    cloudlog.error(
                        f"udp_bridge MPC 연속 실패 {fail_policy.fail_streak} 회 "
                        + f"(status={mpc_res.status}) → pure pursuit 폴백")
            else:
                kappa_raw = kappa_pp

            if v_ego > MIN_LAT_CONTROL_SPEED:
                kappa = smooth_value(kappa_raw, prev_curvature, LAT_SMOOTH_SECONDS)
            else:
                kappa = prev_curvature
            prev_curvature = kappa

            # 직진 데드존: |curvature| 이 임계 이하면 제어 입력을 0 으로
            if abs(kappa) <= CURV_DEADZONE:
                kappa = 0.0

            if use_mpc and MPC_USE_ACCEL and mpc_res is not None and mpc_res.ok:
                a_cmd = float(np.clip(mpc_res.accel_cmd, ACCEL_MIN, ACCEL_MAX))
            else:
                a_cmd = longitudinal_accel(v_ego)
            action = log.ModelDataV2.Action(
                desiredCurvature=float(kappa),
                desiredAcceleration=float(a_cmd),
                shouldStop=False,
            )
            rs = resample_for_viz(path)

            # 디버그 로그: 매 20Hz loop curvature, engage 중에만
            if debug_log is not None:
                if mpc_res is not None:
                    mpc_txt = (f"mpc={mpc_res.status} δ={mpc_res.steering_cmd:+.4f} "
                               + f"κ_mpc={mpc_res.curvature:+.4f} cf={mpc_res.curvature_factor:.4f} "
                               + f"lat_err={mpc_res.lat_error_m:+.2f} reach={mpc_res.ref_reach_m:.1f} "
                               + f"t={mpc_res.solve_time_ms:.1f}ms it={mpc_res.iters} "
                               + f"fail={fail_policy.fail_streak} ")
                else:
                    mpc_txt = ""
                debug_log.write(
                    f"[t={time.monotonic():.3f}] CURV frame={frame_id} v_ego={v_ego:.2f} "
                    + f"raw={kappa_raw:+.4f} sm={kappa:+.4f} {mpc_txt}"
                    + f"κ_pp={kappa_pp:+.4f} L_d_eff={L_d_eff:.2f} i_goal={i_goal} "
                    + f"cte={float(path['y'][0]):+.2f} pkts={recv_count}\n"
                )

            if diag_writer is not None and frame_id % DIAG_CONTROL_EVERY_N == 0:
                diag_writer.writerow(
                    build_path_diag_row(
                        path,
                        event="control",
                        recv_count=recv_count,
                        frame_id=frame_id,
                        v_ego=v_ego,
                        kappa_smoothed=kappa,
                        prev_goal_y_by_ld={},
                        now_wall_us=time.time_ns() // 1000,
                        now_mono_s=time.monotonic(),
                        slice_s=0.0,
                        mpc_res=mpc_res,
                        ego=ego,
                        fail_streak=fail_policy.fail_streak,
                    )
                )

            log_counter += 1
            if log_counter % 20 == 1:   # 1Hz
                cte = float(path['y'][0])
                if mpc_res is not None:
                    src = (f"mpc[{mpc_res.status}] δ={mpc_res.steering_cmd:+.4f} "
                           + f"reach={mpc_res.ref_reach_m:.1f} {mpc_res.solve_time_ms:.1f}ms"
                           + (f" FALLBACK(fail={fail_policy.fail_streak})" if fail_policy.fallback_active else ""))
                else:
                    src = "pure_pursuit"
                cloudlog.warning(
                    f"track: v_ego={v_ego:.2f} target={TARGET_SPEED_MPS:.2f} "
                    + f"κ={kappa:+.4f}(raw {kappa_raw:+.4f}) a={a_cmd:+.2f} "
                    + f"{src} κ_pp={kappa_pp:+.4f} L_d={L_d_eff:.1f} N={path['N']} "
                    + f"cte={cte:+.2f} pkts={recv_count}"
                )
        else:
            action = idle_action()
            rs = default_resampled()
            prev_curvature = 0.0
            mpc_res = None
            if tracker is not None:
                # path 가 끊긴 동안의 warm start 는 무의미 → 재개 시 새로 시작
                tracker.reset()

        # 4. 메시지 발행
        publish_messages(pm, rs, action, frame_id, v_ego)

        # 5. vehicle trail viz
        if world.is_initialized():
            send_vehicle_viz(viz_sock, world, sm["livePose"], frame_id)

        frame_id += 1

        # 6. 20Hz 타이밍 유지
        elapsed = time.monotonic() - loop_start
        sleep_time = loop_period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        cloudlog.warning("udp_bridge got SIGINT")