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
받은 path를 그대로 ego-frame path로 사용한다. 원점(0,0) 재정렬(re-zero) 없이
받은 path 위에서 곧바로 desired curvature 를 산출한다.

횡방향 제어기 3종을 담고 있고 **주행 중 실시간으로 전환** 할 수 있다:

  "pure_pursuit" — 기하 추종. 속도 비례 look-ahead L_d = clip(k·v, MIN, MAX).
                   비용이 사실상 0 이라 항상 같이 계산해 두고 diag 기준선 +
                   MPC 연속 실패 시 폴백으로도 쓴다.
  "comma_mpc"    — comma openpilot 원본 lateral MPC(acados) 부활판. 삭제된
                   lateral_planner.py(66dbadb02^) 와 동일한 모델/게인/사용법으로
                   controls/lib/lateral_mpc_lib 를 그대로 쓰고, reference 만 모델
                   출력(modelV2) 대신 수신 path 에서 뽑아 넣는다.  (~5ms)
  "alpasim_mpc"  — alpasim LinearMPC 이식판. 8-state 동적 자전거 모델을 매 프레임
                   선형화해 ADMM QP 로 푼다. controls/lib/alpasim_mpc 참고.
                   전륜 조향각 δ 를 내고 κ = curvature_factor(v)·δ 로 환산.  (~13ms)

**reference path 소스도 주행 중 전환** 할 수 있다 (제어기와 직교):

  "alpamayo"    — 외부 publisher 가 UDP_PORT 로 보내주는 reference path slice (위 설명)
  "comma_model" — comma 비전모델이 스스로 뽑은 예측경로. modeld 가 매 프레임 계산하지만
                  버리던 plan/position 을 modelLanePath 로 발행하고, 여기서 그걸 받아
                  외부경로와 똑같은 형태의 path 로 만들어 같은 제어기에 물린다.
                  즉 "외부경로 추종" ↔ "comma 자체 주행" 을 같은 파이프라인에서 비교할 수 있다.

전환 방법 (프로세스 재시작 불필요):
    python selfdrive/modeld/lat_ctl.py          # 터미널에서 키 입력으로 전환
                                                #   1/2/3=제어기, p=경로 소스 토글
udp_bridge 는 manager 가 띄우는 프로세스라 stdin 이 터미널이 아니다. 그래서 키
입력은 lat_ctl.py 가 받아 UDP 제어 포트(LAT_CTL_PORT)로 명령을 보내는 구조다.
같은 LAN 의 노트북에서 `--host <device-ip>` 로 원격 조작도 된다.

MPC 는 20Hz 예산(50ms)을 생각해 활성 제어기만 매 loop 돌린다. lat_ctl 의 compare
를 켜면 셋 다 돌려 같은 경로에 대한 출력을 로그·diag 에 나란히 남긴다.

종방향은 TARGET_SPEED_MPS(=15 km/h) 유지 P 제어.
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
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.alpasim_mpc import (
    EgoState,
    MPCGains,
    MPCPathTracker,
    VehicleParameters,
)
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value, MIN_SPEED, CAR_ROTATION_RADIUS
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc, N as LAT_MPC_N
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
TARGET_SPEED_MPS = 8.0 / 3.6        # 15 km/h ≈ 4.17 m/s
LON_KP = 0.3
ACCEL_MIN = -3.5
ACCEL_MAX = 2.0

MIN_LAT_CONTROL_SPEED = 0.0001          # 이 속도 이하에서는 직전 curvature 유지

CURV_DEADZONE = 0.00                  # |curvature| 이 이하면 0 으로 (직진 데드존)

# ── pure pursuit 파라미터 ────────────────────────────
# 속도 비례 look-ahead: L_d = clip(PP_LOOKAHEAD_K_S * v_ego, MIN, MAX)
#   대중적 pure pursuit 방식(L_d ∝ v). 최소 10m, 최대 20m 로 clamp.
PP_LOOKAHEAD_MIN_M = 8.0
PP_LOOKAHEAD_MAX_M = 8.0
PP_LOOKAHEAD_K_S = 3.0               # look-ahead time gain (s) — L_d = k · v
PP_CURV_LIMIT = 0.2

# goal 점 선택 방식:
#   PP_INDEX_FRAC 이 None 이면 → 거리 기반(속도 비례 L_d 만큼 떨어진 점)
#   PP_INDEX_FRAC 이 [0.0, 1.0] 값이면 → 들어온 path 인덱스의 그 비율 지점을 goal 로
#   예) 0.5 → path 중간 인덱스, 1.0 → path 끝점, 0.0 → 첫 점
PP_INDEX_FRAC = None                 # None → 속도 비례 거리 기반 look-ahead 사용


def lookahead_for_speed(v_ego):
    """속도 비례 look-ahead 거리 L_d = clip(k · v, MIN, MAX)."""
    return float(np.clip(PP_LOOKAHEAD_K_S * max(v_ego, 0.0),
                         PP_LOOKAHEAD_MIN_M, PP_LOOKAHEAD_MAX_M))


# ── 횡방향 제어기 선택 ───────────────────────────────
LAT_MODES = ("pure_pursuit", "comma_mpc", "alpasim_mpc")
DEFAULT_LAT_MODE = "comma_mpc"
# 마지막으로 고른 제어기 (재시작해도 유지). openpilot Params 는 등록된 키만
# 받으므로(common/params.cc 화이트리스트) 재빌드가 필요 없는 평범한 파일을 쓴다.
LAT_MODE_FILE = "/data/udp_bridge_lat_mode"
LAT_CTL_PORT = 5009            # lat_ctl.py ↔ udp_bridge 제어/상태 포트
# 활성 MPC 가 이만큼 연속 실패하면 낡은 curvature 를 붙들지 않고 PP 로 폴백
LAT_FAIL_FALLBACK_N = 10

# ── reference path 소스 선택 ─────────────────────────
# 제어기(LAT_MODES)와 직교한다. 같은 제어기에 reference path 만 바꿔 끼우는 구조라
# "외부경로 추종" 과 "comma 자체 주행" 을 같은 조건에서 비교할 수 있다.
#   "alpamayo"    — 외부 publisher 의 UDP reference path slice
#   "comma_model" — modeld 가 발행하는 comma 비전모델 예측경로(modelLanePath)
PATH_SOURCES = ("alpamayo", "comma_model")
DEFAULT_PATH_SOURCE = "alpamayo"
PATH_SOURCE_FILE = "/data/udp_bridge_path_source"

# comma 모델경로 신뢰 조건. 하나라도 어긋나면 path=None → 제어 정지(안전측).
MODEL_PATH_MAX_AGE_S = 0.3     # modelLanePath 가 이보다 오래되면 버림
MODEL_PATH_MIN_RANGE_M = 2.0   # 전방 커버가 이보다 짧으면 버림 (모델경로는 시간축
                               # T_IDXS 33점이라 저속·정차에서 길이가 0 으로 수축한다)
# modelLanePath.positionY → 내부규약(y=LEFT+) 변환 부호. 1.0 = 그대로 사용.
# 모델 예측경로는 이미 내부규약과 같은 프레임이라 변환이 필요 없다. 실차 검증된 값이며
# -1.0 으로 두면 경로가 좌우로 뒤집혀 반대로 조향한다. 건드리지 말 것.
MODEL_Y_TO_INTERNAL_SIGN = 1.0

# ── comma MPC 파라미터 (삭제된 lateral_planner.py 원본 값) ──
COMMA_PATH_COST = 1.0
COMMA_LATERAL_MOTION_COST = 0.11
COMMA_LATERAL_ACCEL_COST = 0.0
COMMA_LATERAL_JERK_COST = 0.04
COMMA_STEERING_RATE_COST = 700.0
COMMA_CURV_LIMIT = 0.2

# ── comma MPC: 짧은 path tail 처리 (path 길이 < v_plan·10s) ──
COMMA_TAIL_HEADING_LEN = 3.0     # path 끝 접선 heading 추정용 lookback [m]
COMMA_TAIL_WEIGHT_TAU = 3.0      # 유효 범위 밖 노드 weight 지수감쇠 시상수 [node]
COMMA_TAIL_WEIGHT_FLOOR = 0.05   # tail reference weight 하한

# ── alpasim MPC 파라미터 (5090_controller 브랜치 실측 튜닝값) ──
# horizon 2.0s(=20×0.1s) 는 줄이면 원호 정상상태 횡오차가 크게 나빠지고, 늘려도
# 이득이 적다(A 행렬을 현재 yaw 로 고정 선형화하므로 예측이 길수록 모델이 틀림).
ALPASIM_N_HORIZON = 20
ALPASIM_DT_MPC = 0.1
ALPASIM_CURV_LIMIT = 0.2
ALPASIM_STEERING_TIME_CONSTANT = 0.1
# alpasim configs/controller/default.yaml 과 동일한 gain.
ALPASIM_GAINS = MPCGains(
    long_position_weight=2.0,
    lat_position_weight=1.0,
    heading_weight=1.0,
    acceleration_weight=0.1,
    rel_front_steering_angle_weight=5.0,
    rel_acceleration_weight=1.0,
    idx_start_penalty=10,
)
CARPARAMS_WAIT_S = 5.0         # CarParams 대기 한도. 넘으면 기본 차량 파라미터로 진행

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
    kappa = float(np.clip(kappa, -PP_CURV_LIMIT, PP_CURV_LIMIT))
    return kappa, goal_idx, L_d_eff


# ── comma MPC (원본 lateral_mpc_lib 부활) ───────────
def sample_path_for_comma_mpc(path_ego, v_plan):
    """ego-frame path(x=fwd, y=left)를 MPC reference 33점(N+1)으로 리샘플.

    MPC shooting node 는 시간축 T_IDXS(33점). node i 의 reference 는 차량이
    v_plan 으로 달릴 때 시각 T_IDXS[i] 에 도달하는 arc-length s_i=v_plan·t_i
    지점의 path 값이다. 거기서 lateral offset y, 접선 heading, yaw rate 를 뽑는다.

    수신 path 길이(~20m)가 가장 먼 노드의 목표 호길이(v_plan·10s)보다 짧으면
    path 끝 너머 노드가 생긴다. 이 구간은 끝값 hold(y 고정=직진 명령) 대신 끝점
    접선(heading) 방향 직선으로 외삽해 기하학적으로 일관되게 채운다. 외삽 구간은
    실제 reference 가 아니므로 호출부에서 weight 를 감쇠시키도록 n_valid 를 함께
    반환한다(s_target ≤ path 길이 인 노드 수).
    """
    x = np.asarray(path_ego['x'], dtype=np.float64)
    y = np.asarray(path_ego['y'], dtype=np.float64)
    ds = np.hypot(np.diff(x), np.diff(y))
    s_path = np.concatenate([[0.0], np.cumsum(ds)])
    s_end = float(s_path[-1])
    s_target = T_IDXS * max(float(v_plan), MIN_SPEED)

    # path 안에 들어오는 (= 실제 데이터로 채워지는) shooting node 수
    n_valid = int(np.count_nonzero(s_target <= s_end + 1e-6))
    n_valid = max(n_valid, 1)   # 최소 원점 노드는 항상 유효

    # 호길이 기준 리샘플 (밖은 일단 끝점 clamp)
    x_s = np.interp(s_target, s_path, x)
    y_s = np.interp(s_target, s_path, y)

    # ── path 끝 너머는 끝점 접선 방향 직선으로 외삽 ──
    if n_valid < IDX_N:
        # 끝점 접선 heading: resample 노이즈를 피해 끝에서 살짝 뒤 구간 방향 사용
        s_back = max(s_end - COMMA_TAIL_HEADING_LEN, 0.0)
        x_back = float(np.interp(s_back, s_path, x))
        y_back = float(np.interp(s_back, s_path, y))
        x_end, y_end = float(x[-1]), float(y[-1])
        psi_end = np.arctan2(y_end - y_back, max(x_end - x_back, 1e-3))
        ds_ext = s_target - s_end                      # 끝 너머 노드는 양수
        ext = s_target > s_end + 1e-6
        x_s[ext] = x_end + ds_ext[ext] * np.cos(psi_end)
        y_s[ext] = y_end + ds_ext[ext] * np.sin(psi_end)

    heading = np.arctan2(np.gradient(y_s), np.maximum(np.gradient(x_s), 1e-3))
    yaw_rate = np.gradient(heading, T_IDXS)
    return y_s, heading, yaw_rate, n_valid


class CommaMpcController:
    """받은 ego-frame path 한 개에 대해 lateral MPC 를 돌려 desired curvature 산출.

    삭제된 comma lateral_planner.py(66dbadb02^) 와 동일한 dynamics/weight/사용법.
    거기서는 reference 가 modelV2(position.y / orientation.z / orientationRate.z)
    였고, 여기서는 같은 자리에 수신 reference path 를 시간축 리샘플해 넣는다.
    x0=[x, y, psi, psi_rate] 중 x/y/psi 는 매 프레임 ego 원점이므로 0, psi_rate
    (=desired yaw rate) 만 다음 iteration 으로 carry-over. curvature=psi_rate/v.
    """

    def __init__(self):
        self.lat_mpc = LateralMpc()
        self.x0 = np.zeros(4)
        self.reset()

    def reset(self):
        self.x0 = np.zeros(4)
        self.lat_mpc.reset(x0=self.x0)

    def update(self, path_ego, v_ego):
        """return (curvature, valid, solve_time). valid=False 면 caller 가 직전 값 유지."""
        v_plan = max(float(v_ego), MIN_SPEED)
        y_pts, heading_pts, yaw_rate_pts, n_valid = sample_path_for_comma_mpc(path_ego, v_plan)

        # path 끝 너머(외삽) 노드는 reference-tracking weight 를 지수감쇠시켜
        # fabricate 한 tail 을 MPC 가 추종하지 않게 한다. node n_valid 부터 감쇠.
        if n_valid < LAT_MPC_N + 1:
            over = np.clip(np.arange(LAT_MPC_N + 1) - (n_valid - 1), 0, None)
            node_weights = np.maximum(COMMA_TAIL_WEIGHT_FLOOR,
                                      np.exp(-over / COMMA_TAIL_WEIGHT_TAU))
        else:
            node_weights = None

        self.lat_mpc.set_weights(COMMA_PATH_COST, COMMA_LATERAL_MOTION_COST,
                                 COMMA_LATERAL_ACCEL_COST, COMMA_LATERAL_JERK_COST,
                                 COMMA_STEERING_RATE_COST, node_weights=node_weights)
        v_arr = np.full(LAT_MPC_N + 1, v_plan)
        p = np.column_stack([v_arr, np.full(LAT_MPC_N + 1, CAR_ROTATION_RADIUS)])
        self.lat_mpc.run(self.x0, p, y_pts, heading_pts, yaw_rate_pts)

        mpc_nans = bool(np.isnan(self.lat_mpc.x_sol[:, 3]).any())
        if mpc_nans or self.lat_mpc.solution_status != 0:
            self.reset()
            return 0.0, False, self.lat_mpc.solve_time

        # 다음 iteration init 용 + 현재 command: DT_MDL 앞 desired yaw rate
        self.x0[3] = float(np.interp(DT_MDL, T_IDXS[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3]))
        kappa = float(np.clip(self.x0[3] / v_plan, -COMMA_CURV_LIMIT, COMMA_CURV_LIMIT))
        return kappa, True, self.lat_mpc.solve_time


# ── alpasim MPC (alpasim LinearMPC 이식판) ───────────
def load_car_params(timeout_s=CARPARAMS_WAIT_S):
    """CarParams 를 non-blocking 으로 기다린다. 못 받으면 None.

    controlsd 처럼 block=True 로 무한 대기하면 CarParams 가 없는 흐름
    (오프로드 replay 등)에서 udp_bridge 가 멈춘다. alpasim MPC 는 기본 차량
    파라미터만으로도 돌아가므로 타임아웃 후 진행한다.
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


class AlpasimMpcController:
    """alpasim LinearMPC(8-state 동적 자전거, ADMM QP)로 desired curvature 산출.

    MPC 원출력은 전륜 조향각 δ 이고 κ = curvature_factor(v)·δ 로 환산한다.
    curvature_factor 는 opendbc VehicleModel(언더스티어 반영)을 쓰고, CarParams
    를 못 받으면 kinematic 1/wheelbase 로 대체한다.

    comma MPC 와 달리 횡속도·yaw rate·조향각 실측(livePose/carState)을 상태로
    받으므로 SubMaster 를 통째로 받아 EgoState 를 만든다.
    """

    def __init__(self, CP):
        if CP is not None:
            veh = VehicleParameters.from_car_params(
                CP, steering_time_constant=ALPASIM_STEERING_TIME_CONSTANT)
            try:
                vm = VehicleModel(CP)
            except Exception as e:
                cloudlog.warning(f"udp_bridge: VehicleModel 생성 실패 ({e}), kinematic 1/L 사용")
                vm = None
            self.default_steer_ratio = float(CP.steerRatio) if CP.steerRatio > 0.1 else None
        else:
            veh = VehicleParameters(steering_time_constant=ALPASIM_STEERING_TIME_CONSTANT)
            vm = None
            self.default_steer_ratio = None

        self.tracker = MPCPathTracker(
            vehicle_params=veh,
            gains=ALPASIM_GAINS,
            n_horizon=ALPASIM_N_HORIZON,
            dt_mpc=ALPASIM_DT_MPC,
            target_speed=TARGET_SPEED_MPS,
            lon_kp=LON_KP,
            accel_min=ACCEL_MIN,
            accel_max=ACCEL_MAX,
            curv_limit=ALPASIM_CURV_LIMIT,
            vm=vm,
        )
        self.last_result = None
        cloudlog.warning(
            f"udp_bridge alpasim MPC: horizon={ALPASIM_N_HORIZON}×{ALPASIM_DT_MPC}s="
            f"{self.tracker.horizon_seconds:.1f}s "
            f"(≈{self.tracker.reach_at_speed(TARGET_SPEED_MPS):.1f}m @ {TARGET_SPEED_MPS * 3.6:.0f}km/h), "
            f"mass={veh.mass:.0f}kg L={veh.wheelbase:.2f}m "
            f"tau_s={veh.steering_time_constant:.2f}s VM={'opendbc' if vm else 'kinematic'}"
        )

    def reset(self):
        self.tracker.reset()
        self.last_result = None

    def _ego_state(self, sm):
        """SubMaster → EgoState. livePose 가 죽어 있으면 횡속도/yaw rate 0."""
        if sm.alive["liveParameters"]:
            lp = sm["liveParameters"]
            steer_ratio = float(lp.steerRatio) if lp.steerRatio > 0.1 else self.default_steer_ratio
            angle_offset = float(lp.angleOffsetDeg)
        else:
            steer_ratio = self.default_steer_ratio
            angle_offset = 0.0
        return EgoState.from_messages(
            sm["carState"],
            live_pose=sm["livePose"] if sm.alive["livePose"] else None,
            steer_ratio=steer_ratio,
            angle_offset_deg=angle_offset,
        )

    def update(self, path_ego, sm):
        """return (curvature, valid, solve_time_s)."""
        res = self.tracker.update(path_ego['x'], path_ego['y'], self._ego_state(sm))
        self.last_result = res
        if not res.ok or not math.isfinite(res.curvature):
            return 0.0, False, res.solve_time_ms / 1e3
        return float(res.curvature), True, res.solve_time_ms / 1e3


# ── 런타임 제어기 전환 (UDP 제어 포트) ───────────────
#
# udp_bridge 는 manager 가 띄우는 프로세스라 stdin 이 터미널이 아니다. 그래서
# 여기서 직접 키를 읽지 않고, 별도 CLI(selfdrive/modeld/lat_ctl.py)가 터미널
# 키 입력을 받아 이 UDP 포트로 명령을 보낸다. 프로세스 재시작 없이 주행 중에
# 제어기를 바꿀 수 있고, 같은 LAN 의 노트북에서 원격으로도 조작할 수 있다.
#
# 프로토콜 (JSON, 요청 1패킷 → 응답 1패킷):
#   {"cmd": "get"}                    → 현재 상태
#   {"cmd": "set", "mode": "<mode>"}  → 제어기 전환 후 상태
#   {"cmd": "cycle"}                  → 다음 제어기로 순환 후 상태
#   {"cmd": "compare", "on": bool}    → 비활성 제어기도 매 loop 계산(로그 비교용)
#   {"cmd": "reset"}                  → 활성 제어기 내부 상태 리셋
#   {"cmd": "source"}                 → reference path 소스 토글 후 상태
#   {"cmd": "source", "src": "<name>"}→ reference path 소스 지정 후 상태
# 응답은 아래 LatModeState.status() 의 dict 를 JSON 으로 직렬화한 것.
class LatModeState:
    """현재 선택된 횡제어기 + reference path 소스 + 제어 포트 서버 (non-blocking)."""

    def __init__(self, mode, sock, source=DEFAULT_PATH_SOURCE):
        self.mode = mode
        self.source = source        # reference path 소스 (제어기와 직교)
        self.compare = False        # True 면 비활성 제어기도 매 loop 계산(로그용)
        self.sock = sock
        self.reset_request = False  # main loop 가 소비하는 1회성 플래그
        self.switch_count = 0
        self.source_switch_count = 0
        self.last_cmd_from = None
        # main loop 가 매 frame 채워 넣는 텔레메트리 (status 응답용)
        self.telemetry = {}

    # ── 상태 조회/변경 ──
    def set_mode(self, mode):
        mode = str(mode).strip().lower()
        if mode not in LAT_MODES:
            return False, f"unknown mode {mode!r} (choose from {', '.join(LAT_MODES)})"
        if mode == self.mode:
            return True, f"already {mode}"
        prev, self.mode = self.mode, mode
        self.switch_count += 1
        self.reset_request = True     # 새로 붙는 제어기의 누적 상태를 비우고 시작
        save_lat_mode(mode)
        msg = f"lateral controller {prev} -> {mode}"
        cloudlog.warning(f"udp_bridge: {msg}")
        print(f"[LAT] {msg}", flush=True)
        return True, msg

    def cycle(self):
        return self.set_mode(LAT_MODES[(LAT_MODES.index(self.mode) + 1) % len(LAT_MODES)])

    def set_source(self, source):
        """reference path 소스 전환. 제어기 선택과는 독립이다."""
        source = str(source).strip().lower()
        if source not in PATH_SOURCES:
            return False, f"unknown source {source!r} (choose from {', '.join(PATH_SOURCES)})"
        if source == self.source:
            return True, f"already {source}"
        prev, self.source = self.source, source
        self.source_switch_count += 1
        # reference 가 불연속으로 바뀌므로 제어기 누적 상태(x0/warm start)를 비운다
        self.reset_request = True
        save_path_source(source)
        msg = f"path source {prev} -> {source}"
        cloudlog.warning(f"udp_bridge: {msg}")
        print(f"[PATH] {msg}", flush=True)
        return True, msg

    def toggle_source(self):
        nxt = PATH_SOURCES[(PATH_SOURCES.index(self.source) + 1) % len(PATH_SOURCES)]
        return self.set_source(nxt)

    def status(self):
        st = {
            "mode": self.mode,
            "modes": list(LAT_MODES),
            "source": self.source,
            "sources": list(PATH_SOURCES),
            "compare": self.compare,
            "switch_count": self.switch_count,
            "source_switch_count": self.source_switch_count,
        }
        st.update(self.telemetry)
        return st

    # ── 제어 포트 서비스 (매 loop 1회 호출) ──
    def poll(self):
        """대기 중인 명령을 전부 처리하고 각각에 상태를 응답한다."""
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                return
            except OSError:
                return
            ok, msg = True, ""
            try:
                req = json.loads(data.decode("utf-8"))
                cmd = str(req.get("cmd", "get")).lower()
                if cmd == "set":
                    ok, msg = self.set_mode(req.get("mode", ""))
                elif cmd == "cycle":
                    ok, msg = self.cycle()
                elif cmd == "compare":
                    self.compare = bool(req.get("on", not self.compare))
                    msg = f"compare={'on' if self.compare else 'off'}"
                elif cmd == "source":
                    src = req.get("src")
                    if src in (None, "", "toggle"):
                        ok, msg = self.toggle_source()
                    else:
                        ok, msg = self.set_source(src)
                elif cmd == "reset":
                    self.reset_request = True
                    msg = "controller state reset"
                elif cmd != "get":
                    ok, msg = False, f"unknown cmd {cmd!r}"
            except Exception as e:
                ok, msg = False, f"bad request ({e})"
            self.last_cmd_from = addr
            resp = self.status()
            resp["ok"] = ok
            resp["msg"] = msg
            try:
                self.sock.sendto(json.dumps(resp).encode("utf-8"), addr)
            except OSError:
                pass


def load_lat_mode():
    """시작 시 모드 결정: 환경변수 > 마지막 선택 저장값 > 기본값."""
    env = os.environ.get("UDP_BRIDGE_LAT_MODE", "").strip().lower()
    if env in LAT_MODES:
        return env
    if env:
        cloudlog.warning(f"udp_bridge: unknown UDP_BRIDGE_LAT_MODE {env!r}, ignoring")
    try:
        with open(LAT_MODE_FILE, encoding="utf-8") as f:
            saved = f.read().strip().lower()
        if saved in LAT_MODES:
            return saved
    except OSError:
        pass
    return DEFAULT_LAT_MODE


def save_lat_mode(mode):
    """다음 실행에서도 같은 제어기로 뜨도록 저장 (실패해도 무시)."""
    try:
        with open(LAT_MODE_FILE, "w", encoding="utf-8") as f:
            f.write(mode)
    except OSError as e:
        cloudlog.warning(f"udp_bridge: lat mode 저장 실패 ({e})")


def load_path_source():
    """시작 시 경로 소스 결정: 환경변수 > 마지막 선택 저장값 > 기본값."""
    env = os.environ.get("UDP_BRIDGE_PATH_SOURCE", "").strip().lower()
    if env in PATH_SOURCES:
        return env
    if env:
        cloudlog.warning(f"udp_bridge: unknown UDP_BRIDGE_PATH_SOURCE {env!r}, ignoring")
    try:
        with open(PATH_SOURCE_FILE, encoding="utf-8") as f:
            saved = f.read().strip().lower()
        if saved in PATH_SOURCES:
            return saved
    except OSError:
        pass
    return DEFAULT_PATH_SOURCE


def save_path_source(source):
    """다음 실행에서도 같은 소스로 뜨도록 저장 (실패해도 무시)."""
    try:
        with open(PATH_SOURCE_FILE, "w", encoding="utf-8") as f:
            f.write(source)
    except OSError as e:
        cloudlog.warning(f"udp_bridge: path source 저장 실패 ({e})")


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


def build_path_diag_row(path_ego, *, event, recv_count, frame_id, v_ego, kappa_smoothed,
                        prev_goal_y_by_ld, now_wall_us, now_mono_s, slice_s=None,
                        path_source=""):
    meta = dict(path_ego.get("meta") or {})
    row = {
        "event": event,
        "path_source": path_source,
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
        "path_source",
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


# ── comma 모델 예측경로 (modeld → modelLanePath) ─────
def build_model_path(sm, now_mono_s):
    """modelLanePath → 내부규약(y=LEFT+) 모델경로 dict. 못 쓰면 None.

    return (path|None, info). info 는 lat_ctl 표시·로그용 진단 상태로, path 가
    None 일 때 그 이유(reason)를 담는다. 모델경로는 시간축 T_IDXS 33점이라 전방
    커버가 속도에 비례한다 — 정차 중엔 길이가 거의 0 이므로 제어에 쓰면 안 된다.
    """
    info = {"alive": bool(sm.alive["modelLanePath"]), "age": None, "n": 0,
            "range_m": None, "curv": None, "frame_id": None, "reason": ""}
    if not info["alive"]:
        info["reason"] = "modeld 미수신"
        return None, info

    mlp = sm["modelLanePath"]
    info["age"] = now_mono_s - sm.logMonoTime["modelLanePath"] / 1e9
    info["n"] = len(mlp.positionX)
    info["curv"] = float(mlp.desiredCurvature)
    info["frame_id"] = int(mlp.frameId)

    if not mlp.valid:
        info["reason"] = "캘리브레이션 전"
        return None, info
    if info["n"] < 2:
        info["reason"] = f"점 부족 ({info['n']})"
        return None, info
    if info["age"] > MODEL_PATH_MAX_AGE_S:
        info["reason"] = f"stale {info['age']:.2f}s"
        return None, info

    x = np.asarray(mlp.positionX, dtype=np.float64)
    y = MODEL_Y_TO_INTERNAL_SIGN * np.asarray(mlp.positionY, dtype=np.float64)
    info["range_m"] = float(x[-1] - x[0])
    if info["range_m"] < MODEL_PATH_MIN_RANGE_M:
        info["reason"] = f"전방커버 {info['range_m']:.1f}m (저속/정차)"
        return None, info

    return {"x": x, "y": y, "age": info["age"], "frame_id": info["frame_id"],
            "model_curv": info["curv"], "v_ego": float(mlp.vEgo)}, info


def model_telemetry(model_info):
    """build_model_path 의 info → lat_ctl 표시용 텔레메트리 키."""
    return {
        "model_alive": bool(model_info["alive"]),
        "model_age": model_info["age"],
        "model_n": model_info["n"],
        "model_range_m": model_info["range_m"],
        "model_curv": model_info["curv"],
        "model_reason": model_info["reason"],
    }


def model_path_to_packet(model_path):
    """모델경로 → 외부 UDP 패킷과 동일한 path dict.

    이렇게 맞춰두면 pure pursuit / comma MPC / alpasim MPC 셋 다 소스를 모른 채
    지금까지와 똑같이 동작한다. raw_y 는 진단 표시용 우향(+) 복원.
    """
    x = np.asarray(model_path["x"], dtype=np.float64)
    y = np.asarray(model_path["y"], dtype=np.float64)
    return {
        "x": x, "y": y, "raw_y": -y, "N": int(x.shape[0]),
        "meta": {"udp_mode": "comma_model", "label": "modelLanePath",
                 "plan_seq": int(model_path["frame_id"])},
    }


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
    cloudlog.warning("udp_bridge init (ref-path slice mode, target=15km/h)")

    pm = PubMaster(["modelV2", "drivingModelData", "longitudinalPlan", "driverAssistance"])
    sm = SubMaster(["carState", "livePose", "selfdriveState", "liveParameters",
                    "modelLanePath"])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.setblocking(False)

    viz_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    viz_sock.setblocking(False)

    # 제어기 전환 명령 수신 포트 (lat_ctl.py)
    ctl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ctl_sock.bind(('0.0.0.0', LAT_CTL_PORT))
    ctl_sock.setblocking(False)

    cloudlog.warning(f"udp_bridge listening on port {UDP_PORT} (ref path slice JSON @ ~10Hz)")
    cloudlog.warning(f"udp_bridge viz: vehicle/trail→{VEHICLE_VIZ_PORT}, raw mirror→{LOCAL_PATH_VIZ_PORT}, "
                     f"world path→{WORLD_PATH_VIZ_PORT}")
    cloudlog.warning(f"udp_bridge debug log: engage 시 {DEBUG_LOG_DIR}/udp_bridge_debug_*.log 생성")

    world = LocalWorld()        # viz only (vehicle trail)

    # 제어기 3종을 전부 만들어 두고 lat_state.mode 로 고른다. 주행 중 전환은
    # lat_ctl.py 가 제어 포트로 보내는 명령으로 처리(프로세스 재시작 불필요).
    comma_ctl = CommaMpcController()
    alpasim_ctl = AlpasimMpcController(load_car_params())
    lat_state = LatModeState(load_lat_mode(), ctl_sock, load_path_source())
    lat_fail_streak = 0
    last_loop_ms = 0.0
    cloudlog.warning(f"udp_bridge lateral controller: {lat_state.mode} "
                     f"(전환: python selfdrive/modeld/lat_ctl.py, 포트 {LAT_CTL_PORT})")
    cloudlog.warning(f"udp_bridge reference path source: {lat_state.source} "
                     f"(전환: lat_ctl.py 의 [p])")
    print(f"[LAT] lateral controller = {lat_state.mode}  "
          f"(전환: python selfdrive/modeld/lat_ctl.py)", flush=True)
    print(f"[PATH] reference path source = {lat_state.source}  ([p] 로 토글)", flush=True)

    frame_id = 0
    ext_path = None            # 외부 UDP 로 받은 최신 reference path
    recv_count = 0             # 외부 UDP 패킷 수신 누계
    path_seq = 0               # 실제로 채택한 path 의 일련번호 (소스 무관, 로그 기준)
    last_model_frame_id = None # 같은 모델 프레임을 두 번 세지 않기 위한 마커
    log_counter = 0
    prev_curvature = 0.0

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
                    ext_path = pkt
                    # 소스가 외부경로일 때만 '채택한 path' 로 세고 기록한다.
                    # (comma_model 로 돌려놔도 수신·미러링은 계속해서 즉시 되돌릴 수 있게 둔다)
                    if lat_state.source == "alpamayo":
                        path_seq += 1
                    # 디버그 로그: 수신 path (내부 좌표계, y=left). 한 줄=한 path. engage 중에만.
                    if debug_log is not None and lat_state.source == "alpamayo":
                        debug_log.write(
                            f"[t={time.monotonic():.3f}] PATH recv #{path_seq} src=alpamayo "
                            f"N={pkt['N']} pts={format_xy_points(pkt['x'], pkt['y'])}\n"
                        )
                    # raw mirror → viz (ego-frame 원본)
                    try:
                        viz_sock.sendto(data, ('127.0.0.1', LOCAL_PATH_VIZ_PORT))
                    except OSError:
                        pass
                    # 패킷 도착 시점 pure pursuit 1회 → goal + raw kappa snapshot
                    v_ego_now = max(sm["carState"].vEgo, 0.0) if sm.alive["carState"] else 0.0
                    kappa_pp_pkt, i_goal_pkt, L_d_eff_pkt = pure_pursuit_curvature(pkt, v_ego_now, index_frac=PP_INDEX_FRAC)
                    if diag_writer is not None and lat_state.source == "alpamayo":
                        now_wall_us = time.time_ns() // 1000
                        now_mono_s = time.monotonic()
                        diag_row = build_path_diag_row(
                            pkt,
                            event="recv",
                            recv_count=path_seq,
                            frame_id=frame_id,
                            v_ego=v_ego_now,
                            kappa_smoothed=None,
                            prev_goal_y_by_ld=prev_goal_y_by_ld,
                            now_wall_us=now_wall_us,
                            now_mono_s=now_mono_s,
                            slice_s=0.0,
                            path_source="alpamayo",
                        )
                        diag_writer.writerow(diag_row)
                        for ld_m in DIAG_LD_VALUES_M:
                            tag = f"ld{ld_m:g}"
                            prev_goal_y_by_ld[float(ld_m)] = diag_row.get(f"{tag}_goal_y_m")
                    goal_xy = (float(pkt['x'][i_goal_pkt]), float(pkt['y'][i_goal_pkt]))
                    # world frame 변환 → viz(5008) (LocalWorld init 되어 있을 때만)
                    if world.is_initialized():
                        _, x0, y0, yaw0 = world.current()
                        send_world_path_viz(viz_sock, pkt, (x0, y0, yaw0), recv_count,
                                            goal_xy, i_goal_pkt, L_d_eff_pkt, kappa_pp_pkt)
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

        # 2.6. 제어 포트 서비스 — 주행 중 제어기 전환 (lat_ctl.py)
        lat_state.poll()
        if lat_state.reset_request:
            comma_ctl.reset()
            alpasim_ctl.reset()
            lat_fail_streak = 0
            lat_state.reset_request = False

        # 2.7. reference path 소스 선택 — 외부 UDP 경로 ↔ comma 모델 자체 경로
        # 두 소스 모두 항상 받아두고 여기서 고르기만 한다. 아래 제어기들은 소스를
        # 모른 채 동일한 path dict 를 받으므로 전환에 재시작·재초기화가 필요 없다.
        model_path, model_info = build_model_path(sm, loop_start)
        if lat_state.source == "comma_model":
            if model_path is None:
                path = None          # 모델경로를 못 믿으면 제어 정지 (안전측)
            else:
                path = model_path_to_packet(model_path)
                # 모델은 20Hz 라 loop 와 1:1 이지만, 프레임이 밀리면 같은 경로가
                # 두 번 온다. frameId 로 새 경로일 때만 새 번호를 준다.
                if model_path["frame_id"] != last_model_frame_id:
                    last_model_frame_id = model_path["frame_id"]
                    path_seq += 1
                    if debug_log is not None:
                        debug_log.write(
                            f"[t={time.monotonic():.3f}] PATH recv #{path_seq} src=comma_model "
                            f"N={path['N']} pts={format_xy_points(path['x'], path['y'])}\n"
                        )
                    if diag_writer is not None:
                        diag_writer.writerow(build_path_diag_row(
                            path,
                            event="recv",
                            recv_count=path_seq,
                            frame_id=frame_id,
                            v_ego=v_ego,
                            kappa_smoothed=None,
                            prev_goal_y_by_ld={},
                            now_wall_us=time.time_ns() // 1000,
                            now_mono_s=time.monotonic(),
                            slice_s=0.0,
                            path_source="comma_model",
                        ))
        else:
            path = ext_path

        # 3. tracker — 받은 path 를 그대로(원점 재정렬 없이) 추종
        if path is not None:
            mode = lat_state.mode
            # pure pursuit 는 비용이 없으니 항상 계산 (diag 기준선 + 폴백용)
            kappa_pp, i_goal, L_d_eff = pure_pursuit_curvature(path, v_ego, index_frac=PP_INDEX_FRAC)

            # MPC 는 20Hz 예산(50ms)을 생각해 활성 제어기만 돌린다.
            # compare 를 켜면 둘 다 돌려 같은 경로에 대한 출력을 로그에 남긴다.
            kappa_comma = kappa_alpasim = float('nan')
            comma_ok = alpasim_ok = False
            solve_comma = solve_alpasim = 0.0
            if mode == "comma_mpc" or lat_state.compare:
                kappa_comma, comma_ok, solve_comma = comma_ctl.update(path, v_ego)
            if mode == "alpasim_mpc" or lat_state.compare:
                kappa_alpasim, alpasim_ok, solve_alpasim = alpasim_ctl.update(path, sm)

            if mode == "comma_mpc":
                kappa_raw, ctl_valid, solve_s = kappa_comma, comma_ok, solve_comma
            elif mode == "alpasim_mpc":
                kappa_raw, ctl_valid, solve_s = kappa_alpasim, alpasim_ok, solve_alpasim
            else:
                kappa_raw, ctl_valid, solve_s = kappa_pp, True, 0.0

            # 솔버 실패: 짧으면 직전 curvature 유지(0 으로 튀지 않게), 연속으로
            # 이어지면 낡은 값을 계속 붙들지 않고 pure pursuit 으로 폴백한다.
            if ctl_valid:
                lat_fail_streak = 0
            else:
                lat_fail_streak += 1
                if lat_fail_streak == LAT_FAIL_FALLBACK_N:
                    cloudlog.warning(f"udp_bridge: {mode} 연속 실패 {lat_fail_streak}회 "
                                     f"→ pure pursuit 폴백")
                    print(f"[LAT] {mode} 연속 실패 → pure pursuit 폴백", flush=True)
                if lat_fail_streak >= LAT_FAIL_FALLBACK_N:
                    kappa_raw, ctl_valid = kappa_pp, True

            if ctl_valid and v_ego > MIN_LAT_CONTROL_SPEED:
                kappa = smooth_value(kappa_raw, prev_curvature, LAT_SMOOTH_SECONDS)
            else:
                kappa = prev_curvature
            prev_curvature = kappa

            lat_state.telemetry = {
                "kappa_cmd": kappa,
                "kappa_pp": kappa_pp,
                "kappa_comma": None if math.isnan(kappa_comma) else kappa_comma,
                "kappa_alpasim": None if math.isnan(kappa_alpasim) else kappa_alpasim,
                "solve_ms": solve_s * 1e3,
                "fail_streak": lat_fail_streak,
                "v_ego": v_ego,
                "cte_m": float(path['y'][0]),
                "path_points": int(path['N']),
                "pkts": recv_count,
                "path_seq": path_seq,
                "engaged": engaged,
                "frame": frame_id,
                "loop_ms": last_loop_ms,
                "has_path": True,
                **model_telemetry(model_info),
            }

            # 직진 데드존: |curvature| 이 임계 이하면 제어 입력을 0 으로
            if abs(kappa) <= CURV_DEADZONE:
                kappa = 0.0

            a_cmd = longitudinal_accel(v_ego)
            action = log.ModelDataV2.Action(
                desiredCurvature=float(kappa),
                desiredAcceleration=float(a_cmd),
                shouldStop=False,
            )
            rs = resample_for_viz(path)

            # 디버그 로그: 매 20Hz loop curvature, engage 중에만
            if debug_log is not None:
                mc = model_info["curv"]
                ma = model_info["age"]
                debug_log.write(
                    f"[t={time.monotonic():.3f}] CURV frame={frame_id} v_ego={v_ego:.2f} "
                    f"src={lat_state.source} mode={mode} "
                    f"comma={kappa_comma:+.4f}({'ok' if comma_ok else '-'}) "
                    f"alpasim={kappa_alpasim:+.4f}({'ok' if alpasim_ok else '-'}) "
                    f"pp={kappa_pp:+.4f} sm={kappa:+.4f} "
                    f"L_d_eff={L_d_eff:.2f} i_goal={i_goal} solve={solve_s * 1e3:.1f}ms "
                    f"cte={float(path['y'][0]):+.2f} path={path_seq} pkts={recv_count} "
                    f"model_curv={'nan' if mc is None else f'{mc:+.4f}'} "
                    f"model_age={'nan' if ma is None else f'{ma:.3f}'}\n"
                )

            if diag_writer is not None and frame_id % DIAG_CONTROL_EVERY_N == 0:
                diag_writer.writerow(
                    build_path_diag_row(
                        path,
                        event="control",
                        recv_count=path_seq,
                        frame_id=frame_id,
                        v_ego=v_ego,
                        kappa_smoothed=kappa,
                        prev_goal_y_by_ld={},
                        now_wall_us=time.time_ns() // 1000,
                        now_mono_s=time.monotonic(),
                        slice_s=0.0,
                        path_source=lat_state.source,
                    )
                )

            log_counter += 1
            if log_counter % 20 == 1:   # 1Hz
                cte = float(path['y'][0])
                cloudlog.warning(
                    f"track[{lat_state.source}]: v_ego={v_ego:.2f} target={TARGET_SPEED_MPS:.2f} "
                    f"κ={kappa:+.4f}[{mode}](comma {kappa_comma:+.4f} "
                    f"alpasim {kappa_alpasim:+.4f} pp {kappa_pp:+.4f}) a={a_cmd:+.2f} "
                    f"L_d={L_d_eff:.1f} i_goal={i_goal} N={path['N']} "
                    f"cte={cte:+.2f} path={path_seq} pkts={recv_count}"
                )
        else:
            action = idle_action()
            rs = default_resampled()
            prev_curvature = 0.0
            lat_fail_streak = 0
            comma_ctl.reset()
            alpasim_ctl.reset()
            lat_state.telemetry = {
                "v_ego": v_ego,
                "pkts": recv_count,
                "path_seq": path_seq,
                "engaged": engaged,
                "frame": frame_id,
                "loop_ms": last_loop_ms,
                "has_path": False,
                **model_telemetry(model_info),
            }

        # 4. 메시지 발행
        publish_messages(pm, rs, action, frame_id, v_ego)

        # 5. vehicle trail viz
        if world.is_initialized():
            send_vehicle_viz(viz_sock, world, sm["livePose"], frame_id)

        frame_id += 1

        # 6. 20Hz 타이밍 유지
        elapsed = time.monotonic() - loop_start
        last_loop_ms = elapsed * 1e3
        sleep_time = loop_period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        cloudlog.warning("udp_bridge got SIGINT")
