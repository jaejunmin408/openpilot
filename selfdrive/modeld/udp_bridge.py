#!/usr/bin/env python3
"""
Alpamayo UDP Bridge — arclength-based path tracking (livePose 미사용)

외부에서 UDP 로 ac_decoded_path.json 을 받아 path 를 그 시점 ego frame 그대로 보관.
매 20Hz tick 마다 v_ego(휠속) 만 적분해 path 위 누적 진행거리 s_now 를 추적.
pure pursuit goal 은 path 위 arclength = s_now + L_d 지점.

좌표·노이즈:
  - locationd / livePose 의존 0 (yaw/translation drift 가 κ 로 침투할 통로 없음).
  - 노이즈 침투원은 v_ego 적분 하나 (수 초 적분해도 cm 단위).
  - 새 packet 도착 시 s_now = v_ego * inference_time_s 로 회고 보상 (이미 지나간 추론 지연).
  - 액추에이터 지연(lateralDelay) 은 controlsd 의 latcontrol_torque buffer 가 처리하므로
    udp_bridge 에서는 손대지 않음 (이중 보상 방지).

부호 컨벤션:
  - Alpamayo(y=LEFT) → 내부 ego(y=RIGHT). parse 시 y, yaw 부호 반전.
  - openpilot desiredCurvature 는 LEFT 양수 → 내부 y=RIGHT 기반 κ 에 부호 반전.
"""
import json
import math
import socket
import time
import numpy as np

import cereal.messaging as messaging
from cereal import log
from cereal.messaging import PubMaster, SubMaster
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value
from openpilot.selfdrive.controls.lib.local_world import LocalWorld
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS

# ── 설정 ──────────────────────────────────────────────
UDP_PORT = 5005
LOCAL_PATH_VIZ_PORT = 5007   # 수신한 Alpamayo JSON 원본 mirror (ego-frame)
VEHICLE_VIZ_PORT = 5006      # 차량 pose/trail + pp_goal viz
WORLD_PATH_VIZ_PORT = 5008   # packet 받은 시점 LocalWorld anchor 로 박은 path (viz only)
RECV_BUF_SIZE = 65535

X_IDXS = np.array(ModelConstants.X_IDXS, dtype=np.float64)
IDX_N = ModelConstants.IDX_N

# ── 제어 파라미터 ────────────────────────────────────
MIN_LAT_CONTROL_SPEED = 0.3

ACCEL_MIN = -3.5
ACCEL_MAX = 2.0

LON_KP = 0.3
TARGET_SPEED_KPH = 10.0
TARGET_SPEED_MPS = TARGET_SPEED_KPH / 3.6
STOP_DIST_M = 1.0

# ── pure pursuit 파라미터 ────────────────────────────
PP_LOOKAHEAD_M = 5.0          # path 위 s_now 에서 이만큼 떨어진 arclength 점이 goal
PP_CURV_LIMIT = 0.2           # |κ| clip

MAX_TICK_DT_S = 0.5           # 이상치 dt 무시용 (s_now 폭주 방지)


# ── packet 파싱 + arclength 사전 계산 ────────────────
def parse_action_packet(data: bytes):
    """ac_decoded_path.json 바이트 → 내부 ego frame(y=RIGHT) path dict.
    누적 arclength 's' 도 함께 계산해서 반환.
    """
    try:
        d = json.loads(data.decode())
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
        cloudlog.warning(f"udp_bridge: invalid JSON ({e})")
        return None

    try:
        ra = d['raw_action']
        a = np.asarray(ra['accel_mps2'], dtype=np.float32)
        c = np.asarray(ra['curvature'], dtype=np.float32)
        dt_s = float(d.get('plan_dt_s', 0.1))
        inference_time_s = float(d.get('inference_time_s', 0.0))

        pred_xyz = np.asarray(d['pred_xyz'], dtype=np.float32)       # (N,3)
        pred_yaw = np.asarray(d['pred_yaw_rad'], dtype=np.float32)   # (N,)
        pred_v = np.asarray(d['pred_v_mps'], dtype=np.float32)       # (N,)
    except (KeyError, TypeError, ValueError) as e:
        cloudlog.warning(f"udp_bridge: malformed packet ({e})")
        return None

    N = len(a)
    if N < 2 or len(c) != N or pred_xyz.shape[0] != N or pred_yaw.shape[0] != N or pred_v.shape[0] != N:
        cloudlog.warning(f"udp_bridge: length mismatch (a={N}, c={len(c)}, "
                         f"xyz={pred_xyz.shape[0]}, yaw={pred_yaw.shape[0]}, v={pred_v.shape[0]})")
        return None

    # Alpamayo(y=LEFT, yaw=CCW) → 내부(y=RIGHT, yaw=CW)
    ego_x =  pred_xyz[:, 0].astype(np.float64)
    ego_y = -pred_xyz[:, 1].astype(np.float64)
    ego_z =  pred_xyz[:, 2].astype(np.float64)
    ego_yaw = -pred_yaw.astype(np.float64)
    path_v = np.full(N, TARGET_SPEED_MPS, dtype=np.float64)  # 고정 속도로 덮어씀
    a_ff = a.astype(np.float64)
    path_curv = c.astype(np.float64)

    # 누적 arclength (보간/추종거리 계산에 사용, 단조 증가)
    ds = np.hypot(np.diff(ego_x), np.diff(ego_y))
    s = np.concatenate([[0.0], np.cumsum(ds)])

    return {
        'ego_x': ego_x, 'ego_y': ego_y, 'ego_z': ego_z,
        'ego_yaw': ego_yaw,
        'path_v': path_v, 'path_curv': path_curv, 'a_ff': a_ff,
        's': s, 'dt_s': dt_s, 'N': N,
        'inference_time_s': inference_time_s,
    }


# ── 보간 helper ──────────────────────────────────────
def interp_path_at_s(pkt, s_query):
    """path 위 arclength s_query 지점의 (x, y, tx_hat, ty_hat) 반환.
    tx_hat, ty_hat: 단위 접선 벡터.
    s_query 가 [0, s_max] 범위 밖이면 가장 가까운 끝점/끝접선으로 clamp.
    """
    s_arr = pkt['s']
    x_arr = pkt['ego_x']
    y_arr = pkt['ego_y']
    s_c = float(np.clip(s_query, s_arr[0], s_arr[-1]))
    x = float(np.interp(s_c, s_arr, x_arr))
    y = float(np.interp(s_c, s_arr, y_arr))
    # 수치 미분 (작은 ds 양쪽)
    ds = 0.1
    s_lo = max(s_c - ds, s_arr[0])
    s_hi = min(s_c + ds, s_arr[-1])
    if s_hi - s_lo < 1e-6:
        return x, y, 1.0, 0.0
    dx = float(np.interp(s_hi, s_arr, x_arr) - np.interp(s_lo, s_arr, x_arr))
    dy = float(np.interp(s_hi, s_arr, y_arr) - np.interp(s_lo, s_arr, y_arr))
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return x, y, 1.0, 0.0
    return x, y, dx / norm, dy / norm


# ── pure pursuit (arclength-based, path-tangent frame at s_now) ──
def pure_pursuit_curvature(pkt, s_now):
    """path 위 s_now 지점을 차량 위치+heading 으로 가정 (CTE/heading 오차 무시).
    그 점의 path-tangent frame 에서 goal(= path 위 s_now+L_d) 의 lateral 로 κ 산출.

    부호: 내부 y=RIGHT. lateral_right>0 → 우회전 →
          openpilot LEFT-positive κ 컨벤션에서 κ<0 (부호 반전).
    """
    s_max = float(pkt['s'][-1])
    L_d = PP_LOOKAHEAD_M
    s_goal = min(s_now + L_d, s_max)

    x_h, y_h, tx, ty = interp_path_at_s(pkt, s_now)
    x_g, y_g, _, _   = interp_path_at_s(pkt, s_goal)

    Gx = x_g - x_h
    Gy = y_g - y_h

    # (x_fwd, y_right) frame 에서 perp_right = R(+90° z_down) · T = (-Ty, Tx)
    lateral_right = -ty * Gx + tx * Gy
    L_d_eff = max(math.hypot(Gx, Gy), 1e-3)

    kappa = -2.0 * lateral_right / (L_d_eff * L_d_eff)
    kappa = float(np.clip(kappa, -PP_CURV_LIMIT, PP_CURV_LIMIT))
    return kappa, s_goal, L_d_eff, lateral_right


# ── longitudinal ─────────────────────────────────────
def longitudinal_accel(pkt, s_now, v_ego):
    """고정 목표속도 추종 + path 끝 도달 시 정지."""
    s_max = float(pkt['s'][-1])
    remaining = max(s_max - s_now, 0.0)
    v_ref = TARGET_SPEED_MPS
    should_stop = False
    if remaining < STOP_DIST_M:
        v_ref = 0.0
        should_stop = True
    a_cmd = LON_KP * (v_ref - max(v_ego, 0.0))
    a_cmd = float(np.clip(a_cmd, ACCEL_MIN, ACCEL_MAX))
    return a_cmd, should_stop, v_ref, remaining


# ── 현재 ego-frame 근사 path (modelV2.position 용) ──
def slice_path_current_ego(pkt, s_now):
    """s_now 이후 path 를 s_now 지점 path-tangent frame 에 표현.
    원점=path[s_now], +x=접선, +y=접선 RIGHT.
    """
    s_arr = pkt['s']
    if s_now >= s_arr[-1]:
        return default_path_viz()

    i_start = int(np.searchsorted(s_arr, s_now, side='right'))
    if i_start >= len(s_arr):
        return default_path_viz()

    x_h, y_h, tx, ty = interp_path_at_s(pkt, s_now)
    tangent_angle = math.atan2(ty, tx)

    # head 점(=차량 위치 추정) + 이후 원본 path 점들
    xs_src = np.concatenate([[x_h], pkt['ego_x'][i_start:]]).astype(np.float64)
    ys_src = np.concatenate([[y_h], pkt['ego_y'][i_start:]]).astype(np.float64)
    z_at_h = float(np.interp(s_now, s_arr, pkt['ego_z']))
    zs_src = np.concatenate([[z_at_h], pkt['ego_z'][i_start:]]).astype(np.float64)
    yaws_src = np.concatenate([[tangent_angle], pkt['ego_yaw'][i_start:]]).astype(np.float64)
    vs_src = np.concatenate([[TARGET_SPEED_MPS], pkt['path_v'][i_start:]]).astype(np.float64)

    # path-tangent frame 으로 회전
    dx = xs_src - x_h
    dy = ys_src - y_h
    x_curr =  tx * dx + ty * dy
    y_curr = -ty * dx + tx * dy
    z_curr = zs_src - z_at_h
    yaw_curr = yaws_src - tangent_angle

    n = len(x_curr)
    t = np.arange(n, dtype=np.float64) * pkt['dt_s']
    vx = (vs_src * np.cos(yaw_curr)).astype(np.float32)
    vy = (vs_src * np.sin(yaw_curr)).astype(np.float32)
    return {
        't': t,
        'x': x_curr.astype(np.float32),
        'y': y_curr.astype(np.float32),
        'z': z_curr.astype(np.float32),
        'yaw': yaw_curr.astype(np.float32),
        'v': vs_src.astype(np.float32),
        'vx': vx, 'vy': vy,
    }


def default_path_viz():
    empty_f32 = np.zeros(0, dtype=np.float32)
    return {
        't': np.zeros(0, dtype=np.float64),
        'x': empty_f32.copy(), 'y': empty_f32.copy(), 'z': empty_f32.copy(),
        'yaw': empty_f32.copy(), 'v': empty_f32.copy(),
        'vx': empty_f32.copy(), 'vy': empty_f32.copy(),
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
    """modelV2 + drivingModelData + longitudinalPlan + driverAssistance 발행.
    rs: slice_path_current_ego 결과 (또는 default_path_viz).
    """
    now_ns = int(time.monotonic() * 1e9)

    n = len(rs['x'])
    t_path = rs['t'].tolist()
    zeros_n = np.zeros(n, dtype=np.float32)
    low_std_n = np.full(n, 0.1, dtype=np.float32)
    zeros_33 = np.zeros(IDX_N, dtype=np.float32)

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

    fill_xyzt(mv2.position, t_path, rs['x'], rs['y'], rs['z'],
              x_std=low_std_n, y_std=low_std_n, z_std=low_std_n)
    fill_xyzt(mv2.velocity, t_path, rs['vx'], rs['vy'], zeros_n)
    fill_xyzt(mv2.acceleration, t_path, zeros_n, zeros_n, zeros_n)
    fill_xyzt(mv2.orientation, t_path, zeros_n, zeros_n, rs['yaw'])
    fill_xyzt(mv2.orientationRate, t_path, zeros_n, zeros_n, zeros_n)

    mv2.action = action

    # lane lines (4, dummy)
    mv2.init('laneLines', 4)
    default_lane_y = [1.8, 1.8, -1.8, -1.8]
    for i in range(4):
        ll = mv2.laneLines[i]
        lane_y = np.full(IDX_N, default_lane_y[i], dtype=np.float32)
        fill_xyzt(ll, [], X_IDXS.astype(np.float32), lane_y, zeros_33)
    mv2.laneLineStds = [0.0, 0.0, 0.0, 0.0]
    mv2.laneLineProbs = [0.0, 0.0, 0.0, 0.0]

    # road edges (2, dummy)
    mv2.init('roadEdges', 2)
    default_edge_y = [3.0, -3.0]
    for i in range(2):
        re = mv2.roadEdges[i]
        edge_y = np.full(IDX_N, default_edge_y[i], dtype=np.float32)
        fill_xyzt(re, [], X_IDXS.astype(np.float32), edge_y, zeros_33)
    mv2.roadEdgeStds = [0.0, 0.0]

    # leads (3, dummy)
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

    # meta
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

    deg = ModelConstants.POLY_PATH_DEGREE
    if n >= deg + 1:
        xyz = np.stack([rs['x'], rs['y'], rs['z']], axis=1)
        coeffs = np.polynomial.polynomial.polyfit(rs['t'], xyz, deg=deg)
        dmd.path.xCoefficients = coeffs[:, 0].tolist()
        dmd.path.yCoefficients = coeffs[:, 1].tolist()
        dmd.path.zCoefficients = coeffs[:, 2].tolist()
    else:
        dmd.path.xCoefficients = [0.0] * (deg + 1)
        dmd.path.yCoefficients = [0.0] * (deg + 1)
        dmd.path.zCoefficients = [0.0] * (deg + 1)

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


# ── viz 송신 ─────────────────────────────────────────
def build_world_path(pkt, viz_anchor):
    """packet 수신 시점 LocalWorld pose(viz_anchor=x0,y0,yaw0)를 *한 번* 사용해
    pkt 의 ego frame path 를 world 좌표 list 로 변환 (viz only, drift 누적 없음).
    body(forward, right) → world(north, east) 표준 NED 회전.
    """
    x0, y0, yaw0 = viz_anchor
    c0, s0 = math.cos(yaw0), math.sin(yaw0)
    px = pkt['ego_x']
    py = pkt['ego_y']
    wx = x0 + c0 * px - s0 * py
    wy = y0 + s0 * px + c0 * py
    wyaw = yaw0 + (-pkt['ego_yaw'])
    wyaw_wrapped = np.arctan2(np.sin(wyaw), np.cos(wyaw))
    N = int(pkt['N'])
    out = []
    for i in range(N):
        out.append({
            "x": float(wx[i]),
            "y": float(wy[i]),
            "yaw": float(wyaw_wrapped[i]),
            "vel": float(pkt['path_v'][i]),
            "curvature": 0.0,
        })
    return out


def send_vehicle_viz(viz_sock, world, lp, frame_id):
    """LocalWorld 현재 pose + 6초 history 를 viz(5006) 로 송신 (viz only)."""
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


def send_pp_goal_viz(viz_sock, pkt, viz_anchor, s_goal, L_d_eff, frame_id):
    """pure pursuit goal 점(arclength s_goal)을 viz_anchor 기준 world 좌표로 viz(5006) 송신."""
    if viz_anchor is None:
        return
    x_eg, y_eg, _, _ = interp_path_at_s(pkt, s_goal)
    x0, y0, yaw0 = viz_anchor
    c0, s0 = math.cos(yaw0), math.sin(yaw0)
    wx = x0 + c0 * x_eg - s0 * y_eg
    wy = y0 + s0 * x_eg + c0 * y_eg
    msg = {
        "type": "pp_goal",
        "x": float(wx), "y": float(wy),
        "idx": 0,
        "L_d": float(L_d_eff),
        "frame": int(frame_id),
    }
    try:
        viz_sock.sendto(json.dumps(msg).encode(), ("127.0.0.1", VEHICLE_VIZ_PORT))
    except OSError:
        pass


def send_world_path_viz(viz_sock, points, dt_s, seq):
    """viz_anchor 로 world 변환된 path 를 viz(5008) 로 송신 (viz only)."""
    msg = {
        "type": "trajectory_world",
        "seq": int(seq),
        "num_points": len(points),
        "dt_s": float(dt_s),
        "points": points,
        "packet_count": int(seq),
    }
    try:
        viz_sock.sendto(json.dumps(msg).encode(), ("127.0.0.1", WORLD_PATH_VIZ_PORT))
    except OSError:
        pass


# ── main ──────────────────────────────────────────────
def main():
    cloudlog.warning("udp_bridge init (arclength-based tracking)")

    pm = PubMaster(["modelV2", "drivingModelData", "longitudinalPlan", "driverAssistance"])
    sm = SubMaster(["carState", "livePose"])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.setblocking(False)

    viz_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    viz_sock.setblocking(False)

    cloudlog.warning(f"udp_bridge listening on port {UDP_PORT} (JSON ac_decoded_path)")
    cloudlog.warning(f"udp_bridge viz publish: vehicle/trail/pp_goal→{VEHICLE_VIZ_PORT}, "
                     f"raw mirror→{LOCAL_PATH_VIZ_PORT}, worldFrame→{WORLD_PATH_VIZ_PORT}")

    world = LocalWorld()         # viz only (vehicle trail)
    frame_id = 0
    stored = None                # 받은 packet (그 시점 ego frame)
    s_now = 0.0                  # path 위 누적 진행거리 (m)
    t_prev_tick = None
    viz_anchor = None            # packet 받은 시점 LocalWorld pose snapshot (viz only)
    recv_count = 0
    log_counter = 0
    prev_curvature = 0.0

    loop_period = 1.0 / ModelConstants.MODEL_RUN_FREQ  # 50ms = 20Hz

    while True:
        loop_start = time.monotonic()

        # 1. UDP 수신 (non-blocking, 최신 packet 만 유효 처리)
        try:
            while True:
                data, _ = sock.recvfrom(RECV_BUF_SIZE)
                pkt = parse_action_packet(data)
                if pkt is None:
                    continue
                recv_count += 1
                v_ego_now = max(sm["carState"].vEgo, 0.0) if sm.alive["carState"] else 0.0
                stored = pkt
                # inference_time_s 동안 차가 path 따라 이미 진행한 거리만큼 s_now 초기화
                s_now = v_ego_now * pkt['inference_time_s']
                t_prev_tick = time.monotonic()
                # viz only: LocalWorld pose 스냅샷 (있으면). 이후 갱신 없음.
                viz_anchor = world.current()[1:] if world.is_initialized() else None
                if viz_anchor is not None:
                    world_points = build_world_path(pkt, viz_anchor)
                    send_world_path_viz(viz_sock, world_points, pkt['dt_s'], recv_count)
                cloudlog.warning(f"udp_bridge: pkt #{recv_count} "
                                 f"N={pkt['N']} dt={pkt['dt_s']:.3f}s "
                                 f"inference={pkt['inference_time_s']:.3f}s "
                                 f"s_now_init={s_now:.2f}m path_len={pkt['s'][-1]:.1f}m "
                                 f"({len(data)}B)")
                # raw mirror → viz
                try:
                    viz_sock.sendto(data, ('127.0.0.1', LOCAL_PATH_VIZ_PORT))
                except OSError:
                    pass
        except BlockingIOError:
            pass

        # 2. SubMaster 갱신 + LocalWorld 적분 (viz only)
        sm.update(0)
        if sm.updated["livePose"]:
            world.update(sm["livePose"], sm.logMonoTime["livePose"])

        v_ego = max(sm["carState"].vEgo, 0.0)

        # 3. tracker — arclength 기반
        if stored is not None:
            # v_ego 적분으로 s_now 전진
            now = time.monotonic()
            if t_prev_tick is not None:
                dt = now - t_prev_tick
                if 0.0 < dt < MAX_TICK_DT_S:
                    s_now += v_ego * dt
            t_prev_tick = now

            kappa_pp, s_goal, L_d_eff, lateral_right = pure_pursuit_curvature(stored, s_now)

            # smooth + 저속 hold
            if v_ego > MIN_LAT_CONTROL_SPEED:
                kappa = smooth_value(kappa_pp, prev_curvature, LAT_SMOOTH_SECONDS)
            else:
                kappa = prev_curvature
            prev_curvature = kappa

            a_cmd, should_stop, v_ref, remaining = longitudinal_accel(stored, s_now, v_ego)
            action = log.ModelDataV2.Action(
                desiredCurvature=float(kappa),
                desiredAcceleration=float(a_cmd),
                shouldStop=bool(should_stop),
            )
            rs = slice_path_current_ego(stored, s_now)

            send_pp_goal_viz(viz_sock, stored, viz_anchor, s_goal, L_d_eff, frame_id)

            log_counter += 1
            if log_counter % 20 == 1:   # 1Hz
                cloudlog.warning(
                    f"track: v_ego={v_ego:.2f} v_ref={v_ref:.2f} "
                    f"s_now={s_now:.2f} s_goal={s_goal:.2f} rem={remaining:.1f} "
                    f"lat_r={lateral_right:+.2f} "
                    f"κ={kappa:+.4f}(raw {kappa_pp:+.4f}) a={a_cmd:+.2f} "
                    f"L_d_eff={L_d_eff:.1f} stop={should_stop}"
                )
        else:
            action = idle_action()
            rs = default_path_viz()
            prev_curvature = 0.0
            t_prev_tick = time.monotonic()

        # 4. 메시지 발행
        publish_messages(pm, rs, action, frame_id, v_ego)

        # 5. vehicle viz (LocalWorld trail, viz only)
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
