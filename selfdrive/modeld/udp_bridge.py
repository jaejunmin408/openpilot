#!/usr/bin/env python3
"""
Alpamayo UDP Bridge (ac_decoded_path.json 포맷) — 위치 기반 경로 추종 모드

외부에서 UDP로 1회 전송한 ac_decoded_path.json 전체(JSON bytes)를 수신하여:
  - pred_xyz / pred_yaw_rad / pred_v_mps 를 수신 시점 LocalWorld 앵커 기준 global 좌표로 변환해 저장
  - 20Hz 루프에서 livePose를 LocalWorld에 적분 → 현재 pose에서 Pure Pursuit/종방향 추종기 실행
  - 결과 desiredCurvature / desiredAcceleration 을 modelV2.action 으로 발행
  - pred_xyz 는 현재 ego frame으로 재표현해 modelV2.position 에 실어 UI 표시

좌표 변환: Alpamayo(y=LEFT, yaw=CCW, curv=CCW) → openpilot(y=RIGHT, yaw=CW, curv=CW)
           y, yaw, curvature 부호 반전.
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
from openpilot.selfdrive.controls.lib.local_world import LocalWorld
from openpilot.selfdrive.modeld.constants import ModelConstants

# ── 설정 ──────────────────────────────────────────────
UDP_PORT = 5005
LOCAL_PATH_VIZ_PORT = 5007   # 수신한 Alpamayo JSON 원본을 viz에 미러 (ego-frame, trajectory_local)
VEHICLE_VIZ_PORT = 5006      # LocalWorld 현재 pose + 6초 trail
WORLD_PATH_VIZ_PORT = 5008   # 과거 anchor로 월드에 박힌 경로 (trajectory_world)
RECV_BUF_SIZE = 65535

T_IDXS = np.array(ModelConstants.T_IDXS, dtype=np.float64)
X_IDXS = np.array(ModelConstants.X_IDXS, dtype=np.float64)
IDX_N = ModelConstants.IDX_N   # 33

# ── 추종기 파라미터 ──────────────────────────────────
WHEELBASE_M = 2.8                    # Pure Pursuit에 사용 (sim 기준 근사)
LOOKAHEAD_MIN_M = 10.0               # 최소 lookahead
LOOKAHEAD_K = 0.6                    # L_d = max(MIN, K * v_ego)  (≈0.6s 선행)
LOOKAHEAD_MAX_M = 20.0

CURV_CLAMP = 0.5                     # ±0.5 1/m
ACCEL_MIN = -3.5
ACCEL_MAX = 2.0

LON_KP = 0.3                         # v_error → accel 게인
LON_USE_FEEDFORWARD = False          # 초기엔 raw_action.accel 사용 안 함 (튜닝 후 on)

STOP_DIST_M = 1.0                    # path 끝까지 남은 거리가 이 값 이하면 정지


# ── JSON 패킷 파싱 ───────────────────────────────────
def parse_action_packet(data: bytes):
    """ac_decoded_path.json 바이트를 파싱해 ego-frame(수신시점) path 반환.
    실패 시 None. 좌표계 변환(y, yaw, curvature 부호 반전)을 여기서 수행.
    반환 path는 x=forward, y=right (openpilot body frame).
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

    # Alpamayo(y=LEFT) → openpilot(y=RIGHT) 변환
    ego_x =  pred_xyz[:, 0].astype(np.float64)
    ego_y = -pred_xyz[:, 1].astype(np.float64)
    ego_z =  pred_xyz[:, 2].astype(np.float64)
    ego_yaw = -pred_yaw.astype(np.float64)
    path_v = pred_v.astype(np.float64)
    a_ff = a.astype(np.float64)                       # feed-forward용 종가속 (ego-frame, t기반)

    return {
        'ego_x': ego_x, 'ego_y': ego_y, 'ego_z': ego_z,
        'ego_yaw': ego_yaw, 'path_v': path_v,
        'a_ff': a_ff, 'dt_s': dt_s, 'N': N,
        'inference_time_s': inference_time_s,
    }


# ── 좌표 변환: 수신시점 ego frame → LocalWorld 전역 frame ──
def path_ego_to_world(pkt, anchor):
    """pkt(수신시점 ego frame, x=fwd/y=right)를 anchor=(x0,y0,yaw0) 기준 LocalWorld 좌표로.
    LocalWorld는 R(yaw) = [[cos,-sin],[sin,cos]] 적분 사용 → body-y는 "right" 가정과 부합하도록
    py 부호를 반전해 world frame에 둔다.
    """
    x0, y0, yaw0 = anchor
    c0, s0 = math.cos(yaw0), math.sin(yaw0)
    px = pkt['ego_x']
    py = pkt['ego_y']      # right-positive
    # body→world (body-y=left 규약이므로 py를 flip)
    wx = x0 + c0 * px - s0 * (-py)
    wy = y0 + s0 * px + c0 * (-py)
    wyaw = yaw0 + (-pkt['ego_yaw'])  # LocalWorld CCW 가정에 맞춰 부호 반전
    return {
        'world_x': wx,                  # (N,)
        'world_y': wy,
        'world_z': pkt['ego_z'],
        'world_yaw': wyaw,
        'path_v': pkt['path_v'],
        'a_ff': pkt['a_ff'],
        'N': pkt['N'],
        'dt_s': pkt['dt_s'],
    }


def path_world_to_current_ego(stored, cur):
    """저장된 world path를 현재 LocalWorld pose로 ego frame(x=fwd,y=right)에 재표현.
    반환: dict with 'x','y','yaw','v' (각 길이 N, np.float64).
    """
    _, xc, yc, yawc = cur
    cc, sc = math.cos(yawc), math.sin(yawc)
    dx = stored['world_x'] - xc
    dy = stored['world_y'] - yc
    # world→body (body-y=left 규약에서 변환 후 py=right로 flip)
    bx =  cc * dx + sc * dy
    by_left = -sc * dx + cc * dy
    px = bx
    py_right = -by_left
    pyaw = -(stored['world_yaw'] - yawc)   # world_yaw→ego yaw (부호 규약 뒤집기)
    return {
        'x': px.astype(np.float64),
        'y': py_right.astype(np.float64),
        'yaw': pyaw.astype(np.float64),
        'v': stored['path_v'].astype(np.float64),
        'z': stored['world_z'].astype(np.float64),
    }


# ── arc-length 및 근사 함수 ──────────────────────────
def path_arclengths(x, y):
    """누적 arc-length (N,) 반환. 첫 값 0."""
    dx = np.diff(x)
    dy = np.diff(y)
    ds = np.hypot(dx, dy)
    return np.concatenate([[0.0], np.cumsum(ds)])


def nearest_index_ahead(x, y):
    """원점(차량 현재 위치) 기준으로 전방(x>0) 중 가장 가까운 점 idx.
    전방에 점이 없으면 전체 중 가장 가까운 idx 반환.
    """
    ahead_mask = x > 0.0
    if ahead_mask.any():
        d2 = x * x + y * y
        d2_masked = np.where(ahead_mask, d2, np.inf)
        return int(np.argmin(d2_masked))
    return int(np.argmin(x * x + y * y))


# ── lateral tracker: Pure Pursuit ────────────────────
def pure_pursuit_curvature(x, y, v_ego):
    """현재 ego-frame path 상에서 lookahead 점 찾아 curvature 반환.
    x=fwd, y=right. curvature>0 이면 우회전(openpilot 규약).
    """
    L_d = max(LOOKAHEAD_MIN_M, min(LOOKAHEAD_MAX_M, LOOKAHEAD_K * max(v_ego, 0.0)))
    # 원점 기준 전방 점 중 |p| >= L_d 인 첫 점
    dist = np.hypot(x, y)
    ahead = x > 0.0
    cand = np.where(ahead & (dist >= L_d))[0]
    if len(cand) > 0:
        i = int(cand[0])
    else:
        # lookahead까지 못 미침 → 가장 먼 전방 점
        ahead_idx = np.where(ahead)[0]
        if len(ahead_idx) == 0:
            return 0.0, L_d
        i = int(ahead_idx[-1])
    tx, ty = x[i], y[i]
    Ld_actual = math.hypot(tx, ty)
    if Ld_actual < 1e-3:
        return 0.0, L_d
    # Pure Pursuit: κ = 2·sin(α)/L_d, α=heading error to target
    # body frame에서 target 각도 α = atan2(y_right, x_fwd) (오른쪽이면 α>0, 우회전)
    alpha = math.atan2(ty, tx)
    kappa = 2.0 * math.sin(alpha) / Ld_actual
    return float(np.clip(kappa, -CURV_CLAMP, CURV_CLAMP)), L_d


# ── longitudinal tracker ─────────────────────────────
def longitudinal_accel(path_ego, v_ego, s_ref_total):
    """현재 위치 기준 전방 nearest 점의 pred_v를 target 삼아 accel 산출.
    s_ref_total: path 전체 arc-length. path 끝까지 남은 거리로 stop 판단.
    """
    x = path_ego['x']; y = path_ego['y']; v_path = path_ego['v']
    i = nearest_index_ahead(x, y)
    v_ref = float(v_path[i])

    # path 끝까지 남은 거리
    s = path_arclengths(x, y)
    remaining = s[-1] - s[i]

    should_stop = False
    if remaining < STOP_DIST_M and v_ref < 0.5:
        v_ref = 0.0
        should_stop = True

    a_cmd = LON_KP * (v_ref - max(v_ego, 0.0))
    if LON_USE_FEEDFORWARD:
        a_cmd += float(path_ego.get('a_ff_at_nearest', 0.0))
    a_cmd = float(np.clip(a_cmd, ACCEL_MIN, ACCEL_MAX))
    return a_cmd, should_stop, v_ref, remaining


# ── 경로 리샘플 (현재 ego-frame path → T_IDXS 33점, 시각화용) ──
def resample_for_viz(path_ego, N_src, dt_src):
    """path_ego(현재 ego frame, arc-length 샘플 아님)를 T_IDXS 33점으로 리샘플.
    path_ego의 원본은 dt_src 간격 시계열이므로 시간축 보간 유지.
    """
    src_t = np.arange(N_src, dtype=np.float64) * dt_src
    x = np.interp(T_IDXS, src_t, path_ego['x']).astype(np.float32)
    y = np.interp(T_IDXS, src_t, path_ego['y']).astype(np.float32)
    z = np.interp(T_IDXS, src_t, path_ego['z']).astype(np.float32)
    yaw = np.interp(T_IDXS, src_t, path_ego['yaw']).astype(np.float32)
    v = np.interp(T_IDXS, src_t, path_ego['v']).astype(np.float32)
    vx = (v * np.cos(yaw)).astype(np.float32)
    vy = (v * np.sin(yaw)).astype(np.float32)
    return {
        'x': x, 'y': y, 'z': z,
        'yaw': yaw, 'v': v,
        'vx': vx, 'vy': vy,
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
    """modelV2 + drivingModelData + longitudinalPlan + driverAssistance 발행.
    rs: resample_path 결과 (또는 default_resampled)
    """
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

    # position — pred_xyz 기반
    fill_xyzt(mv2.position, t_list, rs['x'], rs['y'], rs['z'],
              x_std=low_std, y_std=low_std, z_std=low_std)
    # velocity — pred_v_mps · cos/sin(pred_yaw)
    fill_xyzt(mv2.velocity, t_list, rs['vx'], rs['vy'], zeros_33)
    # acceleration — 0
    fill_xyzt(mv2.acceleration, t_list, zeros_33, zeros_33, zeros_33)
    # orientation (x=roll, y=pitch, z=yaw)
    fill_xyzt(mv2.orientation, t_list, zeros_33, zeros_33, rs['yaw'])
    # orientationRate — 0
    fill_xyzt(mv2.orientationRate, t_list, zeros_33, zeros_33, zeros_33)

    # action
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

    # path polynomial (pred_xyz 기반)
    xyz = np.stack([rs['x'], rs['y'], rs['z']], axis=1)
    coeffs = np.polynomial.polynomial.polyfit(T_IDXS, xyz, deg=ModelConstants.POLY_PATH_DEGREE)
    dmd.path.xCoefficients = coeffs[:, 0].tolist()
    dmd.path.yCoefficients = coeffs[:, 1].tolist()
    dmd.path.zCoefficients = coeffs[:, 2].tolist()

    # lane line meta
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

    # ── send ──
    pm.send('modelV2', modelv2_send)
    pm.send('drivingModelData', dmd_send)
    pm.send('longitudinalPlan', plan_send)
    pm.send('driverAssistance', assist_send)


# ── viz 송신 ─────────────────────────────────────────
def build_world_path(stored):
    """stored(world frame)를 viz 렌더러가 기대하는 dict 리스트로 포장.
    stored['world_x'/'world_y'/'world_yaw']는 이미 LocalWorld(LEFT/CCW) 좌표이므로
    추가 변환 없이 그대로 직렬화만 수행. yaw는 [-π, π]로 wrap.
    """
    xs = np.asarray(stored['world_x'], dtype=np.float64)
    ys = np.asarray(stored['world_y'], dtype=np.float64)
    yaws = np.asarray(stored['world_yaw'], dtype=np.float64)
    yaws_wrapped = np.arctan2(np.sin(yaws), np.cos(yaws))
    path_v = np.asarray(stored['path_v'], dtype=np.float64)
    N = int(stored['N'])
    out = []
    for i in range(N):
        out.append({
            "x": float(xs[i]),
            "y": float(ys[i]),
            "yaw": float(yaws_wrapped[i]),
            "vel": float(path_v[i]),
            "curvature": 0.0,
        })
    return out


def send_vehicle_viz(viz_sock, world, lp, frame_id):
    """LocalWorld 현재 pose + 6초 history를 viz(5006) 로 송신."""
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


def send_world_path_viz(viz_sock, points, dt_s, seq):
    """과거 anchor로 월드에 박힌 경로를 viz(5008) 로 송신."""
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
    cloudlog.warning("udp_bridge init (position-tracking mode)")

    pm = PubMaster(["modelV2", "drivingModelData", "longitudinalPlan", "driverAssistance"])
    sm = SubMaster(["carState", "livePose"])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.setblocking(False)

    viz_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    viz_sock.setblocking(False)

    cloudlog.warning(f"udp_bridge listening on port {UDP_PORT} (JSON ac_decoded_path)")
    cloudlog.warning(f"udp_bridge viz publish: vehicle/trail→{VEHICLE_VIZ_PORT}, "
                     f"raw mirror→{LOCAL_PATH_VIZ_PORT}, worldFrame→{WORLD_PATH_VIZ_PORT}")

    world = LocalWorld()
    frame_id = 0
    stored = None            # world 좌표 path
    pending_pkt = None       # LocalWorld 초기화 대기 중인 ego-frame 패킷
    recv_count = 0
    log_counter = 0

    loop_period = 1.0 / ModelConstants.MODEL_RUN_FREQ  # 50ms = 20Hz

    while True:
        loop_start = time.monotonic()

        # 1. UDP 패킷 수신 (non-blocking, 최신만 사용)
        try:
            while True:
                data, _ = sock.recvfrom(RECV_BUF_SIZE)
                pkt = parse_action_packet(data)
                if pkt is not None:
                    recv_count += 1
                    pkt['recv_mono_ns'] = time.monotonic_ns()
                    pending_pkt = pkt       # anchor 잡기 전까지 보관
                    cloudlog.warning(f"udp_bridge: received plan #{recv_count} "
                                     f"(N={pkt['N']}, dt={pkt['dt_s']:.3f}s, "
                                     f"inference={pkt['inference_time_s']:.3f}s, {len(data)}B) — awaiting anchor")
                    # raw mirror → viz (ego-frame 원본, trajectory_local)
                    try:
                        viz_sock.sendto(data, ('127.0.0.1', LOCAL_PATH_VIZ_PORT))
                    except OSError:
                        pass
        except BlockingIOError:
            pass

        # 2. SubMaster 업데이트 + LocalWorld 적분
        sm.update(0)
        if sm.updated["livePose"]:
            world.update(sm["livePose"], sm.logMonoTime["livePose"])

        # 3. pending 패킷이 있고 LocalWorld 초기화되면 anchor 잡아 world frame으로 저장
        #    inference_time_s 만큼 과거의 ego pose를 LocalWorld history에서 조회해 anchor로 사용
        #    (path[0] = Alpamayo가 캡처한 시점의 ego 위치·방향에 맞물림)
        if pending_pkt is not None and world.is_initialized():
            #inference_time_s = pending_pkt['inference_time_s']
            inference_time_s = 1.0
            past_t_ns = pending_pkt['recv_mono_ns'] - int(inference_time_s * 1e9)
            past = world.at(past_t_ns)       # 범위 밖이면 가장 가까운 끝점으로 clamp
            anchor = (past[1], past[2], past[3])
            stored = path_ego_to_world(pending_pkt, anchor)
            clamped = (past[0] != past_t_ns)
            cloudlog.warning(f"udp_bridge: anchor set at "
                             f"x={anchor[0]:.2f} y={anchor[1]:.2f} yaw={math.degrees(anchor[2]):.1f}° "
                             f"(inference={inference_time_s:.3f}s"
                             f"{', CLAMPED' if clamped else ''})")
            # world path viz → 5008 (anchor에 박힌 상태 그대로 1회 송신)
            world_points = build_world_path(stored)
            send_world_path_viz(viz_sock, world_points, pending_pkt['dt_s'], recv_count)
            pending_pkt = None

        v_ego = max(sm["carState"].vEgo, 0.0)

        # 4. tracker 실행
        if stored is not None and world.is_initialized():
            cur = world.current()
            path_ego = path_world_to_current_ego(stored, cur)

            kappa, L_d = pure_pursuit_curvature(path_ego['x'], path_ego['y'], v_ego)
            a_cmd, should_stop, v_ref, remaining = longitudinal_accel(
                path_ego, v_ego, s_ref_total=None,
            )
            action = log.ModelDataV2.Action(
                desiredCurvature=float(kappa),
                desiredAcceleration=float(a_cmd),
                shouldStop=bool(should_stop),
            )
            rs = resample_for_viz(path_ego, stored['N'], stored['dt_s'])

            log_counter += 1
            if log_counter % 20 == 1:   # 1Hz 로그
                cte = float(path_ego['y'][nearest_index_ahead(path_ego['x'], path_ego['y'])])
                cloudlog.warning(
                    f"track: v_ego={v_ego:.2f} v_ref={v_ref:.2f} cte={cte:+.2f}m "
                    f"κ={kappa:+.4f} a={a_cmd:+.2f} Ld={L_d:.1f} rem={remaining:.1f} stop={should_stop}"
                )
        else:
            action = idle_action()
            rs = default_resampled()

        # 5. 메시지 발행
        publish_messages(pm, rs, action, frame_id, v_ego)

        # 6. viz 송신 — LocalWorld 현재 pose + 6초 trail (5006)
        if world.is_initialized():
            send_vehicle_viz(viz_sock, world, sm["livePose"], frame_id)

        frame_id += 1

        # 7. 20Hz 타이밍 유지
        elapsed = time.monotonic() - loop_start
        sleep_time = loop_period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        cloudlog.warning("udp_bridge got SIGINT")
