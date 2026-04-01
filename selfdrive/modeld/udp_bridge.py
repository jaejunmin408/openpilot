#!/usr/bin/env python3
"""
ADCM UDP Bridge
Orin ADCM에서 UDP로 전송한 궤적 패킷을 수신하여
modelV2 / drivingModelData 메시지로 변환/발행한다.
cameraOdometry는 modeld가 카메라 기반으로 발행한다.
"""
import socket
import struct
import time
import numpy as np

import cereal.messaging as messaging
from cereal import log
from cereal.messaging import PubMaster, SubMaster
from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.drive_helpers import (
  get_accel_from_plan, get_curvature_from_plan, smooth_value,
)

# ── 설정 ──────────────────────────────────────────────
UDP_PORT = 5005
UDP_TIMEOUT_S = 0.05          # 50 ms
PACKET_SIZE = 1659
MAGIC = 0x41444301
MAX_POINTS = 50

LONG_SMOOTH_SECONDS = 0.3
LAT_SMOOTH_SECONDS = 0.0
MIN_LAT_CONTROL_SPEED = 0.3

T_IDXS = np.array(ModelConstants.T_IDXS, dtype=np.float64)
X_IDXS = np.array(ModelConstants.X_IDXS, dtype=np.float64)
IDX_N = ModelConstants.IDX_N   # 33


# ── 패킷 파싱 ────────────────────────────────────────
def parse_packet(data: bytes):
    """1659 byte ADCM UDP 패킷 파싱.
    Returns (points, ego, meta) or None on failure.
      points: (n_valid, 4) float64  -- x, y, yaw, velocity per point
      ego:    dict  -- ego_x, ego_y, ego_yaw
      meta:   dict  -- target_accel, drive_mode, emergency, turn_signal
    """
    if len(data) != PACKET_SIZE:
        return None

    # Header (16B)
    magic, seq, timestamp = struct.unpack_from('<IId', data, 0)
    if magic != MAGIC:
        return None

    offset = 16

    # Points (1600B = 50 x 4 x float64)
    points = np.frombuffer(data, dtype=np.float64, count=MAX_POINTS * 4, offset=offset).reshape(MAX_POINTS, 4).copy()
    offset += MAX_POINTS * 4 * 8

    # Ego (24B)
    ego_x, ego_y, ego_yaw = struct.unpack_from('<ddd', data, offset)
    offset += 24

    # Meta (18B): target_accel(8) + drive_mode(1) + emergency(8) + turn_signal(1)
    target_accel = struct.unpack_from('<d', data, offset)[0]
    offset += 8
    drive_mode = struct.unpack_from('<?', data, offset)[0]
    offset += 1
    emergency = struct.unpack_from('<d', data, offset)[0]
    offset += 8
    turn_signal = struct.unpack_from('<B', data, offset)[0]
    offset += 1

    # Footer (1B)
    n_valid = struct.unpack_from('<B', data, offset)[0]
    n_valid = min(n_valid, MAX_POINTS)

    ego = {'x': ego_x, 'y': ego_y, 'yaw': ego_yaw}
    meta = {
        'target_accel': target_accel,
        'drive_mode': drive_mode,
        'emergency': emergency,
        'turn_signal': turn_signal,
        'n_valid': n_valid,
        'seq': seq,
        'timestamp': timestamp,
    }
    return points[:n_valid], ego, meta


# ── 좌표 변환 ────────────────────────────────────────
def to_relative(points: np.ndarray, ego: dict):
    """글로벌 좌표 -> 차량 기준 좌표 변환."""
    dx = points[:, 0] - ego['x']
    dy = points[:, 1] - ego['y']
    c = np.cos(-ego['yaw'])
    s = np.sin(-ego['yaw'])
    rel_x = dx * c - dy * s
    rel_y = dx * s + dy * c
    rel_yaw = points[:, 2] - ego['yaw']
    velocity = points[:, 3]
    return rel_x, rel_y, rel_yaw, velocity


# ── 시간축 생성 ──────────────────────────────────────
def build_cumulative_time(rel_x: np.ndarray, rel_y: np.ndarray, velocity: np.ndarray):
    """거리 + 속도 -> 누적 시간 배열 생성."""
    n = len(rel_x)
    cum_time = np.zeros(n, dtype=np.float64)
    for i in range(1, n):
        ds = np.hypot(rel_x[i] - rel_x[i - 1], rel_y[i] - rel_y[i - 1])
        v_avg = max((velocity[i] + velocity[i - 1]) / 2.0, 0.1)
        cum_time[i] = cum_time[i - 1] + ds / v_avg
    return cum_time


# ── T_IDXS 보간 ──────────────────────────────────────
def interpolate_to_tidxs(rel_x, rel_y, rel_yaw, velocity, cum_time):
    """원본 포인트를 T_IDXS 33개로 보간."""
    ix = np.interp(T_IDXS, cum_time, rel_x)
    iy = np.interp(T_IDXS, cum_time, rel_y)
    iyaw = np.interp(T_IDXS, cum_time, rel_yaw)
    ivel = np.interp(T_IDXS, cum_time, velocity)
    return {
        'x': ix.astype(np.float32),
        'y': iy.astype(np.float32),
        'yaw': iyaw.astype(np.float32),
        'vel': ivel.astype(np.float32),
    }


# ── 미분값 계산 ──────────────────────────────────────
def compute_derivatives(interp: dict):
    """velocity_x/y, acceleration_x/y, yaw_rate 계산."""
    yaw = interp['yaw']
    vel = interp['vel']
    vx = vel * np.cos(yaw)
    vy = vel * np.sin(yaw)
    ax = np.gradient(vx, T_IDXS).astype(np.float32)
    ay = np.gradient(vy, T_IDXS).astype(np.float32)
    yaw_rate = np.gradient(yaw, T_IDXS).astype(np.float32)
    return {
        'vx': vx.astype(np.float32),
        'vy': vy.astype(np.float32),
        'ax': ax,
        'ay': ay,
        'yaw_rate': yaw_rate,
    }


# ── action 계산 ──────────────────────────────────────
def compute_action(interp, deriv, prev_action, v_ego, lat_delay, long_delay, adcm_meta):
    """desiredCurvature, desiredAcceleration, shouldStop 계산."""
    plan_vel_x = deriv['vx']
    plan_acc_x = deriv['ax']
    plan_yaw = interp['yaw']
    plan_yaw_rate = deriv['yaw_rate']

    # 종방향
    desired_accel, should_stop = get_accel_from_plan(
        plan_vel_x, plan_acc_x, T_IDXS, action_t=long_delay + DT_MDL,
    )
    desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, LONG_SMOOTH_SECONDS)

    if not adcm_meta['drive_mode'] or adcm_meta['n_valid'] == 0:
        should_stop = True

    # 횡방향
    desired_curvature = get_curvature_from_plan(
        plan_yaw, plan_yaw_rate, T_IDXS, v_ego, lat_delay + DT_MDL,
    )
    if v_ego > MIN_LAT_CONTROL_SPEED:
        desired_curvature = smooth_value(desired_curvature, prev_action.desiredCurvature, LAT_SMOOTH_SECONDS)
    else:
        desired_curvature = prev_action.desiredCurvature

    return log.ModelDataV2.Action(
        desiredCurvature=float(desired_curvature),
        desiredAcceleration=float(desired_accel),
        shouldStop=bool(should_stop),
    )


# ── 메시지 발행 ──────────────────────────────────────
def fill_xyzt(builder, t, x, y, z, x_std=None, y_std=None, z_std=None):
    builder.t = t
    builder.x = x.tolist()
    builder.y = y.tolist()
    builder.z = z.tolist()
    if x_std is not None:
        builder.xStd = x_std.tolist()
    if y_std is not None:
        builder.yStd = y_std.tolist()
    if z_std is not None:
        builder.zStd = z_std.tolist()


def publish_messages(pm, interp, deriv, action, frame_id, adcm_meta):
    """modelV2 + drivingModelData 발행. (cameraOdometry는 modeld가 담당)"""
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

    # position
    fill_xyzt(mv2.position, t_list, interp['x'], interp['y'], zeros_33,
              x_std=low_std, y_std=low_std, z_std=low_std)
    # velocity
    fill_xyzt(mv2.velocity, t_list, deriv['vx'], deriv['vy'], zeros_33)
    # acceleration
    fill_xyzt(mv2.acceleration, t_list, deriv['ax'], deriv['ay'], zeros_33)
    # orientation (x=roll, y=pitch, z=yaw)
    fill_xyzt(mv2.orientation, t_list, zeros_33, zeros_33, interp['yaw'])
    # orientationRate
    fill_xyzt(mv2.orientationRate, t_list, zeros_33, zeros_33, deriv['yaw_rate'])

    # action
    mv2.action = action

    # lane lines (4, dummy)
    mv2.init('laneLines', 4)
    default_lane_y = [-1.8, -1.8, 1.8, 1.8]
    for i in range(4):
        ll = mv2.laneLines[i]
        lane_y = np.full(IDX_N, default_lane_y[i], dtype=np.float32)
        fill_xyzt(ll, [], X_IDXS.astype(np.float32), lane_y, zeros_33)
    mv2.laneLineStds = [0.0, 0.0, 0.0, 0.0]
    mv2.laneLineProbs = [0.0, 0.0, 0.0, 0.0]

    # road edges (2, dummy)
    mv2.init('roadEdges', 2)
    default_edge_y = [-3.0, 3.0]
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

    # path polynomial
    xyz = np.stack([interp['x'], interp['y'], zeros_33], axis=1)
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

    # ── send ──
    pm.send('modelV2', modelv2_send)
    pm.send('drivingModelData', dmd_send)


# ── 기본 보간 결과 (패킷 수신 전 또는 실패 시) ────────
def get_default_interp():
    return {
        'x': np.zeros(IDX_N, dtype=np.float32),
        'y': np.zeros(IDX_N, dtype=np.float32),
        'yaw': np.zeros(IDX_N, dtype=np.float32),
        'vel': np.zeros(IDX_N, dtype=np.float32),
    }

def get_default_deriv():
    return {
        'vx': np.zeros(IDX_N, dtype=np.float32),
        'vy': np.zeros(IDX_N, dtype=np.float32),
        'ax': np.zeros(IDX_N, dtype=np.float32),
        'ay': np.zeros(IDX_N, dtype=np.float32),
        'yaw_rate': np.zeros(IDX_N, dtype=np.float32),
    }

DEFAULT_META = {
    'target_accel': 0.0,
    'drive_mode': False,
    'emergency': 0.0,
    'turn_signal': 0,
    'n_valid': 0,
    'seq': 0,
    'timestamp': 0.0,
}


# ── main ──────────────────────────────────────────────
def main():
    cloudlog.warning("udp_bridge init")

    pm = PubMaster(["modelV2", "drivingModelData"])
    sm = SubMaster(["carState", "carControl", "liveDelay"])

    # UDP 소켓 설정
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.setblocking(False)
    cloudlog.warning(f"udp_bridge listening on port {UDP_PORT}")

    frame_id = 0
    prev_action = log.ModelDataV2.Action()

    cur_interp = get_default_interp()
    cur_deriv = get_default_deriv()
    cur_meta = DEFAULT_META.copy()

    loop_period = 1.0 / ModelConstants.MODEL_RUN_FREQ  # 50ms = 20Hz

    while True:
        loop_start = time.monotonic()

        # 1. UDP 패킷 수신 (timeout 내에 최신 패킷 사용)
        packet_received = False
        try:
            while True:
                data, addr = sock.recvfrom(2048)
                result = parse_packet(data)
                if result is not None:
                    points, ego, meta = result
                    if meta['n_valid'] >= 2:
                        rel_x, rel_y, rel_yaw, velocity = to_relative(points, ego)
                        cum_time = build_cumulative_time(rel_x, rel_y, velocity)
                        if cum_time[-1] > 0.1:
                            cur_interp = interpolate_to_tidxs(rel_x, rel_y, rel_yaw, velocity, cum_time)
                            cur_deriv = compute_derivatives(cur_interp)
                            cur_meta = meta
                            packet_received = True
        except BlockingIOError:
            pass

        # 2. SubMaster 업데이트
        sm.update(0)
        v_ego = max(sm["carState"].vEgo, 0.0)

        lat_delay = 0.0
        long_delay = 0.0
        if sm.seen['liveDelay']:
            lat_delay = sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS
        if sm.seen['liveDelay']:
            long_delay = sm["liveDelay"].lateralDelay + LONG_SMOOTH_SECONDS

        # 3. action 계산
        action = compute_action(cur_interp, cur_deriv, prev_action,
                                v_ego, lat_delay, long_delay, cur_meta)
        prev_action = action

        # 4. 메시지 발행 (modelV2 + drivingModelData only)
        publish_messages(pm, cur_interp, cur_deriv, action, frame_id, cur_meta)

        frame_id += 1

        # 5. 20Hz 타이밍 유지
        elapsed = time.monotonic() - loop_start
        sleep_time = loop_period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        cloudlog.warning("udp_bridge got SIGINT")
