#!/usr/bin/env python3
"""
Alpamayo UDP Bridge
Alpamayo에서 UDP로 전송한 궤적 패킷을 수신하여
modelV2 / drivingModelData / longitudinalPlan / driverAssistance 메시지로 변환/발행한다.
cameraOdometry는 modeld가 카메라 기반으로 발행한다.

패킷 포맷 (PACKET_SPEC.md 참조):
  Header 44B: magic('ALPA') + version(u16) + flags(u16) + tx_seq(u32) + plan_seq(u32)
              + sample_id(u32) + source_t0_us(u64) + tx_time_us(u64)
              + coord_mode(u16) + num_points(u16) + dt_s(f32)
  Points: num_points x 20B (x_m, y_m, yaw_rad, v_mps, curvature as f32)
  Trailer 4B: CRC32
"""
import socket
import struct
import time
import zlib
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
MAX_POINTS = 64
PLAN_USE_SECONDS = 3.0   # 경로 수신 후 사용 시간 (초), 이후 정지

LONG_SMOOTH_SECONDS = 0.3
LAT_SMOOTH_SECONDS = 0.0
MIN_LAT_CONTROL_SPEED = 0.3

T_IDXS = np.array(ModelConstants.T_IDXS, dtype=np.float64)
X_IDXS = np.array(ModelConstants.X_IDXS, dtype=np.float64)
IDX_N = ModelConstants.IDX_N   # 33

# Alpamayo 패킷 상수
ALPA_MAGIC = b'ALPA'
HEADER_FMT = '<4sHHIIIQQHHf'
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 44
POINT_FMT = '<5f'
POINT_SIZE = struct.calcsize(POINT_FMT)    # 20
CRC_SIZE = 4

FLAG_VALID = 1 << 0
FLAG_END_OF_STREAM = 1 << 2


# ── 패킷 파싱 ────────────────────────────────────────
def parse_packet(data: bytes):
    """Alpamayo UDP 패킷 파싱.
    Returns dict with 'x', 'y', 'yaw', 'vel', 'curvature', 'dt_s', 'flags', 'num_points', 'seq'
    or None on failure.
    좌표는 이미 로컬(차량 기준)이므로 변환 불필요.
    """
    if len(data) < HEADER_SIZE + CRC_SIZE:
        return None

    # Header
    hdr = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
    magic = hdr[0]
    if magic != ALPA_MAGIC:
        return None

    flags = hdr[2]
    tx_seq = hdr[3]
    num_points = hdr[9]
    dt_s = hdr[10]

    # 패킷 크기 검증
    expected_size = HEADER_SIZE + num_points * POINT_SIZE + CRC_SIZE
    if len(data) != expected_size:
        return None

    # CRC32 검증
    crc_expected = struct.unpack('<I', data[-CRC_SIZE:])[0]
    crc_actual = zlib.crc32(data[:-CRC_SIZE]) & 0xFFFFFFFF
    if crc_actual != crc_expected:
        return None

    if num_points == 0:
        return None

    # 포인트 파싱 (float32 x 5 per point)
    points_data = np.frombuffer(data, dtype=np.float32,
                                count=num_points * 5,
                                offset=HEADER_SIZE).reshape(num_points, 5).copy()

    return {
        'x': points_data[:, 0],           # meters, local frame
        'y': points_data[:, 1],           # meters, local frame
        'yaw': points_data[:, 2],         # radians
        'vel': points_data[:, 3],         # m/s
        'curvature': points_data[:, 4],   # 1/m
        'dt_s': dt_s,
        'num_points': num_points,
        'flags': flags,
        'seq': tx_seq,
    }


# ── 시간 기반 경로 슬라이싱 ──────────────────────────
def slice_trajectory_at_age(packet, age):
    """저장된 경로에서 age 시점 기준 T_IDXS 33개를 잘라냄.
    현재 ego 위치를 원점으로 회전·평행이동 보정.
    """
    n = packet['num_points']
    dt_s = packet['dt_s']
    src_time = np.arange(n, dtype=np.float64) * dt_s

    # age 시점부터의 미래 시간으로 보간
    query_time = np.clip(T_IDXS + age, 0.0, src_time[-1])

    x_raw   = np.interp(query_time, src_time, packet['x'])
    y_raw   = np.interp(query_time, src_time, packet['y'])
    yaw_raw = np.interp(query_time, src_time, packet['yaw'])
    vel_raw = np.interp(query_time, src_time, packet['vel'])

    # 현재 ego 위치·heading 기준으로 좌표 변환
    x_ref, y_ref, yaw_ref = x_raw[0], y_raw[0], yaw_raw[0]
    dx = x_raw - x_ref
    dy = y_raw - y_ref
    cos_r = np.cos(-yaw_ref)
    sin_r = np.sin(-yaw_ref)
    x_ego = cos_r * dx - sin_r * dy
    y_ego = sin_r * dx + cos_r * dy
    yaw_ego = yaw_raw - yaw_ref

    return {
        'x':    x_ego.astype(np.float32),
        'y':    y_ego.astype(np.float32),
        'yaw':  yaw_ego.astype(np.float32),
        'vel':  vel_raw.astype(np.float32),
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
def compute_action(interp, deriv, prev_action, v_ego, lat_delay, long_delay, flags):
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

    # FLAG_VALID가 없거나 END_OF_STREAM이면 정지
    if not (flags & FLAG_VALID) or (flags & FLAG_END_OF_STREAM):
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


def publish_messages(pm, interp, deriv, action, frame_id, v_ego):
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


# ── main ──────────────────────────────────────────────
def main():
    cloudlog.warning("udp_bridge init (Alpamayo mode)")

    pm = PubMaster(["modelV2", "drivingModelData", "longitudinalPlan", "driverAssistance"])
    sm = SubMaster(["carState", "carControl", "liveDelay"])

    # UDP 소켓 설정 (non-blocking)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.setblocking(False)
    cloudlog.warning(f"udp_bridge listening on port {UDP_PORT} (Alpamayo packet format)")

    frame_id = 0
    prev_action = log.ModelDataV2.Action()

    stored_packet = None    # 최근 수신한 6.4s 경로
    t_recv = 0.0            # 수신 시각 (monotonic)

    loop_period = 1.0 / ModelConstants.MODEL_RUN_FREQ  # 50ms = 20Hz

    while True:
        loop_start = time.monotonic()

        # 1. UDP 패킷 수신 (non-blocking, 최신 패킷만 사용)
        try:
            while True:
                data, addr = sock.recvfrom(HEADER_SIZE + MAX_POINTS * POINT_SIZE + CRC_SIZE + 64)
                packet = parse_packet(data)
                if packet is not None and packet['num_points'] >= 2:
                    stored_packet = packet
                    t_recv = time.monotonic()
                    cloudlog.info(f"udp_bridge: new trajectory received (seq={packet['seq']}, "
                                  f"pts={packet['num_points']}, dt={packet['dt_s']:.3f}s)")
        except BlockingIOError:
            pass

        # 2. 저장된 경로에서 현재 시점 기준 슬라이싱
        if stored_packet is not None:
            age = time.monotonic() - t_recv
            cur_interp = slice_trajectory_at_age(stored_packet, age)
            cur_deriv = compute_derivatives(cur_interp)
            cur_flags = stored_packet['flags']

            # 3초 경과 시 정지
            if age >= PLAN_USE_SECONDS:
                cur_flags = cur_flags & ~FLAG_VALID
        else:
            cur_interp = get_default_interp()
            cur_deriv = get_default_deriv()
            cur_flags = 0

        # 3. SubMaster 업데이트
        sm.update(0)
        v_ego = max(sm["carState"].vEgo, 0.0)

        lat_delay = 0.0
        long_delay = 0.0
        if sm.seen['liveDelay']:
            lat_delay = sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS
        if sm.seen['liveDelay']:
            long_delay = sm["liveDelay"].lateralDelay + LONG_SMOOTH_SECONDS

        # 4. action 계산
        action = compute_action(cur_interp, cur_deriv, prev_action,
                                v_ego, lat_delay, long_delay, cur_flags)
        prev_action = action

        # 5. 메시지 발행
        publish_messages(pm, cur_interp, cur_deriv, action, frame_id, v_ego)

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
