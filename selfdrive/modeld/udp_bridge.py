#!/usr/bin/env python3
"""
Alpamayo UDP Bridge (ac_decoded_path.json 포맷)

외부에서 UDP로 1회 전송한 ac_decoded_path.json 전체(JSON bytes)를 수신하여:
  - raw_action.accel_mps2 / raw_action.curvature 를 0.1s 간격 N=64 샘플로 저장
  - 20Hz 루프에서 수신 후 경과시간에 맞춰 선형 보간해 desiredAcceleration/desiredCurvature 생성
  - pred_xyz / pred_yaw_rad / pred_v_mps 로 modelV2 경로 필드 채워 UI 표시
  - horizon 종료(= (N-1)*dt ≈ 6.3s) 이후에는 shouldStop=True

좌표 변환: Alpamayo(y=LEFT, yaw=CCW, curv=CCW) → openpilot(y=RIGHT, yaw=CW, curv=CW)
           y, yaw, curvature 부호 반전.
"""
import json
import socket
import time
import numpy as np

import cereal.messaging as messaging
from cereal import log
from cereal.messaging import PubMaster, SubMaster
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants

# ── 설정 ──────────────────────────────────────────────
UDP_PORT = 5005
RECV_BUF_SIZE = 65535

T_IDXS = np.array(ModelConstants.T_IDXS, dtype=np.float64)
X_IDXS = np.array(ModelConstants.X_IDXS, dtype=np.float64)
IDX_N = ModelConstants.IDX_N   # 33


# ── JSON 패킷 파싱 ───────────────────────────────────
def parse_action_packet(data: bytes):
    """ac_decoded_path.json 바이트를 파싱해 필요한 필드만 추출.
    실패 시 None.
    좌표계 변환(y, yaw, curvature 부호 반전)도 여기서 수행.
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

    # 좌표 변환: Alpamayo → openpilot
    path_x =  pred_xyz[:, 0].copy()
    path_y = -pred_xyz[:, 1].copy()
    path_z =  pred_xyz[:, 2].copy()
    path_yaw = -pred_yaw.copy()
    path_v = pred_v.copy()
    c_op = -c.copy()

    return {
        'a': a,                  # (N,) m/s² (종방향, 부호 유지)
        'c': c_op,               # (N,) 1/m (openpilot 부호)
        'dt_s': dt_s,
        'N': N,
        'path_x': path_x,
        'path_y': path_y,
        'path_z': path_z,
        'path_yaw': path_yaw,
        'path_v': path_v,
    }


# ── 경로 리샘플 (N점 0.1s 간격 → T_IDXS 33점) ────────
def resample_path(stored):
    """pred_xyz / pred_yaw / pred_v 를 T_IDXS(33점)에 선형 보간.
    N-1 시점 초과분(T_IDXS는 10s까지) 은 마지막 값 유지.
    반환: position/velocity/orientation 채우기용 dict.
    """
    N = stored['N']
    dt = stored['dt_s']
    src_t = np.arange(N, dtype=np.float64) * dt  # [0, 0.1, ..., 6.3]
    # np.interp는 bounds-outside 자동 ZOH (마지막 값 유지)

    x = np.interp(T_IDXS, src_t, stored['path_x']).astype(np.float32)
    y = np.interp(T_IDXS, src_t, stored['path_y']).astype(np.float32)
    z = np.interp(T_IDXS, src_t, stored['path_z']).astype(np.float32)
    yaw = np.interp(T_IDXS, src_t, stored['path_yaw']).astype(np.float32)
    v = np.interp(T_IDXS, src_t, stored['path_v']).astype(np.float32)

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


# ── action 샘플링 (선형 보간) ────────────────────────
def sample_action(stored, t_rel):
    """저장된 a,c 배열에서 t_rel 시점의 값을 선형 보간해 반환.
    horizon((N-1)*dt) 종료 시 shouldStop=True."""
    N = stored['N']
    dt = stored['dt_s']
    horizon = (N - 1) * dt

    if t_rel >= horizon:
        return log.ModelDataV2.Action(
            desiredCurvature=float(stored['c'][-1]),
            desiredAcceleration=0.0,
            shouldStop=True,
        )

    idx = t_rel / dt
    if idx <= 0.0:
        i0 = 0; i1 = 1; f = 0.0
    else:
        i0 = int(idx)
        i1 = min(i0 + 1, N - 1)
        f = idx - i0

    a = (1.0 - f) * stored['a'][i0] + f * stored['a'][i1]
    c = (1.0 - f) * stored['c'][i0] + f * stored['c'][i1]

    return log.ModelDataV2.Action(
        desiredCurvature=float(c),
        desiredAcceleration=float(a),
        shouldStop=False,
    )


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


# ── main ──────────────────────────────────────────────
def main():
    cloudlog.warning("udp_bridge init (ac_decoded_path JSON mode)")

    pm = PubMaster(["modelV2", "drivingModelData", "longitudinalPlan", "driverAssistance"])
    sm = SubMaster(["carState", "carControl", "liveDelay"])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.setblocking(False)
    cloudlog.warning(f"udp_bridge listening on port {UDP_PORT} (JSON ac_decoded_path)")

    frame_id = 0
    stored = None        # 가장 최근 수신한 plan
    t_recv = 0.0         # 수신 시각 (monotonic)
    recv_count = 0

    loop_period = 1.0 / ModelConstants.MODEL_RUN_FREQ  # 50ms = 20Hz

    while True:
        loop_start = time.monotonic()

        # 1. UDP 패킷 수신 (non-blocking, 최신만 사용)
        try:
            while True:
                data, _ = sock.recvfrom(RECV_BUF_SIZE)
                pkt = parse_action_packet(data)
                if pkt is not None:
                    stored = pkt
                    t_recv = time.monotonic()
                    recv_count += 1
                    cloudlog.warning(f"udp_bridge: received action plan #{recv_count} "
                                     f"(N={pkt['N']}, dt={pkt['dt_s']:.3f}s, "
                                     f"{len(data)}B)")
        except BlockingIOError:
            pass

        # 2. action + 리샘플된 경로 계산
        if stored is not None:
            t_rel = time.monotonic() - t_recv
            action = sample_action(stored, t_rel)
            rs = resample_path(stored)
        else:
            action = idle_action()
            rs = default_resampled()

        # 3. SubMaster 업데이트 (v_ego만 사용)
        sm.update(0)
        v_ego = max(sm["carState"].vEgo, 0.0)

        # 4. 메시지 발행
        publish_messages(pm, rs, action, frame_id, v_ego)

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
