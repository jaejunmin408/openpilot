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
from openpilot.selfdrive.controls.lib.local_world import LocalWorld
from openpilot.selfdrive.modeld.constants import ModelConstants

# ── 설정 ──────────────────────────────────────────────
UDP_PORT = 5005
LOCAL_PATH_VIZ_PORT = 5007  # 수신한 Alpamayo JSON 원본을 viz(server.py AcPathUDPProtocol)에 미러 — ego-frame, trajectory_local
VEHICLE_VIZ_PORT = 5006     # LocalWorld 현재 pose + 6초 trail (server.py VehicleStateUDPProtocol, type: vehicle / trajectory)
WORLD_PATH_VIZ_PORT = 5008  # P_snap으로 월드 변환된 Alpamayo 경로 (server.py WorldPathUDPProtocol, type: trajectory_world)
RECV_BUF_SIZE = 65535
MAX_AGE_NS = int(5.5e9)     # inference_age_ns 허용 상한 (LocalWorld HISTORY_SECONDS=6.0s 대비 안전 마진)

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
        'inference_age_ns': int(d.get('inference_age_ns', 0)),
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


# ── worldFrame 변환 + viz 송신 ────────────────────────
def ego_to_world(pose_snap, xs, ys, yaws):
    """ego-frame 점 배열을 P_snap 기준 worldFrame 으로 변환.
    pose_snap: (t_ns, x_s, y_s, yaw_s) — yaw는 orientationNED.z (CW from North).
    반환: (xw, yw, yaw_w) — 각각 np.ndarray, yaw는 [-pi, pi] wrap.
    """
    _, x_s, y_s, yaw_s = pose_snap
    c, s = np.cos(yaw_s), np.sin(yaw_s)
    xs = np.asarray(xs, dtype=np.float64)
    ys = np.asarray(ys, dtype=np.float64)
    yaws = np.asarray(yaws, dtype=np.float64)
    xw = x_s + xs * c - ys * s
    yw = y_s + xs * s + ys * c
    yaw_sum = yaw_s + yaws
    yaw_w = np.arctan2(np.sin(yaw_sum), np.cos(yaw_sum))
    return xw, yw, yaw_w


def send_vehicle_viz(viz_sock, world, lp, frame_id):
    """LocalWorld 현재 pose + 6초 history 를 viz(UDP 5006) 로 송신.
    기존 livepose_to_viz.py 의 publishing 역할을 여기서 흡수한다.
    """
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
            {
                "x": float(hx), "y": float(hy),
                "yaw": float(hyaw),
                "vel": speed,
                "curvature": 0.0,
            }
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
    """P_snap 으로 worldFrame 변환된 경로를 viz(UDP 5008) 로 송신."""
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


def build_world_path(stored, pose_snap):
    """stored(ego-frame) + P_snap → worldFrame point dict 리스트.

    viz 렌더에서 기존 plannedTrajectory(server.py::AcPathUDPProtocol 가 pred_xyz/pred_yaw 를
    부호 반전 없이 그대로 흘려보내고, index.html 의 localToGlobal 이 그 값으로 변환한 것)와
    시각적으로 일치해야 하므로, 여기서도 Alpamayo 원본 축 규약(y=LEFT, yaw=CCW)을 써서
    월드 변환한다. parse_action_packet 가 제어 루프용으로 부호 반전한 값을 한 번 더 뒤집어
    원본으로 복원한 뒤 rotation 적용.
    """
    xs_ego = np.asarray(stored['path_x'], dtype=np.float64)
    ys_ego = -np.asarray(stored['path_y'], dtype=np.float64)      # openpilot RIGHT → Alpamayo LEFT 복원
    yaws_ego = -np.asarray(stored['path_yaw'], dtype=np.float64)  # openpilot CW → Alpamayo CCW 복원

    xw, yw, yaw_w = ego_to_world(pose_snap, xs_ego, ys_ego, yaws_ego)
    path_v = np.asarray(stored['path_v'], dtype=np.float64)
    c_op = np.asarray(stored['c'], dtype=np.float64)
    N = int(stored['N'])
    out = []
    for i in range(N):
        out.append({
            "x": float(xw[i]),
            "y": float(yw[i]),
            "yaw": float(yaw_w[i]),
            "vel": float(path_v[i]),
            "curvature": float(c_op[i]) if i < len(c_op) else 0.0,
        })
    return out


# ── main ──────────────────────────────────────────────
def main():
    cloudlog.warning("udp_bridge init (ac_decoded_path JSON mode, with LocalWorld)")

    pm = PubMaster(["modelV2", "drivingModelData", "longitudinalPlan", "driverAssistance"])
    sm = SubMaster(["carState", "carControl", "liveDelay", "livePose"])
    world = LocalWorld()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.setblocking(False)

    viz_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    viz_sock.setblocking(False)

    cloudlog.warning(f"udp_bridge listening on port {UDP_PORT} (JSON ac_decoded_path)")
    cloudlog.warning(f"udp_bridge viz publish: vehicle/trail→{VEHICLE_VIZ_PORT}, "
                     f"raw mirror→{LOCAL_PATH_VIZ_PORT}, worldFrame→{WORLD_PATH_VIZ_PORT}")

    frame_id = 0
    stored = None        # 가장 최근 수신한 plan
    t_recv = 0.0         # 수신 시각 (monotonic, action 샘플링용)
    recv_count = 0

    loop_period = 1.0 / ModelConstants.MODEL_RUN_FREQ  # 50ms = 20Hz

    while True:
        loop_start = time.monotonic()

        # 1. SubMaster & LocalWorld 업데이트 (packet 처리보다 먼저 — 최신 pose 확보)
        sm.update(0)
        if sm.updated["livePose"]:
            world.update(sm["livePose"], sm.logMonoTime["livePose"])

        # 2. UDP 패킷 수신 (non-blocking, 최신만 사용)
        try:
            while True:
                data, _ = sock.recvfrom(RECV_BUF_SIZE)
                pkt = parse_action_packet(data)
                if pkt is None:
                    continue

                stored = pkt
                t_recv = time.monotonic()
                t_recv_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
                recv_count += 1
                age_ns = pkt['inference_age_ns']
                cloudlog.warning(f"udp_bridge: received action plan #{recv_count} "
                                 f"(N={pkt['N']}, dt={pkt['dt_s']:.3f}s, "
                                 f"age_ns={age_ns}, {len(data)}B)")

                # 2a. 기존 raw mirror (trajectory_local, ego-frame 그대로)
                try:
                    viz_sock.sendto(data, ('127.0.0.1', LOCAL_PATH_VIZ_PORT))
                except OSError:
                    pass

                # 2b. worldFrame 경로 publish (P_snap 기준 앵커링)
                if age_ns < 0 or age_ns > MAX_AGE_NS:
                    cloudlog.warning(f"udp_bridge: inference_age_ns {age_ns} out of range, skipping worldFrame publish")
                    continue

                if not world.is_initialized():
                    cloudlog.warning("udp_bridge: LocalWorld not initialized, skipping worldFrame publish")
                    continue

                t_snap = t_recv_ns - age_ns
                pose_snap = world.at(t_snap)
                if pose_snap is None:
                    cloudlog.warning("udp_bridge: LocalWorld buffer empty, skipping worldFrame publish")
                    continue

                world_points = build_world_path(pkt, pose_snap)
                send_world_path_viz(viz_sock, world_points, pkt['dt_s'], recv_count)
        except BlockingIOError:
            pass

        # 3. action + 리샘플된 경로 계산
        if stored is not None:
            t_rel = time.monotonic() - t_recv
            action = sample_action(stored, t_rel)
            rs = resample_path(stored)
        else:
            action = idle_action()
            rs = default_resampled()

        v_ego = max(sm["carState"].vEgo, 0.0)

        # 4. cereal 메시지 발행 (기존 (a,c) 제어 경로 — 변경 없음)
        publish_messages(pm, rs, action, frame_id, v_ego)

        # 5. viz 송신 — LocalWorld 기반 현재 차량 pose + 6초 trail
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
