#!/usr/bin/env python3
"""
ADCM trajectory sender (projection + smooth-merge):
  reference path JSON 을 읽어 매 tick 차의 GPS 를 path 위에 투영하고
  lateral 오프셋을 smoothstep 으로 감쇠하는 50 점 trajectory 를 그려
  ADCM 네이티브 UDP 패킷(1243B) 으로 송신.

동작:
  1. 첫 GPS+livePose alive 시점에 anchor 캡처 → JSON path 를 GPS 프레임으로
     일괄 변환한 path_world (N, 3) 한 번 캐시.
  2. 매 tick (기본 20Hz):
     - 차의 현재 GPS pose 로 path_world 위 closest index 검색
     - signed lateral error e (path tangent 의 왼쪽이 양수)
     - 50 점 trajectory 생성: i=0 이 ego 정확 일치, i ≥ n_blend 는 path 그대로,
       그 사이는 smoothstep 으로 lateral 오프셋 감쇠
     - 종방향: P-loop 으로 target_accel = clip(Kp · (target_mps − v_ego), …)
     - packet 빌드 & 송신

패킷 구조 (udp_bridge.py 와 일치 필수):
  Points (1200B): 50 × (x, y, yaw) × float64   -- global 좌표
  Ego     (24B) : ego_x, ego_y, ego_yaw        -- 각 float64
  Meta    (18B) : target_accel(f64) + drive_mode(bool) + emergency(f64) + turn_signal(u8)
  Footer   (1B) : sizeof_trajectory (uint8)
  Total = 1243

Usage:
  python3 adcm_trajectory_sender.py --json orin/test_trajectory_metadrive.json --loop
  python3 adcm_trajectory_sender.py --ip 127.0.0.1 --port 10002 --json FILE
"""

import argparse
import json
import math
import socket
import struct
import time

import numpy as np

import cereal.messaging as messaging


# 반드시 udp_bridge.py 의 상수와 일치
PACKET_SIZE = 1243
MAX_POINTS = 50

# GPS lat/lon → m 환원 상수. tools/sim/lib/common.py GPSState.from_xy 와 일치 필수
GPS_BASE_LAT = 32.75308505188913
GPS_BASE_LON = -117.2095393365393
GPS_DEG_TO_METERS = 100000

DEFAULT_HZ = 20.0
DEFAULT_N_BLEND = 20  # 합류 거리 점 수 (≈10m at 0.5m spacing)


def get_pose_from_messages(sm):
    """gpsLocationExternal + livePose 에서 (x, y, yaw) 추출. alive 안 됐으면 None."""
    if not (sm.alive["gpsLocationExternal"] and sm.alive["livePose"]):
        return None
    gps = sm["gpsLocationExternal"]
    lp = sm["livePose"]
    x = (gps.latitude - GPS_BASE_LAT) * GPS_DEG_TO_METERS
    y = (gps.longitude - GPS_BASE_LON) * GPS_DEG_TO_METERS
    # NED → ENU: 절대값 맞추기 (East 시작에서 yaw=0) + 회전 방향 맞추기
    yaw = float(lp.orientationNED.z) - math.pi / 2
    return (x, y, yaw)


def make_transform_params(anchor_pose, json_first_pose):
    """JSON 좌표계 점을 anchor_pose 기준 GPS 프레임으로 매핑하는 파라미터 반환.
    매핑:
      rx = (x - jx) * cos_d - (y - jy) * sin_d + ax
      ry = (x - jx) * sin_d + (y - jy) * cos_d + ay
      ryaw = yaw + delta_yaw
    """
    ax, ay, ayaw = anchor_pose
    jx, jy, jyaw = json_first_pose
    delta_yaw = ayaw - jyaw
    return (ax, ay, jx, jy, delta_yaw, math.cos(delta_yaw), math.sin(delta_yaw))


def apply_transform_to_path(transform, path_json):
    """JSON path (N, 3) 을 GPS 프레임으로 일괄 변환."""
    ax, ay, jx, jy, delta_yaw, cos_d, sin_d = transform
    dx = path_json[:, 0] - jx
    dy = path_json[:, 1] - jy
    rx = dx * cos_d - dy * sin_d + ax
    ry = dx * sin_d + dy * cos_d + ay
    ryaw = path_json[:, 2] + delta_yaw
    return np.stack([rx, ry, ryaw], axis=1)


def smoothstep(t):
    """smoothstep(0)=0, smoothstep(1)=1, 양 끝 1차 미분=0."""
    return 3.0 * t * t - 2.0 * t * t * t


def build_smooth_trajectory(ego, path_world, i_closest, n_blend, n_points=MAX_POINTS):
    """ego 에서 출발해 path_world 의 lane center 로 부드럽게 합류하는 50 점 생성.

    구성:
      pts[i] = path[(i_closest+i) % N] + e · (1 − smoothstep(i/n_blend)) · perp_i
    그리고 pts[0] 은 ego 로 강제. closest 점이 vertex 라 ~spacing/2 의 longitudinal
    오차가 있을 수 있어, udp_bridge 의 rel[0]=(0,0,0) 가정을 정확히 만족시키려면 강제가
    필요하다.

    yaw:
      pts[0].yaw = ego.yaw  (rel_yaw[0]=0)
      pts[i].yaw (1≤i≤n−2) = atan2(pts[i+1] − pts[i])  (forward-diff)
      pts[n−1].yaw = path tangent
    """
    gx, gy, gyaw = ego
    N = len(path_world)

    # signed lateral error: path tangent 의 왼쪽 = +
    px, py, pyaw = path_world[i_closest]
    perp_cx, perp_cy = -math.sin(pyaw), math.cos(pyaw)
    e = (gx - px) * perp_cx + (gy - py) * perp_cy

    # base 점들 (i_closest..i_closest+n_points−1)
    idxs = (i_closest + np.arange(n_points)) % N
    base = path_world[idxs]                       # (n_points, 3)
    base_yaw = base[:, 2]
    perp_x = -np.sin(base_yaw)
    perp_y = np.cos(base_yaw)

    # blend weight: 1 at i=0, 0 at i ≥ n_blend
    i_arr = np.arange(n_points, dtype=np.float64)
    t = np.minimum(i_arr / max(float(n_blend), 1.0), 1.0)
    w = 1.0 - smoothstep(t)

    pts = np.zeros((n_points, 3), dtype=np.float64)
    pts[:, 0] = base[:, 0] + e * w * perp_x
    pts[:, 1] = base[:, 1] + e * w * perp_y

    # pts[0] = ego 강제 (longitudinal 오차 흡수, rel[0]=(0,0,0) 보장)
    pts[0, 0] = gx
    pts[0, 1] = gy
    pts[0, 2] = gyaw

    # forward-diff yaw for 1..n−2
    if n_points > 2:
        dxs = pts[2:, 0] - pts[1:-1, 0]
        dys = pts[2:, 1] - pts[1:-1, 1]
        pts[1:-1, 2] = np.arctan2(dys, dxs)

    # 마지막 점은 path tangent
    pts[-1, 2] = base_yaw[-1]

    return pts


def pack_packet(points, ego, target_accel, drive_mode, emergency, turn_signal, n_valid):
    """50 점 + ego + meta → 1243B UDP 패킷."""
    # Points (1200B): 50 × (x, y, yaw) × f64. n_valid 미만은 0 패딩.
    pts = np.zeros((MAX_POINTS, 3), dtype=np.float64)
    n = max(0, min(int(n_valid), MAX_POINTS))
    pts[:n] = points[:n]
    points_data = pts.tobytes()

    ego_data = struct.pack("<ddd", ego[0], ego[1], ego[2])

    meta_data = struct.pack("<d", float(target_accel))
    meta_data += struct.pack("<?", bool(drive_mode))
    meta_data += struct.pack("<d", float(emergency))
    meta_data += struct.pack("<B", int(turn_signal) & 0xFF)

    footer = struct.pack("<B", n & 0xFF)

    pkt = points_data + ego_data + meta_data + footer
    assert len(pkt) == PACKET_SIZE, f"Packet size mismatch: {len(pkt)} != {PACKET_SIZE}"
    return pkt


def _sleep_remainder(t_start, dt):
    elapsed = time.monotonic() - t_start
    if elapsed < dt:
        time.sleep(dt - elapsed)


def main():
    parser = argparse.ArgumentParser(
        description="ADCM reference path JSON → projection-based UDP sender (1243B)")
    parser.add_argument("--ip", default="127.0.0.1", help="Target IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=10002,
                        help="Target UDP port (udp_bridge default: 10002)")
    parser.add_argument("--json", required=True, help="Reference path JSON file")
    parser.add_argument("--loop", action="store_true",
                        help="path 끝 도달 시 wrap (closed loop). 미지정 시 stop frame 후 종료")
    parser.add_argument("--hz", type=float, default=DEFAULT_HZ,
                        help=f"송신 주기 Hz (default: {DEFAULT_HZ})")
    parser.add_argument("--n-blend", type=int, default=DEFAULT_N_BLEND,
                        help=f"lateral 합류 점 수 (default: {DEFAULT_N_BLEND}, ≈10m@0.5m spacing)")
    parser.add_argument("--kp", type=float, default=1.0,
                        help="속도 P 게인 (m/s² per m/s err, default: 1.0)")
    parser.add_argument("--max-accel", type=float, default=2.0,
                        help="max target accel m/s² (default: 2.0)")
    parser.add_argument("--max-decel", type=float, default=2.0,
                        help="max target decel m/s² (절댓값, default: 2.0)")
    parser.add_argument("--target-speed", type=float, default=None,
                        help="폐루프 속도 목표 m/s (default: JSON speed_mps)")
    args = parser.parse_args()

    print(f"Loading: {args.json}")
    with open(args.json) as f:
        data = json.load(f)
    path_pts = data.get("path")
    if not path_pts:
        print("ERROR: JSON 에 'path' 가 없습니다.")
        return

    path_json = np.array([(p["x"], p["y"], p["yaw"]) for p in path_pts], dtype=np.float64)

    target_mps = float(args.target_speed if args.target_speed is not None
                       else data.get("speed_mps", 2.78))
    drive_mode_const = bool(data.get("drive_mode", True))
    turn_signal_const = int(data.get("turn_signal", 0))
    emergency_const = float(data.get("emergency_acceleration", 0.0))
    spacing = float(data.get("point_spacing_m", 0.5))

    dt = 1.0 / float(args.hz)

    print(f"  {len(path_json)} path 점, {data.get('total_path_length_m', 0.0):.1f}m, "
          f"spacing={spacing:.2f}m")
    print(f"  Scenario: {data.get('description', 'N/A')}")
    print(f"  Speed P-loop: target={target_mps:.2f}m/s, Kp={args.kp:.2f}, "
          f"accel∈[{-args.max_decel:+.1f}, {args.max_accel:+.1f}] m/s²")
    print(f"  Smooth merge: n_blend={args.n_blend} (≈{args.n_blend * spacing:.1f}m)")
    print(f"  Loop: {args.loop}")
    print(f"  Packet: {PACKET_SIZE} bytes (ADCM native)")
    print(f"\nSending to {args.ip}:{args.port} @ {args.hz:.1f}Hz\nPress Ctrl+C to stop\n")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sm = messaging.SubMaster(["carState", "gpsLocationExternal", "livePose"])

    json_first_pose = (float(path_json[0, 0]), float(path_json[0, 1]), float(path_json[0, 2]))

    path_world = None
    pose_warn_last = 0.0
    seq = 0
    last_i = 0
    last_ego = (0.0, 0.0, 0.0)

    def send_stop_packet(ego):
        zero_pts = np.zeros((MAX_POINTS, 3), dtype=np.float64)
        sock.sendto(
            pack_packet(zero_pts, ego, target_accel=0.0,
                        drive_mode=False, emergency=emergency_const,
                        turn_signal=turn_signal_const, n_valid=0),
            (args.ip, args.port),
        )

    try:
        while True:
            t_start = time.monotonic()

            sm.update(0)
            v_ego = max(sm["carState"].vEgo, 0.0)

            cur_pose = get_pose_from_messages(sm)
            if cur_pose is None:
                if t_start - pose_warn_last > 2.0:
                    print("[anchor] waiting for gpsLocationExternal+livePose alive…")
                    pose_warn_last = t_start
                _sleep_remainder(t_start, dt)
                continue
            last_ego = cur_pose

            # 첫 alive 에서 anchor 캡처 + path 를 GPS 프레임으로 변환
            if path_world is None:
                transform = make_transform_params(cur_pose, json_first_pose)
                path_world = apply_transform_to_path(transform, path_json)
                print(f"[anchor] gps=({cur_pose[0]:.1f}, {cur_pose[1]:.1f}) "
                      f"yaw={math.degrees(cur_pose[2]):.1f}°  "
                      f"json_first=({json_first_pose[0]:.1f}, {json_first_pose[1]:.1f}) "
                      f"yaw={math.degrees(json_first_pose[2]):.1f}°  "
                      f"path={len(path_world)} 점")

            # closest path index
            d2 = (path_world[:, 0] - cur_pose[0]) ** 2 + (path_world[:, 1] - cur_pose[1]) ** 2
            i_closest = int(np.argmin(d2))

            # path 끝 도달 (non-loop)
            if not args.loop and i_closest >= len(path_world) - MAX_POINTS:
                print(f"\n[DONE] path 끝 도달 (i_closest={i_closest}/{len(path_world)}). "
                      f"Stop frame 송신.")
                send_stop_packet(cur_pose)
                break

            # smooth-merge trajectory
            pts = build_smooth_trajectory(cur_pose, path_world, i_closest, args.n_blend)

            # P-loop accel
            err = target_mps - v_ego
            target_accel = float(np.clip(args.kp * err, -args.max_decel, args.max_accel))

            packet = pack_packet(
                pts, cur_pose,
                target_accel=target_accel,
                drive_mode=drive_mode_const,
                emergency=emergency_const,
                turn_signal=turn_signal_const,
                n_valid=MAX_POINTS,
            )
            sock.sendto(packet, (args.ip, args.port))

            if seq % max(int(args.hz), 1) == 0:
                px, py, pyaw = path_world[i_closest]
                perp_cx, perp_cy = -math.sin(pyaw), math.cos(pyaw)
                lat_err = (cur_pose[0] - px) * perp_cx + (cur_pose[1] - py) * perp_cy
                print(f"[seq={seq:5d}] i={i_closest:5d}/{len(path_world)} "
                      f"v_ego={v_ego * 3.6:5.1f}km/h a_tgt={target_accel:+.1f} "
                      f"lat_err={lat_err:+.2f}m "
                      f"ego=({cur_pose[0]:7.1f}, {cur_pose[1]:7.1f}, "
                      f"yaw={math.degrees(cur_pose[2]):6.1f}°)")

            # loop wrap detection (큰 점프로 단조 감소 → 한 바퀴)
            if args.loop and i_closest < last_i - len(path_world) // 2:
                print(f"\n--- Loop wrap (i: {last_i} → {i_closest}) ---\n")
            last_i = i_closest

            seq += 1
            _sleep_remainder(t_start, dt)

    except KeyboardInterrupt:
        send_stop_packet(last_ego)
        print(f"\n[ABORT] Stop frame sent. Total: {seq} frames.")


if __name__ == "__main__":
    main()
