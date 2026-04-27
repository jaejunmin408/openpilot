#!/usr/bin/env python3
"""
ADCM 궤적 sender: JSON 테스트 파일을 읽어 ADCM 네이티브 UDP 패킷(1243B)으로 전송.

패킷 구조 (udp_bridge.py 및 tools/sim/test_udp_sender.py 와 일치해야 함):
  Points (1200B): 50 × (x, y, yaw) × float64   -- global 좌표, velocity 없음
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


# 반드시 udp_bridge.py 의 상수와 일치
PACKET_SIZE = 1243
MAX_POINTS = 50


def pack_frame(frame):
    """JSON 한 프레임을 1243B UDP 바이트로 패킹."""
    traj = frame.get("trajectory", [])
    n = min(len(traj), MAX_POINTS)

    # Points (1200B): 50 × (x, y, yaw) × f64, 나머지는 0 패딩
    pts = np.zeros((MAX_POINTS, 3), dtype=np.float64)
    for i in range(n):
        p = traj[i]
        pts[i, 0] = p["x"]
        pts[i, 1] = p["y"]
        pts[i, 2] = p["yaw"]
    points_data = pts.tobytes()

    # Ego (24B)
    ego = frame["ego_position"]
    ego_data = struct.pack("<ddd", ego["x"], ego["y"], ego["yaw"])

    # Meta (18B): target_accel + drive_mode + emergency + turn_signal
    meta_data = struct.pack("<d", float(frame.get("target_speed", 0.0)))
    meta_data += struct.pack("<?", bool(frame.get("drive_mode", True)))
    meta_data += struct.pack("<d", float(frame.get("emergency_acceleration", 0.0)))
    meta_data += struct.pack("<B", int(frame.get("turn_signal", 0)))

    # Footer (1B)
    footer = struct.pack("<B", n)

    pkt = points_data + ego_data + meta_data + footer
    assert len(pkt) == PACKET_SIZE, f"Packet size mismatch: {len(pkt)} != {PACKET_SIZE}"
    return pkt


def main():
    parser = argparse.ArgumentParser(description="ADCM trajectory JSON → UDP sender (1243B)")
    parser.add_argument("--ip", default="127.0.0.1", help="Target IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=10002,
                        help="Target UDP port (udp_bridge default: 10002)")
    parser.add_argument("--json", required=True, help="Trajectory JSON file")
    parser.add_argument("--loop", action="store_true", help="Loop trajectory continuously")
    parser.add_argument("--hz", type=float, default=None,
                        help="Override send rate (default: JSON's sim_hz, fallback 20)")
    args = parser.parse_args()

    print(f"Loading: {args.json}")
    with open(args.json) as f:
        data = json.load(f)
    frames = data["frames"]
    if not frames:
        print("ERROR: JSON 에 frame 이 없습니다.")
        return

    sim_hz = float(args.hz if args.hz is not None else data.get("sim_hz", 20))
    dt = 1.0 / sim_hz

    print(f"  {len(frames)} frames, {sim_hz:.1f}Hz")
    print(f"  Scenario: {data.get('description', 'N/A')}")
    print(f"  Packet size: {PACKET_SIZE} bytes (ADCM native, no header)")
    print(f"\nSending to {args.ip}:{args.port}\nPress Ctrl+C to stop\n")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0

    def send_stop_frame():
        stop_frame = dict(frames[-1])
        stop_frame["drive_mode"] = False
        stop_frame["target_speed"] = 0.0
        sock.sendto(pack_frame(stop_frame), (args.ip, args.port))

    try:
        while True:
            for frame in frames:
                t_start = time.monotonic()

                sock.sendto(pack_frame(frame), (args.ip, args.port))

                if seq % max(int(sim_hz), 1) == 0:
                    ego = frame["ego_position"]
                    print(f"[t={frame.get('time', 0.0):6.2f}s] seq={seq:5d} "
                          f"ego=({ego['x']:7.1f}, {ego['y']:7.1f}, "
                          f"yaw={math.degrees(ego['yaw']):6.1f}°) "
                          f"turn_signal={frame.get('turn_signal', 0)}")

                seq += 1

                elapsed = time.monotonic() - t_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)

            if not args.loop:
                send_stop_frame()
                print(f"\n[DONE] Sent {seq} frames. Stop frame sent.")
                break
            print(f"\n--- Loop restart (seq={seq}) ---\n")

    except KeyboardInterrupt:
        send_stop_frame()
        print(f"\n[ABORT] Stop frame sent. Total: {seq} frames.")


if __name__ == "__main__":
    main()
