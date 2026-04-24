#!/usr/bin/env python3
"""
ADCM 궤적 sender: JSON 테스트 파일을 읽어 ADCM 형식 UDP 패킷으로 전송

패킷 구조 (ADCM 형식):
  Header (16 bytes): magic(uint32) + seq(uint32) + timestamp(float64)
  N points (N * 32 bytes each): x(f64) + y(f64) + yaw(f64) + velocity(f64)
  Ego (24 bytes): ego_x(f64) + ego_y(f64) + ego_yaw(f64)
  Meta (18 bytes): target_accel(f64) + drive_mode(uint8) + emergency(f64) + turn_signal(uint8)
  Footer (1 byte): sizeof_trajectory(uint8)

Usage:
  python3 adcm_trajectory_sender.py [--ip COMMA_IP] [--port 5005] [--json test_trajectory_left_turn.json]
"""

import argparse
import json
import math
import socket
import struct
import time
import sys


MAGIC = 0x41444301  # 'ADC\x01'
HEADER_FMT = "<IId"       # magic, seq, timestamp
POINT_FMT = "<dddd"       # x, y, yaw, velocity (4 doubles)
EGO_FMT = "<ddd"          # ego_x, ego_y, ego_yaw
META_FMT = "<d?dB"        # target_accel, drive_mode, emergency, turn_signal
FOOTER_FMT = "<B"         # sizeof_trajectory

HEADER_SIZE = struct.calcsize(HEADER_FMT)
POINT_SIZE = struct.calcsize(POINT_FMT)
EGO_SIZE = struct.calcsize(EGO_FMT)
META_SIZE = struct.calcsize(META_FMT)
FOOTER_SIZE = struct.calcsize(FOOTER_FMT)

MAX_POINTS = 50
SEND_HZ = 20


def pack_frame(seq, frame):
    """Pack one ADCM frame into UDP bytes."""
    buf = bytearray()

    # Header
    buf += struct.pack(HEADER_FMT, MAGIC, seq, time.time())

    # Trajectory points (x, y, yaw, velocity)
    traj = frame["trajectory"]
    velocities = frame["target_velocity_per_point"]
    n_pts = min(len(traj), MAX_POINTS)

    for i in range(n_pts):
        p = traj[i]
        v = velocities[i] if i < len(velocities) else 0.0
        buf += struct.pack(POINT_FMT, p["x"], p["y"], p["yaw"], v)

    # Pad remaining points with zeros
    for _ in range(MAX_POINTS - n_pts):
        buf += struct.pack(POINT_FMT, 0.0, 0.0, 0.0, 0.0)

    # Ego position
    ego = frame["ego_position"]
    buf += struct.pack(EGO_FMT, ego["x"], ego["y"], ego["yaw"])

    # Metadata
    buf += struct.pack(META_FMT,
                       frame.get("target_speed", 0.0),
                       frame.get("drive_mode", True),
                       frame.get("emergency_acceleration", 1.0),
                       frame.get("turn_signal", 0))

    # Footer
    buf += struct.pack(FOOTER_FMT, n_pts)

    return bytes(buf)


def main():
    parser = argparse.ArgumentParser(description="ADCM trajectory sender")
    parser.add_argument("--ip", default="10.200.147.253", help="Target IP (comma device)")
    parser.add_argument("--port", type=int, default=5005, help="Target UDP port")
    parser.add_argument("--json", default="/home/a/orin/work/test_trajectory_left_turn.json", help="Trajectory JSON file")
    parser.add_argument("--loop", action="store_true", help="Loop trajectory continuously")
    args = parser.parse_args()

    # Load JSON
    print(f"Loading: {args.json}")
    with open(args.json) as f:
        data = json.load(f)
    frames = data["frames"]
    print(f"  {len(frames)} frames, {data.get('sim_hz', 20)}Hz")
    print(f"  Scenario: {data.get('description', 'N/A')}")

    # Compute packet size
    pkt_size = HEADER_SIZE + POINT_SIZE * MAX_POINTS + EGO_SIZE + META_SIZE + FOOTER_SIZE
    print(f"  Packet size: {pkt_size} bytes")
    print(f"\nSending to {args.ip}:{args.port} at {SEND_HZ}Hz")
    print("Press Ctrl+C to stop\n")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    dt = 1.0 / SEND_HZ

    try:
        while True:
            for i, frame in enumerate(frames):
                t_start = time.monotonic()

                pkt = pack_frame(seq, frame)
                sock.sendto(pkt, (args.ip, args.port))

                if seq % SEND_HZ == 0:
                    ego = frame["ego_position"]
                    print(f"[t={frame['time']:.2f}s] seq={seq} "
                          f"ego=({ego['x']:.1f}, {ego['y']:.1f}, yaw={math.degrees(ego['yaw']):.1f}°) "
                          f"turn_signal={frame.get('turn_signal', 0)}")

                seq += 1

                elapsed = time.monotonic() - t_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)

            if not args.loop:
                # Send stop frame
                stop_frame = frames[-1].copy()
                stop_frame["drive_mode"] = False
                stop_frame["target_speed"] = 0.0
                pkt = pack_frame(seq, stop_frame)
                sock.sendto(pkt, (args.ip, args.port))
                print(f"\n[DONE] Sent {seq} frames. Final stop frame sent.")
                break

            print(f"\n--- Loop restart (seq={seq}) ---\n")

    except KeyboardInterrupt:
        # Send stop frame on interrupt
        stop_frame = frames[-1].copy()
        stop_frame["drive_mode"] = False
        stop_frame["target_speed"] = 0.0
        pkt = pack_frame(seq, stop_frame)
        sock.sendto(pkt, (args.ip, args.port))
        print(f"\n[ABORT] Stop frame sent. Total: {seq} frames.")


if __name__ == "__main__":
    main()
