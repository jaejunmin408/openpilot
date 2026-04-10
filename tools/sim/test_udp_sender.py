#!/usr/bin/env python3
"""
Test UDP sender for Alpamayo udp_bridge.
Sends a straight-line trajectory at constant speed to verify the full pipeline:
  UDP → udp_bridge → [modelV2, longitudinalPlan] → controlsd → carControl → SimulatedCar
"""
import socket
import struct
import time
import zlib
import argparse
import numpy as np

# Packet constants (must match udp_bridge.py)
ALPA_MAGIC = b'ALPA'
HEADER_FMT = '<4sHHIIIQQHHf'
HEADER_SIZE = struct.calcsize(HEADER_FMT)
POINT_FMT = '<5f'
POINT_SIZE = struct.calcsize(POINT_FMT)

FLAG_VALID = 1 << 0
FLAG_END_OF_STREAM = 1 << 2


def build_packet(points_x, points_y, points_yaw, points_vel, points_curvature,
                 dt_s, tx_seq, flags=FLAG_VALID):
    """Build a complete ALPA UDP packet with CRC."""
    num_points = len(points_x)
    now_us = int(time.time() * 1e6)

    # Header
    header = struct.pack(HEADER_FMT,
        ALPA_MAGIC,    # magic
        1,             # version
        flags,         # flags
        tx_seq,        # tx_seq
        0,             # plan_seq
        0,             # sample_id
        now_us,        # source_t0_us
        now_us,        # tx_time_us
        0,             # coord_mode (local)
        num_points,    # num_points
        dt_s,          # dt_s
    )

    # Points
    points_data = b''
    for i in range(num_points):
        points_data += struct.pack(POINT_FMT,
            float(points_x[i]),
            float(points_y[i]),
            float(points_yaw[i]),
            float(points_vel[i]),
            float(points_curvature[i]),
        )

    # CRC32
    payload = header + points_data
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    trailer = struct.pack('<I', crc)

    return payload + trailer


def make_straight_trajectory(speed_mps, v_ego=0.0, a_max=2.0, dt_s=0.1, num_points=50):
    """Generate a straight-line trajectory that ramps from v_ego to speed_mps."""
    t = np.arange(num_points) * dt_s
    vel = np.minimum(v_ego + a_max * t, speed_mps)
    x = np.cumsum(vel) * dt_s
    x -= x[0]
    y = np.zeros(num_points)
    yaw = np.zeros(num_points)
    curvature = np.zeros(num_points)
    return x, y, yaw, vel, curvature, dt_s


def make_curve_trajectory(speed_mps, radius, v_ego=0.0, a_max=2.0, dt_s=0.1, num_points=50):
    """Generate a curved trajectory (constant radius) that ramps from v_ego to speed_mps.

    속도가 가변이므로 각도는 arc length s = ∫v dt 에서 theta = s/radius 로 계산한다.
    (단순히 theta = omega*t 로 하면 정지 상태에서 출발이 불가능해짐)
    """
    t = np.arange(num_points) * dt_s
    vel = np.minimum(v_ego + a_max * t, speed_mps)
    s = np.cumsum(vel) * dt_s
    s -= s[0]
    theta = s / radius
    x = radius * np.sin(theta)
    y = radius * (1 - np.cos(theta))
    yaw = theta
    curvature = np.full(num_points, 1.0 / radius)
    return x, y, yaw, vel, curvature, dt_s


def main():
    parser = argparse.ArgumentParser(description='Test UDP sender for Alpamayo udp_bridge')
    parser.add_argument('--speed', type=float, default=10.0, help='Target speed m/s (default: 10 = 36km/h)')
    parser.add_argument('--duration', type=float, default=30.0, help='Send duration in seconds')
    parser.add_argument('--hz', type=float, default=20.0, help='Send rate Hz')
    parser.add_argument('--host', default='127.0.0.1', help='Target host')
    parser.add_argument('--port', type=int, default=5005, help='Target port')
    parser.add_argument('--curve', type=float, default=0.0, help='Turn radius (0=straight)')
    parser.add_argument('--stop-at-end', action='store_true', help='Send stop signal at end')
    args = parser.parse_args()

    import cereal.messaging as messaging

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, args.port)
    sm = messaging.SubMaster(['carState'])

    print(f"Sending trajectory: speed={args.speed}m/s ({args.speed*3.6:.0f}km/h), "
          f"{'curve r=' + str(args.curve) + 'm' if args.curve else 'straight'}")
    print(f"Target: {target}, Rate: {args.hz}Hz, Duration: {args.duration}s")
    print("---")

    tx_seq = 0
    period = 1.0 / args.hz
    start = time.monotonic()

    try:
        while time.monotonic() - start < args.duration:
            loop_start = time.monotonic()

            sm.update(0)
            v_ego = max(sm['carState'].vEgo, 0.0)

            if args.curve > 0:
                x, y, yaw, vel, curv, dt_s = make_curve_trajectory(args.speed, args.curve, v_ego=v_ego)
            else:
                x, y, yaw, vel, curv, dt_s = make_straight_trajectory(args.speed, v_ego=v_ego)

            pkt = build_packet(x, y, yaw, vel, curv, dt_s, tx_seq, flags=FLAG_VALID)
            sock.sendto(pkt, target)

            elapsed_total = time.monotonic() - start
            if tx_seq % int(args.hz * 2) == 0:  # print every 2 seconds
                print(f"[{elapsed_total:6.1f}s] seq={tx_seq:5d} v_ego={v_ego*3.6:5.1f}km/h")

            tx_seq += 1

            sleep_time = period - (time.monotonic() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

        if args.stop_at_end:
            print("Sending END_OF_STREAM...")
            x, y, yaw, vel, curv, dt_s = make_straight_trajectory(0.0, v_ego=0.0)
            pkt = build_packet(x, y, yaw, vel, curv, dt_s, tx_seq, flags=FLAG_END_OF_STREAM)
            sock.sendto(pkt, target)

    except KeyboardInterrupt:
        print("\nInterrupted")

    print(f"Done. Sent {tx_seq} packets in {time.monotonic() - start:.1f}s")
    sock.close()


if __name__ == "__main__":
    main()
