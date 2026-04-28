#!/usr/bin/env python3
"""
Test UDP sender for ADCM udp_bridge — sim 모드에서 실 ADCM 을 모방.
Sends a trajectory at constant speed to verify the full pipeline:
  UDP -> udp_bridge -> [modelV2, drivingModelData, longitudinalPlan] -> controlsd -> carControl -> SimulatedCar

Ego pose source (sim 모드, 실 ADCM 동작 모방):
  - position: gpsLocationExternal.{latitude, longitude} → MetaDrive xy 환원
  - heading : livePose.orientationNED.z → ENU yaw 변환

Packet format (1243 bytes, must match udp_bridge.py / xDrivingTrajectory_UdpPacket):
  Points (1200B): 50 x 3 x float64  (x, y, yaw)  -- global coords, no velocity
  Ego     (24B): ego_x(8) + ego_y(8) + ego_yaw(8)
  Meta    (18B): target_accel(8) + drive_mode(1) + emergency(8) + turn_signal(1)
  Footer   (1B): sizeof_trajectory (n_valid)
  Total = 1243
"""
import math
import socket
import struct
import time
import argparse
import numpy as np

# Packet constants (must match udp_bridge.py)
PACKET_SIZE = 1243
MAX_POINTS = 50

# GPS lat/lon → m 환원 상수. tools/sim/lib/common.py GPSState.from_xy 와 일치 필수
GPS_BASE_LAT = 32.75308505188913
GPS_BASE_LON = -117.2095393365393
GPS_DEG_TO_METERS = 100000


def build_packet(points_xyz, ego, meta):
    """Build a 1243-byte ADCM UDP packet.

    Args:
        points_xyz: (n, 3) float64 array -- x, y, yaw (global coords, no velocity)
        ego:        dict with keys x, y, yaw
        meta:       dict with keys target_accel, drive_mode, emergency, turn_signal
    """
    n_valid = min(len(points_xyz), MAX_POINTS)

    # Points (1200B) -- pad to MAX_POINTS
    pts = np.zeros((MAX_POINTS, 3), dtype=np.float64)
    pts[:n_valid] = points_xyz[:n_valid]
    points_data = pts.tobytes()

    # Ego (24B)
    ego_data = struct.pack('<ddd', ego['x'], ego['y'], ego['yaw'])

    # Meta (18B): target_accel(8) + drive_mode(1) + emergency(8) + turn_signal(1)
    meta_data = struct.pack('<d', meta['target_accel'])
    meta_data += struct.pack('<?', meta['drive_mode'])
    meta_data += struct.pack('<d', meta['emergency'])
    meta_data += struct.pack('<B', meta['turn_signal'])

    # Footer (1B)
    footer = struct.pack('<B', n_valid)

    pkt = points_data + ego_data + meta_data + footer
    assert len(pkt) == PACKET_SIZE, f"Packet size mismatch: {len(pkt)} != {PACKET_SIZE}"
    return pkt


def make_straight_trajectory(ego_x, ego_y, ego_yaw, spacing, num_points=50):
    """Generate a straight-line trajectory in global coordinates.
    Arc-length spacing (like real ADCM): each point is 'spacing' meters apart.
    """
    s = np.arange(num_points) * spacing
    x = ego_x + s * np.cos(ego_yaw)
    y = ego_y + s * np.sin(ego_yaw)
    yaw = np.full(num_points, ego_yaw)
    return np.column_stack([x, y, yaw])


def make_curve_trajectory(radius, ego_x, ego_y, ego_yaw, spacing, num_points=50):
    """Generate a curved trajectory (constant radius) in global coordinates.
    Arc-length spacing. Positive radius = left turn, negative = right turn.
    """
    s = np.arange(num_points) * spacing
    theta = s / abs(radius)
    sign = 1.0 if radius > 0 else -1.0

    cx = ego_x - sign * abs(radius) * np.sin(ego_yaw)
    cy = ego_y + sign * abs(radius) * np.cos(ego_yaw)

    angle = ego_yaw - sign * np.pi / 2 + sign * theta
    x = cx + abs(radius) * np.cos(angle)
    y = cy + abs(radius) * np.sin(angle)
    yaw = ego_yaw + sign * theta
    return np.column_stack([x, y, yaw])


def compute_spacing(v_ego):
    """Replicate ADCM spacing logic: horizon = clamp(v * clamp(0.2*v, 2, 5), 20, 200).
    spacing = horizon / 79, but only first 50 points are sent via UDP.
    """
    lookahead_time = np.clip(0.2 * v_ego, 2.0, 5.0)
    horizon = np.clip(v_ego * lookahead_time, 20.0, 200.0)
    return horizon / 79.0


def main():
    parser = argparse.ArgumentParser(description='Test UDP sender for ADCM udp_bridge')
    parser.add_argument('--speed', type=float, default=10.0, help='Target speed m/s (default: 10 = 36km/h)')
    parser.add_argument('--accel', type=float, default=2.0, help='Target acceleration m/s^2 (default: 2.0)')
    parser.add_argument('--duration', type=float, default=30.0, help='Send duration in seconds')
    parser.add_argument('--hz', type=float, default=10.0, help='Send rate Hz (ADCM ~10Hz)')
    parser.add_argument('--host', default='127.0.0.1', help='Target host')
    parser.add_argument('--port', type=int, default=10002, help='Target port')
    parser.add_argument('--curve', type=float, default=0.0, help='Turn radius in meters (0=straight, positive=left, negative=right)')
    parser.add_argument('--stop-at-end', action='store_true', help='Send stop signal at end (drive_mode=False)')
    args = parser.parse_args()

    import cereal.messaging as messaging

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, args.port)
    sm = messaging.SubMaster(['carState', 'gpsLocationExternal', 'livePose'])

    print(f"Sending ADCM trajectory: speed={args.speed}m/s ({args.speed*3.6:.0f}km/h), "
          f"accel={args.accel}m/s^2, "
          f"{'curve r=' + str(args.curve) + 'm' if args.curve else 'straight'}")
    print(f"Target: {target}, Rate: {args.hz}Hz, Duration: {args.duration}s")
    print(f"Packet size: {PACKET_SIZE}B (no header, ADCM native format)")
    print(f"Ego pose: gpsLocationExternal (position) + livePose.orientationNED (heading)")
    print("---")

    ego_x = ego_y = ego_yaw = 0.0
    seq = 0
    sent_count = 0
    pose_warn_last = 0.0
    period = 1.0 / args.hz
    start = time.monotonic()

    try:
        while time.monotonic() - start < args.duration:
            loop_start = time.monotonic()

            sm.update(0)
            v_ego = max(sm['carState'].vEgo, 0.0)

            # Ego: GPS + livePose 직송 (실 ADCM 의 자체 localization 을 모방)
            if not (sm.alive['gpsLocationExternal'] and sm.alive['livePose']):
                if loop_start - pose_warn_last > 2.0:
                    print(f"[t={loop_start-start:6.2f}s] waiting for gpsLocationExternal/livePose alive…")
                    pose_warn_last = loop_start
                seq += 1
                sleep_time = period - (time.monotonic() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue

            gps = sm['gpsLocationExternal']
            lp = sm['livePose']
            ego_x = (gps.latitude - GPS_BASE_LAT) * GPS_DEG_TO_METERS
            ego_y = (gps.longitude - GPS_BASE_LON) * GPS_DEG_TO_METERS
            # NED → ENU 변환: 절대값 맞추기 (East 시작에서 yaw=0) + 회전 방향 맞추기.
            # 옛 viz 의 (π/2 + yaw_ned) + yaw_offset 잠금 (절대값 빼기) 와 등가.
            ego_yaw = float(lp.orientationNED.z) - math.pi / 2

            # ADCM-style spacing based on current speed
            spacing = compute_spacing(v_ego)

            # Generate trajectory (x, y, yaw only -- no velocity, like real ADCM)
            if args.curve != 0:
                points = make_curve_trajectory(args.curve, ego_x, ego_y, ego_yaw, spacing)
            else:
                points = make_straight_trajectory(ego_x, ego_y, ego_yaw, spacing)

            # target_accel: ramp up until speed reached, then 0
            if v_ego < args.speed:
                target_accel = args.accel
            else:
                target_accel = 0.0

            ego = {'x': ego_x, 'y': ego_y, 'yaw': ego_yaw}
            meta = {
                'target_accel': target_accel,
                'drive_mode': True,
                'emergency': 0.0,
                'turn_signal': 0,
            }

            pkt = build_packet(points, ego, meta)
            sock.sendto(pkt, target)

            elapsed_total = time.monotonic() - start
            if seq % int(args.hz * 2) == 0:  # print every 2 seconds
                print(f"[{elapsed_total:6.1f}s] seq={seq:5d} v_ego={v_ego*3.6:5.1f}km/h "
                      f"accel={target_accel:+.1f}m/s^2 spacing={spacing:.2f}m "
                      f"ego=({ego_x:.1f}, {ego_y:.1f}, {np.degrees(ego_yaw):.1f}deg)")

            seq += 1

            sleep_time = period - (time.monotonic() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

        if args.stop_at_end:
            print("Sending stop (drive_mode=False)...")
            spacing = compute_spacing(0.0)
            points = make_straight_trajectory(ego_x, ego_y, ego_yaw, spacing)
            ego = {'x': ego_x, 'y': ego_y, 'yaw': ego_yaw}
            meta = {
                'target_accel': 0.0,
                'drive_mode': False,
                'emergency': 0.0,
                'turn_signal': 0,
            }
            pkt = build_packet(points, ego, meta)
            sock.sendto(pkt, target)

    except KeyboardInterrupt:
        print("\nInterrupted")

    print(f"Done. Sent {seq} packets in {time.monotonic() - start:.1f}s")
    sock.close()


if __name__ == "__main__":
    main()
