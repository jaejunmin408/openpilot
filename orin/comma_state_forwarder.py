#!/usr/bin/env python3
"""
comma 차량 상태 → PC UDP 중계

comma에서 실행하여 carState/livePose를 PC로 전송
realtime_plotter.py가 이 데이터로 실제 차량 위치를 표시

사용법 (comma에서):
  python3 comma_state_forwarder.py --ip PC_IP --port 5005
"""

import argparse
import socket
import struct
import time

import cereal.messaging as messaging

MAGIC = 0x434F4D41  # 'COMA'
PACKET_FMT = "<IId ddddd"
# magic, seq, timestamp, vEgo, aEgo, yaw_rate, steer_angle_deg, curvature

SEND_HZ = 20


def main():
    parser = argparse.ArgumentParser(description="comma state forwarder")
    parser.add_argument("--ip", required=True, help="PC IP (plotter)")
    parser.add_argument("--port", type=int, default=5005, help="PC UDP port")
    args = parser.parse_args()

    sm = messaging.SubMaster(['carState', 'controlsState', 'livePose'])
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    seq = 0
    dt = 1.0 / SEND_HZ

    print(f"[forwarder] Sending to {args.ip}:{args.port} at {SEND_HZ}Hz")
    print("[forwarder] Ctrl+C to stop\n")

    try:
        while True:
            t_start = time.monotonic()

            sm.update(100)
            if not sm.updated['carState']:
                continue

            car = sm['carState']
            cs = sm['controlsState']
            lp = sm['livePose']

            yaw_rate = lp.angularVelocityDevice.z.value if sm.valid['livePose'] else 0.0

            pkt = struct.pack(
                PACKET_FMT,
                MAGIC, seq, time.time(),
                car.vEgo,
                car.aEgo,
                yaw_rate,
                car.steeringAngleDeg,
                cs.curvature,
            )
            sock.sendto(pkt, (args.ip, args.port))

            if seq % (SEND_HZ // 4) == 0:  # 4Hz로 로그 출력
                print(f"[{seq:5d}] des_curv={cs.desiredCurvature:+.5f}  "
                      f"act_curv={cs.curvature:+.5f}  "
                      f"err={cs.desiredCurvature - cs.curvature:+.5f}  "
                      f"v={car.vEgo * 3.6:.1f}km/h  "
                      f"steer={car.steeringAngleDeg:.1f}°")

            seq += 1
            elapsed = time.monotonic() - t_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print(f"\n[forwarder] Stopped. Total: {seq} packets.")


if __name__ == "__main__":
    main()
