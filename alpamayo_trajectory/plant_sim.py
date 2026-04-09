#!/usr/bin/env python3
"""
Plant Simulator — openpilot의 modelV2.action 출력으로 차량 운동학 적분

udp_bridge가 발행한 modelV2.action.desiredAcceleration / desiredCurvature를 읽어서
bicycle model로 (x, y, heading, speed) 적분.
결과를 UDP로 viz server에 전송.

실행:
  python3 plant_sim.py [--init-x 0] [--init-y 0] [--init-yaw 0]
"""
import argparse
import json
import math
import socket
import time

import cereal.messaging as messaging

DT = 0.05  # 20Hz (modelV2 발행 주기)
VIZ_UDP_PORT = 5006  # viz server로 보내는 포트


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--init-x', type=float, default=0.0)
    parser.add_argument('--init-y', type=float, default=0.0)
    parser.add_argument('--init-yaw', type=float, default=0.0)
    parser.add_argument('--viz-host', default='127.0.0.1')
    parser.add_argument('--viz-port', type=int, default=VIZ_UDP_PORT)
    args = parser.parse_args()

    sm = messaging.SubMaster(['modelV2'], poll='modelV2')

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.viz_host, args.viz_port)

    # 차량 상태 (global ENU)
    x = args.init_x
    y = args.init_y
    heading = args.init_yaw  # rad, CCW from east
    speed = 0.0

    frame = 0
    print(f"[plant_sim] waiting for modelV2 from udp_bridge...")
    print(f"[plant_sim] init pos=({x:.1f}, {y:.1f}) yaw={math.degrees(heading):.1f}°")
    print(f"[plant_sim] sending to {target}")

    while True:
        sm.update(100)  # 100ms timeout

        if not sm.updated['modelV2']:
            continue

        mv2 = sm['modelV2']
        desired_accel = mv2.action.desiredAcceleration
        desired_curvature = mv2.action.desiredCurvature
        should_stop = mv2.action.shouldStop

        # 정지 명령
        if should_stop and speed < 0.1:
            desired_accel = min(desired_accel, 0.0)

        # 속도 적분
        speed += desired_accel * DT
        speed = max(speed, 0.0)  # 후진 방지

        # heading 적분 (curvature = 1/R, yaw_rate = v * curvature)
        yaw_rate = speed * desired_curvature
        heading += yaw_rate * DT

        # 위치 적분
        x += speed * math.cos(heading) * DT
        y += speed * math.sin(heading) * DT

        # viz server로 전송
        state = {
            'type': 'vehicle',
            'x': round(x, 4),
            'y': round(y, 4),
            'heading': round(heading, 4),
            'speed': round(speed, 3),
            'accel': round(desired_accel, 3),
            'curvature': round(desired_curvature, 5),
            'should_stop': should_stop,
            'frame': frame,
        }
        sock.sendto(json.dumps(state).encode(), target)

        if frame % 20 == 0:  # 1Hz 로그
            print(f"[plant] f={frame:5d} pos=({x:.2f},{y:.2f}) "
                  f"hdg={math.degrees(heading):.1f}° v={speed:.2f}m/s "
                  f"a={desired_accel:.2f} c={desired_curvature:.4f} "
                  f"stop={should_stop}")

        frame += 1


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n[plant_sim] stopped")
