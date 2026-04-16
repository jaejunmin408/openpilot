#!/usr/bin/env python3
"""
시각화 테스트: server.py에 가짜 차량 상태 + 궤적 JSON을 직접 전송.
openpilot / udp_bridge 없이 브라우저 시각화만 확인 가능.

사용법:
  터미널 1: cd /work/openpilot/alpamayo_trajectory && python3 server.py
  터미널 2: cd /work/openpilot/alpamayo_trajectory && python3 test_viz.py
  브라우저: http://localhost:8080
"""
import json
import math
import socket
import time

VEHICLE_PORT = 5006  # server.py 차량 상태
TRAJ_PORT = 5007     # server.py 궤적 JSON

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target_vehicle = ('127.0.0.1', VEHICLE_PORT)
target_traj = ('127.0.0.1', TRAJ_PORT)

# 원형 주행 시뮬레이션
speed = 3.0       # m/s
radius = 20.0     # m
dt = 0.05         # 20Hz
x, y, heading = 0.0, 0.0, 0.0
frame = 0

print(f"[test_viz] 차량 상태 → :{VEHICLE_PORT}, 궤적 → :{TRAJ_PORT}")
print(f"[test_viz] 브라우저에서 http://localhost:8080 을 열어 확인")

while True:
    # ── 차량 상태 전송 ──
    yaw_rate = speed / radius
    heading += yaw_rate * dt
    x += speed * math.cos(heading) * dt
    y += speed * math.sin(heading) * dt

    vehicle = {
        'type': 'vehicle',
        'x': round(x, 4),
        'y': round(y, 4),
        'heading': round(heading, 4),
        'speed': round(speed, 3),
        'accel': 0.0,
        'curvature': round(1.0 / radius, 5),
        'should_stop': False,
        'frame': frame,
    }
    sock.sendto(json.dumps(vehicle).encode(), target_vehicle)

    # ── 궤적 전송 (10Hz, 2프레임마다) ──
    if frame % 2 == 0:
        N = 30
        plan_dt = 0.2
        pred_xyz = []
        pred_yaw = []
        pred_v = []
        h = 0.0
        lx, ly = 0.0, 0.0
        for i in range(N):
            pred_xyz.append([round(lx, 4), round(ly, 4), 0.0])
            pred_yaw.append(round(h, 4))
            pred_v.append(round(speed, 3))
            # 다음 점: 원형 궤적
            h += yaw_rate * plan_dt
            lx += speed * math.cos(h) * plan_dt
            ly += speed * math.sin(h) * plan_dt

        traj = {
            'raw_action': {
                'accel_mps2': [0.0] * N,
                'curvature': [round(1.0 / radius, 5)] * N,
            },
            'pred_xyz': pred_xyz,
            'pred_yaw_rad': pred_yaw,
            'pred_v_mps': pred_v,
            'plan_dt_s': plan_dt,
        }
        sock.sendto(json.dumps(traj).encode(), target_traj)

    frame += 1
    time.sleep(dt)
