#!/usr/bin/env python3
"""livePose → LocalWorld → 시각화 서버로 실시간 stream.

LocalWorld 모듈 검증용.
브라우저(http://localhost:8080) 에서 LocalWorld 좌표계 위 차량 위치 + 6초 buffer 경로 시각화.

전제:
  - alpamayo_trajectory/server.py 가 띄워져 있어야 함
  - cereal 메시징 시스템에서 livePose 가 publish 되어야 함
    (시뮬 또는 실차에서 locationd 동작 중이어야 함)

사용:
  python3 alpamayo_trajectory/livepose_to_viz.py
"""
import json
import math
import socket
import sys

import cereal.messaging as messaging

from openpilot.selfdrive.controls.lib.local_world import LocalWorld

VIZ_HOST = "127.0.0.1"
VIZ_PORT = 5006  # PlantUDPProtocol 이 vehicle/trajectory JSON 받는 포트


def main() -> None:
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  world = LocalWorld()
  sm = messaging.SubMaster(["livePose"], poll="livePose")

  print(f"[livepose_to_viz] subscribed to livePose, sending to {VIZ_HOST}:{VIZ_PORT}")
  print("[livepose_to_viz] open http://localhost:8080 in browser")

  frame = 0
  bootstrap_logged = False

  while True:
    sm.update()
    if not sm.updated["livePose"]:
      continue

    lp = sm["livePose"]
    t_ns = sm.logMonoTime["livePose"]
    world.update(lp, t_ns)

    cur = world.current()
    if cur is None:
      continue

    if not bootstrap_logged and world.is_initialized():
      print(f"[livepose_to_viz] LocalWorld initialized at yaw={cur[3]:.3f} rad")
      bootstrap_logged = True

    _, x, y, yaw = cur
    speed = math.hypot(lp.velocityDevice.x, lp.velocityDevice.y)

    # vehicle 메시지: 현재 LocalWorld pose
    vehicle_msg = {
      "type": "vehicle",
      "x": float(x),
      "y": float(y),
      "heading": float(yaw),
      "speed": float(speed),
      "accel": float(lp.accelerationDevice.x),
      "curvature": 0.0,
      "should_stop": False,
      "frame": frame,
    }
    sock.sendto(json.dumps(vehicle_msg).encode(), (VIZ_HOST, VIZ_PORT))

    # trajectory 메시지: buffer 전체 (6초 = 최대 120 점)
    hist = world.history()
    if len(hist) >= 2:
      pts = [
        {
          "x": float(hx),
          "y": float(hy),
          "yaw": float(hyaw),
          "vel": float(speed),  # 색 표시용. 일정 색.
          "curvature": 0.0,
        }
        for (_, hx, hy, hyaw) in hist
      ]
      traj_msg = {
        "type": "trajectory",
        "seq": frame,
        "plan_seq": frame,
        "coord_mode": 1,  # global
        "num_points": len(pts),
        "dt_s": 0.05,
        "points": pts,
        "packet_count": frame,
      }
      sock.sendto(json.dumps(traj_msg).encode(), (VIZ_HOST, VIZ_PORT))

    frame += 1
    if frame % 100 == 0:
      print(f"[livepose_to_viz] frame={frame} pos=({x:.2f}, {y:.2f}) "
            f"yaw={math.degrees(yaw):.1f}° hist_len={len(hist)}")


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    print("\n[livepose_to_viz] stopped")
    sys.exit(0)
