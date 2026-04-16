#!/usr/bin/env python3
"""LocalWorld 궤적 시각화. 녹화된 route 의 livePose 를 적분해서 (x, y) 플롯.

사용:
  python alpamayo_trajectory/check_local_world.py <route_or_log_path>

기대:
  직선 구간은 직선, 코너는 코너, 원형 주행은 원으로 보여야 함.
"""
import math
import sys

import matplotlib.pyplot as plt

from openpilot.selfdrive.controls.lib.local_world import LocalWorld
from openpilot.tools.lib.logreader import LogReader


def main(route: str) -> None:
  lr = LogReader(route)
  w = LocalWorld(history_len=10**6)  # 시각화용 - 전체 보존

  xs, ys, yaws, ts = [], [], [], []
  n_msgs = 0
  for msg in lr:
    if msg.which() != "livePose":
      continue
    w.update(msg.livePose, msg.logMonoTime)
    cur = w.current()
    if cur is not None:
      t, x, y, yaw = cur
      ts.append(t)
      xs.append(x)
      ys.append(y)
      yaws.append(yaw)
    n_msgs += 1

  if not xs:
    print("No livePose messages produced a valid pose. Check log.")
    return

  duration_sec = (ts[-1] - ts[0]) / 1e9
  total_dist = sum(
    ((xs[i] - xs[i - 1]) ** 2 + (ys[i] - ys[i - 1]) ** 2) ** 0.5
    for i in range(1, len(xs))
  )
  print(f"livePose 메시지: {n_msgs}, 적분 sample: {len(xs)}")
  print(f"총 시간: {duration_sec:.1f}s, 총 이동거리: {total_dist:.1f}m")
  print(f"최종 위치: ({xs[-1]:.1f}, {ys[-1]:.1f})")

  fig, axes = plt.subplots(1, 2, figsize=(14, 6))

  axes[0].plot(xs, ys, linewidth=1)
  axes[0].plot(xs[0], ys[0], "go", markersize=8, label="start")
  axes[0].plot(xs[-1], ys[-1], "rx", markersize=10, label="end")
  axes[0].set_xlabel("x [m]")
  axes[0].set_ylabel("y [m]")
  axes[0].set_title("LocalWorld trajectory (top-down)")
  axes[0].axis("equal")
  axes[0].grid(True, alpha=0.3)
  axes[0].legend()

  t_rel = [(t - ts[0]) / 1e9 for t in ts]
  axes[1].plot(t_rel, [math.degrees(y) for y in yaws], linewidth=1)
  axes[1].set_xlabel("time [s]")
  axes[1].set_ylabel("yaw [deg]")
  axes[1].set_title("yaw over time (orientationNED.z)")
  axes[1].grid(True, alpha=0.3)

  plt.tight_layout()
  plt.show()


if __name__ == "__main__":
  if len(sys.argv) != 2:
    print("Usage: python check_local_world.py <route_or_log_path>")
    sys.exit(1)
  main(sys.argv[1])
