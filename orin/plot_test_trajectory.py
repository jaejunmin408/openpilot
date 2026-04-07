#!/usr/bin/env python3
"""테스트 궤적 시각화"""
import json
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

with open("/home/a/orin/work/test_trajectory_left_turn.json") as f:
    data = json.load(f)

frames = data["frames"]

fig, axes = plt.subplots(1, 3, figsize=(18, 6))

# --- Plot 1: 전체 기준 경로 + ego 궤적 ---
ax = axes[0]
ax.set_title("Full Path + Ego Trajectory", fontsize=12)

# ego 궤적
ego_x = [f["ego_position"]["x"] for f in frames]
ego_y = [f["ego_position"]["y"] for f in frames]
ax.plot(ego_x, ego_y, 'b-', linewidth=2, label='ego path')

# 시작/끝 마커
ax.plot(ego_x[0], ego_y[0], 'go', markersize=10, label='start')
ax.plot(ego_x[-1], ego_y[-1], 'rs', markersize=10, label='end')

# 2초 시점 표시 (frame index 40 = 2.0s)
if len(frames) > 40:
    ax.plot(ego_x[40], ego_y[40], 'mo', markersize=10, label='t=2.0s (turn start)')

ax.set_xlabel("X (m) - East")
ax.set_ylabel("Y (m) - North")
ax.set_aspect('equal')
ax.legend()
ax.grid(True, alpha=0.3)

# --- Plot 2: 주요 시점별 궤적 스냅샷 ---
ax = axes[1]
ax.set_title("Trajectory Snapshots at Key Times", fontsize=12)

colors = ['green', 'blue', 'orange', 'red', 'purple']
times = [0.0, 1.0, 2.0, 3.0, 4.0]

for t, color in zip(times, colors):
    idx = int(t * 20)
    if idx >= len(frames):
        break
    f = frames[idx]
    ego = f["ego_position"]
    traj_x = [p["x"] for p in f["trajectory"]]
    traj_y = [p["y"] for p in f["trajectory"]]

    ax.plot(traj_x, traj_y, '-', color=color, linewidth=1.5, alpha=0.7, label=f't={t:.0f}s')
    ax.plot(ego["x"], ego["y"], 'o', color=color, markersize=8)

    # heading 화살표
    dx = 3 * math.cos(ego["yaw"])
    dy = 3 * math.sin(ego["yaw"])
    ax.annotate('', xy=(ego["x"]+dx, ego["y"]+dy), xytext=(ego["x"], ego["y"]),
                arrowprops=dict(arrowstyle='->', color=color, lw=2))

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_aspect('equal')
ax.legend()
ax.grid(True, alpha=0.3)

# --- Plot 3: yaw 변화 ---
ax = axes[2]
ax.set_title("Ego Heading (yaw) over Time", fontsize=12)

times_all = [f["time"] for f in frames]
yaws_deg = [math.degrees(f["ego_position"]["yaw"]) for f in frames]
turn_signals = [f["turn_signal"] for f in frames]

ax.plot(times_all, yaws_deg, 'b-', linewidth=2, label='yaw (deg)')
ax.axvline(x=2.0, color='r', linestyle='--', alpha=0.5, label='turn start (2s)')
ax.axhline(y=90, color='gray', linestyle=':', alpha=0.5, label='90° (north)')

# turn signal 배경
for i in range(len(frames) - 1):
    if turn_signals[i] == 1:
        ax.axvspan(times_all[i], times_all[i+1], alpha=0.1, color='orange')

ax.set_xlabel("Time (s)")
ax.set_ylabel("Heading (degrees)")
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("/home/a/orin/work/test_trajectory_plot.png", dpi=150)
print("Saved: /home/a/orin/work/test_trajectory_plot.png")
