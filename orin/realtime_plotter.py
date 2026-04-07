#!/usr/bin/env python3
"""
ADCM 경로 추종 실시간 시각화

JSON 기준 경로 위에:
  - ADCM planned ego (빨간) — sender가 보내는 계획 위치
  - 실제 차량 위치 (초록) — comma에서 중계한 vEgo/yaw_rate로 dead reckoning

사용법:
  1. PC에서 plotter 실행
     python3 realtime_plotter.py --json test_trajectory_right_turn.json

  2. sender에 --viz 옵션 추가
     python3 adcm_trajectory_sender.py --ip COMMA_IP --viz PC_IP:5005 --json test_trajectory_right_turn.json

  3. comma에서 forwarder 실행
     python3 comma_state_forwarder.py --ip PC_IP --port 5005
"""

import argparse
import json
import math
import socket
import struct
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# ── ADCM 패킷 (sender) ──
ADCM_MAGIC = 0x41444301
ADCM_HEADER_FMT = "<IId"
ADCM_POINT_FMT = "<dddd"
ADCM_EGO_FMT = "<ddd"
ADCM_META_FMT = "<d?dB"
ADCM_FOOTER_FMT = "<B"
MAX_POINTS = 50

ADCM_HEADER_SIZE = struct.calcsize(ADCM_HEADER_FMT)
ADCM_POINT_SIZE = struct.calcsize(ADCM_POINT_FMT)
ADCM_EGO_SIZE = struct.calcsize(ADCM_EGO_FMT)
ADCM_META_SIZE = struct.calcsize(ADCM_META_FMT)
ADCM_FOOTER_SIZE = struct.calcsize(ADCM_FOOTER_FMT)
ADCM_PACKET_SIZE = (ADCM_HEADER_SIZE + ADCM_POINT_SIZE * MAX_POINTS
                    + ADCM_EGO_SIZE + ADCM_META_SIZE + ADCM_FOOTER_SIZE)

# ── comma 패킷 (forwarder) ──
COMMA_MAGIC = 0x434F4D41
COMMA_FMT = "<IId ddddd"  # magic, seq, ts, vEgo, aEgo, yaw_rate, steer_deg, curvature
COMMA_PACKET_SIZE = struct.calcsize(COMMA_FMT)


def parse_adcm(data):
    if len(data) < ADCM_PACKET_SIZE:
        return None
    offset = 0
    magic, seq, ts = struct.unpack(ADCM_HEADER_FMT, data[offset:offset + ADCM_HEADER_SIZE])
    if magic != ADCM_MAGIC:
        return None
    offset += ADCM_HEADER_SIZE

    points = []
    for _ in range(MAX_POINTS):
        x, y, yaw, vel = struct.unpack(ADCM_POINT_FMT, data[offset:offset + ADCM_POINT_SIZE])
        points.append((x, y, yaw, vel))
        offset += ADCM_POINT_SIZE

    ego_x, ego_y, ego_yaw = struct.unpack(ADCM_EGO_FMT, data[offset:offset + ADCM_EGO_SIZE])
    offset += ADCM_EGO_SIZE

    target_accel, drive_mode, emergency, turn_signal = struct.unpack(
        ADCM_META_FMT, data[offset:offset + ADCM_META_SIZE])
    offset += ADCM_META_SIZE

    n_valid = struct.unpack(ADCM_FOOTER_FMT, data[offset:offset + ADCM_FOOTER_SIZE])[0]

    return {
        "type": "adcm",
        "seq": seq,
        "ego": (ego_x, ego_y, ego_yaw),
        "points": points[:n_valid],
        "drive_mode": drive_mode,
        "turn_signal": turn_signal,
        "target_accel": target_accel,
    }


def parse_comma(data):
    if len(data) < COMMA_PACKET_SIZE:
        return None
    magic, seq, ts, v_ego, a_ego, yaw_rate, steer_deg, curvature = struct.unpack(
        COMMA_FMT, data[:COMMA_PACKET_SIZE])
    if magic != COMMA_MAGIC:
        return None
    return {
        "type": "comma",
        "seq": seq,
        "ts": ts,
        "v_ego": v_ego,
        "a_ego": a_ego,
        "yaw_rate": yaw_rate,
        "steer_deg": steer_deg,
        "curvature": curvature,
    }


def parse_any(data):
    """magic 값으로 ADCM / comma 패킷 구분"""
    if len(data) < 4:
        return None
    magic = struct.unpack("<I", data[:4])[0]
    if magic == ADCM_MAGIC:
        return parse_adcm(data)
    elif magic == COMMA_MAGIC:
        return parse_comma(data)
    return None


def main():
    parser = argparse.ArgumentParser(description="ADCM 실시간 경로 추종 시각화")
    parser.add_argument("--json", default="/home/a/orin/work/test_trajectory_right_turn.json",
                        help="기준 경로 JSON 파일")
    parser.add_argument("--port", type=int, default=5005, help="UDP 수신 포트")
    parser.add_argument("--trail", type=int, default=500, help="궤적 표시 개수")
    args = parser.parse_args()

    # --- 기준 경로 로드 ---
    with open(args.json) as f:
        data = json.load(f)
    frames = data["frames"]

    ref_ego_x = [f["ego_position"]["x"] for f in frames]
    ref_ego_y = [f["ego_position"]["y"] for f in frames]

    ref_full_x = ref_ego_x.copy()
    ref_full_y = ref_ego_y.copy()
    for p in frames[-1]["trajectory"]:
        ref_full_x.append(p["x"])
        ref_full_y.append(p["y"])

    # --- UDP 소켓 ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    sock.setblocking(False)
    print(f"[plotter] UDP 수신 대기: 0.0.0.0:{args.port}")
    print(f"[plotter] 기준 경로: {len(frames)} frames")
    print(f"[plotter] ADCM 패킷: {ADCM_PACKET_SIZE}B, comma 패킷: {COMMA_PACKET_SIZE}B")

    # --- ADCM 상태 (planned ego) ---
    adcm_trail_x = deque(maxlen=args.trail)
    adcm_trail_y = deque(maxlen=args.trail)
    adcm_state = {
        "ego": None, "points": None, "seq": 0,
        "drive_mode": False, "turn_signal": 0, "target_accel": 0.0,
    }

    # --- 실차 상태 (dead reckoning) ---
    actual_trail_x = deque(maxlen=args.trail)
    actual_trail_y = deque(maxlen=args.trail)
    actual_state = {
        "x": None, "y": None, "yaw": None,  # UTM 좌표 (첫 ADCM ego에서 초기화)
        "v_ego": 0.0, "a_ego": 0.0, "yaw_rate": 0.0,
        "steer_deg": 0.0, "curvature": 0.0,
        "initialized": False,
        "last_ts": None,
    }

    # --- 플롯 설정 ---
    fig, (ax, ax_info) = plt.subplots(
        1, 2, figsize=(16, 9), gridspec_kw={"width_ratios": [3, 1]})
    fig.suptitle("ADCM 실시간 경로 추종", fontsize=14, fontweight='bold')

    # 기준 경로 (고정)
    ax.plot(ref_full_x, ref_full_y, 'k--', linewidth=1.5, alpha=0.3, label='reference')
    ax.plot(ref_full_x[0], ref_full_y[0], 'g^', markersize=14, zorder=4, label='start')
    ax.plot(ref_full_x[-1], ref_full_y[-1], 'rs', markersize=12, zorder=4, label='end')

    # ADCM planned (빨간 계열)
    adcm_trail_line, = ax.plot([], [], 'r-', linewidth=2, alpha=0.5, label='planned ego')
    adcm_dot, = ax.plot([], [], 'ro', markersize=10, zorder=6)
    adcm_arrow = ax.quiver(0, 0, 1, 0, scale=1, scale_units='xy', angles='xy',
                           color='red', width=0.005, zorder=7)
    traj_line, = ax.plot([], [], 'c-', linewidth=1.5, alpha=0.5, label='planned traj')
    traj_dots, = ax.plot([], [], 'c.', markersize=2, alpha=0.4)

    # 실차 actual (초록 계열)
    actual_trail_line, = ax.plot([], [], 'g-', linewidth=2.5, alpha=0.8, label='actual vehicle')
    actual_dot, = ax.plot([], [], 'go', markersize=10, zorder=6)
    actual_arrow = ax.quiver(0, 0, 1, 0, scale=1, scale_units='xy', angles='xy',
                             color='green', width=0.005, zorder=7)

    margin = 5
    ax.set_xlim(min(ref_full_x) - margin, max(ref_full_x) + margin)
    ax.set_ylim(min(ref_full_y) - margin, max(ref_full_y) + margin)
    ax.set_xlabel("X (m) — East")
    ax.set_ylabel("Y (m) — North")
    ax.set_aspect('equal')
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(True, alpha=0.3)

    # 정보 패널
    ax_info.axis('off')
    info_text = ax_info.text(
        0.05, 0.95, "waiting for data...",
        transform=ax_info.transAxes, fontsize=11, fontfamily='monospace',
        verticalalignment='top',
        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))

    # --- 애니메이션 ---
    def update(_frame_num):
        latest_adcm = None
        latest_comma = None

        # 소켓에서 모든 패킷 수신, 종류별 최신만 유지
        while True:
            try:
                pkt, _addr = sock.recvfrom(max(ADCM_PACKET_SIZE, COMMA_PACKET_SIZE) + 64)
                parsed = parse_any(pkt)
                if parsed is None:
                    continue
                if parsed["type"] == "adcm":
                    latest_adcm = parsed
                elif parsed["type"] == "comma":
                    latest_comma = parsed
            except BlockingIOError:
                break

        # ── ADCM 업데이트 ──
        if latest_adcm:
            adcm_state.update({
                "ego": latest_adcm["ego"],
                "points": latest_adcm["points"],
                "seq": latest_adcm["seq"],
                "drive_mode": latest_adcm["drive_mode"],
                "turn_signal": latest_adcm["turn_signal"],
                "target_accel": latest_adcm["target_accel"],
            })
            ex, ey, _ = latest_adcm["ego"]
            adcm_trail_x.append(ex)
            adcm_trail_y.append(ey)

            # 실차 위치 초기화 (첫 ADCM 패킷의 ego 위치에서 시작)
            if not actual_state["initialized"]:
                actual_state["x"] = ex
                actual_state["y"] = ey
                actual_state["yaw"] = latest_adcm["ego"][2]
                actual_state["initialized"] = True
                print(f"[plotter] 실차 위치 초기화: ({ex:.2f}, {ey:.2f})")

        # ── comma 업데이트 (dead reckoning) ──
        if latest_comma and actual_state["initialized"]:
            actual_state["v_ego"] = latest_comma["v_ego"]
            actual_state["a_ego"] = latest_comma["a_ego"]
            actual_state["yaw_rate"] = latest_comma["yaw_rate"]
            actual_state["steer_deg"] = latest_comma["steer_deg"]
            actual_state["curvature"] = latest_comma["curvature"]

            ts = latest_comma["ts"]
            if actual_state["last_ts"] is not None:
                dt = ts - actual_state["last_ts"]
                dt = max(0.001, min(dt, 0.2))  # clamp

                v = latest_comma["v_ego"]
                yr = latest_comma["yaw_rate"]

                actual_state["yaw"] += yr * dt
                actual_state["x"] += v * math.cos(actual_state["yaw"]) * dt
                actual_state["y"] += v * math.sin(actual_state["yaw"]) * dt

            actual_state["last_ts"] = ts

            actual_trail_x.append(actual_state["x"])
            actual_trail_y.append(actual_state["y"])

        # ── 그리기: ADCM planned ──
        if adcm_trail_x:
            adcm_trail_line.set_data(list(adcm_trail_x), list(adcm_trail_y))

        if adcm_state["ego"]:
            ex, ey, eyaw = adcm_state["ego"]
            adcm_dot.set_data([ex], [ey])
            adcm_arrow.set_offsets([[ex, ey]])
            adcm_arrow.set_UVC(3.0 * math.cos(eyaw), 3.0 * math.sin(eyaw))

        if adcm_state["points"]:
            tx = [p[0] for p in adcm_state["points"]]
            ty = [p[1] for p in adcm_state["points"]]
            traj_line.set_data(tx, ty)
            traj_dots.set_data(tx, ty)

        # ── 그리기: actual vehicle ──
        if actual_trail_x:
            actual_trail_line.set_data(list(actual_trail_x), list(actual_trail_y))

        if actual_state["initialized"] and actual_state["last_ts"] is not None:
            ax_val = actual_state["x"]
            ay_val = actual_state["y"]
            ayaw = actual_state["yaw"]
            actual_dot.set_data([ax_val], [ay_val])
            actual_arrow.set_offsets([[ax_val, ay_val]])
            actual_arrow.set_UVC(3.0 * math.cos(ayaw), 3.0 * math.sin(ayaw))

        # ── 정보 텍스트 ──
        lines = []
        if adcm_state["ego"]:
            ex, ey, eyaw = adcm_state["ego"]
            ts_map = {0: "OFF", 1: "LEFT", 2: "RIGHT"}
            vel = adcm_state["points"][0][3] if adcm_state["points"] else 0
            lines.append(
                f"── ADCM planned ──\n"
                f"seq:     {adcm_state['seq']}\n"
                f"x:       {ex:.2f} m\n"
                f"y:       {ey:.2f} m\n"
                f"heading: {math.degrees(eyaw):.1f}°\n"
                f"vel:     {vel:.2f} m/s ({vel * 3.6:.1f} km/h)\n"
                f"drive:   {'AUTO' if adcm_state['drive_mode'] else 'MANUAL'}\n"
                f"signal:  {ts_map.get(adcm_state['turn_signal'], '?')}"
            )

        if actual_state["initialized"]:
            v = actual_state["v_ego"]
            lines.append(
                f"\n── actual vehicle ──\n"
                f"x:       {actual_state['x']:.2f} m\n"
                f"y:       {actual_state['y']:.2f} m\n"
                f"heading: {math.degrees(actual_state['yaw']):.1f}°\n"
                f"v_ego:   {v:.2f} m/s ({v * 3.6:.1f} km/h)\n"
                f"a_ego:   {actual_state['a_ego']:.2f} m/s²\n"
                f"steer:   {actual_state['steer_deg']:.1f}°\n"
                f"yaw_r:   {actual_state['yaw_rate']:.3f} rad/s"
            )

        # 오차 표시
        if actual_state["initialized"] and adcm_state["ego"] and actual_state["last_ts"]:
            ex, ey, _ = adcm_state["ego"]
            dx = actual_state["x"] - ex
            dy = actual_state["y"] - ey
            dist_err = math.sqrt(dx * dx + dy * dy)
            lines.append(
                f"\n── error ──\n"
                f"dist:    {dist_err:.3f} m\n"
                f"dx:      {dx:.3f} m\n"
                f"dy:      {dy:.3f} m"
            )

        info_text.set_text("\n".join(lines) if lines else "waiting for data...")

        return (adcm_trail_line, adcm_dot, traj_line, traj_dots,
                actual_trail_line, actual_dot, info_text)

    _ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
