#!/usr/bin/env python3
"""
Position Viewer: 계획 경로 vs 실제 차량 위치를 실시간 X-Y 플롯으로 표시.

외부 PC에서 comma의 bridge를 통해 ZMQ로 직접 메시지를 수신합니다.
cereal 빌드 불필요 — pyzmq + pycapnp만 있으면 동작합니다.

사용법:
  pip3 install pyzmq pycapnp matplotlib numpy
  python3 position_viewer.py trajectory_right_turn_v1.json --ip 10.200.147.253
"""

import argparse
import json
import math
import os
import struct
import time

import capnp
import zmq
import matplotlib.pyplot as plt
import numpy as np


# --- capnp 스키마 로드 ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OPENPILOT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

capnp.remove_import_hook()
log_capnp = capnp.load(
    os.path.join(OPENPILOT_ROOT, "cereal", "log.capnp"),
    imports=[
        OPENPILOT_ROOT,
        os.path.join(OPENPILOT_ROOT, "opendbc_repo"),
    ]
)


# --- bridge 포트 계산 (FNV-1a, bridge_zmq.cc와 동일) ---
def get_port(service_name: str) -> int:
    fnv_prime = 0x100000001b3
    hash_value = 0xcbf29ce484222325
    for c in service_name:
        hash_value ^= ord(c)
        hash_value = (hash_value * fnv_prime) & 0xFFFFFFFFFFFFFFFF
    return 8023 + (hash_value % (65535 - 8023))


def load_planned_trajectory(json_path: str):
    if not json_path or not os.path.exists(json_path):
        return [], []
    with open(json_path, "r") as f:
        data = json.load(f)
    xs = [w["x_m"] for w in data["waypoints"]]
    ys = [w["y_m"] for w in data["waypoints"]]
    return xs, ys


def main():
    parser = argparse.ArgumentParser(description="Real-time X-Y trajectory viewer")
    parser.add_argument("json_path", nargs="?", default="", help="Planned trajectory JSON")
    parser.add_argument("--ip", default="10.200.147.253", help="Comma device IP")
    args = parser.parse_args()

    # 계획 경로 로드
    plan_x, plan_y = load_planned_trajectory(args.json_path)

    # ZMQ 구독 설정
    ctx = zmq.Context()

    car_port = get_port("carState")
    ctrl_port = get_port("controlsState")

    car_sock = ctx.socket(zmq.SUB)
    car_sock.connect(f"tcp://{args.ip}:{car_port}")
    car_sock.setsockopt(zmq.SUBSCRIBE, b"")
    car_sock.setsockopt(zmq.RCVTIMEO, 100)
    car_sock.setsockopt(zmq.CONFLATE, 1)  # 최신 메시지만

    ctrl_sock = ctx.socket(zmq.SUB)
    ctrl_sock.connect(f"tcp://{args.ip}:{ctrl_port}")
    ctrl_sock.setsockopt(zmq.SUBSCRIBE, b"")
    ctrl_sock.setsockopt(zmq.RCVTIMEO, 100)
    ctrl_sock.setsockopt(zmq.CONFLATE, 1)

    print(f"Connecting to {args.ip}")
    print(f"  carState port: {car_port}")
    print(f"  controlsState port: {ctrl_port}")

    # ZMQ 연결 워밍업 — bridge가 SUB를 인식할 때까지 대기
    print("Warming up ZMQ connection...", end="", flush=True)
    for _ in range(50):  # 최대 5초
        try:
            raw = car_sock.recv(zmq.NOBLOCK)
            print(" connected!")
            break
        except zmq.Again:
            time.sleep(0.1)
    else:
        print(" timeout, starting anyway.")

    # 계획 경로 범위로 고정 좌표계 계산
    if plan_x and plan_y:
        margin = 5.0
        x_min, x_max = min(plan_x) - margin, max(plan_x) + margin
        y_min, y_max = min(plan_y) - margin, max(plan_y) + margin
    else:
        x_min, x_max = -5, 30
        y_min, y_max = -5, 30

    # matplotlib 실시간 설정
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))

    if plan_x:
        ax.plot(plan_x, plan_y, "r--", linewidth=2, label="Planned")
    actual_line, = ax.plot([], [], "b-", linewidth=2, label="Actual")
    actual_dot, = ax.plot([], [], "bo", markersize=8)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("X-Y Trajectory")
    ax.legend()
    ax.set_aspect("equal")
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.grid(True)

    plt.tight_layout()

    # Dead reckoning 상태
    x, y, yaw = 0.0, 0.0, 0.0
    xs, ys = [], []
    actual_curv = 0.0
    last_time = time.monotonic()

    print("Tracking started! (recording immediately)")

    try:
        while plt.fignum_exists(fig.number):
            # controlsState 수신 (non-blocking)
            try:
                raw = ctrl_sock.recv(zmq.NOBLOCK)
                with log_capnp.Event.from_bytes(raw) as evt:
                    cs = evt.controlsState
                    actual_curv = cs.curvature
            except zmq.Again:
                pass

            # carState 수신
            try:
                raw = car_sock.recv()
                with log_capnp.Event.from_bytes(raw) as evt:
                    car = evt.carState
                    v_ego = car.vEgo
            except zmq.Again:
                plt.pause(0.01)
                continue

            # yawRate = curvature * vEgo (controlsState 기반)
            yaw_rate = actual_curv * v_ego

            now = time.monotonic()

            dt = now - last_time
            last_time = now

            # Dead reckoning
            dyaw = yaw_rate * dt
            mid_yaw = yaw + dyaw / 2.0
            ds = v_ego * dt
            x += ds * math.cos(mid_yaw)
            y += ds * math.sin(mid_yaw)
            yaw += dyaw

            xs.append(x)
            ys.append(y)

            # 10Hz로 화면 갱신
            if len(xs) % 5 == 0:
                actual_line.set_data(xs, ys)
                actual_dot.set_data([x], [y])

                fig.canvas.draw_idle()
                fig.canvas.flush_events()

    except KeyboardInterrupt:
        pass

    print(f"\nTotal: {len(xs)} samples")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
