#!/usr/bin/env python3
"""
External PC → comma UDP trajectory sender

Interactive commands:
  straight              - go straight (curvature=0)
  right <curvature>     - right turn (e.g. "right 0.02")
  left <curvature>      - left turn  (e.g. "left 0.05")
  accel <m/s^2>         - set desired acceleration (e.g. "accel 1.0")
  stop                  - send shouldStop=True, accel=0
  go                    - resume (shouldStop=False)
  speed <m/s>           - set target speed for trajectory (e.g. "speed 15")
  status                - show current state
  quit / q              - exit

Packet format (little-endian):
  Header (16 bytes):  magic(uint32) + seq(uint32) + timestamp(float64)
  Action (9 bytes):   desiredCurvature(f32) + desiredAcceleration(f32) + shouldStop(uint8)
  Trajectory (396 bytes): 33 x (position_x, velocity_x, acceleration_x) as float32
  Total: 421 bytes per packet
"""

import struct
import socket
import time
import threading
import numpy as np

# -- Constants (from openpilot ModelConstants) --
IDX_N = 33
T_IDXS = [(10.0) * ((i / 32) ** 2) for i in range(IDX_N)]

# -- Config --
COMMA_IP = "192.168.217.253"
UDP_PORT = 5005
SEND_HZ = 20

MAGIC = 0x4F505431
HEADER_FMT = "<IId"
ACTION_FMT = "<ffB"
TRAJ_FMT = "<" + "fff" * IDX_N


class TrajectoryState:
    def __init__(self):
        self.curvature = 0.0
        self.acceleration = 0.0
        self.should_stop = False
        self.target_speed = 15.0
        self.lock = threading.Lock()

    def make_trajectory(self) -> dict:
        with self.lock:
            speed = self.target_speed
            curv = self.curvature
            accel = self.acceleration
            stop = self.should_stop

        positions = np.array([speed * t for t in T_IDXS], dtype=np.float32)
        velocities = np.full(IDX_N, speed, dtype=np.float32)
        accelerations = np.full(IDX_N, accel, dtype=np.float32)

        return {
            "position_x": positions,
            "velocity_x": velocities,
            "acceleration_x": accelerations,
            "desired_curvature": curv,
            "desired_acceleration": accel,
            "should_stop": stop,
        }


def pack_packet(seq: int, traj: dict) -> bytes:
    header = struct.pack(HEADER_FMT, MAGIC, seq, time.time())
    action = struct.pack(
        ACTION_FMT,
        traj["desired_curvature"],
        traj["desired_acceleration"],
        int(traj["should_stop"]),
    )
    traj_values = []
    for i in range(IDX_N):
        traj_values.extend([
            traj["position_x"][i],
            traj["velocity_x"][i],
            traj["acceleration_x"][i],
        ])
    trajectory = struct.pack(TRAJ_FMT, *traj_values)
    return header + action + trajectory


def sender_loop(sock, state: TrajectoryState):
    """20Hz UDP send loop (runs in background thread)."""
    seq = 0
    dt = 1.0 / SEND_HZ

    while True:
        t_start = time.monotonic()

        traj = state.make_trajectory()
        packet = pack_packet(seq, traj)
        sock.sendto(packet, (COMMA_IP, UDP_PORT))

        if False:  # silent send loop
            pass

        seq += 1
        elapsed = time.monotonic() - t_start
        if elapsed < dt:
            time.sleep(dt - elapsed)


def print_status(state: TrajectoryState):
    with state.lock:
        direction = "straight"
        if state.curvature > 0:
            direction = f"left {state.curvature:.4f}"
        elif state.curvature < 0:
            direction = f"right {abs(state.curvature):.4f}"
        print(f"\n  curvature:    {state.curvature:+.4f} ({direction})")
        print(f"  acceleration: {state.acceleration:+.2f} m/s^2")
        print(f"  target_speed: {state.target_speed:.1f} m/s ({state.target_speed * 3.6:.1f} km/h)")
        print(f"  should_stop:  {state.should_stop}\n")


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state = TrajectoryState()

    # start sender thread
    t = threading.Thread(target=sender_loop, args=(sock, state), daemon=True)
    t.start()

    print(f"Sending to {COMMA_IP}:{UDP_PORT} at {SEND_HZ}Hz")
    print("Commands: right/left <curv>, straight, accel <val>, speed <val>, stop, go, status, quit")
    print_status(state)

    try:
        while True:
            try:
                cmd = input("> ").strip()
            except EOFError:
                break

            if not cmd:
                continue

            parts = cmd.split()
            command = parts[0].lower()

            try:
                if command in ("quit", "q"):
                    break

                elif command == "right":
                    curv = float(parts[1]) if len(parts) > 1 else 0.02
                    with state.lock:
                        state.curvature = abs(curv)  # right = positive curvature
                    print(f"  -> right turn, curvature={abs(curv):+.4f}")

                elif command == "left":
                    curv = float(parts[1]) if len(parts) > 1 else 0.02
                    with state.lock:
                        state.curvature = -abs(curv)  # left = negative curvature
                    print(f"  -> left turn, curvature={-abs(curv):+.4f}")

                elif command == "straight":
                    with state.lock:
                        state.curvature = 0.0
                    print(f"  -> straight, curvature=0")

                elif command == "accel":
                    val = float(parts[1]) if len(parts) > 1 else 0.0
                    with state.lock:
                        state.acceleration = val
                    print(f"  -> acceleration={val:+.2f} m/s^2")

                elif command == "speed":
                    val = float(parts[1]) if len(parts) > 1 else 15.0
                    with state.lock:
                        state.target_speed = val
                    print(f"  -> target_speed={val:.1f} m/s ({val * 3.6:.1f} km/h)")

                elif command == "stop":
                    with state.lock:
                        state.should_stop = True
                        state.acceleration = 0.0
                    print(f"  -> STOP")

                elif command == "go":
                    with state.lock:
                        state.should_stop = False
                    print(f"  -> GO (shouldStop=False)")

                elif command == "status":
                    print_status(state)

                else:
                    print(f"  unknown command: {command}")
                    print("  commands: right/left <curv>, straight, accel <val>, speed <val>, stop, go, status, quit")

            except (ValueError, IndexError):
                print(f"  invalid input. example: 'right 0.02', 'accel 1.0', 'speed 15'")

    except KeyboardInterrupt:
        pass

    print("\nStopped.")
    sock.close()


if __name__ == "__main__":
    main()
