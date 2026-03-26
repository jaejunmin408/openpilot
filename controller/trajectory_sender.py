#!/usr/bin/env python3
"""
External PC → comma UDP control sender

Interactive commands:
  straight              - go straight (curvature=0)
  right <curvature>     - right turn (e.g. "right 0.02")
  left <curvature>      - left turn  (e.g. "left 0.05")
  accel <m/s^2>         - set desired acceleration (e.g. "accel 1.0")
  stop                  - send shouldStop=True, accel=0
  go                    - resume (shouldStop=False)
  status                - show current state
  quit / q              - exit

Packet format (little-endian):
  Header (16 bytes):  magic(uint32) + seq(uint32) + timestamp(float64)
  Payload (9 bytes):  desiredCurvature(f32) + acceleration(f32) + shouldStop(uint8)
  Total: 25 bytes per packet
"""

import struct
import socket
import time
import threading

# -- Config --
COMMA_IP = "10.200.147.253"
UDP_PORT = 5005
SEND_HZ = 20

MAGIC = 0x4F505431
HEADER_FMT = "<IId"
PAYLOAD_FMT = "<ffB"


class TrajectoryState:
    def __init__(self):
        self.curvature = 0.0
        self.acceleration = 0.0
        self.should_stop = False
        self.lock = threading.Lock()


def pack_packet(seq: int, state: TrajectoryState) -> bytes:
    with state.lock:
        curvature = state.curvature
        acceleration = state.acceleration
        should_stop = state.should_stop
    header = struct.pack(HEADER_FMT, MAGIC, seq, time.time())
    payload = struct.pack(PAYLOAD_FMT, curvature, acceleration, int(should_stop))
    return header + payload


def sender_loop(sock, state: TrajectoryState):
    """20Hz UDP send loop (runs in background thread)."""
    seq = 0
    dt = 1.0 / SEND_HZ
    log_interval = 5.0  # seconds
    last_log = time.monotonic()
    send_errors = 0
    max_jitter_ms = 0.0

    while True:
        t_start = time.monotonic()
        packet = pack_packet(seq, state)
        try:
            sock.sendto(packet, (COMMA_IP, UDP_PORT))
        except OSError as e:
            send_errors += 1
            if send_errors <= 3:
                print(f"  [UDP ERROR] sendto failed: {e}")
        seq += 1
        elapsed = time.monotonic() - t_start
        jitter_ms = abs(elapsed * 1000 - dt * 1000)
        if jitter_ms > max_jitter_ms:
            max_jitter_ms = jitter_ms

        now = time.monotonic()
        if now - last_log >= log_interval:
            with state.lock:
                curv = state.curvature
                accel = state.acceleration
                stop = state.should_stop
            print(f"  [UDP] seq={seq} curv={curv:+.4f} accel={accel:+.2f} stop={stop} "
                  f"max_jitter={max_jitter_ms:.1f}ms errors={send_errors}")
            max_jitter_ms = 0.0
            last_log = now

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
        print(f"  should_stop:  {state.should_stop}\n")


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state = TrajectoryState()

    # start sender thread
    t = threading.Thread(target=sender_loop, args=(sock, state), daemon=True)
    t.start()

    print(f"Sending to {COMMA_IP}:{UDP_PORT} at {SEND_HZ}Hz (25-byte packets)")
    print("Commands: right/left <curv>, straight, accel <val>, stop, go, status, quit")
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
                        state.curvature = abs(curv)
                    print(f"  -> right turn, curvature={abs(curv):+.4f}")

                elif command == "left":
                    curv = float(parts[1]) if len(parts) > 1 else 0.02
                    with state.lock:
                        state.curvature = -abs(curv)
                    print(f"  -> left turn, curvature={-abs(curv):+.4f}")

                elif command == "straight":
                    with state.lock:
                        state.curvature = 0.0
                    print("  -> straight, curvature=0")

                elif command == "accel":
                    val = float(parts[1]) if len(parts) > 1 else 0.0
                    with state.lock:
                        state.acceleration = val
                    print(f"  -> acceleration={val:+.2f} m/s^2")

                elif command == "stop":
                    with state.lock:
                        state.should_stop = True
                        state.acceleration = 0.0
                    print("  -> STOP")

                elif command == "go":
                    with state.lock:
                        state.should_stop = False
                    print("  -> GO (shouldStop=False)")

                elif command == "status":
                    print_status(state)

                else:
                    print(f"  unknown command: {command}")
                    print("  commands: right/left <curv>, straight, accel <val>, stop, go, status, quit")

            except (ValueError, IndexError):
                print("  invalid input. example: 'right 0.02', 'accel 1.0'")

    except KeyboardInterrupt:
        pass

    print("\nStopped.")
    sock.close()


if __name__ == "__main__":
    main()
