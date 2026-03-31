#!/usr/bin/env python3
"""
External PC → comma UDP trajectory replay sender

Reads a trajectory JSON file (0.1s waypoints), interpolates to 0.05s,
computes curvature & acceleration, and sends them sequentially via UDP.

Packet format (little-endian) — same as trajectory_sender.py:
  Header (16 bytes):  magic(uint32) + seq(uint32) + timestamp(float64)
  Payload (9 bytes):  desiredCurvature(f32) + acceleration(f32) + shouldStop(uint8)
  Total: 25 bytes per packet
"""

import argparse
import json
import math
import struct
import socket
import time

# -- Config --
COMMA_IP = "10.200.147.253"
UDP_PORT = 5005
SEND_HZ = 20  # 0.05s interval

MAGIC = 0x4F505431
HEADER_FMT = "<IId"
PAYLOAD_FMT = "<ffB"


def load_waypoints(json_path: str) -> list[dict]:
    with open(json_path, "r") as f:
        data = json.load(f)
    return data["waypoints"]


def compute_commands(waypoints: list[dict]) -> list[dict]:
    """
    1. Compute curvature & acceleration from original 0.1s waypoints.
    2. Interpolate the command values to 0.05s by inserting midpoints (averages).
    """
    orig_dt = 0.1
    n = len(waypoints)

    # Compute speed at each 0.1s segment
    speeds = []
    for i in range(n - 1):
        dx = waypoints[i + 1]["x_m"] - waypoints[i]["x_m"]
        dy = waypoints[i + 1]["y_m"] - waypoints[i]["y_m"]
        ds = math.sqrt(dx * dx + dy * dy)
        speeds.append(ds / orig_dt)

    # Compute curvature & acceleration at each original waypoint
    orig_commands = []
    for i in range(n - 1):
        # curvature = dyaw / ds
        dyaw = waypoints[i + 1]["yaw_rad"] - waypoints[i]["yaw_rad"]
        dx = waypoints[i + 1]["x_m"] - waypoints[i]["x_m"]
        dy = waypoints[i + 1]["y_m"] - waypoints[i]["y_m"]
        ds = math.sqrt(dx * dx + dy * dy)
        curvature = dyaw / ds if ds > 1e-6 else 0.0

        # acceleration = dv / dt
        if i < len(speeds) - 1:
            accel = (speeds[i + 1] - speeds[i]) / orig_dt
        else:
            accel = 0.0

        orig_commands.append({
            "time_s": waypoints[i]["time_from_t0_s"],
            "curvature": curvature,
            "acceleration": accel,
        })

    # Interpolate commands to 0.05s: [cmd0, avg(cmd0,cmd1), cmd1, avg(cmd1,cmd2), ...]
    commands = []
    for i, cmd in enumerate(orig_commands):
        commands.append({
            "time_s": cmd["time_s"],
            "curvature": cmd["curvature"],
            "acceleration": cmd["acceleration"],
            "should_stop": False,
        })
        if i < len(orig_commands) - 1:
            nxt = orig_commands[i + 1]
            commands.append({
                "time_s": (cmd["time_s"] + nxt["time_s"]) / 2.0,
                "curvature": (cmd["curvature"] + nxt["curvature"]) / 2.0,
                "acceleration": (cmd["acceleration"] + nxt["acceleration"]) / 2.0,
                "should_stop": False,
            })

    # Final command: stop
    if commands:
        commands[-1]["should_stop"] = True
        commands[-1]["acceleration"] = 0.0

    return commands


def pack_packet(seq: int, curvature: float, acceleration: float, should_stop: bool) -> bytes:
    header = struct.pack(HEADER_FMT, MAGIC, seq, time.time())
    payload = struct.pack(PAYLOAD_FMT, curvature, acceleration, int(should_stop))
    return header + payload


def main():
    parser = argparse.ArgumentParser(description="Replay trajectory as UDP commands")
    parser.add_argument("json_path", help="Path to trajectory JSON file")
    parser.add_argument("--ip", default=COMMA_IP, help=f"comma IP (default: {COMMA_IP})")
    parser.add_argument("--port", type=int, default=UDP_PORT, help=f"UDP port (default: {UDP_PORT})")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without sending UDP")
    args = parser.parse_args()

    # Load and process
    waypoints = load_waypoints(args.json_path)
    print(f"Loaded {len(waypoints)} waypoints (0.1s interval, {waypoints[-1]['time_from_t0_s']:.1f}s total)")

    commands = compute_commands(waypoints)
    print(f"Generated {len(commands)} commands (0.05s interval)")

    if not commands:
        print("No commands to send.")
        return

    # Setup UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target_ip = args.ip
    target_port = args.port
    dt = 1.0 / SEND_HZ  # 0.05s

    print(f"\n{'[DRY RUN] ' if args.dry_run else ''}Sending to {target_ip}:{target_port} at {SEND_HZ}Hz")
    print(f"Total duration: {len(commands) * dt:.2f}s")
    print("Press Ctrl+C to abort\n")

    try:
        time.sleep(1)  # brief pause before starting
        for seq, cmd in enumerate(commands):
            t_start = time.monotonic()

            packet = pack_packet(seq, cmd["curvature"], cmd["acceleration"], cmd["should_stop"])

            if not args.dry_run:
                try:
                    sock.sendto(packet, (target_ip, target_port))
                except OSError as e:
                    print(f"  [UDP ERROR] {e}")

            # Log every 1 second (every 20 packets)
            if seq % 20 == 0 or cmd["should_stop"]:
                elapsed_total = seq * dt
                print(f"  [{elapsed_total:6.2f}s] seq={seq:4d}  "
                      f"curv={cmd['curvature']:+.5f}  "
                      f"accel={cmd['acceleration']:+.3f}  "
                      f"stop={cmd['should_stop']}")

            # Wait for next tick
            elapsed = time.monotonic() - t_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print(f"\n  Aborted at seq={seq}")
        # Send stop packet
        if not args.dry_run:
            stop_pkt = pack_packet(seq + 1, 0.0, 0.0, True)
            sock.sendto(stop_pkt, (target_ip, target_port))
            print("  Sent stop packet")

    print("\nDone.")
    sock.close()


if __name__ == "__main__":
    main()
