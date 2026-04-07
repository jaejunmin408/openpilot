#!/usr/bin/env python3
"""sender ↔ udp_bridge 패킷 호환성 테스트 (네트워크 없이 로컬 검증)"""

import json
import math
import struct
import numpy as np
import sys
sys.path.insert(0, "/home/a/orin/work")

from adcm_trajectory_sender import pack_frame, MAGIC, HEADER_SIZE as S_HEADER_SIZE, POINT_SIZE as S_POINT_SIZE, MAX_POINTS as S_MAX_POINTS, EGO_SIZE as S_EGO_SIZE, META_SIZE as S_META_SIZE, FOOTER_SIZE as S_FOOTER_SIZE
S_PACKET_SIZE = S_HEADER_SIZE + S_POINT_SIZE * S_MAX_POINTS + S_EGO_SIZE + S_META_SIZE + S_FOOTER_SIZE

# ---- udp_bridge 파싱 로직 재현 (openpilot 의존성 없이) ----
HEADER_FMT = "<IId"
POINT_FMT = "<dddd"
EGO_FMT = "<ddd"
META_FMT = "<d?dB"
FOOTER_FMT = "<B"
MAX_POINTS = 50

HEADER_SIZE = struct.calcsize(HEADER_FMT)
POINT_SIZE = struct.calcsize(POINT_FMT)
EGO_SIZE = struct.calcsize(EGO_FMT)
META_SIZE = struct.calcsize(META_FMT)
FOOTER_SIZE = struct.calcsize(FOOTER_FMT)
EXPECTED_SIZE = HEADER_SIZE + POINT_SIZE * MAX_POINTS + EGO_SIZE + META_SIZE + FOOTER_SIZE

def parse_packet(data):
    offset = 0
    magic, seq, ts = struct.unpack(HEADER_FMT, data[offset:offset+HEADER_SIZE])
    offset += HEADER_SIZE
    points = []
    for _ in range(MAX_POINTS):
        x, y, yaw, vel = struct.unpack(POINT_FMT, data[offset:offset+POINT_SIZE])
        points.append((x, y, yaw, vel))
        offset += POINT_SIZE
    ego_x, ego_y, ego_yaw = struct.unpack(EGO_FMT, data[offset:offset+EGO_SIZE])
    offset += EGO_SIZE
    target_accel, drive_mode, emergency, turn_signal = struct.unpack(META_FMT, data[offset:offset+META_SIZE])
    offset += META_SIZE
    n_valid = struct.unpack(FOOTER_FMT, data[offset:offset+FOOTER_SIZE])[0]
    return {
        "magic": magic, "seq": seq, "points": points[:n_valid],
        "ego": (ego_x, ego_y, ego_yaw), "target_accel": target_accel,
        "drive_mode": drive_mode, "turn_signal": turn_signal, "n_valid": n_valid
    }

# ---- 변환 로직 재현 ----
def convert(parsed):
    points = parsed["points"]
    ego_x, ego_y, ego_yaw = parsed["ego"]
    n = len(points)
    cos_h = math.cos(-ego_yaw)
    sin_h = math.sin(-ego_yaw)
    rel_x, rel_y, rel_yaw, vel = [], [], [], []
    for px, py, pyaw, pvel in points:
        dx, dy = px - ego_x, py - ego_y
        rel_x.append(dx * cos_h - dy * sin_h)
        rel_y.append(dx * sin_h + dy * cos_h)
        rel_yaw.append(pyaw - ego_yaw)
        vel.append(max(pvel, 0.5))
    # Time axis
    cum_time = [0.0]
    for i in range(1, n):
        ds = math.sqrt((rel_x[i]-rel_x[i-1])**2 + (rel_y[i]-rel_y[i-1])**2)
        v_avg = (vel[i] + vel[i-1]) / 2
        cum_time.append(cum_time[-1] + ds / v_avg)
    return rel_x, rel_y, rel_yaw, vel, cum_time

# ---- 테스트 ----
print(f"Expected packet size: {EXPECTED_SIZE}")
print(f"Sender packet size:   {S_PACKET_SIZE}")
assert EXPECTED_SIZE == S_PACKET_SIZE, "SIZE MISMATCH!"
print("✓ Packet size matches\n")

# Load JSON and test first frame
with open("/home/a/orin/work/test_trajectory_left_turn.json") as f:
    data = json.load(f)

# Test frame at t=2.0s (turn start)
frame = data["frames"][40]  # 40th frame = 2.0s
pkt = pack_frame(42, frame)
print(f"Packed {len(pkt)} bytes")
assert len(pkt) == S_PACKET_SIZE, "Packed size mismatch!"
print("✓ Pack size correct\n")

# Parse back
parsed = parse_packet(pkt)
assert parsed["magic"] == MAGIC, "Magic mismatch!"
assert parsed["n_valid"] == 50, f"n_valid={parsed['n_valid']}, expected 50"
assert parsed["drive_mode"] == True
assert parsed["turn_signal"] == 1  # left turn at t=2.0s
print(f"✓ Parsed: seq={parsed['seq']}, n_pts={parsed['n_valid']}, turn_signal={parsed['turn_signal']}")

# Verify ego position
ego = frame["ego_position"]
assert abs(parsed["ego"][0] - ego["x"]) < 1e-6
assert abs(parsed["ego"][1] - ego["y"]) < 1e-6
print(f"✓ Ego position: ({parsed['ego'][0]:.1f}, {parsed['ego'][1]:.1f})")

# Verify trajectory points
p0 = parsed["points"][0]
t0 = frame["trajectory"][0]
assert abs(p0[0] - t0["x"]) < 1e-6
assert abs(p0[1] - t0["y"]) < 1e-6
v0 = frame["target_velocity_per_point"][0]
assert abs(p0[3] - v0) < 1e-6
print(f"✓ Point[0]: ({p0[0]:.2f}, {p0[1]:.2f}, yaw={math.degrees(p0[2]):.1f}°, vel={p0[3]:.1f})")

# Test conversion
rel_x, rel_y, rel_yaw, vel, cum_time = convert(parsed)
print(f"\n✓ Conversion OK:")
print(f"  Vehicle-relative point[0]: fwd={rel_x[0]:.2f}m, left={rel_y[0]:.2f}m, yaw={math.degrees(rel_yaw[0]):.1f}°")
print(f"  Time range: {cum_time[0]:.2f}s ~ {cum_time[-1]:.2f}s")
print(f"  Velocity[0]: {vel[0]:.1f} m/s")

# T_IDXS interpolation check
T_IDXS = [10.0 * (i/32)**2 for i in range(33)]
pos_x_interp = np.interp(T_IDXS, cum_time, rel_x)
print(f"\n✓ T_IDXS interpolation (33 points):")
print(f"  t=0.00s → pos_x={pos_x_interp[0]:.2f}m")
print(f"  t=0.98s → pos_x={pos_x_interp[10]:.2f}m")
print(f"  t={T_IDXS[-1]:.2f}s → pos_x={pos_x_interp[-1]:.2f}m (clamped at trajectory end)")

print("\n=== ALL TESTS PASSED ===")
