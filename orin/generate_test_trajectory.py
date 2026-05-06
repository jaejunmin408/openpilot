#!/usr/bin/env python3
"""
ADCM 형식 테스트 궤적 생성기

시나리오: 2.78 m/s (10 km/h)로 직진 → 2초 후 우회전 (반경 15m)
시작점: GPS (37.4646, 127.12484) → UTM Zone 52N (334168.48, 4148064.95)
출력: ADCM UDP 패킷과 동일한 구조의 JSON 프레임 시퀀스
"""

import json
import math
import os
import numpy as np


# ============================================================
# 1. 파라미터
# ============================================================
# 시작 GPS 좌표 → UTM 변환값
START_UTM_X = 334168.48   # easting (meters)
START_UTM_Y = 4148064.95  # northing (meters)
START_YAW = 0.0           # heading = 동쪽 (x+ 방향)

SPEED = 10.0 / 3.6        # 10 km/h → 2.778 m/s
TURN_RADIUS = 15.0        # m (저속에 맞춰 반경 축소)
STRAIGHT_DIST = SPEED * 2.0  # m (2초 직진 ≈ 5.56m)
POINT_SPACING = 0.5       # m (저속이라 간격 축소)
N_TRAJ_POINTS = 50        # ADCM 궤적 포인트 수
SIM_HZ = 20               # 시뮬레이션 주기 (Hz)
SIM_DURATION = 15.0        # 시뮬레이션 총 시간 (초)
TARGET_ACCEL = 0.0         # 등속 → 가속도 0

# 좌회전 구간 길이 (90도 회전)
TURN_ARC_LEN = TURN_RADIUS * (math.pi / 2)  # ~23.6m


# ============================================================
# 2. GPS ↔ UTM 변환 (Zone 52N)
# ============================================================
def gps_to_utm(lat, lon):
    """WGS84 GPS → UTM Zone 52N"""
    zone = 52
    lon0 = (zone - 1) * 6 - 180 + 3  # central meridian = 129

    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 2 * f - f * f
    e_prime2 = e2 / (1 - e2)
    k0 = 0.9996

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lon0_rad = math.radians(lon0)

    N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)
    T = math.tan(lat_rad) ** 2
    C = e_prime2 * math.cos(lat_rad) ** 2
    A = math.cos(lat_rad) * (lon_rad - lon0_rad)

    M = a * ((1 - e2/4 - 3*e2**2/64 - 5*e2**3/256) * lat_rad
             - (3*e2/8 + 3*e2**2/32 + 45*e2**3/1024) * math.sin(2*lat_rad)
             + (15*e2**2/256 + 45*e2**3/1024) * math.sin(4*lat_rad)
             - (35*e2**3/3072) * math.sin(6*lat_rad))

    easting = k0 * N * (A + (1-T+C)*A**3/6 + (5-18*T+T**2+72*C-58*e_prime2)*A**5/120) + 500000
    northing = k0 * (M + N * math.tan(lat_rad) * (A**2/2 + (5-T+9*C+4*C**2)*A**4/24
                      + (61-58*T+T**2+600*C-330*e_prime2)*A**6/720))
    return easting, northing


# ============================================================
# 3. 기준 경로 생성 (전체 참조 경로)
# ============================================================
def generate_reference_path():
    """직진 + 좌회전 + 직진 경로를 POINT_SPACING 간격으로 생성"""
    points = []

    ox = START_UTM_X
    oy = START_UTM_Y

    # --- 직진 구간 ---
    n_straight = int(STRAIGHT_DIST / POINT_SPACING)
    for i in range(n_straight + 1):
        s = i * POINT_SPACING
        points.append({
            "x": ox + s,
            "y": oy,
            "yaw": 0.0,
            "target_velocity": SPEED,
        })

    # --- 우회전 구간 (시계 방향) ---
    cx = ox + STRAIGHT_DIST           # 원 중심 x
    cy = oy - TURN_RADIUS             # 원 중심 y (우회전이므로 아래쪽)
    n_turn = int(TURN_ARC_LEN / POINT_SPACING)
    for i in range(1, n_turn + 1):
        theta = (i * POINT_SPACING) / TURN_RADIUS
        x = cx + TURN_RADIUS * math.sin(theta)
        y = cy + TURN_RADIUS * math.cos(theta)
        yaw = -theta  # 시계 방향 → 음수 yaw
        points.append({
            "x": x,
            "y": y,
            "yaw": yaw,
            "target_velocity": SPEED,
        })

    # --- 우회전 후 직진 (heading = -pi/2, 남쪽 방향) ---
    last = points[-1]
    n_straight_after = 80
    for i in range(1, n_straight_after + 1):
        s = i * POINT_SPACING
        points.append({
            "x": last["x"],
            "y": last["y"] - s,
            "yaw": -math.pi / 2,
            "target_velocity": SPEED,
        })

    return points


# ============================================================
# 4. 기준 경로 위에서 ego 위치 시뮬레이션
# ============================================================
def simulate_ego_on_path(ref_path, hz, duration):
    dt = 1.0 / hz
    n_steps = int(duration * hz)

    cum_dist = [0.0]
    for i in range(1, len(ref_path)):
        dx = ref_path[i]["x"] - ref_path[i-1]["x"]
        dy = ref_path[i]["y"] - ref_path[i-1]["y"]
        cum_dist.append(cum_dist[-1] + math.sqrt(dx*dx + dy*dy))

    total_path_len = cum_dist[-1]
    frames = []

    for step in range(n_steps):
        t = step * dt
        ego_s = SPEED * t

        if ego_s >= total_path_len - N_TRAJ_POINTS * POINT_SPACING:
            break

        ego = interp_on_path(ref_path, cum_dist, ego_s)

        traj = []
        for i in range(N_TRAJ_POINTS):
            s = ego_s + (i + 1) * POINT_SPACING
            if s > total_path_len:
                traj.append(traj[-1].copy())
            else:
                traj.append(interp_on_path(ref_path, cum_dist, s))

        frame = {
            "time": round(t, 3),
            "ego_position": {
                "x": ego["x"],
                "y": ego["y"],
                "yaw": ego["yaw"],
            },
            "trajectory": [
                {
                    "x": p["x"],
                    "y": p["y"],
                    "yaw": p["yaw"],
                }
                for p in traj
            ],
            "target_velocity_per_point": [p["target_velocity"] for p in traj],
            "target_speed": TARGET_ACCEL,
            "drive_mode": True,
            "emergency_acceleration": 1.0,
            "turn_signal": 2 if t >= 1.5 else 0,
            "high_way": 0,
            "sizeof_trajectory": N_TRAJ_POINTS,
        }
        frames.append(frame)

    return frames


def interp_on_path(ref_path, cum_dist, s):
    idx = np.searchsorted(cum_dist, s) - 1
    idx = max(0, min(idx, len(ref_path) - 2))

    seg_len = cum_dist[idx + 1] - cum_dist[idx]
    if seg_len < 1e-9:
        r = 0.0
    else:
        r = (s - cum_dist[idx]) / seg_len

    p0 = ref_path[idx]
    p1 = ref_path[idx + 1]

    x = p0["x"] + r * (p1["x"] - p0["x"])
    y = p0["y"] + r * (p1["y"] - p0["y"])

    dyaw = p1["yaw"] - p0["yaw"]
    while dyaw > math.pi: dyaw -= 2 * math.pi
    while dyaw < -math.pi: dyaw += 2 * math.pi
    yaw = p0["yaw"] + r * dyaw

    vel = p0["target_velocity"] + r * (p1["target_velocity"] - p0["target_velocity"])

    return {"x": x, "y": y, "yaw": yaw, "target_velocity": vel}


# ============================================================
# 5. 실행
# ============================================================
if __name__ == "__main__":
    print("=== ADCM 테스트 궤적 생성 ===")
    print(f"시작점: GPS (37.4646, 127.12484)")
    print(f"        UTM ({START_UTM_X:.2f}, {START_UTM_Y:.2f})")
    print(f"속도: {SPEED:.2f} m/s ({SPEED*3.6:.0f} km/h)")
    print(f"직진: {STRAIGHT_DIST:.1f}m → 우회전 R={TURN_RADIUS}m → 직진")

    ref_path = generate_reference_path()
    print(f"\n기준 경로: {len(ref_path)}개 포인트")

    frames = simulate_ego_on_path(ref_path, SIM_HZ, SIM_DURATION)
    print(f"생성 프레임: {len(frames)}개 ({SIM_HZ}Hz, {len(frames)/SIM_HZ:.1f}초)")

    output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "test_trajectory_right_turn.json")
    with open(output_path, "w") as f:
        json.dump({
            "description": "ADCM test: GPS(37.4646,127.12484), 10km/h straight 2s then right turn R=15m",
            "start_gps": {"lat": 37.4646, "lon": 127.12484},
            "start_utm": {"easting": START_UTM_X, "northing": START_UTM_Y, "zone": "52N"},
            "speed_mps": SPEED,
            "speed_kmh": 10.0,
            "turn_radius_m": TURN_RADIUS,
            "point_spacing_m": POINT_SPACING,
            "sim_hz": SIM_HZ,
            "coordinate_frame": "UTM Zone 52N (x=easting, y=northing, yaw=0=east, CCW positive)",
            "notes": {
                "target_speed": "Actually TargetAcceleration (m/s^2), not speed",
                "target_velocity_per_point": "Per-point velocity needed for time axis conversion",
            },
            "frames": frames,
        }, f, indent=2)

    print(f"\n저장: {output_path}")

    print("\n=== 프레임 샘플 ===")
    for i in [0, 40, 80, 120, 160]:
        if i >= len(frames):
            break
        f = frames[i]
        ego = f["ego_position"]
        t0 = f["trajectory"][0]
        t49 = f["trajectory"][-1]
        print(f"\n[t={f['time']:.2f}s] ego=({ego['x']:.2f}, {ego['y']:.2f}, yaw={math.degrees(ego['yaw']):.1f}°)")
        print(f"  traj[0] =({t0['x']:.2f}, {t0['y']:.2f}, yaw={math.degrees(t0['yaw']):.1f}°)")
        print(f"  traj[49]=({t49['x']:.2f}, {t49['y']:.2f}, yaw={math.degrees(t49['yaw']):.1f}°)")
        print(f"  turn_signal={f['turn_signal']}, vel={f['target_velocity_per_point'][0]:.2f}m/s")
