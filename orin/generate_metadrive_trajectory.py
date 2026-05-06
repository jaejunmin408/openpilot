#!/usr/bin/env python3
"""
MetaDrive world 의 실제 도로 경로를 따라가는 ADCM reference path JSON 생성기.

동작:
  1. MetaDrive bridge 와 동일한 config 로 env 를 헤드리스로 띄운다.
  2. 간단한 P-controller 로 lane center 를 추종하며 자동 주행 → 월드 좌표의
     (x, y, yaw) 경로를 수집한다.
  3. POINT_SPACING 간격으로 리샘플, yaw unwrap → reference path 로 저장.

이 파일은 frame 단위 시뮬레이션을 하지 않는다 (이전 버전의 simulate_ego_on_path
제거). adcm_trajectory_sender.py 가 매 tick 차 GPS 를 path 위에 투영하고 lateral
오프셋을 smoothstep 으로 감쇠하는 50 점 trajectory 를 동적으로 그려 송신한다.

Usage (openpilot venv 안에서):
  cd /home/gnu/workspace/openpilot-steer-limit-gwlee
  python3 orin/generate_metadrive_trajectory.py
  python3 orin/generate_metadrive_trajectory.py --render        # 시각화
  python3 orin/generate_metadrive_trajectory.py --speed 5.0     # 18km/h

재생:
  python3 orin/adcm_trajectory_sender.py --ip 127.0.0.1 --port 10002 \\
      --json orin/test_trajectory_metadrive.json --loop
"""

import argparse
import json
import math
import os
import sys

import numpy as np

# openpilot root 를 PYTHONPATH 에 추가 (orin/ 한 단계 위)
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from metadrive.envs.metadrive_env import MetaDriveEnv  # noqa: E402

from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import create_map  # noqa: E402
from openpilot.tools.sim.bridge.metadrive.metadrive_process import apply_metadrive_patches  # noqa: E402


# ========= 파라미터 =========
DEFAULT_SPEED = 10.0 / 3.6   # 10 km/h
POINT_SPACING = 0.5          # m
TRACE_MAX_STEPS = 8000       # env step 최대 수 (안전 장치)
LOOP_CLOSE_DIST = 3.0        # 출발점 반경 (m), 이 안에 다시 들어오면 루프 종료

# P-controller 게인
K_HEADING = 1.5
K_LAT = 0.08
TRACE_THROTTLE = 0.25        # lane trace 시 일정 throttle


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def build_env(render: bool):
    """metadrive_bridge.py 의 config 를 재사용. 다만 camera/sensor 는 제거."""
    # arrive_dest_done 은 MetaDriveEnv config 키가 아니라 bridge 가 pop 해서
    # 패치에만 쓰는 플래그. env 생성 전에 패치만 적용하고 키는 넘기지 않는다.
    apply_metadrive_patches(arrive_dest_done=False)

    config = dict(
        use_render=render,
        vehicle_config=dict(enable_reverse=False, render_vehicle=False),
        image_observation=False,
        interface_panel=[],
        out_of_route_done=False,
        on_continuous_line_done=False,
        crash_vehicle_done=False,
        crash_object_done=False,
        traffic_density=0.0,
        map_config=create_map(),
        decision_repeat=1,
        physics_world_step_size=0.05,
        preload_models=False,
        show_logo=False,
    )
    env = MetaDriveEnv(config)
    return env


def get_target_heading(lane, long_pos):
    """lane 접선 방향 (rad). heading_theta_at → heading_at → 수치 미분 순으로 시도."""
    for name in ("heading_theta_at", "heading_at"):
        fn = getattr(lane, name, None)
        if fn is not None:
            try:
                return float(fn(long_pos))
            except Exception:
                pass
    try:
        p0 = lane.position(long_pos, 0.0)
        p1 = lane.position(long_pos + 0.5, 0.0)
        return math.atan2(float(p1[1] - p0[1]), float(p1[0] - p0[0]))
    except Exception:
        return None


def trace_path(env, max_steps: int, start_pose):
    """P-controller 로 자동 주행하며 (x, y, yaw) 기록. 한 바퀴 돌면 조기 종료."""
    v = env.vehicle
    sx, sy, _ = start_pose

    path = [(float(v.position[0]), float(v.position[1]), float(v.heading_theta))]
    departed = False  # 출발점에서 일정 거리 벗어났는지

    for step in range(max_steps):
        steer = 0.0
        try:
            ref_lanes = v.navigation.current_ref_lanes
            if ref_lanes:
                lane = ref_lanes[0]
                long_pos, lat_pos = lane.local_coordinates(v.position)
                tgt_hdg = get_target_heading(lane, long_pos)
                if tgt_hdg is not None:
                    hdg_err = wrap_pi(tgt_hdg - float(v.heading_theta))
                    steer = K_HEADING * hdg_err - K_LAT * float(lat_pos)
                    steer = max(-1.0, min(1.0, steer))
        except Exception:
            steer = 0.0

        action = [steer, TRACE_THROTTLE]
        result = env.step(action)
        # gym API 호환 (5-tuple 또는 4-tuple)
        if len(result) == 5:
            _, _, terminated, truncated, _ = result
            done = bool(terminated or truncated)
        else:
            _, _, done, _ = result

        x = float(v.position[0])
        y = float(v.position[1])
        yaw = float(v.heading_theta)
        path.append((x, y, yaw))

        d_from_start = math.hypot(x - sx, y - sy)
        if not departed and d_from_start > 20.0:
            departed = True
        if departed and d_from_start < LOOP_CLOSE_DIST and step > 100:
            # 한 바퀴 완료
            break
        if done:
            break

    return path


def resample_path(path, spacing):
    if len(path) < 2:
        return path
    out = [path[0]]
    target = spacing
    cum = 0.0
    for i in range(1, len(path)):
        x0, y0, yaw0 = path[i - 1]
        x1, y1, yaw1 = path[i]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-9:
            continue
        while cum + seg >= target:
            r = (target - cum) / seg
            nx = x0 + r * (x1 - x0)
            ny = y0 + r * (y1 - y0)
            dyaw = wrap_pi(yaw1 - yaw0)
            nyaw = wrap_pi(yaw0 + r * dyaw)
            out.append((nx, ny, nyaw))
            target += spacing
        cum += seg
    return out


def main():
    parser = argparse.ArgumentParser(description="MetaDrive 맵 기반 ADCM reference path JSON 생성")
    parser.add_argument("--output", default=os.path.join(_THIS_DIR, "test_trajectory_metadrive.json"))
    parser.add_argument("--render", action="store_true", help="MetaDrive 창 띄움 (trace 확인용)")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="cruise 속도 m/s (기본 2.78 = 10km/h)")
    parser.add_argument("--max-steps", type=int, default=TRACE_MAX_STEPS, help="trace 최대 step 수")
    args = parser.parse_args()

    print("=== MetaDrive 경로 dump ===")
    print("맵: metadrive_bridge.create_map() — 직진 60m + R=120 좌회전 × 4 루프")

    env = build_env(render=args.render)
    try:
        env.reset()
        v = env.vehicle
        start_pose = (float(v.position[0]), float(v.position[1]), float(v.heading_theta))
        print(f"시작 pose: x={start_pose[0]:.2f}, y={start_pose[1]:.2f}, "
              f"yaw={math.degrees(start_pose[2]):.1f}°")

        print(f"lane 추종 자동 주행 시작 (max_steps={args.max_steps})")
        raw_path = trace_path(env, args.max_steps, start_pose)
        print(f"  → {len(raw_path)}개 raw 점")
    finally:
        try:
            env.close()
        except Exception:
            pass

    path = resample_path(raw_path, POINT_SPACING)
    total_len = sum(math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
                    for i in range(1, len(path)))
    print(f"리샘플 {POINT_SPACING}m 간격: {len(path)} 점, 총 {total_len:.1f}m")

    # 누적 yaw unwrap — wrap 점에서 ±2π 점프 제거 (sender 의 forward-diff/blend 안정성)
    yaws_unwrapped = np.unwrap(np.array([p[2] for p in path]))

    path_out = [
        {"x": float(p[0]), "y": float(p[1]), "yaw": float(yaws_unwrapped[i])}
        for i, p in enumerate(path)
    ]

    out = {
        "description": "MetaDrive world lane-follow reference path (auto-traced)",
        "map": "metadrive_bridge.create_map() — 4x (straight 60m + curve R=120 90°)",
        "coordinate_frame": "MetaDrive world (x,y meters; yaw rad, CCW from sim +x; unwrapped)",
        "start_pose": {"x": start_pose[0], "y": start_pose[1], "yaw": start_pose[2]},
        "speed_mps": args.speed,
        "speed_kmh": args.speed * 3.6,
        "point_spacing_m": POINT_SPACING,
        "total_path_length_m": total_len,
        "drive_mode": True,
        "turn_signal": 0,
        "emergency_acceleration": 1.0,
        "path": path_out,
    }

    with open(args.output, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\n저장: {args.output}")

    if path:
        print("\n=== path 샘플 ===")
        for idx in [0, len(path) // 4, len(path) // 2, 3 * len(path) // 4, len(path) - 1]:
            x, y = path[idx][0], path[idx][1]
            yaw = yaws_unwrapped[idx]
            print(f"path[{idx:5d}] ({x:7.2f}, {y:7.2f})  yaw={math.degrees(yaw):7.1f}°")


if __name__ == "__main__":
    main()
