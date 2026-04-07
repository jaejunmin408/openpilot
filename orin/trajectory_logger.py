#!/usr/bin/env python3
"""
ADCM 경로 추종 데이터 로거 — comma에서 실행
desired trajectory vs actual vehicle state를 CSV로 저장

사용법:
  python3 trajectory_logger.py [--duration 60] [--output /tmp/traj_log.csv]

comma에서 실행 후 CSV를 PC로 복사하여 trajectory_plotter.py로 시각화
"""

import argparse
import csv
import time
import sys

import cereal.messaging as messaging


def main():
    parser = argparse.ArgumentParser(description="ADCM trajectory tracking logger")
    parser.add_argument("--duration", type=float, default=60, help="로깅 시간(초)")
    parser.add_argument("--output", type=str, default="/tmp/traj_log.csv", help="CSV 출력 경로")
    args = parser.parse_args()

    sm = messaging.SubMaster([
        'modelV2', 'carState', 'controlsState',
        'longitudinalPlan', 'livePose',
    ])

    fields = [
        'time',
        # desired (from modelV2 / action)
        'desired_curv', 'desired_accel', 'should_stop',
        # desired trajectory 첫 5개 포인트 (x, y)
        'traj_x0', 'traj_y0',
        'traj_x5', 'traj_y5',
        'traj_x10', 'traj_y10',
        'traj_x15', 'traj_y15',
        'traj_x20', 'traj_y20',
        # actual vehicle state
        'v_ego', 'a_ego',
        'steer_angle_deg', 'steer_rate_deg',
        'actual_curv',
        # lateral
        'lateral_offset_y0',
        # livePose
        'yaw_rate',
        'vel_x', 'vel_y',
    ]

    f = open(args.output, 'w', newline='')
    writer = csv.DictWriter(f, fieldnames=fields)
    writer.writeheader()

    t_start = time.monotonic()
    count = 0

    print(f"[trajectory_logger] 로깅 시작: {args.duration}초, 출력: {args.output}")
    print(f"[trajectory_logger] Ctrl+C로 조기 종료 가능")

    try:
        while time.monotonic() - t_start < args.duration:
            sm.update(100)

            if not sm.updated['controlsState']:
                continue

            t = time.monotonic() - t_start
            cs = sm['controlsState']
            car = sm['carState']
            mv2 = sm['modelV2']
            lp = sm['livePose']

            row = {
                'time': round(t, 3),
                # desired
                'desired_curv': round(cs.desiredCurvature, 5),
                'desired_accel': round(mv2.action.desiredAcceleration, 3) if mv2.valid else 0,
                'should_stop': int(mv2.action.shouldStop) if mv2.valid else 1,
                # trajectory points (sampled)
                'traj_x0': round(mv2.position.x[0], 3) if len(mv2.position.x) > 0 else 0,
                'traj_y0': round(mv2.position.y[0], 3) if len(mv2.position.y) > 0 else 0,
                'traj_x5': round(mv2.position.x[5], 3) if len(mv2.position.x) > 5 else 0,
                'traj_y5': round(mv2.position.y[5], 3) if len(mv2.position.y) > 5 else 0,
                'traj_x10': round(mv2.position.x[10], 3) if len(mv2.position.x) > 10 else 0,
                'traj_y10': round(mv2.position.y[10], 3) if len(mv2.position.y) > 10 else 0,
                'traj_x15': round(mv2.position.x[15], 3) if len(mv2.position.x) > 15 else 0,
                'traj_y15': round(mv2.position.y[15], 3) if len(mv2.position.y) > 15 else 0,
                'traj_x20': round(mv2.position.x[20], 3) if len(mv2.position.x) > 20 else 0,
                'traj_y20': round(mv2.position.y[20], 3) if len(mv2.position.y) > 20 else 0,
                # actual
                'v_ego': round(car.vEgo, 3),
                'a_ego': round(car.aEgo, 3),
                'steer_angle_deg': round(car.steeringAngleDeg, 2),
                'steer_rate_deg': round(car.steeringRateDeg, 2),
                'actual_curv': round(cs.curvature, 5),
                # lateral offset
                'lateral_offset_y0': round(mv2.position.y[0], 4) if len(mv2.position.y) > 0 else 0,
                # livePose
                'yaw_rate': round(lp.angularVelocityDevice.z.value, 4) if lp.valid else 0,
                'vel_x': round(lp.velocityDevice.x.value, 3) if lp.valid else 0,
                'vel_y': round(lp.velocityDevice.y.value, 3) if lp.valid else 0,
            }

            writer.writerow(row)
            count += 1

            if count % 100 == 0:
                f.flush()
                print(f"  [{t:.1f}s] curv: des={cs.desiredCurvature:.4f} act={cs.curvature:.4f} | "
                      f"v={car.vEgo*3.6:.1f}km/h | steer={car.steeringAngleDeg:.1f}°")

    except KeyboardInterrupt:
        print("\n[trajectory_logger] 사용자 중단")

    f.close()
    elapsed = time.monotonic() - t_start
    print(f"[trajectory_logger] 완료: {count}개 샘플, {elapsed:.1f}초, 저장: {args.output}")


if __name__ == "__main__":
    main()
