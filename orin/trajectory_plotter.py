#!/usr/bin/env python3
"""
ADCM 경로 추종 시각화 — PC에서 실행
trajectory_logger.py가 생성한 CSV를 읽어 matplotlib으로 플롯

사용법:
  scp comma:/tmp/traj_log.csv .
  python3 trajectory_plotter.py traj_log.csv
"""

import sys
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


def plot_trajectory_tracking(csv_path, save_path=None):
    df = pd.read_csv(csv_path)
    t = df['time'].values

    fig = plt.figure(figsize=(18, 14))
    fig.suptitle('ADCM 경로 추종 분석', fontsize=16, fontweight='bold')
    gs = GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.3)

    # ─────────────────────────────────────────
    # 1. 경로 (XY 평면) — desired trajectory snapshots
    # ─────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.set_title('Desired Trajectory (vehicle frame)')
    ax1.set_xlabel('x (forward) [m]')
    ax1.set_ylabel('y (lateral) [m]')

    # 몇 개 시점의 trajectory를 그림
    n_snapshots = min(8, len(df))
    snapshot_idxs = np.linspace(0, len(df) - 1, n_snapshots, dtype=int)
    colors = plt.cm.viridis(np.linspace(0, 1, n_snapshots))

    for idx, c in zip(snapshot_idxs, colors):
        row = df.iloc[idx]
        xs = [row['traj_x0'], row['traj_x5'], row['traj_x10'], row['traj_x15'], row['traj_x20']]
        ys = [row['traj_y0'], row['traj_y5'], row['traj_y10'], row['traj_y15'], row['traj_y20']]
        ax1.plot(xs, ys, 'o-', color=c, markersize=3, linewidth=1.2,
                 label=f't={row["time"]:.1f}s')

    ax1.legend(fontsize=7, loc='best')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(0, color='gray', linewidth=0.5, linestyle='--')

    # ─────────────────────────────────────────
    # 2. Curvature: desired vs actual
    # ─────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.set_title('Curvature Tracking')
    ax2.plot(t, df['desired_curv'], 'b-', linewidth=1.5, label='desired', alpha=0.8)
    ax2.plot(t, df['actual_curv'], 'r-', linewidth=1.0, label='actual', alpha=0.7)
    ax2.set_xlabel('time [s]')
    ax2.set_ylabel('curvature [1/m]')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # curvature error
    ax2_err = ax2.twinx()
    curv_err = df['desired_curv'] - df['actual_curv']
    ax2_err.fill_between(t, curv_err, alpha=0.15, color='orange', label='error')
    ax2_err.set_ylabel('error [1/m]', color='orange')
    ax2_err.tick_params(axis='y', labelcolor='orange')

    # ─────────────────────────────────────────
    # 3. Speed: desired vs actual
    # ─────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.set_title('Speed')
    v_ego_kmh = df['v_ego'] * 3.6
    ax3.plot(t, v_ego_kmh, 'r-', linewidth=1.5, label='actual (vEgo)')
    ax3.set_xlabel('time [s]')
    ax3.set_ylabel('speed [km/h]')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # ─────────────────────────────────────────
    # 4. Steering angle
    # ─────────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.set_title('Steering Angle')
    ax4.plot(t, df['steer_angle_deg'], 'g-', linewidth=1.2, label='steering angle')
    ax4.set_xlabel('time [s]')
    ax4.set_ylabel('angle [deg]')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.axhline(0, color='gray', linewidth=0.5, linestyle='--')

    # ─────────────────────────────────────────
    # 5. Lateral offset (position.y[0])
    # ─────────────────────────────────────────
    ax5 = fig.add_subplot(gs[2, 0])
    ax5.set_title('Lateral Offset (position.y[0])')
    ax5.plot(t, df['lateral_offset_y0'], 'm-', linewidth=1.2)
    ax5.set_xlabel('time [s]')
    ax5.set_ylabel('offset [m]')
    ax5.grid(True, alpha=0.3)
    ax5.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    ax5.fill_between(t, df['lateral_offset_y0'], alpha=0.2, color='magenta')

    # ─────────────────────────────────────────
    # 6. Acceleration + shouldStop
    # ─────────────────────────────────────────
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.set_title('Acceleration & shouldStop')
    ax6.plot(t, df['a_ego'], 'b-', linewidth=1.2, label='a_ego (actual)')
    ax6.plot(t, df['desired_accel'], 'r--', linewidth=1.0, label='desired_accel')
    ax6.set_xlabel('time [s]')
    ax6.set_ylabel('accel [m/s²]')
    ax6.legend(loc='upper left')
    ax6.grid(True, alpha=0.3)

    # shouldStop 구간 표시
    stop_mask = df['should_stop'].values.astype(bool)
    if stop_mask.any():
        ax6.fill_between(t, ax6.get_ylim()[0], ax6.get_ylim()[1],
                         where=stop_mask, alpha=0.2, color='red', label='shouldStop')

    # ─────────────────────────────────────────
    # 요약 통계
    # ─────────────────────────────────────────
    curv_err_abs = np.abs(curv_err)
    stats_text = (
        f"── 추종 성능 요약 ──\n"
        f"Curvature RMSE: {np.sqrt(np.mean(curv_err**2)):.5f} 1/m\n"
        f"Curvature MAE:  {curv_err_abs.mean():.5f} 1/m\n"
        f"Curvature Max:  {curv_err_abs.max():.5f} 1/m\n"
        f"Lat offset RMS: {np.sqrt(np.mean(df['lateral_offset_y0']**2)):.3f} m\n"
        f"Speed avg:      {v_ego_kmh.mean():.1f} km/h\n"
        f"Samples:        {len(df)}\n"
        f"Duration:       {t[-1]:.1f} s"
    )
    fig.text(0.02, 0.02, stats_text, fontsize=9, fontfamily='monospace',
             verticalalignment='bottom',
             bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"저장: {save_path}")
    else:
        save_path = csv_path.replace('.csv', '.png')
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"저장: {save_path}")

    plt.show()


def main():
    parser = argparse.ArgumentParser(description="ADCM trajectory tracking plotter")
    parser.add_argument("csv", help="trajectory_logger.py가 생성한 CSV 파일")
    parser.add_argument("--save", type=str, default=None, help="PNG 저장 경로")
    args = parser.parse_args()

    plot_trajectory_tracking(args.csv, args.save)


if __name__ == "__main__":
    main()
