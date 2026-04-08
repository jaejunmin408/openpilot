#!/usr/bin/env python3
"""
Alpamayo 경로 생성기 & UDP 전송기
정지 → 5초 후 15km/h → 10초 후 정지 직선 경로

사용법:
  python tools/trajectory_sender.py --host <comma_ip> --port 5005

패킷 포맷: ALPA header(44B) + 25 points(500B) + CRC32(4B)
udp_bridge.py가 5005 포트에서 수신하여 modelV2 등으로 변환/발행
"""
import argparse
import socket
import struct
import time
import zlib

import numpy as np

# ── Alpamayo 패킷 상수 ─────────────────────────────────
MAGIC = b"ALPA"
VERSION = 1
COORD_MODE_LOCAL = 0
FLAG_VALID = 1 << 0
FLAG_END_OF_STREAM = 1 << 2

HEADER_FMT = "<4sHHIIIQQHHf"
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 44
POINT_FMT = "<5f"
POINT_SIZE = struct.calcsize(POINT_FMT)     # 20

# ── 경로 파라미터 ──────────────────────────────────────
CONTROL_DT = 0.02       # 20ms (패킷 내 포인트 간격)
CONTROL_POINTS = 25     # 패킷당 포인트 수 (0.5s horizon)
TOTAL_DURATION = 10.0   # 전체 경로 시간 (초)
ACCEL_END = 5.0         # 가속 종료 시점
DECEL_START = 5.0       # 감속 시작 시점
TARGET_V = 15.0 / 3.6   # 15 km/h → 4.1667 m/s


# ── 속도/위치 프로파일 ─────────────────────────────────
def velocity_at(t: np.ndarray) -> np.ndarray:
    """시간 t에서의 목표 속도 (m/s). 배열/스칼라 모두 가능."""
    t = np.asarray(t, dtype=np.float64)
    t = np.clip(t, 0.0, TOTAL_DURATION)
    v = np.where(
        t <= ACCEL_END,
        TARGET_V * (t / ACCEL_END),                              # 선형 가속
        TARGET_V * (1.0 - (t - DECEL_START) / (TOTAL_DURATION - DECEL_START)),  # 선형 감속
    )
    return np.clip(v, 0.0, TARGET_V)


def position_at(t: np.ndarray) -> np.ndarray:
    """시간 t에서의 누적 x 위치 (속도 프로파일 적분)."""
    t = np.asarray(t, dtype=np.float64)
    a_up = TARGET_V / ACCEL_END
    a_down = TARGET_V / (TOTAL_DURATION - DECEL_START)
    x_at_mid = 0.5 * a_up * ACCEL_END ** 2          # t=5에서의 위치
    x_at_end = x_at_mid + TARGET_V * (TOTAL_DURATION - DECEL_START) \
               - 0.5 * a_down * (TOTAL_DURATION - DECEL_START) ** 2  # t=10에서의 위치

    x = np.zeros_like(t)
    # 가속 구간
    m1 = t <= ACCEL_END
    x = np.where(m1, 0.5 * a_up * t ** 2, x)
    # 감속 구간
    m2 = (t > ACCEL_END) & (t <= TOTAL_DURATION)
    dt2 = t - DECEL_START
    x = np.where(m2, x_at_mid + TARGET_V * dt2 - 0.5 * a_down * dt2 ** 2, x)
    # 정지 후
    m3 = t > TOTAL_DURATION
    x = np.where(m3, x_at_end, x)
    return x


# ── 패킷 생성 ─────────────────────────────────────────
def pack_packet(tx_seq: int, dt_s: float,
                x: np.ndarray, y: np.ndarray, yaw: np.ndarray,
                v: np.ndarray, curv: np.ndarray,
                flags: int = FLAG_VALID) -> bytes:
    n = len(x)
    header = struct.pack(
        HEADER_FMT,
        MAGIC, VERSION, flags,
        tx_seq, 0, 0,
        int(time.time() * 1e6),
        int(time.time() * 1e6),
        COORD_MODE_LOCAL, n, dt_s,
    )
    pts = bytearray()
    for i in range(n):
        pts.extend(struct.pack(POINT_FMT,
                               float(x[i]), float(y[i]), float(yaw[i]),
                               float(v[i]), float(curv[i])))
    payload = header + bytes(pts)
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    return payload + struct.pack("<I", crc)


# ── main ───────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="정지→15km/h→정지 직선 경로를 Alpamayo UDP 패킷으로 전송")
    parser.add_argument("--host", default="127.0.0.1",
                        help="comma 장치 IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5005,
                        help="UDP 포트 (default: 5005, udp_bridge 수신 포트)")
    parser.add_argument("--hold", type=float, default=3.0,
                        help="경로 종료 후 정지 유지 시간 (초)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, args.port)

    send_duration = TOTAL_DURATION + args.hold
    tx_seq = 0
    t0 = time.perf_counter()

    print("=" * 60)
    print(f"경로: 정지 → 15km/h(5s) → 정지(10s)")
    print(f"목표 속도: {TARGET_V:.3f} m/s ({TARGET_V * 3.6:.1f} km/h)")
    print(f"총 이동 거리: {float(position_at(np.array([TOTAL_DURATION]))[0]):.2f} m")
    print(f"전송 대상: {args.host}:{args.port}")
    print(f"패킷: dt={CONTROL_DT}s, points={CONTROL_POINTS}, horizon={CONTROL_DT * CONTROL_POINTS:.2f}s")
    print("=" * 60)

    while True:
        t_now = time.perf_counter() - t0
        if t_now > send_duration:
            break

        # 현재 시각 기준 25개 미래 포인트의 절대 시각
        t_pts = t_now + np.arange(CONTROL_POINTS) * CONTROL_DT

        # 각 포인트의 속도
        v_pts = velocity_at(t_pts).astype(np.float32)

        # ego 기준 상대 x 위치 (현재 위치 빼기)
        x_abs = position_at(t_pts)
        x_ego = float(position_at(np.array([t_now]))[0])
        x_pts = (x_abs - x_ego).astype(np.float32)

        # 직선 경로: y=0, yaw=0, curvature=0
        zeros = np.zeros(CONTROL_POINTS, dtype=np.float32)

        # 플래그 설정
        flags = FLAG_VALID
        if t_now >= TOTAL_DURATION:
            flags |= FLAG_END_OF_STREAM

        pkt = pack_packet(tx_seq, CONTROL_DT, x_pts, zeros, zeros, v_pts, zeros, flags)
        sock.sendto(pkt, target)

        if tx_seq % 50 == 0:
            v_now = float(velocity_at(np.array([t_now]))[0])
            print(f"  t={t_now:6.2f}s  v={v_now:5.2f}m/s ({v_now * 3.6:5.1f}km/h)  "
                  f"x={x_ego:6.2f}m  tx={tx_seq}  flags=0x{flags:04x}")

        tx_seq += 1

        # 20ms 주기 유지
        next_t = t0 + tx_seq * CONTROL_DT
        dt_sleep = next_t - time.perf_counter()
        if dt_sleep > 0:
            time.sleep(dt_sleep)

    print("=" * 60)
    print(f"전송 완료: {tx_seq}패킷, {time.perf_counter() - t0:.2f}초")
    sock.close()


if __name__ == "__main__":
    main()
