#!/usr/bin/env python3
"""
Browser-based trajectory visualizer for ADCM_test.

Canvas/WebSocket UI consuming the binary UDP streams from selfdrive/modeld/udp_bridge.py:
  - UDP 5006: COMA 56B (livePose-based vehicle state)
  - UDP 5007: 1243B ADCM mirror (50 trajectory points + ego + meta)

Publishes over WebSocket 8765 and serves index.html on HTTP 8080.

Coordinate anchor: first ADCM ego (UTM Zone 52N) is subtracted from all x/y
before emission so the browser works with small numbers.

Usage:
  python3 orin/web_viz/server.py
  # open http://localhost:8080/
"""
from __future__ import annotations

import argparse
import asyncio
import http.server
import json
import math
import struct
import sys
import threading
import time
from pathlib import Path

import websockets


# ── Protocol constants (must match udp_bridge.py) ────────────────────────────
ADCM_PACKET_SIZE = 1243
MAX_POINTS = 50
ADCM_POINT_FMT = "<ddd"
ADCM_EGO_FMT = "<ddd"
ADCM_META_FMT = "<d?dB"   # target_accel, drive_mode, emergency, turn_signal
ADCM_FOOTER_FMT = "<B"
ADCM_POINT_SIZE = struct.calcsize(ADCM_POINT_FMT)
ADCM_EGO_SIZE = struct.calcsize(ADCM_EGO_FMT)
ADCM_META_SIZE = struct.calcsize(ADCM_META_FMT)
ADCM_FOOTER_SIZE = struct.calcsize(ADCM_FOOTER_FMT)

COMA_MAGIC = 0x434F4D41  # 'COMA'
COMA_FMT = "<IIdddddd"   # magic, seq, ts, yaw_ned, v_fwd, v_right, yaw_rate, a_fwd
COMA_PACKET_SIZE = struct.calcsize(COMA_FMT)


# ── Binary packet parsers ────────────────────────────────────────────────────
def parse_adcm(data: bytes):
    if len(data) != ADCM_PACKET_SIZE:
        return None
    offset = 0
    points = []
    for _ in range(MAX_POINTS):
        x, y, yaw = struct.unpack_from(ADCM_POINT_FMT, data, offset)
        points.append((x, y, yaw))
        offset += ADCM_POINT_SIZE
    ego_x, ego_y, ego_yaw = struct.unpack_from(ADCM_EGO_FMT, data, offset)
    offset += ADCM_EGO_SIZE
    target_accel, drive_mode, emergency, turn_signal = struct.unpack_from(
        ADCM_META_FMT, data, offset)
    offset += ADCM_META_SIZE
    n_valid = struct.unpack_from(ADCM_FOOTER_FMT, data, offset)[0]
    n_valid = min(n_valid, MAX_POINTS)
    return {
        "ego": (ego_x, ego_y, ego_yaw),
        "points": points[:n_valid],
        "n_valid": n_valid,
        "drive_mode": bool(drive_mode),
        "turn_signal": int(turn_signal),
        "target_accel": float(target_accel),
        "emergency": float(emergency),
    }


def parse_coma(data: bytes):
    if len(data) != COMA_PACKET_SIZE:
        return None
    magic, seq, ts, yaw_ned, v_fwd, v_right, yaw_rate, a_fwd = struct.unpack(COMA_FMT, data)
    if magic != COMA_MAGIC:
        return None
    return {
        "seq": int(seq), "ts": float(ts),
        "yaw_ned": float(yaw_ned),
        "v_fwd": float(v_fwd), "v_right": float(v_right),
        "yaw_rate": float(yaw_rate), "a_fwd": float(a_fwd),
    }


# ── Runtime state ────────────────────────────────────────────────────────────
class State:
    # Pose source = livePose (COMA). Every COMA packet updates position/heading
    # via dead-reckon from velocityDevice + orientationNED, regardless of mode.
    # ADCM ego (planner) is used only for: (1) UTM anchor, (2) planned-ego broadcast.

    anchor: tuple[float, float] | None = None  # set on first ADCM packet
    first_planned_yaw: float | None = None     # ADCM ego_yaw at anchor
    yaw_offset: float | None = None            # yaw_enu − first_planned_yaw, locked on first COMA post-anchor.
                                                # Aligns livePose yaw frame to planner frame (same starting heading).

    # Vehicle state in UTM frame (pre-anchor subtraction happens at broadcast time).
    #   position/heading — COMA dead-reckon (livePose.velocityDevice + orientationNED.z)
    #   dynamics         — COMA (livePose.velocityDevice / accelerationDevice / angularVelocityDevice)
    vx: float = 0.0
    vy: float = 0.0
    vheading: float = 0.0
    vspeed: float = 0.0
    vaccel: float = 0.0
    vv_fwd: float = 0.0
    vv_right: float = 0.0
    vyaw_rate: float = 0.0
    vts: float = 0.0
    vframe: int = 0

    actual_last_ts: float | None = None  # last COMA ts (dead-reckon dt)
    actual_initialized: bool = False     # flipped by first ADCM

    coma_drop_warn_last: float = 0.0  # rate-limit warn
    adcm_count: int = 0


clients: set = set()
# Latest broadcast per type — new clients receive all on connect
latest_by_type: dict[str, str] = {}


# ── WebSocket ────────────────────────────────────────────────────────────────
async def ws_handler(websocket):
    clients.add(websocket)
    try:
        for msg in latest_by_type.values():
            await websocket.send(msg)
        async for _ in websocket:
            pass
    except websockets.ConnectionClosed:
        pass
    finally:
        clients.discard(websocket)


async def broadcast(msg_type: str, payload: dict):
    msg = json.dumps(payload)
    latest_by_type[msg_type] = msg
    if clients:
        await asyncio.gather(
            *[c.send(msg) for c in clients],
            return_exceptions=True,
        )


def schedule_broadcast(msg_type: str, payload: dict):
    """Callable from sync code (UDP protocols)."""
    asyncio.ensure_future(broadcast(msg_type, payload))


def broadcast_vehicle():
    """Emit merged vehicle state. Requires anchor + initialized; otherwise no-op."""
    if not State.actual_initialized or State.anchor is None:
        return
    ax, ay = State.anchor
    schedule_broadcast("vehicle", {
        "type": "vehicle",
        "x": round(State.vx - ax, 4),
        "y": round(State.vy - ay, 4),
        "heading": round(State.vheading, 5),
        "speed": round(State.vspeed, 4),
        "accel": round(State.vaccel, 4),
        "v_fwd": round(State.vv_fwd, 4),
        "v_right": round(State.vv_right, 4),
        "yaw_rate": round(State.vyaw_rate, 5),
        "ts": State.vts,
        "frame": State.vframe,
    })


# ── UDP protocols ────────────────────────────────────────────────────────────
class ComaProtocol(asyncio.DatagramProtocol):
    """Port 5006 — COMA 56B binary packet from udp_bridge."""

    def datagram_received(self, data, addr):
        parsed = parse_coma(data)
        if parsed is None:
            return
        if not State.actual_initialized:
            now = time.monotonic()
            if now - State.coma_drop_warn_last > 2.0:
                print(f"[COMA] dropping packet — waiting for first ADCM anchor", file=sys.stderr)
                State.coma_drop_warn_last = now
            return

        ts = parsed["ts"]

        # Dynamics: always sourced from COMA regardless of mode (ADCM carries no speed/accel).
        State.vspeed = math.hypot(parsed["v_fwd"], parsed["v_right"])
        State.vaccel = parsed["a_fwd"]
        State.vv_fwd = parsed["v_fwd"]
        State.vv_right = parsed["v_right"]
        State.vyaw_rate = parsed["yaw_rate"]
        State.vts = ts
        State.vframe = parsed["seq"]

        # Pose (position + heading) 를 livePose 기반으로 dead-reckon.
        #
        # yaw: livePose.orientationNED.z 는 locationd 의 PoseKalman 이 cameraOdometry +
        #      IMU 를 EKF 로 융합해 만든 절대 yaw. GYRO_BIAS 를 state 로 온라인 학습하므로
        #      drift 가 bounded. 매 tick 절대값으로 덮어쓰기 때문에 viz 측에서 적분 불필요.
        #      (옛 carState/yaw_rate 적분 방식은 open-loop drift 가 unbounded 였음.)
        #
        # position: livePose.velocityDevice.x (= PoseKalman DEVICE_VELOCITY 슬라이스의 전진)
        #           를 fresh yaw 축으로 투영해서 적분. yaw 가 매 tick 리셋되므로 drift 는
        #           1차 랜덤워크로 한정 (옛 carState.vEgo × 적분된 yaw 의 2차 drift 대비 축소).
        #           yaw 와 velocity 가 같은 PoseKalman state 에서 나오므로 좌표계·타이밍 정합.
        #
        # NED → ENU 부호: 일반 수식은 (π/2 − yaw_ned) 지만, 현재 테스트 환경 (MetaDrive) 의
        # heading_theta 는 수학 CCW 를 쓰고 simulated_sensors 의 vNED = [-vy, vx] 매핑으로
        # bearing 이 사용자 시점의 회전 방향과 한 번 더 반전되어 들어옴. 결과적으로 COMA 의
        # yaw_ned 부호가 기대와 뒤집혀 있어 `+` 로 받는 것이 실제 rotation 방향과 일치한다.
        # (실차 NED 는 `π/2 − yaw_ned` — 추후 플래그로 분기 예정.)
        yaw_enu = math.pi / 2 + parsed["yaw_ned"]

        # Lock yaw_offset on first COMA after anchor — aligns livePose yaw frame to planner frame
        if State.yaw_offset is None:
            State.yaw_offset = yaw_enu - (State.first_planned_yaw or 0.0)
            print(f"[COMA] yaw offset locked at {State.yaw_offset:+.3f} rad "
                  f"(first yaw_enu={yaw_enu:+.3f}, first planned yaw={State.first_planned_yaw:+.3f})")
        corrected_yaw = yaw_enu - State.yaw_offset

        if State.actual_last_ts is not None:
            dt = max(0.001, min(ts - State.actual_last_ts, 0.2))
            v = parsed["v_fwd"]
            State.vx += v * math.cos(corrected_yaw) * dt
            State.vy += v * math.sin(corrected_yaw) * dt
        State.vheading = corrected_yaw

        State.actual_last_ts = ts
        broadcast_vehicle()


class AdcmProtocol(asyncio.DatagramProtocol):
    """Port 5007 — 1243B ADCM mirror from udp_bridge."""

    def datagram_received(self, data, addr):
        parsed = parse_adcm(data)
        if parsed is None:
            return
        State.adcm_count += 1
        ego_x, ego_y, ego_yaw = parsed["ego"]

        if State.anchor is None:
            State.anchor = (ego_x, ego_y)
            State.first_planned_yaw = ego_yaw
            State.vx = ego_x
            State.vy = ego_y
            State.vheading = ego_yaw
            State.actual_initialized = True
            print(f"[ADCM] anchor set to ({ego_x:.2f}, {ego_y:.2f}) yaw={ego_yaw:+.3f} rad")

        ax, ay = State.anchor
        n_valid = parsed["n_valid"]
        pts = [
            {"x": round(x - ax, 4), "y": round(y - ay, 4), "yaw": round(yaw, 5)}
            for (x, y, yaw) in parsed["points"][:n_valid]
        ]
        schedule_broadcast("trajectory_world", {
            "type": "trajectory_world",
            "seq": State.adcm_count,
            "num_points": n_valid,
            "points": pts,
        })
        schedule_broadcast("vehicle_planned", {
            "type": "vehicle_planned",
            "x": round(ego_x - ax, 4),
            "y": round(ego_y - ay, 4),
            "yaw": round(ego_yaw, 5),
            "target_accel": round(parsed["target_accel"], 4),
            "drive_mode": parsed["drive_mode"],
            "turn_signal": parsed["turn_signal"],
            "emergency": round(parsed["emergency"], 4),
            "n_valid": n_valid,
            "frame": State.adcm_count,
        })
        # Actual pose 는 ComaProtocol (livePose) 이 담당. 여기서는 planner ego 만 broadcast.


# ── HTTP (serves index.html) ─────────────────────────────────────────────────
SERVE_DIR = Path(__file__).parent


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def log_message(self, fmt, *args):
        pass  # quiet


def start_http_server(host: str, port: int) -> int:
    handler = lambda *args, **kwargs: NoCacheHandler(
        *args, directory=str(SERVE_DIR), **kwargs)
    httpd = http.server.HTTPServer((host, port), handler)
    actual_port = httpd.server_address[1]
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return actual_port


# ── main ─────────────────────────────────────────────────────────────────────
async def run(args):
    print("[server] pose source: livePose dead-reckon (orientationNED.z + velocityDevice)")

    # 1. HTTP
    http_port = start_http_server(args.host, args.http_port)
    print(f"[server] HTTP    http://{args.host}:{http_port}/")

    # 2. UDP endpoints
    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(
        lambda: ComaProtocol(),
        local_addr=(args.host, args.coma_port),
    )
    print(f"[server] COMA    udp://{args.host}:{args.coma_port} ({COMA_PACKET_SIZE}B)")
    await loop.create_datagram_endpoint(
        lambda: AdcmProtocol(),
        local_addr=(args.host, args.adcm_port),
    )
    print(f"[server] ADCM    udp://{args.host}:{args.adcm_port} ({ADCM_PACKET_SIZE}B)")

    # 3. WebSocket
    print(f"[server] WS      ws://{args.host}:{args.ws_port}")
    print(f"[server] waiting for first ADCM packet to set UTM anchor …")
    async with websockets.serve(ws_handler, args.host, args.ws_port):
        await asyncio.Future()


def main():
    parser = argparse.ArgumentParser(
        description="Browser-based ADCM trajectory visualizer")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--coma-port", type=int, default=5006)
    parser.add_argument("--adcm-port", type=int, default=5007)
    parser.add_argument("--ws-port", type=int, default=8765)
    parser.add_argument("--http-port", type=int, default=8080,
                        help="0 = auto-pick free port (printed on startup)")
    args = parser.parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n[server] stopped")


if __name__ == "__main__":
    main()
