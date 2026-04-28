#!/usr/bin/env python3
"""
Browser-based trajectory visualizer for ADCM_test.

Canvas/WebSocket UI consuming the binary UDP streams from selfdrive/modeld/udp_bridge.py:
  - UDP 5006: COMA 36B (livePose dynamics — speed/accel/yaw_rate 표시 전용)
  - UDP 5007: 1243B ADCM mirror (50 trajectory points + ego + meta)

차량 위치/heading 은 항상 ADCM ego 직송 (실차/sim 동일). sim 모드에선 sender
(test_udp_sender / adcm_trajectory_sender) 가 gpsLocationExternal + livePose 로
실 ADCM 의 자체 localization 을 모방해 보내주므로 viz 측 분기 불필요.

Publishes over WebSocket 8765 and serves index.html on HTTP 8080.

Coordinate anchor: first ADCM ego is subtracted from all x/y before emission
so the browser works with small numbers (실차 UTM 330km easting 도, sim MetaDrive
xy 도 양쪽 다 처리).

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
COMA_FMT = "<Idddd"      # 36B: magic, v_fwd, v_right, yaw_rate, a_fwd
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
    magic, v_fwd, v_right, yaw_rate, a_fwd = struct.unpack(COMA_FMT, data)
    if magic != COMA_MAGIC:
        return None
    return {
        "v_fwd": float(v_fwd), "v_right": float(v_right),
        "yaw_rate": float(yaw_rate), "a_fwd": float(a_fwd),
    }


# ── Runtime state ────────────────────────────────────────────────────────────
class State:
    # 차량 pose 는 항상 ADCM ego 직송 (실차/sim 동일). sender 가 gpsLocationExternal +
    # livePose 로 진짜 ADCM localization 을 모방해 보내주므로 viz 측 적분/dead-reckon 없음.
    # COMA 는 dynamics (speed / accel / yaw_rate) 표시 용도 only.

    anchor: tuple[float, float] | None = None  # set on first ADCM packet

    # Vehicle state in global frame (pre-anchor subtraction at broadcast time).
    #   position/heading — ADCM ego 직송
    #   dynamics         — COMA (livePose.velocityDevice / accelerationDevice / angularVelocityDevice)
    vx: float = 0.0
    vy: float = 0.0
    vheading: float = 0.0
    vspeed: float = 0.0
    vaccel: float = 0.0
    vv_fwd: float = 0.0
    vv_right: float = 0.0
    vyaw_rate: float = 0.0

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

        # Dynamics 만 갱신 (속도/가속/yaw_rate 표시용). 위치/heading 은 AdcmProtocol 담당.
        # ADCM 자체에 speed/accel 없으므로 COMA 의 livePose 파생값을 쓰는 것.
        State.vspeed = math.hypot(parsed["v_fwd"], parsed["v_right"])
        State.vaccel = parsed["a_fwd"]
        State.vv_fwd = parsed["v_fwd"]
        State.vv_right = parsed["v_right"]
        State.vyaw_rate = parsed["yaw_rate"]

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

        # ADCM ego 를 그대로 차량 위치/heading 으로 신뢰. 실차에선 Orin 의 GPS+IMU+카메라
        # 융합 결과, sim 에선 sender 가 gpsLocationExternal+livePose 로 모방한 값이라
        # 어느 모드든 "가장 좋은 추정치" 임. 적분 경로 자체가 없으므로 drift 0.
        State.vx = ego_x
        State.vy = ego_y
        State.vheading = ego_yaw
        broadcast_vehicle()


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
    print(f"[server] pose source: ADCM ego 직송 (sim/실차 동일)")

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
    print(f"[server] waiting for first ADCM packet to set anchor …")
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
