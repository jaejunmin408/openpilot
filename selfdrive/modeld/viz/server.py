#!/usr/bin/env python3
"""
udp_bridge.py 의 viz 송신을 받아 브라우저로 보여주는 서버. comma 디바이스에서 직접 실행.

수신 (모두 127.0.0.1):
  - UDP 5006: vehicle / trajectory JSON  (LocalWorld pose + 6초 trail)
  - UDP 5008: trajectory_world JSON     (anchor 박힌 path, 새 패킷마다 1회)

송신:
  - HTTP 8080  : index.html (같은 디렉토리)
  - WS   8765  : 위 JSON 들 그대로 broadcast + 첫 vehicle 좌표를 display anchor 로 빼서 작은 수로 변환

좌표:
  LocalWorld 원점은 udp_bridge 가 처음 본 livePose 시점 ego 위치. 시간이 지나면 값이 커지므로
  viz 가 첫 vehicle 패킷의 (x,y) 를 display anchor 로 잡아 모든 좌표에서 빼고 broadcast.
  geometry 는 보존됨 (모든 점에 같은 offset).
"""
from __future__ import annotations

import argparse
import asyncio
import http.server
import json
import os
import sys
import threading
from pathlib import Path

import websockets


# ── State ────────────────────────────────────────────────────────────────────
class State:
  # display anchor — 첫 vehicle 패킷의 (x, y) 를 빼서 작은 좌표로 변환
  display_anchor: tuple[float, float] | None = None
  vehicle_count: int = 0
  world_path_count: int = 0


clients: set = set()
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
    await asyncio.gather(*[c.send(msg) for c in clients], return_exceptions=True)


def schedule_broadcast(msg_type: str, payload: dict):
  asyncio.ensure_future(broadcast(msg_type, payload))


def offset_xy(x: float, y: float) -> tuple[float, float]:
  if State.display_anchor is None:
    return x, y
  ax, ay = State.display_anchor
  return x - ax, y - ay


# ── UDP 5006: vehicle / trajectory ───────────────────────────────────────────
class VehicleProtocol(asyncio.DatagramProtocol):
  def datagram_received(self, data, addr):
    try:
      msg = json.loads(data.decode())
    except (UnicodeDecodeError, json.JSONDecodeError):
      return

    t = msg.get("type")
    if t == "vehicle":
      x = float(msg.get("x", 0.0))
      y = float(msg.get("y", 0.0))
      if State.display_anchor is None:
        State.display_anchor = (x, y)
        print(f"[viz] display anchor set to ({x:.2f}, {y:.2f})", file=sys.stderr)
      ox, oy = offset_xy(x, y)
      State.vehicle_count += 1
      schedule_broadcast("vehicle", {
        "type": "vehicle",
        "x": round(ox, 4),
        "y": round(oy, 4),
        "heading": round(float(msg.get("heading", 0.0)), 5),
        "speed": round(float(msg.get("speed", 0.0)), 4),
        "accel": round(float(msg.get("accel", 0.0)), 4),
        "frame": int(msg.get("frame", State.vehicle_count)),
      })

    elif t == "trajectory":
      if State.display_anchor is None:
        return
      pts = msg.get("points", [])
      out_pts = []
      for p in pts:
        ox, oy = offset_xy(float(p.get("x", 0.0)), float(p.get("y", 0.0)))
        out_pts.append({
          "x": round(ox, 4),
          "y": round(oy, 4),
          "yaw": round(float(p.get("yaw", 0.0)), 5),
        })
      schedule_broadcast("vehicle_trail", {
        "type": "vehicle_trail",
        "seq": int(msg.get("seq", 0)),
        "num_points": len(out_pts),
        "points": out_pts,
      })

    elif t == "pp_goal":
      if State.display_anchor is None:
        return
      ox, oy = offset_xy(float(msg.get("x", 0.0)), float(msg.get("y", 0.0)))
      schedule_broadcast("pp_goal", {
        "type": "pp_goal",
        "x": round(ox, 4),
        "y": round(oy, 4),
        "idx": int(msg.get("idx", 0)),
        "L_d": round(float(msg.get("L_d", 0.0)), 3),
        "frame": int(msg.get("frame", 0)),
      })


# ── UDP 5008: trajectory_world (anchor 박힌 path) ────────────────────────────
class WorldPathProtocol(asyncio.DatagramProtocol):
  def datagram_received(self, data, addr):
    try:
      msg = json.loads(data.decode())
    except (UnicodeDecodeError, json.JSONDecodeError):
      return
    if msg.get("type") != "trajectory_world":
      return
    if State.display_anchor is None:
      # vehicle 패킷이 아직 안 와서 anchor 미정 → drop (다음 패킷에서 다시 받음)
      print("[viz] world path dropped — waiting for first vehicle packet", file=sys.stderr)
      return

    pts = msg.get("points", [])
    out_pts = []
    for p in pts:
      ox, oy = offset_xy(float(p.get("x", 0.0)), float(p.get("y", 0.0)))
      out_pts.append({
        "x": round(ox, 4),
        "y": round(oy, 4),
        "yaw": round(float(p.get("yaw", 0.0)), 5),
        "vel": round(float(p.get("vel", 0.0)), 4),
      })
    State.world_path_count += 1
    schedule_broadcast("world_path", {
      "type": "world_path",
      "seq": int(msg.get("seq", State.world_path_count)),
      "num_points": len(out_pts),
      "dt_s": float(msg.get("dt_s", 0.0)),
      "points": out_pts,
    })
    print(f"[viz] world_path #{State.world_path_count} broadcast "
          f"({len(out_pts)} pts, anchor at ({out_pts[0]['x']:+.2f}, {out_pts[0]['y']:+.2f}) from display origin)",
          file=sys.stderr)


# ── HTTP (index.html) ────────────────────────────────────────────────────────
SERVE_DIR = Path(__file__).parent


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
  def end_headers(self):
    self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
    super().end_headers()

  def log_message(self, fmt, *args):
    pass


def start_http_server(host: str, port: int) -> int:
  handler = lambda *args, **kwargs: NoCacheHandler(*args, directory=str(SERVE_DIR), **kwargs)
  httpd = http.server.HTTPServer((host, port), handler)
  threading.Thread(target=httpd.serve_forever, daemon=True).start()
  return httpd.server_address[1]


# ── main ─────────────────────────────────────────────────────────────────────
async def run(args):
  http_port = start_http_server(args.host, args.http_port)
  print(f"[viz] HTTP    http://{args.host}:{http_port}/")

  loop = asyncio.get_running_loop()
  await loop.create_datagram_endpoint(VehicleProtocol, local_addr=("127.0.0.1", args.vehicle_port))
  print(f"[viz] vehicle udp://127.0.0.1:{args.vehicle_port}")
  await loop.create_datagram_endpoint(WorldPathProtocol, local_addr=("127.0.0.1", args.world_port))
  print(f"[viz] world   udp://127.0.0.1:{args.world_port}")

  print(f"[viz] WS      ws://{args.host}:{args.ws_port}")
  print(f"[viz] waiting for first vehicle packet to set display anchor…")
  async with websockets.serve(ws_handler, args.host, args.ws_port):
    await asyncio.Future()


def main():
  parser = argparse.ArgumentParser(description="udp_bridge viz server (comma local)")
  parser.add_argument("--host", default="0.0.0.0")
  parser.add_argument("--vehicle-port", type=int, default=5006)
  parser.add_argument("--world-port", type=int, default=5008)
  parser.add_argument("--ws-port", type=int, default=8765)
  parser.add_argument("--http-port", type=int, default=8080)
  args = parser.parse_args()
  try:
    asyncio.run(run(args))
  except KeyboardInterrupt:
    print("\n[viz] stopped")


if __name__ == "__main__":
  main()
