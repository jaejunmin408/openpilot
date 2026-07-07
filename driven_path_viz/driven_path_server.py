#!/usr/bin/env python3
"""실시간 '차가 실제로 지나간 경로' 뷰어 서버 (standalone).

udp_bridge 와 무관하게 동작한다. locationd 가 발행하는 livePose 만 있으면
(순정 modeld 든 udp_bridge 든 sim 이든) 실제 주행 궤적을 그린다.

의존성: cereal.messaging + Python stdlib 뿐. (websockets 불필요 — SSE 사용)

동작:
  - 백그라운드 스레드: SubMaster(["livePose", "carState"]) 구독
      → PoseIntegrator 로 속도·방향 적분해서 로컬 frame (x, y, yaw) 산출
      → MIN_STEP_M 이상 움직일 때마다 '전체 누적 경로' 에 점 추가
  - HTTP 서버(스레드별): index.html 서빙 + /stream(SSE)로 브라우저에 push

좌표계: 첫 valid livePose 시점 ego 위치를 원점(0,0)으로 하는 로컬 world frame.
  x, y 단위 m (시작점 기준 이동량). NED 규약(x=North, y=East)을 그대로 쓰며
  브라우저가 자동 fit 하므로 절대 방위는 신경 쓰지 않아도 된다.

실행 (openpilot 루트에서, 그 기계 아키텍처의 venv 로):
  # comma 기기(aarch64) — 실차 실시간:
  cd /data/openpilot && ./.venv/bin/python driven_path_viz/driven_path_server.py
  # PC sim(x86):
  cd /home/a/communication/openpilot_carla/openpilot && \
      ./.venv/bin/python <이 파일 경로>/driven_path_server.py
기기에서 실행 후, PC 브라우저에서 http://<기기IP>:8080 (또는 SSH 포워딩).
"""
from __future__ import annotations

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from cereal.messaging import SubMaster

SERVE_DIR = Path(__file__).parent
INDEX_FILE = "driven_path.html"

# 이 거리(m) 이상 움직였을 때만 경로에 새 점을 추가 (누적 경로 밀도 제한)
MIN_STEP_M = 0.2
# 적분 파라미터 (openpilot LocalWorld 와 동일 로직)
MAX_DT_SEC = 0.5   # 이보다 dt 크면 gap 으로 보고 적분 건너뜀


class PoseIntegrator:
  """livePose(velocityDevice + orientationNED) 적분 → 로컬 world frame (x, y, yaw).

  openpilot selfdrive.controls.lib.local_world.LocalWorld 의 적분식을 그대로 인라인.
  특정 체크아웃 파일에 의존하지 않도록 자립형으로 포함한다.
  좌표계: 첫 valid pose 를 원점(0,0), yaw=orientationNED.z 로.
  """
  def __init__(self):
    self.x = 0.0
    self.y = 0.0
    self.yaw = 0.0
    self._last_t_ns: int | None = None
    self._init = False

  def update(self, lp, t_ns: int):
    if not self._init:
      if lp.orientationNED.valid:
        self.yaw = lp.orientationNED.z
        self._init = True
        self._last_t_ns = t_ns
      return
    dt = (t_ns - self._last_t_ns) / 1e9
    if dt <= 0 or dt > MAX_DT_SEC:
      self._last_t_ns = t_ns
      return
    if not (lp.orientationNED.valid and lp.velocityDevice.valid):
      self._last_t_ns = t_ns
      return
    yaw_new = lp.orientationNED.z
    yaw_mid = 0.5 * (self.yaw + yaw_new)
    c, s = math.cos(yaw_mid), math.sin(yaw_mid)
    vx, vy = lp.velocityDevice.x, lp.velocityDevice.y
    self.x += (vx * c - vy * s) * dt
    self.y += (vx * s + vy * c) * dt
    self.yaw = yaw_new
    self._last_t_ns = t_ns

  def initialized(self) -> bool:
    return self._init


class State:
  """스레드(생산) ↔ HTTP 핸들러(소비) 공유 상태. lock 으로 보호."""
  def __init__(self):
    self.lock = threading.Lock()
    self.path: list[dict] = []          # 전체 누적 경로 [{x, y, yaw}, ...]
    self.have_pose = False
    self.x = 0.0
    self.y = 0.0
    self.yaw = 0.0
    self.speed = 0.0                     # m/s
    self.frame = 0
    self._last_pt: tuple[float, float] | None = None

  def push_pose(self, x, y, yaw, speed):
    with self.lock:
      self.have_pose = True
      self.x, self.y, self.yaw, self.speed = x, y, yaw, speed
      self.frame += 1
      if self._last_pt is None or math.hypot(x - self._last_pt[0], y - self._last_pt[1]) >= MIN_STEP_M:
        self.path.append({"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4)})
        self._last_pt = (x, y)

  def snapshot(self, from_idx):
    """현재 pose + from_idx 이후의 새 경로점 복사본을 반환."""
    with self.lock:
      new_pts = self.path[from_idx:]
      total = len(self.path)
      pose = {
        "have": self.have_pose,
        "x": round(self.x, 3), "y": round(self.y, 3),
        "yaw": round(self.yaw, 4), "speed": round(self.speed, 3),
        "frame": self.frame,
      }
    return pose, new_pts, total


STATE = State()
PUSH_HZ = 20.0


# ── 생산 스레드: livePose 적분 → 경로 누적 ─────────────────────────────
def pose_loop(stop: threading.Event):
  sm = SubMaster(["livePose", "carState"])
  integ = PoseIntegrator()
  while not stop.is_set():
    sm.update(100)
    if not sm.updated["livePose"]:
      continue
    lp = sm["livePose"]
    integ.update(lp, sm.logMonoTime["livePose"])
    if not integ.initialized():
      continue
    if sm.alive["carState"]:
      speed = max(float(sm["carState"].vEgo), 0.0)
    else:
      speed = math.hypot(float(lp.velocityDevice.x), float(lp.velocityDevice.y))
    STATE.push_pose(integ.x, integ.y, integ.yaw, speed)


# ── HTTP + SSE ─────────────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):
  def do_GET(self):
    if self.path.split("?")[0] in ("/", "/index.html"):
      self._serve_index()
    elif self.path.split("?")[0] == "/stream":
      self._serve_stream()
    else:
      self.send_error(404)

  def _serve_index(self):
    try:
      body = (SERVE_DIR / INDEX_FILE).read_bytes()
    except OSError:
      self.send_error(404, "index not found")
      return
    self.send_response(200)
    self.send_header("Content-Type", "text/html; charset=utf-8")
    self.send_header("Content-Length", str(len(body)))
    self.send_header("Cache-Control", "no-store")
    self.end_headers()
    self.wfile.write(body)

  def _serve_stream(self):
    self.send_response(200)
    self.send_header("Content-Type", "text/event-stream")
    self.send_header("Cache-Control", "no-cache")
    self.send_header("Connection", "keep-alive")
    self.end_headers()
    period = 1.0 / PUSH_HZ
    cursor = 0
    try:
      while True:
        pose, new_pts, total = STATE.snapshot(cursor)
        msg = {"vehicle": pose, "points": new_pts, "reset": cursor == 0}
        self.wfile.write(b"data: " + json.dumps(msg).encode() + b"\n\n")
        self.wfile.flush()
        cursor = total
        time.sleep(period)
    except (BrokenPipeError, ConnectionResetError, OSError):
      return   # 브라우저 연결 종료

  def log_message(self, fmt, *args):
    pass


def main():
  ap = argparse.ArgumentParser(description="실시간 실제 주행 경로 뷰어 (standalone, SSE)")
  ap.add_argument("--host", default="0.0.0.0")
  ap.add_argument("--port", type=int, default=8080)
  ap.add_argument("--rate", type=float, default=20.0, help="브라우저 push 주기 (Hz)")
  args = ap.parse_args()

  global PUSH_HZ
  PUSH_HZ = args.rate

  stop = threading.Event()
  threading.Thread(target=pose_loop, args=(stop,), daemon=True).start()

  httpd = ThreadingHTTPServer((args.host, args.port), Handler)
  print(f"[driven_path] http://{args.host}:{args.port}/  (SSE /stream, push {PUSH_HZ:g}Hz)")
  print(f"[driven_path] livePose 구독 중… (locationd 필요, MIN_STEP={MIN_STEP_M}m)")
  try:
    httpd.serve_forever()
  except KeyboardInterrupt:
    print("\n[driven_path] stopped")
    stop.set()


if __name__ == "__main__":
  main()
