#!/usr/bin/env python3
"""udp_bridge curvature 로그 뷰어 서버 (standalone).

logs/udp_bridge_debug_*.log 를 브라우저에서 열어보기 위한 최소 HTTP 서버.
파싱/그리기는 전부 브라우저(curv_log.html)에서 하고, 이 서버는
  - 로그 파일 목록 (/api/logs)
  - 로그 파일 원문 (/api/log?name=...)
  - 뷰어 HTML (/)
만 서빙한다. 의존성은 Python stdlib 뿐이라 openpilot venv 없이도 돈다.

실행 (기기에서):
  cd /data/openpilot && python3 curv_log_viz/curv_log_server.py
  # 다른 로그 디렉터리를 보고 싶으면
  python3 curv_log_viz/curv_log_server.py --dir /data/openpilot/logs --port 8081

그 다음 PC 브라우저에서 http://<기기IP>:8081 접속.
서버 없이 curv_log.html 을 그냥 열어서 로그 파일을 드래그&드롭 해도 된다.
"""
from __future__ import annotations

import argparse
import json
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

SERVE_DIR = Path(__file__).parent
INDEX_FILE = "curv_log.html"
DEFAULT_LOG_DIR = SERVE_DIR.parent / "logs"
LOG_GLOBS = ("udp_bridge_debug_*.log",)
SAFE_NAME = re.compile(r"^[A-Za-z0-9._-]+$")

LOG_DIR = DEFAULT_LOG_DIR   # main() 에서 덮어씀


def list_logs():
  out = []
  for pattern in LOG_GLOBS:
    for p in LOG_DIR.glob(pattern):
      if not p.is_file():
        continue
      st = p.stat()
      out.append({"name": p.name, "size": st.st_size, "mtime": st.st_mtime})
  out.sort(key=lambda d: d["mtime"], reverse=True)
  return out


def resolve_log(name: str) -> Path | None:
  """디렉터리 밖으로 나가는 이름은 거부."""
  if not name or not SAFE_NAME.match(name):
    return None
  p = (LOG_DIR / name).resolve()
  try:
    p.relative_to(LOG_DIR.resolve())
  except ValueError:
    return None
  return p if p.is_file() else None


class Handler(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"

  def log_message(self, fmt, *args):   # 접속 로그 조용히
    pass

  def _send(self, code, body: bytes, ctype="text/plain; charset=utf-8"):
    self.send_response(code)
    self.send_header("Content-Type", ctype)
    self.send_header("Content-Length", str(len(body)))
    self.send_header("Cache-Control", "no-store")
    self.end_headers()
    try:
      self.wfile.write(body)
    except (BrokenPipeError, ConnectionResetError):
      pass

  def _send_json(self, code, obj):
    self._send(code, json.dumps(obj).encode(), "application/json; charset=utf-8")

  def do_GET(self):
    u = urlparse(self.path)
    q = parse_qs(u.query)

    if u.path in ("/", "/index.html", "/" + INDEX_FILE):
      f = SERVE_DIR / INDEX_FILE
      if not f.is_file():
        self._send(500, f"{INDEX_FILE} 없음".encode())
        return
      self._send(200, f.read_bytes(), "text/html; charset=utf-8")
      return

    if u.path == "/api/logs":
      self._send_json(200, {"dir": str(LOG_DIR), "logs": list_logs()})
      return

    if u.path == "/api/log":
      name = (q.get("name") or [""])[0]
      p = resolve_log(name)
      if p is None:
        self._send_json(404, {"error": f"로그 없음: {name!r}"})
        return
      self._send(200, p.read_bytes(), "text/plain; charset=utf-8")
      return

    self._send(404, b"not found")


def main():
  global LOG_DIR
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument("--dir", default=str(DEFAULT_LOG_DIR), help="로그 디렉터리 (기본: openpilot/logs)")
  ap.add_argument("--port", type=int, default=8081)
  ap.add_argument("--host", default="0.0.0.0")
  args = ap.parse_args()

  LOG_DIR = Path(args.dir).expanduser().resolve()
  if not LOG_DIR.is_dir():
    raise SystemExit(f"로그 디렉터리가 없습니다: {LOG_DIR}")

  n = len(list_logs())
  print(f"curv_log_viz: {LOG_DIR} ({n}개 로그)")
  print(f"curv_log_viz: http://{args.host}:{args.port}  (PC 브라우저에서 기기 IP 로 접속)")
  ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
  main()
