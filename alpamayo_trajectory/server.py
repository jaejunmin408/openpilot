#!/usr/bin/env python3
"""
브라우저 시각화 서버.

수신:
  - UDP 5006 (VehicleStateUDPProtocol): livepose_to_viz.py 가 보내는
    LocalWorld pose + 6초 trail (type=vehicle, trajectory)
  - UDP 5007 (AcPathUDPProtocol): udp_bridge.py 가 수신한 ac_decoded_path.json
    을 그대로 미러링한 것 (→ type=trajectory_local 로 변환 후 브로드캐스트)

전송:
  - WebSocket 8765: 위 수신 JSON 을 연결된 모든 브라우저에 브로드캐스트
  - HTTP  8080 : index.html 서빙
"""
import asyncio
import json
from pathlib import Path

import websockets
import http.server
import threading

VEHICLE_STATE_UDP_PORT = 5006  # livepose_to_viz → VehicleStateUDPProtocol
AC_PATH_UDP_PORT = 5007        # udp_bridge 미러 → AcPathUDPProtocol
WS_PORT = 8765
HTTP_PORT = 8080


# ── WebSocket 클라이언트 관리 ──
clients = set()
latest_data = None


async def ws_handler(websocket):
    clients.add(websocket)
    try:
        # 연결 시 최신 데이터가 있으면 즉시 전송
        if latest_data is not None:
            await websocket.send(latest_data)
        async for _ in websocket:
            pass  # 클라이언트 메시지 무시
    finally:
        clients.discard(websocket)


async def broadcast(message):
    global latest_data
    latest_data = message
    if clients:
        await asyncio.gather(
            *[c.send(message) for c in clients],
            return_exceptions=True,
        )


# ── ac_decoded_path.json 수신 (pred_xyz, local) ──
class AcPathUDPProtocol(asyncio.DatagramProtocol):
    def __init__(self):
        self.count = 0

    def datagram_received(self, data, addr):
        try:
            doc = json.loads(data.decode())
        except (json.JSONDecodeError, UnicodeDecodeError):
            return

        pred_xyz = doc.get('pred_xyz')
        if not isinstance(pred_xyz, list) or not pred_xyz:
            return
        pred_yaw = doc.get('pred_yaw_rad') or []
        pred_v = doc.get('pred_v_mps') or []
        pred_curv = doc.get('pred_curvature') or []
        dt_s = float(doc.get('plan_dt_s', 0.1))

        self.count += 1
        points = []
        for i, xyz in enumerate(pred_xyz):
            x = float(xyz[0]); y = float(xyz[1])
            yaw = float(pred_yaw[i]) if i < len(pred_yaw) else 0.0
            v = float(pred_v[i]) if i < len(pred_v) else 0.0
            curv = float(pred_curv[i]) if i < len(pred_curv) else 0.0
            points.append({
                'x': round(x, 4),
                'y': round(y, 4),
                'yaw': round(yaw, 4),
                'vel': round(v, 3),
                'curvature': round(curv, 5),
            })

        msg = json.dumps({
            'type': 'trajectory_local',
            'seq': self.count,
            'num_points': len(points),
            'dt_s': dt_s,
            'points': points,
            'packet_count': self.count,
        })
        asyncio.ensure_future(broadcast(msg))
        print(f"[AC_PATH] pkt#{self.count} pts={len(points)} dt={dt_s:.3f}s "
              f"label={doc.get('label')}")


# ── vehicle state + trajectory (LocalWorld) JSON 수신 ──
class VehicleStateUDPProtocol(asyncio.DatagramProtocol):
    ALLOWED_TYPES = {'vehicle', 'trajectory'}

    def datagram_received(self, data, addr):
        try:
            state = json.loads(data.decode())
            if state.get('type') in self.ALLOWED_TYPES:
                msg = json.dumps(state)
                asyncio.ensure_future(broadcast(msg))
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass


# ── HTTP 서버 (index.html 서빙, 별도 스레드) ──
SERVE_DIR = Path(__file__).parent


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()


def start_http_server():
    handler = lambda *args, **kwargs: NoCacheHandler(
        *args, directory=str(SERVE_DIR), **kwargs
    )
    httpd = http.server.HTTPServer(('0.0.0.0', HTTP_PORT), handler)
    httpd.serve_forever()


async def main():
    print(f"[Server] UDP vehicle state on port {VEHICLE_STATE_UDP_PORT}")
    print(f"[Server] UDP ac_decoded_path on port {AC_PATH_UDP_PORT}")
    print(f"[Server] WebSocket on port {WS_PORT}")
    print(f"[Server] HTTP on port {HTTP_PORT}")
    print(f"[Server] Open http://localhost:{HTTP_PORT}/ in browser")

    # HTTP (별도 스레드)
    threading.Thread(target=start_http_server, daemon=True).start()

    loop = asyncio.get_event_loop()

    # UDP — vehicle state + trajectory (LocalWorld)
    await loop.create_datagram_endpoint(
        lambda: VehicleStateUDPProtocol(),
        local_addr=('0.0.0.0', VEHICLE_STATE_UDP_PORT),
    )

    # UDP — ac_decoded_path.json 미러 (udp_bridge 가 포워드)
    await loop.create_datagram_endpoint(
        lambda: AcPathUDPProtocol(),
        local_addr=('0.0.0.0', AC_PATH_UDP_PORT),
    )

    # WebSocket
    async with websockets.serve(ws_handler, '0.0.0.0', WS_PORT):
        await asyncio.Future()  # run forever


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Server] stopped")
