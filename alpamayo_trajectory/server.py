#!/usr/bin/env python3
"""
Alpamayo 경로 시각화 서버
UDP로 ALPA 패킷 수신 → global 좌표 그대로 WebSocket으로 브라우저 전송
브라우저에서 차량 이동 애니메이션 + local 변환 처리
"""
import asyncio
import json
import math
import os
import struct
import zlib
from pathlib import Path

import websockets
import http.server
import threading

# ── ALPA 패킷 상수 ──
ALPA_MAGIC = b'ALPA'
HEADER_FMT = '<4sHHIIIQQHHf'
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 44
POINT_FMT = '<5f'
POINT_SIZE = struct.calcsize(POINT_FMT)    # 20
CRC_SIZE = 4

UDP_PORT = 5005       # Alpamayo 경로 패킷
PLANT_UDP_PORT = 5006 # plant_sim 차량 상태
AC_PATH_UDP_PORT = 5007 # ac_decoded_path.json 미러 (pred_xyz, local)
WS_PORT = 8765
HTTP_PORT = 8080


def parse_packet(data: bytes):
    """ALPA 패킷 파싱. 실패시 None 반환."""
    if len(data) < HEADER_SIZE + CRC_SIZE:
        return None

    hdr = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
    if hdr[0] != ALPA_MAGIC:
        return None

    flags = hdr[2]
    tx_seq = hdr[3]
    plan_seq = hdr[4]
    sample_id = hdr[5]
    source_t0_us = hdr[6]
    tx_time_us = hdr[7]
    coord_mode = hdr[8]
    num_points = hdr[9]
    dt_s = hdr[10]

    expected = HEADER_SIZE + num_points * POINT_SIZE + CRC_SIZE
    if len(data) != expected:
        return None

    crc_expected = struct.unpack('<I', data[-CRC_SIZE:])[0]
    crc_actual = zlib.crc32(data[:-CRC_SIZE]) & 0xFFFFFFFF
    if crc_actual != crc_expected:
        return None

    if num_points == 0:
        return None

    points = []
    offset = HEADER_SIZE
    for _ in range(num_points):
        x, y, yaw, v, curv = struct.unpack(POINT_FMT, data[offset:offset + POINT_SIZE])
        points.append((x, y, yaw, v, curv))
        offset += POINT_SIZE

    return {
        'flags': flags,
        'tx_seq': tx_seq,
        'plan_seq': plan_seq,
        'coord_mode': coord_mode,
        'num_points': num_points,
        'dt_s': dt_s,
        'points': points,
    }


def global_to_local(points):
    """첫 점 기준으로 global → local 변환.
    local: x=전방, y=좌측 (Alpamayo 규약)
    """
    if not points:
        return []

    x0, y0, yaw0 = points[0][0], points[0][1], points[0][2]
    cos_r = math.cos(-yaw0)
    sin_r = math.sin(-yaw0)

    local_points = []
    for x, y, yaw, v, curv in points:
        dx = x - x0
        dy = y - y0
        lx = cos_r * dx - sin_r * dy
        ly = sin_r * dx + cos_r * dy
        local_yaw = yaw - yaw0
        local_points.append({
            'x': round(lx, 4),
            'y': round(ly, 4),
            'yaw': round(local_yaw, 4),
            'vel': round(v, 3),
            'curvature': round(curv, 5),
        })

    return local_points


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


# ── UDP 수신 ──
class UDPProtocol(asyncio.DatagramProtocol):
    def __init__(self, loop):
        self.loop = loop
        self.packet_count = 0

    def datagram_received(self, data, addr):
        packet = parse_packet(data)
        if packet is None:
            return

        self.packet_count += 1

        # global 좌표 그대로 전송 (브라우저에서 local 변환)
        global_pts = []
        for x, y, yaw, v, curv in packet['points']:
            global_pts.append({
                'x': round(x, 4),
                'y': round(y, 4),
                'yaw': round(yaw, 4),
                'vel': round(v, 3),
                'curvature': round(curv, 5),
            })

        msg = json.dumps({
            'type': 'trajectory',
            'seq': packet['tx_seq'],
            'plan_seq': packet['plan_seq'],
            'coord_mode': packet['coord_mode'],
            'num_points': packet['num_points'],
            'dt_s': packet['dt_s'],
            'points': global_pts,
            'packet_count': self.packet_count,
        })

        asyncio.ensure_future(broadcast(msg))

        if self.packet_count % 50 == 1:
            print(f"[UDP] pkt#{self.packet_count} seq={packet['tx_seq']} "
                  f"pts={packet['num_points']} dt={packet['dt_s']:.3f}s "
                  f"mode={'world' if packet['coord_mode'] == 1 else 'local'}")


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


# ── plant_sim 차량 상태 수신 ──
class PlantUDPProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        try:
            state = json.loads(data.decode())
            if state.get('type') == 'vehicle':
                msg = json.dumps(state)
                asyncio.ensure_future(broadcast(msg))
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass


# ── HTTP 서버 (index.html 서빙, 별도 스레드) ──
SERVE_DIR = Path(__file__).parent


def start_http_server():
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(SERVE_DIR), **kwargs
    )
    httpd = http.server.HTTPServer(('0.0.0.0', HTTP_PORT), handler)
    httpd.serve_forever()


async def main():
    print(f"[Server] UDP trajectory on port {UDP_PORT}")
    print(f"[Server] UDP plant_sim on port {PLANT_UDP_PORT}")
    print(f"[Server] UDP ac_decoded_path on port {AC_PATH_UDP_PORT}")
    print(f"[Server] WebSocket on port {WS_PORT}")
    print(f"[Server] HTTP on port {HTTP_PORT}")
    print(f"[Server] Open http://localhost:{HTTP_PORT}/ in browser")

    # HTTP (별도 스레드)
    threading.Thread(target=start_http_server, daemon=True).start()

    loop = asyncio.get_event_loop()

    # UDP — Alpamayo 경로 (openpilot udp_bridge가 5005를 점유하는 경우 VIZ_DISABLE_ALPA=1 로 스킵)
    if os.environ.get('VIZ_DISABLE_ALPA') != '1':
        await loop.create_datagram_endpoint(
            lambda: UDPProtocol(loop),
            local_addr=('0.0.0.0', UDP_PORT),
        )
    else:
        print(f"[Server] ALPA UDP(:{UDP_PORT}) disabled via VIZ_DISABLE_ALPA")

    # UDP — plant_sim 차량 상태
    await loop.create_datagram_endpoint(
        lambda: PlantUDPProtocol(),
        local_addr=('0.0.0.0', PLANT_UDP_PORT),
    )

    # UDP — ac_decoded_path.json 미러
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
