#!/usr/bin/env python3
"""
ac_decoded_path.json을 UDP로 1회 전송.
udp_bridge.py가 수신해 raw_action(a,c)과 경로(pred_xyz, pred_yaw_rad, pred_v_mps)를 사용.
"""
import argparse
import socket
import sys
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default='chunk_0001_55s_traj/ac_decoded_path.json',
                    help='ac_decoded_path.json 경로')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5005)
    ap.add_argument('--viz-port', type=int, default=5007,
                    help='viz 서버 미러 포트 (0이면 미전송)')
    args = ap.parse_args()

    path = Path(args.file)
    if not path.is_file():
        print(f"file not found: {path}", file=sys.stderr)
        return 1

    data = path.read_bytes()
    if len(data) > 65507:
        print(f"payload {len(data)}B exceeds UDP max 65507B", file=sys.stderr)
        return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(data, (args.host, args.port))
    print(f"sent {len(data)}B to {args.host}:{args.port} ({path.name})")

    if args.viz_port:
        sock.sendto(data, (args.host, args.viz_port))
        print(f"sent {len(data)}B to {args.host}:{args.viz_port} (viz mirror)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
