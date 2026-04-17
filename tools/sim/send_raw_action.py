#!/usr/bin/env python3
"""
ac_decoded_path.json을 UDP로 1회 전송 → udp_bridge가 수신.
viz 미러는 udp_bridge가 담당하므로 여기서는 순수 발행만.
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
    return 0


if __name__ == '__main__':
    sys.exit(main())
