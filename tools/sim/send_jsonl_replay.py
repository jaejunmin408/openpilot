#!/usr/bin/env python3
"""
precomputed_udp_payloads.jsonl 를 순서대로 UDP 전송 (replay 모드).
각 줄이 ac_decoded_path.json 포맷의 JSON 페이로드이며, --interval 간격으로 전송.

사용법:
  python tools/sim/send_jsonl_replay.py
  python tools/sim/send_jsonl_replay.py --interval 2.0 --loop
  python tools/sim/send_jsonl_replay.py --file alpamayo_trajectory/precomputed_udp_payloads.jsonl --host 192.168.1.100
"""
import argparse
import socket
import sys
import time
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default='alpamayo_trajectory/precomputed_udp_payloads.jsonl',
                    help='JSONL 파일 경로 (한 줄 = 하나의 UDP 페이로드)')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5005)
    ap.add_argument('--interval', type=float, default=1.0,
                    help='패킷 간 전송 간격 (초, default: 1.0)')
    ap.add_argument('--loop', action='store_true',
                    help='파일 끝에 도달하면 처음부터 반복')
    args = ap.parse_args()

    path = Path(args.file)
    if not path.is_file():
        print(f"file not found: {path}", file=sys.stderr)
        return 1

    lines = [l.strip() for l in path.read_text().splitlines() if l.strip()]
    if not lines:
        print("JSONL 파일이 비어 있습니다.", file=sys.stderr)
        return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, args.port)

    print(f"replay: {len(lines)}개 패킷, interval={args.interval}s → {args.host}:{args.port}"
          + (" (loop)" if args.loop else ""))

    iteration = 0
    while True:
        for i, line in enumerate(lines):
            data = line.encode()
            if len(data) > 65507:
                print(f"[{i}] 페이로드 {len(data)}B > UDP 최대 65507B, 건너뜀", file=sys.stderr)
                continue

            sock.sendto(data, target)
            print(f"[{iteration * len(lines) + i + 1:4d}] sent {len(data)}B  ({i + 1}/{len(lines)})")

            if i < len(lines) - 1:
                time.sleep(args.interval)

        iteration += 1
        if not args.loop:
            break

        print(f"--- loop {iteration} 완료, 재시작 ---")
        time.sleep(args.interval)

    print("전송 완료")
    return 0


if __name__ == '__main__':
    sys.exit(main())
