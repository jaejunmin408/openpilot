#!/usr/bin/env python3
"""udp_bridge 횡제어기 실시간 전환 콘솔.

udp_bridge 는 manager 가 띄우는 프로세스라 stdin 이 터미널이 아니다(키를 직접
읽으면 manager 터미널과 서로 뺏는다). 그래서 키 입력은 이 도구가 받아 UDP 제어
포트로 명령을 보내고, 돌아온 상태를 화면에 갱신한다. udp_bridge 는 파이썬이라
제어기 전환에 프로세스 재시작이 필요 없다 — 주행 중에 바로 바뀐다.

  디바이스에서:  python selfdrive/modeld/lat_ctl.py
  노트북에서:    python lat_ctl.py --host 192.168.43.1

키:
  1 / 2 / 3   pure_pursuit / comma_mpc / alpasim_mpc 선택
  m           다음 제어기로 순환
  c           compare 토글 (비활성 MPC 도 매 loop 계산해 로그에 남김)
  x           활성 제어기 내부 상태 리셋
  q / Ctrl-C  종료

TTY 가 아니거나(파이프·스크립트) 일회성 조작만 필요하면 인자 모드를 쓴다:
  python lat_ctl.py --set comma_mpc
  python lat_ctl.py --cycle
  python lat_ctl.py --compare on
  python lat_ctl.py --status
"""
from __future__ import annotations

import argparse
import json
import os
import select
import socket
import sys
import termios
import tty

DEFAULT_PORT = 5009            # udp_bridge.LAT_CTL_PORT
REQUEST_TIMEOUT_S = 0.5
REFRESH_S = 0.2                # 상태 폴링 주기

MODE_KEYS = {"1": "pure_pursuit", "2": "comma_mpc", "3": "alpasim_mpc"}
MODE_LABEL = {
    "pure_pursuit": "pure pursuit  (기하 추종, look-ahead)",
    "comma_mpc": "comma MPC     (원본 lateral_mpc_lib, acados)",
    "alpasim_mpc": "alpasim MPC   (LinearMPC 이식, ADMM QP)",
}


def request(sock, addr, payload, timeout=REQUEST_TIMEOUT_S):
    """명령 1개 전송 후 상태 응답 1개 수신. 응답 없으면 None."""
    try:
        sock.sendto(json.dumps(payload).encode("utf-8"), addr)
    except OSError as e:
        return {"error": f"send 실패: {e}"}
    ready, _, _ = select.select([sock], [], [], timeout)
    if not ready:
        return None
    try:
        data, _ = sock.recvfrom(8192)
        return json.loads(data.decode("utf-8"))
    except (OSError, ValueError) as e:
        return {"error": f"응답 파싱 실패: {e}"}


def fmt_kappa(v):
    return "     —" if v is None else f"{v:+.5f}"


def fmt_num(st, key, fmt, default="—"):
    v = st.get(key)
    return default if v is None else format(v, fmt)


def render(st, addr, note=""):
    """상태 dict → 화면 문자열. 고정 높이라 커서를 올려 제자리 갱신한다."""
    w = 64
    lines = [f"┌─ udp_bridge 횡제어기 ─{'─' * (w - 23)}┐"]
    if st is None:
        lines += [
            f"  대상 {addr[0]}:{addr[1]}   상태: 응답 없음",
            "",
            "  udp_bridge 가 떠 있지 않습니다. 온로드 상태인지 확인하세요",
            "  (manager 의 udp_bridge 는 only_onroad 프로세스입니다).",
            "", "", "",
        ]
    elif "error" in st:
        lines += [f"  오류: {st['error']}", "", "", "", "", "", ""]
    else:
        mode = st.get("mode", "?")
        modes = st.get("modes", list(MODE_KEYS.values()))
        lines.append(f"  대상 {addr[0]}:{addr[1]}   frame {fmt_num(st, 'frame', 'd')}"
                     f"   loop {fmt_num(st, 'loop_ms', '.1f')}ms / 50ms"
                     f"   {'engaged' if st.get('engaged') else 'disengaged'}")
        lines.append("")
        for i, m in enumerate(modes, start=1):
            mark = "◀ 활성" if m == mode else "      "
            lines.append(f"   [{i}] {MODE_LABEL.get(m, m):<44} {mark}")
        lines.append("")
        if st.get("has_path"):
            lines.append(f"   κ 지령 {fmt_kappa(st.get('kappa_cmd'))}"
                         f"   v {fmt_num(st, 'v_ego', '.2f')} m/s"
                         f"   cte {fmt_num(st, 'cte_m', '+.3f')} m"
                         f"   solve {fmt_num(st, 'solve_ms', '.1f')}ms")
            lines.append(f"   κ  pp  {fmt_kappa(st.get('kappa_pp'))}"
                         f"   comma {fmt_kappa(st.get('kappa_comma'))}"
                         f"   alpasim {fmt_kappa(st.get('kappa_alpasim'))}")
        else:
            lines.append(f"   경로 수신 대기 중 (수신 {fmt_num(st, 'pkts', 'd')} 패킷)"
                         f"   v {fmt_num(st, 'v_ego', '.2f')} m/s")
            lines.append("")
        lines.append(f"   compare {'on ' if st.get('compare') else 'off'}"
                     f"   연속실패 {fmt_num(st, 'fail_streak', 'd', '0')}"
                     f"   전환 {fmt_num(st, 'switch_count', 'd', '0')}회"
                     f"   수신 {fmt_num(st, 'pkts', 'd', '0')} 패킷")
    lines.append(f"└{'─' * (w - 1)}┘")
    lines.append("  [1/2/3] 선택   [m] 순환   [c] compare   [x] 리셋   [q] 종료")
    lines.append(f"  {note}")
    return lines


def interactive(sock, addr):
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    prev_lines = 0
    note = ""
    st = request(sock, addr, {"cmd": "get"})
    try:
        tty.setcbreak(fd)
        sys.stdout.write("\033[?25l")     # 커서 숨김
        while True:
            lines = render(st, addr, note)
            if prev_lines:
                sys.stdout.write(f"\033[{prev_lines}A")
            sys.stdout.write("".join(f"\033[2K{ln}\n" for ln in lines))
            sys.stdout.flush()
            prev_lines = len(lines)

            ready, _, _ = select.select([sys.stdin], [], [], REFRESH_S)
            if not ready:
                st = request(sock, addr, {"cmd": "get"})   # 주기 갱신
                continue

            key = sys.stdin.read(1)
            if key in ("q", "Q", "\x03", "\x04"):
                return
            elif key in MODE_KEYS:
                st = request(sock, addr, {"cmd": "set", "mode": MODE_KEYS[key]})
            elif key in ("m", "M"):
                st = request(sock, addr, {"cmd": "cycle"})
            elif key in ("c", "C"):
                st = request(sock, addr, {"cmd": "compare"})
            elif key in ("x", "X"):
                st = request(sock, addr, {"cmd": "reset"})
            else:
                note = f"알 수 없는 키 {key!r}"
                continue
            note = "" if st is None else str(st.get("msg", ""))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        sys.stdout.write("\033[?25h\n")   # 커서 복구
        sys.stdout.flush()


def one_shot(sock, addr, payload):
    st = request(sock, addr, payload)
    if st is None:
        print("응답 없음 — udp_bridge 가 떠 있지 않습니다 (only_onroad 프로세스).",
              file=sys.stderr)
        return 1
    if "error" in st:
        print(st["error"], file=sys.stderr)
        return 1
    print("\n".join(render(st, addr)[:-1]))
    return 0 if st.get("ok", True) else 1


def main():
    ap = argparse.ArgumentParser(description="udp_bridge 횡제어기 실시간 전환")
    ap.add_argument("--host", default="127.0.0.1", help="udp_bridge 호스트 (기본 127.0.0.1)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--set", dest="set_mode", choices=sorted(MODE_KEYS.values()),
                   help="제어기 지정 후 종료")
    g.add_argument("--cycle", action="store_true", help="다음 제어기로 순환 후 종료")
    g.add_argument("--compare", choices=("on", "off"), help="compare 토글 후 종료")
    g.add_argument("--reset", action="store_true", help="제어기 상태 리셋 후 종료")
    g.add_argument("--status", action="store_true", help="현재 상태만 출력")
    args = ap.parse_args()

    addr = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    if args.set_mode:
        return one_shot(sock, addr, {"cmd": "set", "mode": args.set_mode})
    if args.cycle:
        return one_shot(sock, addr, {"cmd": "cycle"})
    if args.compare:
        return one_shot(sock, addr, {"cmd": "compare", "on": args.compare == "on"})
    if args.reset:
        return one_shot(sock, addr, {"cmd": "reset"})
    if args.status or not (sys.stdin.isatty() and sys.stdout.isatty()):
        if not args.status:
            print("TTY 가 아니라 상태만 출력합니다 (전환은 --set / --cycle 사용).",
                  file=sys.stderr)
        return one_shot(sock, addr, {"cmd": "get"})

    try:
        interactive(sock, addr)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
