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
  p           reference path 소스 순환
              alpamayo(외부 UDP x,y 경로) → comma_model(comma 비전모델 자체 경로)
              → alpa_action(외부 raw_action 직결)
              제어기 선택과 직교한다 — 같은 제어기로 여러 경로를 번갈아 비교할 수 있다.
  a           raw action 직결 ↔ 직전 경로소스 1키 토글
              alpa_action 은 x,y 경로 대신 accel/curvature 시계열(6.4s@0.1s)을 받아
              그 curvature 를 그대로 desiredCurvature 로 실어 latcontrol_torque 로
              넘긴다. **pure pursuit / MPC 가 전혀 개입하지 않는다**(우회).
              p 로 순환하면 비교 대상 사이에 세 번째 소스가 끼므로, A/B 는 이 키로.
  c           compare 토글 (비활성 MPC 도 매 loop 계산해 로그에 남김)
  x           활성 제어기 내부 상태 리셋
  q / Ctrl-C  종료

TTY 가 아니거나(파이프·스크립트) 일회성 조작만 필요하면 인자 모드를 쓴다:
  python lat_ctl.py --set comma_mpc
  python lat_ctl.py --cycle
  python lat_ctl.py --source alpa_action
  python lat_ctl.py --toggle-source
  python lat_ctl.py --raw-toggle
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
SOURCES = ("alpamayo", "comma_model", "alpa_action")
SOURCE_LABEL = {
    "alpamayo": "alpamayo     (외부 publisher 의 x,y 경로)",
    "comma_model": "comma_model  (comma 비전모델이 직접 뽑은 예측경로)",
    "alpa_action": "alpa_action  (외부 raw_action 직결 — 제어기 우회)",
}
# 제어기를 우회하는 소스 (udp_bridge.BYPASS_SOURCES 와 같은 값). 상태 응답에
# bypass_sources 가 오면 그걸 쓰고, 구버전 udp_bridge 면 이 기본값으로 표시한다.
BYPASS_SOURCES = ("alpa_action",)
SOURCE_KEY = {"alpa_action": "[a]"}      # 그 외는 [p]
BODY_LINES = 15                          # 테두리 내부 줄 수 (짧은 화면도 여기까지 패딩)


def model_status_line(st):
    """comma 모델경로 상태 한 줄. 소스가 comma_model 일 때 제어 가능 여부가 여기서 보인다."""
    if not st.get("model_alive"):
        return "모델경로: 수신 없음 (modeld 미발행 — 온로드/카메라 확인)"
    line = (f"모델경로: age {fmt_num(st, 'model_age', '.3f')}s"
            f"  N {fmt_num(st, 'model_n', 'd')}"
            f"  전방 {fmt_num(st, 'model_range_m', '.1f')}m"
            f"  κ {fmt_kappa(st.get('model_curv'))}")
    reason = st.get("model_reason") or ""
    return line + (f"  ⚠ {reason}" if reason else "  ok")


def action_status_line(st):
    """raw_action 상태 한 줄. 소스가 alpa_action 일 때 제어 가능 여부가 여기서 보인다.

    t = 시계열에서 읽는 시점(패킷 t=0 기준, 기본 고정 0.3s), idx = t/dt 로 고른 점,
    lead = t - age = "지금보다 얼마나 앞선 값인가". age 는 publisher 추론시간 + 전송
    지연이라, 고정 t 를 쓰면 패킷이 늙은 만큼 lead 가 깎인다 — lead 가 0 이하로 가면
    선행이 없거나(⚠) 과거 값을 싣고 있다는 뜻이므로 t 를 키워야 한다.
    """
    if not st.get("action_alive"):
        return "raw action: 수신 없음 (publisher 가 raw_action 을 안 보냄)"
    n = st.get("action_n")
    idx = st.get("action_idx")
    # a/v 는 publisher 필드 raw_accel_mps2 / accel_mps2 다. 후자는 이름과 달리 속도(m/s).
    line = (f"raw action: age {fmt_num(st, 'action_age', '.3f')}s"
            f"  t {fmt_num(st, 'action_t_query', '.2f')}s"
            f"  lead {fmt_num(st, 'action_lead', '+.3f')}s"
            f"  idx {'—' if idx is None else idx}/{'—' if not n else n - 1}"
            f"  κ {fmt_kappa(st.get('action_curv'))}"
            f"  a {fmt_num(st, 'action_accel', '+.2f')}"
            f"  v {fmt_num(st, 'action_v_ref', '.2f')}")
    reason = st.get("action_reason") or ""
    return line + (f"  ⚠ {reason}" if reason else "  ok")


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
    body = []
    if st is None:
        body += [
            f"  대상 {addr[0]}:{addr[1]}   상태: 응답 없음",
            "",
            "  udp_bridge 가 떠 있지 않습니다. 온로드 상태인지 확인하세요",
            "  (manager 의 udp_bridge 는 only_onroad 프로세스입니다).",
        ]
    elif "error" in st:
        body += [f"  오류: {st['error']}"]
    else:
        mode = st.get("mode", "?")
        modes = st.get("modes", list(MODE_KEYS.values()))
        # 우회 소스에서는 제어기 선택이 무의미하다 (curvature 를 그대로 쓴다)
        bypass = bool(st.get("bypass", st.get("source") in BYPASS_SOURCES))
        body.append(f"  대상 {addr[0]}:{addr[1]}   frame {fmt_num(st, 'frame', 'd')}"
                    f"   loop {fmt_num(st, 'loop_ms', '.1f')}ms / 50ms"
                    f"   {'engaged' if st.get('engaged') else 'disengaged'}")
        body.append("")
        for i, m in enumerate(modes, start=1):
            if m != mode:
                mark = "      "
            else:
                mark = "◀ 우회 중" if bypass else "◀ 활성"
            body.append(f"   [{i}] {MODE_LABEL.get(m, m):<44} {mark}")
        body.append("")
        # ── reference path 소스 ([p] 순환 / [a] raw action 토글) ──
        src = st.get("source", "?")
        bypass_srcs = tuple(st.get("bypass_sources", BYPASS_SOURCES))
        for sname in st.get("sources", list(SOURCES)):
            mark = "◀ 활성" if sname == src else "      "
            bullet = "●" if sname == src else "○"
            key = SOURCE_KEY.get(sname, "[p]") if sname in bypass_srcs else "[p]"
            body.append(f"   {key} {bullet} {SOURCE_LABEL.get(sname, sname):<42} {mark}")
        body.append(f"       {model_status_line(st)}")
        body.append(f"       {action_status_line(st)}")
        body.append("")
        if st.get("has_path"):
            body.append(f"   κ 지령 {fmt_kappa(st.get('kappa_cmd'))}"
                        f"   v {fmt_num(st, 'v_ego', '.2f')} m/s"
                        f"   cte {fmt_num(st, 'cte_m', '+.3f')} m"
                        f"   solve {fmt_num(st, 'solve_ms', '.1f')}ms")
            if bypass:
                # 제어기 출력이 없다 — 대신 그대로 실어 보낸 raw 값을 보여준다
                body.append(f"   raw κ {fmt_kappa(st.get('action_curv'))}"
                            f"   raw a {fmt_num(st, 'action_accel', '+.2f')} m/s²"
                            f"   raw v {fmt_num(st, 'action_v_ref', '.2f')} m/s"
                            f"   (pure pursuit / MPC 미사용)")
            else:
                body.append(f"   κ  pp  {fmt_kappa(st.get('kappa_pp'))}"
                            f"   comma {fmt_kappa(st.get('kappa_comma'))}"
                            f"   alpasim {fmt_kappa(st.get('kappa_alpasim'))}")
        else:
            what = "raw action" if bypass else "경로"
            body.append(f"   {what} 수신 대기 중 (경로 {fmt_num(st, 'pkts', 'd')} / "
                        f"action {fmt_num(st, 'action_pkts', 'd')} 패킷)"
                        f"   v {fmt_num(st, 'v_ego', '.2f')} m/s")
            body.append("")
        body.append(f"   compare {'on ' if st.get('compare') else 'off'}"
                    f"   연속실패 {fmt_num(st, 'fail_streak', 'd', '0')}"
                    f"   전환 {fmt_num(st, 'switch_count', 'd', '0')}/"
                    f"{fmt_num(st, 'source_switch_count', 'd', '0')}회"
                    f"   경로 #{fmt_num(st, 'path_seq', 'd', '0')}"
                    f"   수신 {fmt_num(st, 'pkts', 'd', '0')}/"
                    f"{fmt_num(st, 'action_pkts', 'd', '0')} 패킷")
    # 화면 높이를 고정한다 — 짧게 그리면 이전 프레임의 아래쪽 줄이 남는다
    body += [""] * max(0, BODY_LINES - len(body))
    return ([f"┌─ udp_bridge 횡제어 / 경로소스 ─{'─' * (w - 32)}┐"] + body
            + [f"└{'─' * (w - 1)}┘",
               "  [1/2/3] 제어기  [m] 순환  [p] 경로소스  [a] raw action  "
               "[c] compare  [x] 리셋  [q] 종료",
               f"  {note}"])


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
            elif key in ("p", "P"):
                st = request(sock, addr, {"cmd": "source"})
            elif key in ("a", "A"):
                st = request(sock, addr, {"cmd": "raw_toggle"})
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
    g.add_argument("--source", choices=SOURCES, help="reference path 소스 지정 후 종료")
    g.add_argument("--toggle-source", action="store_true", dest="toggle_source",
                   help="reference path 소스 순환 후 종료")
    g.add_argument("--raw-toggle", action="store_true", dest="raw_toggle",
                   help="raw action 직결 ↔ 직전 경로소스 토글 후 종료")
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
    if args.source:
        return one_shot(sock, addr, {"cmd": "source", "src": args.source})
    if args.toggle_source:
        return one_shot(sock, addr, {"cmd": "source"})
    if args.raw_toggle:
        return one_shot(sock, addr, {"cmd": "raw_toggle"})
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
