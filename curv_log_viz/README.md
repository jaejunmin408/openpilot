# curv_log_viz — udp_bridge curvature 로그 뷰어

`logs/udp_bridge_debug_*.log` (engage 할 때마다 새로 생기는 디버그 로그)를
브라우저에서 열어 **수신 경로(XY)** 와 **그 경로에서 뽑힌 curvature 들** 을 같이 보는 도구.

경로는 ~10Hz 로 들어오는데 제어 루프는 20Hz 라, 한 경로가 보통 5~7회(패킷이 밀리면
20회 넘게) 재사용된다. 로그의 `CURV ... pkts=<N>` 이 그때 쓰인 경로 번호(`PATH recv #<N>`)라
이 둘을 묶어 "같은 경로 = 한 세트" 로 보여준다.

## 실행

    cd /data/openpilot
    python3 curv_log_viz/curv_log_server.py            # 기본 포트 8081, logs/ 를 봄
    python3 curv_log_viz/curv_log_server.py --dir /data/media/0/logs --port 8082

PC 브라우저에서 `http://<기기IP>:8081` 접속 (예: `http://192.168.43.1:8081`).
서버 없이 `curv_log.html` 을 그냥 열고 로그 파일을 **드래그&드롭** 해도 된다.
openpilot venv 필요 없음 (stdlib 만 사용).

## 화면

| 영역 | 내용 |
|---|---|
| 상단 요약 | 파일/길이/경로 수/κ 계산 수, 선택 series 의 평균·\|평균\|·σ·범위 |
| 왼쪽 목록 | 경로 번호 `#id`, 시각, 점 수 `N`, 그 경로로 계산한 횟수 `×n`, κ 평균 |
| 가운데 XY | ego frame 경로(x=전방↑, y=좌측←) + 그 경로에서 나온 κ 들의 원호(계산 순서대로 cyan→orange) + goal 점 |
| 오른쪽 위 | κ 시계열 (세로 띠 하나 = 경로 하나). 클릭하면 그 경로로 이동 |
| 오른쪽 가운데 | κ 히스토그램 + 평균/중앙값 선. 아래 눈금 = 선택 경로의 계산들 |
| 오른쪽 아래 | series 별 통계 (n / 평균 / \|평균\| / σ / 중앙값 / p5 / p95 / min / max) |
| 하단 표 | 선택 경로의 계산 하나하나: Δt, frame, mode, v_ego, κ(sm/comma/pp/alpasim), i_goal, L_d, solve, cte |

`y확대` 는 횡방향만 늘려 보는 배율(기본 auto). `1×` 가 실제 비율이고, 경로와 원호가
같은 배율로 늘어나므로 둘의 비교는 확대해도 그대로 유효하다.

단축키: `←/→` 경로 이동, `↑/↓` 10개씩, `space` 자동재생.
`κ 원호` 를 끄면 경로만, 표 행에 마우스를 올리면 그 계산의 원호와 goal 점이 강조된다.

## 로그 형식 (파서가 기대하는 것)

    [t=6258.059] PATH recv #7612 N=64 pts=[(0.00,-0.00), (0.43,-0.00), ...]
    [t=6257.818] CURV frame=123918 v_ego=4.58 mode=comma_mpc comma=-0.0082(ok) \
        alpasim=+nan(-) pp=-0.0057 sm=-0.0082 L_d_eff=8.41 i_goal=18 solve=1.9ms \
        cte=-0.00 pkts=7611

좌표는 openpilot body frame (x=전방, y=좌측이 +), κ>0 = 좌회전.
프로세스가 죽어 마지막 줄이 잘려도 파싱되며, 잘린 줄 수는 상단에 표시된다.

### `alpa_action` (raw action 직결) 모드

외부 publisher 가 x,y 경로 대신 `raw_action`(accel/curvature 시계열)을 보내고
제어기(pure pursuit/MPC)를 우회하는 소스. 이 모드도 **같은 PATH/CURV 형식으로**
찍히므로 이 뷰어를 그대로 쓴다. 다른 점만:

    [t=..] ACTION recv #12 N=64 dt=0.100s horizon=6.40s infer=0.120s int_v=4.12 \
        curv=[...] raw_a=[...] raw_v=[...]
    [t=..] PATH recv #12 src=alpa_action N=33 pts=[(0.00,0.00), ...]
    [t=..] CURV frame=. v_ego=. src=alpa_action mode=bypass raw=+0.00300 sm=+0.00300 \
        a=+0.35 idx=3/63 t_query=0.300s lead=+0.180s lat_delay=0.150s age=0.120s \
        raw_a=+0.47 raw_v=4.15 path=12 pkts=812

- `ACTION` 줄 — 받은 시계열 **원본**. `curv` 가 6.4s/0.1s = 64점 곡률, `raw_a` 는
  종가속도(raw_accel_mps2), `raw_v` 는 속도(publisher 필드명은 `accel_mps2`).
  이 줄은 뷰어가 무시한다 (원본 숫자 확인용).
- `PATH` 줄 — x,y 를 안 받으므로, **명령 곡률을 `int_v` 속도로 적분한 경로** 를
  남긴다. 기기 화면(modelV2)에 뜨는 것과 같은 경로다. 등속 가정이라 속도가
  변하면 같은 곡률에도 모양이 바뀌는 점만 유의.
- `CURV` 줄 — `sm` 이 실제로 실어 보낸 curvature, `raw` 가 clip·smoothing 전 원값.
  제어기를 안 쓰므로 `comma`/`pp`/`alpasim`/`L_d_eff`/`i_goal`/`cte` 는 없다
  (뷰에서 빈칸).
  - `t_query` — 시계열에서 읽은 시점. 기본은 **패킷 나이 + `lat_delay`** 이고
    `idx = t_query/dt`. `RAW_ACTION_FIXED_T_S` 에 숫자를 넣으면 패킷 `t=0` 기준
    고정 오프셋으로 바뀐다(예: 0.3 → dt 0.1s 에서 항상 idx 3).
  - `lead = t_query - age` — **지금보다 얼마나 앞선 값인가**. 여기가 핵심 지표다.
    기본 설정에서는 lead 가 곧 `lat_delay` 다. 0 근처면 `liveDelay` 가 아직 추정
    전이라 지연 보상이 안 되고 있다는 뜻. 고정 오프셋을 쓸 때는 패킷이 늙은 만큼
    lead 가 깎이므로(age 는 publisher 추론시간 + 전송·loop 지연) 음수로 가면
    과거 값을 싣고 있다는 뜻이다.
  - `lat_delay` — `liveDelay.lateralDelay` (기본 설정에서 인덱스 계산에 쓰인다).
- 제어가 멈추면 `... mode=bypass STOP reason='stale 0.71s' ...` 줄이 남는다.

즉 XY 패널·κ 시계열·히스토그램·통계는 그대로 보이고, 제어기 비교 series 3개만
비어 있다.
