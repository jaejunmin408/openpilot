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
