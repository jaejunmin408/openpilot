"""
alpasim LinearMPC ↔ openpilot 연결 계층.

udp_bridge 가 받는 것은 "ego frame 공간 path"(타임스탬프 없음)이고,
alpasim MPC 가 원하는 것은 "dt_mpc 간격으로 샘플된 시간축 reference"다.
그 변환과 상태/부호 변환, 그리고 δ → desiredCurvature 환산을 여기서 한다.

부호 규약 (전부 여기서 통일):
  path (udp_bridge 가 이미 변환) : x=전방, y=좌(+)        = alpasim / openpilot body frame
  livePose device frame          : x=전방, y=우(+), z=하(+)
      → v_y(좌+)      = -velocityDevice.y
      → yaw_rate(CCW+) = -angularVelocityDevice.z
  carState.steeringAngleDeg      : +가 우회전
      → δ(좌+) = -radians(steeringAngleDeg - angleOffsetDeg) / steerRatio
  desiredCurvature               : +가 좌회전  → alpasim δ(좌+) 와 같은 방향, 부호 반전 없음
      (controlsd.py:79 의 `curvature = -VM.calc_curvature(sa, ...)` 로 확인)
"""
import math
from dataclasses import dataclass, field

import numpy as np

from openpilot.selfdrive.controls.lib.alpasim_mpc.linear_mpc import (
    DEFAULT_DT_MPC,
    DEFAULT_N_HORIZON,
    LinearMPC,
    MPCGains,
)
from openpilot.selfdrive.controls.lib.alpasim_mpc.vehicle_model import VehicleParameters

# 이보다 가까운 연속 path 점은 heading 정보가 없다고 보고 버린다 [m]
MIN_PATH_POINT_SPACING_M = 1e-3


@dataclass
class EgoState:
    """MPC 상태 구성에 필요한 자차 상태. 부호는 전부 좌(+)/CCW(+) 규약."""

    v_ego: float = 0.0           # 종속도 (rig) [m/s]
    v_lat: float = 0.0           # 횡속도 (rig), 좌(+) [m/s]
    yaw_rate: float = 0.0        # yaw rate, CCW(+) [rad/s]
    steering_angle: float = 0.0  # 전륜 조향각, 좌(+) [rad]
    accel_long: float = 0.0      # 종가속 [m/s²]

    @classmethod
    def from_messages(cls, car_state, live_pose=None, steer_ratio=None,
                      angle_offset_deg=0.0):
        """cereal carState / livePose → EgoState (부호 변환 포함).

        Args:
            car_state: cereal carState
            live_pose: cereal livePose (없으면 횡속도/yaw rate 를 0 으로 둔다)
            steer_ratio: 조향비. None 이면 steeringAngleDeg 를 그대로 전륜각으로
                쓸 수 없으므로 조향 상태를 0 으로 둔다.
            angle_offset_deg: liveParameters.angleOffsetDeg
        """
        v_ego = max(float(car_state.vEgo), 0.0)

        v_lat = 0.0
        yaw_rate = 0.0
        accel_long = 0.0
        if live_pose is not None:
            # device frame: y=우(+), z=하(+) → 내부 규약(좌+/CCW+)으로 반전
            v_lat = -float(live_pose.velocityDevice.y)
            yaw_rate = -float(live_pose.angularVelocityDevice.z)
            accel_long = float(live_pose.accelerationDevice.x)

        steering_angle = 0.0
        if steer_ratio is not None and steer_ratio > 0.1:
            # openpilot steeringAngleDeg 는 +가 우회전 → 좌(+) 규약으로 반전
            steering_angle = -math.radians(float(car_state.steeringAngleDeg) - float(angle_offset_deg)) / float(steer_ratio)

        return cls(
            v_ego=v_ego,
            v_lat=v_lat,
            yaw_rate=yaw_rate,
            steering_angle=steering_angle,
            accel_long=accel_long,
        )


@dataclass
class TrackerResult:
    """MPCPathTracker.update() 결과."""

    ok: bool
    curvature: float = 0.0        # openpilot desiredCurvature 규약 (좌+) [1/m]
    steering_cmd: float = 0.0     # MPC 원출력 δ [rad], 좌(+)
    accel_cmd: float = 0.0        # MPC 원출력 a [m/s²]
    status: str = "not_run"
    solve_time_ms: float = 0.0
    iters: int = 0
    lat_error_m: float = 0.0      # ego → path 수직거리, 좌(+)
    s_proj_m: float = 0.0         # ego 를 path 에 투영한 arc-length
    ref_reach_m: float = 0.0      # reference 가 커버한 arc-length (s_proj 로부터)
    ref_extrapolated_m: float = 0.0   # path 끝을 넘어 외삽한 거리
    curvature_factor: float = 0.0     # δ → κ 환산 계수 [1/m/rad]
    x_ref: np.ndarray | None = field(default=None, repr=False)
    predicted_states: np.ndarray | None = field(default=None, repr=False)
    speed_profile: np.ndarray | None = field(default=None, repr=False)


def path_arclength_and_heading(path_x, path_y):
    """공간 path → (s, x, y, psi). 중복점 제거 + heading unwrap.

    Returns:
        s: 누적 arc-length (M,)
        x, y: 중복 제거된 좌표 (M,)
        psi: 접선 방향 [rad], unwrap 됨 (M,)
    """
    x = np.asarray(path_x, dtype=np.float64).reshape(-1)
    y = np.asarray(path_y, dtype=np.float64).reshape(-1)
    if x.shape[0] != y.shape[0]:
        raise ValueError("path_x / path_y length mismatch")
    if x.shape[0] < 2:
        raise ValueError("path must have >= 2 points")

    ds = np.hypot(np.diff(x), np.diff(y))
    # 사실상 겹치는 점은 제거한다. 완전 중복이면 heading 이 0/0 이고, 아주
    # 가까우면(1 mm 등) np.gradient 가 미분값을 폭발시켜 psi 가 쓰레기가 된다.
    # 20 m 스케일 path 에서 1 mm 이내 두 점은 heading 정보를 담고 있지 않다.
    keep = np.concatenate([[True], ds > MIN_PATH_POINT_SPACING_M])
    x = x[keep]
    y = y[keep]
    if x.shape[0] < 2:
        raise ValueError("path degenerate after removing duplicate points")

    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    # 양 끝점도 2차 정확도로. edge_order=1(기본)이면 끝점 heading 이 반 스텝만큼,
    # 즉 κ·ds/2 정도 틀린다 — 마지막 접선은 path 끝을 넘는 외삽에 쓰이므로 중요하다.
    edge_order = 2 if x.shape[0] >= 3 else 1
    psi = np.unwrap(np.arctan2(np.gradient(y, s, edge_order=edge_order),
                               np.gradient(x, s, edge_order=edge_order)))
    return s, x, y, psi


def project_ego_onto_path(s, x, y):
    """원점(ego)을 path 폴리라인에 투영. 세그먼트 단위 투영으로 정확도 확보.

    Returns:
        s_proj: 투영점의 arc-length
        lat_error: ego 에서 본 path 의 횡방향 위치, 좌(+). path 가 왼쪽에 있으면 +.
    """
    x0, y0 = x[:-1], y[:-1]
    dx, dy = np.diff(x), np.diff(y)
    seg_len2 = dx * dx + dy * dy

    # 각 세그먼트에서 원점의 정규화 투영 파라미터 t ∈ [0, 1]
    t = np.clip(-(x0 * dx + y0 * dy) / seg_len2, 0.0, 1.0)
    px = x0 + t * dx
    py = y0 + t * dy
    d2 = px * px + py * py

    i = int(np.argmin(d2))
    s_proj = float(s[i] + t[i] * np.hypot(dx[i], dy[i]))
    # 투영점의 y 부호가 path 의 횡방향 위치 (ego 는 원점이므로 그대로)
    lat_error = float(py[i])
    return s_proj, lat_error


def speed_profile(v0, n_steps, dt, target_speed, lon_kp, accel_min, accel_max):
    """종방향 P 제어기를 그대로 앞으로 굴려 예측 속도 프로파일 생성.

    udp_bridge 의 longitudinal_accel() 과 같은 식이라, reference 의 종방향
    위치가 실제 종제어와 일치한다 → MPC 의 x 오차가 인위적으로 커지지 않는다.
    """
    v = np.empty(n_steps + 1)
    v[0] = max(float(v0), 0.0)
    for k in range(n_steps):
        a = np.clip(lon_kp * (target_speed - v[k]), accel_min, accel_max)
        v[k + 1] = max(v[k] + a * dt, 0.0)
    return v


def build_reference(path_x, path_y, v0, n_horizon, dt_mpc, target_speed,
                    lon_kp, accel_min, accel_max, nx=8,
                    align_to_closest=True, extrapolate=True):
    """공간 path + 예측 속도 프로파일 → MPC reference (N+1, nx).

    alpasim 원본은 timestamp 로 Trajectory 를 보간하고 범위를 clip 했다.
    여기서는 path 에 시간 정보가 없으므로 arc-length 로 파라미터화한다:

        s_k = s_proj + ∫₀^{k·dt} v dt          (v = 예측 속도 프로파일)
        x_ref[k] = path(s_k),  yaw_ref[k] = path 접선각(s_k)

    align_to_closest=True 면 s=0 을 "ego 를 path 에 투영한 점"으로 잡는다.
    이러면 종방향 reference 오차가 0 에서 시작하므로(횡오차는 그대로 남는다)
    종제어를 외부 P 제어기에 맡긴 상태에서 MPC 가 x 오차를 잡으려고
    조향까지 흔드는 일이 없어진다.

    Returns:
        x_ref (N+1, nx), 진단 dict
    """
    s, px, py, psi = path_arclength_and_heading(path_x, path_y)

    if align_to_closest:
        s_proj, lat_error = project_ego_onto_path(s, px, py)
    else:
        s_proj, lat_error = 0.0, float(py[0])

    v = speed_profile(v0, n_horizon, dt_mpc, target_speed, lon_kp, accel_min, accel_max)
    # s_k: 속도 프로파일 사다리꼴 적분
    ds = 0.5 * (v[:-1] + v[1:]) * dt_mpc
    s_ref = s_proj + np.concatenate([[0.0], np.cumsum(ds)])

    s_end = float(s[-1])
    if extrapolate and s_ref[-1] > s_end:
        # path 끝을 넘는 구간은 마지막 접선 방향으로 직선 외삽.
        # np.interp 의 기본 동작(끝점 hold)은 "여기서 멈춰라"는 가짜 목표를 만든다.
        over = np.maximum(s_ref - s_end, 0.0)
        s_clamped = np.minimum(s_ref, s_end)
        x_ref_pos = np.interp(s_clamped, s, px) + over * math.cos(psi[-1])
        y_ref_pos = np.interp(s_clamped, s, py) + over * math.sin(psi[-1])
        extrapolated = float(over[-1])
    else:
        x_ref_pos = np.interp(s_ref, s, px)
        y_ref_pos = np.interp(s_ref, s, py)
        extrapolated = 0.0

    yaw_ref = np.interp(s_ref, s, psi)

    x_ref = np.zeros((n_horizon + 1, nx))
    x_ref[:, LinearMPC.IX] = x_ref_pos
    x_ref[:, LinearMPC.IY] = y_ref_pos
    x_ref[:, LinearMPC.IYAW] = yaw_ref

    diag = {
        "s_proj_m": s_proj,
        "lat_error_m": lat_error,
        "ref_reach_m": float(s_ref[-1] - s_proj),
        "ref_extrapolated_m": extrapolated,
        "speed_profile": v,
        "path_length_m": s_end,
    }
    return x_ref, diag


class SolverFailurePolicy:
    """MPC 해가 실패했을 때 어떤 curvature 를 쓸지 결정한다.

    alpasim 원본은 QP 실패 시 u=0 을 낸다 — 조향이 0 으로 스냅한다는 뜻이다.
    시뮬에서는 무해하지만 실차에서 선회 중에 그러면 위험하다. 그래서:

      성공                        → MPC 값
      짧은 실패                   → 직전 curvature 유지
      max_consecutive_fails 이상  → 폴백값(pure pursuit) 사용

    select() 는 (curvature, event) 를 돌려주고 event 는 로깅용으로
    None / "recovered" / "fallback_entered" 중 하나다.
    """

    def __init__(self, max_consecutive_fails=10, use_fallback=True):
        self.max_consecutive_fails = int(max_consecutive_fails)
        self.use_fallback = bool(use_fallback)
        self.fail_streak = 0
        self.fallback_active = False

    def reset(self):
        self.fail_streak = 0
        self.fallback_active = False

    def select(self, mpc_ok, mpc_curvature, prev_curvature, fallback_curvature):
        if mpc_ok:
            event = "recovered" if self.fail_streak else None
            self.fail_streak = 0
            self.fallback_active = False
            return float(mpc_curvature), event

        self.fail_streak += 1
        if self.use_fallback and self.fail_streak >= self.max_consecutive_fails:
            event = None if self.fallback_active else "fallback_entered"
            self.fallback_active = True
            return float(fallback_curvature), event
        return float(prev_curvature), None


class MPCPathTracker:
    """alpasim LinearMPC 로 ego frame 공간 path 를 추종해 curvature 를 낸다.

    출력은 openpilot `desiredCurvature` 규약(좌+, 1/m). MPC 자체는 전륜
    조향각 δ 를 내므로 κ = curvature_factor(v) · δ 로 환산한다.
    curvature_factor 는 opendbc VehicleModel 의 것을 쓴다(언더스티어 반영);
    VehicleModel 이 없으면 kinematic 1/wheelbase 로 대체한다.
    """

    def __init__(self, vehicle_params=None, gains=None,
                 n_horizon=DEFAULT_N_HORIZON, dt_mpc=DEFAULT_DT_MPC,
                 target_speed=4.1667, lon_kp=0.3,
                 accel_min=-3.5, accel_max=2.0,
                 curv_limit=0.2, vm=None,
                 align_to_closest=True, extrapolate=True):
        self._params = vehicle_params or VehicleParameters()
        self.mpc = LinearMPC(
            vehicle_params=self._params,
            gains=gains or MPCGains(),
            n_horizon=n_horizon,
            dt_mpc=dt_mpc,
        )
        self.target_speed = target_speed
        self.lon_kp = lon_kp
        self.accel_min = accel_min
        self.accel_max = accel_max
        self.curv_limit = curv_limit
        self.align_to_closest = align_to_closest
        self.extrapolate = extrapolate
        self._vm = vm

    def reset(self):
        self.mpc.reset()

    @property
    def horizon_seconds(self):
        return self.mpc.horizon_seconds

    def reach_at_speed(self, v):
        """참고용: 속도 v 에서 horizon 이 실제로 내다보는 거리 [m]."""
        return self.mpc.horizon_seconds * max(float(v), 0.0)

    def curvature_factor(self, v_ego):
        """δ(전륜 조향각) → 곡률 환산 계수 [1/m/rad]."""
        if self._vm is not None:
            try:
                return float(self._vm.curvature_factor(max(float(v_ego), 0.0)))
            except Exception:
                pass
        return 1.0 / self._params.wheelbase

    def build_state(self, ego: EgoState):
        """EgoState → MPC 상태 (8,). ego frame 이므로 위치/자세는 0."""
        state = np.zeros(LinearMPC.NX)
        state[LinearMPC.IVX] = ego.v_ego
        # rig 횡속도 → CoG 횡속도 (alpasim system.py _dynamic_state_to_cg_velocity)
        state[LinearMPC.IVY] = ego.v_lat + self._params.l_rig_to_cg * ego.yaw_rate
        state[LinearMPC.IYAW_RATE] = ego.yaw_rate
        state[LinearMPC.ISTEERING] = ego.steering_angle
        state[LinearMPC.IACCEL] = ego.accel_long
        return self.mpc.clip_state_to_bounds(state)

    def update(self, path_x, path_y, ego: EgoState):
        """path 추종 1 스텝.

        Args:
            path_x, path_y: ego frame path (x=전방, y=좌(+))
            ego: 자차 상태

        Returns:
            TrackerResult
        """
        try:
            x_ref, diag = build_reference(
                path_x, path_y, ego.v_ego,
                self.mpc.n_horizon, self.mpc.dt_mpc,
                self.target_speed, self.lon_kp, self.accel_min, self.accel_max,
                nx=LinearMPC.NX,
                align_to_closest=self.align_to_closest,
                extrapolate=self.extrapolate,
            )
        except ValueError as e:
            return TrackerResult(ok=False, status=f"bad_path: {e}")

        state = self.build_state(ego)
        out = self.mpc.compute_control(state, x_ref)

        cf = self.curvature_factor(ego.v_ego)
        delta = float(out.control[0])
        # δ 와 desiredCurvature 는 둘 다 좌(+) → 부호 반전 없음
        kappa = float(np.clip(cf * delta, -self.curv_limit, self.curv_limit))

        return TrackerResult(
            ok=out.solved,
            curvature=kappa if out.solved else 0.0,
            steering_cmd=delta,
            accel_cmd=float(out.control[1]),
            status=out.status,
            solve_time_ms=out.solve_time_ms,
            iters=out.iters,
            lat_error_m=diag["lat_error_m"],
            s_proj_m=diag["s_proj_m"],
            ref_reach_m=diag["ref_reach_m"],
            ref_extrapolated_m=diag["ref_extrapolated_m"],
            curvature_factor=cf,
            x_ref=x_ref,
            predicted_states=out.predicted_states,
            speed_profile=diag["speed_profile"],
        )
