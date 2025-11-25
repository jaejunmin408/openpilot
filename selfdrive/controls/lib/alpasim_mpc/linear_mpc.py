"""
alpasim LinearMPC 이식판 (openpilot 용).

원본: alpasim src/controller/alpasim_controller/mpc_impl/linear_mpc.py
      (+ mpc_controller.py 의 MPCGains / ControllerOutput)

동역학 선형화 · 이산화 · condensed QP 정식화는 원본과 동일하다. 달라진 점:

  1. reference 를 alpasim_utils.geometry.Trajectory(시간 인덱스) 대신
     (N+1, NX) ndarray 로 직접 받는다. → alpasim_utils / alpasim_grpc 의존성 제거.
     시간축 보간은 호출자(path_tracker) 가 담당한다.
  2. scipy.linalg.expm → numpy_qp.expm
  3. osqp + scipy.sparse → numpy_qp.BoxQPSolver (dense ADMM, 알고리즘 동일)
  4. Q_blk 를 (N+1)·nx 정사각 블록대각으로 만들지 않고 penalized 블록만
     누적한다. 수식은 완전히 동일하고 20 Hz 루프에서 훨씬 싸다.
  5. idx_start_penalty >= n_horizon 이면 생성 시 에러.
     원본은 이 경우 Q_blk 가 전부 0 이 되어 u=0 을 조용히 내보낸다.
  6. clip_state_to_bounds(): 실차 센서 상태가 상태제약 박스를 벗어나면
     QP 가 infeasible 해지므로 x0 를 박스 안으로 넣어주는 헬퍼 추가.
     (alpasim 은 자기 플랜트가 제약을 지키므로 필요 없었다.)
"""
import logging
import math
from dataclasses import dataclass

import numpy as np

from openpilot.selfdrive.controls.lib.alpasim_mpc.numpy_qp import BoxQPSolver, expm
from openpilot.selfdrive.controls.lib.alpasim_mpc.vehicle_model import VehicleParameters

# alpasim mpc_controller.py 의 모듈 레벨 기본값
DEFAULT_N_HORIZON = 20
DEFAULT_DT_MPC = 0.1


@dataclass
class MPCGains:
    """MPC 코스트 가중치. 기본값은 alpasim configs/controller/default.yaml 과 동일.

    Attributes:
        long_position_weight: 종방향 위치 오차 페널티
        lat_position_weight: 횡방향 위치 오차 페널티
        heading_weight: heading 오차 페널티
        acceleration_weight: 가속 상태 페널티
        rel_front_steering_angle_weight: 조향 입력 페널티
        rel_acceleration_weight: 가속 입력 페널티
        idx_start_penalty: 이 horizon 인덱스 이전에는 추종 코스트를 적용하지 않는다.
            기본값 10 · dt_mpc 0.1 → 앞 1.0 초 구간은 추종 목표 없음(지연/초기
            오프셋 보상). 상태제약과 입력 코스트는 전 구간 그대로 적용된다.

    주의: 원본 LinearMPC 는 rel_* 가중치를 입력의 "절대 크기" uᵀRu 에 적용한다
    (do_mpc 쪽 NonlinearMPC 의 set_rterm 은 Δu 에 적용 — 이름과 달리 두 구현의
    의미가 다르다). 여기서는 LinearMPC 원본 동작을 그대로 유지한다.
    """

    long_position_weight: float = 2.0
    lat_position_weight: float = 1.0
    heading_weight: float = 1.0
    acceleration_weight: float = 0.1
    rel_front_steering_angle_weight: float = 5.0
    rel_acceleration_weight: float = 1.0
    idx_start_penalty: int = 10


@dataclass
class MPCOutput:
    """MPC 결과. 원본 ControllerOutput + 디버그 필드."""

    control: np.ndarray          # [steering_cmd(rad, 좌회전+), accel_cmd(m/s²)]
    solve_time_ms: float
    status: str
    iters: int = 0
    predicted_states: np.ndarray | None = None   # (N+1, NX), 로깅/viz 용

    @property
    def solved(self):
        return self.status in ("solved", "solved_inaccurate")


class LinearMPC:
    """OSQP 계열 ADMM 으로 푸는 선형 MPC.

    상태: [x, y, yaw, vx_cg, vy_cg, yaw_rate, steering, accel]
      - x, y, yaw : rig 원점의 위치/자세. ego frame 기준이라 x0 에서는 (0,0,0)
      - y / yaw 부호: y = 좌(+), yaw = CCW(+)  ← openpilot body frame 과 동일
      - vx_cg, vy_cg : CoG 기준 body frame 속도
      - steering : 전륜 조향각 [rad], 좌회전(+)
      - accel : 종가속 상태 [m/s²]

    입력: [steering_cmd, accel_cmd] — 액추에이터 1차 지연(시정수 tau)을 통해
          steering/accel 상태에 반영된다.
    """

    # 상태 인덱스 (원본과 동일)
    IX = 0
    IY = 1
    IYAW = 2
    IVX = 3
    IVY = 4
    IYAW_RATE = 5
    ISTEERING = 6
    IACCEL = 7

    NX = 8
    NU = 2

    # 솔버 파라미터 (원본 OSQP 설정과 동일한 허용오차/반복수)
    EPS_ABS = 1e-4
    EPS_REL = 1e-4
    MAX_ITER = 500
    QP_REGULARIZATION = 1e-6

    def __init__(self, vehicle_params=None, gains=None,
                 n_horizon=DEFAULT_N_HORIZON, dt_mpc=DEFAULT_DT_MPC):
        self._vehicle_params = vehicle_params or VehicleParameters()
        self._gains = gains or MPCGains()
        self._n_horizon = int(n_horizon)
        self._dt_mpc = float(dt_mpc)

        if self._n_horizon < 1:
            raise ValueError(f"n_horizon must be >= 1, got {self._n_horizon}")
        if self._dt_mpc <= 0.0:
            raise ValueError(f"dt_mpc must be > 0, got {self._dt_mpc}")
        # 원본의 함정: idx_start_penalty > n_horizon 이면 Q_blk 가 전부 0 이 되어
        # QP 가 min uᵀRu → u=0 으로 축퇴하고, 조향/가속이 조용히 0 이 된다.
        if self._gains.idx_start_penalty >= self._n_horizon:
            raise ValueError(
                f"idx_start_penalty({self._gains.idx_start_penalty}) must be < "
                + f"n_horizon({self._n_horizon}); 아니면 추종 코스트가 사라져 u=0 이 된다"
            )

        self._solver = BoxQPSolver(
            sigma=self.QP_REGULARIZATION,
            eps_abs=self.EPS_ABS,
            eps_rel=self.EPS_REL,
            max_iter=self.MAX_ITER,
        )

        # 코스트 행렬 (원본과 동일, 대각이므로 벡터로 보관)
        self._q_diag = np.array([
            self._gains.long_position_weight,    # x
            self._gains.lat_position_weight,     # y
            self._gains.heading_weight,          # yaw
            0.0,                                 # vx  - 추종 안 함
            0.0,                                 # vy  - 추종 안 함
            0.0,                                 # yaw_rate - 추종 안 함
            0.0,                                 # steering - 추종 안 함
            self._gains.acceleration_weight,     # accel
        ])
        self._r_diag = np.array([
            self._gains.rel_front_steering_angle_weight,
            self._gains.rel_acceleration_weight,
        ])

        # 일부 상태에만 제약 (원본과 동일)
        self._constrained_state_indices = [self.IYAW, self.IVX, self.ISTEERING, self.IACCEL]
        self._x_min_constrained = np.array([
            -math.pi / 2,   # yaw [rad]  ±90°
            0.0,            # vx [m/s]   후진 금지
            -math.pi / 4,   # steering [rad] ±45°
            -8.0,           # accel [m/s²] 제동 한계
        ])
        self._x_max_constrained = np.array([
            math.pi / 2,
            35.0,           # ~125 km/h
            math.pi / 4,
            6.0,
        ])
        self._u_min = np.array([-2.0, -9.0])
        self._u_max = np.array([2.0, 6.0])

    @property
    def name(self):
        return "alpasim_linear_mpc"

    @property
    def dt_mpc(self):
        return self._dt_mpc

    @property
    def n_horizon(self):
        return self._n_horizon

    @property
    def gains(self):
        return self._gains

    @property
    def vehicle_params(self):
        return self._vehicle_params

    @property
    def horizon_seconds(self):
        return self._n_horizon * self._dt_mpc

    def reset(self):
        """warm start 폐기. engage/disengage, path 끊김 등 불연속 시점에 호출."""
        self._solver.reset()

    def clip_state_to_bounds(self, state):
        """x0 를 상태제약 박스 안으로 넣는다.

        실차 센서에서 온 상태가 박스를 벗어나면(예: |steering| > 45°, accel < -8)
        QP 가 사실상 infeasible 해져 해가 무의미해진다. 원본에는 없는 방어 코드.
        """
        state = np.asarray(state, dtype=np.float64).copy()
        for i, idx in enumerate(self._constrained_state_indices):
            state[idx] = np.clip(state[idx], self._x_min_constrained[i], self._x_max_constrained[i])
        return state

    def compute_control(self, state, x_ref):
        """현재 상태와 reference 로부터 최적 입력 계산.

        Args:
            state: 현재 상태 (NX,) — ego frame 이므로 보통 x=y=yaw=0
            x_ref: reference 상태 (N+1, NX). x/y/yaw/accel 열만 코스트에 쓰인다.

        Returns:
            MPCOutput
        """
        import time

        start_time = time.perf_counter()

        x0 = np.asarray(state, dtype=np.float64).reshape(-1)
        if x0.shape[0] != self.NX:
            raise ValueError(f"state must have {self.NX} elements, got {x0.shape[0]}")

        x_ref = np.asarray(x_ref, dtype=np.float64)
        if x_ref.shape != (self._n_horizon + 1, self.NX):
            raise ValueError(
                f"x_ref must be ({self._n_horizon + 1}, {self.NX}), got {x_ref.shape}"
            )

        u_opt, status, iters, x_pred = self._solve_qp(x0, x_ref)
        solve_time_ms = (time.perf_counter() - start_time) * 1000.0

        return MPCOutput(
            control=u_opt,
            solve_time_ms=solve_time_ms,
            status=status,
            iters=iters,
            predicted_states=x_pred,
        )

    # ── 선형화 (원본 _linearize_dynamics 그대로) ──────
    def _linearize_dynamics(self, x_op):
        """bicycle 모델을 동작점 기준 선형화 → 이산 (A_d, B_d).

            x_{k+1} = A_d x_k + B_d u_k
        """
        params = self._vehicle_params
        dt = self._dt_mpc

        yaw = x_op[self.IYAW]
        v_cg_x = x_op[self.IVX]
        v_cg_y = x_op[self.IVY]
        yaw_rate = x_op[self.IYAW_RATE]

        use_kinematic = v_cg_x < params.kinematic_threshold_speed

        A = np.zeros((self.NX, self.NX))
        B = np.zeros((self.NX, self.NU))

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        if use_kinematic:
            # kinematic 모델 선형화
            A[self.IX, self.IYAW] = -v_cg_x * sin_yaw
            A[self.IX, self.IVX] = cos_yaw

            A[self.IY, self.IYAW] = v_cg_x * cos_yaw
            A[self.IY, self.IVX] = sin_yaw

            A[self.IYAW, self.IYAW_RATE] = 1.0
            A[self.IVX, self.IACCEL] = 1.0

            # v_cg_x 는 scheduling 파라미터로 고정한다. 곱미분 항을 affine 잔차
            # 없이 넣으면 동작점 항이 이중 계산된다.
            GAIN = 10.0
            l_r = params.l_rig_to_cg
            L = params.wheelbase
            A[self.IVY, self.IVY] = -GAIN
            A[self.IVY, self.ISTEERING] = GAIN * v_cg_x * l_r / L

            A[self.IYAW_RATE, self.ISTEERING] = GAIN * v_cg_x / L
            A[self.IYAW_RATE, self.IYAW_RATE] = -GAIN

        else:
            # dynamic 모델 선형화
            kinetic_mass = params.mass * v_cg_x
            kinetic_inertia = params.inertia * v_cg_x

            lf = params.wheelbase - params.l_rig_to_cg
            lr = params.l_rig_to_cg
            Caf = params.front_cornering_stiffness
            Car = params.rear_cornering_stiffness

            lf_caf = lf * Caf
            lr_car = lr * Car

            a_00 = -2 * (Caf + Car) / kinetic_mass
            a_01 = -v_cg_x - 2 * (lf_caf - lr_car) / kinetic_mass
            a_10 = -2 * (lf_caf - lr_car) / kinetic_inertia
            a_11 = -2 * (lf * lf_caf + lr * lr_car) / kinetic_inertia

            b_00 = 2 * Caf / params.mass
            b_10 = 2 * lf_caf / params.inertia

            v_rig_y = v_cg_y - lr * yaw_rate

            A[self.IX, self.IYAW] = -v_cg_x * sin_yaw - v_rig_y * cos_yaw
            A[self.IX, self.IVX] = cos_yaw
            A[self.IX, self.IVY] = -sin_yaw
            A[self.IX, self.IYAW_RATE] = lr * sin_yaw

            A[self.IY, self.IYAW] = v_cg_x * cos_yaw - v_rig_y * sin_yaw
            A[self.IY, self.IVX] = sin_yaw
            A[self.IY, self.IVY] = cos_yaw
            A[self.IY, self.IYAW_RATE] = -lr * cos_yaw

            A[self.IYAW, self.IYAW_RATE] = 1.0
            A[self.IVX, self.IACCEL] = 1.0

            A[self.IVY, self.IVY] = a_00
            A[self.IVY, self.IYAW_RATE] = a_01
            A[self.IVY, self.ISTEERING] = b_00

            A[self.IYAW_RATE, self.IVY] = a_10
            A[self.IYAW_RATE, self.IYAW_RATE] = a_11
            A[self.IYAW_RATE, self.ISTEERING] = b_10

        # 조향 액추에이터 동역학
        tau_s = params.steering_time_constant
        A[self.ISTEERING, self.ISTEERING] = -1.0 / tau_s
        B[self.ISTEERING, 0] = 1.0 / tau_s

        # 가속 액추에이터 동역학
        tau_a = params.acceleration_time_constant
        A[self.IACCEL, self.IACCEL] = -1.0 / tau_a
        B[self.IACCEL, 1] = 1.0 / tau_a

        # matrix exponential 로 이산화
        nx = self.NX
        nu = self.NU
        M = np.zeros((nx + nu, nx + nu))
        M[:nx, :nx] = A * dt
        M[:nx, nx:] = B * dt

        expM = expm(M)
        return expM[:nx, :nx], expM[:nx, nx:]

    # ── condensed QP (원본 _solve_qp 그대로) ─────────
    def _solve_qp(self, x0, x_ref):
        """condensed 형태로 MPC QP 를 푼다.

        J = Σ_k (x_k − x_ref_k)ᵀ Q (x_k − x_ref_k) + u_kᵀ R u_k
        s.t. x_{k+1} = A x_k + B u_k,  u_min ≤ u_k ≤ u_max,  x_min ≤ x_k ≤ x_max
        """
        N = self._n_horizon
        nx = self.NX
        nu = self.NU
        idx_start = self._gains.idx_start_penalty

        A_d, B_d = self._linearize_dynamics(x0)

        # condensed prediction:  x_k = S_x[k] x0 + S_u[k] U
        S_x = np.zeros(((N + 1) * nx, nx))
        S_u = np.zeros(((N + 1) * nx, N * nu))

        A_pow = np.eye(nx)
        for k in range(N + 1):
            S_x[k * nx:(k + 1) * nx, :] = A_pow
            if k < N:
                A_pow = A_d @ A_pow

        for j in range(N):
            Psi = B_d
            for k in range(j, N):
                S_u[(k + 1) * nx:(k + 2) * nx, j * nu:(j + 1) * nu] = Psi
                Psi = A_d @ Psi

        # 제어 0 일 때의 자유 응답
        x_pred_free = S_x @ x0

        # condensed cost:  min 0.5 Uᵀ H U + gᵀ U
        # 원본은 (N+1)nx 정사각 Q_blk 를 만들어 S_uᵀ Q_blk S_u 를 계산하는데,
        # Q_blk 가 k >= idx_start 구간만 채워진 블록대각이라 아래와 수식이 동일하다.
        H = np.diag(np.tile(self._r_diag, N))
        g = np.zeros(N * nu)
        q = self._q_diag
        for k in range(idx_start, N + 1):
            Sk = S_u[k * nx:(k + 1) * nx, :]                 # (nx, N·nu)
            qSk = q[:, None] * Sk                            # Q 가 대각이므로
            H += Sk.T @ qSk
            dx = x_pred_free[k * nx:(k + 1) * nx] - x_ref[k]
            g += Sk.T @ (q * dx)

        H = 0.5 * (H + H.T) + self.QP_REGULARIZATION * np.eye(N * nu)

        # 제약: 입력 박스 + 일부 상태 박스
        constrained_rows = [
            k * nx + idx
            for k in range(1, N + 1)
            for idx in self._constrained_state_indices
        ]
        A_ineq = np.vstack([np.eye(N * nu), S_u[constrained_rows, :]])

        x_pred_at_rows = x_pred_free[constrained_rows]
        l_ineq = np.concatenate([
            np.tile(self._u_min, N),
            np.tile(self._x_min_constrained, N) - x_pred_at_rows,
        ])
        u_ineq = np.concatenate([
            np.tile(self._u_max, N),
            np.tile(self._x_max_constrained, N) - x_pred_at_rows,
        ])

        result = self._solver.solve(H, g, A_ineq, l_ineq, u_ineq)

        if not result.solved:
            logging.debug(
                "alpasim MPC QP status=%s iters=%d prim=%.2e dual=%.2e",
                result.status, result.iters, result.prim_res, result.dual_res,
            )
            return np.zeros(nu), result.status, result.iters, None

        U = result.x
        x_pred = (x_pred_free + S_u @ U).reshape(N + 1, nx)
        return U[:nu].copy(), result.status, result.iters, x_pred
