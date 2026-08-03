"""
numpy 전용 QP 솔버 + matrix exponential.

alpasim 의 LinearMPC 는 osqp + scipy(linalg.expm, sparse) 에 의존하지만
openpilot 실행 환경(디바이스 포함)에는 둘 다 없다. 여기서 동일한 알고리즘을
numpy 만으로 구현해 의존성 없이 돌아가게 한다.

  - expm()     : Higham(2005) scaling-and-squaring + Padé(13). scipy.linalg.expm 대체.
  - solve_qp() : OSQP 논문의 ADMM(Algorithm 1) 을 reduced KKT 형태로 dense 구현.

문제 크기가 작아서(변수 N·nu=40, 제약 ~120) dense 로 충분히 빠르다.
나중에 osqp 를 설치하게 되면 solve_qp() 만 교체하면 된다.
"""
import numpy as np

# ── matrix exponential ───────────────────────────────
# Padé(13) 계수 (Higham 2005, Table 2.3)
_PADE13_B = (
    64764752532480000.0,
    32382376266240000.0,
    7771770303897600.0,
    1187353796428800.0,
    129060195264000.0,
    10559470521600.0,
    670442572800.0,
    33522128640.0,
    1323241920.0,
    40840800.0,
    960960.0,
    16380.0,
    182.0,
    1.0,
)
_PADE13_THETA = 5.371920351148152


def expm(M):
    """행렬 지수 exp(M). scipy.linalg.expm 과 동일한 알고리즘/정확도.

    scaling-and-squaring: ||M||_1 을 theta_13 이하로 줄인 뒤 Padé(13) 근사,
    그 후 제곱을 s 번 반복해 복원한다.
    """
    M = np.asarray(M, dtype=np.float64)
    n = M.shape[0]
    ident = np.eye(n)

    norm1 = float(np.max(np.sum(np.abs(M), axis=0))) if n else 0.0
    if not np.isfinite(norm1):
        raise FloatingPointError("expm: non-finite input")

    # 스케일링 횟수 s: ||M/2^s||_1 <= theta_13
    s = 0
    if norm1 > _PADE13_THETA:
        s = int(np.ceil(np.log2(norm1 / _PADE13_THETA)))
        M = M / (2.0**s)

    b = _PADE13_B
    M2 = M @ M
    M4 = M2 @ M2
    M6 = M2 @ M4

    U = M @ (
        M6 @ (b[13] * M6 + b[11] * M4 + b[9] * M2)
        + b[7] * M6 + b[5] * M4 + b[3] * M2 + b[1] * ident
    )
    V = (
        M6 @ (b[12] * M6 + b[10] * M4 + b[8] * M2)
        + b[6] * M6 + b[4] * M4 + b[2] * M2 + b[0] * ident
    )

    F = np.linalg.solve(V - U, V + U)
    for _ in range(s):
        F = F @ F
    return F


# ── QP 솔버 (ADMM / OSQP) ────────────────────────────
class QPSolution:
    """solve_qp() 결과. osqp 의 result 객체와 비슷한 모양."""

    __slots__ = ("x", "y", "z", "status", "iters", "prim_res", "dual_res")

    def __init__(self, x, y, z, status, iters, prim_res, dual_res):
        self.x = x
        self.y = y
        self.z = z
        self.status = status
        self.iters = iters
        self.prim_res = prim_res
        self.dual_res = dual_res

    @property
    def solved(self):
        return self.status in ("solved", "solved_inaccurate")


class BoxQPSolver:
    """min 0.5 xᵀHx + gᵀx  s.t.  l <= Ax <= u  를 ADMM 으로 푸는 솔버.

    OSQP 논문 Algorithm 1 을 reduced KKT 로 정리한 형태:

        (H + σI + ρAᵀA) x̃ = σx − g + Aᵀ(ρz − y)
        z̃ = A x̃                      (reduced 형태에서 정확히 성립)
        x ← α x̃ + (1−α) x
        z ← Π_[l,u]( α z̃ + (1−α) z + y/ρ )
        y ← y + ρ( α z̃ + (1−α) z − z )

    H 가 매 스텝 바뀌므로(현재 상태 기준 선형화) 인수분해는 매 solve 마다
    다시 하지만, x/y/z 는 인스턴스에 유지해 warm start 로 재사용한다.
    20 Hz 로 연속 호출되면 문제가 거의 비슷해서 반복 횟수가 크게 줄어든다.
    """

    def __init__(self, sigma=1e-6, rho=0.1, alpha=1.6,
                 eps_abs=1e-4, eps_rel=1e-4, max_iter=500, check_every=25,
                 adapt_rho=True):
        self.sigma = sigma
        self.rho_init = rho
        self.alpha = alpha
        self.eps_abs = eps_abs
        self.eps_rel = eps_rel
        self.max_iter = max_iter
        self.check_every = check_every
        self.adapt_rho = adapt_rho

        # warm start 상태
        self._x = None
        self._y = None
        self._z = None
        self._rho = rho

    def reset(self):
        """warm start 상태 폐기 (engage/disengage 등 불연속 시점에 호출)."""
        self._x = None
        self._y = None
        self._z = None
        self._rho = self.rho_init

    def solve(self, H, g, A, l, u, warm_start=True):
        n = H.shape[0]
        m = A.shape[0]

        if warm_start and self._x is not None and self._x.shape[0] == n and self._z.shape[0] == m:
            x = self._x.copy()
            y = self._y.copy()
            z = self._z.copy()
            rho = self._rho
        else:
            x = np.zeros(n)
            y = np.zeros(m)
            z = np.zeros(m)
            rho = self.rho_init

        sigma_I = self.sigma * np.eye(n)
        AtA = A.T @ A
        At = A.T
        alpha = self.alpha

        def factorize(rho_):
            # H + σI + ρAᵀA 는 SPD (H ⪰ 0, σ>0). 40×40 수준이라 역행렬로 충분.
            return np.linalg.inv(H + sigma_I + rho_ * AtA)

        Minv = factorize(rho)

        g_inf = float(np.max(np.abs(g))) if g.size else 0.0
        status = "max_iter"
        prim_res = dual_res = np.inf
        it = 0

        for it in range(1, self.max_iter + 1):
            xt = Minv @ (self.sigma * x - g + At @ (rho * z - y))
            zt = A @ xt                       # reduced 형태에서 z̃ = A x̃

            x = alpha * xt + (1.0 - alpha) * x
            z_relaxed = alpha * zt + (1.0 - alpha) * z
            z_new = np.clip(z_relaxed + y / rho, l, u)
            y = y + rho * (z_relaxed - z_new)
            z = z_new

            if it % self.check_every == 0 or it == self.max_iter:
                Ax = A @ x
                Hx = H @ x
                Aty = At @ y
                prim_res = float(np.max(np.abs(Ax - z))) if m else 0.0
                dual_res = float(np.max(np.abs(Hx + g + Aty)))

                ax_inf = float(np.max(np.abs(Ax))) if m else 0.0
                z_inf = float(np.max(np.abs(z))) if m else 0.0
                hx_inf = float(np.max(np.abs(Hx)))
                aty_inf = float(np.max(np.abs(Aty))) if m else 0.0

                eps_prim = self.eps_abs + self.eps_rel * max(ax_inf, z_inf)
                eps_dual = self.eps_abs + self.eps_rel * max(hx_inf, aty_inf, g_inf)

                if prim_res <= eps_prim and dual_res <= eps_dual:
                    status = "solved"
                    break

                if self.adapt_rho and it < self.max_iter:
                    num = prim_res / max(ax_inf, z_inf, 1e-10)
                    den = dual_res / max(hx_inf, aty_inf, g_inf, 1e-10)
                    ratio = np.sqrt(num / max(den, 1e-16))
                    if ratio > 5.0 or ratio < 0.2:
                        rho = float(np.clip(rho * ratio, 1e-6, 1e6))
                        Minv = factorize(rho)

        # warm start 용으로 보관
        self._x, self._y, self._z, self._rho = x.copy(), y.copy(), z.copy(), rho

        if status == "max_iter":
            # 허용오차의 10배 안이면 근사해로 인정 (osqp 의 solved_inaccurate 와 같은 취지)
            if prim_res <= 10.0 * self.eps_abs and dual_res <= 10.0 * self.eps_abs:
                status = "solved_inaccurate"

        return QPSolution(x, y, z, status, it, prim_res, dual_res)
