"""numpy_qp (expm + ADMM QP) 검증."""
import math

import numpy as np

from openpilot.selfdrive.controls.lib.alpasim_mpc.numpy_qp import BoxQPSolver, expm


def taylor_expm(M, terms=60):
    """참조용 Taylor 급수 exp(M). norm 이 작을 때만 신뢰할 수 있다."""
    M = np.asarray(M, dtype=np.float64)
    out = np.eye(M.shape[0])
    term = np.eye(M.shape[0])
    for k in range(1, terms):
        term = term @ M / k
        out = out + term
    return out


class TestExpm:
    def test_zero(self):
        assert np.allclose(expm(np.zeros((4, 4))), np.eye(4))

    def test_diagonal(self):
        d = np.array([-3.0, 0.5, 2.0, -0.1])
        assert np.allclose(expm(np.diag(d)), np.diag(np.exp(d)))

    def test_rotation_generator(self):
        # [[0,-t],[t,0]] → 회전행렬
        t = 0.7
        got = expm(np.array([[0.0, -t], [t, 0.0]]))
        want = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
        assert np.allclose(got, want, atol=1e-12)

    def test_nilpotent(self):
        # N² = 0 → exp(N) = I + N 정확히
        N = np.array([[0.0, 2.0], [0.0, 0.0]])
        assert np.allclose(expm(N), np.eye(2) + N, atol=1e-14)

    def test_matches_taylor_small_norm(self):
        rng = np.random.default_rng(0)
        for _ in range(20):
            M = rng.normal(size=(8, 8)) * 0.15
            assert np.allclose(expm(M), taylor_expm(M), atol=1e-11)

    def test_matches_taylor_large_norm(self):
        """scaling-and-squaring 경로(||M||_1 > theta_13)를 타는지 확인."""
        rng = np.random.default_rng(1)
        M = rng.normal(size=(6, 6)) * 2.0
        assert np.max(np.sum(np.abs(M), axis=0)) > 5.371920351148152
        assert np.allclose(expm(M), taylor_expm(M, terms=200), rtol=1e-9, atol=1e-9)

    def test_semigroup_property(self):
        """exp(M) 두 번 = exp(2M) — 독립적인 정확도 확인."""
        rng = np.random.default_rng(2)
        M = rng.normal(size=(10, 10)) * 0.8
        E = expm(M)
        assert np.allclose(E @ E, expm(2.0 * M), rtol=1e-9, atol=1e-9)

    def test_mpc_sized_stiff_matrix(self):
        """실제 MPC 이산화에서 나오는 크기/stiffness 의 행렬."""
        A = np.zeros((10, 10))
        A[2, 5] = 1.0
        A[3, 7] = 1.0
        A[4, 4] = -10.0
        A[5, 5] = -10.0
        A[4, 6] = 10.0 * 4.17 * 1.59 / 2.85
        A[5, 6] = 10.0 * 4.17 / 2.85
        A[6, 6] = -10.0
        A[7, 7] = -10.0
        A[6, 8] = 10.0
        A[7, 9] = 10.0
        M = A * 0.1
        assert np.allclose(expm(M), taylor_expm(M, terms=120), atol=1e-11)


class TestBoxQPSolver:
    def test_unconstrained_matches_analytic(self):
        """제약이 느슨하면 해는 x = -H⁻¹g."""
        rng = np.random.default_rng(3)
        n = 12
        L = rng.normal(size=(n, n))
        H = L @ L.T + np.eye(n) * 0.5
        g = rng.normal(size=n)
        A = np.eye(n)
        big = np.full(n, 1e3)

        s = BoxQPSolver(eps_abs=1e-9, eps_rel=1e-9, max_iter=20000, check_every=10)
        res = s.solve(H, g, A, -big, big)
        assert res.solved, res.status
        assert np.allclose(res.x, np.linalg.solve(H, -g), atol=1e-5)

    def test_box_active_1d(self):
        """min 0.5x² - 5x  s.t. x <= 2  →  x = 2."""
        s = BoxQPSolver(eps_abs=1e-10, eps_rel=1e-10, max_iter=20000, check_every=10)
        res = s.solve(np.array([[1.0]]), np.array([-5.0]),
                      np.array([[1.0]]), np.array([-1e6]), np.array([2.0]))
        assert res.solved, res.status
        assert abs(res.x[0] - 2.0) < 1e-5

    def test_box_active_multi(self):
        """대각 H, 박스가 전부 활성인 경우 → 해는 각 성분의 clip."""
        n = 6
        H = np.diag(np.arange(1.0, n + 1))
        g = -np.arange(1.0, n + 1) * 10.0     # 무제약 해 = 10 (모든 성분)
        A = np.eye(n)
        lo = np.full(n, -3.0)
        hi = np.full(n, 2.5)
        s = BoxQPSolver(eps_abs=1e-10, eps_rel=1e-10, max_iter=20000, check_every=10)
        res = s.solve(H, g, A, lo, hi)
        assert res.solved, res.status
        assert np.allclose(res.x, hi, atol=1e-5)

    def test_general_inequality_kkt(self):
        """일반 부등식 제약에서 KKT 조건을 직접 검증."""
        rng = np.random.default_rng(4)
        n, m = 8, 14
        L = rng.normal(size=(n, n))
        H = L @ L.T + np.eye(n) * 0.2
        g = rng.normal(size=n)
        A = rng.normal(size=(m, n))
        center = A @ rng.normal(size=n) * 0.3
        lo = center - 0.5
        hi = center + 0.5

        s = BoxQPSolver(eps_abs=1e-8, eps_rel=1e-8, max_iter=50000, check_every=25)
        res = s.solve(H, g, A, lo, hi)
        assert res.solved, res.status

        x, y, z = res.x, res.y, res.z
        # 1. primal feasibility
        assert np.all(A @ x <= hi + 1e-5)
        assert np.all(A @ x >= lo - 1e-5)
        # 2. stationarity
        assert np.max(np.abs(H @ x + g + A.T @ y)) < 1e-5
        # 3. 부호 조건: y > 0 → 상한 활성, y < 0 → 하한 활성
        for i in range(m):
            if y[i] > 1e-6:
                assert abs(z[i] - hi[i]) < 1e-5
            elif y[i] < -1e-6:
                assert abs(z[i] - lo[i]) < 1e-5

    def test_warm_start_reduces_iterations(self):
        """비슷한 문제를 연속으로 풀면 반복수가 줄어든다 (20 Hz 루프의 근거)."""
        rng = np.random.default_rng(5)
        n, m = 20, 30
        L = rng.normal(size=(n, n))
        H = L @ L.T + np.eye(n)
        g = rng.normal(size=n)
        A = rng.normal(size=(m, n))
        lo = np.full(m, -1.0)
        hi = np.full(m, 1.0)

        s = BoxQPSolver()
        first = s.solve(H, g, A, lo, hi)
        second = s.solve(H, g * 1.01, A, lo, hi)
        assert first.solved and second.solved
        assert second.iters <= first.iters

    def test_reset_clears_warm_start(self):
        s = BoxQPSolver()
        H = np.eye(3)
        g = np.ones(3)
        A = np.eye(3)
        s.solve(H, g, A, np.full(3, -1.0), np.full(3, 1.0))
        assert s._x is not None
        s.reset()
        assert s._x is None

    def test_dimension_change_ignores_stale_warm_start(self):
        s = BoxQPSolver()
        s.solve(np.eye(3), np.ones(3), np.eye(3), np.full(3, -1.0), np.full(3, 1.0))
        res = s.solve(np.eye(5), np.ones(5), np.eye(5), np.full(5, -1.0), np.full(5, 1.0))
        assert res.solved
        assert np.allclose(res.x, -np.ones(5), atol=1e-4)
