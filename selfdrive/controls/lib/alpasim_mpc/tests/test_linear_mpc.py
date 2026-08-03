"""alpasim LinearMPC 이식판 검증 — reference 생성, 단발 해, 폐루프 추종."""
import math

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.alpasim_mpc.linear_mpc import LinearMPC, MPCGains
from openpilot.selfdrive.controls.lib.alpasim_mpc.path_tracker import (
    EgoState,
    MPCPathTracker,
    SolverFailurePolicy,
    build_reference,
    path_arclength_and_heading,
    project_ego_onto_path,
    speed_profile,
)
from openpilot.selfdrive.controls.lib.alpasim_mpc.tests.plant import Plant
from openpilot.selfdrive.controls.lib.alpasim_mpc.vehicle_model import VehicleParameters

TARGET_SPEED = 15.0 / 3.6
LON_KP = 0.3
ACCEL_MIN, ACCEL_MAX = -3.5, 2.0


def straight_path(length=25.0, n=26, y_offset=0.0):
    x = np.linspace(0.0, length, n)
    return x, np.full(n, y_offset)


def arc_path(curvature, length=25.0, n=51):
    """곡률 일정한 원호. curvature > 0 이면 좌회전(y=좌+)."""
    s = np.linspace(0.0, length, n)
    if abs(curvature) < 1e-9:
        return s, np.zeros(n)
    r = 1.0 / curvature
    theta = s * curvature
    return r * np.sin(theta), r * (1.0 - np.cos(theta))


# ── reference 생성 ────────────────────────────────────
class TestPathGeometry:
    def test_arclength_straight(self):
        s, x, y, psi = path_arclength_and_heading(*straight_path(10.0, 11))
        assert np.allclose(s, np.linspace(0, 10, 11))
        assert np.allclose(psi, 0.0, atol=1e-12)

    def test_heading_of_45deg_line(self):
        t = np.linspace(0, 10, 21)
        s, x, y, psi = path_arclength_and_heading(t, t)
        assert np.allclose(psi, math.pi / 4, atol=1e-9)
        assert s[-1] == pytest.approx(10.0 * math.sqrt(2.0))

    def test_heading_of_arc(self):
        k = 0.05
        px, py = arc_path(k, length=20.0, n=201)
        s, x, y, psi = path_arclength_and_heading(px, py)
        # 원호에서 psi = k·s
        assert np.allclose(psi, k * s, atol=2e-3)

    def test_duplicate_points_removed(self):
        x = np.array([0.0, 1.0, 1.0, 2.0, 3.0])
        y = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        s, xx, yy, psi = path_arclength_and_heading(x, y)
        assert len(xx) == 4
        assert np.all(np.diff(s) > 0)

    def test_near_duplicate_points_removed(self):
        """1 mm 이내 점은 제거 — 남기면 gradient 가 heading 을 폭발시킨다."""
        x = np.array([0.0, 1.0, 1.0 + 1e-5, 2.0, 3.0])
        y = np.array([0.0, 0.0, 4e-5, 0.0, 0.0])
        s, xx, yy, psi = path_arclength_and_heading(x, y)
        assert len(xx) == 4
        assert np.all(np.abs(psi) < 0.1), f"heading blew up: {psi}"

    def test_near_duplicates_do_not_break_reference(self):
        x = np.array([0.0, 5.0, 5.0 + 2e-5, 10.0, 15.0, 20.0, 25.0])
        y = np.zeros(7)
        x_ref, _ = build_reference(x, y, TARGET_SPEED, 20, 0.1, TARGET_SPEED,
                                   LON_KP, ACCEL_MIN, ACCEL_MAX)
        assert np.all(np.isfinite(x_ref))
        assert np.allclose(x_ref[:, LinearMPC.IY], 0.0, atol=1e-9)
        assert np.allclose(x_ref[:, LinearMPC.IYAW], 0.0, atol=1e-9)

    def test_too_short_path_raises(self):
        with pytest.raises(ValueError):
            path_arclength_and_heading([0.0], [0.0])

    def test_all_duplicate_points_raises(self):
        with pytest.raises(ValueError):
            path_arclength_and_heading([1.0, 1.0, 1.0], [2.0, 2.0, 2.0])

    def test_projection_on_offset_straight_path(self):
        """path 가 왼쪽으로 1.5 m 떨어진 직선 → lat_error = +1.5, s_proj = 0 근처."""
        s, x, y, _ = path_arclength_and_heading(*straight_path(20.0, 21, y_offset=1.5))
        s_proj, lat = project_ego_onto_path(s, x, y)
        assert lat == pytest.approx(1.5, abs=1e-9)
        assert s_proj == pytest.approx(0.0, abs=1e-9)

    def test_projection_when_path_starts_behind(self):
        """path 가 뒤쪽에서 시작하면 s_proj 가 ego 위치까지 전진한다."""
        x = np.linspace(-5.0, 15.0, 41)
        y = np.zeros(41)
        s, x, y, _ = path_arclength_and_heading(x, y)
        s_proj, lat = project_ego_onto_path(s, x, y)
        assert s_proj == pytest.approx(5.0, abs=1e-6)
        assert lat == pytest.approx(0.0, abs=1e-9)

    def test_projection_sign_right_side(self):
        s, x, y, _ = path_arclength_and_heading(*straight_path(20.0, 21, y_offset=-0.8))
        _, lat = project_ego_onto_path(s, x, y)
        assert lat == pytest.approx(-0.8, abs=1e-9)


class TestSpeedProfile:
    def test_converges_to_target(self):
        # P 게인 0.3 → 시정수 1/0.3 = 3.33 s. 10 s 면 95 % 수준까지만 온다.
        v = speed_profile(0.0, 100, 0.1, TARGET_SPEED, LON_KP, ACCEL_MIN, ACCEL_MAX)
        assert v[0] == 0.0
        assert 0.94 * TARGET_SPEED < v[-1] < TARGET_SPEED
        assert np.all(np.diff(v) >= -1e-12)
        # 충분히 길게 굴리면 목표에 수렴
        v_long = speed_profile(0.0, 600, 0.1, TARGET_SPEED, LON_KP, ACCEL_MIN, ACCEL_MAX)
        assert v_long[-1] == pytest.approx(TARGET_SPEED, abs=0.01)

    def test_respects_accel_limits(self):
        v = speed_profile(0.0, 20, 0.1, 100.0, 10.0, ACCEL_MIN, ACCEL_MAX)
        assert np.max(np.diff(v) / 0.1) <= ACCEL_MAX + 1e-9

    def test_never_negative(self):
        v = speed_profile(1.0, 50, 0.1, 0.0, 10.0, ACCEL_MIN, ACCEL_MAX)
        assert np.all(v >= 0.0)


class TestBuildReference:
    def test_shape_and_zero_columns(self):
        x_ref, diag = build_reference(*straight_path(), TARGET_SPEED, 20, 0.1,
                                      TARGET_SPEED, LON_KP, ACCEL_MIN, ACCEL_MAX)
        assert x_ref.shape == (21, 8)
        # x/y/yaw 외 열은 0 (Q 에서 가중치 0 이거나 accel 목표 0)
        for col in (LinearMPC.IVX, LinearMPC.IVY, LinearMPC.IYAW_RATE,
                    LinearMPC.ISTEERING, LinearMPC.IACCEL):
            assert np.all(x_ref[:, col] == 0.0)

    def test_x_advances_with_speed(self):
        x_ref, diag = build_reference(*straight_path(), TARGET_SPEED, 20, 0.1,
                                      TARGET_SPEED, LON_KP, ACCEL_MIN, ACCEL_MAX)
        # 정속이므로 x_ref[k] ≈ k·dt·v
        expect = np.arange(21) * 0.1 * TARGET_SPEED
        assert np.allclose(x_ref[:, LinearMPC.IX], expect, atol=1e-6)
        assert diag["ref_reach_m"] == pytest.approx(2.0 * TARGET_SPEED, abs=1e-6)

    def test_align_removes_offset_when_path_starts_behind(self):
        """path 가 자차 뒤에서 시작하면 투영으로 종방향 오차가 제거된다.

        udp_bridge 패킷은 패킷 시점 자차 위치에서 잘려 오므로, 제어 시점까지의
        이동(최대 ~100 ms) 만큼 path[0] 이 뒤로 밀린다. 그걸 흡수하는 게 목적.
        """
        x = np.linspace(-2.0, 20.0, 45)
        y = np.zeros(45)
        aligned, d_aligned = build_reference(x, y, TARGET_SPEED, 20, 0.1, TARGET_SPEED,
                                             LON_KP, ACCEL_MIN, ACCEL_MAX, align_to_closest=True)
        raw, _ = build_reference(x, y, TARGET_SPEED, 20, 0.1, TARGET_SPEED,
                                 LON_KP, ACCEL_MIN, ACCEL_MAX, align_to_closest=False)
        assert aligned[0, LinearMPC.IX] == pytest.approx(0.0, abs=1e-6)
        assert d_aligned["s_proj_m"] == pytest.approx(2.0, abs=1e-6)
        # 투영 없이는 reference 가 2 m 뒤에서 시작해 종방향 오차가 남는다
        assert raw[0, LinearMPC.IX] == pytest.approx(-2.0, abs=1e-6)

    def test_align_clamps_when_path_starts_ahead(self):
        """path 가 전부 앞쪽에 있으면 투영점은 path[0] 이고 종방향 갭이 남는다.

        이건 한계가 아니라 사실의 반영이다 — 갭이 실재하므로 투영으로 없앨 수
        없다. 종제어를 외부에 맡긴 채 이 갭이 조향에 섞이는 걸 막고 싶으면
        long_position_weight 를 0 으로 두면 된다.
        """
        x = np.linspace(3.0, 25.0, 45)
        y = np.zeros(45)
        aligned, diag = build_reference(x, y, TARGET_SPEED, 20, 0.1, TARGET_SPEED,
                                        LON_KP, ACCEL_MIN, ACCEL_MAX, align_to_closest=True)
        assert diag["s_proj_m"] == pytest.approx(0.0, abs=1e-9)
        assert aligned[0, LinearMPC.IX] == pytest.approx(3.0, abs=1e-6)

class TestLongitudinalLateralDecoupling:
    """ego frame(yaw=0) 선형화에서 종/횡이 정확히 분리된다는 구조적 사실.

    이게 성립하므로 MPC 의 accel_cmd 를 버리고 기존 P 제어기를 써도
    조향 출력이 전혀 달라지지 않는다 (udp_bridge 의 MPC_USE_ACCEL=False 근거).
    """

    LAT_STATES = [LinearMPC.IY, LinearMPC.IYAW, LinearMPC.IVY,
                  LinearMPC.IYAW_RATE, LinearMPC.ISTEERING]
    LON_STATES = [LinearMPC.IX, LinearMPC.IVX, LinearMPC.IACCEL]

    @pytest.mark.parametrize("vx", [0.5, 4.17, 10.0, 20.0])
    def test_linearized_dynamics_are_block_diagonal(self, vx):
        """yaw=0 에서 A, B 의 종↔횡 교차 블록이 정확히 0.

        yaw=0 이면 A[x, yaw] = -vx·sin(0) = 0, A[y, vx] = sin(0) = 0 이 되어
        x 는 vx 에만, y 는 yaw 에만 의존한다. kinematic/dynamic 두 분기 모두.
        """
        mpc = LinearMPC(vehicle_params=VehicleParameters())
        x0 = np.zeros(LinearMPC.NX)
        x0[LinearMPC.IVX] = vx
        A, B = mpc._linearize_dynamics(x0)

        assert np.all(A[np.ix_(self.LON_STATES, self.LAT_STATES)] == 0.0)
        assert np.all(A[np.ix_(self.LAT_STATES, self.LON_STATES)] == 0.0)
        assert np.all(B[np.ix_(self.LON_STATES, [0])] == 0.0)   # 조향입력 → 종상태 없음
        assert np.all(B[np.ix_(self.LAT_STATES, [1])] == 0.0)   # 가속입력 → 횡상태 없음

    def test_coupling_appears_only_off_ego_frame(self):
        """yaw != 0 이면 결합이 생긴다 — ego frame 이라 실사용에서는 안 생긴다."""
        mpc = LinearMPC(vehicle_params=VehicleParameters())
        x0 = np.zeros(LinearMPC.NX)
        x0[LinearMPC.IVX] = 4.17
        x0[LinearMPC.IYAW] = 0.2
        A, _ = mpc._linearize_dynamics(x0)
        assert np.abs(A[np.ix_(self.LON_STATES, self.LAT_STATES)]).max() > 1e-3

    def test_ego_state_always_has_zero_yaw(self):
        """tracker 가 만드는 상태는 항상 yaw=0 — 위 분리가 늘 성립하는 근거."""
        tr = MPCPathTracker(target_speed=TARGET_SPEED, lon_kp=LON_KP,
                            accel_min=ACCEL_MIN, accel_max=ACCEL_MAX)
        st = tr.build_state(EgoState(v_ego=9.0, v_lat=0.3, yaw_rate=0.2,
                                     steering_angle=0.1, accel_long=1.0))
        assert st[LinearMPC.IYAW] == 0.0

    def test_long_position_weight_does_not_affect_steering(self):
        """long_position_weight 를 0 으로 바꿔도 조향 출력이 비트단위로 같다."""
        kw = dict(target_speed=TARGET_SPEED, lon_kp=LON_KP,
                  accel_min=ACCEL_MIN, accel_max=ACCEL_MAX)
        ego = EgoState(v_ego=TARGET_SPEED)
        for path in (straight_path(25.0, 26, y_offset=0.8), arc_path(0.04, 30.0)):
            a = MPCPathTracker(gains=MPCGains(long_position_weight=2.0), **kw).update(*path, ego)
            b = MPCPathTracker(gains=MPCGains(long_position_weight=0.0), **kw).update(*path, ego)
            assert a.ok and b.ok
            assert a.steering_cmd == b.steering_cmd

    def test_longitudinal_reference_gap_does_not_affect_steering(self):
        """path 가 앞쪽에서 시작해 종방향 갭이 생겨도 조향은 그대로다."""
        kw = dict(gains=MPCGains(), target_speed=TARGET_SPEED, lon_kp=LON_KP,
                  accel_min=ACCEL_MIN, accel_max=ACCEL_MAX)
        ego = EgoState(v_ego=TARGET_SPEED)
        near = straight_path(25.0, 26, y_offset=0.8)
        far = (np.linspace(3.0, 28.0, 26), np.full(26, 0.8))
        a = MPCPathTracker(**kw).update(*near, ego)
        b = MPCPathTracker(**kw).update(*far, ego)
        assert a.ok and b.ok
        assert a.steering_cmd == pytest.approx(b.steering_cmd, abs=1e-9)

    def test_mpc_accel_matches_external_p_controller(self):
        """MPC 의 accel_cmd 는 우리가 넣은 속도 프로파일의 되풀이다.

        reference 를 같은 P 제어기로 만들었으므로 새 정보가 없다 —
        MPC_USE_ACCEL=False 가 잃는 게 없다는 근거.
        """
        tr = MPCPathTracker(target_speed=TARGET_SPEED, lon_kp=LON_KP,
                            accel_min=ACCEL_MIN, accel_max=ACCEL_MAX)
        path = straight_path(60.0, 121)
        for v in (0.5, 2.0, 3.5):
            res = tr.update(*path, EgoState(v_ego=v))
            a_p = float(np.clip(LON_KP * (TARGET_SPEED - v), ACCEL_MIN, ACCEL_MAX))
            assert res.ok
            assert res.accel_cmd == pytest.approx(a_p, abs=0.15), \
                f"v={v}: mpc {res.accel_cmd:+.3f} vs P {a_p:+.3f}"

    def test_at_target_speed_accel_cmd_is_zero(self):
        tr = MPCPathTracker(target_speed=TARGET_SPEED, lon_kp=LON_KP,
                            accel_min=ACCEL_MIN, accel_max=ACCEL_MAX)
        res = tr.update(*straight_path(60.0, 121), EgoState(v_ego=TARGET_SPEED))
        assert res.ok
        assert abs(res.accel_cmd) < 1e-3

    def test_align_preserves_lateral_error(self):
        """투영해도 횡오차는 사라지지 않는다 (이게 MPC 가 잡아야 하는 오차)."""
        x_ref, diag = build_reference(*straight_path(y_offset=1.2), TARGET_SPEED,
                                      20, 0.1, TARGET_SPEED, LON_KP, ACCEL_MIN, ACCEL_MAX)
        assert diag["lat_error_m"] == pytest.approx(1.2, abs=1e-9)
        assert np.allclose(x_ref[:, LinearMPC.IY], 1.2, atol=1e-9)

    def test_extrapolation_beyond_path_end(self):
        """horizon 이 path 보다 멀리 보면 마지막 접선으로 직선 외삽."""
        x_ref, diag = build_reference(*straight_path(length=5.0, n=11), TARGET_SPEED,
                                      20, 0.1, TARGET_SPEED, LON_KP, ACCEL_MIN, ACCEL_MAX,
                                      extrapolate=True)
        assert diag["ref_extrapolated_m"] > 0.0
        # 끝점 hold 가 아니라 계속 전진해야 한다
        assert x_ref[-1, LinearMPC.IX] > 5.0
        assert np.all(np.diff(x_ref[:, LinearMPC.IX]) > 0)

    def test_hold_when_extrapolation_disabled(self):
        x_ref, _ = build_reference(*straight_path(length=5.0, n=11), TARGET_SPEED,
                                   20, 0.1, TARGET_SPEED, LON_KP, ACCEL_MIN, ACCEL_MAX,
                                   extrapolate=False)
        assert x_ref[-1, LinearMPC.IX] == pytest.approx(5.0, abs=1e-9)

    def test_arc_reference_heading_increases(self):
        x_ref, _ = build_reference(*arc_path(0.05), TARGET_SPEED, 20, 0.1,
                                   TARGET_SPEED, LON_KP, ACCEL_MIN, ACCEL_MAX)
        yaw = x_ref[:, LinearMPC.IYAW]
        assert np.all(np.diff(yaw) > 0)          # 좌회전 → yaw 증가
        assert yaw[-1] == pytest.approx(0.05 * 2.0 * TARGET_SPEED, abs=0.01)


# ── MPC 구성 검증 ─────────────────────────────────────
class TestLinearMPCConstruction:
    def test_rejects_idx_start_penalty_ge_horizon(self):
        """원본의 함정: 이 경우 Q 가 전부 0 이 되어 u=0 이 조용히 나온다."""
        with pytest.raises(ValueError, match="idx_start_penalty"):
            LinearMPC(gains=MPCGains(idx_start_penalty=20), n_horizon=20)
        with pytest.raises(ValueError, match="idx_start_penalty"):
            LinearMPC(gains=MPCGains(idx_start_penalty=10), n_horizon=5)

    def test_accepts_valid_config(self):
        mpc = LinearMPC(gains=MPCGains(idx_start_penalty=10), n_horizon=20)
        assert mpc.horizon_seconds == pytest.approx(2.0)

    def test_rejects_bad_horizon(self):
        with pytest.raises(ValueError):
            LinearMPC(n_horizon=0)
        with pytest.raises(ValueError):
            LinearMPC(dt_mpc=0.0)

    def test_rejects_wrong_ref_shape(self):
        mpc = LinearMPC()
        with pytest.raises(ValueError, match="x_ref"):
            mpc.compute_control(np.zeros(8), np.zeros((10, 8)))

    def test_rejects_wrong_state_shape(self):
        mpc = LinearMPC()
        with pytest.raises(ValueError, match="state"):
            mpc.compute_control(np.zeros(5), np.zeros((21, 8)))

    def test_clip_state_to_bounds(self):
        mpc = LinearMPC()
        bad = np.zeros(8)
        bad[LinearMPC.ISTEERING] = 1.5       # > 45°
        bad[LinearMPC.IACCEL] = -20.0        # < -8
        bad[LinearMPC.IVX] = -3.0            # 후진
        ok = mpc.clip_state_to_bounds(bad)
        assert ok[LinearMPC.ISTEERING] == pytest.approx(math.pi / 4)
        assert ok[LinearMPC.IACCEL] == pytest.approx(-8.0)
        assert ok[LinearMPC.IVX] == 0.0

    def test_discretization_is_consistent_with_plant(self):
        """선형화·이산화가 플랜트 적분과 한 스텝 일치하는지 (동작점 근처)."""
        params = VehicleParameters()
        mpc = LinearMPC(vehicle_params=params, dt_mpc=0.1)
        x0 = np.zeros(8)
        x0[LinearMPC.IVX] = 4.0
        u = np.array([0.02, 0.0])

        A_d, B_d = mpc._linearize_dynamics(x0)
        x_lin = A_d @ x0 + B_d @ u

        plant = Plant(params, initial_velocity=(4.0, 0.0))
        plant.state = x0.copy()
        x_plant = plant.advance(u, 0.1).copy()

        assert np.allclose(x_lin[:3], x_plant[:3], atol=2e-3)
        # 조향 동역학은 정확히 선형이라 expm 이산화는 해석해와 같다. 남는 차이는
        # 플랜트 쪽 RK2(10 ms 서브스텝) 적분오차 (~1e-5).
        assert np.allclose(x_lin[LinearMPC.ISTEERING], x_plant[LinearMPC.ISTEERING], atol=1e-4)
        tau = params.steering_time_constant
        exact = u[0] * (1.0 - math.exp(-0.1 / tau))
        assert x_lin[LinearMPC.ISTEERING] == pytest.approx(exact, abs=1e-12)


# ── 단발 해의 방향/부호 ──────────────────────────────
class TestSingleSolveSigns:
    def make_tracker(self, **kw):
        kw.setdefault("target_speed", TARGET_SPEED)
        kw.setdefault("lon_kp", LON_KP)
        kw.setdefault("accel_min", ACCEL_MIN)
        kw.setdefault("accel_max", ACCEL_MAX)
        return MPCPathTracker(**kw)

    def test_centered_straight_path_no_steer(self):
        tr = self.make_tracker()
        res = tr.update(*straight_path(), EgoState(v_ego=TARGET_SPEED))
        assert res.ok, res.status
        assert abs(res.steering_cmd) < 1e-3
        assert abs(res.curvature) < 1e-3

    def test_path_on_left_steers_left(self):
        """path 가 왼쪽에 있으면 좌회전 → δ > 0, κ > 0 (openpilot 좌+ 규약)."""
        tr = self.make_tracker()
        res = tr.update(*straight_path(y_offset=1.0), EgoState(v_ego=TARGET_SPEED))
        assert res.ok, res.status
        assert res.steering_cmd > 0.0
        assert res.curvature > 0.0
        assert res.lat_error_m == pytest.approx(1.0, abs=1e-9)

    def test_path_on_right_steers_right(self):
        tr = self.make_tracker()
        res = tr.update(*straight_path(y_offset=-1.0), EgoState(v_ego=TARGET_SPEED))
        assert res.ok, res.status
        assert res.steering_cmd < 0.0
        assert res.curvature < 0.0

    def test_left_arc_commands_overshoot_from_cold_start(self):
        """좌회전 원호 + 조향 0 에서 출발하면 정상상태보다 큰 값을 지령한다.

        액추에이터 지연(tau=0.1)과 idx_start_penalty 로 앞 1 초가 마스킹된 상태에서
        1~2 초 뒤 지점을 맞추려면 초기에 과지령하는 것이 최적이다 — MPC 의
        정상 거동이고 부호/스케일 오류가 아니다.
        정상상태 값이 경로 곡률과 맞는지는 TestClosedLoop 에서 확인한다.
        """
        k = 0.05
        tr = self.make_tracker()
        res = tr.update(*arc_path(k, length=30.0), EgoState(v_ego=TARGET_SPEED))
        assert res.ok, res.status
        assert res.curvature > k
        assert res.curvature < 2.0 * k

    def test_right_arc_curvature_sign(self):
        tr = self.make_tracker()
        res = tr.update(*arc_path(-0.05, length=30.0), EgoState(v_ego=TARGET_SPEED))
        assert res.ok, res.status
        assert res.curvature < 0.0

    def test_curvature_is_clipped(self):
        tr = self.make_tracker(curv_limit=0.01)
        res = tr.update(*straight_path(y_offset=3.0), EgoState(v_ego=TARGET_SPEED))
        assert res.ok, res.status
        assert abs(res.curvature) <= 0.01 + 1e-12

    def test_bad_path_reports_not_ok(self):
        tr = self.make_tracker()
        res = tr.update([0.0], [0.0], EgoState(v_ego=TARGET_SPEED))
        assert not res.ok
        assert res.curvature == 0.0
        assert res.status.startswith("bad_path")

    def test_solve_is_fast_enough_for_20hz(self):
        """20 Hz(50 ms) 루프 안에서 돌아야 한다. warm start 후 정상 상태 기준."""
        tr = self.make_tracker()
        ego = EgoState(v_ego=TARGET_SPEED)
        for _ in range(5):
            tr.update(*arc_path(0.02, length=30.0), ego)
        times = [tr.update(*arc_path(0.02, length=30.0), ego).solve_time_ms for _ in range(20)]
        assert max(times) < 25.0, f"solve too slow: max {max(times):.1f} ms"

    def test_state_signs_from_ego_state(self):
        """EgoState → MPC 상태 변환: rig 횡속도 → CoG 횡속도."""
        tr = self.make_tracker()
        p = tr._params
        ego = EgoState(v_ego=5.0, v_lat=0.2, yaw_rate=0.1, steering_angle=0.03, accel_long=0.5)
        st = tr.build_state(ego)
        assert st[LinearMPC.IVX] == pytest.approx(5.0)
        assert st[LinearMPC.IVY] == pytest.approx(0.2 + p.l_rig_to_cg * 0.1)
        assert st[LinearMPC.IYAW_RATE] == pytest.approx(0.1)
        assert st[LinearMPC.ISTEERING] == pytest.approx(0.03)
        assert st[LinearMPC.IACCEL] == pytest.approx(0.5)
        assert st[LinearMPC.IX] == 0.0 and st[LinearMPC.IY] == 0.0 and st[LinearMPC.IYAW] == 0.0


# ── 해 실패 정책 ─────────────────────────────────────
class TestSolverFailurePolicy:
    def test_success_uses_mpc_value(self):
        pol = SolverFailurePolicy(max_consecutive_fails=3)
        k, ev = pol.select(True, 0.05, prev_curvature=0.01, fallback_curvature=0.09)
        assert k == 0.05
        assert ev is None
        assert pol.fail_streak == 0

    def test_short_failure_holds_previous(self):
        """짧은 실패에는 직전 값 유지 — 조향 0 스냅 방지."""
        pol = SolverFailurePolicy(max_consecutive_fails=3)
        for _ in range(2):
            k, ev = pol.select(False, 0.0, prev_curvature=0.04, fallback_curvature=0.09)
            assert k == 0.04
            assert ev is None
        assert pol.fail_streak == 2
        assert not pol.fallback_active

    def test_falls_back_after_threshold(self):
        pol = SolverFailurePolicy(max_consecutive_fails=3)
        pol.select(False, 0.0, 0.04, 0.09)
        pol.select(False, 0.0, 0.04, 0.09)
        k, ev = pol.select(False, 0.0, 0.04, 0.09)
        assert k == 0.09
        assert ev == "fallback_entered"
        assert pol.fallback_active

    def test_fallback_event_fires_once(self):
        pol = SolverFailurePolicy(max_consecutive_fails=1)
        _, ev1 = pol.select(False, 0.0, 0.0, 0.09)
        _, ev2 = pol.select(False, 0.0, 0.0, 0.09)
        assert ev1 == "fallback_entered"
        assert ev2 is None

    def test_recovery_event_and_clear(self):
        pol = SolverFailurePolicy(max_consecutive_fails=2)
        pol.select(False, 0.0, 0.0, 0.09)
        pol.select(False, 0.0, 0.0, 0.09)
        assert pol.fallback_active
        k, ev = pol.select(True, 0.06, 0.0, 0.09)
        assert k == 0.06
        assert ev == "recovered"
        assert pol.fail_streak == 0
        assert not pol.fallback_active

    def test_fallback_disabled_holds_forever(self):
        pol = SolverFailurePolicy(max_consecutive_fails=2, use_fallback=False)
        for _ in range(20):
            k, ev = pol.select(False, 0.0, prev_curvature=0.04, fallback_curvature=0.09)
            assert k == 0.04
            assert ev is None
        assert pol.fail_streak == 20
        assert not pol.fallback_active

    def test_reset(self):
        pol = SolverFailurePolicy(max_consecutive_fails=1)
        pol.select(False, 0.0, 0.0, 0.09)
        assert pol.fallback_active
        pol.reset()
        assert pol.fail_streak == 0
        assert not pol.fallback_active


# ── 폐루프 추종 (alpasim 플랜트) ─────────────────────
class PathSlicer:
    """실제 publisher 처럼 "경로 위 현재 위치에서 앞으로 slice_m" 를 잘라 준다.

    ego frame x 윈도우로 자르면 안 된다: 반경이 작은 원호는 몇 바퀴 감기면서
    다음 바퀴의 점이 그 윈도우에 들어오고, 투영이 그걸 집어버린다.
    실제 publisher 도 자기가 경로 위 어디인지 알고 앞쪽만 보내므로, 인덱스를
    단조 전진시키며 자르는 게 실제 동작에도 더 가깝다.
    """

    def __init__(self, world_x, world_y, slice_m=20.0, behind_m=3.0):
        self.wx = np.asarray(world_x, dtype=np.float64)
        self.wy = np.asarray(world_y, dtype=np.float64)
        ds = np.hypot(np.diff(self.wx), np.diff(self.wy))
        self.s = np.concatenate([[0.0], np.cumsum(ds)])
        self.slice_m = slice_m
        self.behind_m = behind_m
        self.idx = 0

    def slice_for(self, plant, search_ahead_m=15.0):
        """현재 차량 위치 기준 앞쪽 slice 를 ego frame 으로 돌려준다.

        Returns (px, py) 또는 경로가 소진되면 None.
        """
        # 직전 인덱스에서 앞쪽으로만 최근접점을 찾는다 (단조 전진)
        hi = int(np.searchsorted(self.s, self.s[self.idx] + search_ahead_m))
        hi = min(max(hi, self.idx + 1), len(self.s))
        d2 = (self.wx[self.idx:hi] - plant.state[0]) ** 2 + (self.wy[self.idx:hi] - plant.state[1]) ** 2
        self.idx += int(np.argmin(d2))

        lo = int(np.searchsorted(self.s, self.s[self.idx] - self.behind_m))
        end = int(np.searchsorted(self.s, self.s[self.idx] + self.slice_m))
        if end - lo < 3 or end >= len(self.s):
            return None
        return plant.path_in_ego_frame(self.wx[lo:end], self.wy[lo:end])


def run_closed_loop(world_x, world_y, y0=0.0, yaw0=0.0, v0=TARGET_SPEED,
                    duration=12.0, dt=0.05, tracker_kw=None, use_curvature=False,
                    return_curvatures=False, slice_m=20.0):
    """alpasim 플랜트 + MPC 폐루프. udp_bridge 와 같은 20 Hz 구조.

    use_curvature=True 면 κ 로 환산한 뒤 다시 δ 로 되돌려 넣는다
    (openpilot 의 δ→κ→latcontrol 경로를 흉내낸 것).
    """
    params = VehicleParameters()
    tr = MPCPathTracker(vehicle_params=params, target_speed=TARGET_SPEED,
                        lon_kp=LON_KP, accel_min=ACCEL_MIN, accel_max=ACCEL_MAX,
                        **(tracker_kw or {}))
    plant = Plant(params, initial_velocity=(v0, 0.0), initial_pose=(0.0, y0, yaw0))
    slicer = PathSlicer(world_x, world_y, slice_m=slice_m)

    lat_errors = []
    statuses = []
    curvatures = []
    u = np.array([0.0, 0.0])
    for _ in range(int(duration / dt)):
        sliced = slicer.slice_for(plant)
        if sliced is None:
            break
        px, py = sliced
        st = plant.state
        ego = EgoState(
            v_ego=st[3],
            v_lat=st[4] - params.l_rig_to_cg * st[5],   # CoG → rig 횡속도
            yaw_rate=st[5],
            steering_angle=st[6],
            accel_long=st[7],
        )
        res = tr.update(px, py, ego)
        statuses.append(res.status)
        if res.ok:
            if use_curvature:
                # δ → κ → δ 왕복 (환산 계수가 일관되면 무손실)
                delta = res.curvature / max(res.curvature_factor, 1e-9)
            else:
                delta = res.steering_cmd
            u = np.array([delta, res.accel_cmd])
        lat_errors.append(res.lat_error_m)
        curvatures.append(res.curvature)
        plant.advance(u, dt)

    if return_curvatures:
        return np.array(lat_errors), statuses, plant, np.array(curvatures)
    return np.array(lat_errors), statuses, plant


class TestClosedLoop:
    def test_all_solves_succeed_on_straight(self):
        _, statuses, _ = run_closed_loop(*straight_path(400.0, 401), y0=1.0)
        bad = [s for s in statuses if s not in ("solved", "solved_inaccurate")]
        assert not bad, f"{len(bad)} failed solves, e.g. {bad[:3]}"

    def test_converges_from_lateral_offset(self):
        """1.5 m 왼쪽으로 벗어난 상태에서 직선 경로에 수렴해야 한다."""
        errs, _, _ = run_closed_loop(*straight_path(400.0, 401), y0=1.5)
        assert abs(errs[0]) == pytest.approx(1.5, abs=0.05)
        assert abs(errs[-1]) < 0.10, f"final lat error {errs[-1]:.3f} m"

    def test_converges_from_right_offset(self):
        errs, _, _ = run_closed_loop(*straight_path(400.0, 401), y0=-1.5)
        assert abs(errs[-1]) < 0.10, f"final lat error {errs[-1]:.3f} m"

    def test_converges_from_heading_error(self):
        errs, _, _ = run_closed_loop(*straight_path(400.0, 401), y0=0.0, yaw0=0.15)
        assert abs(errs[-1]) < 0.10, f"final lat error {errs[-1]:.3f} m"

    def test_no_sustained_oscillation(self):
        """수렴 후 진동이 남지 않아야 한다 (부호 반전/과도 게인 탐지)."""
        errs, _, _ = run_closed_loop(*straight_path(400.0, 401), y0=1.0)
        tail = errs[len(errs) // 2:]
        assert np.std(tail) < 0.05, f"tail std {np.std(tail):.3f} m"

    def test_tracks_constant_curvature_arc(self):
        wx, wy = arc_path(0.03, length=200.0, n=401)
        errs, _, _ = run_closed_loop(wx, wy, y0=0.0)
        # 정상상태 추종오차는 남을 수 있지만 발산하지 않아야 한다
        assert abs(errs[-1]) < 0.40, f"final lat error {errs[-1]:.3f} m"
        assert np.max(np.abs(errs)) < 1.0

    @pytest.mark.parametrize("k_path", [0.02, 0.03, 0.05, -0.04])
    def test_steady_state_curvature_matches_path(self, k_path):
        """정상상태 지령 곡률이 경로 곡률과 일치 → δ→κ 환산 계수/부호 검증.

        단발 해는 cold start 과도 때문에 과지령되므로, 스케일 검증은 반드시
        정상상태에서 해야 한다.
        """
        wx, wy = arc_path(k_path, length=300.0, n=601)
        _, _, _, ks = run_closed_loop(wx, wy, y0=0.0, duration=20.0,
                                      return_curvatures=True)
        settled = float(np.mean(ks[-50:]))
        assert settled == pytest.approx(k_path, rel=0.10), \
            f"path κ={k_path}, settled κ={settled:.4f}"

    def test_tracks_right_arc(self):
        wx, wy = arc_path(-0.03, length=200.0, n=401)
        errs, _, _ = run_closed_loop(wx, wy, y0=0.0)
        assert abs(errs[-1]) < 0.35, f"final lat error {errs[-1]:.3f} m"

    def test_curvature_roundtrip_matches_direct_steering(self):
        """δ→κ 환산 경로가 δ 직접 사용과 같은 거동을 내야 한다 (부호/계수 검증)."""
        direct, _, _ = run_closed_loop(*straight_path(400.0, 401), y0=1.2, use_curvature=False)
        via_kappa, _, _ = run_closed_loop(*straight_path(400.0, 401), y0=1.2, use_curvature=True)
        assert np.allclose(direct, via_kappa, atol=1e-6)

    def test_vehicle_actually_moved_forward(self):
        _, _, plant = run_closed_loop(*straight_path(400.0, 401), y0=1.0)
        assert plant.state[0] > 30.0
        assert plant.state[3] == pytest.approx(TARGET_SPEED, abs=0.3)

    def test_longer_horizon_still_solves_on_straight(self):
        """dt_mpc 를 늘린 설정도 해가 나오고 직선에서는 수렴한다."""
        errs, statuses, _ = run_closed_loop(
            *straight_path(400.0, 401), y0=1.5,
            tracker_kw={"n_horizon": 20, "dt_mpc": 0.2},
        )
        bad = [s for s in statuses if s not in ("solved", "solved_inaccurate")]
        assert not bad, f"{len(bad)} failed solves"
        assert abs(errs[-1]) < 0.25, f"final lat error {errs[-1]:.3f} m"


class TestHorizonAndTuningCharacteristics:
    """측정으로 확인한 이 정식화의 특성을 고정한다 (튜닝 시 근거).

    수치 자체보다 대소관계를 검증한다 — 수치는 차량 파라미터에 딸려 움직인다.
    """

    def _arc_error(self, k_path=0.05, dur=25.0, **tracker_kw):
        wx, wy = arc_path(k_path, length=400.0, n=1601)
        errs, statuses, _ = run_closed_loop(wx, wy, y0=0.0, duration=dur,
                                            tracker_kw=tracker_kw)
        bad = [s for s in statuses if s not in ("solved", "solved_inaccurate")]
        assert not bad, f"{len(bad)} failed solves"
        return float(errs[-1])

    @pytest.mark.parametrize("k_path", [0.02, 0.05, 0.10])
    def test_arc_tracking_is_accurate_with_defaults(self, k_path):
        """기본 설정에서 원호 정상상태 횡오차가 0.2 m 안쪽이어야 한다."""
        assert abs(self._arc_error(k_path)) < 0.20

    @pytest.mark.parametrize("k_path", [0.02, 0.05, 0.10])
    def test_default_cuts_corner_slightly(self, k_path):
        """기본 gain 은 곡선에서 안쪽으로 파고든다(음수) — idx_start_penalty 영향."""
        assert self._arc_error(k_path) < 0.0

    def test_shortening_horizon_degrades_arc_tracking(self):
        """horizon 을 2 s 아래로 줄이면 곡선에서 바깥쪽으로 크게 벌어진다.

        이 정식화에서 2 s 는 줄이면 안 되는 값이다. 늘리는 쪽은 이득이 작다
        (test_extending_horizon_is_not_a_win 참고).
        """
        base = self._arc_error(0.10, n_horizon=20, dt_mpc=0.1)     # 2.0 s
        short = self._arc_error(0.10, n_horizon=20, dt_mpc=0.05)   # 1.0 s
        assert base < 0.0 < short, f"2.0s {base:+.3f}, 1.0s {short:+.3f}"
        assert abs(short) > 3.0 * abs(base)

    def test_extending_horizon_is_not_a_win(self):
        """horizon 을 4 s 로 늘려도 큰 곡률에서는 개선되지 않는다.

        A 를 현재 yaw 에 고정해 선형화하므로 예측 구간에서 yaw 가 많이 변하면
        모델이 틀려진다. solve 비용만 2 배 되므로 기본값을 유지할 근거.
        """
        base = self._arc_error(0.10, n_horizon=20, dt_mpc=0.1)
        long_dt = self._arc_error(0.10, n_horizon=20, dt_mpc=0.2)
        assert abs(long_dt) > abs(base), f"2.0s {base:+.3f}, 4.0s(dt=0.2) {long_dt:+.3f}"

    def test_idx_start_penalty_monotonically_affects_corner_cutting(self):
        """isp 를 올리면 corner cutting 이 커진다. 단조롭지만 효과는 작다."""
        errs = [self._arc_error(0.05, gains=MPCGains(idx_start_penalty=isp))
                for isp in (1, 5, 10, 15)]
        assert all(e < 0.0 for e in errs)
        # isp 증가 → |오차| 증가 (안쪽으로 더 파고듦)
        assert all(abs(a) <= abs(b) + 1e-9 for a, b in zip(errs[:-1], errs[1:], strict=True)), errs
        # 다만 개선폭은 작다 — isp 를 노브로 크게 기대하면 안 된다
        assert abs(errs[0]) > 0.3 * abs(errs[-1])

    def test_straight_line_convergence_insensitive_to_idx_start_penalty(self):
        """직선 오프셋 수렴은 isp 와 무관하게 0 에 붙는다 — 곡선에서만 차이."""
        for isp in (3, 5, 10):
            errs, _, _ = run_closed_loop(*straight_path(400.0, 801), y0=1.5, duration=25.0,
                                         tracker_kw={"gains": MPCGains(idx_start_penalty=isp)})
            assert abs(errs[-1]) < 0.05, f"isp={isp} final {errs[-1]:+.3f} m"

    def test_solve_time_budget_at_20hz(self):
        """20 Hz 루프(50 ms) 대비 여유가 충분한지. 곡선이 가장 반복수가 많다."""
        wx, wy = arc_path(0.05, length=400.0, n=801)
        params = VehicleParameters()
        tr = MPCPathTracker(vehicle_params=params, target_speed=TARGET_SPEED,
                            lon_kp=LON_KP, accel_min=ACCEL_MIN, accel_max=ACCEL_MAX)
        plant = Plant(params, initial_velocity=(TARGET_SPEED, 0.0))
        times = []
        u = np.array([0.0, 0.0])
        for _ in range(200):
            px, py = plant.path_in_ego_frame(wx, wy)
            keep = (px > -3.0) & (px < 20.0)
            px, py = px[keep], py[keep]
            st = plant.state
            ego = EgoState(v_ego=st[3], v_lat=st[4] - params.l_rig_to_cg * st[5],
                           yaw_rate=st[5], steering_angle=st[6], accel_long=st[7])
            res = tr.update(px, py, ego)
            times.append(res.solve_time_ms)
            if res.ok:
                u = np.array([res.steering_cmd, res.accel_cmd])
            plant.advance(u, 0.05)
        assert np.mean(times) < 10.0, f"mean solve {np.mean(times):.1f} ms"
        assert np.max(times) < 30.0, f"max solve {np.max(times):.1f} ms"
