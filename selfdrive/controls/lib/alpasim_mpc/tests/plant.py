"""
테스트 전용 플랜트: alpasim VehicleModel 의 적분기 이식판.

원본 alpasim src/controller/alpasim_controller/vehicle_model.py 의
_derivs() + advance() (RK2, 10 ms 서브스텝) 를 그대로 옮긴 것.
프로덕션 경로에서는 실차/시뮬이 플랜트이므로 쓰지 않고, MPC 폐루프
검증에만 쓴다.
"""
import math

import numpy as np

from openpilot.selfdrive.controls.lib.alpasim_mpc.vehicle_model import VehicleParameters


class Plant:
    """alpasim planar dynamic bicycle model. 상태는 LinearMPC 와 동일한 8차원."""

    def __init__(self, params=None, initial_velocity=(0.0, 0.0), initial_yaw_rate=0.0,
                 initial_pose=(0.0, 0.0, 0.0)):
        self.p = params or VehicleParameters()
        if initial_velocity[0] > 0.25:
            steer0 = math.atan(initial_yaw_rate / initial_velocity[0] * self.p.wheelbase)
        else:
            steer0 = 0.0
        self.state = np.array([
            initial_pose[0], initial_pose[1], initial_pose[2],
            initial_velocity[0], initial_velocity[1], initial_yaw_rate,
            steer0, 0.0,
        ], dtype=np.float64)

    def _derivs(self, state, u):
        p = self.p
        yaw_angle = state[2]
        v_x = state[3]
        v_y = state[4]
        yaw_rate = state[5]
        delta = state[6]
        accel = state[7]

        if v_x < p.kinematic_threshold_speed:
            steady_v_y = v_x * delta * p.l_rig_to_cg / p.wheelbase
            steady_yaw_rate = v_x * delta / p.wheelbase
            GAIN = 10.0
            v_y_rig = 0.0
            d_v_y = GAIN * (steady_v_y - v_y)
            d_yaw_rate = GAIN * (steady_yaw_rate - yaw_rate)
        else:
            kinetic_mass = p.mass * v_x
            kinetic_inertia = p.inertia * v_x
            lf = p.wheelbase - p.l_rig_to_cg
            lf_caf = lf * p.front_cornering_stiffness
            lr_car = p.l_rig_to_cg * p.rear_cornering_stiffness
            lf_sq_caf = lf * lf_caf
            lr_sq_car = p.l_rig_to_cg * lr_car

            a_00 = -2 * (p.front_cornering_stiffness + p.rear_cornering_stiffness) / kinetic_mass
            a_01 = -v_x - 2 * (lf_caf - lr_car) / kinetic_mass
            a_10 = -2 * (lf_caf - lr_car) / kinetic_inertia
            a_11 = -2 * (lf_sq_caf + lr_sq_car) / kinetic_inertia
            b_00 = 2 * p.front_cornering_stiffness / p.mass
            b_10 = 2 * lf_caf / p.inertia

            v_y_rig = v_y - yaw_rate * p.l_rig_to_cg
            d_v_y = a_00 * v_y + a_01 * yaw_rate + b_00 * delta
            d_yaw_rate = a_10 * v_y + a_11 * yaw_rate + b_10 * delta

        return np.array([
            v_x * math.cos(yaw_angle) - v_y_rig * math.sin(yaw_angle),
            v_x * math.sin(yaw_angle) + v_y_rig * math.cos(yaw_angle),
            yaw_rate,
            accel,
            d_v_y,
            d_yaw_rate,
            (u[0] - delta) / p.steering_time_constant,
            (u[1] - accel) / p.acceleration_time_constant,
        ])

    def advance(self, u, dt):
        DT_STEP_MAX = 0.01
        total = 0.0
        while total < dt:
            step = min(DT_STEP_MAX, dt - total)
            total += step
            k1 = step * self._derivs(self.state, u)
            k2 = step * self._derivs(self.state + k1 / 2.0, u)
            self.state += k2
            self.state[3] = max(0.0, self.state[3])
        return self.state

    def path_in_ego_frame(self, world_x, world_y):
        """world frame path → 현재 ego frame (x=전방, y=좌(+))."""
        x0, y0, yaw = self.state[0], self.state[1], self.state[2]
        dx = np.asarray(world_x, dtype=np.float64) - x0
        dy = np.asarray(world_y, dtype=np.float64) - y0
        c, s = math.cos(yaw), math.sin(yaw)
        return c * dx + s * dy, -s * dx + c * dy
