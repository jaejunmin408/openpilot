"""
alpasim MPC 가 쓰는 차량 파라미터.

alpasim 원본(src/controller/alpasim_controller/vehicle_model.py)의
VehicleModel.Parameters 를 그대로 옮긴 것. 원본의 VehicleModel 본체(플랜트
시뮬레이터)는 이식하지 않는다 — openpilot 에서는 실차/시뮬이 플랜트 역할을
하고 MPC 는 예측 모델로만 이 파라미터를 쓴다.

openpilot CarParams 에서 값을 끌어오는 from_car_params() 를 추가했다.
"""
import math
from dataclasses import dataclass

# alpasim ↔ openpilot 코너링 강성 규약 차이
#   alpasim  : a_00 = -2·(Caf + Car) / (m·vx)   → Caf 는 "타이어 1개" 기준
#   openpilot: A[0,0] = -(cF + cR) / (m·u)      → cF 는 "축(axle) 전체" 기준
# 따라서 Caf = cF / 2. (a_01/b_00 항까지 대입해보면 정확히 일치한다.)
_STIFFNESS_PER_TIRE = 0.5


@dataclass
class VehicleParameters:
    """차량 파라미터. 기본값은 alpasim 원본과 동일(Ford Fusion)."""

    mass: float = 2014.4                        # 질량 [kg]
    inertia: float = 3414.2                     # z축 관성모멘트 [kg·m²]
    l_rig_to_cg: float = 1.59                   # 뒷바퀴 → CoG 거리 [m]
    wheelbase: float = 2.85                     # 축거 [m]
    front_cornering_stiffness: float = 93534.5  # 전륜 코너링 강성 [N/rad], 타이어 1개 기준
    rear_cornering_stiffness: float = 176162.1  # 후륜 코너링 강성 [N/rad], 타이어 1개 기준
    steering_time_constant: float = 0.1         # 조향 응답 시정수 [s]
    acceleration_time_constant: float = 0.1     # 가속 응답 시정수 [s]
    kinematic_threshold_speed: float = 5.0      # 이 속도 이하는 kinematic 모델 [m/s]

    @classmethod
    def from_car_params(cls, CP, steering_time_constant=None):
        """openpilot CarParams → alpasim 파라미터.

        Args:
            CP: cereal car.CarParams (또는 동일 필드를 가진 객체)
            steering_time_constant: 지정 시 조향 시정수 override.
                openpilot 은 이 값을 CarParams 로 들고 있지 않으므로
                liveDelay.lateralDelay 등을 넣어주고 싶을 때 사용.

        CarParams 값이 비어 있으면(0 이하) 해당 필드는 기본값을 유지한다.
        """
        defaults = cls()

        def pick(value, fallback):
            try:
                value = float(value)
            except (TypeError, ValueError):
                return fallback
            return value if math.isfinite(value) and value > 0.0 else fallback

        wheelbase = pick(getattr(CP, "wheelbase", None), defaults.wheelbase)
        center_to_front = pick(getattr(CP, "centerToFront", None), wheelbase * 0.44)
        # alpasim l_rig_to_cg = 뒷바퀴→CoG = wheelbase − centerToFront (openpilot aR)
        l_rig_to_cg = wheelbase - center_to_front
        if not (0.05 < l_rig_to_cg < wheelbase):
            l_rig_to_cg = defaults.l_rig_to_cg

        return cls(
            mass=pick(getattr(CP, "mass", None), defaults.mass),
            inertia=pick(getattr(CP, "rotationalInertia", None), defaults.inertia),
            l_rig_to_cg=l_rig_to_cg,
            wheelbase=wheelbase,
            front_cornering_stiffness=pick(
                getattr(CP, "tireStiffnessFront", None) * _STIFFNESS_PER_TIRE
                if getattr(CP, "tireStiffnessFront", None) else None,
                defaults.front_cornering_stiffness,
            ),
            rear_cornering_stiffness=pick(
                getattr(CP, "tireStiffnessRear", None) * _STIFFNESS_PER_TIRE
                if getattr(CP, "tireStiffnessRear", None) else None,
                defaults.rear_cornering_stiffness,
            ),
            steering_time_constant=pick(steering_time_constant, defaults.steering_time_constant),
            acceleration_time_constant=defaults.acceleration_time_constant,
            kinematic_threshold_speed=defaults.kinematic_threshold_speed,
        )
