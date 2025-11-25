"""
alpasim LinearMPC 이식판.

원본: NVIDIA alpasim, src/controller/alpasim_controller/
      - mpc_impl/linear_mpc.py  (OSQP 기반 선형 MPC)
      - mpc_controller.py       (MPCGains 등)
      - vehicle_model.py        (차량 파라미터)

openpilot 환경에 osqp / scipy / casadi 가 없어서 QP 솔버와 matrix exponential 을
numpy 로 자체 구현했다(numpy_qp.py). 알고리즘은 원본과 동일(ADMM / Padé-13).
do_mpc 기반 NonlinearMPC 는 CasADi·ipopt 의존성 때문에 이식하지 않았다.
"""
from openpilot.selfdrive.controls.lib.alpasim_mpc.linear_mpc import (
    DEFAULT_DT_MPC,
    DEFAULT_N_HORIZON,
    LinearMPC,
    MPCGains,
    MPCOutput,
)
from openpilot.selfdrive.controls.lib.alpasim_mpc.numpy_qp import BoxQPSolver, expm
from openpilot.selfdrive.controls.lib.alpasim_mpc.path_tracker import (
    EgoState,
    MPCPathTracker,
    SolverFailurePolicy,
    TrackerResult,
    build_reference,
)
from openpilot.selfdrive.controls.lib.alpasim_mpc.vehicle_model import VehicleParameters

__all__ = [
    "BoxQPSolver",
    "DEFAULT_DT_MPC",
    "DEFAULT_N_HORIZON",
    "EgoState",
    "LinearMPC",
    "MPCGains",
    "MPCOutput",
    "MPCPathTracker",
    "SolverFailurePolicy",
    "TrackerResult",
    "VehicleParameters",
    "build_reference",
    "expm",
]
