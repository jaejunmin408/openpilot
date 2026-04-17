#!/usr/bin/env python3
"""
locationd_sim: 시뮬(MetaDrive) 전용 livePose 발행기.

실차용 locationd의 EKF(PoseKalman)를 우회하고, 시뮬이 이미 제공하는 참값
(carState + gpsLocationExternal)에서 livePose를 직접 합성한다.
process_config.py가 SIMULATION=1 환경변수일 때 locationd 대신 본 모듈을 기동한다.

좌표계 convention:
  - gpsLocationExternal.bearingDeg: compass (N=0, 시계방향 +, 단위 deg)
  - livePose.orientationNED.z:      NED yaw  (N=0, 시계방향 +, 단위 rad)  → radians(bearingDeg)
  - gpsLocationExternal.vNED:       [v_north, v_east, v_down]
  - livePose.velocityDevice:        [forward, right, down]  (device frame)

발행 주기:
  cameraOdometry(20Hz, camera_odometry_stub가 publish)에 pin.
"""
import math

import cereal.messaging as messaging
from openpilot.common.realtime import config_realtime_process
from openpilot.selfdrive.locationd.locationd import init_xyz_measurement

MAX_YAW_DT = 0.5  # seconds, gps 끊김 판정 한계

# 시뮬 참값이라 std는 작은 고정값 (downstream이 valid=True로 판정할 수준)
STD_ORIENT = [0.01, 0.01, 0.005]
STD_VEL    = [0.05, 0.05, 0.1]
STD_ACC    = [0.1, 0.1, 0.1]
STD_GYRO   = [0.01, 0.01, 0.01]


class SimPoseState:
  def __init__(self):
    self.yaw = 0.0              # rad, NED
    self.yaw_rate = 0.0         # rad/s
    self.v_ned = [0.0, 0.0, 0.0]
    self.v_ego = 0.0
    self.a_ego = 0.0
    self.last_bearing_rad: float | None = None
    self.last_bearing_t_ns: int | None = None

  def update_carState(self, cs) -> None:
    self.v_ego = float(cs.vEgo)
    self.a_ego = float(cs.aEgo)

  def update_gps(self, gps, t_ns: int) -> None:
    bearing_rad = math.radians(gps.bearingDeg)
    if self.last_bearing_rad is not None and self.last_bearing_t_ns is not None:
      dt = (t_ns - self.last_bearing_t_ns) / 1e9
      if 0.0 < dt < MAX_YAW_DT:
        dyaw = bearing_rad - self.last_bearing_rad
        dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi  # wrap to [-pi, pi]
        self.yaw_rate = dyaw / dt
    self.yaw = bearing_rad
    self.v_ned = [float(v) for v in gps.vNED]
    self.last_bearing_rad = bearing_rad
    self.last_bearing_t_ns = t_ns

  def body_velocity(self) -> tuple[float, float, float]:
    """vNED(North, East, Down) → device frame (forward, right, down)."""
    vn, ve, vd = self.v_ned
    c, s = math.cos(self.yaw), math.sin(self.yaw)
    fwd = vn * c + ve * s
    right = -vn * s + ve * c
    return fwd, right, vd

  def build_live_pose(self):
    msg = messaging.new_message('livePose')
    msg.valid = True
    lp = msg.livePose

    init_xyz_measurement(lp.orientationNED, [0.0, 0.0, self.yaw], STD_ORIENT, True)

    fwd, right, down = self.body_velocity()
    init_xyz_measurement(lp.velocityDevice, [fwd, right, down], STD_VEL, True)

    init_xyz_measurement(lp.accelerationDevice, [self.a_ego, 0.0, 0.0], STD_ACC, True)
    init_xyz_measurement(lp.angularVelocityDevice, [0.0, 0.0, self.yaw_rate], STD_GYRO, True)

    lp.inputsOK = True
    lp.posenetOK = True
    lp.sensorsOK = True
    return msg


def main():
  config_realtime_process([0, 1, 2, 3], 5)

  pm = messaging.PubMaster(['livePose'])
  sm = messaging.SubMaster(
    ['carState', 'gpsLocationExternal', 'liveCalibration', 'cameraOdometry'],
    poll='cameraOdometry',
  )

  state = SimPoseState()
  initialized = False

  while True:
    sm.update()

    if sm.updated['carState']:
      state.update_carState(sm['carState'])

    if sm.updated['gpsLocationExternal']:
      state.update_gps(sm['gpsLocationExternal'], sm.logMonoTime['gpsLocationExternal'])

    if not initialized:
      initialized = (
        sm.alive['carState']
        and sm.alive['gpsLocationExternal']
        and state.last_bearing_rad is not None
      )
      continue

    if sm.updated['cameraOdometry']:
      pm.send('livePose', state.build_live_pose())


if __name__ == "__main__":
  main()
