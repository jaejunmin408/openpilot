#!/usr/bin/env python3
"""
UDP Bridge: receives trajectory from external PC via UDP,
publishes modelV2 and drivingModelData cereal messages
(replaces modeld in the pipeline).

Packet format must match trajectory_sender.py:
  Header (16 bytes): magic(uint32) + seq(uint32) + timestamp(float64)
  Action (9 bytes):  desiredCurvature(f32) + desiredAcceleration(f32) + shouldStop(uint8)
  Trajectory (396 bytes): 33 x (position_x, velocity_x, acceleration_x) as float32
"""

import struct
import socket
import time
import numpy as np

import cereal.messaging as messaging
from cereal import log
from cereal.messaging import PubMaster
from openpilot.common.realtime import config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants

# -- UDP config --
UDP_PORT = 5005
RECV_TIMEOUT = 0.1  # seconds

# -- Packet format --
MAGIC = 0x4F505431
HEADER_FMT = "<IId"
ACTION_FMT = "<ffB"
TRAJ_FMT = "<" + "fff" * ModelConstants.IDX_N
PACKET_SIZE = struct.calcsize(HEADER_FMT) + struct.calcsize(ACTION_FMT) + struct.calcsize(TRAJ_FMT)

HEADER_SIZE = struct.calcsize(HEADER_FMT)
ACTION_SIZE = struct.calcsize(ACTION_FMT)
TRAJ_SIZE = struct.calcsize(TRAJ_FMT)


def parse_packet(data: bytes):
  """Parse UDP packet into action and trajectory dicts."""
  if len(data) != PACKET_SIZE:
    return None, None, None

  # header
  magic, seq, timestamp = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
  if magic != MAGIC:
    return None, None, None

  # action
  offset = HEADER_SIZE
  curv, accel, stop = struct.unpack(ACTION_FMT, data[offset:offset + ACTION_SIZE])

  # trajectory
  offset += ACTION_SIZE
  traj_flat = struct.unpack(TRAJ_FMT, data[offset:offset + TRAJ_SIZE])
  traj = np.array(traj_flat, dtype=np.float32).reshape(ModelConstants.IDX_N, 3)

  action = {
    "desired_curvature": curv,
    "desired_acceleration": accel,
    "should_stop": bool(stop),
  }
  trajectory = {
    "position_x": traj[:, 0],
    "velocity_x": traj[:, 1],
    "acceleration_x": traj[:, 2],
  }
  return seq, action, trajectory


def fill_model_msg_from_udp(modelv2_send, drivingdata_send, action, trajectory, frame_id):
  """Fill cereal modelV2 and drivingModelData messages from UDP data."""
  t_idxs = list(ModelConstants.T_IDXS)
  x_idxs = list(ModelConstants.X_IDXS)

  # ---- drivingModelData ----
  dmd = drivingdata_send.drivingModelData
  dmd.frameId = frame_id
  dmd.frameIdExtra = frame_id
  dmd.frameDropPerc = 0.0
  dmd.modelExecutionTime = 0.0

  # action
  dmd.action.desiredCurvature = action["desired_curvature"]
  dmd.action.desiredAcceleration = action["desired_acceleration"]
  dmd.action.shouldStop = action["should_stop"]

  # poly path (fit polynomial from position data)
  pos_x = trajectory["position_x"]
  # y and z are zero for now (straight-ahead reference frame)
  pos_y = np.zeros(ModelConstants.IDX_N, dtype=np.float32)
  pos_z = np.zeros(ModelConstants.IDX_N, dtype=np.float32)

  # fit 4th degree polynomial on time
  xyz = np.stack([pos_x, pos_y, pos_z], axis=1)
  coeffs = np.polynomial.polynomial.polyfit(t_idxs, xyz, deg=ModelConstants.POLY_PATH_DEGREE)
  dmd.path.xCoefficients = coeffs[:, 0].tolist()
  dmd.path.yCoefficients = coeffs[:, 1].tolist()
  dmd.path.zCoefficients = coeffs[:, 2].tolist()

  # lane line meta (defaults)
  dmd.laneLineMeta.leftY = 1.8
  dmd.laneLineMeta.leftProb = 0.0
  dmd.laneLineMeta.rightY = -1.8
  dmd.laneLineMeta.rightProb = 0.0

  # meta (defaults)
  dmd.meta.laneChangeState = log.LaneChangeState.off
  dmd.meta.laneChangeDirection = log.LaneChangeDirection.none

  # ---- modelV2 ----
  mv2 = modelv2_send.modelV2
  mv2.frameId = frame_id
  mv2.frameIdExtra = frame_id
  mv2.frameAge = 0
  mv2.frameDropPerc = 0.0
  mv2.timestampEof = int(time.monotonic() * 1e9)
  mv2.modelExecutionTime = 0.0

  # action (same as drivingModelData)
  mv2.action.desiredCurvature = action["desired_curvature"]
  mv2.action.desiredAcceleration = action["desired_acceleration"]
  mv2.action.shouldStop = action["should_stop"]

  # position (forward distance over time)
  mv2.position.t = t_idxs
  mv2.position.x = pos_x.tolist()
  mv2.position.y = pos_y.tolist()
  mv2.position.z = pos_z.tolist()
  mv2.position.xStd = [0.0] * ModelConstants.IDX_N
  mv2.position.yStd = [0.0] * ModelConstants.IDX_N
  mv2.position.zStd = [0.0] * ModelConstants.IDX_N

  # velocity
  mv2.velocity.t = t_idxs
  mv2.velocity.x = trajectory["velocity_x"].tolist()
  mv2.velocity.y = [0.0] * ModelConstants.IDX_N
  mv2.velocity.z = [0.0] * ModelConstants.IDX_N

  # acceleration
  mv2.acceleration.t = t_idxs
  mv2.acceleration.x = trajectory["acceleration_x"].tolist()
  mv2.acceleration.y = [0.0] * ModelConstants.IDX_N
  mv2.acceleration.z = [0.0] * ModelConstants.IDX_N

  # orientation (yaw = 0 for straight, can be computed from curvature later)
  mv2.orientation.t = t_idxs
  mv2.orientation.x = [0.0] * ModelConstants.IDX_N
  mv2.orientation.y = [0.0] * ModelConstants.IDX_N
  mv2.orientation.z = [0.0] * ModelConstants.IDX_N

  # orientation rate
  mv2.orientationRate.t = t_idxs
  mv2.orientationRate.x = [0.0] * ModelConstants.IDX_N
  mv2.orientationRate.y = [0.0] * ModelConstants.IDX_N
  mv2.orientationRate.z = [0.0] * ModelConstants.IDX_N

  # lane lines (4 lines, default positions)
  mv2.init('laneLines', 4)
  default_y_offsets = [3.6, 1.8, -1.8, -3.6]
  for i in range(4):
    ll = mv2.laneLines[i]
    ll.t = []
    ll.x = x_idxs
    ll.y = [default_y_offsets[i]] * ModelConstants.IDX_N
    ll.z = [0.0] * ModelConstants.IDX_N
  mv2.laneLineStds = [0.0, 0.0, 0.0, 0.0]
  mv2.laneLineProbs = [0.0, 0.0, 0.0, 0.0]

  # road edges (2 edges)
  mv2.init('roadEdges', 2)
  default_edge_y = [5.0, -5.0]
  for i in range(2):
    re = mv2.roadEdges[i]
    re.t = []
    re.x = x_idxs
    re.y = [default_edge_y[i]] * ModelConstants.IDX_N
    re.z = [0.0] * ModelConstants.IDX_N
  mv2.roadEdgeStds = [1.0, 1.0]

  # leads (3 leads, no detection)
  lead_t_idxs = list(ModelConstants.LEAD_T_IDXS)
  n_lead = len(lead_t_idxs)
  mv2.init('leadsV3', 3)
  for i in range(3):
    lead = mv2.leadsV3[i]
    lead.t = lead_t_idxs
    lead.x = [200.0] * n_lead
    lead.y = [0.0] * n_lead
    lead.v = [0.0] * n_lead
    lead.a = [0.0] * n_lead
    lead.xStd = [100.0] * n_lead
    lead.yStd = [100.0] * n_lead
    lead.vStd = [100.0] * n_lead
    lead.aStd = [100.0] * n_lead
    lead.prob = 0.0
    lead.probTime = ModelConstants.LEAD_T_OFFSETS[i]

  # meta
  meta = mv2.meta
  meta.desireState = [0.0] * ModelConstants.DESIRE_LEN
  meta.desirePrediction = [0.0] * (ModelConstants.DESIRE_PRED_LEN * ModelConstants.DESIRE_PRED_WIDTH)
  meta.engagedProb = 1.0
  meta.hardBrakePredicted = False
  meta.laneChangeState = log.LaneChangeState.off
  meta.laneChangeDirection = log.LaneChangeDirection.none

  meta.init('disengagePredictions')
  dp = meta.disengagePredictions
  dp.t = list(ModelConstants.META_T_IDXS)
  n_meta = len(ModelConstants.META_T_IDXS)
  dp.brakeDisengageProbs = [0.0] * n_meta
  dp.gasDisengageProbs = [0.0] * n_meta
  dp.steerOverrideProbs = [0.0] * n_meta
  dp.brake3MetersPerSecondSquaredProbs = [0.0] * n_meta
  dp.brake4MetersPerSecondSquaredProbs = [0.0] * n_meta
  dp.brake5MetersPerSecondSquaredProbs = [0.0] * n_meta
  dp.gasPressProbs = [1.0] * n_meta  # allow throttle
  dp.brakePressProbs = [0.0] * n_meta

  # confidence
  mv2.confidence = log.ModelDataV2.ConfidenceClass.green

  # mark valid
  modelv2_send.valid = True
  drivingdata_send.valid = True


def main():
  cloudlog.warning("udp_bridge init")
  config_realtime_process(7, 54)

  pm = PubMaster(["modelV2", "drivingModelData"])

  # UDP socket
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  sock.bind(("0.0.0.0", UDP_PORT))
  sock.settimeout(RECV_TIMEOUT)

  cloudlog.warning(f"udp_bridge listening on port {UDP_PORT}, packet size={PACKET_SIZE} bytes")

  frame_id = 0
  last_log_time = time.monotonic()

  while True:
    try:
      data, addr = sock.recvfrom(PACKET_SIZE + 64)  # small buffer margin
    except socket.timeout:
      continue

    seq, action, trajectory = parse_packet(data)
    if seq is None:
      cloudlog.error(f"udp_bridge: invalid packet from {addr}, size={len(data)}")
      continue

    # build and publish cereal messages
    modelv2_send = messaging.new_message('modelV2')
    drivingdata_send = messaging.new_message('drivingModelData')

    fill_model_msg_from_udp(modelv2_send, drivingdata_send, action, trajectory, frame_id)

    pm.send('modelV2', modelv2_send)
    pm.send('drivingModelData', drivingdata_send)

    frame_id += 1

    # periodic logging
    now = time.monotonic()
    if now - last_log_time > 5.0:
      cloudlog.info(f"udp_bridge: seq={seq} frame={frame_id} from {addr} "
                    f"curv={action['desired_curvature']:.4f} accel={action['desired_acceleration']:.2f}")
      last_log_time = now


if __name__ == "__main__":
  main()
