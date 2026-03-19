#!/usr/bin/env python3
"""
ext_controllerd: External controller bridge for openpilot "control-only" mode.

Receives (curvature, acceleration, shouldStop) via UDP from external PC,
publishes longitudinalPlan, cameraOdometry, and driverAssistance messages.

UDP Packet format (25 bytes, little-endian):
  Header (16 bytes): magic(uint32) + seq(uint32) + timestamp(float64)
  Payload (9 bytes): desiredCurvature(f32) + acceleration(f32) + shouldStop(uint8)
"""

import socket
import struct
import time

import cereal.messaging as messaging
from openpilot.common.realtime import config_realtime_process
from openpilot.common.swaglog import cloudlog

# Constants
UDP_PORT = 5005
PUBLISH_HZ = 20
DT = 1.0 / PUBLISH_HZ
MAGIC = 0x4F505431
PACKET_SIZE = 25  # 16 header + 9 payload
HEADER_FMT = "<IId"
PAYLOAD_FMT = "<ffB"
TIMEOUT_SEC = 0.5

# Safe defaults
DEFAULT_CURVATURE = 0.0
DEFAULT_ACCELERATION = 0.0
DEFAULT_SHOULD_STOP = True


def parse_packet(data):
  """Parse UDP packet. Returns (seq, curvature, acceleration, should_stop) or None."""
  if len(data) != PACKET_SIZE:
    return None

  try:
    magic, seq, timestamp = struct.unpack(HEADER_FMT, data[:16])
    if magic != MAGIC:
      return None
    curvature, acceleration, should_stop = struct.unpack(PAYLOAD_FMT, data[16:25])
    return seq, curvature, acceleration, bool(should_stop)
  except struct.error:
    return None


def fill_camera_odometry(msg, frame_id):
  """Fill dummy cameraOdometry message for calibrationd/locationd."""
  msg.valid = True
  co = msg.cameraOdometry
  co.frameId = frame_id
  co.timestampEof = int(time.monotonic() * 1e9)
  co.trans = [0.0, 0.0, 0.0]
  co.rot = [0.0, 0.0, 0.0]
  co.wideFromDeviceEuler = [0.0, 0.0, 0.0]
  co.roadTransformTrans = [0.0, 0.0, 1.2]
  co.transStd = [1.0, 1.0, 1.0]
  co.rotStd = [1.0, 1.0, 1.0]
  co.wideFromDeviceEulerStd = [1.0, 1.0, 1.0]
  co.roadTransformTransStd = [1.0, 1.0, 1.0]


def main():
  cloudlog.warning("ext_controllerd init")
  config_realtime_process(7, 54)

  pm = messaging.PubMaster(['longitudinalPlan', 'cameraOdometry', 'driverAssistance'])

  # UDP socket (non-blocking)
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  sock.bind(("0.0.0.0", UDP_PORT))
  sock.setblocking(False)

  cloudlog.warning(f"ext_controllerd listening on port {UDP_PORT}, packet size={PACKET_SIZE} bytes")
  cloudlog.warning("ext_controllerd: waiting for UDP connection from external PC...")

  frame_id = 0
  last_log_time = time.monotonic()
  start_time = time.monotonic()
  last_seq = -1
  last_valid_time = 0.0
  udp_connected = False

  # Start with safe defaults
  curvature = DEFAULT_CURVATURE
  acceleration = DEFAULT_ACCELERATION
  should_stop = DEFAULT_SHOULD_STOP

  while True:
    t_start = time.monotonic()

    # Drain all pending UDP packets, keep the latest
    got_new = False
    while True:
      try:
        data, addr = sock.recvfrom(PACKET_SIZE + 64)
      except BlockingIOError:
        break

      result = parse_packet(data)
      if result is not None:
        seq, curvature, acceleration, should_stop = result
        last_seq = seq
        last_valid_time = time.monotonic()
        got_new = True
        if not udp_connected:
          elapsed_wait = time.monotonic() - start_time
          cloudlog.warning(f"ext_controllerd: === UDP CONNECTED === from {addr} (waited {elapsed_wait:.1f}s)")
          cloudlog.warning(f"ext_controllerd: first packet seq={seq} "
                           f"curv={curvature:.4f} accel={acceleration:.2f} stop={should_stop}")
          udp_connected = True

    # Timeout check: revert to safe defaults if no valid packet in 500ms
    if udp_connected and (time.monotonic() - last_valid_time > TIMEOUT_SEC):
      if should_stop != DEFAULT_SHOULD_STOP or acceleration != DEFAULT_ACCELERATION:
        cloudlog.warning("ext_controllerd: UDP timeout, reverting to safe defaults")
      curvature = DEFAULT_CURVATURE
      acceleration = DEFAULT_ACCELERATION
      should_stop = DEFAULT_SHOULD_STOP

    # Log waiting status before connection
    if not udp_connected:
      now = time.monotonic()
      if now - last_log_time > 2.0:
        elapsed_wait = now - start_time
        cloudlog.warning(f"ext_controllerd: waiting for UDP... ({elapsed_wait:.0f}s elapsed, publishing defaults at {PUBLISH_HZ}Hz)")
        last_log_time = now

    # Publish longitudinalPlan
    plan_send = messaging.new_message('longitudinalPlan')
    plan_send.valid = True
    lp = plan_send.longitudinalPlan
    lp.aTarget = acceleration
    lp.shouldStop = should_stop
    lp.allowBrake = True
    lp.allowThrottle = True
    lp.hasLead = True
    lp.speeds = [0.2]  # triggers carControl.cruiseControl.resume
    lp.desiredCurvature = curvature
    pm.send('longitudinalPlan', plan_send)

    # Publish cameraOdometry (dummy, for calibrationd/locationd)
    posenet_send = messaging.new_message('cameraOdometry')
    fill_camera_odometry(posenet_send, frame_id)
    pm.send('cameraOdometry', posenet_send)

    # Publish driverAssistance (empty, for selfdrived alive check)
    assistance_send = messaging.new_message('driverAssistance')
    assistance_send.valid = True
    pm.send('driverAssistance', assistance_send)

    frame_id += 1

    # Periodic logging after connected
    now = time.monotonic()
    if udp_connected and now - last_log_time > 5.0:
      cloudlog.warning(f"ext_controllerd [LIVE]: seq={last_seq} frame={frame_id} "
                       f"curv={curvature:.4f} accel={acceleration:.2f} stop={should_stop} "
                       f"new_data={got_new}")
      last_log_time = now

    # Maintain 20Hz
    elapsed = time.monotonic() - t_start
    if elapsed < DT:
      time.sleep(DT - elapsed)


if __name__ == "__main__":
  main()
