import ctypes
import functools
import json
import math
import multiprocessing
import numpy as np
import os
import socket
import time

from multiprocessing import Pipe, Array

from openpilot.tools.sim.bridge.common import QueueMessage, QueueMessageType
from openpilot.tools.sim.bridge.metadrive.metadrive_process import (metadrive_process, metadrive_simulation_state,
                                                                    metadrive_vehicle_state)
from openpilot.tools.sim.lib.common import SimulatorState, World
from openpilot.tools.sim.lib.camerad import W, H


class MetaDriveWorld(World):
  def __init__(self, status_q, config, test_duration, test_run, dual_camera=False):
    super().__init__(dual_camera)
    self.status_q = status_q
    self.camera_array = Array(ctypes.c_uint8, W*H*3)
    self.road_image = np.frombuffer(self.camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
    self.wide_camera_array = None
    if dual_camera:
      self.wide_camera_array = Array(ctypes.c_uint8, W*H*3)
      self.wide_road_image = np.frombuffer(self.wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

    self.controls_send, self.controls_recv = Pipe()
    self.simulation_state_send, self.simulation_state_recv = Pipe()
    self.vehicle_state_send, self.vehicle_state_recv = Pipe()

    self.exit_event = multiprocessing.Event()
    self.op_engaged = multiprocessing.Event()

    self.test_run = test_run

    self.first_engage = None
    self.last_check_timestamp = 0
    self.distance_moved = 0

    self.metadrive_process = multiprocessing.Process(name="metadrive process", target=
                              functools.partial(metadrive_process, dual_camera, config,
                                                self.camera_array, self.wide_camera_array, self.image_lock,
                                                self.controls_recv, self.simulation_state_send,
                                                self.vehicle_state_send, self.exit_event, self.op_engaged, test_duration, self.test_run))

    self.metadrive_process.start()
    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, "starting"))

    print("----------------------------------------------------------")
    print("---- Spawning Metadrive world, this might take awhile ----")
    print("----------------------------------------------------------")

    self.vehicle_last_pos = self.vehicle_state_recv.recv().position # wait for a state message to ensure metadrive is launched
    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, "started"))

    self.steer_ratio = 15
    self.vc = [0.0,0.0]
    self.reset_time = 0
    self.should_reset = False

    viz_host = os.environ.get("SIM_VIZ_HOST", "127.0.0.1")
    viz_port = int(os.environ.get("SIM_VIZ_PORT", "5006"))
    self._viz_addr = (viz_host, viz_port)
    self._viz_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self._viz_last_tx = 0.0
    self._viz_frame = 0

  def apply_controls(self, steer_angle, throttle_out, brake_out):
    if (time.monotonic() - self.reset_time) > 2:
      self.vc[0] = steer_angle

      if throttle_out:
        self.vc[1] = throttle_out
      else:
        self.vc[1] = -brake_out
    else:
      self.vc[0] = 0
      self.vc[1] = 0

    self.controls_send.send([*self.vc, self.should_reset])
    self.should_reset = False

  def read_state(self):
    while self.simulation_state_recv.poll(0):
      md_state: metadrive_simulation_state = self.simulation_state_recv.recv()
      if md_state.done:
        self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, md_state.done_info))
        self.exit_event.set()

  def read_sensors(self, state: SimulatorState):
    while self.vehicle_state_recv.poll(0):
      md_vehicle: metadrive_vehicle_state = self.vehicle_state_recv.recv()
      curr_pos = md_vehicle.position

      state.velocity = md_vehicle.velocity
      state.bearing = md_vehicle.bearing
      state.steering_angle = md_vehicle.steering_angle
      state.gps.from_xy(curr_pos)
      # simulated_sensors.send_gps_message()가 bearingDeg로 읽어감 → locationd_sim의 yaw 소스
      state.imu.bearing = md_vehicle.bearing
      state.valid = True

      now = time.monotonic()
      if now - self._viz_last_tx >= 0.05:  # 20Hz
        self._viz_last_tx = now
        vx = float(md_vehicle.velocity.x)
        vy = float(md_vehicle.velocity.y)
        speed = math.hypot(vx, vy)
        heading_rad = math.radians(md_vehicle.bearing)
        msg = {
          'type': 'vehicle',
          'x': round(float(curr_pos[0]), 4),
          'y': round(float(curr_pos[1]), 4),
          'heading': round(heading_rad, 4),
          'speed': round(speed, 3),
          'accel': 0.0,
          'curvature': 0.0,
          'should_stop': False,
          'frame': self._viz_frame,
        }
        try:
          self._viz_sock.sendto(json.dumps(msg).encode(), self._viz_addr)
        except OSError:
          pass
        self._viz_frame += 1

      is_engaged = state.is_engaged
      if is_engaged and self.first_engage is None:
        self.first_engage = time.monotonic()
        self.op_engaged.set()

      # check moving 5 seconds after engaged, doesn't move right away
      after_engaged_check = is_engaged and time.monotonic() - self.first_engage >= 5 and self.test_run

      x_dist = abs(curr_pos[0] - self.vehicle_last_pos[0])
      y_dist = abs(curr_pos[1] - self.vehicle_last_pos[1])
      dist_threshold = 1
      if x_dist >= dist_threshold or y_dist >= dist_threshold: # position not the same during staying still, > threshold is considered moving
        self.distance_moved += x_dist + y_dist

      time_check_threshold = 29
      current_time = time.monotonic()
      since_last_check = current_time - self.last_check_timestamp
      if since_last_check >= time_check_threshold:
        if after_engaged_check and self.distance_moved == 0:
          self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, {"vehicle_not_moving" : True}))
          self.exit_event.set()

        self.last_check_timestamp = current_time
        self.distance_moved = 0
        self.vehicle_last_pos = curr_pos

  def read_cameras(self):
    pass

  def tick(self):
    pass

  def reset(self):
    self.should_reset = True
    self.reset_time = time.monotonic()

  def close(self, reason: str):
    self.status_q.put(QueueMessage(QueueMessageType.CLOSE_STATUS, reason))
    self.exit_event.set()
    self.metadrive_process.join()
