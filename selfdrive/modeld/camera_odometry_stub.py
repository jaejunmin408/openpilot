#!/usr/bin/env python3
"""
cameraOdometry stub for simulation.
modeld의 모델 파일 없이 cameraOdometry만 발행하여
locationd/calibrationd 의존 체인을 유지한다.

의존 체인:
  이 스텁 → cameraOdometry → locationd (poll) → livePose (posenetOK, inputsOK)
                            → calibrationd (poll) → liveCalibration
  → selfdrived가 posenetInvalid / locationdTemporaryError를 발생시키지 않음
"""
import time
import cereal.messaging as messaging
from openpilot.selfdrive.modeld.constants import ModelConstants

def main():
  pm = messaging.PubMaster(["cameraOdometry"])
  frame_id = 0
  period = 1.0 / ModelConstants.MODEL_RUN_FREQ  # 20Hz

  while True:
    loop_start = time.monotonic()

    msg = messaging.new_message('cameraOdometry')
    msg.valid = True
    co = msg.cameraOdometry

    co.frameId = frame_id
    co.timestampEof = int(time.monotonic() * 1e9)

    # Near-zero pose (stationary camera)
    co.trans = [0.0, 0.0, 0.0]
    co.rot = [0.0, 0.0, 0.0]
    co.transStd = [0.01, 0.01, 0.01]
    co.rotStd = [0.01, 0.01, 0.01]
    co.wideFromDeviceEuler = [0.0, 0.0, 0.0]
    co.wideFromDeviceEulerStd = [0.01, 0.01, 0.01]
    co.roadTransformTrans = [0.0, 0.0, 1.2]  # camera height ~1.2m
    co.roadTransformTransStd = [0.01, 0.01, 0.01]

    pm.send('cameraOdometry', msg)
    frame_id += 1

    elapsed = time.monotonic() - loop_start
    sleep_time = period - elapsed
    if sleep_time > 0:
      time.sleep(sleep_time)

if __name__ == "__main__":
  main()
