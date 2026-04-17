#!/usr/bin/env python3
"""
시뮬(SIMULATION=1) 전용 cameraOdometry 더미 발행기 (20Hz).
process_config.py 에서 modeld 대신 본 모듈을 기동한다.

## 왜 존재하나
두 소비자에게 cameraOdometry 를 공급하기 위함이며, 역할이 서로 다르다.

  (1) calibrationd — **유일한 "실제 값 소비자"**
      cameraOdometry.trans / rot / wideFromDeviceEuler 를 읽어
      liveCalibration 을 산출한다. 본 스텁이 발행하는 값은 전부 더미
      (trans=[vEgo,0,0], rot=0, wideFromDeviceEuler=0)라 실제 캘리브
      의미는 없지만, calStatus=CALIBRATED 로 진입시켜 selfdrived 의
      calibrationInvalid / Incomplete / Recalibrating 이벤트를 막는다.
      liveCalibration 은 이어서 selfdrived(calStatus·rpyCalib),
      controlsd(rpyCalib → pose_calibrator), monitoring, UI 렌더러가
      소비하므로 시뮬에서도 반드시 필요하다.

  (2) locationd_sim — **값은 안 읽음. 순수 20Hz poll 클럭 용도**
      locationd_sim 은 cameraOdometry 메시지 내용을 전혀 참조하지 않고,
      오직 `poll='cameraOdometry'` 로 20Hz 주기만 얻어 쓴다.
      livePose.posenetOK / inputsOK 는 locationd_sim 이 하드코딩으로
      True 로 세팅하므로, 이 스텁이 그 플래그를 세워주는 것은 아니다.

## 제거하려면 (메모)
본 스텁을 완전히 없애려면 두 곳을 함께 바꿔야 한다.
  - locationd_sim 의 poll 소스를 carState 등 다른 주기 메시지로 교체
  - calibrationd 를 시뮬에서 끄고, 대신 liveCalibration 을
    calStatus=CALIBRATED / rpyCalib=0 고정으로 직접 발행하는 stub 으로 대체
현재는 간단함을 위해 본 더미 발행기를 그대로 둔다.
"""
import time
import cereal.messaging as messaging
from openpilot.selfdrive.modeld.constants import ModelConstants

def main():
  pm = messaging.PubMaster(["cameraOdometry"])
  sm = messaging.SubMaster(['carState'])
  frame_id = 0
  period = 1.0 / ModelConstants.MODEL_RUN_FREQ  # 20Hz

  while True:
    loop_start = time.monotonic()

    sm.update(0)
    v_ego = sm['carState'].vEgo if sm.alive['carState'] else 0.0

    msg = messaging.new_message('cameraOdometry')
    msg.valid = True
    co = msg.cameraOdometry

    co.frameId = frame_id
    co.timestampEof = int(time.monotonic() * 1e9)

    co.trans = [v_ego, 0.0, 0.0]
    co.rot = [0.0, 0.0, 0.0]
    co.transStd = [0.5, 0.5, 0.5]
    co.rotStd = [1.0, 1.0, 1.0]
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
