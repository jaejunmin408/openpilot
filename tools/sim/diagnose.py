#!/usr/bin/env python3
"""Diagnose why engagement fails. Run while openpilot + bridge are running."""
import time
import cereal.messaging as messaging

sm = messaging.SubMaster([
    'selfdriveState', 'onroadEvents', 'managerState', 'livePose',
    'modelV2', 'longitudinalPlan', 'accelerometer', 'gyroscope',
    'carState', 'cameraOdometry', 'liveCalibration',
])

print("Monitoring for 5 seconds...")
for i in range(10):
    sm.update(500)

sd = sm['selfdriveState']
print(f"\n{'='*60}")
print(f"engageable={sd.engageable}  active={sd.active}")
print(f"\n--- Message Status ---")
for s in ['modelV2','longitudinalPlan','cameraOdometry','livePose',
          'liveCalibration','accelerometer','gyroscope','carState']:
    print(f"  {s:25s} alive={sm.alive[s]!s:5s} valid={sm.valid[s]!s:5s}")

lp = sm['livePose']
print(f"\n--- Pose ---")
print(f"  posenetOK={lp.posenetOK}  inputsOK={lp.inputsOK}")

print(f"\n--- Blocking Events ---")
blocking = [e for e in sm['onroadEvents'] if any([e.noEntry, e.softDisable, e.immediateDisable])]
if blocking:
    for e in blocking:
        print(f"  {e.name}")
else:
    print("  (none)")

print(f"\n--- All Events ---")
for e in sm['onroadEvents']:
    print(f"  {e.name}")

print(f"\n--- Processes not running ---")
not_running = [p.name for p in sm['managerState'].processes if not p.running and p.shouldBeRunning]
if not_running:
    for n in not_running:
        print(f"  {n}")
else:
    print("  (none)")
