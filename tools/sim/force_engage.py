#!/usr/bin/env python3
"""Force engage openpilot by sending cruise control commands via carState."""
import time
import cereal.messaging as messaging

sm = messaging.SubMaster(['selfdriveState', 'carState', 'carControl', 'longitudinalPlan'])

print("Waiting for engageable...")
for i in range(300):  # 30 seconds max
    sm.update(100)
    sd = sm['selfdriveState']

    if sd.active:
        print(f"\n*** ENGAGED at v={sm['carState'].vEgo*3.6:.1f}km/h ***")
        break

    if i % 10 == 0:
        print(f"[{i/10:.0f}s] engageable={sd.engageable} active={sd.active} "
              f"v={sm['carState'].vEgo*3.6:.1f}km/h "
              f"accel={sm['carControl'].actuators.accel:+.3f} "
              f"aTarget={sm['longitudinalPlan'].aTarget:+.3f}")

if not sm['selfdriveState'].active:
    print("\nNot engaged after 30s. Check diagnose.py output.")
else:
    print("\nMonitoring engaged state...")
    for i in range(200):
        sm.update(250)
        cs = sm['carState']
        cc = sm['carControl']
        lp = sm['longitudinalPlan']
        sd = sm['selfdriveState']
        if i % 4 == 0:
            print(f"v={cs.vEgo*3.6:5.1f}km/h accel={cc.actuators.accel:+6.3f} "
                  f"steer={cc.actuators.steeringAngleDeg:+7.2f}deg "
                  f"aTarget={lp.aTarget:+6.3f} active={sd.active}")
        if not sd.active:
            print("*** DISENGAGED ***")
            break
