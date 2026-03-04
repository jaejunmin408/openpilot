#!/usr/bin/env python3
import time
from panda import Panda, PandaDFU

class GPIO:
    STM_RST_N = 124
    STM_BOOT0 = 134

def gpio_init(pin, output):
    with open(f"/sys/class/gpio/gpio{pin}/direction", 'wb') as f:
        f.write(b"out" if output else b"in")

def gpio_set(pin, high):
    with open(f"/sys/class/gpio/gpio{pin}/value", 'wb') as f:
        f.write(b"1" if high else b"0")

for pin in (GPIO.STM_RST_N, GPIO.STM_BOOT0):
    gpio_init(pin, True)

for attempt in range(10):
    print(f"Attempt {attempt + 1}...")
    print("resetting into DFU")
    gpio_set(GPIO.STM_RST_N, 1)
    gpio_set(GPIO.STM_BOOT0, 1)
    time.sleep(0.2)
    gpio_set(GPIO.STM_RST_N, 0)
    gpio_set(GPIO.STM_BOOT0, 0)
    print("flashing bootstub")
    if not Panda.wait_for_dfu(None, 5):
        print("DFU not found, retrying...")
        continue
    try:
        PandaDFU(None).recover()
        print("Bootstub SUCCESS")
        break
    except Exception as e:
        print(f"Failed: {e}")
else:
    print("All attempts failed")
    exit(1)

print("flashing app")
assert Panda.wait_for_panda(None, 5)
p = Panda()
assert p.bootstub
p.flash()
print("Version:", p.get_version())
print("Signature match:", p.up_to_date())