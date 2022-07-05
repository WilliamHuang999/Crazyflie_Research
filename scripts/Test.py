#

import logging
from shutil import move
import sys
import time
from threading import Event
import struct

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.commander import Commander
from cflib.utils import uri_helper

from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort


# uri = "usb://0"
uri = "radio://0/80/2M/"

baseAlt = 0.5

# Ascend to a given height in a certain number of steps
def ascend(cf, alt, steps):
    for y in range(alt * steps):
        cf.commander.send_hover_setpoint(0, 0, 0, y / steps)
        time.sleep(0.1)


# Set angular rate setpoints
def send_rates(cf, roll_rate, pitch_rate, yaw_rate):
    pk = CRTPPacket()
    pk.port = CRTPPort.COMMANDER_GENERIC
    pk.data = struct.pack("<Bffff", 8, roll_rate, pitch_rate, yaw_rate)
    cf.send_packet(pk)


# checks that a deck is installed
def param_deck(id, value_str):
    id = id[7:-1]
    value = int(value_str)
    if value:
        print(f"{id} deck is attached!")
    else:
        print(f"{id} deck is NOT attached!")


# Initialize all the CrazyFlie drivers:
cflib.crtp.init_drivers(enable_debug_driver=False)

# Scan for Crazyflies in range of the antenna:
print("Scanning interfaces for Crazyflies...")
available = cflib.crtp.scan_interfaces()

# List local CrazyFlie devices:
print("Crazyflies found:")
for i in available:
    print(i[0])

# Check that CrazyFlie devices are available:
if len(available) == 0:
    print("No Crazyflies found, cannot run example")
else:
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck)

        time.sleep(2)

        ascend(cf, baseAlt, 10)
        cf.commander.send_setpoint()
