#

import logging
from shutil import move
import sys
import time
from threading import Event
import struct
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# import pandas

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.commander import Commander
from cflib.utils import uri_helper

from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort

debug = False


class data:
    t = 0
    roll = 0
    pitch = 0
    yaw = 0
    thrust = 0

    def toString(self):
        return f"{myData.t}: {round(myData.roll, 2)}, {round(myData.pitch, 2)}, {round(myData.yaw, 2)}, {round(myData.thrust, 2)}"


myData = data()

# uri = "usb://0"
uri = "radio://0/80/2M/"

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

baseAlt = 0.5

# Add log config for asynchronous logging
def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)


# Logging callback function
def log_stab_callback(timestamp, data, logconf):
    myData.t = timestamp
    myData.roll = data["stabilizer.roll"]
    myData.pitch = data["stabilizer.pitch"]
    myData.yaw = data["stabilizer.yaw"]
    myData.thrust = data["stabilizer.thrust"]

    if debug:
        print("[%d][%s]: %s" % (timestamp, logconf.name, data))


# Ascend to a given height in a certain number of steps
def ascend(cf, alt, steps):
    for y in range(round(alt * steps)):
        cf.commander.send_hover_setpoint(0, 0, 0, y / steps)
        time.sleep(0.1)


# Set angular rate setpoints
# Not working -- unclear whether the problem is this or the modified firmware
def send_rates(cf, roll, pitch, yaw):
    pk = CRTPPacket()
    pk.port = CRTPPort.COMMANDER_GENERIC
    pk.data = struct.pack("<Bfff", 8, roll, pitch, yaw)
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

# Configure logging
lg_stab = LogConfig(name="Stabilizer", period_in_ms=10)
lg_stab.add_variable("stabilizer.roll", "float")
lg_stab.add_variable("stabilizer.pitch", "float")
lg_stab.add_variable("stabilizer.yaw", "float")
lg_stab.add_variable("stabilizer.thrust", "float")

# Scan for Crazyflies in range of the antenna:
print("Scanning interfaces for Crazyflies...")
available = cflib.crtp.scan_interfaces()

# List local CrazyFlie devices:
print("Crazyflies found:")
for i in available:
    print(i[0])

# Check that CrazyFlie devices are available:
if len(available) == 0:
    print("No Crazyflies found")
else:
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck)
        simple_log_async(scf, lg_stab)
        lg_stab.start()

        # ascend(cf, baseAlt, 10)
        # cf.commander.send_setpoint()

        t0 = time.time()
        elapsed = 0

        # Main loop
        while elapsed < 5:
            cf.commander.send_hover_setpoint(0, 0, 0, 1)

            print(myData.toString())
            elapsed = time.time() - t0
            time.sleep(0.05)

        while elapsed < 10:
            send_rates(cf, 0, 0, 10000)

            print(myData.toString())
            elapsed = time.time() - t0
            time.sleep(0.05)

        cf.commander.send_hover_setpoint(0, 0, 0, 0)

        lg_stab.stop()
