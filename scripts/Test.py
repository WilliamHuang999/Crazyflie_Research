import logging
from shutil import move
import sys
import time
from threading import Event
import struct
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

from utils.Data import Data
from utils.Command import send_thrust
from utils.Command import send_rates

# import pandas


# Import CrazyFlie modules
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.commander import Commander
from cflib.utils import uri_helper

debug = False

myData = Data(10000, 5, unwrap=[0, 1, 2])

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

    # rollRate = data["pid_rate.roll_outP"]
    # pitchRate = data["pid_rate.pitch_outP"]
    # yawRate = data["pid_rate.yaw_outP"]
    roll = data["stabilizer.roll"]
    pitch = data["stabilizer.pitch"]
    yaw = data["stabilizer.yaw"]
    thrust = data["stabilizer.thrust"]
    roll_P = data["pid_rate.roll_outP"]

    myData.addSeries(timestamp, [roll, pitch, yaw, thrust, roll_P])

    if debug:
        print("[%d][%s]: %s" % (timestamp, logconf.name, data))


# Ascend to a given height in a certain number of steps
def ascend(cf, alt, steps):
    for y in range(round(alt * steps)):
        cf.commander.send_hover_setpoint(0, 0, 0, y / steps)
        time.sleep(0.1)


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
lg_stab.add_variable("pid_rate.roll_outP", "float")

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

        # send_thrust(cf, 0)

        # while elapsed < 0.5:
        #     cf.commander.send_velocity_world_setpoint(0, 0, 100, 0)

        #     elapsed = time.time() - t0

        # Hover for 5 seconds
        while elapsed < 5:
            cf.commander.send_hover_setpoint(0, 0, 0, 1.5)

            elapsed = time.time() - t0
            time.sleep(0.05)

        # # Flip
        # startRoll = myData.getSeries()[1]
        # while myData.getSeries()[1] - startRoll < 360:
        #     send_rates(cf, 1000, 0, 0, 1.5)

        #     elapsed = time.time() - t0
        #     # time.sleep(0.05)

        # send_rates(cf, -100, 0, 0, 1.5)
        # cf.commander.send_hover_setpoint(0, 0, 0, 1.5)

        # cf.commander.send_hover_setpoint(0, 0, 0, 0)

        lg_stab.stop()
        myData.plot(plot=[0, 1, 2, 4], labels=["roll", "pitch", "yaw", "thrust", ""])
        # myData.save("data")
