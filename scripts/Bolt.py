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
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.commander import Commander
from cflib.utils import uri_helper

myData = Data(10000)

uri = "usb://0"

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Add log config for asynchronous logging
def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_callback)


# Add log config for asynchronous logging
def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_callback)


# Logging callback function
def log_callback(timestamp, data, logconf):

    roll = data["stabilizer.roll"]
    pitch = data["stabilizer.pitch"]
    yaw = data["stabilizer.yaw"]
    thrust = data["stabilizer.thrust"]

    myData.addSeries(timestamp, roll, pitch, yaw, thrust)


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

# Scan for connected crazyflies
print("Scanning interfaces for Crazyflies...")
available = cflib.crtp.scan_interfaces()

# List local Crazyflie devices:
print("Crazyflie found:")
for i in available:
    print(i[0])

# Check that Crazyflie devices are available:
if len(available) == 0:
    print("No Crazyflies found")
else:
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck)
        simple_log_async(scf, lg_stab)
        lg_stab.start()

        t0 = time.time()
        elapsed = 0

        # ascend to 0.5 m
        while elapsed < 1:
            cf.commander.send_hover_setpoint(0, 0, 0, 0.5)

            elapsed = time.time() - t0

        # descend and land
        cf.commander.send_hover_setpoint(0, 0, 0, 0.1)
        cf.commander.send_stop_setpoint()

        lg_stab.stop()
        myData.plot()
        myData.save("data")
