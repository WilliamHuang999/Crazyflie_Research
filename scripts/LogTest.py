# This script attempts to hover using the bolt platform, recording the drone's angular rates
# and angular rate setpoints. This should be run remotely over a radio connection with the
# Bolt instead of through the Pi on the drone.

import logging
from os import times
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

DEG2RAD = np.pi / 180
RAD2DEG = 180 / np.pi
LOGT = 10  # logging period in ms
URI = "radio://0/80/2M/"

myData = Data(1000, 1)

# checks that a deck is installed
def param_deck(id, value_str):
    id = id[7:-1]
    value = int(value_str)
    if value:
        print(f"{id} deck is attached!")
    else:
        print(f"{id} deck is NOT attached!")


# Add log config for asynchronous logging
def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_callback)


# Logging callback function
def log_callback(timestamp, data, logconf):
    altitude = data["stateEstimate.z"]

    myData.addSeries(timestamp, [altitude])


def plot(data):
    plt.plot(data[0, :], data[1, :])

    plt.show()


def configLog():
    logOut = LogConfig(name="Altitude", period_in_ms=LOGT)
    logOut.add_variable("stateEstimate.z", "float")

    return logOut


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Initialize all the CrazyFlie drivers:
cflib.crtp.init_drivers(enable_debug_driver=False)

# Configure logging
myLog = configLog()


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
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck)
        simple_log_async(scf, myLog)

        # initialize logging and arm props
        myLog.start()

        time.sleep(10)

        myLog.stop()

        # plot data
        out = myData.getData()
        plot(out)
