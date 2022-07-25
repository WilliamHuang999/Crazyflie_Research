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

LOGGING = "rates"
DEG2RAD = np.pi / 180
RAD2DEG = 180 / np.pi
LOGT = 10  # logging period in ms
URI = "radio://0/80/2M/"

if LOGGING == "pitch":
    myData = Data(10000, 4, unwrap=[2])
elif LOGGING == "motors":
    myData = Data(10000, 6, unwrap=[4, 5])
elif LOGGING == "rates":
    myData = Data(10000, 6, unwrap=[4, 5])

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
    if LOGGING == "pitch":
        pitchRate = data["stateEstimateZ.ratePitch"]
        pitchRateSetpoint = data["controller.pitchRate"]
        pitch = data["stabilizer.pitch"]
        pitchSetpoint = data["controller.pitch"]

        myData.addSeries(timestamp, [pitchRate, pitchRateSetpoint, pitch, pitchSetpoint])

    elif LOGGING == "rates":
        rollRate = data["stateEstimateZ.rateRoll"]
        pitchRate = data["stateEstimateZ.ratePitch"]
        rollRateSetpoint = data["controller.rollRate"]
        pitchRateSetpoint = data["controller.pitchRate"]
        roll = data["stabilizer.roll"]
        pitch = data["stabilizer.pitch"]

        myData.addSeries(timestamp, [rollRate, pitchRate, rollRateSetpoint, pitchRateSetpoint, roll, pitch])

    elif LOGGING == "motors":
        m1 = data["motor.m1"]
        m2 = data["motor.m2"]
        m3 = data["motor.m3"]
        m4 = data["motor.m4"]
        roll = data["stabilizer.roll"]
        pitch = data["stabilizer.pitch"]

        myData.addSeries(timestamp, [m1, m2, m3, m4, roll, pitch])


def plot(data):
    if LOGGING == "motors":
        fig, axs = plt.subplots(1, 6, sharex=True)

        axs[0].plot(data[0, :], data[1, :], color="#1c7fff")
        axs[0].set_xlabel("timestamp")
        axs[0].set_ylabel("motor power (%)")
        axs[0].tick_params(labelbottom=True, labelleft=True, direction="in")
        axs[0].set_title("Motor 1")

        axs[1].plot(data[0, :], data[2, :], color="#1c7fff")
        axs[1].set_xlabel("timestamp")
        axs[1].set_ylabel("motor power (%)")
        axs[1].tick_params(labelbottom=True, labelleft=True, direction="in")
        axs[1].set_title("Motor 2")

        axs[2].plot(data[0, :], data[3, :], color="#1c7fff")
        axs[2].set_xlabel("timestamp")
        axs[2].set_ylabel("motor power (%)")
        axs[2].tick_params(labelbottom=True, labelleft=True, direction="in")
        axs[2].set_title("Motor 3")

        axs[3].plot(data[0, :], data[4, :], color="#1c7fff")
        axs[3].set_xlabel("timestamp")
        axs[3].set_ylabel("motor power (%)")
        axs[3].tick_params(labelbottom=True, labelleft=True, direction="in")
        axs[3].set_title("Motor 4")

        axs[4].plot(data[0, :], data[5, :], color="#1c7fff")
        axs[4].set_xlabel("timestamp")
        axs[4].set_ylabel("andle (deg)")
        axs[4].tick_params(labelbottom=True, labelleft=True, direction="in")
        axs[4].set_title("Roll")

        axs[5].plot(data[0, :], data[6, :], color="#1c7fff")
        axs[5].set_xlabel("timestamp")
        axs[5].set_ylabel("angle (deg)")
        axs[5].tick_params(labelbottom=True, labelleft=True, direction="in")
        axs[5].set_title("Pitch")

        # fig.tight_layout()
        fig.suptitle(f"Crazyflie Motor Power and Attitude ({round(1000/LOGT)} Hz sampling)")

    elif LOGGING == "pitch":
        fig, axs = plt.subplots(1, 2, sharex=True, sharey=True)

        axs[0].plot(data[0, :], data[2, :] * DEG2RAD, "--", color="#96c4ff")
        axs[0].plot(data[0, :], data[1, :] / 1000, color="#1c7fff")
        axs[0].set_xlabel("timestamp")
        axs[0].set_ylabel("angular velocity (rads/sec)")
        axs[0].tick_params(labelbottom=True, labelleft=True, direction="in")
        axs[0].set_title("Pitch Rate")

        axs[1].plot(data[0, :], data[4, :] * DEG2RAD, "--", color="#96c4ff")
        axs[1].plot(data[0, :], data[3, :] * DEG2RAD, color="#1c7fff")
        axs[1].set_xlabel("timestamp")
        axs[1].set_ylabel("angle (rad)")
        axs[1].tick_params(labelbottom=True, labelleft=True, direction="in")
        axs[1].set_title("Pitch")

        fig.suptitle(f"Crazyflie Pitch ({round(1000/LOGT)} Hz sampling)")

    elif LOGGING == "rates":
        fig, axs = plt.subplots(1, 4, sharex=True, sharey=True)

        axs[0].plot(data[0, :], data[3, :] * DEG2RAD, "--", color="#96c4ff")
        axs[0].plot(data[0, :], data[1, :] / 1000, color="#1c7fff")
        axs[0].set_xlabel("timestamp")
        axs[0].set_ylabel("angular velocity (rads/sec)")
        axs[0].tick_params(labelbottom=True, labelleft=True, direction="in")
        axs[0].set_title("Roll Rate")

        axs[1].plot(data[0, :], data[4, :] * DEG2RAD, "--", color="#96c4ff")
        axs[1].plot(data[0, :], data[2, :] / 1000, color="#1c7fff")
        axs[1].set_xlabel("timestamp")
        axs[1].set_ylabel("angular velocity (rads/sec)")
        axs[1].tick_params(labelbottom=True, labelleft=True, direction="in")
        axs[1].set_title("Pitch Rate")

        axs[2].plot(data[0, :], data[5, :] * DEG2RAD, color="#1c7fff")
        axs[2].set_xlabel("timestamp")
        axs[2].set_ylabel("angle (deg)")
        axs[2].tick_params(labelbottom=True, labelleft=True, direction="in")
        axs[2].set_title("Roll")

        axs[3].plot(data[0, :], data[6, :] * DEG2RAD, color="#1c7fff")
        axs[3].set_xlabel("timestamp")
        axs[3].set_ylabel("angle (deg)")
        axs[3].tick_params(labelbottom=True, labelleft=True, direction="in")
        axs[3].set_title("Pitch")

        # fig.tight_layout()
        fig.suptitle(f"Crazyflie Rates ({round(1000/LOGT)} Hz sampling)")

    plt.show()


def configLog():

    if LOGGING == "motors":
        logOut = LogConfig(name="Motors", period_in_ms=LOGT)
        logOut.add_variable("motor.m1", "uint32_t")
        logOut.add_variable("motor.m2", "uint32_t")
        logOut.add_variable("motor.m3", "uint32_t")
        logOut.add_variable("motor.m4", "uint32_t")
        logOut.add_variable("stabilizer.roll", "float")
        logOut.add_variable("stabilizer.pitch", "float")

    elif LOGGING == "rates":
        logOut = LogConfig(name="Rates", period_in_ms=LOGT)
        logOut.add_variable("stateEstimateZ.rateRoll", "int16_t")  # milliradians / sec
        logOut.add_variable("stateEstimateZ.ratePitch", "int16_t")  # milliradians / sec
        logOut.add_variable("controller.rollRate", "float")
        logOut.add_variable("controller.pitchRate", "float")
        logOut.add_variable("stabilizer.roll", "float")
        logOut.add_variable("stabilizer.pitch", "float")

    elif LOGGING == "pitch":
        logOut = LogConfig(name="Pitch", period_in_ms=LOGT)
        logOut.add_variable("controller.pitch", "float")
        logOut.add_variable("stabilizer.pitch", "float")
        logOut.add_variable("stateEstimateZ.ratePitch", "int16_t")  # milliradians / sec
        logOut.add_variable("controller.pitchRate", "float")

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
        cf.param.set_value("system.forceArm", 1)
        time.sleep(5)

        t0 = time.time()
        elapsed = 0

        # ascend and hover
        while elapsed < 5:
            cf.commander.send_hover_setpoint(0, 0, 0, 0.5)

            elapsed = time.time() - t0
            time.sleep(0.05)

        # land, disarm props, and stop logging data
        cf.commander.send_hover_setpoint(0, 0, 0, 0.1)
        time.sleep(0.5)
        cf.param.set_value("system.forceArm", 0)
        myLog.stop()

        # plot data
        out = myData.getData()
        plot(out)
