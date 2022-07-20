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
DEG2RAD = np.pi / 180
RAD2DEG = 180 / np.pi
LOGT = 10  # logging period in ms

myData = Data(10000, 6)

# uri = "usb://0"
uri = "radio://0/1/2M/"

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

baseAlt = 0.5

# Add log config for asynchronous logging
def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_rate_callback)


# Logging callback function
def log_rate_callback(timestamp, data, logconf):

    rollRate = data["stateEstimateZ.rateRoll"]
    pitchRate = data["stateEstimateZ.ratePitch"]
    yawRate = data["stateEstimateZ.rateYaw"]
    rollRateSetpoint = data["controller.rollRate"]
    pitchRateSetpoint = data["controller.pitchRate"]
    yawRateSetpoint = data["controller.yawRate"]

    myData.addSeries(
        timestamp,
        [rollRate, pitchRate, yawRate, rollRateSetpoint, pitchRateSetpoint, yawRateSetpoint],
    )

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


def plot(data):
    fig, axs = plt.subplots(1, 3, sharex=True, sharey=True)

    axs[0].plot(data[0, :], data[4, :] * DEG2RAD)
    axs[0].plot(data[0, :], data[1, :] / 1000, "r--")
    axs[0].set_xlabel("timestamp")
    axs[0].set_ylabel("angular velocity (rads/sec)")
    axs[0].tick_params(labelbottom=True, labelleft=True, direction="in")
    axs[0].set_title("Roll Rate")

    axs[1].plot(data[0, :], data[5, :] * DEG2RAD)
    axs[1].plot(data[0, :], data[2, :] / 1000, "r--")
    axs[1].set_xlabel("timestamp")
    axs[1].set_ylabel("angular velocity (rads/sec)")
    axs[1].tick_params(labelbottom=True, labelleft=True, direction="in")
    axs[1].set_title("Pitch Rate")

    axs[2].plot(data[0, :], data[6, :] * DEG2RAD)
    axs[2].plot(data[0, :], data[3, :] / 1000, "r--")
    axs[2].set_xlabel("timestamp")
    axs[2].set_ylabel("angular velocity (rads/sec)")
    axs[2].tick_params(labelbottom=True, labelleft=True, direction="in")
    axs[2].set_title("Yaw Rate")

    # fig.tight_layout()
    fig.suptitle(f"Crazyflie Rates ({round(1000/LOGT)} Hz sampling)")
    plt.show()


# Initialize all the CrazyFlie drivers:
cflib.crtp.init_drivers(enable_debug_driver=False)

# Configure logging
lg_rate = LogConfig(name="Rates", period_in_ms=LOGT)
lg_rate.add_variable("stateEstimateZ.rateRoll", "int16_t")  # milliradians / sec
lg_rate.add_variable("stateEstimateZ.rateYaw", "int16_t")  # milliradians / sec
lg_rate.add_variable("stateEstimateZ.ratePitch", "int16_t")  # milliradians / sec
lg_rate.add_variable("controller.rollRate", "float")
lg_rate.add_variable("controller.pitchRate", "float")
lg_rate.add_variable("controller.yawRate", "float")

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
        simple_log_async(scf, lg_rate)
        lg_rate.start()

        # ascend(cf, baseAlt, 10)
        # cf.commander.send_setpoint()

        cf.param.set_value("system.forceArm", 1)

        t0 = time.time()
        elapsed = 0

        # send_thrust(cf, 0)

        # while elapsed < 0.5:
        #     cf.commander.send_velocity_world_setpoint(0, 0, 100, 0)

        #     elapsed = time.time() - t0

        # Hover for 5 seconds

        while elapsed < 5:
            cf.commander.send_hover_setpoint(0, 0, 100, 0.5)

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

        cf.commander.send_hover_setpoint(0, 0, 0, 0.1)

        time.sleep(0.5)

        cf.param.set_value("system.forceArm", 0)
        lg_rate.stop()

        out = myData.getData()
        plot(out)
