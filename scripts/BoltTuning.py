# This script attempts to hover using the bolt platform, recording the drone's angular rates
# and angular rate setpoints. This should be run remotely over a radio connection with the
# Bolt instead of through the Pi on the drone.

import pickle
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

# Create objects for storing data
motorData = Data(10000, 4)
rateData = Data(10000, 6)
attitudeData = Data(10000, 6, unwrap=[0, 1, 2, 3, 4, 5])
velocityData = Data(10000, 6)
positionData = Data(10000, 6)

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
    if logconf.name == "Motors":

        m1 = data["motor.m1"]
        m2 = data["motor.m2"]
        m3 = data["motor.m3"]
        m4 = data["motor.m4"]

        motorData.addSeries(timestamp, [m1, m2, m3, m4])

    elif logconf.name == "Rates":

        rollRate = data["stateEstimateZ.rateRoll"] / 1000
        pitchRate = data["stateEstimateZ.ratePitch"] / 1000
        yawRate = data["stateEstimateZ.rateYaw"] / 1000
        rollRateSetpoint = data["controller.rollRate"] * DEG2RAD
        pitchRateSetpoint = data["controller.pitchRate"] * DEG2RAD
        yawRateSetpoint = data["controller.yawRate"] * DEG2RAD

        rateData.addSeries(
            timestamp, [rollRate, pitchRate, yawRate, rollRateSetpoint, pitchRateSetpoint, yawRateSetpoint]
        )

    elif logconf.name == "Attitude":

        roll = data["stabilizer.roll"]
        pitch = data["stabilizer.pitch"]
        yaw = data["stabilizer.yaw"]
        rollSetpoint = data["controller.roll"]
        pitchSetpoint = data["controller.pitch"]
        yawSetpoint = data["controller.yaw"]

        attitudeData.addSeries(timestamp, [roll, pitch, yaw, rollSetpoint, pitchSetpoint, yawSetpoint])
<<<<<<< HEAD

    elif logconf.name == "Velocity":

        xVelo = data["stateEstimate.vx"]
        yVelo = data["stateEstimate.vy"]
        zVelo = data["stateEstimate.vz"]
        xVeloSetpoint = data["ctrltarget.vx"]
        yVeloSetpoint = data["ctrltarget.vy"]
        zVeloSetpoint = data["ctrltarget.vz"]

        velocityData.addSeries(timestamp, [xVelo, yVelo, zVelo, xVeloSetpoint, yVeloSetpoint, zVeloSetpoint])

    elif logconf.name == "Position":

        xPos = data["stateEstimate.x"]
        yPos = data["stateEstimate.y"]
        zPos = data["stateEstimate.z"]
        xPosSetpoint = data["ctrltarget.x"]
        yPosSetpoint = data["ctrltarget.y"]
        zPosSetpoint = data["ctrltarget.z"]

        positionData.addSeries(timestamp, [xPos, yPos, zPos, xPosSetpoint, yPosSetpoint, zPosSetpoint])
=======
>>>>>>> 324d9adc826089636f5eae28607de0a0b186b9e0


def plot(motorArray, rateArray, attitudeArray, velocityArray, positionArray):
    motorFig, motorAxs = plt.subplots(1, 4, sharex=True, sharey=True)
    rateFig, rateAxs = plt.subplots(1, 3, sharex=True, sharey=True)
    attitudeFig, attitudeAxs = plt.subplots(1, 3, sharex=True, sharey=True)
    velocityFig, velocityAxs = plt.subplots(1, 3, sharex=True)
    positionFig, positionAxs = plt.subplots(1, 3, sharex=True)

    # Motor 1
    motorAxs[0].plot(motorArray[0, :], motorArray[1, :], color="#1c7fff")
    motorAxs[0].set_xlabel("timestamp")
    motorAxs[0].set_ylabel("motor power (%)")
    motorAxs[0].tick_params(labelbottom=True, labelleft=True, direction="in")
    motorAxs[0].set_title("Motor 1")

    # Motor 2
    motorAxs[1].plot(motorArray[0, :], motorArray[2, :], color="#1c7fff")
    motorAxs[1].set_xlabel("timestamp")
    motorAxs[1].set_ylabel("motor power (%)")
    motorAxs[1].tick_params(labelbottom=True, labelleft=True, direction="in")
    motorAxs[1].set_title("Motor 2")

    # Motor 3
    motorAxs[2].plot(motorArray[0, :], motorArray[3, :], color="#1c7fff")
    motorAxs[2].set_xlabel("timestamp")
    motorAxs[2].set_ylabel("motor power (%)")
    motorAxs[2].tick_params(labelbottom=True, labelleft=True, direction="in")
    motorAxs[2].set_title("Motor 3")

    # Motor 4
    motorAxs[3].plot(motorArray[0, :], motorArray[4, :], color="#1c7fff")
    motorAxs[3].set_xlabel("timestamp")
    motorAxs[3].set_ylabel("motor power (%)")
    motorAxs[3].tick_params(labelbottom=True, labelleft=True, direction="in")
    motorAxs[3].set_title("Motor 4")

    # Roll Rate
    rateAxs[0].plot(rateArray[0, :], rateArray[1, :], color="#1c7fff")
    rateAxs[0].plot(rateArray[0, :], rateArray[4, :], "--", color="#96c4ff")
    rateAxs[0].set_xlabel("timestamp")
    rateAxs[0].set_ylabel("angular velocity (rads/sec)")
    rateAxs[0].tick_params(labelbottom=True, labelleft=True, direction="in")
    rateAxs[0].set_title("Roll Rate")

    # Pitch Rate
    rateAxs[1].plot(rateArray[0, :], rateArray[2, :], color="#1c7fff")
    rateAxs[1].plot(rateArray[0, :], rateArray[5, :], "--", color="#96c4ff")
    rateAxs[1].set_xlabel("timestamp")
    rateAxs[1].set_ylabel("angular velocity (rads/sec)")
    rateAxs[1].tick_params(labelbottom=True, labelleft=True, direction="in")
    rateAxs[1].set_title("Pitch Rate")

    # Yaw Rate
    rateAxs[2].plot(rateArray[0, :], rateArray[3, :], color="#1c7fff")
    rateAxs[2].plot(rateArray[0, :], rateArray[6, :], "--", color="#96c4ff")
    rateAxs[2].set_xlabel("timestamp")
    rateAxs[2].set_ylabel("angular velocity (rads/sec)")
    rateAxs[2].tick_params(labelbottom=True, labelleft=True, direction="in")
    rateAxs[2].set_title("Yaw Rate")

    # Roll
    attitudeAxs[0].plot(attitudeArray[0, :], attitudeArray[1, :], color="#1c7fff")
    attitudeAxs[0].plot(attitudeArray[0, :], attitudeArray[4, :], "--", color="#96c4ff")
    attitudeAxs[0].set_xlabel("timestamp")
    attitudeAxs[0].set_ylabel("attitude (deg)")
    attitudeAxs[0].tick_params(labelbottom=True, labelleft=True, direction="in")
    attitudeAxs[0].set_title("Roll")

    # Pitch
    attitudeAxs[1].plot(attitudeArray[0, :], attitudeArray[2, :], color="#1c7fff")
    attitudeAxs[1].plot(attitudeArray[0, :], attitudeArray[5, :], "--", color="#96c4ff")
    attitudeAxs[1].set_xlabel("timestamp")
    attitudeAxs[1].set_ylabel("attitude (deg)")
    attitudeAxs[1].tick_params(labelbottom=True, labelleft=True, direction="in")
    attitudeAxs[1].set_title("Pitch")

    # Yaw
    attitudeAxs[2].plot(attitudeArray[0, :], attitudeArray[3, :], color="#1c7fff")
    attitudeAxs[2].plot(attitudeArray[0, :], attitudeArray[6, :], "--", color="#96c4ff")
    attitudeAxs[2].set_xlabel("timestamp")
    attitudeAxs[2].set_ylabel("attitude (deg)")
    attitudeAxs[2].tick_params(labelbottom=True, labelleft=True, direction="in")
    attitudeAxs[2].set_title("Yaw")

    # X Velocity
    velocityAxs[0].plot(velocityArray[0, :], velocityArray[1, :], color="#1c7fff")
    velocityAxs[0].plot(velocityArray[0, :], velocityArray[4, :], "--", color="#96c4ff")
    velocityAxs[0].set_xlabel("timestamp")
    velocityAxs[0].set_ylabel("velocity (m/s)")
    velocityAxs[0].tick_params(labelbottom=True, labelleft=True, direction="in")
    velocityAxs[0].set_title("X Velocity")

    # Y Velocity
    velocityAxs[1].plot(velocityArray[0, :], velocityArray[2, :], color="#1c7fff")
    velocityAxs[1].plot(velocityArray[0, :], velocityArray[5, :], "--", color="#96c4ff")
    velocityAxs[1].set_xlabel("timestamp")
    velocityAxs[1].set_ylabel("velocity (m/s)")
    velocityAxs[1].tick_params(labelbottom=True, labelleft=True, direction="in")
    velocityAxs[1].set_title("Y Velocity")

    # Z Velocity
    velocityAxs[2].plot(velocityArray[0, :], velocityArray[3, :], color="#1c7fff")
    velocityAxs[2].plot(velocityArray[0, :], velocityArray[6, :], "--", color="#96c4ff")
    velocityAxs[2].set_xlabel("timestamp")
    velocityAxs[2].set_ylabel("velocity (m/s)")
    velocityAxs[2].tick_params(labelbottom=True, labelleft=True, direction="in")
    velocityAxs[2].set_title("Z Velocity")

    # X Position
    positionAxs[0].plot(positionArray[0, :], positionArray[1, :], color="#1c7fff")
    positionAxs[0].plot(positionArray[0, :], positionArray[4, :], "--", color="#96c4ff")
    positionAxs[0].set_xlabel("timestamp")
    positionAxs[0].set_ylabel("position (m)")
    positionAxs[0].tick_params(labelbottom=True, labelleft=True, direction="in")
    positionAxs[0].set_title("X Position")

    # Y Position
    positionAxs[1].plot(positionArray[0, :], positionArray[2, :], color="#1c7fff")
    positionAxs[1].plot(positionArray[0, :], positionArray[5, :], "--", color="#96c4ff")
    positionAxs[1].set_xlabel("timestamp")
    positionAxs[1].set_ylabel("position (m)")
    positionAxs[1].tick_params(labelbottom=True, labelleft=True, direction="in")
    positionAxs[1].set_title("Y Position")

    # Z Position
    positionAxs[2].plot(positionArray[0, :], positionArray[3, :], color="#1c7fff")
    positionAxs[2].plot(positionArray[0, :], positionArray[6, :], "--", color="#96c4ff")
    positionAxs[2].set_xlabel("timestamp")
    positionAxs[2].set_ylabel("position (m)")
    positionAxs[2].tick_params(labelbottom=True, labelleft=True, direction="in")
    positionAxs[2].set_title("Z Position")

    motorFig.suptitle(f"Crazyflie Motor Power and Attitude ({round(1000/LOGT)} Hz sampling)")
    rateFig.suptitle(f"Crazyflie Rates ({round(1000/LOGT)} Hz sampling)")
    attitudeFig.suptitle(f"Crazyflie Attitude ({round(1000/LOGT)} Hz sampling)")
    velocityFig.suptitle(f"Crazyflie Velocity ({round(1000/LOGT)} Hz sampling)")
    positionFig.suptitle(f"Crazyflie Position ({round(1000/LOGT)} Hz sampling)")

    plt.show()


def configLog():

    logMotors = LogConfig(name="Motors", period_in_ms=LOGT)
    logMotors.add_variable("motor.m1", "uint32_t")
    logMotors.add_variable("motor.m2", "uint32_t")
    logMotors.add_variable("motor.m3", "uint32_t")
    logMotors.add_variable("motor.m4", "uint32_t")
    logMotors.add_variable("stabilizer.thrust", "float")

    logRates = LogConfig(name="Rates", period_in_ms=LOGT)
    logRates.add_variable("stateEstimateZ.rateRoll", "int16_t")  # milliradians / sec
    logRates.add_variable("stateEstimateZ.ratePitch", "int16_t")  # milliradians / sec
    logRates.add_variable("stateEstimateZ.rateYaw", "int16_t")
    logRates.add_variable("controller.rollRate", "float")
    logRates.add_variable("controller.pitchRate", "float")
    logRates.add_variable("controller.yawRate", "float")

    logAttitude = LogConfig(name="Attitude", period_in_ms=LOGT)
    logAttitude.add_variable("controller.roll", "float")
    logAttitude.add_variable("controller.pitch", "float")
    logAttitude.add_variable("controller.yaw", "float")
    logAttitude.add_variable("stabilizer.roll", "float")
    logAttitude.add_variable("stabilizer.pitch", "float")
    logAttitude.add_variable("stabilizer.yaw", "float")

    logPosition = LogConfig(name="Position", period_in_ms=LOGT)
    logPosition.add_variable("stateEstimate.x", "float")
    logPosition.add_variable("stateEstimate.y", "float")
    logPosition.add_variable("stateEstimate.z", "float")
    logPosition.add_variable("ctrltarget.x", "float")
    logPosition.add_variable("ctrltarget.y", "float")
    logPosition.add_variable("ctrltarget.z", "float")

    logVelocity = LogConfig(name="Velocity", period_in_ms=LOGT)
    logVelocity.add_variable("stateEstimate.vx", "float")
    logVelocity.add_variable("stateEstimate.vy", "float")
    logVelocity.add_variable("stateEstimate.vz", "float")
    logVelocity.add_variable("ctrltarget.vx", "float")
    logVelocity.add_variable("ctrltarget.vy", "float")
    logVelocity.add_variable("ctrltarget.vz", "float")

    return logMotors, logRates, logAttitude, logVelocity, logPosition


def set_gains(cf, group, val, kp, ki, kd):
    
    if group == "pid_rate" or group == "pid_attitude":
        prefix = group + "." + val
        cf.param.set_value(prefix + "_kp", kp)
        cf.param.set_value(prefix + "_ki", ki)
        cf.param.set_value(prefix + "_kd", kd)
    
    elif group == "velCtlPid" or group == "posCtlPid":
        prefix = group + "." + val
        cf.param.set_value(prefix + "Kp", kp)
        cf.param.set_value(prefix + "Ki", ki)
        cf.param.set_value(prefix + "Kd", kd)



def get_gains(cf, group, val):

    if group == "pid_rate" or group == "pid_attitude":
        prefix = group + "." + val
        kp = cf.param.get_value(prefix + "_kp")
        ki = cf.param.get_value(prefix + "_ki")
        kd = cf.param.get_value(prefix + "_kd")
    
    elif group == "velCtlPid" or group == "posCtlPid":
        prefix = group + "." + val
        kp = cf.param.get_value(prefix + "Kp")
        ki = cf.param.get_value(prefix + "Ki")
        kd = cf.param.get_value(prefix + "Kd")
    
    else:
        print("Not a PID group")
        return -1, -1, -1

    return kp, ki, kd


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Initialize all the CrazyFlie drivers:
cflib.crtp.init_drivers(enable_debug_driver=False)

# Configure logging
[logMotors, logRates, logAttitude, logVelocity, logPosition] = configLog()


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
        simple_log_async(scf, logMotors)
        simple_log_async(scf, logRates)
        simple_log_async(scf, logAttitude)
        simple_log_async(scf, logVelocity)
        simple_log_async(scf, logPosition)

        # Initial Gains
        set_gains(cf, group="pid_rate", val="roll", kp=100, ki=100, kd=1.2)
        set_gains(cf, group="pid_rate", val="pitch", kp=100, ki=100, kd=1.2)
        print("Roll Rate:", get_gains(cf, "pid_rate", "roll"))
        print("Pitch Rate:", get_gains(cf, "pid_rate", "pitch"))

        # initialize logging and arm props
        logMotors.start()
        logRates.start()
        logAttitude.start()
        logVelocity.start()
        logPosition.start()

        cf.param.set_value("system.forceArm", 1)
        time.sleep(5)

        t0 = time.time()
        elapsed = 0

        # ascend and hover
        while elapsed < 3:
            cf.commander.send_hover_setpoint(0, 0, 0, 0.5)

            elapsed = time.time() - t0
            time.sleep(0.05)

        print("Changing gains")
        # Change gains after takeoff
        set_gains(cf, group="pid_rate", val="roll", kp=100, ki=100, kd=1.2)
        set_gains(cf, group="pid_rate", val="pitch", kp=100, ki=100, kd=1.2)
        print("Gains Changed")
<<<<<<< HEAD
        print("Roll Rate:", get_gains(cf, "pid_rate", "roll"))
        print("Pitch Rate:", get_gains(cf, "pid_rate", "pitch"))
=======
        print("Roll Rate:" , get_gains(cf, "pid_rate", "roll") )
        print("Pitch Rate:" , get_gains(cf, "pid_rate", "pitch") )
        
>>>>>>> 324d9adc826089636f5eae28607de0a0b186b9e0

        # Continue Hovering
        elapsed = 0
        t1 = time.time()
        while elapsed < 5:
            cf.commander.send_hover_setpoint(0, 0, 0, 0.5)

            elapsed = time.time() - t1
            time.sleep(0.05)

        # land, disarm props, and stop logging data
        # Descend:
        for y in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
            time.sleep(0.1)
        # Stop all motion:
        for i in range(10):
            cf.commander.send_stop_setpoint()
            time.sleep(0.1)

        cf.param.set_value("system.forceArm", 0)
        logMotors.stop()
        logRates.stop()
        logAttitude.stop()
        logVelocity.stop()
        logPosition.stop()

        # plot data
        motor = motorData.getData()
        rate = rateData.getData()
        attitude = attitudeData.getData()
        velocity = velocityData.getData()
        position = positionData.getData()
        plot(motor, rate, attitude, velocity, position)
