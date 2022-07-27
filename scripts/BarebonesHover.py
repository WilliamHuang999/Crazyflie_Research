# Hover with as few lines as possible

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

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.commander import Commander
from cflib.utils import uri_helper


URI = "radio://0/80/2M/"

cflib.crtp.init_drivers(enable_debug_driver=False)

with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
    cf = scf.cf

    # ascend and hover
    for i in range(10):

        # send (x_velocity, y_velocity, yaw_rate, z_dist) setpoint
        cf.commander.send_hover_setpoint(0, 0, 0, 0.5)

        time.sleep(0.05)

    # land
    cf.commander.send_hover_setpoint(0, 0, 0, 0)
    time.sleep(0.5)
