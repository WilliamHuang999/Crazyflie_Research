# Follow a trajectory with as few lines as possible

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
from cflib.positioning.position_hl_commander import PositionHlCommander


URI = "radio://0/80/2M/"

cflib.crtp.init_drivers(enable_debug_driver=False)

with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
    with PositionHlCommander(scf, controller=PositionHlCommander.CONTROLLER_PID) as pc:

        # fly in a 1m square at 0.5m altitude
        pc.go_to(0.0, 0.0, 0.5)
        pc.go_to(1.0, 0.0, 0.5)
        pc.go_to(1.0, 1.0, 0.5)
        pc.go_to(0.0, 1.0, 0.5)
        pc.go_to(0.0, 0.0, 0.5)
