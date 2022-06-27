import logging
from shutil import move
import sys
import time
from threading import Event

import pyrealsense2 as rs
import cv2 as cv
import time
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.commander import Commander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from read_deck_mem import ReadMem

# DRONE PARAMETERS
radio_uri = "radio://0/1/2M"
usb_uri = "usb://0"
deck_attached_event = Event()
DEFAULT_HEIGHT = 1

stopDist = 0.5  # Distance in meters away from an object that prevents the drone from moving forward
lastTime = time.time()

IMG_WIDTH, IMG_HEIGHT = (640, 480)
FOV = 65

middle_running_average = np.empty((1, IMG_WIDTH))
target_running_average = []

ceiling_m = 2  # depth ceiling in meters
meters_per_pixel = 2 * ceiling_m / IMG_WIDTH * np.tan(0.5 * np.radians(FOV))


# checks whether flowdeck is installed
def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print("Deck is attached!")
    else:
        print("Deck is NOT attached!")

def linInterp(y1, y2, len, x):
    return y1 + x * (y2 - y1) / len

def establish_stream():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, IMG_WIDTH, IMG_HEIGHT, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)

    return pipeline

# Initialize all the CrazyFlie drivers:
cflib.crtp.init_drivers(enable_debug_driver=False)

#Establish stream
pipeline = establish_stream()
print("Stream started")

with SyncCrazyflie(usb_uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    cf = scf.cf
    
    #rm = ReadMem(usb_uri)
    cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
    time.sleep(1)


    t = time.time()
    elapsed = time.time() - t
    try:
        while(elapsed < 10):

            # Get Realsense Data
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            # Convert image to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())

             # Take middle slice of image
            middle_depth = depth_image[(int)(IMG_HEIGHT / 2) - 10 : (int)(IMG_HEIGHT / 2) + 10, :]
            middle_depth_averages = np.mean(middle_depth, axis=0)

            # Take running average of middle slice
            if np.size(middle_running_average[:, 0]) < 10:
                middle_running_average = np.vstack((middle_running_average, middle_depth_averages))
            else:
                middle_running_average = middle_running_average[1:, :]
                middle_running_average = np.vstack((middle_running_average, middle_depth_averages))
            middle_depth_filtered = np.mean(middle_running_average, axis=0)


            # Is there clear path in front?
            clear = True
            ceiling = ceiling_m / depth_frame.get_units()  # in RealSense depth units
            for i in range((int)(IMG_WIDTH/2 - 10), (int)(IMG_WIDTH/2 + 10)):
                if middle_depth_filtered[i] < ceiling:
                    clear = False
                    continue
                    
            if clear:
                print("Path is clear")
                cf.commander.send_hover_setpoint(0, 0, 0, 0.2)
            else:
                print("Path is not clear")
                cf.commander.send_stop_setpoint()

            elapsed = time.time() - t
            time.sleep(0.1)

    finally:
        # Stop streaming
        pipeline.stop()