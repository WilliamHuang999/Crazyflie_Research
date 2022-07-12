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


# Camera (Intel Realsense D415) properties
IMG_WIDTH, IMG_HEIGHT = (640, 360)
DFOV = 65
HFOV = 69
BASELINE = 0.055 # 55mm between left and right imager

# Start streaming
visualize = False
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, IMG_WIDTH, IMG_HEIGHT, rs.format.z16, 30)
if visualize: config.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, 30)
pipeline.start(config)

# DRONE PARAMETERS
radio_uri = "radio://0/6/2M"
usb_uri = "usb://0"
deck_attached_event = Event()
DEFAULT_HEIGHT = 0.5 # Default heigh the drone will take off to
ceiling_m = 2  # ceiling in meters
stopDist = 0.5  # Distance in meters away from an object that prevents the drone from moving forward
SPEED = 0.5 #  Speed the drone will move at
lastTime = time.time()

# Image properties
meters_per_pixel = 2 * ceiling_m / IMG_WIDTH * np.tan(0.5 * np.radians(DFOV))
SCENE_DISTANCE = 1 # 1m from scene
invalid_band_ratio = BASELINE / (2*SCENE_DISTANCE * np.tan(np.radians(HFOV/2)))
invalid_band_size = (int)(invalid_band_ratio * IMG_WIDTH)
TRIMMED_WIDTH = (int) (IMG_WIDTH - 2*invalid_band_size)

middle_running_average = np.empty((1, TRIMMED_WIDTH))
target_running_average = []


# checks whether flowdeck is installed
def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print("Deck is attached!")
    else:
        print("Deck is NOT attached!")

# Initialize all the CrazyFlie drivers:
cflib.crtp.init_drivers(enable_debug_driver=False)

with SyncCrazyflie(radio_uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    cf = scf.cf
    cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
    mc = MotionCommander(cf, default_height = DEFAULT_HEIGHT)
    mc.take_off()
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

            # Cut off invalid depth band (and equal width on opposite side)
            depth_image = depth_image[: , invalid_band_size : IMG_WIDTH  - invalid_band_size]

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

            print(middle_depth_filtered)

            # Find largest gap above depth ceiling
            ceiling = ceiling_m / depth_frame.get_units()  # in RealSense depth units

            # get black/white image
            middle_depth_bw = np.empty_like(middle_depth_filtered)
            for i in range(0, np.size(middle_depth_filtered)):
                if middle_depth_filtered[i] > ceiling:
                    middle_depth_bw[i] = 1
                else:
                    middle_depth_bw[i] = 0

            # mean filter
            averageLength = 9
            for i in range(0, np.size(middle_depth_bw)):
                if i > averageLength and np.size(middle_depth_bw) - i - 1 > averageLength:
                    newVal = np.sum(middle_depth_bw[i - averageLength : i + averageLength + 1]) / (2 * averageLength + 1)

                    newVal = round(newVal)

                    middle_depth_bw[i] = newVal


            # Find biggest gap
            count = 0
            longest = -1
            longestStart = -1
            longestEnd = -1
            for i in range(0, np.size(middle_depth_bw)):
                if middle_depth_bw[i] >= ceiling:
                    count += 1
                else:
                    if count > longest:
                        longest = count
                        longestEnd = i - 1
                        longestStart = longestEnd - count
                        count = 0

            # Corner case for when the gap reaches the side
            if count > longest:
                        longest = count
                        longestEnd = i - 1
                        longestStart = longestEnd - count
                        count = 0

            gapCenter = (int)((longestStart + longestEnd) / 2)
            width = longest * meters_per_pixel

            # If the gap is less than 0.5 m, stop the drone, otherwise start moving towards the center of the gap
            if width < 0.5:
                mc.start_linear_motion(0, 0, 0, 0)
                print("Stop: longest gap is ", width)
                gapCenter = 0
            else:

                x = ceiling_m
                y = meters_per_pixel * (IMG_WIDTH/2 - gapCenter)
                vy = (y*SPEED)/(np.sqrt(x**2 + y**2))
                vx = (x*SPEED)/(np.sqrt(x**2 + y**2))
                print(gapCenter)
                print("vx: ", vx, "    vy: ", vy)
                mc.start_linear_motion(vx, vy, 0, 0)



                elapsed = time.time() - t
                time.sleep(0.1)

    finally:
        # Stop streaming
        pipeline.stop()

    mc.land()