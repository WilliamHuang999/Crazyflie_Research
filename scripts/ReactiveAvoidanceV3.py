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

# DRONE PARAMETERS
radio_uri = "radio://0/6/2M"
usb_uri = "usb://0"
deck_attached_event = Event()
DEFAULT_HEIGHT = 0.5

stopDist = 0.5  # Distance in meters away from an object that prevents the drone from moving forward
lastTime = time.time()

IMG_WIDTH, IMG_HEIGHT = (640, 480)
FOV = 65
SPEED = 0.2

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


            # Find largest gap above depth ceiling
            ceiling = ceiling_m / depth_frame.get_units()  # in RealSense depth units

            # count = 0
            # threshold = 100
            # # get rid of skinny obstacles
            # for i in range(0, np.size(middle_depth_filtered)):
            #     if middle_depth_filtered[i] < ceiling:
            #         count += 1
            #     elif count < threshold:
            #         end = i - 1
            #         start = end - count

            #         endDepth = middle_depth_filtered[end]
            #         startDepth = middle_depth_filtered[start - 1]

            #         for i in range(count):
            #             insert = linInterp(startDepth, endDepth, count, i)
            #             middle_depth_filtered[start + i] = insert

            #         count = 0

            count = 0
            longest = -1
            longestStart = -1
            longestEnd = -1

            # Find biggest gap
            for i in range(0, np.size(middle_depth_filtered)):
                if middle_depth_filtered[i] >= ceiling:
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

            if width < 0.5:
                # stop drone
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