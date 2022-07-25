from GapFinder import GapFinder
import pyrealsense2 as rs
import cv2 as cv
from controller import PIDController as PID
import time
import numpy as np


def linInterp(y1, y2, len, x):
    return y1 + x * (y2 - y1) / len


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# # Get device product line for setting a supporting resolution
# pipeline_wrapper = rs.pipeline_wrapper(pipeline)
# pipeline_profile = config.resolve(pipeline_wrapper)
# device = pipeline_profile.get_device()
# device_product_line = str(device.get_info(rs.camera_info.product_line))

visualize = True

# Camera properties
IMG_WIDTH, IMG_HEIGHT = (640, 360)
DFOV = 65
HFOV = 69

# Start streaming
config.enable_stream(rs.stream.depth, IMG_WIDTH, IMG_HEIGHT, rs.format.z16, 30)
if visualize: config.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, 30)
pipeline.start(config)

# More properties
ceiling_m = 2  # ceiling in meters
meters_per_pixel = 2 * ceiling_m / IMG_WIDTH * np.tan(0.5 * np.radians(DFOV))

BASELINE = 0.055 # 55mm between left and right imager
SCENE_DISTANCE = 1 # 1m from scene
invalid_band_ratio = BASELINE / (2*SCENE_DISTANCE * np.tan(np.radians(HFOV/2)))
invalid_band_size = (int)(invalid_band_ratio * IMG_WIDTH)
TRIMMED_WIDTH = (int) (IMG_WIDTH - 2*invalid_band_size)

# DRONE PARAMETERS

stopDist = 0.5  # Distance in meters away from an object that prevents the drone from moving forward
lastTime = time.time()

middle_running_average = np.empty((1, TRIMMED_WIDTH))
target_running_average = []

gapFinder = GapFinder(invalid_band_size)



try:
    while True:

        # Wait for a depth frame
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        gapFinder.set_depth_frame_units(depth_frame.get_units())
        if visualize: color_frame = frames.get_color_frame()

        # Convert image to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        if visualize: color_image = np.asanyarray(color_frame.get_data())

        # Cut off invalid depth band (and equal width on opposite side)
        depth_image = depth_image[: , invalid_band_size : IMG_WIDTH  - invalid_band_size]
            
        gapFinder.addFrame(depth_image)
        gapCenter, longest = gapFinder.findGap()
        width = longest*meters_per_pixel

        if width < 0.5:
            # stop drone
            print("Stop")
            gapCenter = 0


        if visualize:
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
            depth_colormap_dim = depth_colormap.shape
            cv.circle(depth_colormap, (gapCenter, (int)(IMG_HEIGHT / 2)), 10, (0, 0, 0), 3)  # Black

            # Show images
            cv.imshow("Original DepthMap", depth_colormap)
            cv.imshow("RGB", color_image)

        if cv.waitKey(1) == ord("q"):
            break

finally:

    # Stop streaming
    pipeline.stop()

