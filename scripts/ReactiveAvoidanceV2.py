import pyrealsense2 as rs
import cv2 as cv
from controller import PIDController as PID
import time
import numpy as np

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

# Establish depth stream
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# DRONE PARAMETERS

stopDist = 0.5  # Distance in meters away from an object that prevents the drone from moving forward
lastTime = time.time()

try:
    while True:

        # Wait for a depth frame
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert image to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        [IMG_HEIGHT, IMG_WIDTH] = np.shape(depth_image)

        #Take middle slice of image
        middle_depth = depth_image[(int)(IMG_HEIGHT/2)-10:(int)(IMG_HEIGHT/2)+10, :]
        middle_depth = middle_depth * depth_frame.get_units()
        print(depth_frame.get_units())
        middle_depth_averages = np.mean(middle_depth, axis = 1)

        #Eliminate noise by setting depth ceiling
        ceiling = 5
        for depth in middle_depth_averages:
            if depth > 5:
                depth = 5
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
        depth_colormap_dim = depth_colormap.shape

        # Show images
        cv.imshow("Original DepthMap", depth_colormap)
        cv.imshow("RGB", color_image)

        cv.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()
