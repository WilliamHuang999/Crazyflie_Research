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
IMG_HEIGHT, IMG_WIDTH = (720, 1280)

config.enable_stream(rs.stream.depth, IMG_WIDTH, IMG_HEIGHT, rs.format.z16, 30)
config.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# DRONE PARAMETERS

stopDist = 0.5  # Distance in meters away from an object that prevents the drone from moving forward
lastTime = time.time()

middle_running_average = np.empty(IMG_WIDTH)
target_running_average = []

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

        # Take middle slice of image
        middle_depth = depth_image[(int)(IMG_HEIGHT/2)-10:(int)(IMG_HEIGHT/2)+10, :]
        middle_depth_averages = np.mean(middle_depth, axis = 0)

        
        if np.size(middle_running_average[:,0]) < 10: 
            middle_running_average = np.vstack(middle_running_average, middle_depth_averages)
        else:
            middle_running_average = middle_running_average[1:,:]
            middle_running_average = np.vstack(middle_running_average, middle_depth_averages)

        middle_depth_filtered = np.mean(middle_running_average, axis = 0)
        
        
        # Find largest gap above depth ceiling
        ceiling_m = 2 # floor in meters
        ceiling = ceiling_m/depth_frame.get_units() # in RealSense depth units

        count = 0
        longest = -1
        longestStart = -1
        longestEnd = -1
        for i in range(0, np.size(middle_depth_filtered)):
            if middle_depth_filtered[i] > ceiling:
                count += 1
            elif count > longest:
                longest = count
                longestEnd = i - 1
                longestStart = longestEnd - count
                count = 0
        gapCenter = (int)((longestStart + longestEnd)/2)
            
        #Take running average of gapCenter
        if np.size(target_running_average) < 10: target_running_average.append(gapCenter)
        else:
            target_running_average = target_running_average[1:]
            target_running_average.append(gapCenter)

        gapCenterFiltered = (int)(np.mean(target_running_average))
        
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
        depth_colormap_dim = depth_colormap.shape
        cv.circle(depth_colormap, (gapCenterFiltered, (int)(IMG_HEIGHT/2)), 10, (0, 0, 0), 3) #Black

        """
        # Make colormap of middle_depth_averages
        middle_depth_average_expanded = np.empty((IMG_HEIGHT, IMG_WIDTH))
        for i in range(0, IMG_HEIGHT):
            middle_depth_average_expanded[i] = middle_depth_filtered
        middle_depths_colormap = cv.applyColorMap(\
            cv.convertScaleAbs(middle_depth_average_expanded, alpha = 0.03), cv.COLORMAP_JET)
        #cv.circle(middle_depths_colormap, (gapCenter, (int)(IMG_HEIGHT/2)), 10, (0, 0, 0), 3) #Black
        """

        # Show images
        cv.imshow("Original DepthMap", depth_colormap)
        #cv.imshow("RGB", color_image)
        #cv.imshow("Center Depths", middle_depths_colormap)

        #print(middle_depth_averages[(int)(IMG_WIDTH/2)]*depth_frame.get_units())

        if cv.waitKey(1) == ord('q'):
            break

finally:

    # Stop streaming
    pipeline.stop()

