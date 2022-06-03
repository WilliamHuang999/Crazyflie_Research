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
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

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

        #Processing of depth image
        trimmed_depth_image = depth_image[(int)(IMG_HEIGHT/10):(int)(IMG_HEIGHT*9/10), \
                                    (int)(IMG_WIDTH/10):(int)(IMG_WIDTH*9/10)]   #Trim off edges of depth image
        [TRIMMED_HEIGHT, TRIMMED_WIDTH] = np.shape(trimmed_depth_image)
        trimmed_depth_image = cv.convertScaleAbs(trimmed_depth_image, alpha=0.03) #Convert to 8 bit values
        processed_depth_image = cv.bilateralFilter(trimmed_depth_image, 9, 200, 200) #Noise Reduction
        

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
        blurred_depth_colormap = cv.applyColorMap(processed_depth_image, cv.COLORMAP_JET)

        # Find closest and furthest pixels: y is left-right, z is up-down (x is forward-back)
        furthest_id = np.argmax(processed_depth_image)  # find index of furthest pixel
        furthest_y, furthest_z = np.unravel_index(furthest_id, (TRIMMED_HEIGHT, TRIMMED_WIDTH))
        furthest = depth_image[furthest_y, furthest_z]  # find depth of furthest pixel

        closest_id = np.argmin(processed_depth_image)  # find index of closest pixel
        closest_y, closest_z = np.unravel_index(closest_id, (TRIMMED_HEIGHT, TRIMMED_WIDTH))
        closest = depth_image[closest_y, closest_z]  # find depth of closest pixel

        # Add circles on closest and furthest points (BGR)
        cv.circle(depth_colormap, (furthest_y, furthest_z), 10, (0, 0, 0), 3) #Black
        cv.circle(depth_colormap, (closest_y, closest_z), 10, (255, 255, 255), 3) #White

        # y_controller = PID(IMG_WIDTH / 2, 1, 1, 1, 0)
        # z_controller = PID(IMG_HEIGHT / 2, 1, 1, 1, 0)

        # thisTime = time.time()
        # y_controller.updateError(furthest_y, thisTime - lastTime)
        # z_controller.updateError(furthest_z, thisTime - lastTime)
        # lastTime = thisTime

        depth_colormap_dim = depth_colormap.shape

        # Show images
        cv.imshow("Original DepthMap", depth_colormap)
        cv.imshow("Blurred DepthMap", blurred_depth_colormap)
        cv.imshow("RGB", color_image)

        cv.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()
