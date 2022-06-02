import pyrealsense2 as rs
import cv2 as cv

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

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a depth frame
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        # Convert image to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )

        depth_colormap_dim = depth_colormap.shape

        # Show image
        cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("RealSense", images)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()
