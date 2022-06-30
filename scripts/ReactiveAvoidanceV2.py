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

visualize = False

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
invalid_band_ratio = BASELINE / (2*ceiling_m * np.tan(HFOV/2))
invalid_band_size = (int)(invalid_band_ratio * IMG_WIDTH)
TRIMMED_WIDTH = (int) (IMG_WIDTH - 2*invalid_band_size)

# DRONE PARAMETERS

stopDist = 0.5  # Distance in meters away from an object that prevents the drone from moving forward
lastTime = time.time()

middle_running_average = np.empty((1, TRIMMED_WIDTH))
target_running_average = []



try:
    while True:

        # Wait for a depth frame
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        
        if visualize: color_frame = frames.get_color_frame()

        # Convert image to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        if visualize: color_image = np.asanyarray(color_frame.get_data())

        # Cut off invalid depth band (and equal width on opposite side)
        depth_image = depth_image[: , invalid_band_size : IMG_WIDTH - 1 - invalid_band_size]
        IMG_WIDTH = IMG_WIDTH - invalid_band_size * 2

        # Take middle slice of image
        middle_depth = depth_image[(int)(IMG_HEIGHT / 2) - 10 : (int)(IMG_HEIGHT / 2) + 10, :]
        middle_depth_averages = np.mean(middle_depth, axis=0)

        # Get running average of middle slice
        if np.size(middle_running_average[:, 0]) < 10:
            middle_running_average = np.vstack((middle_running_average, middle_depth_averages))
        else:
            middle_running_average = middle_running_average[1:, :]
            middle_running_average = np.vstack((middle_running_average, middle_depth_averages))
        middle_depth_filtered = np.mean(middle_running_average, axis=0)

        ceiling = ceiling_m / depth_frame.get_units()  # in RealSense depth units

        # get black/white image
        middle_depth_bw = middle_depth_filtered > ceiling
        # middle_depth_bw = np.empty_like(middle_depth_filtered)
        # for i in range(0, np.size(middle_depth_filtered)):
        #     if middle_depth_filtered[i] > ceiling:
        #         middle_depth_bw[i] = 1
        #     else:
        #         middle_depth_bw[i] = 0

        # mean filter
        averageLength = 9
        for i in range(0, np.size(middle_depth_bw)):
            if i > averageLength and np.size(middle_depth_bw) - i - 1 > averageLength:
                newVal = np.sum(middle_depth_bw[i - averageLength : i + averageLength + 1]) / (2 * averageLength + 1)

                newVal = round(newVal)

                middle_depth_bw[i] = newVal

        # # remove skinny obstacles (shadows)
        # threshold = 100
        # count = 0
        # for i in range(0, np.size(middle_depth_bw)):
        #     if middle_depth_bw[i] == 0:
        #         count += 1
        #     elif count < threshold and count != 0:
        #         end = i
        #         start = end - count

        #         print("Found")
        #         print(count)

        #         middle_depth_bw[start:end] = np.ones((1, count))

        #         count = 0

        # Find largest gap above depth ceiling
        count = 0
        longest = -1
        longestStart = -1
        longestEnd = -1
        for i in range(0, np.size(middle_depth_bw)):
            if middle_depth_bw[i] > 0.5:
                count += 1
            elif count > longest:
                longest = count
                longestEnd = i
                longestStart = longestEnd - count
                count = 0
        width = longest * meters_per_pixel

        gapCenter = (int)((longestStart + longestEnd) / 2)
        print("Gap Center: ", gapCenter)
        print("Image width: ", TRIMMED_WIDTH)
        if width < 0.5:
            # stop drone
            gapCenter = 0


        if visualize:
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
            depth_colormap_dim = depth_colormap.shape
            cv.circle(depth_colormap, (gapCenter, (int)(IMG_HEIGHT / 2)), 10, (0, 0, 0), 3)  # Black

            # Make colormap of middle_depth_averages
            middle_depth_average_expanded = np.empty((IMG_HEIGHT, TRIMMED_WIDTH))
            middle_depth_bw_expanded = np.empty_like(middle_depth_average_expanded)
            for i in range(0, IMG_HEIGHT):
                middle_depth_average_expanded[i] = middle_depth_filtered
                middle_depth_bw_expanded[i] = middle_depth_bw
            # middle_depths_colormap = cv.applyColorMap(
            #    cv.convertScaleAbs(middle_depth_average_expanded, alpha=0.03), cv.COLORMAP_JET
            # )

            # Show images
            cv.imshow("Original DepthMap", depth_colormap)
            cv.imshow("RGB", color_image)
            cv.imshow("Center Depths", cv.convertScaleAbs(middle_depth_average_expanded, alpha=0.03))
            cv.imshow("Black White", middle_depth_bw_expanded * 255)

        if cv.waitKey(1) == ord("q"):
            break

finally:

    # Stop streaming
    pipeline.stop()
