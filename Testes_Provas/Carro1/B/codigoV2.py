# Libraries
import pyrealsense2 as rs
#import mediapipe as mp
import numpy as np
import cv2

# --------------------- Camera settings and configuration ---------------------

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

# Configuration of the resolution
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)

# Background removing
clipping_distance = 1 / depth_scale

# Create an aligned object
# rs.align allows a performed alignment of depth frames to others frames
# The "align_to" is the stream type to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)



while True:

    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    # Align the depth frame to color frame
    aligned_frames = align.process(frames)
    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame()   
    color_frame = aligned_frames.get_color_frame()
   
   
    # Validate that both frames are valid
    if not aligned_depth_frame or not color_frame:
        continue
    
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
  
    depth_image_3d = np.dstack((depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
     # Render images:
    #   depth align to color on left
    #   depth on right
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    point=(300,300)
    cv2.circle(color_image, point, 5, (0,0,255))
    depth = aligned_depth_frame.get_distance(int(point[1]), int(point[0]))
  
    cv2.imshow('RealSense color', color_image)
    cv2.imshow('RealSense depth ', depth_colormap)
    print(depth)

    # Show images
    key = cv2.waitKey(1)
    if key == 27:
        pipeline.stop()
        break
