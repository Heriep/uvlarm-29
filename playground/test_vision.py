#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import math
import cv2,time

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
colorizer = rs.colorizer()

config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

align_to = rs.stream.depth
align = rs.align(align_to)

color_info=(0, 0, 255)
rayon=10

count=1
refTime= time.process_time()
freq= 60


def souris(event, x, y, flags, param):
    global lo, hi, color

    if event==cv2.EVENT_LBUTTONDOWN:
        if color>5:
            color-=1

    if event==cv2.EVENT_RBUTTONDOWN:
        if color<250:
            color+=1

    lo[0]=color-10
    hi[0]=color+10

color=50

lo=np.array([color-10, 50, 50])
hi=np.array([color+10, 255, 255])

color_info=(0, 0, 255)

kernel = np.ones((5, 5), np.uint8)

cv2.namedWindow('Camera')
cv2.setMouseCallback('Camera', souris)

try:
    while True:
        # This call waits until a new coherent set of frames is available on a device
        frames = pipeline.wait_for_frames()

        #Aligning color frame to depth frame
        aligned_frames =  align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        frame = frames.get_color_frame()

        if not depth_frame or not frame: continue

        # Two ways to colorized the depth map
        # first : using colorizer of pyrealsense                
        colorized_depth = colorizer.colorize(depth_frame)
        depth_colormap = np.asanyarray(colorized_depth.get_data())

        # Get the intrinsic parameters
        color_intrin = frame.profile.as_video_stream_profile().intrinsics

        color_image_bgr = np.asanyarray(frame.get_data())
        color_image = cv2.cvtColor(color_image_bgr, cv2.COLOR_BGR2HSV)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        #Use pixel value of  depth-aligned color image to get 3D axes
        x, y = int(color_colormap_dim[1]/2), int(color_colormap_dim[0]/2)
        depth = depth_frame.get_distance(x, y)
        dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
        distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))

        #print("Distance from camera to pixel:", distance)
        #print("Z-depth from camera surface to pixel surface:", depth)

        mask=cv2.inRange(color_image, lo, hi)
        mask=cv2.erode(mask, kernel, iterations=1)
        mask=cv2.dilate(mask, kernel, iterations=1)
        mask=cv2.medianBlur(mask, 5)
        pixel_count = np.sum(mask)
        if pixel_count > 250000:
            print("OUI")
        else:
            print("NON")
        image2=cv2.bitwise_and(color_image_bgr, color_image_bgr, mask= mask)
        cv2.putText(color_image_bgr, "Couleur: {:d}".format(color), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)

        cv2.imshow('Camera', color_image_bgr)
        cv2.imshow('image2', image2)
        cv2.imshow('Mask', mask)

        if cv2.waitKey(1)&0xFF==ord('q'):
            break

        # Frequency:
        if count == 10 :
            newTime= time.process_time()
            freq= 10/((newTime-refTime))
            refTime= newTime
            count= 0
        count+= 1

except Exception as e:
    print(e)
    pass

finally:
    pipeline.stop()