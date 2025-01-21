#!/usr/bin/env python3

###############################################
##          Read and publish stream          ##
###############################################

import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2, rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Node processes:
def process_img(args=None):
    rclpy.init(args=args)
    rsNode = Realsense()
    while True:
        rsNode.read_imgs()
        rsNode.publish_imgs()
        rclpy.spin_once(rsNode, timeout_sec=0.01)
    # Stop streaming
    print("Ending...")
    rsNode.pipeline.stop()
    # Clean end
    rsNode.destroy_node()
    rclpy.shutdown()

# Realsense Node:
class Realsense(Node):
    def __init__(self, fps= 60):
        super().__init__('realsense')
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.count = 0
        cv2.setMouseCallback('image', self.mouse_callback)

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        print( f"Connect: {device_product_line}" )
        found_rgb = True
        for s in device.sensors:
            print( "Name:" + s.get_info(rs.camera_info.name) )
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True

        if not (found_rgb):
            print("Depth camera equired !!!")
            exit(0)

        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)

        #IR
        self.config.enable_stream(rs.stream.infrared, 1, 848, 480, rs.format.y8, 60)
        self.config.enable_stream(rs.stream.infrared, 2, 848, 480, rs.format.y8, 60)


        # Start streaming
        self.pipeline.start(self.config)

        # Create bridge
        self.bridge=CvBridge()

        # Create publishers
        self.image_publisher = self.create_publisher(Image, '/sensor_msgs/image', 10 )
        self.depth_publisher = self.create_publisher(Image, '/sensor_msgs/image', 10 )

        #IR publishers
        self.infra_publisher_1 = self.create_publisher(Image, 'infrared_1',10)
        self.infra_publisher_2 = self.create_publisher(Image, 'infrared_2',10)

    def read_imgs(self):
        
        # Wait for a coherent tuple of frames: depth, color and accel
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.first(rs.stream.color)
        depth_frame = frames.first(rs.stream.depth)

        if not (depth_frame and color_frame):
            return

        # Convert images to numpy arrays
        self.depth_image = np.asanyarray(depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())

        #IR
        infra_frame_1 = frames.get_infrared_frame(1)
        infra_frame_2 = frames.get_infrared_frame(2)
        self.infra_image_1 = np.asanyarray(infra_frame_1.get_data())
        self.infra_image_2 = np.asanyarray(infra_frame_2.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = self.color_image.shape

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', self.color_image)
        cv2.waitKey(1)

    def publish_imgs(self):
        msg_image = self.bridge.cv2_to_imgmsg(self.color_image,"bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)


        # Utilisation de colormap sur l'image depth de la Realsense (image convertie en 8-bit par pixel)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

        msg_depth = self.bridge.cv2_to_imgmsg(depth_colormap,"bgr8")
        msg_depth.header.stamp = msg_image.header.stamp
        msg_depth.header.frame_id = "depth"
        self.depth_publisher.publish(msg_depth)

        # Utilisation de colormap sur l'image infrared de la Realsense (image convertie en 8-bit par pixel)
        self.infra_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(self.infra_image_1, alpha=0.03), cv2.COLORMAP_JET)

        # Utilisation de colormap sur l'image infrared de la Realsense (image convertie en 8-bit par pixel)
        self.infra_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(self.infra_image_2, alpha=0.03), cv2.COLORMAP_JET)

        msg_infra = self.bridge.cv2_to_imgmsg(self.infra_colormap_1,"bgr8")
        msg_infra.header.stamp = msg_image.header.stamp
        msg_infra.header.frame_id = "infrared_1"
        self.infra_publisher_1.publish(msg_infra)

        msg_infra = self.bridge.cv2_to_imgmsg(self.infra_colormap_2,"bgr8")
        msg_infra.header.stamp = msg_image.header.stamp
        msg_infra.header.frame_id = "infrared_2"
        self.infra_publisher_2.publish(msg_infra)


    def mouse_callback(self, event,x,y,flags,param):
        global mouseX,mouseY
        if event == cv2.EVENT_LBUTTONDBLCLK:
            cv2.imwrite('image{self.count}', self.color_image)
            self.count=+1


# Execute the function.
if __name__ == "__main__":
    process_img()